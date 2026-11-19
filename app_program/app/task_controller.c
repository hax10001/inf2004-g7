/** @file task_controller.c
 * @brief Own line following, barcode routing and obstacle bypass movement.
 */
#include "task_controller.h"
#include "app_config.h"
#include "nav_core.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/line_barcode/line_follow.h"
#include "modules/motion/motion.h"
#include "modules/obstacle/obstacle.h"
#include "modules/obstacle/scan_plan.h"
#include "modules/barcode_scanning/bc_wave.h"
#include "modules/terrain/terrain.h"
#include <tm/tmonitor.h>
#ifdef B1_LIVE_COMMANDS
#include "modules/telemetry/buddy1_protocol.h"
#include "modules/telemetry/buddy1_mqtt.h"
#include "modules/telemetry/buddy1_rt.h"
#endif

#define JUNCTION_HOLD_CYCLES \
    (APP_JUNCTION_HOLD_MSEC / APP_CONTROLLER_PERIOD_MSEC)
#define JUNCTION_SETTLE_CYCLES \
    (APP_JUNCTION_SETTLE_MSEC / APP_CONTROLLER_PERIOD_MSEC)

typedef enum
{
    ROBOT_IDLE,
    ROBOT_LINE_FOLLOWING,
    ROBOT_AT_JUNCTION,
    ROBOT_STOPPED,
    ROBOT_NAV_SAFETY_STOP, /**< Recoverable ONLY after explicit STOP. */
    ROBOT_ERROR, /**< Hardware/OS error: stays latched until reset. */
    ROBOT_REMOTE_HOLD,
    ROBOT_OBSTACLE,
    ROBOT_LIFTED /**< Picked up: motors off until placed back down. */
} robot_state_t;

typedef struct
{
    line_follow_action_t previous_action;
    bool                 b_have_previous_action;
    uint32_t             junction_count;
    /* Both stop counting at their limits (5 and 3 periods by default). */
    uint8_t              junction_cycles; /**< Periods since stopping. */
    uint8_t              settle_cycles;   /**< Periods back on a line. */
    bool                 b_rearmed;       /**< Both black after the hold. */
} controller_history_t;
#if (JUNCTION_HOLD_CYCLES > 255U) || (JUNCTION_SETTLE_CYCLES > 255U)
#error Junction hold/settle periods no longer fit their 8-bit counters
#endif
#if APP_IMU_ENABLED
static uint32_t g_imu_last_good_msec;
#endif
#if APP_IMU_ENABLED && APP_LIFT_STOP_ENABLED && APP_LIFT_RESUME_ENABLED
/* When the robot was first seen standing on the floor again while
 * ROBOT_LIFTED; 0 while it is not. Controller task only. */
static uint32_t g_lift_floor_since;
#endif
/* Single-controller ownership; the remote mutex protects published copies. */
static struct
{
    unsigned int event:5;
    unsigned int value:4;
    unsigned int sequence:16;
    unsigned int fault:4;
    unsigned int request_ok:1;
    unsigned int capture_known:1;
    unsigned int imu_seen:1;
} g_control = {0};
static robot_state_t controller_cycle(robot_state_t state,
                                       controller_history_t *p_history);

/** @brief Wait for startup calibration and fresh IR data. */
static robot_state_t handle_idle (void);
/** @brief Follow the line, preserving stop and error behaviour. */
static robot_state_t handle_following (controller_history_t * p_history);
/** @brief Advance the supplied bypass before returning to barcode routing. */
static robot_state_t handle_obstacle (controller_history_t * p_history);
/** @brief Stay stopped at a junction until placed back on a line.
 *
 * The IR sensors read black when lifted. Both black seen after the hold
 * period means the robot is still on the junction or has been picked up;
 * a coast off the junction ends within the hold and is not counted. A
 * steady line reading after that resumes following.
 */
static robot_state_t handle_junction (controller_history_t * p_history);
#if APP_IMU_ENABLED && APP_LIFT_STOP_ENABLED && APP_LIFT_RESUME_ENABLED
/** @brief While lifted, resume once standing on the floor long enough.
 * @param b_on_floor This period's fresh IMU and IR say it is on the floor.
 * @return ROBOT_IDLE to resume (IDLE rechecks IR before following), or
 * ROBOT_LIFTED to keep waiting.
 */
static robot_state_t lift_try_resume (bool b_on_floor, uint32_t now_msec,
                                      controller_history_t * p_history);
#endif
/** @brief Apply and log an action.
 * @return ROBOT_AT_JUNCTION when both sensors are black.
 */
#ifndef B1_LIVE_COMMANDS
static robot_state_t apply_action (ir_shared_snapshot_t const * p_reading,
                                   controller_history_t *       p_history);
#endif

#ifdef B1_LIVE_COMMANDS
/* All remote requests and reported states share one RTOS mutex. No
 * GCC atomic read-modify-write runtime is required on Cortex-M0+.
 * Neither the network task nor the callback may write motor PWM. */
static ID              g_remote_mutex = 0;
static uint8_t         g_stop_pending = 0U;
static uint8_t         g_start_pending = 0U;
/* Controller-owned validity of the last mailbox read. */
static robot_state_t   g_reported_state = ROBOT_REMOTE_HOLD;
static nav_controller_t g_navigation;
static uint8_t g_nav_report_event;
static uint8_t g_nav_report_value;
static uint16_t g_nav_report_seq;
/* Controller has sole write ownership; network copies under remote mutex. */
void g7_controller_navigation_report(uint8_t *kind, uint8_t *value,
                                     uint16_t *sequence)
{
    if (!kind || !value || !sequence || g_remote_mutex <= E_OK) return;
    if (tk_loc_mtx(g_remote_mutex, TMO_POL) < E_OK) return;
    *kind = g_nav_report_event;
    *value = g_nav_report_value;
    *sequence = g_nav_report_seq;
    (void)tk_unl_mtx(g_remote_mutex);
}
static void nav_report(nav_event_t kind, uint8_t value)
{
    if (kind == NAV_EVENT_NONE) return;
    g_control.event = (unsigned int)kind;
    g_control.value = value;
    ++g_control.sequence;
}
/* Both wheels' movement added together, in either direction, since boot.
 * Never decreases, even while a pivot runs one wheel backwards (the old
 * |count| average shrank then). Forward travel is half of it; a pivot's
 * angle is its growth over the track. Called once per controller period.
 */
static uint32_t g_nav_path_counts = 0U;
/* Nonzero while the barcode handoff to navigation could not be enabled
 * mid-run; retried every period while following. Controller task only. */
static uint8_t g_nav_handoff_retry = 0U;
/* Forward travel at the last period a barcode capture was active. */
static uint32_t g_capture_end_travel = 0U;
/* Forward travel when the running capture began (junction-tape check). */
static uint32_t g_capture_start_travel = 0U;
static uint8_t g_capture_was_active = 0U;
static int32_t g_nav_last_left = 0;
static int32_t g_nav_last_right = 0;
static int32_t g_nav_fwd_left = 0;
static int32_t g_nav_fwd_right = 0;

/* Heading relative to the line, for the obstacle scan's centre. While line
 * following, the robot's heading is averaged over the last
 * LINE_HEADING_TAU_MM of travel: that average is the line's direction (the
 * follower zig-zags about it). When an obstacle stops the robot, its
 * heading minus the average is how far it is turned off the line, so the
 * scan can split left/right about the line instead of sonar zero (logged
 * 19 Nov: a robot stopped 10-20 degrees off picked the longer side). */
#define LINE_HEADING_TAU_MM 150.0f
#define LINE_HEADING_MIN_MM 300.0f
#define LINE_HEADING_MM_PER_COUNT (219.8f / 1056.0f)
#define LINE_HEADING_TRACK_MM 110.0f
static float g_line_heading_deg = 0.0f;
static float g_line_average_deg = 0.0f;
static float g_line_tracked_mm = 0.0f;

static void line_heading_reset (void)
{
    g_line_heading_deg = 0.0f;
    g_line_average_deg = 0.0f;
    g_line_tracked_mm = 0.0f;
}

#ifdef B1_LIVE_COMMANDS
static void line_heading_update (void)
{
    float const left_mm = (float)g_nav_fwd_left * LINE_HEADING_MM_PER_COUNT;
    float const right_mm = (float)g_nav_fwd_right * LINE_HEADING_MM_PER_COUNT;
    float const travel = 0.5f * (left_mm + right_mm);
    float weight = (travel > 0.0f) ? (travel / LINE_HEADING_TAU_MM) : 0.0f;

    if ((travel > 50.0f) || (travel < -50.0f))
    {
        /* Not driven there in one period: picked up or a stale reading. */
        line_heading_reset();
        return;
    }
    if (weight > 1.0f)
    {
        weight = 1.0f;
    }
    g_line_heading_deg += ((right_mm - left_mm) / LINE_HEADING_TRACK_MM) *
                          57.29578f;
    g_line_average_deg += (g_line_heading_deg - g_line_average_deg) * weight;
    if (travel > 0.0f)
    {
        g_line_tracked_mm += travel;
    }
}
#endif

/* Robot heading minus the line's, + = turned left; publishes it (or "not
 * known" after too little straight following) for the obstacle scan. */
static void line_heading_publish (void)
{
    if (g_line_tracked_mm >= LINE_HEADING_MIN_MM)
    {
        float const off = g_line_heading_deg - g_line_average_deg;

        scan_plan_set_line_offset_ddeg((int32_t)(off * 10.0f));
        tm_printf((UB *)"[OBS] Heading off the line at the stop: %d.%d deg "
                        "(+ = left), over %d mm of following\n",
                  (INT)((int32_t)(off * 10.0f) / 10),
                  (INT)(((int32_t)(off * 10.0f) % 10 + 10) % 10),
                  (INT)g_line_tracked_mm);
    }
    else
    {
        scan_plan_set_line_offset_ddeg(SCAN_PLAN_LINE_OFFSET_UNKNOWN);
        tm_printf((UB *)"[OBS] Heading off the line unknown (%d mm of "
                        "following); scan centre is sonar zero\n",
                  (INT)g_line_tracked_mm);
    }
    line_heading_reset();
}

static uint32_t nav_path_counts(void)
{
    encoder_sample_t left = {0};
    encoder_sample_t right = {0};
    uint32_t dl, dr;
    encoder_read(ENCODER_LEFT, &left);
    encoder_read(ENCODER_RIGHT, &right);
    /* Last counts start at the boot-time zero, as the old measure did. */
    dl = (uint32_t)left.count - (uint32_t)g_nav_last_left;
    dr = (uint32_t)right.count - (uint32_t)g_nav_last_right;
    g_nav_last_left = left.count;
    g_nav_last_right = right.count;
    /* Forward travel per wheel: the left encoder counts down when driving
     * forward (logged: encL falls while encR rises). */
    g_nav_fwd_left = -(int32_t)dl;
    g_nav_fwd_right = (int32_t)dr;
    g_nav_path_counts += (dl > INT32_MAX ? 0U - dl : dl) +
                         (dr > INT32_MAX ? 0U - dr : dr);
    return g_nav_path_counts;
}

/** @brief Mid-run: enable the barcode handoff to navigation.
 * @return true to carry on line following. With APP_NAV_SCAN_FAIL_STOP 0 a
 * failure is retried each period while the robot keeps following (no
 * barcodes reach navigation until it succeeds); with 1 it stops.
 */
static bool nav_handoff_continue (void)
{
    if (g7_nav_continue_run())
    {
        g_nav_handoff_retry = 0U;
        return true;
    }

    tm_printf((UB *)"[NAV] Barcode handoff failed; %s\n",
              (APP_NAV_SCAN_FAIL_STOP != 0) ? "-> STOPPED" :
              "continuing without barcodes, retrying");
    g_nav_handoff_retry = 1U;
    return APP_NAV_SCAN_FAIL_STOP == 0;
}

ER g7_controller_remote_init (void)
{
    T_CMTX mutex_config = {0};
    ID h_mutex;

    mutex_config.mtxatr = TA_INHERIT;
    h_mutex = tk_cre_mtx(&mutex_config);
    if (h_mutex <= E_OK)
    {
        return (h_mutex == E_OK) ? E_OBJ : h_mutex;
    }
    g_remote_mutex = h_mutex;
    return E_OK;
}

uint8_t g7_controller_mission_state (void)
{
    robot_state_t state;

    if (g_remote_mutex <= E_OK ||
        tk_loc_mtx(g_remote_mutex, TMO_POL) < E_OK)
    {
        return 6U; /* Fail closed: do not advertise a ready controller. */
    }
    state = g_reported_state;
    (void)tk_unl_mtx(g_remote_mutex);

    switch (state)
    {
        case ROBOT_LINE_FOLLOWING: return 2U;
        case ROBOT_OBSTACLE: return 3U;
        case ROBOT_AT_JUNCTION: return 6U;
        case ROBOT_STOPPED: return 6U;
        case ROBOT_NAV_SAFETY_STOP: return 5U; /* Navigation safe stop */
        case ROBOT_ERROR: return 7U;
        case ROBOT_REMOTE_HOLD: return 6U;
        case ROBOT_LIFTED: return 4U; /* Paused; resumes when put down. */
        default: return 1U;
    }
}

/* Readiness bit meanings are mirrored in dashboard/server/protocol.js.
 * 1: MQTT offline; 2: not held; 4: STOP/START pending;
 * 8: IMU uncalibrated; 16: IR invalid; 32: IR stale;
 * 64: controller unavailable or obstacle sensor fault. */
uint8_t g7_controller_remote_blockers (void)
{
    ir_shared_snapshot_t reading = {0};
    SYSTIM now = {0};
    uint8_t reason = 0U;

    if (!b1_mqtt_connected())
    {
        reason |= 1U;
    }
    if (g_remote_mutex <= E_OK ||
        tk_loc_mtx(g_remote_mutex, TMO_POL) < E_OK)
    {
        return (uint8_t)(reason | 64U);
    }
    if (g_reported_state != ROBOT_REMOTE_HOLD)
    {
        reason |= 2U;
    }
    if (g_stop_pending != 0U || g_start_pending != 0U)
    {
        reason |= 4U;
    }
    (void)tk_unl_mtx(g_remote_mutex);
#if APP_IMU_ENABLED
    if (!terrain_is_calibrated())
    {
        reason |= 8U;
    }
    terrain_snapshot terrain = {0};
    if (terrain_get_snapshot(&terrain) < E_OK || !terrain.b_valid ||
        tk_get_otm(&now) < E_OK ||
        (uint32_t)(now.lo - terrain.timestamp_msec) > APP_IR_MAX_AGE_MSEC)
        reason |= 8U;
#endif
    if (ir_shared_get_latest(&reading) < E_OK || !reading.b_valid)
    {
        reason |= 16U;
    }
    else if (tk_get_otm(&now) < E_OK)
    {
        reason |= 64U;
    }
    else if ((UW)(now.lo - reading.timestamp_msec) >
             APP_IR_MAX_AGE_MSEC)
    {
        reason |= 32U;
    }
    /* CLEAR/DONE are safe to reset, but cannot capture the first barcode.
     * Require the learned, armed phase before accepting movement. */
    if (!bc_wave_is_ready())
    {
        reason |= 128U;
    }
    if (obstacle_has_fault())
    {
        reason |= 64U;
    }
    uint16_t left_pwm = 0U, right_pwm = 0U;
    motor_last_command(&left_pwm, &right_pwm);
    if (left_pwm || right_pwm) reason |= 64U;
    return reason;
}

uint8_t g7_controller_remote_ready (void)
{
    return (uint8_t)(g7_controller_remote_blockers() == 0U);
}

uint8_t g7_controller_remote_hold_applied (void)
{
    uint8_t held;

    if (g_remote_mutex <= E_OK ||
        tk_loc_mtx(g_remote_mutex, TMO_POL) < E_OK)
    {
        return 0U;
    }
    held = (uint8_t)(g_reported_state == ROBOT_REMOTE_HOLD &&
                     g_stop_pending == 0U && g_start_pending == 0U);
    (void)tk_unl_mtx(g_remote_mutex);
    uint16_t left_pwm = 0U, right_pwm = 0U;
    motor_last_command(&left_pwm, &right_pwm);
    held = (uint8_t)(held && !left_pwm && !right_pwm);
    return held;
}

uint8_t g7_controller_remote_command (uint8_t cmd, void * p_context)
{
    (void)p_context;

    if (g_remote_mutex <= E_OK)
    {
        return B1_NOT_READY;
    }
    if (cmd == B1_CMD_STOP)
    {
        /* STOP wins over any queued START, even from another task. */
        if (tk_loc_mtx(g_remote_mutex, TMO_FEVR) < E_OK)
        {
            return B1_NOT_READY;
        }
        g_stop_pending = 1U;
        g_start_pending = 0U;
        (void)tk_unl_mtx(g_remote_mutex);
        return B1_OK; /* Queued: controller must still apply the stop. */
    }
    if (cmd == B1_CMD_START)
    {
        if (!g7_controller_remote_ready() ||
            tk_loc_mtx(g_remote_mutex, TMO_FEVR) < E_OK)
        {
            return B1_NOT_READY;
        }
        if (g_reported_state != ROBOT_REMOTE_HOLD ||
            g_stop_pending != 0U)
        {
            (void)tk_unl_mtx(g_remote_mutex);
            return B1_NOT_READY;
        }
        g_start_pending = 1U;
        (void)tk_unl_mtx(g_remote_mutex);
        return B1_OK; /* Queued: controller verifies network on receipt. */
    }
    if (cmd == B1_CMD_RECAL_BARCODE)
    {
        if (!g7_controller_remote_hold_applied() || !bc_wave_can_rearm())
            return B1_NOT_READY;
        /* Telemetry already refused this while running. The barcode task
         * drops any capture and relearns white on its next poll. */
        bc_wave_request_relearn();
        return B1_OK;
    }
    /* No live IMU re-baselining API is available in this version. */
    return B1_NOT_READY;
}

/** @brief A navigation-only safety stop may be acknowledged/reset by
 * an explicit operator STOP; a real controller fault must remain latched.
 * STOP never authorizes movement, regardless of previous state.
 * It also clears the pending navigation maneuver and fault marker.
 */
static robot_state_t remote_reset_after_stop(robot_state_t prior)
{
    obstacle_abort();
    motion_reset_run();
    /* Also forgets a repeat-mode action: each START begins clean. */
    nav_forget(&g_navigation);
    g_control.fault = 0U;
    return (prior == ROBOT_ERROR) ? ROBOT_ERROR : ROBOT_REMOTE_HOLD;
}

/* Consecutive periods the request mailbox could not be read. */
static uint8_t g_remote_lock_misses = 0U;
#if (APP_REMOTE_LOCK_MISS_LIMIT < 1) || (APP_REMOTE_LOCK_MISS_LIMIT > 255)
#error APP_REMOTE_LOCK_MISS_LIMIT must be 1..255
#endif

/** @brief Read queued requests, waiting at most APP_REMOTE_LOCK_WAIT_MSEC.
 * The telemetry task holds this mutex briefly several times per period, so
 * a single miss is normal contention, not a STOP: the pending request stays
 * queued and is read next period. Only APP_REMOTE_LOCK_MISS_LIMIT misses
 * in a row (a stuck mailbox) force a stop.
 */
static void remote_take_requests (uint8_t * p_stop, uint8_t * p_start,
                                  robot_state_t state)
{
    *p_stop = 0U;
    *p_start = 0U;
    g_control.request_ok = 0U;
    if (g_remote_mutex <= E_OK ||
        tk_loc_mtx(g_remote_mutex, (TMO)APP_REMOTE_LOCK_WAIT_MSEC) < E_OK)
    {
        if (g_remote_lock_misses < (uint8_t)APP_REMOTE_LOCK_MISS_LIMIT)
        {
            ++g_remote_lock_misses;
        }
        if (g_remote_lock_misses >= (uint8_t)APP_REMOTE_LOCK_MISS_LIMIT)
        {
            if ((uint8_t)APP_REMOTE_LOCK_MISS_LIMIT == g_remote_lock_misses)
            {
                tm_printf((UB *)"[CTRL] Command mailbox unavailable for %u "
                                "periods -> motors STOPPED\n",
                          (UINT)g_remote_lock_misses);
            }
            *p_stop = 1U;
        }
        return;
    }
    g_remote_lock_misses = 0U;
    *p_stop = g_stop_pending;
    g_control.request_ok = 1U;
    *p_start = g_start_pending;
    g_stop_pending = 0U;
    g_start_pending = 0U;
    g_reported_state = state;
    (void)tk_unl_mtx(g_remote_mutex);
}

/** @brief Publish a controller-only state copy for the network reader. */
static void remote_publish_state (robot_state_t state)
{
    if (g_remote_mutex > E_OK &&
        tk_loc_mtx(g_remote_mutex, TMO_POL) >= E_OK)
    {
        g_reported_state = state;
        g_nav_report_event = (uint8_t)g_control.event;
        g_nav_report_value = (uint8_t)g_control.value;
        g_nav_report_seq = (uint16_t)g_control.sequence;
        (void)tk_unl_mtx(g_remote_mutex);
    }
}
#endif

/* Fault source values are the V15 EVENT kind 21 contract. */
static void report_fault(uint8_t reason)
{
#ifdef B1_LIVE_COMMANDS
    if (g_control.fault != reason)
    {
        g_control.fault = reason;
        nav_report(NAV_EVENT_SAFE_STOP, reason);
    }
#else
    (void)reason;
    /* Unused in the motor-free barcode capture profile (no IMU either). */
    (void)g_control;
#endif
}

void task_controller_task (INT start_code, void * p_context)
{
#ifdef B1_LIVE_COMMANDS
    robot_state_t        state   = ROBOT_REMOTE_HOLD;
#else
    robot_state_t        state   = ROBOT_IDLE;
#endif
    controller_history_t history = {LINE_UNCERTAIN, false, 0U, 0U, 0U, false};
    /* Casting unused arguments to void cannot alter their values. */
    (void)start_code;
    (void)p_context;

#ifdef B1_LIVE_COMMANDS
    {
        nav_config_t cfg = {
            APP_NAV_JUNCTION_CONFIRM_MSEC,
            APP_NAV_MIN_TURN_MSEC,
            APP_NAV_MAX_TURN_MSEC,
            APP_NAV_MAX_CROSS_MSEC,
            APP_NAV_BARCODE_TO_JUNCTION_MIN_COUNTS,
            APP_NAV_JUNCTION_CONFIRM_COUNTS,
            APP_NAV_JUNCTION_CONFIRM_MAX_MSEC,
            APP_NAV_DECODE_WAIT_MSEC,
            APP_NAV_DECODE_GRACE_MSEC,
            APP_NAV_CENTRE_COUNTS,
            APP_NAV_CENTRE_MAX_MSEC,
            APP_NAV_TURN90_COUNTS,
            APP_NAV_REPEAT_ACTION,
            APP_NAV_STRAIGHT_DEFAULT,
            (APP_NAV_SCAN_FAIL_STOP != 0) ? 0U : 1U
        };
        nav_init(&g_navigation, &cfg);
    }
#endif
    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[CTRL] Controller started\n");
#ifdef B1_LIVE_COMMANDS
    /* The controlled profile never leaves boot with a motor target. */
    motion_apply(LINE_UNCERTAIN);
    tm_printf((UB *)"[CTRL] Waiting for dashboard START\n");
#endif

#if APP_IMU_ENABLED
    motion_apply(LINE_UNCERTAIN);
    /* UB views read-only string bytes; the monitor does not modify them. */
    tm_printf((UB *)"[CTRL] Waiting for IMU level reference\n");
#endif

    for (;;)
    {
        state = controller_cycle(state, &history);
        tk_dly_tsk(APP_CONTROLLER_PERIOD_MSEC);
    }
}

/* One real production period, also exercised by deterministic host tests. */
static robot_state_t controller_cycle(robot_state_t state,
                                       controller_history_t *p_history)
{
#ifdef B1_LIVE_COMMANDS
        {
            uint8_t stop_request;
            uint8_t start_request;
            uint8_t explicit_stop;

            remote_take_requests(&stop_request, &start_request, state);
            explicit_stop = (uint8_t)(stop_request && g_control.request_ok);
            /* Network failure must never resume a stopped robot. */
            if ((state == ROBOT_LINE_FOLLOWING || state == ROBOT_OBSTACLE ||
                 state == ROBOT_IDLE || state == ROBOT_AT_JUNCTION ||
                 state == ROBOT_LIFTED) &&
                !b1_mqtt_connected())
            {
                stop_request = 1U;
                tm_printf((UB *)
                    "[CTRL] MQTT lost -> motors STOPPED\n");
            }
            if (stop_request != 0U)
            {
                /* Apply physical zero-output request before touching the
                 * network event mailbox; STOP always wins. */
                motion_stop_now();
                g7_nav_clear_decoded();
                *p_history = (controller_history_t){0};
                /* A navigation safety stop is recoverable only by STOP.
                 * A real error remains latched. No automatic restart. */
                if (explicit_stop) state = remote_reset_after_stop(state);
                else
                {
                    /* Link loss/contention cannot acknowledge a fault. */
                    obstacle_abort();
                    nav_forget(&g_navigation);
                    if (state != ROBOT_ERROR && state != ROBOT_STOPPED &&
                        state != ROBOT_NAV_SAFETY_STOP)
                        state = ROBOT_REMOTE_HOLD;
                }
                if (explicit_stop)
                    tm_printf((UB *)"[CTRL] Remote STOP applied\n");
                g_capture_end_travel = 0U;
                g_control.capture_known = 0U;
            }
            else if (state == ROBOT_REMOTE_HOLD && start_request != 0U &&
                     b1_mqtt_connected())
            {
                SYSTIM started = {0};
                /* The scanner rearms itself after its worker finishes.
                 * This START is the only authorization to begin movement;
                 * neither scan completion nor MQTT reconnect can do so. */
                if (g7_controller_remote_ready() &&
                    tk_get_otm(&started) >= E_OK)
                {
                    if (g7_nav_begin_run())
                    {
                        g_nav_handoff_retry = 0U;
                        nav_start(&g_navigation, started.lo);
                        state = ROBOT_IDLE;
                        tm_printf((UB *)"[CTRL] Remote START accepted\n");
                    }
                }
            }
            remote_publish_state(state);
        }
#endif
        /* Driving off the track round an obstacle passes floor edges and
         * marks that are not barcodes; do not start captures then.
         */
        {
            bool b_suppress = (state != ROBOT_LINE_FOLLOWING);
#if defined(B1_LIVE_COMMANDS) && APP_BARCODE_SUPPRESS_IN_TURNS
            /* Centring on, pivoting at and reacquiring after a junction
             * sweep the barcode sensor across the track lines. */
            b_suppress = b_suppress ||
                (g_navigation.state == NAV_CENTRE) ||
                (g_navigation.state == NAV_TURN) ||
                (g_navigation.state == NAV_REACQUIRE) ||
                (g_navigation.state == NAV_RESUME);
#endif
            bc_wave_set_suppressed((uint8_t)b_suppress);
        }

        terrain_snapshot snapshot = {0};
        ER terrain_error = terrain_get_snapshot(&snapshot);
#if APP_IMU_ENABLED
        SYSTIM terrain_now = {0};
        bool const imu_clock_ok = tk_get_otm(&terrain_now) >= E_OK;
        bool const imu_fresh = terrain_error >= E_OK && imu_clock_ok &&
            snapshot.b_valid &&
            (uint32_t)(terrain_now.lo - snapshot.timestamp_msec) <= 200U;
        if (imu_fresh)
        {
            g_imu_last_good_msec = snapshot.timestamp_msec;
            g_control.imu_seen = 1U;
        }
#endif
        if (state == ROBOT_LINE_FOLLOWING || state == ROBOT_OBSTACLE)
        {
#if APP_IMU_ENABLED
            if (imu_clock_ok)
                motion_observe_terrain(&snapshot, terrain_now.lo);
#endif
            if (motion_downhill_fault())
            {
                motion_apply(LINE_UNCERTAIN);
                tm_printf((UB *)"[CTRL] SAFE STOP: downhill fault=%u\n",
                          (UINT)motion_downhill_fault());
                state = ROBOT_STOPPED;
                report_fault(motion_downhill_fault() == 2U ? 9U : 11U);
            }
        }
#if APP_IMU_ENABLED
        if ((state == ROBOT_LINE_FOLLOWING || state == ROBOT_OBSTACLE) &&
            (!imu_clock_ok || !g_control.imu_seen ||
             (uint32_t)(terrain_now.lo - g_imu_last_good_msec) > 1000U))
        {
            motion_apply(LINE_UNCERTAIN);
            state = ROBOT_STOPPED;
            tm_printf((UB *)"[CTRL] SAFE STOP: IMU unavailable\n");
            report_fault(10U);
        }
#if APP_LIFT_STOP_ENABLED
        /* Picked up while it may drive: cut the motors at once. With
         * APP_LIFT_RESUME_ENABLED the robot pauses and carries on line
         * following once it is put back down; otherwise the stop is
         * latched like the other safety stops (STOP, then START). */
        if (imu_fresh && snapshot.b_lifted &&
            (state == ROBOT_IDLE || state == ROBOT_LINE_FOLLOWING ||
             state == ROBOT_AT_JUNCTION || state == ROBOT_OBSTACLE))
        {
            motion_stop_now();
            /* A bypass or capture in progress is meaningless once moved. */
            obstacle_abort();
#ifdef B1_LIVE_COMMANDS
            g7_nav_clear_decoded();
#endif
            tm_printf((UB *)"[CTRL] %s: lifted about %u mm (limit %u mm)\n",
                      APP_LIFT_RESUME_ENABLED ? "LIFTED, paused" :
                                                "SAFE STOP",
                      (UINT)snapshot.lift_height_mm,
                      (UINT)APP_LIFT_STOP_HEIGHT_MM);
            state = APP_LIFT_RESUME_ENABLED ? ROBOT_LIFTED : ROBOT_STOPPED;
#if APP_LIFT_RESUME_ENABLED
            g_lift_floor_since = 0U;
#endif
            report_fault(14U);
        }
#if APP_LIFT_RESUME_ENABLED
        else if (state == ROBOT_LIFTED)
        {
            state = lift_try_resume(imu_fresh &&
                                        snapshot.b_floor_contact &&
                                        !snapshot.b_lifted &&
                                        (snapshot.lift_height_mm <
                                         APP_LIFT_STOP_HEIGHT_MM),
                                    terrain_now.lo, p_history);
        }
#endif
#endif
#else
        (void)terrain_error;
#endif
        switch (state)
        {
            case ROBOT_IDLE:
                state = handle_idle();
            break;

            case ROBOT_LINE_FOLLOWING:
                state = handle_following(p_history);
            break;

            case ROBOT_OBSTACLE:
                state = handle_obstacle(p_history);
            break;

            case ROBOT_AT_JUNCTION:
                state = handle_junction(p_history);
            break;

            case ROBOT_REMOTE_HOLD:
                /* STOP is latched until a new explicit authorised START. */
                motion_apply(LINE_UNCERTAIN);
            break;

            case ROBOT_LIFTED:
                /* Held off until lift_try_resume() sees the floor. */
                motion_apply(LINE_UNCERTAIN);
            break;

            case ROBOT_STOPPED:
            case ROBOT_NAV_SAFETY_STOP:
                /* Do not move until an explicit STOP resets to remote hold
                 * and a later, separately authorized START is received. */
                motion_apply(LINE_UNCERTAIN);
            break;

            case ROBOT_ERROR:
                /* Remain in error until reset or future recovery. */
                motion_apply(LINE_UNCERTAIN);
            break;

            default:
                state = ROBOT_ERROR;
                motion_apply(LINE_UNCERTAIN);
            break;
        }

#ifdef B1_LIVE_COMMANDS
        remote_publish_state(state);
#endif
    return state;
}

#if APP_IMU_ENABLED && APP_LIFT_STOP_ENABLED && APP_LIFT_RESUME_ENABLED
static robot_state_t lift_try_resume (bool b_on_floor, uint32_t now_msec,
                                      controller_history_t * p_history)
{
    if (!b_on_floor)
    {
        /* Still in the air, or a hand under a sensor: start over. */
        g_lift_floor_since = 0U;
        return ROBOT_LIFTED;
    }

    if (0U == g_lift_floor_since)
    {
        /* At clock wrap, wait for the next nonzero time to start. */
        g_lift_floor_since = now_msec;
        tm_printf((UB *)"[CTRL] Put down -> resuming if it stays down\n");
        return ROBOT_LIFTED;
    }

    if ((uint32_t)(now_msec - g_lift_floor_since) < APP_LIFT_RESUME_MSEC)
    {
        return ROBOT_LIFTED;
    }

#ifdef B1_LIVE_COMMANDS
    /* The scanner was cancelled at the lift and rearms by itself; it may
     * finish while driving, as after an obstacle bypass. */
    if (!nav_handoff_continue())
    {
        g_lift_floor_since = 0U;
        return ROBOT_STOPPED;
    }

    nav_start(&g_navigation, now_msec);
#endif
    /* Line touches latched while carried are not steering. */
    {
        bool b_left = false;
        bool b_right = false;
        bool b_seen = false;

        (void)ir_shared_take_junction(&b_seen);
        (void)ir_shared_take_sides(&b_left, &b_right);
        (void)ir_shared_take_both_gap(&b_seen);
    }
    p_history->b_have_previous_action = false;
    g_lift_floor_since = 0U;
    /* Let the next lift be reported again. */
    g_control.fault = 0U;
    tm_printf((UB *)"[CTRL] Back on the floor -> resuming line following\n");
    return ROBOT_IDLE;
}
#endif

static robot_state_t handle_idle (void)
{
    if (obstacle_has_fault())
    {
        motion_apply(LINE_UNCERTAIN);
        obstacle_abort();
        return ROBOT_ERROR;
    }
#if APP_IMU_ENABLED
    if (!terrain_is_calibrated())
    {
        return ROBOT_IDLE;
    }
#endif

    ir_shared_snapshot_t reading = {0};
    ER                   error   = ir_shared_get_latest(&reading);

    if (error < E_OK)
    {
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] IR read error: %d\n", error);
        return ROBOT_ERROR;
    }

    if (!reading.b_valid)
    {
        return ROBOT_IDLE;
    }

    SYSTIM now = {0};
    error      = tk_get_otm(&now);

    if (error < E_OK)
    {
        return ROBOT_ERROR;
    }

    /* UW is the BSP's unsigned 32-bit type; subtraction wraps as before. */
    if ((UW)(now.lo - reading.timestamp_msec) <= APP_IR_MAX_AGE_MSEC)
    {
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] IDLE -> LINE_FOLLOWING\n");
        return ROBOT_LINE_FOLLOWING;
    }

    return ROBOT_IDLE;
}

static robot_state_t handle_following (controller_history_t * p_history)
{
#ifdef B1_LIVE_COMMANDS
    (void)p_history;
#endif
    ir_shared_snapshot_t reading = {0};
    ER                   error   = ir_shared_get_latest(&reading);

    if (error < E_OK)
    {
        motion_apply(LINE_UNCERTAIN);
        tm_printf((UB *)"[CTRL] IR snapshot read error %d -> ERROR\n",
                  (INT)error);
        report_fault(3U);
        return ROBOT_ERROR;
    }

    SYSTIM now = {0};
    error      = tk_get_otm(&now);

    if (error < E_OK)
    {
        motion_apply(LINE_UNCERTAIN);
        tm_printf((UB *)"[CTRL] Clock read error %d -> ERROR\n", (INT)error);
        report_fault(4U);
        return ROBOT_ERROR;
    }

    /* UW is unsigned 32-bit; elapsed time retains modulo-2^32 behaviour. */
    if ((!reading.b_valid) ||
        ((UW)(now.lo - reading.timestamp_msec) > APP_IR_MAX_AGE_MSEC))
    {
        motion_apply(LINE_UNCERTAIN);
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] IR data stale (%s, age %u ms) -> STOPPED\n",
                  reading.b_valid ? "old" : "invalid",
                  (UINT)(now.lo - reading.timestamp_msec));
        report_fault(2U);
        return ROBOT_STOPPED;
    }

    /* The uploaded obstacle policy takes priority over barcode maneuvers. */
    if (obstacle_has_fault())
    {
        motion_apply(LINE_UNCERTAIN);
        obstacle_abort();
        tm_printf((UB *)"[CTRL] Obstacle sensor fault -> ERROR\n");
        report_fault(5U);
        return ROBOT_ERROR;
    }
    if (obstacle_is_blocked())
    {
        line_heading_publish();
        motion_apply(LINE_UNCERTAIN);
#ifdef B1_LIVE_COMMANDS
        /* A route queued before leaving the line cannot drive the bypass. */
        nav_stop(&g_navigation);
        g7_nav_clear_decoded();
#endif
        tm_printf((UB *)"[CTRL] Obstacle confirmed -> OBSTACLE\n");
        return ROBOT_OBSTACLE;
    }

    bool b_junction = false;
    bool b_left     = false;
    bool b_right    = false;
    error           = ir_shared_take_junction(&b_junction);

    if (error >= E_OK)
    {
        error = ir_shared_take_sides(&b_left, &b_right);
    }

    if (error < E_OK)
    {
        motion_apply(LINE_UNCERTAIN);
        tm_printf((UB *)"[CTRL] IR latch read error %d -> ERROR\n",
                  (INT)error);
        return ROBOT_ERROR;
    }

#ifndef B1_LIVE_COMMANDS
    if (b_junction && ((1U != reading.left) || (1U != reading.right)))
    {
        /* Legacy profile preserves between-reads junction latching. */
        reading.left = 1U;
        reading.right = 1U;
        tm_printf((UB *)"[CTRL] Junction seen between reads\n");
    }
    else
#endif
    if ((b_left != b_right) && (0U == reading.left) &&
        (0U == reading.right))
    {
        /* One edge was touched between reads; still correct for it. Touches
         * on both sides at different times are left to the latest reading.
         */
        reading.left  = b_left ? 1U : 0U;
        reading.right = b_right ? 1U : 0U;
        /* UB views the same string bytes; tm_printf does not modify them. */
        //tm_printf((UB *)"[CTRL] Line touch seen between reads\n");
    }
    else
    {
        /* The latest reading already reflects what was seen. */
    }

    /* Where the line is, for motion's lost-line search, even in periods
     * navigation drives straight (both-black runs) and steering is told
     * nothing. */
    motion_line_seen(0U != reading.left, 0U != reading.right);

#ifdef B1_LIVE_COMMANDS
    {
        nav_sample_t sample = {0};
        nav_result_t decision;
        uint8_t letter = 0U, mapped = 0U;
        uint16_t sequence = 0U;
        uint32_t captured_msec = 0U;
        uint32_t completed_msec = 0U;
        uint8_t scan_pending = 1U;
        int received;
        bool b_gap = false;
        if ((0U != g_nav_handoff_retry) && g7_nav_continue_run())
        {
            g_nav_handoff_retry = 0U;
            tm_printf((UB *)"[NAV] Barcode handoff restored\n");
        }
        /* Was both-black broken by white between these 50 ms reads? A
         * failed read is not worth stopping the robot for: treat it as a
         * gap, which only restarts the junction distance measurement. */
        if (ir_shared_take_both_gap(&b_gap) < E_OK)
        {
            b_gap = true;
        }
        sample.both_interrupted = (uint8_t)b_gap;
        sample.now_msec = now.lo;
        sample.turn_counts = nav_path_counts();
        sample.travel_counts = sample.turn_counts / 2U;
        sample.left_black = (uint8_t)(reading.left != 0U);
        sample.right_black = (uint8_t)(reading.right != 0U);
        sample.barcode_capturing = bc_wave_navigation_guard_active();
        if (bc_wave_capture_active())
        {
            if (0U == g_capture_was_active)
            {
                g_capture_start_travel = sample.travel_counts;
            }
            g_capture_was_active = 1U;
            sample.capture_start_known = 1U;
            sample.capture_start_counts = g_capture_start_travel;
            /* Where the robot was when the barcode was last under it. */
            g_capture_end_travel = sample.travel_counts;
            g_control.capture_known = 1U;
        }
        else
        {
            g_capture_was_active = 0U;
        }
        received = g7_nav_take_scan(&letter, &mapped, &sequence,
                                     &captured_msec, &completed_msec,
                                     &scan_pending);
        sample.barcode_decoding = (uint8_t)(scan_pending && received == 0);
        if (received > 0)
        {
            /* The letter arrives ~2 s after the barcode (decoding), so
             * distance to the junction is counted from where the capture
             * ended, not from now. Logged: F arrived 0.7 s before a junction
             * ~190 mm past the barcode and was refused as "too close". */
            uint32_t const at_barcode =
                g_control.capture_known ?
                g_capture_end_travel : sample.travel_counts;
            int queued = nav_barcode(&g_navigation, letter, mapped,
                                     sequence, completed_msec, at_barcode);
            if (queued == 1)
            {
                nav_report(NAV_EVENT_QUEUED, mapped);
                tm_printf((UB *)"[NAV] accepted letter=%c action=%u seq=%u "
                          "capture_ms=%u done_ms=%u now=%u state=%u\n",
                          (INT)('A' + letter), (UINT)mapped, (UINT)sequence,
                          (UINT)captured_msec, (UINT)completed_msec,
                          (UINT)now.lo,
                          (UINT)g_navigation.state);
            }
            else if (queued < 0)
            {
                nav_report(NAV_EVENT_SCAN_REJECTED, 1U);
            }
        }
        else if (received < 0)
        {
            /* -1 a letter outside the route mapping (or none loaded),
             * -2 no letter decoded, -3 an unread result overwritten. */
            nav_report(NAV_EVENT_SCAN_REJECTED, (received == -3) ? 3U : 2U);
#if APP_NAV_SCAN_FAIL_STOP
            if (received == -1 || received == -3)
            {
                motion_brake();
                g_navigation.state = NAV_FAULT;
                g_navigation.fault_reason = 1U;
                g_navigation.pending = 0U;
                tm_printf((UB *)"[NAV] SAFE STOP: scan %s seq=%u now=%u\n",
                    received == -1 ? "unmapped" : "handoff overflow",
                    (UINT)sequence, (UINT)now.lo);
                report_fault(received == -1 ? 13U : 12U);
                return ROBOT_NAV_SAFETY_STOP;
            }
#else
            /* Treated like a failed decode: no action is queued and line
             * following carries on. At the next junction the robot goes
             * straight or repeats its remembered action (app_config.h). */
            if (received == -1)
            {
                tm_printf((UB *)"[NAV] letter=%c not in route mapping; "
                                "ignored, continuing seq=%u\n",
                          (INT)('A' + letter), (UINT)sequence);
            }
            else if (received == -3)
            {
                /* The overflow mark stays on the handoff until it is
                 * reset, which would reject every later barcode too. */
                tm_printf((UB *)"[NAV] scan result lost (handoff "
                                "overflow); ignored, continuing seq=%u\n",
                          (UINT)sequence);
                /* Without APP_NAV_SCAN_FAIL_STOP this branch is compiled,
                 * so a failure here keeps following and retries. */
                (void)nav_handoff_continue();
            }
            else
            {
                /* No letter decoded: nothing to do, as before. */
            }
#endif
        }
        decision = nav_step(&g_navigation, &sample);
        if ((NAV_OUTPUT_FOLLOW == decision.drive) ||
            (NAV_OUTPUT_FORWARD == decision.drive))
        {
            line_heading_update();
        }
        else
        {
            /* Junction turns and stops: the line ahead is a new one. */
            line_heading_reset();
        }
        if (0U != g_navigation.tape_capture)
        {
            /* The capture is the crossing tape: drop it so it cannot
             * freeze into a junk letter, and keep going. */
            g_navigation.tape_capture = 0U;
            bc_wave_request_drop();
            tm_printf((UB *)"[NAV] Junction tape under the barcode sensor: "
                            "capture ignored, not waiting for a letter\n");
        }
        if (0U != g_navigation.clearance_dropped)
        {
            g_navigation.clearance_dropped = 0U;
            tm_printf((UB *)"[NAV] Barcode too close to the junction; "
                            "action dropped, crossing straight now=%u\n",
                      (UINT)now.lo);
        }
#if APP_JUNCTION_DEBUG_LOG
        {
            /* What the 1 ms sampler saw over this 50 ms period, printed
             * only while either sensor saw black. run = counts both-black
             * so far (junction at APP_NAV_JUNCTION_CONFIRM_COUNTS). */
            ir_shared_diag_t diag = {0U, 0U, 0U, 0U};
            if ((ir_shared_take_diag(&diag) >= E_OK) &&
                ((diag.left != 0U) || (diag.right != 0U)))
            {
                uint32_t const run = (g_navigation.black_since != 0U) ?
                    (sample.travel_counts - g_navigation.black_from_counts) :
                    0U;
                tm_printf((UB *)"[JCT] n=%u L=%u R=%u both=%u gap=%u "
                          "now=%u%u run=%u drive=%u nav=%u\n",
                          (UINT)diag.samples, (UINT)diag.left,
                          (UINT)diag.right, (UINT)diag.both, (UINT)b_gap,
                          (UINT)sample.left_black, (UINT)sample.right_black,
                          (UINT)run, (UINT)decision.drive,
                          (UINT)g_navigation.state);
            }
        }
#endif
        if ((decision.event == NAV_EVENT_TURN_COMPLETED) ||
            ((decision.event == NAV_EVENT_FAULT) && (decision.value == 2U)))
        {
            /* For calibrating APP_NAV_TRACK_MM: how far the pivot went. */
            tm_printf((UB *)"[NAV] Turn %s after %u counts "
                      "(90 deg = %u, action %u)\n",
                      (decision.event == NAV_EVENT_FAULT) ?
                          "FAILED" : "done",
                      (UINT)(sample.turn_counts - g_navigation.turn_from),
                      (UINT)APP_NAV_TURN90_COUNTS,
                      (UINT)g_navigation.action);
        }
        if (decision.event != NAV_EVENT_NONE)
        {
            nav_report(decision.event, decision.value);
            tm_printf((UB *)"[NAV] event=%u value=%u state=%u seq=%u now=%u\n",
                (UINT)decision.event, (UINT)decision.value,
                (UINT)g_navigation.state, (UINT)sequence, (UINT)now.lo);
            if (decision.event == NAV_EVENT_FAULT)
            {
                static char const *const reasons[] = {
                    "unknown", "no usable action or barcode clearance",
                    "turn timeout or angle limit", "cross/centre timeout",
                    "invalid navigation state", "decode timeout"
                };
                tm_printf((UB *)"[NAV] SAFE STOP: %s seq=%u now=%u\n",
                    reasons[decision.value < 6U ? decision.value : 0U],
                    (UINT)g_navigation.last_barcode_seq, (UINT)now.lo);
                report_fault(decision.value == 5U ? 6U :
                    decision.value == 2U ? 8U : 7U);
            }
        }
        switch (decision.drive)
        {
            case NAV_OUTPUT_FOLLOW:
                /* Both black is a crossing, a tape join or a bar: keep
                 * driving on the current steer. Stopping here zeroed the
                 * speed for a period and the robot then ramped up from
                 * rest (a stop-and-jerk on a straight). nav_core confirms a
                 * real junction over APP_NAV_JUNCTION_CONFIRM_MM of travel
                 * and brakes through NAV_OUTPUT_STOP. */
                if (reading.left && reading.right)
                    motion_apply(LINE_FORWARD);
                else
                    motion_apply(line_follow_decide(reading.left,
                                                   reading.right));
                break;
            case NAV_OUTPUT_FORWARD:
                motion_apply(LINE_FORWARD);
                break;
            case NAV_OUTPUT_LEFT:
                motion_apply_maneuver(NAV_LEFT);
                break;
            case NAV_OUTPUT_RIGHT:
                motion_apply_maneuver(NAV_RIGHT);
                break;
            case NAV_OUTPUT_UTURN:
                motion_apply_maneuver(NAV_UTURN);
                break;
            default:
                /* Junction stop, decode wait or safety stop: brake so the
                 * robot halts where it confirmed, instead of coasting. */
                motion_brake();
                break;
        }
        /* Navigation geometry/line ambiguity is a recoverable SAFE STOP,
         * not a driver/kernel fault. The distinction matters: only a
         * later explicit STOP may clear this latch for a new START. */
        return (g_navigation.state == NAV_FAULT) ?
               ROBOT_NAV_SAFETY_STOP : ROBOT_LINE_FOLLOWING;
    }
#else
    return apply_action(&reading, p_history);
#endif
}

static robot_state_t handle_obstacle (controller_history_t * p_history)
{
    ir_shared_snapshot_t reading = {0};
    SYSTIM now = {0};
    bool b_left = false, b_right = false, b_both = false;
    ER error = ir_shared_get_latest(&reading);
    if (error >= E_OK) error = tk_get_otm(&now);
    if (error >= E_OK) error = ir_shared_take_sides(&b_left, &b_right);
    if (error >= E_OK) error = ir_shared_take_junction(&b_both);
    if (error < E_OK || !reading.b_valid ||
        (UW)(now.lo - reading.timestamp_msec) > APP_IR_MAX_AGE_MSEC)
    {
        motion_apply(LINE_UNCERTAIN);
        obstacle_abort();
        tm_printf((UB *)"[CTRL] IR unavailable during bypass -> STOPPED\n");
        return ROBOT_STOPPED;
    }
    if (obstacle_has_fault())
    {
        motion_apply(LINE_UNCERTAIN);
        obstacle_abort();
        return ROBOT_ERROR;
    }
    obstacle_drive_t const drive = obstacle_bypass_step(
        (b_left || b_both) ? 1U : reading.left,
        (b_right || b_both) ? 1U : reading.right, now.lo);
    if (drive.b_fault)
    {
        motion_apply(LINE_UNCERTAIN);
        obstacle_abort();
        tm_printf((UB *)"[CTRL] Obstacle bypass fault -> ERROR\n");
        return ROBOT_ERROR;
    }
    if (drive.b_done)
    {
        /* Stop before handing ownership back; discard bypass line touches. */
        motion_apply(LINE_UNCERTAIN);
        /* With both sensors white the lost-line check had no side to lean
         * to and drove straight past the line. */
        motion_line_hint((int)drive.line_hint);
        (void)ir_shared_take_junction(&b_both);
        (void)ir_shared_take_sides(&b_left, &b_right);
        p_history->b_have_previous_action = false;
#ifdef B1_LIVE_COMMANDS
        /* The scanner was cancelled when the obstacle was confirmed and
         * usually rearms while suppressed during the bypass. If it is still
         * rearming, keep driving: it finishes on the move, as after every
         * barcode, and hands over the next capture it starts. A barcode
         * passed before it is armed is not read. Do not cancel it again. */
        if (!bc_wave_is_ready())
        {
            /* UB views the same string bytes; tm_printf does not modify. */
            tm_printf((UB *)"[CTRL] Bypass done; barcode scanner still "
                            "rearming, continuing\n");
        }
        if (!nav_handoff_continue())
        {
            motion_apply(LINE_UNCERTAIN);
            return ROBOT_STOPPED;
        }
        nav_start(&g_navigation, now.lo);
#endif
        tm_printf((UB *)"[CTRL] Bypass done -> LINE_FOLLOWING\n");
        return ROBOT_LINE_FOLLOWING;
    }
    if (drive.b_stop)
    {
        motion_apply(LINE_UNCERTAIN);
    }
    else if (drive.b_line_follow)
    {
        line_follow_action_t action =
            line_follow_decide(reading.left, reading.right);
        motion_apply((action == LINE_UNCERTAIN) ? LINE_FORWARD : action);
    }
    else
    {
        motion_drive((float)drive.left_rpm, (float)drive.right_rpm);
    }
    return ROBOT_OBSTACLE;
}

static robot_state_t handle_junction (controller_history_t * p_history)
{
    ir_shared_snapshot_t reading = {0};
    bool                 b_black = false;
    ER                   error   = ir_shared_get_latest(&reading);

    if (error >= E_OK)
    {
        /* Also clears the latch, so stale both-black cannot leak out. */
        error = ir_shared_take_junction(&b_black);
    }

    SYSTIM now = {0};

    if (error >= E_OK)
    {
        error = tk_get_otm(&now);
    }

    if (error < E_OK)
    {
        /* Motors are already stopped. */
        return ROBOT_ERROR;
    }

    /* UW is unsigned 32-bit; elapsed time retains modulo-2^32 behaviour. */
    bool const b_fresh =
        reading.b_valid &&
        ((UW)(now.lo - reading.timestamp_msec) <= APP_IR_MAX_AGE_MSEC);

    if (p_history->junction_cycles < JUNCTION_HOLD_CYCLES)
    {
        /* Ignore readings while the robot may still be coasting. */
        p_history->junction_cycles++;
        return ROBOT_AT_JUNCTION;
    }

    if (b_black)
    {
        if (!p_history->b_rearmed)
        {
            /* UB views the same string bytes; tm_printf does not modify
             * them.
             */
            tm_printf((UB *)"[CTRL] Lifted -> waiting to be placed on a "
                            "line\n");
        }

        p_history->b_rearmed     = true;
        p_history->settle_cycles = 0U;
        return ROBOT_AT_JUNCTION;
    }

    if ((!p_history->b_rearmed) || (!b_fresh))
    {
        p_history->settle_cycles = 0U;
        return ROBOT_AT_JUNCTION;
    }

    p_history->settle_cycles++;

    if (1U == p_history->settle_cycles)
    {
        /* Printed again whenever a both-black reading restarts the wait.
         * UB views the same string bytes; tm_printf does not modify them.
         */
        tm_printf((UB *)"[CTRL] Placed -> starting if the line stays\n");
    }

    if (p_history->settle_cycles < JUNCTION_SETTLE_CYCLES)
    {
        return ROBOT_AT_JUNCTION;
    }

    bool b_left  = false;
    bool b_right = false;

    /* Discard touches from being lifted and placed; they are not steering. */
    if (ir_shared_take_sides(&b_left, &b_right) < E_OK)
    {
        return ROBOT_ERROR;
    }

    /* Log the first action after resuming, whatever it is. */
    p_history->b_have_previous_action = false;
    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[CTRL] Back on line -> LINE_FOLLOWING\n");
    return ROBOT_LINE_FOLLOWING;
}

#ifndef B1_LIVE_COMMANDS
static robot_state_t apply_action (ir_shared_snapshot_t const * p_reading,
                                   controller_history_t *       p_history)
{
    line_follow_action_t action =
        line_follow_decide(p_reading->left, p_reading->right);

    bool const b_new_junction =
        (LINE_UNCERTAIN == action) &&
        ((!p_history->b_have_previous_action) ||
         (LINE_UNCERTAIN != p_history->previous_action));

    if ((LINE_UNCERTAIN == action) && (0 == APP_JUNCTION_STOP_ENABLED))
    {
        /* Stopping is overridden: drive straight across the junction. */
        motion_apply(LINE_FORWARD);

        if (b_new_junction)
        {
            p_history->junction_count++;
            /* UINT matches %u on the 32-bit target; UB preserves bytes. */
            tm_printf((UB *)"[CTRL] Junction %u -> crossing (stop "
                            "disabled)\n",
                      (UINT)p_history->junction_count);
        }

        p_history->previous_action        = LINE_UNCERTAIN;
        p_history->b_have_previous_action = true;
        return ROBOT_LINE_FOLLOWING;
    }

    if (LINE_UNCERTAIN == action)
    {
        motion_apply(LINE_UNCERTAIN);
        p_history->junction_count++;
        p_history->junction_cycles = 0U;
        p_history->settle_cycles   = 0U;
        p_history->b_rearmed       = false;
        /* UINT matches %u on the 32-bit target; UB preserves string bytes. */
        tm_printf((UB *)"[CTRL] Junction %u -> STOPPED until placed on a "
                        "line\n",
                  (UINT)p_history->junction_count);
        return ROBOT_AT_JUNCTION;
    }

    motion_apply(action);

    if ((!p_history->b_have_previous_action) ||
        (action != p_history->previous_action))
    {
        /* UINT matches the original %u arguments on the 32-bit target.
         * UB views string bytes; tm_printf does not modify the string.
         */
        //tm_printf((UB *)"[CTRL] L=%u R=%u ACTION=%s\n", (UINT)p_reading->left,
        //          (UINT)p_reading->right, line_follow_text(action));

        p_history->previous_action        = action;
        p_history->b_have_previous_action = true;
    }

    return ROBOT_LINE_FOLLOWING;
}

#endif /* !B1_LIVE_COMMANDS */

/*** end of file ***/

