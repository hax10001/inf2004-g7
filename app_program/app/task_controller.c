/** @file task_controller.c
 * @brief Preserve controller transitions, freshness checks, and motor actions.
 */
#include "task_controller.h"
#include "app_config.h"
#include "nav_core.h"
#include "drivers/encoder.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/line_barcode/line_follow.h"
#include "modules/motion/motion.h"
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
    ROBOT_REMOTE_HOLD
} robot_state_t;

typedef struct
{
    line_follow_action_t previous_action;
    bool                 b_have_previous_action;
    uint32_t             junction_count;
    uint32_t             junction_cycles; /**< Periods since stopping. */
    uint32_t             settle_cycles;   /**< Periods back on a line. */
    bool                 b_rearmed;       /**< Both black after the hold. */
} controller_history_t;

/** @brief Wait for startup calibration and fresh IR data. */
static robot_state_t handle_idle (void);
/** @brief Follow the line, preserving stop and error behaviour. */
static robot_state_t handle_following (controller_history_t * p_history);
/** @brief Stay stopped at a junction until placed back on a line.
 *
 * The IR sensors read black when lifted. Both black seen after the hold
 * period means the robot is still on the junction or has been picked up;
 * a coast off the junction ends within the hold and is not counted. A
 * steady line reading after that resumes following.
 */
static robot_state_t handle_junction (controller_history_t * p_history);
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
    if (kind == NAV_EVENT_NONE || g_remote_mutex <= E_OK) return;
    if (tk_loc_mtx(g_remote_mutex, TMO_POL) < E_OK) return;
    g_nav_report_event = (uint8_t)kind;
    g_nav_report_value = value;
    ++g_nav_report_seq;
    (void)tk_unl_mtx(g_remote_mutex);
}
static uint32_t nav_travel_counts(void)
{
    encoder_sample_t left = {0};
    encoder_sample_t right = {0};
    int64_t l, r;
    encoder_read(ENCODER_LEFT, &left);
    encoder_read(ENCODER_RIGHT, &right);
    l = left.count;
    r = right.count;
    if (l < 0) l = -l;
    if (r < 0) r = -r;
    return (uint32_t)(((uint64_t)l + (uint64_t)r) / 2U);
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
        case ROBOT_AT_JUNCTION: return 6U;
        case ROBOT_STOPPED: return 6U;
        case ROBOT_NAV_SAFETY_STOP: return 5U; /* Navigation safe stop */
        case ROBOT_ERROR: return 7U;
        case ROBOT_REMOTE_HOLD: return 6U;
        default: return 1U;
    }
}

/* Readiness bit meanings are mirrored in dashboard/server/protocol.js.
 * 1: MQTT offline; 2: not held; 4: STOP/START pending;
 * 8: IMU uncalibrated; 16: IR invalid; 32: IR stale;
 * 64: controller lock/clock unavailable. Never relax these gates. */
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
    /* Active frozen waveform processing cannot safely be rearmed.
     * Completed one-shot captures CAN be rearmed by explicit START. */
    if (!bc_wave_can_rearm())
    {
        reason |= 128U;
    }
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
    nav_stop(&g_navigation);
    return (prior == ROBOT_ERROR) ? ROBOT_ERROR : ROBOT_REMOTE_HOLD;
}

/** @brief Read queued requests without blocking the motor owner.
 * If the mutex is briefly unavailable, force a stop without clearing
 * the pending request; a later iteration will consume it safely.
 */
static void remote_take_requests (uint8_t * p_stop, uint8_t * p_start,
                                  robot_state_t state)
{
    *p_stop = 0U;
    *p_start = 0U;
    if (g_remote_mutex <= E_OK ||
        tk_loc_mtx(g_remote_mutex, TMO_POL) < E_OK)
    {
        *p_stop = 1U;
        return;
    }
    *p_stop = g_stop_pending;
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
        (void)tk_unl_mtx(g_remote_mutex);
    }
}
#endif

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
            APP_NAV_SCAN_CROSS_MAX_MSEC,
            APP_NAV_SCAN_CROSS_MAX_COUNTS
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
#ifdef B1_LIVE_COMMANDS
        {
            uint8_t stop_request;
            uint8_t start_request;

            remote_take_requests(&stop_request, &start_request, state);
            /* Network failure must never resume a stopped robot. */
            if (state != ROBOT_REMOTE_HOLD && state != ROBOT_ERROR &&
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
                motion_apply(LINE_UNCERTAIN);
                g7_nav_clear_decoded();
                /* A navigation safety stop is recoverable only by STOP.
                 * A real error remains latched. No automatic restart. */
                state = remote_reset_after_stop(state);
                tm_printf((UB *)"[CTRL] Remote STOP applied\n");
            }
            else if (state == ROBOT_REMOTE_HOLD && start_request != 0U &&
                     b1_mqtt_connected())
            {
                SYSTIM started = {0};
                /* The scanner rearms itself after its worker finishes.
                 * This START is the only authorization to begin movement;
                 * neither scan completion nor MQTT reconnect can do so. */
                if (tk_get_otm(&started) >= E_OK)
                {
                    g7_nav_clear_decoded();
                    nav_start(&g_navigation, started.lo);
                    state = ROBOT_IDLE;
                    tm_printf((UB *)"[CTRL] Remote START accepted\n");
                }
            }
            remote_publish_state(state);
        }
#endif
        switch (state)
        {
            case ROBOT_IDLE:
                state = handle_idle();
            break;

            case ROBOT_LINE_FOLLOWING:
                state = handle_following(&history);
            break;

            case ROBOT_AT_JUNCTION:
                state = handle_junction(&history);
            break;

            case ROBOT_REMOTE_HOLD:
                /* STOP is latched until a new explicit authorised START. */
            break;

            case ROBOT_STOPPED:
            case ROBOT_NAV_SAFETY_STOP:
                /* Do not move until an explicit STOP resets to remote hold
                 * and a later, separately authorized START is received. */
                motion_apply(LINE_UNCERTAIN);
            break;

            case ROBOT_ERROR:
                /* Remain in error until reset or future recovery. */
            break;

            default:
                state = ROBOT_ERROR;
                motion_apply(LINE_UNCERTAIN);
            break;
        }

#ifdef B1_LIVE_COMMANDS
        remote_publish_state(state);
#endif
        tk_dly_tsk(APP_CONTROLLER_PERIOD_MSEC);
    }
}

static robot_state_t handle_idle (void)
{
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
        return ROBOT_ERROR;
    }

    SYSTIM now = {0};
    error      = tk_get_otm(&now);

    if (error < E_OK)
    {
        motion_apply(LINE_UNCERTAIN);
        return ROBOT_ERROR;
    }

    /* UW is unsigned 32-bit; elapsed time retains modulo-2^32 behaviour. */
    if ((!reading.b_valid) ||
        ((UW)(now.lo - reading.timestamp_msec) > APP_IR_MAX_AGE_MSEC))
    {
        motion_apply(LINE_UNCERTAIN);
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] IR data stale -> STOPPED\n");
        return ROBOT_STOPPED;
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

#ifdef B1_LIVE_COMMANDS
    {
        nav_sample_t sample = {0};
        nav_result_t decision;
        uint8_t letter = 0U, mapped = 0U;
        uint16_t sequence = 0U;
        int received;
        sample.now_msec = now.lo;
        sample.travel_counts = nav_travel_counts();
        sample.left_black = (uint8_t)(reading.left != 0U);
        sample.right_black = (uint8_t)(reading.right != 0U);
        sample.barcode_capturing = bc_wave_navigation_guard_active();
        received = g7_nav_take_decoded(&letter, &mapped, &sequence);
        if (received > 0)
        {
            int queued = nav_barcode(&g_navigation, letter, mapped,
                                     sequence, now.lo, sample.travel_counts);
            if (queued == 1)
            {
                nav_report(NAV_EVENT_QUEUED, mapped);
                tm_printf((UB *)"[NAV] Decoded %c -> action %u queued\n",
                          (INT)('A' + letter), (UINT)mapped);
            }
            else if (queued < 0)
            {
                nav_report(NAV_EVENT_SCAN_REJECTED, 1U);
            }
        }
        else if (received < 0)
        {
            nav_report(NAV_EVENT_SCAN_REJECTED, 2U);
        }
        decision = nav_step(&g_navigation, &sample);
        if (decision.event != NAV_EVENT_NONE)
        {
            nav_report(decision.event, decision.value);
            tm_printf((UB *)"[NAV] event=%u value=%u state=%u\n",
                (UINT)decision.event, (UINT)decision.value,
                (UINT)g_navigation.state);
        }
        switch (decision.drive)
        {
            case NAV_OUTPUT_FOLLOW:
                if (reading.left && reading.right)
                    motion_apply(LINE_UNCERTAIN);
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
                motion_apply(LINE_UNCERTAIN);
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

