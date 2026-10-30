/** @file obstacle.c
 *
 * @brief Lifecycle and the bypass sequencer; shared status is obs_state.
 *
 * Distance and heading come from encoder counts since the bypass began.
 * Headings are in tenths of a degree and positive is a left turn, matching
 * motion.h. All turns are arcs about a stopped wheel because the motor
 * driver is forward-only.
 */
#include "obstacle.h"
#include "obs_config.h"
#include "obs_state.h"
#include "obs_geom.h"
#include "scan_plan.h"
#include "servo.h"
#include "task_obstacle.h"
#include "ultrasonic.h"
#include "app/app_config.h"
#include "app/tasks.h"
#include "modules/motion/motion.h"
#include "drivers/encoder.h"
#include <tm/tmonitor.h>

#define MM_PER_CM 10
#define DDEG_PER_DEG 10
/* Wheel circumference in tenths of a mm; folded at compile time. */
#define OBS_CIRC_MM_X10 ((int32_t)(BOARD_WHEEL_DIAMETER_MM * 31.4159265F))
#define OBS_COUNTS_PER_REV ((int32_t)BOARD_ENC_COUNTS_PER_REV)
/* Radians to tenths of a degree: 572.958 rounded. */
#define OBS_RAD_TO_DDEG 573
#define SEARCH_SCALE_NUM 3
#define SEARCH_SCALE_DEN 2

typedef struct
{
    int32_t dist_mm;
    int32_t heading_ddeg;
    int32_t left_mm;
    int32_t right_mm;
} obstacle_pose_t;

typedef struct
{
    uint32_t          b_active : 1;
    uint32_t          phase : 4;
    uint32_t          line_hits : 2;
    uint32_t          verify_cycles : 2;
    uint32_t          stop_hits : 2;
    uint32_t          rescan_count : 1;
    uint32_t          b_black_seen : 1;
    int32_t           side : 2; /**< 1 left, -1 right, zero before planning. */
    uint32_t          angle_ddeg : 10;
    uint32_t          diag_mm : 13;
    uint32_t          guard_epoch : 8;
    uint32_t          saved_phase : 4;
    uint32_t          b_reassess_scan : 1;
    uint32_t          guard_reason : 2;
    uint32_t          b_drive_left : 1;
    uint32_t          b_drive_right : 1;
    uint32_t          stall_left_msec : 11;
    uint32_t          stall_right_msec : 11;
    uint32_t          enter_msec;
    int32_t           base_left;
    int32_t           base_right;
    int32_t           seg_start_mm;
    int32_t           heading0_ddeg;
    uint32_t          mission_msec;
    int32_t           guard_dist_mm;
    int32_t           guard_heading_ddeg;
    int32_t           saved_start_mm;
    int32_t           rear_left_mm;
    int32_t           rear_right_mm;
    int32_t           previous_left;
    int32_t           previous_right;
    uint32_t          previous_msec;
} obstacle_seq_t;

static obstacle_seq_t g_seq;
static bool g_b_available = false;
static bool volatile g_b_retry_delay = false;
static uint32_t volatile g_resume_usec = 0U;

/** @brief Read the signed wheel counts; false until motion has data. */
static bool counts_read (int32_t * p_left, int32_t * p_right);
/** @brief Check straight-phase heading even while waiting for coverage. */
static bool path_deviated (obstacle_seq_t const * p_seq,
                           obstacle_pose_t const * p_pose);
/** @brief Convert a wheel count difference to millimetres. */
static int32_t counts_to_mm (int32_t counts);
/** @brief Signed count difference with defined arithmetic across wrap. */
static int32_t counts_delta (int32_t current, int32_t base);
/** @brief Require fresh coverage and bound translation and turning per scan. */
static bool permit_motion (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                            obstacle_pose_t const * p_pose, uint32_t now_msec);
/** @brief Stop and begin one bounded stationary reassessment. */
static void reassess (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                       obstacle_pose_t const * p_pose, uint32_t now_msec,
                       uint32_t reason);
/** @brief Start a bypass: zero the odometry and enter STOPPING. */
static void seq_begin (obstacle_seq_t * p_seq, int32_t left_count,
                       int32_t right_count, uint32_t now_msec);
/** @brief Enter a phase and restart its segment and timer. */
static void enter_phase (obstacle_seq_t * p_seq, obs_state_phase_t phase,
                         uint32_t now_msec, int32_t dist_mm);
/** @brief Run the current phase for one controller period. */
static void run_phase (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                       obstacle_pose_t const * p_pose, bool b_black,
                       uint32_t now_msec);
/** @brief Drive one wheel only; positive turn is left. */
static void set_arc (obstacle_drive_t * p_out, int32_t turn);
/** @brief Drive straight, trimming towards a target heading. */
static void set_straight (obstacle_drive_t * p_out, int32_t heading_ddeg,
                          int32_t target_ddeg);
/** @brief Stop, record a fault bit and abort the obstacle task. */
static void raise_fault (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                         uint32_t fault_bit);
/** @brief End the bypass and hand control back. */
static void finish_bypass (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                           bool b_count);
/** @brief Read the plan from the scan and start the first manoeuvre. */
static void handle_plan (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                         obstacle_pose_t const * p_pose, uint32_t now_msec);

ER obstacle_init (void)
{
    ER error = E_OK;

    g_b_available = false;
    g_b_retry_delay = false;

    if (0 != APP_OBSTACLE_ENABLED)
    {
        error = obs_state_init();

        if (error >= E_OK)
        {
            error = ultrasonic_init();
        }

        if (error >= E_OK)
        {
            error = servo_init();
        }

        if (error < E_OK)
        {
            /* UB views the same string bytes; tm_printf does not modify. */
            tm_printf((UB *)"[OBS] Init failed: %d\n", error);
        }
    }

    g_b_available = (error >= E_OK);
    if (error < E_OK)
    {
        tm_printf((UB *)"[OBS] Unavailable -> LINE_FOLLOWING\n");
    }
    return E_OK;
}

ER obstacle_start (void)
{
    ER error = E_OK;

    if ((0 != APP_OBSTACLE_ENABLED) && g_b_available)
    {
        ID const h_task =
            tasks_create(task_obstacle_task, APP_OBSTACLE_PRIORITY);

        if (h_task <= E_OK)
        {
            /* UB views the same string bytes; tm_printf does not modify. */
            tm_printf((UB *)"[OBS] Task creation failed: %d\n", h_task);
            error = (E_OK == h_task) ? E_OBJ : h_task;
        }
        else
        {
            error = tk_sta_tsk(h_task, 0);
        }
    }

    if (error < E_OK)
    {
        g_b_available = false;
        tm_printf((UB *)"[OBS] Task unavailable -> LINE_FOLLOWING\n");
    }
    return E_OK;
}

bool obstacle_is_blocked (void)
{
    bool b_blocked = false;

    if ((0 != APP_OBSTACLE_ENABLED) && g_b_available)
    {
        b_blocked = (0U != (obs_state_flags_read() & OBS_EVT_BLOCKED));
    }

    return b_blocked;
}

bool obstacle_has_fault (void)
{
    bool b_fault = false;

    if ((0 != APP_OBSTACLE_ENABLED) && g_b_available)
    {
        uint32_t const faults = obs_state_status_faults(obs_state_status_get());

        b_fault = (0U != (faults & (OBS_FAULT_NO_ECHO |
                                     OBS_FAULT_TIMEOUT | OBS_FAULT_SERVO |
                                     OBS_FAULT_NO_ROUTE |
                                     OBS_FAULT_LINE_LOST)));
    }

    return b_fault;
}

bool obstacle_front_ready (uint32_t now_msec)
{
    (void)now_msec;
    return true;
}

bool obstacle_monitor_ready (void)
{
    return g_b_available &&
           ((!g_b_retry_delay) ||
            ((encoder_now_usec() - g_resume_usec) >=
             (OBS_RETRY_DELAY_MSEC * 1000U)));
}

void obstacle_abort (void)
{
    if (0 != APP_OBSTACLE_ENABLED)
    {
        if (0U != obs_state_status_faults(obs_state_status_get()))
        {
            g_resume_usec = encoder_now_usec();
            g_b_retry_delay = true;
            tm_printf((UB *)"[OBS] Avoidance failed -> LINE_FOLLOWING\n");
        }
        obs_state_faults_update(0U, OBS_FAULT_NO_ECHO | OBS_FAULT_NO_ROUTE |
                                    OBS_FAULT_LINE_LOST | OBS_FAULT_SERVO |
                                    OBS_FAULT_TIMEOUT);
        obs_state_flags_clear(OBS_EVT_BLOCKED | OBS_EVT_SCAN_REQ |
                              OBS_EVT_SCAN_DONE | OBS_EVT_SIDE_WATCH |
                              OBS_EVT_SIDE_SEEN | OBS_EVT_SIDE_CLEAR);
        obs_state_set_phase(OBS_PHASE_IDLE);
        obs_state_flags_set(OBS_EVT_ABORT);
        g_seq.b_active = false;
        if (OBS_PHASE_FAULT != obs_state_status_phase(obs_state_status_get()))
        {
            obs_state_set_phase(OBS_PHASE_IDLE);
        }
        obs_state_set_route(OBS_ROUTE_NONE);
    }
}

obstacle_drive_t obstacle_bypass_step (uint32_t line_left, uint32_t line_right,
                                       uint32_t now_msec)
{
    obstacle_drive_t out         = {0, 0, false, false, false, false};
    int32_t          left_count  = 0;
    int32_t          right_count = 0;

    if (0 == APP_OBSTACLE_ENABLED)
    {
        out.b_done = true;
    }
    else if (!counts_read(&left_count, &right_count))
    {
        /* Lost odometry must not leave a timed manoeuvre executing. */
        obs_state_faults_update(OBS_FAULT_TIMEOUT, 0U);
        obs_state_set_phase(OBS_PHASE_FAULT);
        obs_state_flags_set(OBS_EVT_ABORT);
        out.b_stop = true;
        out.b_fault = true;
    }
    else
    {
        obstacle_seq_t * const p_seq = &g_seq;

        if (!p_seq->b_active)
        {
            seq_begin(p_seq, left_count, right_count, now_msec);
        }

        int64_t const left_step = (int64_t)counts_delta(
            left_count, p_seq->previous_left) * OBS_LEFT_FORWARD_SIGN;
        int64_t const right_step = (int64_t)counts_delta(
            right_count, p_seq->previous_right) * OBS_RIGHT_FORWARD_SIGN;

        if ((left_step < -2) || (right_step < -2) ||
            (left_step > OBS_ODOM_MAX_STEP_COUNTS) ||
            (right_step > OBS_ODOM_MAX_STEP_COUNTS))
        {
            raise_fault(p_seq, &out, OBS_FAULT_TIMEOUT);
            return out;
        }

        if ((0 == left_step) && (0 == right_step))
        {
            if (p_seq->stop_hits < OBS_STOP_CONFIRM_CYCLES)
            {
                p_seq->stop_hits++;
            }
        }
        else
        {
            p_seq->stop_hits = 0U;
        }

        int32_t const left_mm =
            counts_to_mm(counts_delta(left_count, p_seq->base_left)) *
                OBS_LEFT_FORWARD_SIGN;
        int32_t const right_mm =
            counts_to_mm(counts_delta(right_count, p_seq->base_right)) *
                OBS_RIGHT_FORWARD_SIGN;
        obstacle_pose_t const pose = {(left_mm + right_mm) / 2,
                                      ((right_mm - left_mm) * OBS_RAD_TO_DDEG) /
                                          OBS_TRACK_WIDTH_MM,
                                      left_mm, right_mm};
        bool const b_black = (1U == line_left) != (1U == line_right);
        uint32_t const drive_elapsed = now_msec - p_seq->previous_msec;

        if (left_count != p_seq->previous_left)
        {
            p_seq->stall_left_msec = 0U;
        }
        else if (p_seq->b_drive_left)
        {
            uint32_t const total = p_seq->stall_left_msec + drive_elapsed;
            p_seq->stall_left_msec = (total < OBS_STALL_DRIVE_MSEC)
                                        ? total : OBS_STALL_DRIVE_MSEC;
        }

        if (right_count != p_seq->previous_right)
        {
            p_seq->stall_right_msec = 0U;
        }
        else if (p_seq->b_drive_right)
        {
            uint32_t const total = p_seq->stall_right_msec + drive_elapsed;
            p_seq->stall_right_msec = (total < OBS_STALL_DRIVE_MSEC)
                                         ? total : OBS_STALL_DRIVE_MSEC;
        }

        p_seq->previous_left = left_count;
        p_seq->previous_right = right_count;
        p_seq->previous_msec = now_msec;

        if (path_deviated(p_seq, &pose))
        {
            raise_fault(p_seq, &out, OBS_FAULT_NO_ROUTE);
        }
        else if ((p_seq->stall_left_msec >= OBS_STALL_DRIVE_MSEC) ||
            (p_seq->stall_right_msec >= OBS_STALL_DRIVE_MSEC) ||
            ((now_msec - p_seq->mission_msec) > OBS_BYPASS_TIMEOUT_MSEC))
        {
            raise_fault(p_seq, &out, OBS_FAULT_TIMEOUT);
        }
        else if ((1U == line_left) && (1U == line_right) &&
                 (p_seq->phase >= OBS_PHASE_SEARCH_LINE) &&
                 (p_seq->phase <= OBS_PHASE_VERIFY))
        {
            /* Buddy 3 calls both-black a junction, not a recovered edge. */
            raise_fault(p_seq, &out, OBS_FAULT_LINE_LOST);
        }
        else
        {
            uint32_t const old_phase = p_seq->phase;
            bool const b_moving = (old_phase >= OBS_PHASE_TURN_OUT) &&
                                  (old_phase <= OBS_PHASE_VERIFY);

            if ((!b_moving) || permit_motion(p_seq, &out, &pose, now_msec))
            {
                run_phase(p_seq, &out, &pose, b_black, now_msec);
            }

            if (old_phase != p_seq->phase)
            {
                p_seq->guard_epoch = 0U;
                out.b_stop = true;
            }
        }

        bool const b_driving = (!out.b_stop) && (!out.b_fault) && (!out.b_done);
        p_seq->b_drive_left = b_driving &&
                             (out.b_line_follow || (out.left_rpm > 0U));
        p_seq->b_drive_right = b_driving &&
                              (out.b_line_follow || (out.right_rpm > 0U));
    }

    return out;
}

static void reassess (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                       obstacle_pose_t const * p_pose, uint32_t now_msec,
                       uint32_t reason)
{
    (void)p_pose;
    (void)now_msec;
    /* The controller immediately resumes line following on this report. */
    raise_fault(p_seq, p_out, (OBS_GUARD_REASON_SENSOR == reason)
                                ? OBS_FAULT_NO_ECHO : OBS_FAULT_NO_ROUTE);
}

static bool path_deviated (obstacle_seq_t const * p_seq,
                           obstacle_pose_t const * p_pose)
{
    bool const b_straight = (OBS_PHASE_DIAG_OUT == p_seq->phase) ||
                            (OBS_PHASE_PASS_SIDE == p_seq->phase) ||
                            (OBS_PHASE_PASS_END == p_seq->phase) ||
                            (OBS_PHASE_SEARCH_LINE == p_seq->phase) ||
                            (OBS_PHASE_VERIFY == p_seq->phase);
    int32_t target = 0;

    if (OBS_PHASE_DIAG_OUT == p_seq->phase)
    {
        target = (int32_t)p_seq->angle_ddeg;
    }
    else if (OBS_PHASE_SEARCH_LINE == p_seq->phase)
    {
        target = -(int32_t)p_seq->angle_ddeg;
    }

    int32_t const rel = p_seq->side *
                        (p_pose->heading_ddeg - p_seq->heading0_ddeg);

    return b_straight && (((rel - target) > OBS_PATH_MAX_ERROR_DDEG) ||
                          ((rel - target) < -OBS_PATH_MAX_ERROR_DDEG));
}

static bool permit_motion (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                            obstacle_pose_t const * p_pose, uint32_t now_msec)
{
    if (!obs_geom_is_valid())
    {
        raise_fault(p_seq, p_out, OBS_FAULT_NO_ROUTE);
        return false;
    }

    obs_guard_snapshot_t coverage = {0U, 0U};
    int32_t delta = p_pose->heading_ddeg - p_seq->guard_heading_ddeg;

    if (delta < 0)
    {
        delta = -delta;
    }

    if (!obs_state_guard_load(&coverage))
    {
        raise_fault(p_seq, p_out, OBS_FAULT_TIMEOUT);
        return false;
    }

    uint32_t const age =
        ((now_msec & 0xFFFFFFU) - (coverage.clock >> 8U)) & 0xFFFFFFU;
    uint32_t const valid = (coverage.ranges >> 27U) & OBS_GUARD_ALL_VIEWS;
    uint32_t const reason = coverage.ranges >> 30U;

    if ((0U != p_seq->guard_epoch) && (0U != reason))
    {
        reassess(p_seq, p_out, p_pose, now_msec, reason);
        return false;
    }

    if ((0U != p_seq->guard_epoch) &&
        (age > OBS_GUARD_TIMEOUT_MSEC) && (valid != OBS_GUARD_ALL_VIEWS))
    {
        reassess(p_seq, p_out, p_pose, now_msec, OBS_GUARD_REASON_STALE);
        return false;
    }

    if ((0U != p_seq->guard_epoch) && (valid != OBS_GUARD_ALL_VIEWS) &&
        (((p_pose->dist_mm - p_seq->guard_dist_mm) >= OBS_GUARD_TRAVEL_MM) ||
         (delta >= OBS_GUARD_TURN_DDEG)))
    {
        /* Coasting must not keep resetting an incomplete coverage epoch. */
        reassess(p_seq, p_out, p_pose, now_msec, OBS_GUARD_REASON_STALE);
        return false;
    }

    if ((0U == p_seq->guard_epoch) ||
        ((p_pose->dist_mm - p_seq->guard_dist_mm) >= OBS_GUARD_TRAVEL_MM) ||
        (delta >= OBS_GUARD_TURN_DDEG) || (age > OBS_GUARD_TIMEOUT_MSEC))
    {
        p_seq->guard_epoch = obs_state_guard_request(now_msec);
        p_seq->guard_dist_mm = p_pose->dist_mm;
        p_seq->guard_heading_ddeg = p_pose->heading_ddeg;
        p_out->b_stop = true;

        if (0U == p_seq->guard_epoch)
        {
            raise_fault(p_seq, p_out, OBS_FAULT_TIMEOUT);
        }

        return false;
    }

    if (((coverage.clock & 0xFFU) != p_seq->guard_epoch) ||
        (valid != OBS_GUARD_ALL_VIEWS))
    {
        p_out->b_stop = true;
        return false;
    }

    return true;
}

static void run_phase (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                       obstacle_pose_t const * p_pose, bool b_black,
                       uint32_t now_msec)
{
    UINT const     pattern = obs_state_flags_read();
    uint32_t const elapsed = now_msec - p_seq->enter_msec;
    int32_t const  seg     = p_pose->dist_mm - p_seq->seg_start_mm;
    int32_t const  rel =
        p_seq->side * (p_pose->heading_ddeg - p_seq->heading0_ddeg);
    uint32_t const scan_limit = ((OBS_PHASE_SCANNING == p_seq->phase) ||
                            (OBS_PHASE_REASSESS == p_seq->phase))
                               ? OBS_SCAN_TIMEOUT_MSEC + OBS_STOP_SETTLE_MSEC
                               : OBS_STATE_TIMEOUT_MSEC;
    uint32_t const limit = (OBS_PHASE_STOPPING == p_seq->phase) ?
                           OBS_STOP_TIMEOUT_MSEC : scan_limit;

    if (OBS_PHASE_FAULT == p_seq->phase)
    {
        p_out->b_stop  = true;
        p_out->b_fault = true;
    }
    else if ((0U != (pattern & OBS_EVT_ABORT)) || (elapsed > limit))
    {
        raise_fault(p_seq, p_out, OBS_FAULT_TIMEOUT);
    }
    else if ((OBS_PHASE_PASS_SIDE == p_seq->phase) &&
             (0U != (obs_state_status_faults(obs_state_status_get()) &
                     OBS_FAULT_NO_ECHO)))
    {
        raise_fault(p_seq, p_out, OBS_FAULT_NO_ECHO);
    }
    else
    {
        switch (p_seq->phase)
        {
            case OBS_PHASE_STOPPING:
                p_out->b_stop = true;

                if ((elapsed >= OBS_STOP_SETTLE_MSEC) &&
                    (p_seq->stop_hits >= OBS_STOP_CONFIRM_CYCLES))
                {
                    obs_state_flags_clear(OBS_EVT_SCAN_DONE);
                    obs_state_flags_set(OBS_EVT_SCAN_REQ);
                    enter_phase(p_seq, OBS_PHASE_SCANNING, now_msec,
                                p_pose->dist_mm);
                }
            break;

            case OBS_PHASE_SCANNING:
                p_out->b_stop = true;

                if (0U != (pattern & OBS_EVT_SCAN_DONE))
                {
                    handle_plan(p_seq, p_out, p_pose, now_msec);
                }
            break;

            case OBS_PHASE_TURN_OUT:
                set_arc(p_out, p_seq->side);

                if (rel >= (p_seq->angle_ddeg - OBS_TURN_LEAD_DDEG))
                {
                    enter_phase(p_seq, OBS_PHASE_DIAG_OUT, now_msec,
                                p_pose->dist_mm);
                }
            break;

            case OBS_PHASE_DIAG_OUT:
                set_straight(p_out, p_pose->heading_ddeg,
                             p_seq->heading0_ddeg +
                                 (p_seq->side * p_seq->angle_ddeg));

                if (seg >= p_seq->diag_mm)
                {
                    enter_phase(p_seq, OBS_PHASE_TURN_PARALLEL, now_msec,
                                p_pose->dist_mm);
                }
            break;

            case OBS_PHASE_TURN_PARALLEL:
                set_arc(p_out, -p_seq->side);

                if (rel <= OBS_TURN_LEAD_DDEG)
                {
                    /* After a left bypass, the object is on our right. */
                    obs_state_watch_side_set(-p_seq->side);
                    obs_state_flags_clear(OBS_EVT_SIDE_SEEN |
                                          OBS_EVT_SIDE_CLEAR);
                    obs_state_flags_set(OBS_EVT_SIDE_WATCH);
                    enter_phase(p_seq, OBS_PHASE_PASS_SIDE, now_msec,
                                p_pose->dist_mm);
                }
            break;

            case OBS_PHASE_PASS_SIDE:
                set_straight(p_out, p_pose->heading_ddeg, p_seq->heading0_ddeg);

                if ((0U != (pattern & OBS_EVT_SIDE_CLEAR)) &&
                    (0U != (pattern & OBS_EVT_SIDE_SEEN)))
                {
                    p_seq->rear_left_mm = p_pose->left_mm;
                    p_seq->rear_right_mm = p_pose->right_mm;
                    enter_phase(p_seq, OBS_PHASE_PASS_END, now_msec,
                                p_pose->dist_mm);
                }
                else if (seg >= OBS_PASS_MAX_MM)
                {
                    raise_fault(p_seq, p_out, OBS_FAULT_NO_ROUTE);
                }
                else
                {
                    /* Still alongside the object. */
                }
            break;

            case OBS_PHASE_PASS_END:
                set_straight(p_out, p_pose->heading_ddeg, p_seq->heading0_ddeg);

                {
                    int32_t const left = p_pose->left_mm - p_seq->rear_left_mm;
                    int32_t const right =
                        p_pose->right_mm - p_seq->rear_right_mm;
                    int32_t const progress = ((left < right) ? left : right);
                    obs_guard_snapshot_t coverage = {0U, 0U};
                    bool const b_loaded = obs_state_guard_load(&coverage);
                    int32_t const inner_mm =
                        (int32_t)((coverage.ranges >> 9U) & 0x1FFU) * 10;

                    if ((!b_loaded) || (rel > OBS_ALIGN_TOL_DDEG) ||
                        (rel < -OBS_ALIGN_TOL_DDEG))
                    {
                        raise_fault(p_seq, p_out, OBS_FAULT_NO_ROUTE);
                    }
                    else if ((0U == (pattern & OBS_EVT_SIDE_CLEAR)) ||
                             (inner_mm < OBS_SIDE_NEAR_MM))
                    {
                        enter_phase(p_seq, OBS_PHASE_PASS_SIDE, now_msec,
                                    p_pose->dist_mm);
                    }
                    else if (((progress * 95) / 100) >=
                             obs_geom_rear_clear_mm())
                    {
                        enter_phase(p_seq, OBS_PHASE_TURN_BACK, now_msec,
                                    p_pose->dist_mm);
                    }
                }
            break;

            case OBS_PHASE_REASSESS:
                p_out->b_stop = true;

                if ((!p_seq->b_reassess_scan) &&
                    (elapsed >= OBS_STOP_SETTLE_MSEC) &&
                    (p_seq->stop_hits >= OBS_STOP_CONFIRM_CYCLES))
                {
                    p_seq->rescan_count++;
                    p_seq->b_reassess_scan = true;
                    obs_state_flags_clear(OBS_EVT_SCAN_DONE);
                    obs_state_flags_set(OBS_EVT_SCAN_REQ);
                }
                else if (p_seq->b_reassess_scan &&
                         (0U != (pattern & OBS_EVT_SCAN_DONE)))
                {
                    scan_plan_bypass_t plan = {0U, 0U};
                    bool const b_loaded = obs_state_plan_load(&plan);
                    obs_state_flags_clear(OBS_EVT_SCAN_DONE);

                    if (b_loaded &&
                        (SCAN_PLAN_CLEAR == scan_plan_result(&plan)))
                    {
                        enter_phase(p_seq, p_seq->saved_phase, now_msec,
                                    p_pose->dist_mm);
                        p_seq->seg_start_mm = p_seq->saved_start_mm;
                        p_seq->line_hits = 0U;
                        p_seq->verify_cycles = 0U;
                    }
                    else
                    {
                        raise_fault(p_seq, p_out,
                            (OBS_GUARD_REASON_SENSOR == p_seq->guard_reason)
                                ? OBS_FAULT_NO_ECHO : OBS_FAULT_NO_ROUTE);
                    }
                }
                else
                {
                    /* Keep stopped while the bounded rescan completes. */
                }
            break;

            case OBS_PHASE_TURN_BACK:
                set_arc(p_out, -p_seq->side);

                if (rel <= (OBS_TURN_LEAD_DDEG - p_seq->angle_ddeg))
                {
                    p_seq->line_hits = 0U;
                    enter_phase(p_seq, OBS_PHASE_SEARCH_LINE, now_msec,
                                p_pose->dist_mm);
                }
            break;

            case OBS_PHASE_SEARCH_LINE:
                set_straight(p_out, p_pose->heading_ddeg,
                             p_seq->heading0_ddeg -
                                 (p_seq->side * p_seq->angle_ddeg));
                p_seq->line_hits = b_black ? (p_seq->line_hits + 1U) : 0U;

                if (p_seq->line_hits >= OBS_LINE_CONFIRM_CYCLES)
                {
                    p_seq->b_black_seen = true;
                    enter_phase(p_seq, OBS_PHASE_ALIGN, now_msec,
                                p_pose->dist_mm);
                }
                else if (seg >= (((p_seq->diag_mm * SEARCH_SCALE_NUM) /
                                  SEARCH_SCALE_DEN) +
                                 OBS_SEARCH_MARGIN_MM))
                {
                    raise_fault(p_seq, p_out, OBS_FAULT_LINE_LOST);
                }
                else
                {
                    /* Keep creeping towards the line. */
                }
            break;

            case OBS_PHASE_ALIGN:
                set_arc(p_out, p_seq->side);
                p_seq->b_black_seen = p_seq->b_black_seen || b_black;

                if (rel >= -OBS_ALIGN_TOL_DDEG)
                {
                    p_seq->verify_cycles = 0U;
                    enter_phase(p_seq, OBS_PHASE_VERIFY, now_msec,
                                p_pose->dist_mm);
                }
            break;

            case OBS_PHASE_VERIFY:
                p_out->b_line_follow = true;
                if (b_black)
                {
                    p_seq->verify_cycles++;
                }
                else
                {
                    p_seq->verify_cycles = 0U;
                }

                if (p_seq->verify_cycles >= OBS_LINE_CONFIRM_CYCLES)
                {
                    if (p_seq->b_black_seen)
                    {
                        finish_bypass(p_seq, p_out, true);
                    }
                    else
                    {
                        raise_fault(p_seq, p_out, OBS_FAULT_LINE_LOST);
                    }
                }
            break;

            default:
                raise_fault(p_seq, p_out, OBS_FAULT_TIMEOUT);
            break;
        }
    }
}

static bool counts_read (int32_t * p_left, int32_t * p_right)
{
    motion_status_t status = {0};
    bool const b_ok = (motion_get_status(&status) >= E_OK) && status.b_valid &&
        ((encoder_now_usec() - status.timestamp_usec) <= OBS_ODOM_MAX_AGE_USEC);

    if (b_ok)
    {
        *p_left  = status.left_count;
        *p_right = status.right_count;
    }

    return b_ok;
}

static int32_t counts_to_mm (int32_t counts)
{
    /* Counts since the bypass began stay small, so this cannot overflow. */
    int64_t const distance_x10 =
        (int64_t)counts * (int64_t)OBS_CIRC_MM_X10;
    int64_t const result =
        distance_x10 / (10 * (int64_t)OBS_COUNTS_PER_REV);

    if ((result > INT32_MAX) || (result < INT32_MIN))
    {
        return 0;
    }

    return (int32_t)result;
}

static int32_t counts_delta (int32_t current, int32_t base)
{
    uint32_t const delta = (uint32_t)current - (uint32_t)base;

    if (delta <= INT32_MAX)
    {
        return (int32_t)delta;
    }

    return -(int32_t)(UINT32_MAX - delta) - 1;
}

static void seq_begin (obstacle_seq_t * p_seq, int32_t left_count,
                       int32_t right_count, uint32_t now_msec)
{
    p_seq->b_active      = true;
    p_seq->base_left     = left_count;
    p_seq->base_right    = right_count;
    p_seq->heading0_ddeg = 0;
    p_seq->angle_ddeg    = 0;
    p_seq->diag_mm       = 0;
    p_seq->side          = 0;
    p_seq->line_hits     = 0U;
    p_seq->verify_cycles = 0U;
    p_seq->stop_hits     = 0U;
    p_seq->rescan_count  = 0U;
    p_seq->b_black_seen  = false;
    p_seq->guard_epoch = 0U;
    p_seq->mission_msec = now_msec;
    p_seq->previous_left = left_count;
    p_seq->previous_right = right_count;
    p_seq->previous_msec = now_msec;
    p_seq->b_drive_left = false;
    p_seq->b_drive_right = false;
    p_seq->stall_left_msec = 0U;
    p_seq->stall_right_msec = 0U;
    enter_phase(p_seq, OBS_PHASE_STOPPING, now_msec, 0);
}

static void enter_phase (obstacle_seq_t * p_seq, obs_state_phase_t phase,
                         uint32_t now_msec, int32_t dist_mm)
{
    p_seq->phase        = phase;
    p_seq->enter_msec   = now_msec;
    p_seq->seg_start_mm = dist_mm;
    obs_state_set_phase(phase);
    /* One diagnostic per transition; no per-ping or per-period logging. */
    tm_printf((UB *)"[OBS] phase=%u\n", (UINT)phase);
}

static void set_arc (obstacle_drive_t * p_out, int32_t turn)
{
    p_out->left_rpm  = (turn > 0) ? 0 : OBS_DRIVE_RPM;
    p_out->right_rpm = (turn > 0) ? OBS_DRIVE_RPM : 0;
}

static void set_straight (obstacle_drive_t * p_out, int32_t heading_ddeg,
                          int32_t target_ddeg)
{
    int32_t trim = (target_ddeg - heading_ddeg) / OBS_HEADING_TRIM_DIV;

    if (trim > OBS_HEADING_TRIM_MAX)
    {
        trim = OBS_HEADING_TRIM_MAX;
    }

    if (trim < -OBS_HEADING_TRIM_MAX)
    {
        trim = -OBS_HEADING_TRIM_MAX;
    }

    p_out->left_rpm  = OBS_DRIVE_RPM - trim;
    p_out->right_rpm = OBS_DRIVE_RPM + trim;
}

static void raise_fault (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                         uint32_t fault_bit)
{
    p_seq->phase = OBS_PHASE_FAULT;
    obs_state_faults_update(fault_bit, 0U);
    obs_state_set_phase(OBS_PHASE_FAULT);
    obs_state_flags_set(OBS_EVT_ABORT);
    p_out->b_stop  = true;
    p_out->b_fault = true;
    /* UINT matches %x on the 32-bit target; UB preserves string bytes. */
    tm_printf((UB *)"[OBS] FAULT 0x%x\n", (UINT)fault_bit);
}

static void finish_bypass (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                           bool b_count)
{
    obs_state_front_mark(false, 0U);
    obs_state_flags_clear(OBS_EVT_BLOCKED | OBS_EVT_SCAN_REQ |
                          OBS_EVT_SCAN_DONE | OBS_EVT_SIDE_WATCH |
                          OBS_EVT_SIDE_SEEN | OBS_EVT_SIDE_CLEAR);

    if (b_count)
    {
        obs_state_count_bypass();
    }

    obs_state_set_route(OBS_ROUTE_NONE);
    obs_state_set_phase(OBS_PHASE_IDLE);
    p_seq->b_active = false;
    p_out->b_done   = true;
    tm_printf((UB *)"[OBS] resume line following; recovered=%u\n",
              b_count ? 1U : 0U);
}

static void handle_plan (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                         obstacle_pose_t const * p_pose, uint32_t now_msec)
{
    scan_plan_bypass_t plan = {0U, 0U};

    if (!obs_state_plan_load(&plan))
    {
        raise_fault(p_seq, p_out, OBS_FAULT_TIMEOUT);
        return;
    }

    obs_state_flags_clear(OBS_EVT_SCAN_DONE);

    uint32_t const result = scan_plan_result(&plan);

    if (SCAN_PLAN_CLEAR == result)
    {
        finish_bypass(p_seq, p_out, false);
    }
    else if ((SCAN_PLAN_ROUTE != result) && (SCAN_PLAN_BEST_EFFORT != result))
    {
        raise_fault(p_seq, p_out, OBS_FAULT_NO_ROUTE);
    }
    else
    {
        bool const b_left = scan_plan_side_is_left(&plan);

        p_seq->side = b_left ? 1 : -1;
        obs_state_watch_side_set(-p_seq->side);
        p_seq->angle_ddeg =
            scan_plan_angle_deg(scan_plan_angle_index(&plan)) * DDEG_PER_DEG;
        p_seq->diag_mm       = scan_plan_diag_cm(&plan) * MM_PER_CM;
        p_seq->heading0_ddeg = p_pose->heading_ddeg;
        obs_state_set_route(b_left ? OBS_ROUTE_LEFT : OBS_ROUTE_RIGHT);
        obs_state_set_width_cm(scan_plan_width_cm(&plan));
        if (SCAN_PLAN_BEST_EFFORT == result)
        {
            tm_printf((UB *)"[OBS] BEST_EFFORT side=%s angle=%u\n",
                      b_left ? "LEFT" : "RIGHT",
                      (UINT)(p_seq->angle_ddeg / DDEG_PER_DEG));
        }
        enter_phase(p_seq, OBS_PHASE_TURN_OUT, now_msec, p_pose->dist_mm);
    }
}

/*** end of file ***/

