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
#include "scan_plan.h"
#include "servo.h"
#include "task_obstacle.h"
#include "ultrasonic.h"
#include "app/app_config.h"
#include "app/tasks.h"
#include "modules/motion/motion.h"
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
} obstacle_pose_t;

typedef struct
{
    bool              b_active;
    obs_state_phase_t phase;
    uint32_t          enter_msec;
    int32_t           base_left;
    int32_t           base_right;
    int32_t           seg_start_mm;
    int32_t           heading0_ddeg;
    int32_t           angle_ddeg;
    int32_t           diag_mm;
    int32_t           fallback_mm;
    int32_t           side; /**< 1 bypasses on the left, -1 on the right. */
    uint32_t          line_hits;
    uint32_t          verify_cycles;
    bool              b_black_seen;
} obstacle_seq_t;

static obstacle_seq_t g_seq;

/** @brief Read the signed wheel counts; false until motion has data. */
static bool counts_read (int32_t * p_left, int32_t * p_right);
/** @brief Convert a wheel count difference to millimetres. */
static int32_t counts_to_mm (int32_t counts);
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

    return error;
}

ER obstacle_start (void)
{
    ER error = E_OK;

    if (0 != APP_OBSTACLE_ENABLED)
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

    return error;
}

bool obstacle_is_blocked (void)
{
    bool b_blocked = false;

    if (0 != APP_OBSTACLE_ENABLED)
    {
        b_blocked = (0U != (obs_state_flags_read() & OBS_EVT_BLOCKED));
    }

    return b_blocked;
}

bool obstacle_has_fault (void)
{
    bool b_fault = false;

    if ((0 != APP_OBSTACLE_ENABLED) && (0 != OBS_FAILSAFE_NO_ECHO))
    {
        uint32_t const faults = obs_state_status_faults(obs_state_status_get());

        b_fault = (0U != (faults & OBS_FAULT_NO_ECHO));
    }

    return b_fault;
}

void obstacle_abort (void)
{
    if (0 != APP_OBSTACLE_ENABLED)
    {
        obs_state_flags_set(OBS_EVT_ABORT);
        g_seq.b_active = false;
        obs_state_set_phase(OBS_PHASE_IDLE);
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
        out.b_stop = true;
    }
    else
    {
        obstacle_seq_t * const p_seq = &g_seq;

        if (!p_seq->b_active)
        {
            seq_begin(p_seq, left_count, right_count, now_msec);
        }

        int32_t const left_mm  = counts_to_mm(left_count - p_seq->base_left);
        int32_t const right_mm = counts_to_mm(right_count - p_seq->base_right);
        obstacle_pose_t const pose = {(left_mm + right_mm) / 2,
                                      ((right_mm - left_mm) * OBS_RAD_TO_DDEG) /
                                          OBS_TRACK_WIDTH_MM};
        bool const            b_black = (0U != line_left) || (0U != line_right);

        run_phase(p_seq, &out, &pose, b_black, now_msec);
    }

    return out;
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
    uint32_t const limit = (OBS_PHASE_SCANNING == p_seq->phase)
                               ? OBS_SCAN_TIMEOUT_MSEC
                               : OBS_STATE_TIMEOUT_MSEC;

    if (OBS_PHASE_FAULT == p_seq->phase)
    {
        p_out->b_stop  = true;
        p_out->b_fault = true;
    }
    else if ((0U != (pattern & OBS_EVT_ABORT)) || (elapsed > limit))
    {
        raise_fault(p_seq, p_out, OBS_FAULT_TIMEOUT);
    }
    else
    {
        switch (p_seq->phase)
        {
            case OBS_PHASE_STOPPING:
                p_out->b_stop = true;

                if (elapsed >= OBS_STOP_SETTLE_MSEC)
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
                    obs_state_watch_side_set(p_seq->side);
                    obs_state_flags_clear(OBS_EVT_SIDE_SEEN |
                                          OBS_EVT_SIDE_CLEAR);
                    obs_state_flags_set(OBS_EVT_SIDE_WATCH);
                    enter_phase(p_seq, OBS_PHASE_PASS_SIDE, now_msec,
                                p_pose->dist_mm);
                }
            break;

            case OBS_PHASE_PASS_SIDE:
                set_straight(p_out, p_pose->heading_ddeg, p_seq->heading0_ddeg);

                if ((0U != (pattern & OBS_EVT_SIDE_CLEAR)) ||
                    ((0U == (pattern & OBS_EVT_SIDE_SEEN)) &&
                     (seg >= p_seq->fallback_mm)))
                {
                    obs_state_flags_clear(OBS_EVT_SIDE_WATCH |
                                          OBS_EVT_SIDE_SEEN |
                                          OBS_EVT_SIDE_CLEAR);
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

                if (seg >= (OBS_ROBOT_LENGTH_MM + OBS_PASS_MARGIN_MM))
                {
                    enter_phase(p_seq, OBS_PHASE_TURN_BACK, now_msec,
                                p_pose->dist_mm);
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

                if ((rel >= -OBS_ALIGN_TOL_DDEG) ||
                    (elapsed >= OBS_ALIGN_MAX_MSEC))
                {
                    p_seq->verify_cycles = 0U;
                    enter_phase(p_seq, OBS_PHASE_VERIFY, now_msec,
                                p_pose->dist_mm);
                }
            break;

            case OBS_PHASE_VERIFY:
                p_out->b_line_follow = true;
                p_seq->verify_cycles++;
                p_seq->b_black_seen = p_seq->b_black_seen || b_black;

                if (p_seq->verify_cycles >= OBS_VERIFY_CYCLES)
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
    bool const b_ok = (motion_get_status(&status) >= E_OK) && status.b_valid;

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
    return (counts * OBS_CIRC_MM_X10) / (10 * OBS_COUNTS_PER_REV);
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
    p_seq->fallback_mm   = 0;
    p_seq->side          = 0;
    p_seq->line_hits     = 0U;
    p_seq->verify_cycles = 0U;
    p_seq->b_black_seen  = false;
    enter_phase(p_seq, OBS_PHASE_STOPPING, now_msec, 0);
}

static void enter_phase (obstacle_seq_t * p_seq, obs_state_phase_t phase,
                         uint32_t now_msec, int32_t dist_mm)
{
    p_seq->phase        = phase;
    p_seq->enter_msec   = now_msec;
    p_seq->seg_start_mm = dist_mm;
    obs_state_set_phase(phase);
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
}

static void handle_plan (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                         obstacle_pose_t const * p_pose, uint32_t now_msec)
{
    scan_plan_bypass_t plan = {0U, 0U};

    obs_state_plan_load(&plan);
    obs_state_flags_clear(OBS_EVT_SCAN_DONE);

    uint32_t const result = scan_plan_result(&plan);

    if (SCAN_PLAN_CLEAR == result)
    {
        finish_bypass(p_seq, p_out, false);
    }
    else if (SCAN_PLAN_ROUTE != result)
    {
        raise_fault(p_seq, p_out, OBS_FAULT_NO_ROUTE);
    }
    else
    {
        bool const b_left = scan_plan_side_is_left(&plan);

        p_seq->side = b_left ? 1 : -1;
        p_seq->angle_ddeg =
            scan_plan_angle_deg(scan_plan_angle_index(&plan)) * DDEG_PER_DEG;
        p_seq->diag_mm       = scan_plan_diag_cm(&plan) * MM_PER_CM;
        p_seq->fallback_mm   = scan_plan_fallback_cm(&plan) * MM_PER_CM;
        p_seq->heading0_ddeg = p_pose->heading_ddeg;
        obs_state_set_route(b_left ? OBS_ROUTE_LEFT : OBS_ROUTE_RIGHT);
        obs_state_set_width_cm(scan_plan_width_cm(&plan));
        enter_phase(p_seq, OBS_PHASE_TURN_OUT, now_msec, p_pose->dist_mm);
    }
}

/*** end of file ***/

