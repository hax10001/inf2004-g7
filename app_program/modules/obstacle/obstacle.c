/** @file obstacle.c
 * @brief Lifecycle, shared status, and the bypass sequencer.
 *
 * Heading and distance come from encoder odometry. Positive heading is a
 * left turn, matching motion.h. All turns are arcs about a stopped wheel
 * because the motor driver is forward-only.
 */
#include "obstacle.h"
#include "obs_config.h"
#include "scan_plan.h"
#include "servo.h"
#include "task_obstacle.h"
#include "ultrasonic.h"
#include "app/app_config.h"
#include "app/tasks.h"
#include "modules/motion/motion.h"
#include <tm/tmonitor.h>

#define OBS_MM_PER_COUNT \
    ((3.14159265F * BOARD_WHEEL_DIAMETER_MM) / (float)BOARD_ENC_COUNTS_PER_REV)
#define OBS_RAD_TO_DEG 57.2957795F

/* Status word layout (explicit masks and shifts). */
#define ST_STATE_SHIFT 0U
#define ST_STATE_MASK 0xFU
#define ST_ROUTE_SHIFT 4U
#define ST_ROUTE_MASK 0x3U
#define ST_FAULT_SHIFT 6U
#define ST_FAULT_MASK 0x1FU
#define ST_COUNT_SHIFT 11U
#define ST_COUNT_MASK 0xFU
#define ST_DIST_SHIFT 15U
#define ST_DIST_MASK 0x1FFU
#define ST_WIDTH_SHIFT 24U
#define ST_WIDTH_MASK 0xFFU

typedef struct
{
    bool             b_active;
    obstacle_state_t state;
    uint32_t         enter_msec;
    float            seg_start_mm;
    float            heading0_deg;
    float            angle_deg;
    float            diag_mm;
    float            fallback_mm;
    int8_t           side; /**< +1 bypass on the left, -1 on the right. */
    uint8_t          line_hits;
    uint8_t          verify_cycles;
    uint8_t          black_total;
} seq_t;

static ID                  gh_mutex = 0;
static ID                  gh_flag  = 0;
static uint32_t            g_status = 0U;
static bypass_plan_t       g_plan;
static volatile int8_t     g_watch_side = 1;
static seq_t               g_seq;

static void  status_update (uint32_t shift, uint32_t mask, uint32_t value);
static uint32_t field_get (uint32_t shift, uint32_t mask);
static bool  odometry_read (float * p_dist_mm, float * p_heading_deg);
static void  enter_state (seq_t * p_seq, obstacle_state_t state,
                          uint32_t now_msec, float dist_mm);
static void  set_arc (obstacle_drive_t * p_out, int8_t turn);
static void  set_straight (obstacle_drive_t * p_out, float heading,
                           float target);
static void  raise_fault (seq_t * p_seq, obstacle_drive_t * p_out,
                          uint8_t fault_bit);
static void  finish_bypass (seq_t * p_seq, obstacle_drive_t * p_out,
                            bool b_count);
static void  handle_plan (seq_t * p_seq, obstacle_drive_t * p_out, float hdg,
                          uint32_t now_msec, float dist_mm);

ER obstacle_init (void)
{
    if (0 == APP_OBSTACLE_ENABLED)
    {
        return E_OK;
    }

    T_CMTX mutex_config = {0};
    T_CFLG flag_config  = {0};

    mutex_config.mtxatr = TA_INHERIT;
    gh_mutex            = tk_cre_mtx(&mutex_config);

    if (gh_mutex <= E_OK)
    {
        return (E_OK == gh_mutex) ? E_OBJ : gh_mutex;
    }

    flag_config.flgatr = TA_TFIFO | TA_WMUL;
    gh_flag            = tk_cre_flg(&flag_config);

    if (gh_flag <= E_OK)
    {
        return (E_OK == gh_flag) ? E_OBJ : gh_flag;
    }

    g_status = obstacle_status_pack(OBSTACLE_S_IDLE, OBSTACLE_ROUTE_NONE, 0U,
                                    0U, OBS_DIST_INVALID_CM,
                                    OBS_WIDTH_INVALID_CM);

    ER error = ultrasonic_init();

    if (error < E_OK)
    {
        tm_printf((UB *)"[OBS] Ultrasonic init failed: %d\n", error);
        return error;
    }

    error = servo_init();

    if (error < E_OK)
    {
        tm_printf((UB *)"[OBS] Servo init failed: %d\n", error);
        return error;
    }

    tm_printf((UB *)"[OBS] Init: TRIG GP%d ECHO GP%d SERVO GP%d\n",
              (INT)BOARD_US_TRIG_PIN, (INT)BOARD_US_ECHO_PIN,
              (INT)BOARD_SERVO_PIN);
    return E_OK;
}

ER obstacle_start (void)
{
    if (0 == APP_OBSTACLE_ENABLED)
    {
        return E_OK;
    }

    ID const h_task = tasks_create(task_obstacle_task, APP_OBSTACLE_PRIORITY);

    if (h_task <= E_OK)
    {
        tm_printf((UB *)"[OBS] Task creation failed: %d\n", h_task);
        return (E_OK == h_task) ? E_OBJ : h_task;
    }

    return tk_sta_tsk(h_task, 0);
}

bool obstacle_is_blocked (void)
{
    if (0 == APP_OBSTACLE_ENABLED)
    {
        return false;
    }

    return 0U != (obstacle_link_flags_read() & OBS_EVT_BLOCKED);
}

bool obstacle_has_fault (void)
{
    if ((0 == APP_OBSTACLE_ENABLED) || (0 == OBS_FAILSAFE_NO_ECHO))
    {
        return false;
    }

    return 0U != (field_get(ST_FAULT_SHIFT, ST_FAULT_MASK) & OBS_FAULT_NO_ECHO);
}

void obstacle_abort (void)
{
    if (0 == APP_OBSTACLE_ENABLED)
    {
        return;
    }

    obstacle_link_flags_set(OBS_EVT_ABORT);
    g_seq.b_active = false;
    status_update(ST_STATE_SHIFT, ST_STATE_MASK, (uint32_t)OBSTACLE_S_IDLE);
    status_update(ST_ROUTE_SHIFT, ST_ROUTE_MASK, (uint32_t)OBSTACLE_ROUTE_NONE);
}

obstacle_drive_t obstacle_bypass_step (uint32_t line_left, uint32_t line_right,
                                       uint32_t now_msec)
{
    obstacle_drive_t out = {0.0F, 0.0F, false, false, false, false};
    seq_t * const    p   = &g_seq;
    float            dist = 0.0F;
    float            hdg  = 0.0F;

    if (0 == APP_OBSTACLE_ENABLED)
    {
        out.b_done = true;
        return out;
    }

    if (!odometry_read(&dist, &hdg))
    {
        out.b_stop = true;
        return out;
    }

    if (!p->b_active)
    {
        (void)memset(p, 0, sizeof(*p));
        p->b_active     = true;
        p->heading0_deg = hdg;
        enter_state(p, OBSTACLE_S_STOPPING, now_msec, dist);
    }

    if (OBSTACLE_S_FAULT == p->state)
    {
        out.b_stop  = true;
        out.b_fault = true;
        return out;
    }

    UINT const     ptn     = obstacle_link_flags_read();
    uint32_t const elapsed = now_msec - p->enter_msec;
    float const    seg     = dist - p->seg_start_mm;
    float const    rel     = (float)p->side * (hdg - p->heading0_deg);
    bool const     b_black = (0U != line_left) || (0U != line_right);
    uint32_t const limit   = (OBSTACLE_S_SCANNING == p->state)
                                 ? OBS_SCAN_TIMEOUT_MS
                                 : OBS_STATE_TIMEOUT_MS;

    if (0U != (ptn & OBS_EVT_ABORT))
    {
        raise_fault(p, &out, OBS_FAULT_TIMEOUT);
        return out;
    }

    if (elapsed > limit)
    {
        raise_fault(p, &out, OBS_FAULT_TIMEOUT);
        return out;
    }

    switch (p->state)
    {
        case OBSTACLE_S_STOPPING:
            out.b_stop = true;

            if (elapsed >= OBS_STOP_SETTLE_MS)
            {
                obstacle_link_flags_clear(OBS_EVT_SCAN_DONE);
                obstacle_link_flags_set(OBS_EVT_SCAN_REQ);
                enter_state(p, OBSTACLE_S_SCANNING, now_msec, dist);
            }
        break;

        case OBSTACLE_S_SCANNING:
            out.b_stop = true;

            if (0U != (ptn & OBS_EVT_SCAN_DONE))
            {
                handle_plan(p, &out, hdg, now_msec, dist);
            }
        break;

        case OBSTACLE_S_TURN_OUT:
            set_arc(&out, p->side);

            if (rel >= (p->angle_deg - OBS_TURN_LEAD_DEG))
            {
                enter_state(p, OBSTACLE_S_DIAG_OUT, now_msec, dist);
            }
        break;

        case OBSTACLE_S_DIAG_OUT:
            set_straight(&out, hdg,
                         p->heading0_deg + ((float)p->side * p->angle_deg));

            if (seg >= p->diag_mm)
            {
                enter_state(p, OBSTACLE_S_TURN_PARALLEL, now_msec, dist);
            }
        break;

        case OBSTACLE_S_TURN_PARALLEL:
            set_arc(&out, (int8_t)(-p->side));

            if (rel <= OBS_TURN_LEAD_DEG)
            {
                g_watch_side = p->side;
                obstacle_link_flags_clear(OBS_EVT_SIDE_SEEN |
                                          OBS_EVT_SIDE_CLEAR);
                obstacle_link_flags_set(OBS_EVT_SIDE_WATCH);
                enter_state(p, OBSTACLE_S_PASS_SIDE, now_msec, dist);
            }
        break;

        case OBSTACLE_S_PASS_SIDE:
            set_straight(&out, hdg, p->heading0_deg);

            if ((0U != (ptn & OBS_EVT_SIDE_CLEAR)) ||
                ((0U == (ptn & OBS_EVT_SIDE_SEEN)) &&
                 (seg >= p->fallback_mm)))
            {
                obstacle_link_flags_clear(OBS_EVT_SIDE_WATCH |
                                          OBS_EVT_SIDE_SEEN |
                                          OBS_EVT_SIDE_CLEAR);
                enter_state(p, OBSTACLE_S_PASS_END, now_msec, dist);
            }
            else if (seg >= OBS_PASS_MAX_MM)
            {
                raise_fault(p, &out, OBS_FAULT_NO_ROUTE);
            }
            else
            {
                /* Still alongside the object. */
            }
        break;

        case OBSTACLE_S_PASS_END:
            set_straight(&out, hdg, p->heading0_deg);

            if (seg >= (OBS_ROBOT_LENGTH_MM + OBS_PASS_MARGIN_MM))
            {
                enter_state(p, OBSTACLE_S_TURN_BACK, now_msec, dist);
            }
        break;

        case OBSTACLE_S_TURN_BACK:
            set_arc(&out, (int8_t)(-p->side));

            if (rel <= (-p->angle_deg + OBS_TURN_LEAD_DEG))
            {
                p->line_hits = 0U;
                enter_state(p, OBSTACLE_S_SEARCH_LINE, now_msec, dist);
            }
        break;

        case OBSTACLE_S_SEARCH_LINE:
            set_straight(&out, hdg,
                         p->heading0_deg - ((float)p->side * p->angle_deg));
            p->line_hits = b_black ? (uint8_t)(p->line_hits + 1U) : 0U;

            if (p->line_hits >= OBS_LINE_CONFIRM_CYCLES)
            {
                p->black_total = 1U;
                enter_state(p, OBSTACLE_S_ALIGN, now_msec, dist);
            }
            else if (seg >= ((1.5F * p->diag_mm) + OBS_SEARCH_MARGIN_MM))
            {
                raise_fault(p, &out, OBS_FAULT_LINE_LOST);
            }
            else
            {
                /* Keep creeping towards the line. */
            }
        break;

        case OBSTACLE_S_ALIGN:
            set_arc(&out, p->side);

            if (b_black)
            {
                p->black_total = 1U;
            }

            if (rel >= -OBS_ALIGN_TOL_DEG)
            {
                p->verify_cycles = 0U;
                enter_state(p, OBSTACLE_S_VERIFY, now_msec, dist);
            }
        break;

        case OBSTACLE_S_VERIFY:
            out.b_line_follow = true;
            p->verify_cycles++;

            if (b_black)
            {
                p->black_total = 1U;
            }

            if (p->verify_cycles >= OBS_VERIFY_CYCLES)
            {
                if (0U != p->black_total)
                {
                    finish_bypass(p, &out, true);
                }
                else
                {
                    raise_fault(p, &out, OBS_FAULT_LINE_LOST);
                }
            }
        break;

        default:
            raise_fault(p, &out, OBS_FAULT_TIMEOUT);
        break;
    }

    return out;
}

uint32_t obstacle_get_status (void)
{
    uint32_t word = 0U;

    if ((gh_mutex > E_OK) && (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        word = g_status;
        (void)tk_unl_mtx(gh_mutex);
    }

    return word;
}

uint32_t obstacle_status_pack (obstacle_state_t state, obstacle_route_t route,
                               uint8_t faults, uint8_t count,
                               uint16_t dist_cm, uint8_t width_cm)
{
    return (((uint32_t)state & ST_STATE_MASK) << ST_STATE_SHIFT) |
           (((uint32_t)route & ST_ROUTE_MASK) << ST_ROUTE_SHIFT) |
           (((uint32_t)faults & ST_FAULT_MASK) << ST_FAULT_SHIFT) |
           (((uint32_t)count & ST_COUNT_MASK) << ST_COUNT_SHIFT) |
           (((uint32_t)dist_cm & ST_DIST_MASK) << ST_DIST_SHIFT) |
           (((uint32_t)width_cm & ST_WIDTH_MASK) << ST_WIDTH_SHIFT);
}

obstacle_state_t obstacle_status_state (uint32_t word)
{
    return (obstacle_state_t)((word >> ST_STATE_SHIFT) & ST_STATE_MASK);
}

obstacle_route_t obstacle_status_route (uint32_t word)
{
    return (obstacle_route_t)((word >> ST_ROUTE_SHIFT) & ST_ROUTE_MASK);
}

uint8_t obstacle_status_faults (uint32_t word)
{
    return (uint8_t)((word >> ST_FAULT_SHIFT) & ST_FAULT_MASK);
}

uint8_t obstacle_status_count (uint32_t word)
{
    return (uint8_t)((word >> ST_COUNT_SHIFT) & ST_COUNT_MASK);
}

uint16_t obstacle_status_distance_cm (uint32_t word)
{
    return (uint16_t)((word >> ST_DIST_SHIFT) & ST_DIST_MASK);
}

uint8_t obstacle_status_width_cm (uint32_t word)
{
    return (uint8_t)((word >> ST_WIDTH_SHIFT) & ST_WIDTH_MASK);
}

UINT obstacle_link_flags_read (void)
{
    T_RFLG info = {0};

    if (tk_ref_flg(gh_flag, &info) < E_OK)
    {
        return 0U;
    }

    return info.flgptn;
}

void obstacle_link_flags_set (UINT bits)
{
    (void)tk_set_flg(gh_flag, bits);
}

void obstacle_link_flags_clear (UINT bits)
{
    (void)tk_clr_flg(gh_flag, ~bits);
}

void obstacle_link_publish_distance (uint16_t mm)
{
    uint32_t cm = (uint32_t)(mm / 10U);

    cm = (cm > 510U) ? OBS_DIST_INVALID_CM : cm;
    status_update(ST_DIST_SHIFT, ST_DIST_MASK, cm);
}

void obstacle_link_faults (uint8_t set_mask, uint8_t clear_mask)
{
    if ((gh_mutex <= E_OK) || (tk_loc_mtx(gh_mutex, TMO_FEVR) < E_OK))
    {
        return;
    }

    uint32_t faults = (g_status >> ST_FAULT_SHIFT) & ST_FAULT_MASK;

    faults   = (faults | set_mask) & ~(uint32_t)clear_mask;
    g_status = (g_status & ~(ST_FAULT_MASK << ST_FAULT_SHIFT)) |
               ((faults & ST_FAULT_MASK) << ST_FAULT_SHIFT);
    (void)tk_unl_mtx(gh_mutex);
}

void obstacle_link_confirm (void)
{
    status_update(ST_STATE_SHIFT, ST_STATE_MASK, (uint32_t)OBSTACLE_S_CONFIRM);
    obstacle_link_flags_set(OBS_EVT_BLOCKED);
}

void obstacle_link_store_plan (bypass_plan_t const * p_plan)
{
    g_plan = *p_plan; /* Read only after OBS_EVT_SCAN_DONE is set. */
}

int8_t obstacle_link_watch_side (void)
{
    return g_watch_side;
}

static void status_update (uint32_t shift, uint32_t mask, uint32_t value)
{
    if ((gh_mutex <= E_OK) || (tk_loc_mtx(gh_mutex, TMO_FEVR) < E_OK))
    {
        return;
    }

    g_status = (g_status & ~(mask << shift)) | ((value & mask) << shift);
    (void)tk_unl_mtx(gh_mutex);
}

static uint32_t field_get (uint32_t shift, uint32_t mask)
{
    return (obstacle_get_status() >> shift) & mask;
}

static bool odometry_read (float * p_dist_mm, float * p_heading_deg)
{
    motion_status_t status = {0};

    if ((motion_get_status(&status) < E_OK) || (!status.b_valid))
    {
        return false;
    }

    float const left  = (float)status.left_count * OBS_MM_PER_COUNT;
    float const right = (float)status.right_count * OBS_MM_PER_COUNT;

    *p_dist_mm     = 0.5F * (left + right);
    *p_heading_deg = ((right - left) / OBS_TRACK_WIDTH_MM) * OBS_RAD_TO_DEG;
    return true;
}

static void enter_state (seq_t * p_seq, obstacle_state_t state,
                         uint32_t now_msec, float dist_mm)
{
    p_seq->state        = state;
    p_seq->enter_msec   = now_msec;
    p_seq->seg_start_mm = dist_mm;
    status_update(ST_STATE_SHIFT, ST_STATE_MASK, (uint32_t)state);
}

static void set_arc (obstacle_drive_t * p_out, int8_t turn)
{
    p_out->left_rpm  = (turn > 0) ? 0.0F : OBS_DRIVE_RPM;
    p_out->right_rpm = (turn > 0) ? OBS_DRIVE_RPM : 0.0F;
}

static void set_straight (obstacle_drive_t * p_out, float heading, float target)
{
    float trim = OBS_HEADING_KP * (target - heading);

    trim = (trim > OBS_HEADING_TRIM_MAX) ? OBS_HEADING_TRIM_MAX : trim;
    trim = (trim < -OBS_HEADING_TRIM_MAX) ? -OBS_HEADING_TRIM_MAX : trim;
    p_out->left_rpm  = OBS_DRIVE_RPM - trim;
    p_out->right_rpm = OBS_DRIVE_RPM + trim;
}

static void raise_fault (seq_t * p_seq, obstacle_drive_t * p_out,
                         uint8_t fault_bit)
{
    p_seq->state = OBSTACLE_S_FAULT;
    obstacle_link_faults(fault_bit, 0U);
    status_update(ST_STATE_SHIFT, ST_STATE_MASK, (uint32_t)OBSTACLE_S_FAULT);
    obstacle_link_flags_set(OBS_EVT_ABORT);
    p_out->b_stop  = true;
    p_out->b_fault = true;
    tm_printf((UB *)"[OBS] FAULT 0x%x\n", (UINT)fault_bit);
}

static void finish_bypass (seq_t * p_seq, obstacle_drive_t * p_out,
                           bool b_count)
{
    obstacle_link_flags_clear(OBS_EVT_BLOCKED | OBS_EVT_SCAN_REQ |
                              OBS_EVT_SCAN_DONE | OBS_EVT_SIDE_WATCH |
                              OBS_EVT_SIDE_SEEN | OBS_EVT_SIDE_CLEAR);

    if (b_count)
    {
        uint32_t const count = field_get(ST_COUNT_SHIFT, ST_COUNT_MASK);

        status_update(ST_COUNT_SHIFT, ST_COUNT_MASK,
                      (count < ST_COUNT_MASK) ? (count + 1U) : count);
    }

    status_update(ST_ROUTE_SHIFT, ST_ROUTE_MASK, (uint32_t)OBSTACLE_ROUTE_NONE);
    status_update(ST_STATE_SHIFT, ST_STATE_MASK, (uint32_t)OBSTACLE_S_IDLE);
    p_seq->b_active = false;
    p_out->b_done   = true;
}

static void handle_plan (seq_t * p_seq, obstacle_drive_t * p_out, float hdg,
                         uint32_t now_msec, float dist_mm)
{
    bypass_plan_t const plan = g_plan;

    obstacle_link_flags_clear(OBS_EVT_SCAN_DONE);

    if ((uint32_t)PLAN_CLEAR == plan.result)
    {
        finish_bypass(p_seq, p_out, false);
    }
    else if ((uint32_t)PLAN_ROUTE != plan.result)
    {
        raise_fault(p_seq, p_out, OBS_FAULT_NO_ROUTE);
    }
    else
    {
        p_seq->side         = (0U != plan.side_left) ? 1 : -1;
        p_seq->angle_deg    = scan_plan_angle_deg(plan.angle_idx);
        p_seq->diag_mm      = (float)plan.diag_cm * 10.0F;
        p_seq->fallback_mm  = (float)plan.fallback_cm * 10.0F;
        p_seq->heading0_deg = hdg;
        status_update(ST_ROUTE_SHIFT, ST_ROUTE_MASK,
                      (0U != plan.side_left) ? (uint32_t)OBSTACLE_ROUTE_LEFT
                                             : (uint32_t)OBSTACLE_ROUTE_RIGHT);
        status_update(ST_WIDTH_SHIFT, ST_WIDTH_MASK, plan.width_cm);
        enter_state(p_seq, OBSTACLE_S_TURN_OUT, now_msec, dist_mm);
    }
}

/*** end of file ***/