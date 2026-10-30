/** @file obstacle.c
 * @brief Preemptible bypass with line handoff and bounded replanning.
 */
#include "obstacle.h"
#include "obs_config.h"
#include "obs_guard.h"
#include "task_obstacle.h"
#include "servo.h"
#include "ultrasonic.h"
#include "app/app_config.h"
#include "app/tasks.h"
#include "modules/motion/motion.h"
#include <tm/tmonitor.h>

#define OBS_CIRC_X10 ((int32_t)(BOARD_WHEEL_DIAMETER_MM * 31.4159265F))
#define OBS_CPR ((int32_t)BOARD_ENC_COUNTS_PER_REV)
#define OBS_RAD_DDEG 573

typedef struct
{
    bool active;
    bool armed;
    obs_state_phase_t phase;
    uint32_t entered;
    uint32_t hits;
    uint32_t replans;
    int32_t left0;
    int32_t right0;
    int32_t segment;
    int32_t heading0;
    int32_t angle;
    int32_t diagonal;
    int32_t side;
} bypass_t;

static bypass_t g_seq;

/** @brief Enter a phase without blocking the controller. */
static void enter_phase (obs_state_phase_t phase, uint32_t now,
                         int32_t distance);
/** @brief Return a zero-speed command. */
static obstacle_drive_t stopped (void);
/** @brief Stop and latch a fault. */
static obstacle_drive_t fail (uint32_t fault);
/** @brief Cancel the bypass and return control to normal navigation. */
static obstacle_drive_t finish (bool count);
/** @brief Convert relative encoder counts without a 32-bit product. */
static int32_t wheel_mm (int32_t counts);
/** @brief Load a newly completed scan. */
static obstacle_drive_t take_plan (uint32_t now, int32_t distance,
                                   int32_t heading);
/** @brief Evaluate one motion phase. */
static obstacle_drive_t move_step (uint32_t now, int32_t distance,
                                   int32_t heading, UINT flags);

ER obstacle_init (void)
{
    ER error = E_OK;

    if (APP_OBSTACLE_ENABLED)
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
    }
    return error;
}

ER obstacle_start (void)
{
    if (!APP_OBSTACLE_ENABLED)
    {
        return E_OK;
    }
    ID const task = tasks_create(task_obstacle_task, APP_OBSTACLE_PRIORITY);

    if (task <= E_OK)
    {
        return (task == E_OK) ? E_OBJ : task;
    }
    return tk_sta_tsk(task, 0);
}

bool obstacle_is_blocked (void)
{
    return APP_OBSTACLE_ENABLED &&
           ((obs_state_flags_read() & OBS_EVT_BLOCKED) != 0U);
}

bool obstacle_has_fault (void)
{
    if (!APP_OBSTACLE_ENABLED || !OBS_FAILSAFE_NO_ECHO)
    {
        return false;
    }
    return (obs_state_status_faults(obs_state_status_get()) &
            OBS_FAULT_NO_ECHO) != 0U;
}

void obstacle_abort (void)
{
    if (APP_OBSTACLE_ENABLED)
    {
        obs_state_flags_clear(OBS_EVT_SCAN_REQ | OBS_EVT_SIDE_WATCH |
                              OBS_EVT_FRONT_READY);
        obs_state_flags_set(OBS_EVT_ABORT);
        g_seq.active = false;
        obs_state_set_phase(OBS_PHASE_IDLE);
        obs_state_set_route(OBS_ROUTE_NONE);
    }
}

obstacle_drive_t obstacle_bypass_step (uint32_t left, uint32_t right,
                                       uint32_t now)
{
    obstacle_drive_t out = stopped();
    motion_status_t status = {0};

    if (!APP_OBSTACLE_ENABLED)
    {
        out.b_done = true;
        return out;
    }
    if (!g_seq.active)
    {
        g_seq = (bypass_t){0};
        g_seq.active = true;
        if (motion_get_status(&status) < E_OK || !status.b_valid)
        {
            return fail(OBS_FAULT_TIMEOUT);
        }
        g_seq.left0 = status.left_count;
        g_seq.right0 = status.right_count;
        enter_phase(OBS_PHASE_STOPPING, now, 0);
    }

    UINT const flags = obs_state_flags_read();
    bool const black = (left != 0U) || (right != 0U);
    bool const moving = (g_seq.phase >= OBS_PHASE_TURN_OUT) &&
                        (g_seq.phase <= OBS_PHASE_VERIFY);

    if (g_seq.phase == OBS_PHASE_FAULT)
    {
        out.b_fault = true;
        return out;
    }
    if ((flags & OBS_EVT_ABORT) != 0U)
    {
        return fail(OBS_FAULT_TIMEOUT);
    }

    /* A new forward hazard has precedence over a line encounter. */
    if (moving && ((flags & OBS_EVT_FRONT_BLOCKED) == 0U))
    {
        if (!black)
        {
            if (g_seq.phase >= OBS_PHASE_TURN_PARALLEL)
            {
                g_seq.armed = true;
            }
            g_seq.hits = 0U;
        }
        else if (g_seq.armed)
        {
            g_seq.hits++;
            if (g_seq.hits >= OBS_LINE_CONFIRM_CYCLES)
            {
                return finish(true);
            }
            return out;
        }
    }

    if (motion_get_status(&status) < E_OK || !status.b_valid)
    {
        return fail(OBS_FAULT_TIMEOUT);
    }
    int32_t const lm = wheel_mm(status.left_count - g_seq.left0);
    int32_t const rm = wheel_mm(status.right_count - g_seq.right0);
    int32_t const distance = (lm + rm) / 2;
    int32_t const heading = ((rm - lm) * OBS_RAD_DDEG) /
                            OBS_TRACK_WIDTH_MM;

    if (moving && ((flags & OBS_EVT_FRONT_BLOCKED) != 0U))
    {
        if (g_seq.replans >= OBS_REPLAN_LIMIT)
        {
            return fail(OBS_FAULT_NO_ROUTE);
        }
        g_seq.replans++;
        g_seq.hits = 0U;
        obs_state_flags_clear(OBS_EVT_SIDE_WATCH | OBS_EVT_SIDE_SEEN |
                              OBS_EVT_SIDE_CLEAR | OBS_EVT_FRONT_READY);
        enter_phase(OBS_PHASE_STOPPING, now, distance);
        tm_printf((UB *)"[OBS] New hazard: stop and replan\n");
        return out;
    }

    uint32_t const elapsed = now - g_seq.entered;
    uint32_t limit = OBS_STATE_TIMEOUT_MSEC;

    if (g_seq.phase == OBS_PHASE_SCANNING)
    {
        limit = OBS_SCAN_TIMEOUT_MSEC;
    }
    else if ((g_seq.phase == OBS_PHASE_TURN_OUT) ||
             (g_seq.phase == OBS_PHASE_TURN_PARALLEL) ||
             (g_seq.phase == OBS_PHASE_TURN_BACK))
    {
        limit = OBS_TURN_LIMIT_MSEC;
    }
    if (elapsed > limit)
    {
        return fail(OBS_FAULT_TIMEOUT);
    }
    if (g_seq.phase == OBS_PHASE_STOPPING)
    {
        if (elapsed >= OBS_STOP_SETTLE_MSEC)
        {
            obs_state_flags_clear(OBS_EVT_SCAN_DONE |
                                  OBS_EVT_FRONT_BLOCKED |
                                  OBS_EVT_FRONT_READY);
            enter_phase(OBS_PHASE_SCANNING, now, distance);
            obs_state_flags_set(OBS_EVT_SCAN_REQ);
        }
        return out;
    }
    if (g_seq.phase == OBS_PHASE_SCANNING)
    {
        if ((flags & OBS_EVT_SCAN_DONE) != 0U)
        {
            return take_plan(now, distance, heading);
        }
        return out;
    }
    if ((flags & OBS_EVT_FRONT_READY) == 0U)
    {
        /* Pause while the shared pan sensor is looking sideways. */
        return out;
    }
    return move_step(now, distance, heading, flags);
}

static obstacle_drive_t stopped (void)
{
    obstacle_drive_t out = {0, 0, true, false, false, false};
    return out;
}

static void enter_phase (obs_state_phase_t phase, uint32_t now,
                         int32_t distance)
{
    g_seq.phase = phase;
    g_seq.entered = now;
    g_seq.segment = distance;
    obs_state_set_phase(phase);
    tm_printf((UB *)"[OBS] phase=%u\n", (UINT)phase);
}

static obstacle_drive_t fail (uint32_t fault)
{
    obstacle_drive_t out = stopped();

    g_seq.phase = OBS_PHASE_FAULT;
    obs_state_faults_update(fault, 0U);
    obs_state_set_phase(OBS_PHASE_FAULT);
    obs_state_flags_clear(OBS_EVT_SCAN_REQ | OBS_EVT_FRONT_READY |
                          OBS_EVT_SIDE_WATCH);
    obs_state_flags_set(OBS_EVT_ABORT);
    out.b_fault = true;
    tm_printf((UB *)"[OBS] FAULT 0x%x\n", (UINT)fault);
    return out;
}

static obstacle_drive_t finish (bool count)
{
    obstacle_drive_t out = stopped();

    obs_state_flags_clear(OBS_EVT_BLOCKED | OBS_EVT_SCAN_REQ |
                          OBS_EVT_SCAN_DONE | OBS_EVT_SIDE_WATCH |
                          OBS_EVT_SIDE_SEEN | OBS_EVT_SIDE_CLEAR |
                          OBS_EVT_FRONT_BLOCKED | OBS_EVT_FRONT_READY);
    if (count)
    {
        obs_state_count_bypass();
    }
    obs_state_set_phase(OBS_PHASE_IDLE);
    obs_state_set_route(OBS_ROUTE_NONE);
    g_seq.active = false;
    out.b_done = true;
    return out;
}

static int32_t wheel_mm (int32_t counts)
{
    return (int32_t)(((int64_t)counts * OBS_CIRC_X10) / (10 * OBS_CPR));
}

static obstacle_drive_t take_plan (uint32_t now, int32_t distance,
                                   int32_t heading)
{
    scan_plan_bypass_t plan = {0U, 0U};

    obs_state_plan_load(&plan);
    obs_state_flags_clear(OBS_EVT_SCAN_DONE);
    uint32_t const result = scan_plan_result(&plan);

    if (result == SCAN_PLAN_CLEAR)
    {
        /* Off-line, a clear scan is not proof of reacquisition. */
        if (g_seq.armed)
        {
            return fail(OBS_FAULT_NO_ROUTE);
        }
        return finish(false);
    }
    if (result != SCAN_PLAN_ROUTE)
    {
        return fail(OBS_FAULT_NO_ROUTE);
    }
    g_seq.side = scan_plan_side_is_left(&plan) ? 1 : -1;
    g_seq.angle = scan_plan_angle_deg(scan_plan_angle_index(&plan)) * 10;
    g_seq.diagonal = scan_plan_diag_cm(&plan) * 10;
    g_seq.heading0 = heading;
    obs_state_set_route((g_seq.side > 0) ? OBS_ROUTE_LEFT : OBS_ROUTE_RIGHT);
    obs_state_set_width_cm(scan_plan_width_cm(&plan));
    obs_state_flags_clear(OBS_EVT_FRONT_READY);
    enter_phase(OBS_PHASE_TURN_OUT, now, distance);
    return stopped();
}

static obstacle_drive_t move_step (uint32_t now, int32_t distance,
                                   int32_t heading, UINT flags)
{
    obstacle_drive_t out = {OBS_DRIVE_RPM, OBS_DRIVE_RPM,
                            false, false, false, false};
    int32_t const segment = distance - g_seq.segment;
    int32_t const relative = g_seq.side * (heading - g_seq.heading0);
    int32_t target = g_seq.heading0;
    int32_t turn = 0;
    bool reached = false;
    obs_state_phase_t next = g_seq.phase;

    switch (g_seq.phase)
    {
        case OBS_PHASE_TURN_OUT:
            turn = g_seq.side;
            reached = relative >= (g_seq.angle - OBS_TURN_LEAD_DDEG);
            next = OBS_PHASE_DIAG_OUT;
        break;
        case OBS_PHASE_DIAG_OUT:
            target += g_seq.side * g_seq.angle;
            reached = segment >= g_seq.diagonal;
            next = OBS_PHASE_TURN_PARALLEL;
        break;
        case OBS_PHASE_TURN_PARALLEL:
            turn = -g_seq.side;
            reached = relative <= OBS_TURN_LEAD_DDEG;
            next = OBS_PHASE_PASS_SIDE;
            if (reached)
            {
                /* A left bypass places the obstacle on the right. */
                obs_state_watch_side_set(-g_seq.side);
                obs_state_flags_clear(OBS_EVT_SIDE_SEEN |
                                      OBS_EVT_SIDE_CLEAR);
                obs_state_flags_set(OBS_EVT_SIDE_WATCH);
            }
        break;
        case OBS_PHASE_PASS_SIDE:
            reached = (flags & OBS_EVT_SIDE_CLEAR) != 0U;
            next = OBS_PHASE_PASS_END;
            if (reached)
            {
                obs_state_flags_clear(OBS_EVT_SIDE_WATCH);
            }
            else if (segment >= OBS_PASS_MAX_MM)
            {
                return fail(OBS_FAULT_NO_ROUTE);
            }
        break;
        case OBS_PHASE_PASS_END:
            reached = segment >= (OBS_ROBOT_LENGTH_MM + OBS_PASS_MARGIN_MM);
            next = OBS_PHASE_TURN_BACK;
        break;
        case OBS_PHASE_TURN_BACK:
            turn = -g_seq.side;
            reached = relative <= (OBS_TURN_LEAD_DDEG - g_seq.angle);
            next = OBS_PHASE_SEARCH_LINE;
        break;
        case OBS_PHASE_SEARCH_LINE:
            target -= g_seq.side * g_seq.angle;
            if (segment > ((g_seq.diagonal * 3) / 2 + OBS_SEARCH_MARGIN_MM))
            {
                return fail(OBS_FAULT_LINE_LOST);
            }
        break;
        default:
            return fail(OBS_FAULT_TIMEOUT);
        break;
    }
    if (reached)
    {
        enter_phase(next, now, distance);
        return stopped();
    }
    if (turn != 0)
    {
        out.left_rpm = (turn > 0) ? 0 : OBS_DRIVE_RPM;
        out.right_rpm = (turn > 0) ? OBS_DRIVE_RPM : 0;
    }
    else
    {
        int32_t trim = (target - heading) / OBS_HEADING_TRIM_DIV;

        if (trim > OBS_HEADING_TRIM_MAX)
        {
            trim = OBS_HEADING_TRIM_MAX;
        }
        if (trim < -OBS_HEADING_TRIM_MAX)
        {
            trim = -OBS_HEADING_TRIM_MAX;
        }
        out.left_rpm -= trim;
        out.right_rpm += trim;
    }
    return out;
}

/*** end of file ***/

