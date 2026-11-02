/** @file task_obstacle.c
 *
 * @brief Front monitor, panning scan, side watch and front watch; sole
 * owner of the ultrasonic sensor and servo.
 *
 * Every ping is published with the bearing the servo had while it was
 * taken and with what it found: a real echo, nothing within range, or no
 * echo pulse at all. The orientation state says whether the sensor is
 * settled at FRONT, settled at a side or still moving; no ping is taken
 * while it moves, so a sideways range is never reported as a front range.
 */
#include "task_obstacle.h"
#include "obs_config.h"
#include "obs_state.h"
#include "scan_plan.h"
#include "servo.h"
#include "ultrasonic.h"
#include <stdbool.h>
#include <tm/tmonitor.h>

#define MONITOR_WINDOW 3U
#define SCAN_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MM_PER_CM 10
#define PAIR 2U

typedef struct
{
    int32_t  history[MONITOR_WINDOW];
    uint32_t next;
    uint32_t filled;
    uint32_t hits;
    uint32_t no_echo;
} monitor_t;

typedef struct
{
    bool     b_aimed;
    uint32_t pings;     /**< Side pings since the last front glance. */
    int32_t  angle_deg; /**< Bearing the side watch is aimed at. */
} side_watch_t;

/** @brief Ping the front and raise OBS_EVT_BLOCKED on a confirmed hit. */
static void run_monitor (monitor_t * p_monitor);
/** @brief Judge the filtered front range once the window is full. */
static void judge_front (monitor_t * p_monitor);
/** @brief Sweep the servo, build the plan and flag OBS_EVT_SCAN_DONE. */
static void run_scan (void);
/** @brief Fill the scan bearings, ordered from right to left.
 *  @return Number of bearings.
 */
static int32_t scan_bearings (int32_t * p_bearings);
/** @brief Ping the side and publish it; the sequencer judges it. With
 *  OBS_EVT_FRONT_WATCH also set, glance ahead now and then.
 */
static void run_side_watch (side_watch_t * p_watch, UINT pattern);
/** @brief Ping straight ahead and publish it for the bypass guard. */
static void run_front_watch (void);
/** @brief Return the servo to FRONT and reset the side watch. */
static void leave_side_watch (side_watch_t * p_watch);
/** @brief Cancel everything and park the servo. */
static void abort_all (monitor_t * p_monitor, side_watch_t * p_watch);
/** @brief Filtered range at the current bearing.
 *  @return mm, OBS_FAR_MM when nothing is in range, or -1 when unknown.
 */
static int32_t scan_range (void);
/** @brief Re-centre the servo if it is not at FRONT; old pings are dropped. */
static void face_front (monitor_t * p_monitor);
/** @brief Command a bearing, publish the moving and settled states, and
 *  wait for the servo to settle.
 */
static void move_servo (int32_t angle_deg);
/** @brief Kernel time in milliseconds. */
static uint32_t now_msec (void);
/** @brief Map a ping result to what it says about the space. */
static obs_state_echo_t echo_of (ultrasonic_result_t result);
/** @brief Log the mounting and sweep in use at startup. */
static void log_geometry (void);

void task_obstacle_task (INT start_code, void * p_context)
{
    monitor_t    monitor = {{0, 0, 0}, 0U, 0U, 0U, 0U};
    side_watch_t watch   = {false, 0U, 0};

    /* Casting unused arguments to void cannot alter their values. */
    (void)start_code;
    (void)p_context;

    /* Face FRONT before the first ping, wherever the servo was at power-on. */
    obs_state_aim_set(OBS_AIM_RETURNING, 0, now_msec());
    tk_dly_tsk((RELTIM)servo_home());
    obs_state_aim_set(OBS_AIM_FORWARD, 0, now_msec());
    log_geometry();

    for (;;)
    {
        UINT const pattern = obs_state_flags_read();

        if (0U == (pattern & OBS_EVT_SIDE_WATCH))
        {
            leave_side_watch(&watch);
        }

        if (0U != (pattern & OBS_EVT_ABORT))
        {
            abort_all(&monitor, &watch);
        }
        else if (0U != (pattern & OBS_EVT_SCAN_REQ))
        {
            run_scan();
        }
        else if (0U != (pattern & OBS_EVT_SIDE_WATCH))
        {
            run_side_watch(&watch, pattern);
        }
        else if (0U != (pattern & OBS_EVT_FRONT_WATCH))
        {
            face_front(&monitor);
            run_front_watch();
        }
        else if (0U == (pattern & OBS_EVT_BLOCKED))
        {
            face_front(&monitor);
            run_monitor(&monitor);
        }
        else
        {
            /* Blocked: the controller stops and will request a scan. */
        }

        tk_dly_tsk(OBS_PING_PERIOD_MSEC);
    }
}

static void run_monitor (monitor_t * p_monitor)
{
    int32_t                   range_mm = 0;
    ultrasonic_result_t const result   = ultrasonic_ping(&range_mm);

    if (ULTRASONIC_NO_ECHO == result)
    {
        p_monitor->hits = 0U;

        if (p_monitor->no_echo < OBS_NO_ECHO_LIMIT)
        {
            p_monitor->no_echo++;
        }

        if (p_monitor->no_echo >= OBS_NO_ECHO_LIMIT)
        {
            obs_state_faults_update(OBS_FAULT_NO_ECHO, 0U);
        }
    }
    else
    {
        p_monitor->no_echo = 0U;
        obs_state_faults_update(0U, OBS_FAULT_NO_ECHO);

        if (ULTRASONIC_OK != result)
        {
            range_mm = OBS_FAR_MM;
        }

        p_monitor->history[p_monitor->next] = range_mm;
        p_monitor->next = (p_monitor->next + 1U) % MONITOR_WINDOW;

        if (p_monitor->filled < MONITOR_WINDOW)
        {
            p_monitor->filled++;
        }

        if (p_monitor->filled >= MONITOR_WINDOW)
        {
            judge_front(p_monitor);
        }
    }
}

static void judge_front (monitor_t * p_monitor)
{
    int32_t const median = ultrasonic_median_of_three(
        p_monitor->history[0], p_monitor->history[1], p_monitor->history[2]);

    obs_state_set_distance_mm(median);

    if (median < OBS_STOP_MM)
    {
        p_monitor->hits++;
    }
    else
    {
        p_monitor->hits = 0U;
    }

    if (p_monitor->hits >= OBS_CONFIRM_HITS)
    {
        p_monitor->hits   = 0U;
        p_monitor->filled = 0U;
        obs_state_set_phase(OBS_PHASE_CONFIRM);
        obs_state_flags_set(OBS_EVT_BLOCKED);
        /* UINT matches %u on the 32-bit target; UB preserves bytes. */
        tm_printf((UB *)"[OBS] Obstacle confirmed at %u mm\n", (UINT)median);
    }
}

static void run_scan (void)
{
    scan_plan_point_t points[OBS_SCAN_MAX_POINTS];
    int32_t           bearings[OBS_SCAN_MAX_POINTS];
    int32_t const     count = scan_bearings(bearings);
    int32_t           taken = 0;
    bool              b_ok  = true;

    /* One sweep from right to left keeps servo travel to a minimum. */
    for (int32_t index = 0; (index < count) && b_ok; index++)
    {
        if (0U != (obs_state_flags_read() & OBS_EVT_ABORT))
        {
            b_ok = false; /* The main loop handles the abort. */
        }
        else
        {
            move_servo(bearings[index]);
            points[taken].angle_deg = servo_angle_get();
            points[taken].range_mm  = scan_range();
            taken++;
        }
    }

    if (b_ok)
    {
        scan_plan_bypass_t plan = {0U, 0U};

        /* Back at FRONT, settled and published, before any movement. */
        move_servo(0);

        for (int32_t index = 0; index < taken; index++)
        {
            /* Values fit INT on the 32-bit target; UB preserves bytes. */
            tm_printf((UB *)"[OBS] Scan point %d deg: %d mm\n",
                      (INT)points[index].angle_deg,
                      (INT)points[index].range_mm);
        }

        /* A previous front hit remains valid when the new centre echo
         * is out of range, absent or less reliable after servo movement.
         */
        uint32_t const last_cm =
            obs_state_status_distance_cm(obs_state_status_get());
        /* The field is 9 bits wide, so the product fits in int32_t. */
        int32_t const last_mm = (int32_t)last_cm * MM_PER_CM;

        scan_plan_build_confirmed(points, taken, last_mm, &plan);
        obs_state_plan_store(&plan);
        obs_state_flags_set(OBS_EVT_SCAN_DONE);
        obs_state_flags_clear(OBS_EVT_SCAN_REQ);
        /* Values are non-negative cm; UINT matches %u on the target. */
        tm_printf(
            (UB *)"[OBS] Scan: result=%u left=%u width=%u near=%u "
                  "lateral=%u out=%u deg slack=%u\n",
            (UINT)scan_plan_result(&plan), (UINT)scan_plan_side_is_left(&plan),
            (UINT)scan_plan_width_cm(&plan), (UINT)scan_plan_near_cm(&plan),
            (UINT)scan_plan_lateral_cm(&plan),
            (UINT)scan_plan_angle_deg(scan_plan_angle_index(&plan)),
            (UINT)scan_plan_slack_cm(&plan));
    }
}

static int32_t scan_bearings (int32_t * p_bearings)
{
    /* Whole steps only, so every sample is taken where it is labelled. */
    int32_t const right_reach =
        SCAN_MIN(OBS_SCAN_FAR_DEG, servo_right_limit_deg());
    int32_t const left_reach =
        SCAN_MIN(OBS_SCAN_FAR_DEG, servo_left_limit_deg());
    int32_t const right_steps =
        SCAN_MIN(OBS_SCAN_RIGHT_DEG, right_reach) / OBS_SCAN_STEP_DEG;
    int32_t const left_steps =
        SCAN_MIN(OBS_SCAN_LEFT_DEG, left_reach) / OBS_SCAN_STEP_DEG;
    int32_t count = 0;

    if (right_reach > (right_steps * OBS_SCAN_STEP_DEG))
    {
        p_bearings[count] = -right_reach;
        count++;
    }

    for (int32_t step = -right_steps; step <= left_steps; step++)
    {
        p_bearings[count] = step * OBS_SCAN_STEP_DEG;
        count++;
    }

    if (left_reach > (left_steps * OBS_SCAN_STEP_DEG))
    {
        p_bearings[count] = left_reach;
        count++;
    }

    return count;
}

static void run_side_watch (side_watch_t * p_watch, UINT pattern)
{
    int32_t const wanted_angle = obs_state_watch_angle_get();
    bool const    b_request =
        (!p_watch->b_aimed) || (p_watch->angle_deg != wanted_angle);
    bool const b_glance_due = (0U != (pattern & OBS_EVT_FRONT_WATCH)) &&
                              (p_watch->pings >= OBS_GLANCE_EVERY);
    bool const b_glance_late =
        b_glance_due && (p_watch->pings >= (PAIR * OBS_GLANCE_EVERY));

    /* A newly asked-for bearing is served before a glance falls due, so
     * the return check is ready on time; an overdue glance still wins.
     */
    if (b_glance_due && ((!b_request) || b_glance_late))
    {
        /* Look ahead once, then re-aim at the side on the next pass. */
        p_watch->pings   = 0U;
        p_watch->b_aimed = false;
        move_servo(0);
        run_front_watch();
    }
    else
    {
        int32_t range_mm = 0;

        if (b_request)
        {
            move_servo(wanted_angle);
            p_watch->b_aimed   = true;
            p_watch->angle_deg = wanted_angle;
        }

        ultrasonic_result_t const result = ultrasonic_ping(&range_mm);

        /* The bearing is the clamped one the servo was really sent to. */
        obs_state_side_reading_set(echo_of(result), range_mm,
                                   servo_angle_get(), now_msec());
        p_watch->pings++;
    }
}

static void run_front_watch (void)
{
    int32_t range_mm = 0;

    if (0 != servo_angle_get())
    {
        move_servo(0);
    }

    ultrasonic_result_t const result = ultrasonic_ping(&range_mm);

    obs_state_front_reading_set(echo_of(result), range_mm, now_msec());
}

static void leave_side_watch (side_watch_t * p_watch)
{
    if (p_watch->b_aimed)
    {
        move_servo(0);
    }

    p_watch->b_aimed = false;
    p_watch->pings   = 0U;
}

static void abort_all (monitor_t * p_monitor, side_watch_t * p_watch)
{
    obs_state_flags_clear(OBS_EVT_BLOCKED | OBS_EVT_SCAN_REQ |
                          OBS_EVT_SCAN_DONE | OBS_EVT_SIDE_WATCH |
                          OBS_EVT_FRONT_WATCH | OBS_EVT_ABORT);
    leave_side_watch(p_watch);
    move_servo(0);
    p_monitor->hits   = 0U;
    p_monitor->filled = 0U;
}

static int32_t scan_range (void)
{
    int32_t  values[OBS_SCAN_PINGS];
    uint32_t informative = 0U;
    bool     b_agreed    = false;
    int32_t  result      = -1;

    for (uint32_t ping = 0U; (ping < OBS_SCAN_PINGS) && (!b_agreed); ping++)
    {
        int32_t                   range_mm = 0;
        ultrasonic_result_t const outcome  = ultrasonic_ping(&range_mm);

        /* No echo pulse says nothing about the space, so it is dropped. */
        if (ULTRASONIC_NO_ECHO != outcome)
        {
            values[informative] =
                (ULTRASONIC_OK == outcome) ? range_mm : OBS_FAR_MM;
            informative++;
        }

        tk_dly_tsk(OBS_PING_PERIOD_MSEC);

        if (PAIR == informative)
        {
            int32_t const gap = values[0] - values[1];
            bool const b_both_far =
                (values[0] >= OBS_FAR_MM) && (values[1] >= OBS_FAR_MM);
            bool const b_both_near =
                (values[0] < OBS_FAR_MM) && (values[1] < OBS_FAR_MM) &&
                (gap <= OBS_SCAN_AGREE_MM) && (gap >= -OBS_SCAN_AGREE_MM);

            b_agreed = b_both_far || b_both_near;
        }
    }

    /* Insertion sort of the informative values. */
    for (uint32_t index = 1U; index < informative; index++)
    {
        int32_t const key  = values[index];
        uint32_t      slot = index;

        while ((slot > 0U) && (values[slot - 1U] > key))
        {
            values[slot] = values[slot - 1U];
            slot--;
        }

        values[slot] = key;
    }

    if (informative > PAIR)
    {
        result = values[informative / PAIR];
    }
    else if (PAIR == informative)
    {
        /* Two values bound the median; the nearer one is the safe pick. */
        result = values[0];
    }
    else
    {
        /* One or no informative ping: unknown, never free space. */
    }

    return result;
}

static void face_front (monitor_t * p_monitor)
{
    if (0 != servo_angle_get())
    {
        move_servo(0);
        p_monitor->hits   = 0U;
        p_monitor->filled = 0U;
    }
}

static void move_servo (int32_t angle_deg)
{
    uint32_t const settle  = servo_set_angle(angle_deg);
    int32_t const  bearing = servo_angle_get();
    obs_state_aim_t settled = OBS_AIM_FORWARD;

    if (bearing > 0)
    {
        settled = OBS_AIM_LEFT;
    }
    else if (bearing < 0)
    {
        settled = OBS_AIM_RIGHT;
    }
    else
    {
        /* Bearing zero is FRONT. */
    }

    obs_state_aim_set((0 == bearing) ? OBS_AIM_RETURNING : OBS_AIM_SETTLING,
                      bearing, now_msec());
    tk_dly_tsk((RELTIM)settle);
    obs_state_aim_set(settled, bearing, now_msec());
}

static uint32_t now_msec (void)
{
    SYSTIM now = {0};

    (void)tk_get_otm(&now);

    /* The low word wraps every 49 days; callers use differences only. */
    return (uint32_t)now.lo;
}

static obs_state_echo_t echo_of (ultrasonic_result_t result)
{
    obs_state_echo_t echo = OBS_ECHO_NONE;

    if (ULTRASONIC_OK == result)
    {
        echo = OBS_ECHO_VALID;
    }
    else if (ULTRASONIC_OUT_OF_RANGE == result)
    {
        /* Out of measurement range is not the same as a lost echo. */
        echo = OBS_ECHO_BEYOND;
    }
    else
    {
        /* No echo pulse: wiring, power or sensor, so nothing is known. */
    }

    return echo;
}

static void log_geometry (void)
{
    /* Values fit INT on the 32-bit target; UB preserves the string bytes. */
    tm_printf((UB *)"[OBS] Task started: FRONT=%d usec (shaft %d deg from "
                    "centre), sweep L=%d R=%d deg\n",
              (INT)OBS_SERVO_FRONT_USEC,
              (INT)((OBS_SERVO_LEFT_SIGN *
                     (OBS_SERVO_FRONT_USEC - OBS_SERVO_CENTRE_USEC)) /
                    OBS_SERVO_USEC_PER_DEG),
              (INT)servo_left_limit_deg(), (INT)servo_right_limit_deg());
    tm_printf((UB *)"[OBS] Sensor at x=%d y=%d z=%d mm, arm %d mm\n",
              (INT)OBS_SENSOR_TO_AXLE_MM, (INT)OBS_SENSOR_LATERAL_MM,
              (INT)OBS_SENSOR_HEIGHT_MM, (INT)OBS_SERVO_ARM_MM);

    if (0 == OBS_GEOMETRY_MEASURED)
    {
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[OBS] WARNING: sensor mounting not yet measured\n");
    }
}

/*** end of file ***/

