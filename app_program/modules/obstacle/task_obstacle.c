/** @file task_obstacle.c
 *
 * @brief Front monitor, panning scan, side watch and front watch; sole
 * owner of the ultrasonic sensor and servo.
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
    uint32_t pings; /**< Side pings since the last front glance. */
} side_watch_t;

/** @brief Ping the front and raise OBS_EVT_BLOCKED on a confirmed hit. */
static void run_monitor (monitor_t * p_monitor);
/** @brief Judge the filtered front range once the window is full. */
static void judge_front (monitor_t * p_monitor);
/** @brief Sweep the servo, build the plan and flag OBS_EVT_SCAN_DONE. */
static void run_scan (void);
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
/** @brief Median of OBS_SCAN_PINGS pings; failures count as far. */
static int32_t scan_range (void);
/** @brief Re-centre the servo if it is not at FRONT; old pings are dropped. */
static void face_front (monitor_t * p_monitor);

void task_obstacle_task (INT start_code, void * p_context)
{
    monitor_t    monitor = {{0, 0, 0}, 0U, 0U, 0U, 0U};
    side_watch_t watch   = {false, 0U};

    /* Casting unused arguments to void cannot alter their values. */
    (void)start_code;
    (void)p_context;

    /* Face FRONT before the first ping, wherever the servo was at power-on. */
    tk_dly_tsk((RELTIM)servo_home());
    /* Values fit INT on the 32-bit target; UB preserves the string bytes. */
    tm_printf((UB *)"[OBS] Task started: FRONT=%d usec, sweep L=%d R=%d deg\n",
              (INT)OBS_SERVO_FRONT_USEC, (INT)servo_left_limit_deg(),
              (INT)servo_right_limit_deg());

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
    scan_plan_point_t points[OBS_SCAN_POINTS];
    int32_t           count = 0;
    bool              b_ok  = true;
    /* Whole steps only, so every sample is taken where it is labelled. */
    int32_t const right_steps =
        SCAN_MIN(OBS_SCAN_RIGHT_DEG, servo_right_limit_deg()) /
        OBS_SCAN_STEP_DEG;
    int32_t const left_steps =
        SCAN_MIN(OBS_SCAN_LEFT_DEG, servo_left_limit_deg()) /
        OBS_SCAN_STEP_DEG;

    for (int32_t angle = -right_steps * OBS_SCAN_STEP_DEG;
         (angle <= (left_steps * OBS_SCAN_STEP_DEG)) &&
         (count < OBS_SCAN_POINTS) && b_ok;
         angle += OBS_SCAN_STEP_DEG)
    {
        if (0U != (obs_state_flags_read() & OBS_EVT_ABORT))
        {
            b_ok = false; /* The main loop handles the abort. */
        }
        else
        {
            tk_dly_tsk((RELTIM)servo_set_angle(angle));
            points[count].angle_deg = angle;
            points[count].range_mm  = scan_range();
            count++;
        }
    }

    if (b_ok)
    {
        scan_plan_bypass_t plan = {0U, 0U};

        tk_dly_tsk((RELTIM)servo_set_angle(0));

        for (int32_t index = 0; index < count; index++)
        {
            /* Values fit INT on the 32-bit target; UB preserves bytes. */
            tm_printf((UB *)"[OBS] Scan point %d deg: %d mm\n",
                      (INT)points[index].angle_deg,
                      (INT)points[index].range_mm);
        }

        scan_plan_build(points, count, &plan);
        obs_state_plan_store(&plan);
        obs_state_flags_set(OBS_EVT_SCAN_DONE);
        obs_state_flags_clear(OBS_EVT_SCAN_REQ);
        /* Values are non-negative cm; UINT matches %u on the target. */
        tm_printf(
            (UB *)"[OBS] Scan: result=%u left=%u width=%u near=%u "
                  "lateral=%u out=%u deg\n",
            (UINT)scan_plan_result(&plan), (UINT)scan_plan_side_is_left(&plan),
            (UINT)scan_plan_width_cm(&plan), (UINT)scan_plan_near_cm(&plan),
            (UINT)scan_plan_lateral_cm(&plan),
            (UINT)scan_plan_angle_deg(scan_plan_angle_index(&plan)));
    }
}

static void run_side_watch (side_watch_t * p_watch, UINT pattern)
{
    if ((0U != (pattern & OBS_EVT_FRONT_WATCH)) &&
        (p_watch->pings >= OBS_GLANCE_EVERY))
    {
        /* Look ahead once, then re-aim at the side on the next pass. */
        p_watch->pings   = 0U;
        p_watch->b_aimed = false;
        tk_dly_tsk((RELTIM)servo_set_angle(0));
        run_front_watch();
    }
    else
    {
        int32_t             range_mm = 0;
        ultrasonic_result_t result   = ULTRASONIC_NO_ECHO;

        if (!p_watch->b_aimed)
        {
            tk_dly_tsk((RELTIM)servo_set_angle(obs_state_watch_angle_get()));
            p_watch->b_aimed = true;
        }

        result = ultrasonic_ping(&range_mm);
        obs_state_side_range_set((ULTRASONIC_OK == result) ? range_mm : -1);
        p_watch->pings++;
    }
}

static void run_front_watch (void)
{
    int32_t                   range_mm = 0;
    ultrasonic_result_t const result   = ultrasonic_ping(&range_mm);

    obs_state_front_range_set((ULTRASONIC_OK == result) ? range_mm : -1);
}

static void leave_side_watch (side_watch_t * p_watch)
{
    if (p_watch->b_aimed)
    {
        tk_dly_tsk((RELTIM)servo_set_angle(0));
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
    tk_dly_tsk((RELTIM)servo_set_angle(0));
    p_monitor->hits   = 0U;
    p_monitor->filled = 0U;
}

static int32_t scan_range (void)
{
    int32_t values[OBS_SCAN_PINGS];

    for (uint32_t index = 0U; index < OBS_SCAN_PINGS; index++)
    {
        int32_t range_mm = 0;

        values[index] = (ULTRASONIC_OK == ultrasonic_ping(&range_mm))
                            ? range_mm
                            : OBS_FAR_MM;
        tk_dly_tsk(OBS_PING_PERIOD_MSEC);
    }

    /* Insertion sort; the middle value is the median. */
    for (uint32_t index = 1U; index < OBS_SCAN_PINGS; index++)
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

    return values[OBS_SCAN_PINGS / 2U];
}

static void face_front (monitor_t * p_monitor)
{
    if (0 != servo_angle_get())
    {
        tk_dly_tsk((RELTIM)servo_set_angle(0));
        p_monitor->hits   = 0U;
        p_monitor->filled = 0U;
    }
}

/*** end of file ***/

