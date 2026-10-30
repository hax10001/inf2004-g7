/** @file task_obstacle.c
 *
 * @brief Front monitor, panning scan, side watch and forward checks; sole
 * owner of the ultrasonic sensor and servo.
 *
 * While the robot passes the object the sensor looks at the object, which is
 * the only way to see its far edge. The sensor then cannot see ahead, so the
 * sequencer stops every OBS_FWD_CHECK_MM and asks for a forward check here.
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
#define ALL_BYPASS_FLAGS                                                       \
    (OBS_EVT_BLOCKED | OBS_EVT_SCAN_REQ | OBS_EVT_SCAN_DONE |                  \
     OBS_EVT_SIDE_WATCH | OBS_EVT_SIDE_SEEN | OBS_EVT_SIDE_CLEAR |             \
     OBS_EVT_FWD_REQ | OBS_EVT_FWD_DONE | OBS_EVT_FWD_CLEAR | OBS_EVT_GUARD |  \
     OBS_EVT_FRONT_HIT | OBS_EVT_ABORT)

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
    uint32_t far_hits;
    uint32_t no_echo;
    bool     b_seen;
    bool     b_aimed;
} side_watch_t;

/** @brief Ping the front and raise OBS_EVT_BLOCKED on a confirmed hit. */
static void run_monitor (monitor_t * p_monitor);
/** @brief Judge the filtered front range once the window is full. */
static void judge_front (monitor_t * p_monitor);
/** @brief Ping the front while moving; raise OBS_EVT_FRONT_HIT on a hit. */
static void run_guard (monitor_t * p_guard);
/** @brief Sweep the servo, build the plan and flag OBS_EVT_SCAN_DONE. */
static void run_scan (void);
/** @brief Watch the side for the object to pass. */
static void run_side_watch (side_watch_t * p_watch);
/** @brief Look forward once, stopped, and report clear or not. */
static void run_forward_check (side_watch_t * p_watch);
/** @brief Return the servo to FRONT and reset the side watch. */
static void leave_side_watch (side_watch_t * p_watch);
/** @brief Cancel everything and park the servo. */
static void abort_all (monitor_t * p_monitor, monitor_t * p_guard,
                       side_watch_t * p_watch);
/** @brief Median of OBS_SCAN_PINGS pings; false if the sensor was silent. */
static bool scan_range (int32_t * p_range_mm);
/** @brief Record one silent ping; latch the fault after the limit. */
static void note_silent (uint32_t * p_count);

void task_obstacle_task (INT start_code, void * p_context)
{
    monitor_t    monitor = {{0, 0, 0}, 0U, 0U, 0U, 0U};
    monitor_t    guard   = {{0, 0, 0}, 0U, 0U, 0U, 0U};
    side_watch_t watch   = {0U, 0U, false, false};

    /* Casting unused arguments to void cannot alter their values. */
    (void)start_code;
    (void)p_context;

    tk_dly_tsk((RELTIM)servo_set_angle(0));
    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[OBS] Task started\n");

    for (;;)
    {
        UINT const pattern = obs_state_flags_read();

        if (0U == (pattern & OBS_EVT_SIDE_WATCH))
        {
            leave_side_watch(&watch);
        }

        if (0U != (pattern & OBS_EVT_ABORT))
        {
            abort_all(&monitor, &guard, &watch);
        }
        else if (0U != (pattern & OBS_EVT_FWD_REQ))
        {
            run_forward_check(&watch);
        }
        else if (0U != (pattern & OBS_EVT_SCAN_REQ))
        {
            run_scan();
        }
        else if (0U != (pattern & OBS_EVT_SIDE_WATCH))
        {
            run_side_watch(&watch);
        }
        else if (0U == (pattern & OBS_EVT_BLOCKED))
        {
            run_monitor(&monitor);
        }
        else if (0U != (pattern & OBS_EVT_GUARD))
        {
            run_guard(&guard);
        }
        else
        {
            /* Blocked: the controller stops and will request a scan. */
        }

        tk_dly_tsk(OBS_PING_PERIOD_MSEC);
    }
}

static void note_silent (uint32_t * p_count)
{
    if (*p_count < OBS_NO_ECHO_LIMIT)
    {
        (*p_count)++;
    }

    if (*p_count >= OBS_NO_ECHO_LIMIT)
    {
        obs_state_faults_update(OBS_FAULT_NO_ECHO, 0U);
    }
}

static void run_monitor (monitor_t * p_monitor)
{
    int32_t                   range_mm = 0;
    ultrasonic_result_t const result   = ultrasonic_ping(&range_mm);

    if (ULTRASONIC_NO_ECHO == result)
    {
        p_monitor->hits = 0U;
        note_silent(&p_monitor->no_echo);
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

static void run_guard (monitor_t * p_guard)
{
    int32_t                   range_mm = 0;
    ultrasonic_result_t const result   = ultrasonic_ping(&range_mm);

    if (ULTRASONIC_NO_ECHO == result)
    {
        p_guard->hits = 0U;
        note_silent(&p_guard->no_echo);
    }
    else
    {
        p_guard->no_echo = 0U;

        if ((ULTRASONIC_OK == result) && (range_mm < OBS_BYPASS_STOP_MM))
        {
            p_guard->hits++;
        }
        else
        {
            p_guard->hits = 0U;
        }

        if (p_guard->hits >= OBS_CONFIRM_HITS)
        {
            p_guard->hits = 0U;
            obs_state_flags_set(OBS_EVT_FRONT_HIT);
            /* UINT matches %u on the 32-bit target; UB preserves bytes. */
            tm_printf((UB *)"[OBS] Object ahead in bypass at %u mm\n",
                      (UINT)range_mm);
        }
    }
}

static void run_scan (void)
{
    scan_plan_point_t points[OBS_SCAN_POINTS];
    int32_t           count = 0;
    bool              b_ok  = true;

    for (int32_t angle = -OBS_SCAN_RIGHT_DEG;
         (angle <= OBS_SCAN_LEFT_DEG) && (count < OBS_SCAN_POINTS) && b_ok;
         angle += OBS_SCAN_STEP_DEG)
    {
        if (0U != (obs_state_flags_read() & OBS_EVT_ABORT))
        {
            b_ok = false; /* The main loop handles the abort. */
        }
        else
        {
            int32_t range_mm = 0;

            tk_dly_tsk((RELTIM)servo_set_angle(angle));

            if (scan_range(&range_mm))
            {
                points[count].angle_deg = angle;
                points[count].range_mm  = range_mm;
                count++;
            }
            else
            {
                /* A silent sensor is not free space: no plan is built. */
                obs_state_faults_update(OBS_FAULT_NO_ECHO, 0U);
                b_ok = false;
            }
        }
    }

    if (b_ok)
    {
        scan_plan_bypass_t plan = {0U, 0U};

        tk_dly_tsk((RELTIM)servo_set_angle(0));
        scan_plan_build(points, count, &plan);
        obs_state_plan_store(&plan);
        obs_state_flags_set(OBS_EVT_SCAN_DONE);
        obs_state_flags_clear(OBS_EVT_SCAN_REQ);
        /* Values are non-negative cm; UINT matches %u on the target. */
        tm_printf(
            (UB *)"[OBS] Scan: result=%u left=%u width=%u "
                  "lateral=%u diag=%u\n",
            (UINT)scan_plan_result(&plan), (UINT)scan_plan_side_is_left(&plan),
            (UINT)scan_plan_width_cm(&plan), (UINT)scan_plan_lateral_cm(&plan),
            (UINT)scan_plan_diag_cm(&plan));
    }
}

static void run_side_watch (side_watch_t * p_watch)
{
    int32_t             range_mm = 0;
    ultrasonic_result_t result   = ULTRASONIC_NO_ECHO;

    if (!p_watch->b_aimed)
    {
        tk_dly_tsk((RELTIM)servo_set_angle(obs_state_watch_side_get() *
                                           OBS_SIDE_LOOK_DEG));
        p_watch->b_aimed = true;
    }

    result = ultrasonic_ping(&range_mm);

    if (ULTRASONIC_NO_ECHO == result)
    {
        /* Silence is a sensor fault, never evidence that the way is clear. */
        p_watch->far_hits = 0U;
        note_silent(&p_watch->no_echo);
    }
    else if ((ULTRASONIC_OK == result) && (range_mm < OBS_SIDE_NEAR_MM))
    {
        p_watch->no_echo  = 0U;
        p_watch->b_seen   = true;
        p_watch->far_hits = 0U;
        obs_state_side_range_set(range_mm);
        obs_state_flags_set(OBS_EVT_SIDE_SEEN);
    }
    else if (p_watch->b_seen)
    {
        p_watch->no_echo = 0U;
        p_watch->far_hits++;

        if (p_watch->far_hits >= OBS_SIDE_CLEAR_HITS)
        {
            obs_state_flags_set(OBS_EVT_SIDE_CLEAR);
        }
    }
    else
    {
        /* The object has not been seen yet: keep watching. */
        p_watch->no_echo = 0U;
    }
}

static void run_forward_check (side_watch_t * p_watch)
{
    int32_t values[OBS_FWD_PINGS];
    bool    b_silent = false;

    tk_dly_tsk((RELTIM)servo_set_angle(0));
    /* The servo is front now, so the side watch must aim again. */
    p_watch->b_aimed = false;

    for (uint32_t index = 0U; index < OBS_FWD_PINGS; index++)
    {
        int32_t                   range_mm = 0;
        ultrasonic_result_t const result   = ultrasonic_ping(&range_mm);

        if (ULTRASONIC_NO_ECHO == result)
        {
            b_silent      = true;
            values[index] = 0;
        }
        else
        {
            values[index] = (ULTRASONIC_OK == result) ? range_mm : OBS_FAR_MM;
        }

        tk_dly_tsk(OBS_PING_PERIOD_MSEC);
    }

    /* Insertion sort; the middle value is the median. */
    for (uint32_t index = 1U; index < OBS_FWD_PINGS; index++)
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

    if ((!b_silent) && (values[OBS_FWD_PINGS / 2U] >= OBS_FWD_CLEAR_MM))
    {
        obs_state_flags_set(OBS_EVT_FWD_CLEAR);
    }

    /* DONE is set last, so a reader that sees it also sees CLEAR. */
    obs_state_flags_set(OBS_EVT_FWD_DONE);
    /* UINT matches %u on the 32-bit target; UB preserves bytes. */
    tm_printf((UB *)"[OBS] Forward check: %u mm%s\n",
              (UINT)values[OBS_FWD_PINGS / 2U], b_silent ? " (silent)" : "");
}

static void leave_side_watch (side_watch_t * p_watch)
{
    if (p_watch->b_aimed)
    {
        tk_dly_tsk((RELTIM)servo_set_angle(0));
    }

    p_watch->b_aimed  = false;
    p_watch->b_seen   = false;
    p_watch->far_hits = 0U;
    p_watch->no_echo  = 0U;
}

static void abort_all (monitor_t * p_monitor, monitor_t * p_guard,
                       side_watch_t * p_watch)
{
    obs_state_flags_clear(ALL_BYPASS_FLAGS);
    leave_side_watch(p_watch);
    tk_dly_tsk((RELTIM)servo_set_angle(0));
    p_monitor->hits   = 0U;
    p_monitor->filled = 0U;
    p_guard->hits     = 0U;
    p_guard->no_echo  = 0U;
}

static bool scan_range (int32_t * p_range_mm)
{
    int32_t  values[OBS_SCAN_PINGS];
    uint32_t silent = 0U;

    for (uint32_t index = 0U; index < OBS_SCAN_PINGS; index++)
    {
        int32_t                   range_mm = 0;
        ultrasonic_result_t const result   = ultrasonic_ping(&range_mm);

        if (ULTRASONIC_NO_ECHO == result)
        {
            silent++;
            values[index] = OBS_FAR_MM;
        }
        else
        {
            /* A pulse that never ends or is too long means nothing in range. */
            values[index] = (ULTRASONIC_OK == result) ? range_mm : OBS_FAR_MM;
        }

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

    *p_range_mm = values[OBS_SCAN_PINGS / 2U];

    /* A majority of silent pings means the sensor is not working. */
    return silent <= (OBS_SCAN_PINGS / 2U);
}

/*** end of file ***/

