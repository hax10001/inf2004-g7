/** @file task_obstacle.c
 * @brief Front monitor, panning scan and side watch; sole owner of the
 * ultrasonic sensor and servo.
 */
#include "task_obstacle.h"
#include "obs_config.h"
#include "obstacle.h"
#include "scan_plan.h"
#include "servo.h"
#include "ultrasonic.h"
#include <stdbool.h>
#include <tm/tmonitor.h>

typedef struct
{
    uint16_t history[3];
    uint8_t  filled;
    uint8_t  hits;
    uint8_t  no_echo;
} monitor_t;

typedef struct
{
    uint8_t far_hits;
    bool    b_seen;
    bool    b_aimed;
} side_watch_t;

static void     run_monitor (monitor_t * p_monitor);
static void     run_scan (void);
static void     run_side_watch (side_watch_t * p_watch);
static void     leave_side_watch (side_watch_t * p_watch);
static void     abort_all (monitor_t * p_monitor, side_watch_t * p_watch);
static uint16_t scan_range (void);

void task_obstacle_task (INT start_code, void * p_context)
{
    monitor_t    monitor = {{0U, 0U, 0U}, 0U, 0U, 0U};
    side_watch_t watch   = {0U, false, false};

    (void)start_code;
    (void)p_context;

    (void)tk_dly_tsk((RELTIM)servo_set_angle(0));
    tm_printf((UB *)"[OBS] Task started\n");

    for (;;)
    {
        UINT const ptn = obstacle_link_flags_read();

        if (0U == (ptn & OBS_EVT_SIDE_WATCH))
        {
            leave_side_watch(&watch);
        }

        if (0U != (ptn & OBS_EVT_ABORT))
        {
            abort_all(&monitor, &watch);
        }
        else if (0U != (ptn & OBS_EVT_SCAN_REQ))
        {
            run_scan();
        }
        else if (0U != (ptn & OBS_EVT_SIDE_WATCH))
        {
            run_side_watch(&watch);
        }
        else if (0U == (ptn & OBS_EVT_BLOCKED))
        {
            run_monitor(&monitor);
        }
        else
        {
            /* Blocked: the controller is stopping and will request a scan. */
        }

        (void)tk_dly_tsk(OBS_PING_PERIOD_MS);
    }
}

static void run_monitor (monitor_t * p_monitor)
{
    uint16_t    mm  = 0U;
    us_result_t res = ultrasonic_ping(&mm);

    if (US_NO_ECHO == res)
    {
        p_monitor->hits = 0U;

        if (p_monitor->no_echo < OBS_NO_ECHO_LIMIT)
        {
            p_monitor->no_echo++;
        }

        if (p_monitor->no_echo >= OBS_NO_ECHO_LIMIT)
        {
            obstacle_link_faults(OBS_FAULT_NO_ECHO, 0U);
        }

        return;
    }

    p_monitor->no_echo = 0U;
    obstacle_link_faults(0U, OBS_FAULT_NO_ECHO);

    if (US_OK != res)
    {
        mm = OBS_FAR_MM;
    }

    p_monitor->history[p_monitor->filled % 3U] = mm;
    p_monitor->filled++;

    if (p_monitor->filled < 3U)
    {
        return;
    }

    p_monitor->filled = 3U;
    p_monitor->history[0] = p_monitor->history[0];

    uint16_t const median = ultrasonic_median3(
        p_monitor->history[0], p_monitor->history[1], p_monitor->history[2]);

    obstacle_link_publish_distance(median);

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
        obstacle_link_confirm();
        tm_printf((UB *)"[OBS] Obstacle confirmed at %u mm\n", (UINT)median);
    }
}

static void run_scan (void)
{
    scan_point_t pts[OBS_SCAN_MAX_POINTS];
    uint8_t      count = 0U;

    for (int16_t angle = -OBS_SCAN_LIMIT_DEG; angle <= OBS_SCAN_LIMIT_DEG;
         angle         = (int16_t)(angle + OBS_SCAN_STEP_DEG))
    {
        if ((0U != (obstacle_link_flags_read() & OBS_EVT_ABORT)) ||
            (count >= OBS_SCAN_MAX_POINTS))
        {
            return; /* Abort handled by the main loop. */
        }

        (void)tk_dly_tsk((RELTIM)servo_set_angle(angle));
        pts[count].angle_deg = angle;
        pts[count].range_mm  = scan_range();
        count++;
    }

    (void)tk_dly_tsk((RELTIM)servo_set_angle(0));

    bypass_plan_t plan;

    scan_plan_build(pts, count, &plan);
    obstacle_link_store_plan(&plan);
    obstacle_link_flags_set(OBS_EVT_SCAN_DONE);
    obstacle_link_flags_clear(OBS_EVT_SCAN_REQ);
    tm_printf((UB *)"[OBS] Scan done: result=%u side_left=%u angle_idx=%u "
                    "width_cm=%u lateral_cm=%u diag_cm=%u\n",
              (UINT)plan.result, (UINT)plan.side_left, (UINT)plan.angle_idx,
              (UINT)plan.width_cm, (UINT)plan.lateral_cm, (UINT)plan.diag_cm);
}

static void run_side_watch (side_watch_t * p_watch)
{
    if (!p_watch->b_aimed)
    {
        int16_t const angle =
            (int16_t)(obstacle_link_watch_side() * OBS_SIDE_LOOK_DEG);

        (void)tk_dly_tsk((RELTIM)servo_set_angle(angle));
        p_watch->b_aimed = true;
    }

    uint16_t    mm  = 0U;
    us_result_t res = ultrasonic_ping(&mm);

    if ((US_OK == res) && (mm < OBS_SIDE_NEAR_MM))
    {
        p_watch->b_seen   = true;
        p_watch->far_hits = 0U;
        obstacle_link_flags_set(OBS_EVT_SIDE_SEEN);
    }
    else if (p_watch->b_seen)
    {
        p_watch->far_hits++;

        if (p_watch->far_hits >= OBS_SIDE_CLEAR_HITS)
        {
            obstacle_link_flags_set(OBS_EVT_SIDE_CLEAR);
        }
    }
    else
    {
        /* Object not seen yet: keep watching. */
    }
}

static void leave_side_watch (side_watch_t * p_watch)
{
    if (p_watch->b_aimed)
    {
        (void)tk_dly_tsk((RELTIM)servo_set_angle(0));
    }

    p_watch->b_aimed  = false;
    p_watch->b_seen   = false;
    p_watch->far_hits = 0U;
}

static void abort_all (monitor_t * p_monitor, side_watch_t * p_watch)
{
    obstacle_link_flags_clear(OBS_EVT_BLOCKED | OBS_EVT_SCAN_REQ |
                              OBS_EVT_SCAN_DONE | OBS_EVT_SIDE_WATCH |
                              OBS_EVT_SIDE_SEEN | OBS_EVT_SIDE_CLEAR |
                              OBS_EVT_ABORT);
    leave_side_watch(p_watch);
    (void)tk_dly_tsk((RELTIM)servo_set_angle(0));
    p_monitor->hits   = 0U;
    p_monitor->filled = 0U;
}

static uint16_t scan_range (void)
{
    uint16_t values[OBS_SCAN_PINGS];

    for (uint32_t i = 0U; i < OBS_SCAN_PINGS; i++)
    {
        uint16_t mm = 0U;

        values[i] = (US_OK == ultrasonic_ping(&mm)) ? mm : (uint16_t)OBS_FAR_MM;
        (void)tk_dly_tsk(OBS_PING_PERIOD_MS);
    }

    for (uint32_t i = 1U; i < OBS_SCAN_PINGS; i++)
    {
        uint16_t const key = values[i];
        uint32_t       j   = i;

        while ((j > 0U) && (values[j - 1U] > key))
        {
            values[j] = values[j - 1U];
            j--;
        }

        values[j] = key;
    }

    return values[OBS_SCAN_PINGS / 2U];
}

/*** end of file ***/