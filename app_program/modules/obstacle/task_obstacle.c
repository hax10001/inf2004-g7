/** @file task_obstacle.c
 * @brief Forward guarding, cancellable scans and stopped side observations.
 */
#include "task_obstacle.h"
#include "obs_config.h"
#include "obs_guard.h"
#include "obs_state.h"
#include "scan_plan.h"
#include "servo.h"
#include "ultrasonic.h"
#include "app/app_config.h"
#include <stdbool.h>
#include <tm/tmonitor.h>

static int32_t g_front[3];
static uint32_t g_front_next;
static uint32_t g_front_filled;
static uint32_t g_hits;
static uint32_t g_silent;
static uint32_t g_side_far;
static bool g_side_seen;

/** @brief Whether the pending scan is still owned by the controller. */
static bool scanning (void);
/** @brief Wait in bounded slices, abandoning a cancelled scan. */
static bool scan_wait (uint32_t delay);
/** @brief Build a scan only while the controller requests it. */
static void scan_run (void);
/** @brief Centre and measure the forward path. */
static void front_run (bool bypass);
/** @brief Observe the side while movement is paused. */
static void side_run (void);

void task_obstacle_task (INT start_code, void * p_context)
{
    (void)start_code;
    (void)p_context;
    tk_dly_tsk((RELTIM)servo_set_angle(0));

    for (;;)
    {
        UINT const flags = obs_state_flags_read();
        obs_state_phase_t const phase =
            obs_state_status_phase(obs_state_status_get());
        bool const moving = (phase >= OBS_PHASE_TURN_OUT) &&
                            (phase <= OBS_PHASE_VERIFY);

        if ((flags & OBS_EVT_ABORT) != 0U)
        {
            obs_state_flags_clear(OBS_EVT_SCAN_REQ | OBS_EVT_SCAN_DONE |
                                  OBS_EVT_SIDE_WATCH | OBS_EVT_FRONT_READY |
                                  OBS_EVT_FRONT_BLOCKED | OBS_EVT_ABORT);
            tk_dly_tsk((RELTIM)servo_set_angle(0));
            g_hits = 0U;
            g_front_filled = 0U;
            g_silent = 0U;
            g_side_far = 0U;
            g_side_seen = false;
        }
        else if ((flags & OBS_EVT_SCAN_REQ) != 0U)
        {
            scan_run();
        }
        else if (moving)
        {
            if ((flags & OBS_EVT_SIDE_WATCH) != 0U)
            {
                side_run();
                tk_dly_tsk(OBS_PING_PERIOD_MSEC);
            }
            else
            {
                g_side_seen = false;
                g_side_far = 0U;
            }
            front_run(true);
        }
        else if (phase == OBS_PHASE_IDLE)
        {
            front_run(false);
        }
        else
        {
            obs_state_flags_clear(OBS_EVT_FRONT_READY);
        }
        tk_dly_tsk(OBS_PING_PERIOD_MSEC);
    }
}

static bool scanning (void)
{
    UINT const flags = obs_state_flags_read();

    return ((flags & OBS_EVT_SCAN_REQ) != 0U) &&
           ((flags & OBS_EVT_ABORT) == 0U) &&
           (obs_state_status_phase(obs_state_status_get()) ==
            OBS_PHASE_SCANNING);
}

static bool scan_wait (uint32_t delay)
{
    SYSTIM start = {0};
    SYSTIM now = {0};

    if (tk_get_otm(&start) < E_OK)
    {
        return false;
    }
    now = start;
    while (scanning() && ((uint32_t)(now.lo - start.lo) < delay))
    {
        tk_dly_tsk(10);
        if (tk_get_otm(&now) < E_OK)
        {
            return false;
        }
    }
    return scanning();
}

static void scan_run (void)
{
    scan_plan_point_t points[OBS_SCAN_POINTS];
    int32_t count = 0;

    obs_state_flags_clear(OBS_EVT_FRONT_READY);
    for (int32_t angle = -OBS_SCAN_RIGHT_DEG;
         angle <= OBS_SCAN_LEFT_DEG; angle += OBS_SCAN_STEP_DEG)
    {
        if ((count >= OBS_SCAN_POINTS) || !scanning())
        {
            return;
        }
        if (!scan_wait(servo_set_angle(angle)))
        {
            return;
        }
        int32_t samples[OBS_SCAN_PINGS];
        uint32_t valid = 0U;

        for (uint32_t i = 0U; i < OBS_SCAN_PINGS; i++)
        {
            int32_t mm = 0;
            ultrasonic_result_t const result = ultrasonic_ping(&mm);

            samples[i] = OBS_FAR_MM;
            if (result == ULTRASONIC_OK)
            {
                samples[i] = mm;
                valid++;
            }
            if (!scan_wait(OBS_PING_PERIOD_MSEC))
            {
                return;
            }
        }
        /* Unknown directions cannot establish a safe corridor. */
        if (valid <= (OBS_SCAN_PINGS / 2U))
        {
            obs_state_faults_update(OBS_FAULT_NO_ECHO, 0U);
            obs_state_flags_set(OBS_EVT_FRONT_BLOCKED);
            scan_plan_bypass_t const failed_plan = {SCAN_PLAN_NO_ROUTE, 0U};

            obs_state_plan_store(&failed_plan);
            obs_state_flags_clear(OBS_EVT_SCAN_REQ);
            obs_state_flags_set(OBS_EVT_SCAN_DONE);
            return;
        }
        for (uint32_t i = 1U; i < OBS_SCAN_PINGS; i++)
        {
            int32_t const value = samples[i];
            uint32_t j = i;

            while ((j > 0U) && (samples[j - 1U] > value))
            {
                samples[j] = samples[j - 1U];
                j--;
            }
            samples[j] = value;
        }
        points[count].angle_deg = angle;
        points[count].range_mm = samples[OBS_SCAN_PINGS / 2U];
        count++;
    }
    if (!scan_wait(servo_set_angle(0)))
    {
        return;
    }
    scan_plan_bypass_t plan = {0U, 0U};

    scan_plan_build(points, count, &plan);
    if (scanning())
    {
        obs_state_plan_store(&plan);
        obs_state_flags_clear(OBS_EVT_SCAN_REQ);
        obs_state_flags_set(OBS_EVT_SCAN_DONE);
        tm_printf((UB *)"[OBS] Scan result=%u\n",
                  (UINT)scan_plan_result(&plan));
    }
}

static void front_run (bool bypass)
{
    tk_dly_tsk((RELTIM)servo_set_angle(0));
    int32_t mm = 0;
    ultrasonic_result_t const result = ultrasonic_ping(&mm);
    obs_state_phase_t const phase =
        obs_state_status_phase(obs_state_status_get());

    if (bypass && ((phase < OBS_PHASE_TURN_OUT) ||
                   (phase > OBS_PHASE_VERIFY)))
    {
        return;
    }
    if (!bypass && (phase != OBS_PHASE_IDLE))
    {
        return;
    }
    if (result != ULTRASONIC_OK)
    {
        g_hits = 0U;
        g_front_filled = 0U;
        if (g_silent < OBS_NO_ECHO_LIMIT)
        {
            g_silent++;
        }
        if (bypass || (g_silent >= OBS_NO_ECHO_LIMIT))
        {
            obs_state_faults_update(OBS_FAULT_NO_ECHO, 0U);
        }
        if (bypass)
        {
            obs_state_flags_clear(OBS_EVT_FRONT_READY);
            obs_state_flags_set(OBS_EVT_FRONT_BLOCKED);
        }
        return;
    }
    g_silent = 0U;
    obs_state_faults_update(0U, OBS_FAULT_NO_ECHO);
    obs_state_set_distance_mm(mm);
    if (bypass)
    {
        if (mm < OBS_STOP_MM)
        {
            obs_state_flags_clear(OBS_EVT_FRONT_READY);
            obs_state_flags_set(OBS_EVT_FRONT_BLOCKED);
        }
        else
        {
            obs_state_flags_set(OBS_EVT_FRONT_READY);
        }
    }
    else
    {
        g_front[g_front_next] = mm;
        g_front_next = (g_front_next + 1U) % 3U;
        if (g_front_filled < 3U)
        {
            g_front_filled++;
        }
        if (g_front_filled < 3U)
        {
            return;
        }
        int32_t const median = ultrasonic_median_of_three(
            g_front[0], g_front[1], g_front[2]);

        g_hits = (median < OBS_STOP_MM) ? (g_hits + 1U) : 0U;
        if (g_hits >= OBS_CONFIRM_HITS)
        {
            g_hits = 0U;
            obs_state_set_phase(OBS_PHASE_CONFIRM);
            obs_state_flags_set(OBS_EVT_BLOCKED);
            tm_printf((UB *)"[OBS] Confirmed at %d mm\n", (INT)mm);
        }
    }
}

static void side_run (void)
{
    /* Request a pause before moving the sensor away from the front. */
    obs_state_flags_clear(OBS_EVT_FRONT_READY);
    tk_dly_tsk(OBS_GUARD_WAIT_MSEC + APP_CONTROLLER_PERIOD_MSEC);
    if ((obs_state_flags_read() & OBS_EVT_SIDE_WATCH) == 0U)
    {
        return;
    }
    int32_t const angle = obs_state_watch_side_get() * OBS_SIDE_LOOK_DEG;

    tk_dly_tsk((RELTIM)servo_set_angle(angle));
    int32_t mm = 0;
    ultrasonic_result_t const result = ultrasonic_ping(&mm);

    if ((obs_state_flags_read() & OBS_EVT_SIDE_WATCH) == 0U)
    {
        return;
    }
    if ((result == ULTRASONIC_OK) && (mm < OBS_SIDE_NEAR_MM))
    {
        g_side_seen = true;
        g_side_far = 0U;
        obs_state_flags_set(OBS_EVT_SIDE_SEEN);
    }
    else if ((result == ULTRASONIC_OK) && g_side_seen)
    {
        g_side_far++;
        if (g_side_far >= OBS_SIDE_CLEAR_HITS)
        {
            obs_state_flags_set(OBS_EVT_SIDE_CLEAR);
        }
    }
    else
    {
        g_side_far = 0U;
    }
}

/*** end of file ***/

