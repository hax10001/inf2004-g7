/** @file task_obstacle.c
 *
 * @brief Front monitor, panning scan and side watch; sole owner of the
 * ultrasonic sensor and servo.
 */
#include "task_obstacle.h"
#include "obs_config.h"
#include "obs_state.h"
#include "obstacle.h"
#include "obs_geom.h"
#include "scan_plan.h"
#include "servo.h"
#include "ultrasonic.h"
#include <stdbool.h>
#include <tm/tmonitor.h>

#define MONITOR_WINDOW 3U
#define MONITOR_RANGE_BITS 9U
#define MONITOR_RANGE_MASK 0x1FFU

typedef struct
{
    uint32_t history; /**< Three nine-bit ranges, in centimetres. */
    uint32_t next : 2;
    uint32_t filled : 2;
    uint32_t hits : 2;
    uint32_t no_echo : 3;
} monitor_t;

typedef struct
{
    uint32_t far_hits : 2;
    uint32_t phase : 4;
    uint32_t b_repeat : 1;
    uint32_t next_view : 2;
    int32_t angle_deg : 8;
} side_watch_t;

/** @brief Ping the front and raise OBS_EVT_BLOCKED on a confirmed hit. */
static void run_monitor (monitor_t * p_monitor);
/** @brief Judge the filtered front range once the window is full. */
static void judge_front (monitor_t * p_monitor);
/** @brief Check forward travel during bypass; unknown data cannot clear it. */
static void sample_guard (side_watch_t * p_watch);
/** @brief Coarse then fine sweep and publish a safe profile. */
static void run_scan (void);
/** @brief Cancel everything and park the servo. */
static void abort_all (monitor_t * p_monitor, side_watch_t * p_watch);
/** @brief Median of OBS_SCAN_PINGS pings; invalid majority stays invalid. */
static int32_t scan_range (void);

void task_obstacle_task (INT start_code, void * p_context)
{
    monitor_t    monitor = {0U, 0U, 0U, 0U, 0U};
    side_watch_t watch   = {0U, OBS_PHASE_IDLE, false, 0U, 0};

    /* Casting unused arguments to void cannot alter their values. */
    (void)start_code;
    (void)p_context;

    tk_dly_tsk(servo_set_angle(0));
    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[OBS] Task started\n");

    for (;;)
    {
        UINT const pattern = obs_state_flags_read();

        if (0U != (pattern & OBS_EVT_ABORT))
        {
            abort_all(&monitor, &watch);
        }
        else if (0U != (pattern & OBS_EVT_SCAN_REQ))
        {
            run_scan();
            watch.angle_deg = 0;
        }
        else if (0U == (pattern & OBS_EVT_BLOCKED))
        {
            if (0 != watch.angle_deg)
            {
                tk_dly_tsk(servo_set_angle(0));
                watch.angle_deg = 0;
            }

            run_monitor(&monitor);
        }
        else
        {
            obs_state_phase_t const phase =
                obs_state_status_phase(obs_state_status_get());

            if ((phase >= OBS_PHASE_TURN_OUT) &&
                (phase <= OBS_PHASE_VERIFY))
            {
                sample_guard(&watch);
            }
        }

        tk_dly_tsk(OBS_PING_PERIOD_MSEC);
    }
}

static void run_monitor (monitor_t * p_monitor)
{
    int32_t                   range_mm = 0;
    ultrasonic_result_t const result   = ultrasonic_ping(&range_mm);

    if (!obstacle_monitor_ready())
    {
        p_monitor->hits = 0U;
        p_monitor->filled = 0U;
        return;
    }

    if ((ULTRASONIC_OK != result) || (range_mm < 20))
    {
        obs_state_front_mark(false, 0U);
        p_monitor->hits = 0U;
        p_monitor->filled = 0U;
        p_monitor->next = 0U;

        if (p_monitor->no_echo < OBS_NO_ECHO_LIMIT)
        {
            p_monitor->no_echo++;
        }

    }
    else
    {
        p_monitor->no_echo = 0U;

        SYSTIM now = {0};

        if (tk_get_otm(&now) < E_OK)
        {
            obs_state_front_mark(false, 0U);
            obs_state_faults_update(OBS_FAULT_TIMEOUT, 0U);
            return;
        }

        obs_state_front_mark(true, now.lo);

        uint32_t const shift = p_monitor->next * MONITOR_RANGE_BITS;
        /* Round up so 251 mm cannot be filtered into the 250 mm trigger. */
        uint32_t const range_cm = ((uint32_t)range_mm + 9U) / 10U;
        p_monitor->history =
            (p_monitor->history & ~(MONITOR_RANGE_MASK << shift)) |
            ((range_cm & MONITOR_RANGE_MASK) << shift);
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
        (int32_t)(p_monitor->history & MONITOR_RANGE_MASK) * 10,
        (int32_t)((p_monitor->history >> MONITOR_RANGE_BITS) &
                  MONITOR_RANGE_MASK) * 10,
        (int32_t)((p_monitor->history >> (2U * MONITOR_RANGE_BITS)) &
                  MONITOR_RANGE_MASK) * 10);

    obs_state_set_distance_mm(median);

    if (median <= OBS_STOP_MM)
    {
        p_monitor->hits++;
    }
    else
    {
        p_monitor->hits = 0U;
    }

    if (p_monitor->hits >= OBS_CONFIRM_HITS)
    {
        obs_state_front_mark(false, 0U);
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
    bool found = false;
    bool valid = true;
    uint32_t scanned = 0U;
    uint32_t focus = OBS_SCAN_POINTS / 2U;
    int32_t nearest = OBS_SCAN_RANGE_MM;

    tm_printf((UB *)"[OBS] coarse scan\n");

    /* Coarse 30 degree steps: -60, -30, 0, +30 and +60. */
    for (uint32_t index = 1U; index < OBS_SCAN_POINTS; index += 3U)
    {
        int32_t const angle = -OBS_SCAN_LIMIT_DEG +
                              ((int32_t)index * OBS_SCAN_STEP_DEG);
        int32_t range = -1;

        if (0U != (obs_state_flags_read() & OBS_EVT_ABORT))
        {
            valid = false;
            break;
        }

        tk_dly_tsk(servo_set_angle(angle));
        range = scan_range();
        scan_plan_point_set(&points[index], angle, range);
        scanned |= (1U << index);
        found = found || ((range >= 20) && (range < OBS_SCAN_RANGE_MM));
        valid = valid && (range >= 20);

        if ((range >= 20) && (range < nearest))
        {
            nearest = range;
            focus = index;
        }
    }

    /* The front trigger was previously confirmed. Its disappearance must
     * be reconfirmed rather than treating missing readings as open space.
     */
    if (valid && found)
    {
        /* Return to the nearest obstacle first, then fill corridor samples.
         * The complete grid keeps unsampled side space from becoming CLEAR.
         */
        uint32_t const first = (focus > 0U) ? focus - 1U : 0U;

        tm_printf((UB *)"[OBS] fine scan near index=%u\n", (UINT)focus);

        for (uint32_t offset = 0U; offset < OBS_SCAN_POINTS; offset++)
        {
            uint32_t const index = (first + offset) % OBS_SCAN_POINTS;

            if (0U == (scanned & (1U << index)))
            {
                if (0U != (obs_state_flags_read() & OBS_EVT_ABORT))
                {
                    valid = false;
                    break;
                }

                int32_t const angle = -OBS_SCAN_LIMIT_DEG +
                                      ((int32_t)index * OBS_SCAN_STEP_DEG);
                tk_dly_tsk(servo_set_angle(angle));
                int32_t const range = scan_range();
                scan_plan_point_set(&points[index], angle, range);
                valid = valid && (range >= 20);
            }
        }
    }

    tk_dly_tsk(servo_set_angle(0));

    if (0U != (obs_state_flags_read() & OBS_EVT_ABORT))
    {
        return;
    }

    scan_plan_bypass_t plan = {0U, 0U};

    if (valid)
    {
        /* No front obstacle confirmed: a second front reading is required. */
        if (!found)
        {
            int32_t const confirm_mm = scan_range();
            valid = (confirm_mm >= OBS_SCAN_RANGE_MM);
        }

        if (valid && found)
        {
            scan_plan_build(points, OBS_SCAN_POINTS, &plan);
        }
    }

    if (!valid)
    {
        /* Indeterminate sensor data cannot authorise forward motion. */
        scan_plan_build(NULL, 0, &plan);
    }

    if (!obs_state_plan_store(&plan))
    {
        obs_state_faults_update(OBS_FAULT_TIMEOUT, 0U);
        obs_state_flags_set(OBS_EVT_ABORT);
        return;
    }

    /* Retire this request before publishing completion. On SMP, the
     * controller may immediately request its one rescan after DONE.
     */
    obs_state_flags_clear(OBS_EVT_SCAN_REQ);
    obs_state_flags_set(OBS_EVT_SCAN_DONE);
    tm_printf((UB *)"[OBS] coarse/fine result=%u width=%u near=%u\n",
              (UINT)scan_plan_result(&plan),
              (UINT)scan_plan_width_cm(&plan),
              (UINT)scan_plan_near_cm(&plan));
    tm_printf((UB *)"[OBS] profile left=%u right=%u direction=%d\n",
              (UINT)scan_plan_left_clear_cm(&plan),
              (UINT)scan_plan_right_clear_cm(&plan),
              (INT)scan_plan_near_direction(&plan));
}

static void sample_guard (side_watch_t * p_watch)
{
    obs_guard_snapshot_t snapshot = {0U, 0U};
    obs_state_phase_t const phase =
        obs_state_status_phase(obs_state_status_get());

    if (!obs_state_guard_load(&snapshot))
    {
        obs_state_faults_update(OBS_FAULT_TIMEOUT, 0U);
        return;
    }

    uint32_t const epoch = snapshot.clock & 0xFFU;
    uint32_t const valid = (snapshot.ranges >> 27U) & 7U;
    uint32_t view = 0U;

    if ((0U == epoch) || (0U != (snapshot.ranges >> 30U)))
    {
        return;
    }

    if (valid == OBS_GUARD_ALL_VIEWS)
    {
        /* Repeat the last view once without panning. This live sample
         * occurs during the small movement authorised by full coverage.
         */
        view = p_watch->b_repeat ? OBS_GUARD_OUTER : p_watch->next_view;
        p_watch->b_repeat = false;
    }
    else
    {
        while ((view < 2U) && (0U != (valid & (1U << view))))
        {
            view++;
        }
    }

    int32_t const inner = obs_state_watch_side_get() * OBS_SIDE_LOOK_DEG;
    int32_t const angle = (OBS_GUARD_FRONT == view) ? 0 :
                         ((OBS_GUARD_INNER == view) ? inner : -inner);

    if (p_watch->angle_deg != angle)
    {
        tk_dly_tsk(servo_set_angle(angle));
        p_watch->angle_deg = angle;
    }

    int32_t range_mm = 0;
    ultrasonic_result_t const result = ultrasonic_ping(&range_mm);
    uint32_t reason = OBS_GUARD_REASON_NONE;

    if ((ULTRASONIC_OK != result) || (range_mm < 20))
    {
        reason = OBS_GUARD_REASON_SENSOR;
    }
    else if (range_mm <= obs_geom_clearance_mm(angle))
    {
        reason = OBS_GUARD_REASON_CLOSE;
    }

    if (!obs_state_guard_sample(epoch, view, range_mm, reason))
    {
        /* A changed epoch means this measurement belongs to an old pose. */
        return;
    }

    if ((OBS_GUARD_FRONT == view) && (OBS_GUARD_REASON_NONE == reason))
    {
        obs_state_set_distance_mm(range_mm);
    }

    if (p_watch->phase != (uint32_t)phase)
    {
        p_watch->far_hits = 0U;
        p_watch->phase = phase;
    }

    if ((OBS_GUARD_INNER == view) &&
        ((OBS_PHASE_PASS_SIDE == phase) || (OBS_PHASE_PASS_END == phase)))
    {
        if ((0U != reason) || (range_mm < OBS_SIDE_NEAR_MM))
        {
            p_watch->far_hits = 0U;
            obs_state_flags_clear(OBS_EVT_SIDE_CLEAR);

            if (0U == reason)
            {
                obs_state_flags_set(OBS_EVT_SIDE_SEEN);
            }
        }
        else if (0U != (obs_state_flags_read() & OBS_EVT_SIDE_SEEN))
        {
            if (p_watch->far_hits < OBS_SIDE_CLEAR_HITS)
            {
                p_watch->far_hits++;
            }

            if (p_watch->far_hits >= OBS_SIDE_CLEAR_HITS)
            {
                obs_state_flags_set(OBS_EVT_SIDE_CLEAR);
            }
        }
    }

    if ((OBS_GUARD_OUTER == view) && (valid != OBS_GUARD_ALL_VIEWS))
    {
        p_watch->b_repeat = true;
    }

    p_watch->next_view = (view + 1U) % 3U;
}

static void abort_all (monitor_t * p_monitor, side_watch_t * p_watch)
{
    obs_state_flags_clear(OBS_EVT_BLOCKED | OBS_EVT_SCAN_REQ |
                          OBS_EVT_SCAN_DONE | OBS_EVT_SIDE_WATCH |
                          OBS_EVT_SIDE_SEEN | OBS_EVT_SIDE_CLEAR |
                          OBS_EVT_ABORT);
    p_watch->far_hits = 0U;
    p_watch->angle_deg = 0;
    tk_dly_tsk(servo_set_angle(0));
    p_monitor->hits   = 0U;
    p_monitor->filled = 0U;
}

static int32_t scan_range (void)
{
    int16_t values[OBS_SCAN_PINGS];

    for (uint32_t index = 0U; index < OBS_SCAN_PINGS; index++)
    {
        int32_t range_mm = 0;

        values[index] = (int16_t)(
            (ULTRASONIC_OK == ultrasonic_ping(&range_mm)) ? range_mm : -1);
        tk_dly_tsk(OBS_PING_PERIOD_MSEC);
    }

    /* Insertion sort; the middle value is the median. */
    for (uint32_t index = 1U; index < OBS_SCAN_PINGS; index++)
    {
        int16_t const key  = values[index];
        uint32_t      slot = index;

        while ((slot > 0U) && (values[slot - 1U] > key))
        {
            values[slot] = values[slot - 1U];
            slot--;
        }

        values[slot] = key;
    }

    /* A median with two invalid echoes is invalid; do not assume FAR. */
    return (values[OBS_SCAN_PINGS / 2U] >= 20)
               ? values[OBS_SCAN_PINGS / 2U] : -1;
}

/*** end of file ***/

