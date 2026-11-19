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
/* Wider than any bearing: a pass over the whole sweep. */
#define SCAN_ALL_DEG 180
#if (OBS_SCAN_PINGS != 2U) || (OBS_SCAN_AGREE_MM < 0)
#error "Scans take two pings (and a third on disagreement)."
#endif
#if (OBS_CORNER_PINGS < (OBS_SCAN_PINGS + 1U)) || (OBS_CORNER_PINGS > 8U)
#error "OBS_CORNER_PINGS must be 3..8."
#endif

/* Obstacle task only. Ranges 0..4000 mm; index/fill 0..MONITOR_WINDOW;
 * hits 0..OBS_CONFIRM_HITS; no_echo 0..OBS_NO_ECHO_LIMIT. */
typedef struct
{
    int16_t      history[MONITOR_WINDOW];
    unsigned int next:2;
    unsigned int filled:2;
    unsigned int hits:2;
    unsigned int no_echo:3;
} monitor_t;
#if (MONITOR_WINDOW > 3U) || (OBS_CONFIRM_HITS > 3U) || \
    (OBS_NO_ECHO_LIMIT > 7U)
#error Front monitor limits no longer fit their bit-fields
#endif

/* Obstacle task only: pings 0..OBS_GLANCE_EVERY, servo angle +-90. */
typedef struct
{
    bool         b_aimed:1;
    unsigned int pings:3;   /**< Side pings since the last front glance. */
    int8_t       aimed_deg; /**< Angle the servo was last aimed at. */
} side_watch_t;
#if OBS_GLANCE_EVERY > 7U
#error Side-watch glance count no longer fits 3 bits
#endif

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
/** @brief One sweep over the scan bearings.
 *  @param[in] p_points Bearings to sample, ordered right to left.
 *  @param[out] p_ranges_mm Range of each bearing, same order.
 *  @param[in,out] p_lone_mm Nearest lone corner echo of each bearing (see
 *  scan_plan_point_t); a nearer one replaces it, 0 is none.
 *  @param b_left_first Sweep left to right instead of right to left.
 *  @param first_step Steps already sampled at the start of this sweep.
 *  @param max_abs_deg Only bearings within +-this are sampled; the rest
 *  of p_ranges_mm is left as it is.
 *  @return false if the scan was aborted.
 */
static bool scan_pass (scan_plan_point_t const * p_points,
                       int32_t * p_ranges_mm, int32_t * p_lone_mm,
                       int32_t count, bool b_left_first, int32_t first_step,
                       int32_t max_abs_deg);
/** @brief Wait until OBS_PING_PERIOD_MSEC after the last trigger. Spacing
 *  is trigger to trigger, so time spent on the echo or moving the servo
 *  counts towards it instead of being added on top. */
static void scan_ping_gap (void);
/** @brief Absolute difference of two ranges in mm. */
static int32_t abs_diff (int32_t first, int32_t second);
/** @brief Two pings, a third if they differ by more than OBS_SCAN_AGREE_MM;
 *  the median of three or the average of two. Failures count as far. In
 *  the corner band, up to OBS_CORNER_PINGS until two agree on an echo; it
 *  then reads the nearest agreeing pair (see corner_pair()).
 *  @param angle_deg Bearing being sampled; it selects the corner band.
 *  @param[out] p_lone_mm Corner band: the nearest echo no other ping
 *  agreed with, nearer than the reading; else 0.
 */
static int32_t scan_range (int32_t angle_deg, int32_t * p_lone_mm);
/** @brief True for a corner band bearing: beyond +-OBS_SCAN_FINE_DEG, up to
 *  +-OBS_CORNER_MAX_DEG. */
static bool is_corner_bearing (int32_t angle_deg);
/** @brief The nearest two of these pings that agree within
 *  OBS_SCAN_AGREE_MM, averaged; a lone echo is not trusted.
 *  @param[in] p_values Pings in ascending order.
 *  @param[out] p_range_mm The pair's average, if found.
 *  @return true if such a pair exists.
 */
static bool corner_pair (int32_t const * p_values, uint32_t taken,
                         int32_t * p_range_mm);
/** @brief Sort a few pings into ascending order. */
static void sort_pings (int32_t * p_values, uint32_t taken);
/** @brief Debug log: each sample tagged by side and by how the planner saw
 *  it, then each side's edge, need and corridor, before the decision.
 */
static void log_scan_detail (scan_plan_point_t const * p_points,
                             int32_t count, scan_plan_sides_t const * p_sides);
/** @brief Debug log of one side of the scan. */
static void log_side (UB const * p_name, unsigned int need_mm,
                      scan_plan_side_info_t const * p_info);
/** @brief Re-centre the servo if it is not at FRONT; old pings are dropped. */
static void face_front (monitor_t * p_monitor);

void task_obstacle_task (INT start_code, void * p_context)
{
    monitor_t    monitor = {{0, 0, 0}, 0U, 0U, 0U, 0U};
    side_watch_t watch   = {false, 0U, 0};

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

        /* 0..4000 mm fits int16_t. */
        p_monitor->history[p_monitor->next] = (int16_t)range_mm;
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
    int32_t           first_mm[OBS_SCAN_POINTS];
    int32_t           second_mm[OBS_SCAN_POINTS];
    int32_t           lone_mm[OBS_SCAN_POINTS];
    int32_t           count = 0;
    int32_t           centre_mm = OBS_FAR_MM;
    bool              b_ok  = true;
    /* Bearings the servo can reach; the rest of the sequence is skipped,
     * so every sample is taken where it is labelled. */
    int32_t const right_deg =
        SCAN_MIN(OBS_SCAN_RIGHT_DEG, servo_right_limit_deg());
    int32_t const left_deg =
        SCAN_MIN(OBS_SCAN_LEFT_DEG, servo_left_limit_deg());

    for (int32_t angle = -OBS_SCAN_RIGHT_DEG;
         (angle <= OBS_SCAN_LEFT_DEG) && (count < OBS_SCAN_POINTS);
         angle = scan_plan_next_angle(angle))
    {
        if ((angle >= -right_deg) && (angle <= left_deg))
        {
            points[count].angle_deg = angle;
            points[count].range_mm  = OBS_FAR_MM;
            points[count].lone_mm   = 0U;
            first_mm[count]         = OBS_FAR_MM;
            second_mm[count]        = OBS_FAR_MM;
            lone_mm[count]          = 0;
            count++;
        }
    }

    if ((0 != OBS_FRESH_SCAN_CENTRE) && (count > 0) &&
        (0U == (obs_state_flags_read() & OBS_EVT_ABORT)))
    {
        int32_t lone = 0;
        tk_dly_tsk((RELTIM)servo_set_angle(0));
        centre_mm = scan_range(0, &lone);

        if (0 != OBS_DEBUG_LOG)
        {
            /* The fixed servo pulse defines zero across both passes. */
            tm_printf((UB *)"[OBS] Scan zero: %d usec, range %d mm\n",
                      (INT)OBS_SERVO_FRONT_USEC, (INT)centre_mm);
        }
    }

    /* The first pass runs the configured way (both ways: left first);
     * both-ways mode adds the opposite pass and keeps the nearer reading of
     * each bearing. */
    b_ok = scan_pass(points, first_mm, lone_mm, count,
                     (OBS_SCAN_DIRECTION <= 0), 0, SCAN_ALL_DEG);

    if (b_ok && (0 == OBS_SCAN_DIRECTION) && (count > 0))
    {
        /* The return pass, right to left, re-reads only the central
         * bearings; the others keep the first pass's reading. If it reaches
         * the bearing the first pass ended on, that one is already read from
         * the same side, so it is reused rather than pinged again without
         * moving. */
        for (int32_t index = 0; index < count; index++)
        {
            second_mm[index] = first_mm[index];
        }
        b_ok = scan_pass(points, second_mm, lone_mm, count, false, 1,
                         OBS_SCAN_RETURN_DEG);
    }

    for (int32_t index = 0; (index < count) && b_ok; index++)
    {
        /* The nearer reading: an echo seen on either pass is kept, so no
         * object is lost, and the two sweep-direction errors cancel. */
        int32_t const sweep_mm = (second_mm[index] < first_mm[index])
                                     ? second_mm[index] : first_mm[index];
        int32_t const range_mm = ((0 == points[index].angle_deg) &&
                                  (centre_mm < OBS_SCAN_RANGE_MM))
                                     ? centre_mm : sweep_mm;

        /* Readings are 0..4000 mm and lone echoes below that, so the
         * 12-bit fields hold them. */
        points[index].range_mm = (unsigned int)range_mm;
        points[index].lone_mm  = (unsigned int)lone_mm[index];
    }

    if (b_ok && (0 == OBS_SCAN_DIRECTION) && (0 != OBS_DEBUG_LOG))
    {
        for (int32_t index = 0; index < count; index++)
        {
            /* Values fit INT on the target; UB preserves the bytes. */
            tm_printf((UB *)"[OBS] Pass readings %d deg: L->R %d, R->L %d "
                            "mm\n",
                      (INT)points[index].angle_deg, (INT)first_mm[index],
                      (INT)second_mm[index]);
        }
    }

    if (b_ok)
    {
        scan_plan_bypass_t plan = {0U, 0U};

        tk_dly_tsk((RELTIM)servo_set_angle(0));

        /* An uncertain centre echo does not clear a previously
         * confirmed obstacle in front of the car.
         */
        uint32_t const last_cm =
            obs_state_status_distance_cm(obs_state_status_get());
        int32_t const last_mm = (int32_t)last_cm * 10;

        /* Mid-bypass the route is already set: keep to that side. */
        obs_state_route_t const route =
            obs_state_status_route(obs_state_status_get());
        int32_t const required_side =
            (OBS_ROUTE_LEFT == route)    ? 1
            : (OBS_ROUTE_RIGHT == route) ? -1
                                         : 0;

        scan_plan_sides_t sides = {0};

        scan_plan_build_side_report(points, count, last_mm, required_side,
                                    &plan, &sides);
        obs_state_plan_store(&plan);
        log_scan_detail(points, count, &sides);

        if ((0 != OBS_DEBUG_LOG) && (0U != sides.b_object))
        {
            /* Fields are at most 12 bits, so INT holds them exactly; UB
             * preserves the string bytes. */
            tm_printf((UB *)"[OBS] Side need L=%d R=%d mm, clear L=%d R=%d, "
                            "%s -> %s\n",
                      (INT)sides.left_need_mm, (INT)sides.right_need_mm,
                      (INT)sides.b_left_ok, (INT)sides.b_right_ok,
                      (0 != required_side)        ? (UB *)"side kept"
                      : (0 != OBS_PREFERRED_SIDE) ? (UB *)"preferred side"
                      : (0U != sides.b_tie)
                          ? (UB *)"close comparison"
                          : (UB *)"smaller move",
                      (SCAN_PLAN_ROUTE != scan_plan_result(&plan))
                          ? (UB *)"no route"
                      : scan_plan_side_is_left(&plan) ? (UB *)"left"
                                                      : (UB *)"right");
        }
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
    if ((0 != OBS_PASS_FRONT_GLANCE) &&
        (0U != (pattern & OBS_EVT_FRONT_WATCH)) &&
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

        int32_t const want_deg = obs_state_watch_angle_get();

        /* The sequencer moves the aim while turning out round the object. */
        if ((!p_watch->b_aimed) || (want_deg != p_watch->aimed_deg))
        {
            tk_dly_tsk((RELTIM)servo_set_angle(want_deg));
            p_watch->b_aimed   = true;
            /* Servo angles are within +-90 degrees. */
            p_watch->aimed_deg = (int8_t)want_deg;
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

static bool scan_pass (scan_plan_point_t const * p_points,
                       int32_t * p_ranges_mm, int32_t * p_lone_mm,
                       int32_t count, bool b_left_first, int32_t first_step,
                       int32_t max_abs_deg)
{
    bool b_ok = true;

    if (0 != OBS_DEBUG_LOG)
    {
        /* UB views the same string bytes; tm_printf does not modify. */
        tm_printf((UB *)"[OBS] Scan pass %s\n",
                  b_left_first ? (UB *)"left -> right"
                               : (UB *)"right -> left");
    }

    for (int32_t step = first_step; (step < count) && b_ok; step++)
    {
        int32_t const index = b_left_first ? (count - 1 - step) : step;
        int32_t const angle = p_points[index].angle_deg;

        if ((angle > max_abs_deg) || (angle < -max_abs_deg))
        {
            /* Outside this pass's range: keep the existing reading. */
        }
        else if (0U != (obs_state_flags_read() & OBS_EVT_ABORT))
        {
            b_ok = false; /* The main loop handles the abort. */
        }
        else
        {
            int32_t lone = 0;

            tk_dly_tsk((RELTIM)servo_set_angle(angle));
            p_ranges_mm[index] = scan_range(angle, &lone);

            if ((0 != lone) &&
                ((0 == p_lone_mm[index]) || (lone < p_lone_mm[index])))
            {
                p_lone_mm[index] = lone;
            }
        }
    }

    return b_ok;
}

static int32_t abs_diff (int32_t first, int32_t second)
{
    return (first > second) ? (first - second) : (second - first);
}

static void scan_ping_gap (void)
{
    uint32_t last_msec = 0U;
    SYSTIM   now       = {0};

    if (!ultrasonic_last_trigger(&last_msec))
    {
        /* First ping ever: nothing to space from. */
        return;
    }

    if (tk_get_otm(&now) < E_OK)
    {
        /* Clock unreadable: fall back to the full period. */
        tk_dly_tsk(OBS_PING_PERIOD_MSEC);
        return;
    }

    /* Modulo-2^32 milliseconds, like every other elapsed time here. */
    uint32_t const since = now.lo - last_msec;

    if (since < (uint32_t)OBS_PING_PERIOD_MSEC)
    {
        tk_dly_tsk((RELTIM)((uint32_t)OBS_PING_PERIOD_MSEC - since));
    }
}

static bool is_corner_bearing (int32_t angle_deg)
{
    int32_t const offset = (angle_deg < 0) ? -angle_deg : angle_deg;

    return (offset > OBS_SCAN_FINE_DEG) && (offset <= OBS_CORNER_MAX_DEG);
}

static void sort_pings (int32_t * p_values, uint32_t taken)
{
    /* Insertion sort: at most OBS_CORNER_PINGS values. */
    for (uint32_t index = 1U; index < taken; index++)
    {
        int32_t const key  = p_values[index];
        uint32_t      slot = index;

        while ((slot > 0U) && (p_values[slot - 1U] > key))
        {
            p_values[slot] = p_values[slot - 1U];
            slot--;
        }

        p_values[slot] = key;
    }
}

static bool corner_pair (int32_t const * p_values, uint32_t taken,
                         int32_t * p_range_mm)
{
    /* Ascending, so the first agreeing neighbours are the nearest pair. */
    for (uint32_t index = 1U; index < taken; index++)
    {
        if (abs_diff(p_values[index - 1U], p_values[index]) <=
            OBS_SCAN_AGREE_MM)
        {
            *p_range_mm = (p_values[index - 1U] + p_values[index]) / 2;
            return true;
        }
    }

    return false;
}

static int32_t scan_range (int32_t angle_deg, int32_t * p_lone_mm)
{
    /* Two pings, and a third only if they disagree. In the corner band,
     * more until two agree on an echo. Kept sorted. */
    bool const     b_corner = is_corner_bearing(angle_deg);
    uint32_t const limit    = b_corner ? OBS_CORNER_PINGS
                                       : (OBS_SCAN_PINGS + 1U);
    int32_t        values[OBS_CORNER_PINGS];
    uint32_t       taken    = 0U;
    int32_t        pair_mm  = OBS_FAR_MM;
    bool           b_pair   = false;

    for (uint32_t index = 0U; index < limit; index++)
    {
        int32_t range_mm = 0;

        if (index >= OBS_SCAN_PINGS)
        {
            bool b_done = false;

            if (b_corner)
            {
                /* Two misses agree too, but a corner may still answer. */
                b_pair = corner_pair(values, taken, &pair_mm);
                b_done = b_pair && (pair_mm < OBS_FAR_MM);
            }
            else
            {
                b_done = abs_diff(values[0], values[1]) <= OBS_SCAN_AGREE_MM;
            }

            if (b_done)
            {
                /* Two agree: no more pings needed. */
                break;
            }
        }

        /* No wait after the last ping: the servo moves and settles for
         * the next bearing while the echo dies away. */
        scan_ping_gap();
        values[index] = (ULTRASONIC_OK == ultrasonic_ping(&range_mm))
                            ? range_mm
                            : OBS_FAR_MM;
        taken++;
        sort_pings(values, taken);
    }

    int32_t range_mm = 0;

    if (b_corner)
    {
        /* The nearest agreeing pair: a weak corner echo that only some
         * pings catch is kept, a lone stray is not. No pair: the upper
         * median, as a miss is far more likely than a stray echo. */
        b_pair   = corner_pair(values, taken, &pair_mm);
        range_mm = b_pair ? pair_mm : values[taken / 2U];
    }
    else
    {
        /* Three: the median, so one stray or one missed echo cannot
         * decide. Two that agree: their average. */
        range_mm = (taken > OBS_SCAN_PINGS) ? values[1]
                                            : ((values[0] + values[1]) / 2);
    }

    /* Sorted, so the first ping is the nearest; it is lone if it is not
     * part of the reading. Kept apart: only the corner check uses it. */
    *p_lone_mm = (b_corner && (values[0] < OBS_FAR_MM) &&
                  (values[0] < (range_mm - OBS_SCAN_AGREE_MM)))
                     ? values[0]
                     : 0;

    if (0 != OBS_DEBUG_LOG)
    {
        /* Sorted pings, so a stray echo shows up at either end. Values fit
         * INT on the 32-bit target; UB preserves the string bytes. */
        tm_printf((UB *)"[OBS] Ping %d deg (%s):",
                  (INT)angle_deg,
                  (angle_deg > 0)   ? (UB *)"L"
                  : (angle_deg < 0) ? (UB *)"R"
                                    : (UB *)"front");

        for (uint32_t index = 0U; index < taken; index++)
        {
            tm_printf((UB *)" %d", (INT)values[index]);
        }

        tm_printf((UB *)" -> %d mm", (INT)range_mm);

        if (0 != *p_lone_mm)
        {
            /* UB preserves the string bytes. */
            tm_printf((UB *)" (lone %d)", (INT)*p_lone_mm);
        }

        tm_printf((UB *)"\n");
    }

    return range_mm;
}

static void log_scan_detail (scan_plan_point_t const * p_points,
                             int32_t count, scan_plan_sides_t const * p_sides)
{
    for (int32_t index = 0; index < count; index++)
    {
        int32_t const angle = p_points[index].angle_deg;
        bool const    b_object =
            (0U != p_sides->b_object) && (angle >= p_sides->low_deg) &&
            (angle <= p_sides->high_deg);
        bool const b_near = p_points[index].range_mm < OBS_SCAN_RANGE_MM;

        /* Values fit INT on the 32-bit target; UB preserves bytes. */
        tm_printf((UB *)"[OBS] Scan point %d deg (%s): %d mm %s\n",
                  (INT)angle,
                  (angle > 0)   ? (UB *)"L"
                  : (angle < 0) ? (UB *)"R"
                                : (UB *)"front",
                  (INT)p_points[index].range_mm,
                  b_object ? (UB *)"object"
                  : b_near ? (UB *)"near (other object?)"
                           : (UB *)"open");
    }

    if ((0 != OBS_DEBUG_LOG) && (0U != p_sides->b_object))
    {
        /* Fields are at most 12 bits, so INT holds them; UB preserves the
         * string bytes. */
        tm_printf((UB *)"[OBS] Object %d..%d deg, face %d mm ahead, "
                        "width %d mm (near < %d mm)\n",
                  (INT)p_sides->low_deg, (INT)p_sides->high_deg,
                  (INT)p_sides->near_mm, (INT)p_sides->width_mm,
                  (INT)OBS_SCAN_RANGE_MM);
        if (0U != p_sides->front_width_mm)
        {
            /* Each share is at most the projected width. */
            tm_printf((UB *)"[OBS] Front split: width=%d mm left=%d mm "
                            "right=%d mm%s\n",
                      (INT)p_sides->front_width_mm,
                      (INT)p_sides->front_left_mm,
                      (INT)(p_sides->front_width_mm -
                            p_sides->front_left_mm),
                      (0U != p_sides->b_front_estimate)
                          ? (UB *)" (echo estimate)" : (UB *)"");
        }
        log_side((UB *)"Left ", p_sides->left_need_mm, &p_sides->left);
        log_side((UB *)"Right", p_sides->right_need_mm, &p_sides->right);
        tm_printf((UB *)"[OBS] Measured need L-R = %d mm "
                        "(tie band %d mm)\n",
                  (INT)p_sides->left.compare_mm -
                      (INT)p_sides->right.compare_mm,
                  (INT)OBS_SIDE_PREFERENCE_THRESHOLD_MM);

        if ((0U != p_sides->left.b_seen) && (0U != p_sides->right.b_seen))
        {
            /* Half degrees, so print whole and .5; UB preserves bytes. */
            int32_t const tilt = p_sides->tilt_x2;
            int32_t const mag  = (tilt < 0) ? -tilt : tilt;

            tm_printf((UB *)"[OBS] Square to the face at %s%d.%d deg (+ = "
                            "left); corners count from %d deg off it\n",
                      (tilt < 0) ? (UB *)"-" : (UB *)"", (INT)(mag / 2),
                      (INT)((mag % 2) * 5), (INT)OBS_CORNER_FROM_DEG);
        }

        if (0U != p_sides->b_centre)
        {
            /* Fields are at most 12 bits, so INT holds them; UB preserves
             * the string bytes. A corner of -1 means not seen. */
            tm_printf((UB *)"[OBS] Corner echo L=%d R=%d mm (-1 none): "
                            "object centre %d mm (+ = left), sides chosen "
                            "on it\n",
                      (0U != p_sides->left.b_corner)
                          ? (INT)p_sides->left.corner_mm
                          : (INT)-1,
                      (0U != p_sides->right.b_corner)
                          ? (INT)p_sides->right.corner_mm
                          : (INT)-1,
                      (INT)p_sides->centre_mm);
        }
        else if ((0U != p_sides->left.b_seen) &&
                 (0U != p_sides->right.b_seen))
        {
            /* UB preserves the string bytes. */
            tm_printf((UB *)"[OBS] No confirmed corner echo\n");
        }

        if ((0U != p_sides->left.b_seen) && (0U != p_sides->right.b_seen))
        {
            /* The echo span midpoint is separate from fixed servo zero. */
            int32_t const centre_x10 =
                (p_sides->low_deg + p_sides->high_deg) * 5;
            int32_t const mag  = (centre_x10 < 0) ? -centre_x10 : centre_x10;

            tm_printf((UB *)"[OBS] Echo span midpoint %s%d.%d deg (+ left)\n",
                      (centre_x10 < 0) ? (UB *)"-" : (UB *)"",
                      (INT)(mag / 10), (INT)(mag % 10));
        }
    }
}

static void log_side (UB const * p_name, unsigned int need_mm,
                      scan_plan_side_info_t const * p_info)
{
    static UB const * const reasons[] = {
        (UB *)"edge + half body + margin",
        (UB *)"half body + margin minimum",
        (UB *)"edge beyond sweep: OBS_LATERAL_UNKNOWN_MM",
        (UB *)"far-off edge: OBS_MIN_LATERAL_MM"};

    /* Fields are at most 12 bits, so INT holds them; UB preserves bytes. */
    tm_printf((UB *)"[OBS] %s: edge %d deg%s, %d mm out; need %d mm (%s); "
                    "turn %d deg; open %d mm; ",
              p_name, (INT)p_info->edge_deg,
              (0U != p_info->b_seen) ? (UB *)"" : (UB *)" (sweep end)",
              (INT)p_info->edge_mm, (INT)need_mm,
              reasons[p_info->reason & 0x3U],
              (INT)scan_plan_angle_deg(p_info->angle_index),
              (INT)p_info->open_mm);

    if (p_info->compare_mm != need_mm)
    {
        /* The floor sets the drive only; the sides compare measured. */
        tm_printf((UB *)"compared as %d mm; ", (INT)p_info->compare_mm);
    }

    if (0U != p_info->b_blocked)
    {
        tm_printf((UB *)"BLOCKED by %d deg at %d mm\n",
                  (INT)p_info->blocker_deg, (INT)p_info->blocker_mm);
    }
    else
    {
        tm_printf((UB *)"corridor clear\n");
    }
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

