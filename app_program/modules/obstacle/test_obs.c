/** @file test_obs.c
 *
 * @brief Non-mutating tests. Set OBS_TEST_HW to 1 and APP_OBSTACLE_ENABLED
 * to 0 for the sensor and servo bring-up, so the task does not share the
 * sensor.
 */
#include "test_obs.h"
#include "obs_config.h"
#include "obs_state.h"
#include "scan_plan.h"
#include "servo.h"
#include "ultrasonic.h"
#include <stdbool.h>
#include <tm/tmonitor.h>

#define OBS_TEST_HW 0
#define TEST_PLAN_FAULTS 0x15U
#define TEST_PLAN_COUNT 9U
#define TEST_PLAN_DIST_CM 123U
#define TEST_PLAN_WIDTH_CM 77U

/** @brief Print one PASS or FAIL line. */
static void report (char const * p_name, bool b_pass);
/** @brief Fill a profile with one range at every scan angle. */
static void fill (scan_plan_point_t * p_points, int32_t range_mm);

void test_obs_run (void)
{
    scan_plan_point_t  points[OBS_SCAN_POINTS];
    scan_plan_bypass_t plan = {0U, 0U};
    uint32_t const     word = obs_state_status_pack(
        OBS_PHASE_SEARCH_LINE, OBS_ROUTE_RIGHT, TEST_PLAN_FAULTS,
        TEST_PLAN_COUNT, TEST_PLAN_DIST_CM, TEST_PLAN_WIDTH_CM);

    report("pack round trip",
           (OBS_PHASE_SEARCH_LINE == obs_state_status_phase(word)) &&
               (OBS_ROUTE_RIGHT == obs_state_status_route(word)) &&
               (TEST_PLAN_FAULTS == obs_state_status_faults(word)) &&
               (TEST_PLAN_COUNT == obs_state_status_count(word)) &&
               (TEST_PLAN_DIST_CM == obs_state_status_distance_cm(word)) &&
               (TEST_PLAN_WIDTH_CM == obs_state_status_width_cm(word)));

    fill(points, 1500);
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    report("empty scan is clear", SCAN_PLAN_CLEAR == scan_plan_result(&plan));

    fill(points, 1500);
    points[6].range_mm = 254; /* -10 deg */
    points[7].range_mm = 250; /*   0 deg */
    points[8].range_mm = 254; /* +10 deg */
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    report("narrow object gives a route",
           SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    /* Values are non-negative; UINT matches %u on the 32-bit target. */
    tm_printf((UB *)"[TEST OBS] width=%u lateral=%u diag=%u left=%u\n",
              (UINT)scan_plan_width_cm(&plan),
              (UINT)scan_plan_lateral_cm(&plan), (UINT)scan_plan_diag_cm(&plan),
              (UINT)scan_plan_side_is_left(&plan));

    fill(points, 200);
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    report("wall gives no route",
           SCAN_PLAN_NO_ROUTE == scan_plan_result(&plan));

#if OBS_TEST_HW
    ultrasonic_init();
    servo_init();

    for (uint32_t index = 0U; index < 20U; index++)
    {
        int32_t                   range_mm = 0;
        ultrasonic_result_t const result   = ultrasonic_ping(&range_mm);

        tm_printf((UB *)"[TEST OBS] ping %u: res=%u mm=%d\n", (UINT)index,
                  (UINT)result, (INT)range_mm);
        tk_dly_tsk(OBS_PING_PERIOD_MSEC);
    }

    for (int32_t angle = -OBS_SERVO_LIMIT_DEG; angle <= OBS_SERVO_LIMIT_DEG;
         angle += 20)
    {
        tk_dly_tsk((RELTIM)servo_set_angle(angle));
        tm_printf((UB *)"[TEST OBS] servo angle %d\n", (INT)angle);
        tk_dly_tsk(500);
    }

    servo_set_angle(0);
#endif
}

static void report (char const * p_name, bool b_pass)
{
    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[TEST OBS] %s: %s\n", (UB *)p_name,
              b_pass ? (UB *)"PASS" : (UB *)"FAIL");
}

static void fill (scan_plan_point_t * p_points, int32_t range_mm)
{
    int32_t index = 0;

    for (int32_t angle = -OBS_SCAN_LIMIT_DEG; angle <= OBS_SCAN_LIMIT_DEG;
         angle += OBS_SCAN_STEP_DEG)
    {
        p_points[index].angle_deg = angle;
        p_points[index].range_mm  = range_mm;
        index++;
    }
}

/*** end of file ***/

