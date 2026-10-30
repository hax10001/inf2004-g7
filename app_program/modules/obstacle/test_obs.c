/** @file test_obs.c
 *
 * @brief Non-mutating profile and packed-field tests. These never share
 * servo or sonar hardware with the live obstacle task.
 */
#include "test_obs.h"
#include "obs_config.h"
#include "obs_state.h"
#include "scan_plan.h"
#include "servo.h"
#include "ultrasonic.h"
#include <stdbool.h>
#include <tm/tmonitor.h>

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

    fill(points, 2000);
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    report("scan respects calibration",
           ((0 != OBS_GEOMETRY_VERIFIED) ? SCAN_PLAN_CLEAR : SCAN_PLAN_NO_ROUTE)
               == scan_plan_result(&plan));

    fill(points, 2000);
    scan_plan_point_set(&points[6], -10, 554);
    scan_plan_point_set(&points[7], 0, 550);
    scan_plan_point_set(&points[8], 10, 554);
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    report("narrow object needs calibrated clearance",
           ((0 != OBS_GEOMETRY_VERIFIED) ? SCAN_PLAN_ROUTE : SCAN_PLAN_NO_ROUTE)
               == scan_plan_result(&plan));
    /* Values are non-negative; UINT matches %u on the 32-bit target. */
    tm_printf((UB *)"[TEST OBS] width=%u lateral=%u diag=%u left=%u\n",
              (UINT)scan_plan_width_cm(&plan),
              (UINT)scan_plan_lateral_cm(&plan), (UINT)scan_plan_diag_cm(&plan),
              (UINT)scan_plan_side_is_left(&plan));

    fill(points, 200);
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    report("wall gives no route",
           SCAN_PLAN_NO_ROUTE == scan_plan_result(&plan));

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
        scan_plan_point_set(&p_points[index], angle, range_mm);
        index++;
    }
}

/*** end of file ***/

