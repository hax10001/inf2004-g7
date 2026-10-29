/** @file test_obs.c
 * @brief Non-mutating tests. Set OBS_TEST_HW to 1 and APP_OBSTACLE_ENABLED to
 * 0 for the sensor and servo bring-up, so the task does not share the sensor.
 */
#include "test_obs.h"
#include "obs_config.h"
#include "obstacle.h"
#include "scan_plan.h"
#include "servo.h"
#include "ultrasonic.h"
#include <tm/tmonitor.h>

#define OBS_TEST_HW 0

static void report (char const * p_name, bool b_pass)
{
    tm_printf((UB *)"[TEST OBS] %s: %s\n", (UB *)p_name,
              b_pass ? (UB *)"PASS" : (UB *)"FAIL");
}

static void fill (scan_point_t * p_pts, uint16_t range)
{
    uint8_t i = 0U;

    for (int16_t a = -OBS_SCAN_LIMIT_DEG; a <= OBS_SCAN_LIMIT_DEG;
         a         = (int16_t)(a + OBS_SCAN_STEP_DEG))
    {
        p_pts[i].angle_deg = a;
        p_pts[i].range_mm  = range;
        i++;
    }
}

void test_obs_run (void)
{
    uint32_t const word = obstacle_status_pack(OBSTACLE_S_SEARCH_LINE,
                                               OBSTACLE_ROUTE_RIGHT, 0x15U, 9U,
                                               123U, 77U);

    report("pack round trip",
           (OBSTACLE_S_SEARCH_LINE == obstacle_status_state(word)) &&
               (OBSTACLE_ROUTE_RIGHT == obstacle_status_route(word)) &&
               (0x15U == obstacle_status_faults(word)) &&
               (9U == obstacle_status_count(word)) &&
               (123U == obstacle_status_distance_cm(word)) &&
               (77U == obstacle_status_width_cm(word)));

    scan_point_t  pts[OBS_SCAN_MAX_POINTS];
    bypass_plan_t plan;

    fill(pts, 1500U);
    scan_plan_build(pts, (uint8_t)OBS_SCAN_MAX_POINTS, &plan);
    report("empty scan is clear", (uint32_t)PLAN_CLEAR == plan.result);

    fill(pts, 1500U);
    pts[6].range_mm = 254U; /* -10 deg */
    pts[7].range_mm = 250U; /*   0 deg */
    pts[8].range_mm = 254U; /* +10 deg */
    scan_plan_build(pts, (uint8_t)OBS_SCAN_MAX_POINTS, &plan);
    report("narrow object gives a route", (uint32_t)PLAN_ROUTE == plan.result);
    tm_printf((UB *)"[TEST OBS] width_cm=%u lateral_cm=%u diag_cm=%u "
                    "angle_idx=%u side_left=%u\n",
              (UINT)plan.width_cm, (UINT)plan.lateral_cm, (UINT)plan.diag_cm,
              (UINT)plan.angle_idx, (UINT)plan.side_left);

    fill(pts, 200U);
    scan_plan_build(pts, (uint8_t)OBS_SCAN_MAX_POINTS, &plan);
    report("wall gives no route", (uint32_t)PLAN_NO_ROUTE == plan.result);

#if OBS_TEST_HW
    (void)ultrasonic_init();
    (void)servo_init();

    for (uint32_t i = 0U; i < 20U; i++)
    {
        uint16_t    mm  = 0U;
        us_result_t res = ultrasonic_ping(&mm);

        tm_printf((UB *)"[TEST OBS] ping %u: res=%u mm=%u\n", (UINT)i,
                  (UINT)res, (UINT)mm);
        (void)tk_dly_tsk(OBS_PING_PERIOD_MS);
    }

    for (int16_t a = -OBS_SERVO_LIMIT_DEG; a <= OBS_SERVO_LIMIT_DEG; a += 20)
    {
        (void)tk_dly_tsk((RELTIM)servo_set_angle(a));
        tm_printf((UB *)"[TEST OBS] servo angle %d\n", (INT)a);
        (void)tk_dly_tsk(500);
    }

    (void)servo_set_angle(0);
#endif
}

/*** end of file ***/