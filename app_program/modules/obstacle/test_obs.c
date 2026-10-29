/** @file test_obs.c
 *
 * @brief Non-mutating tests. Set OBS_TEST_HW to 1 for a sensor ping test,
 * OBS_TEST_SERVO_CAL to 1 for the servo calibration check, or
 * OBS_TEST_FIND_LIMITS to 1 to find the servo limits. For any of these,
 * set APP_OBSTACLE_ENABLED to 0 so the task does not share the sensor.
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
#define OBS_TEST_SERVO_CAL 0
#define OBS_TEST_FIND_LIMITS 0
#define TEST_PLAN_FAULTS 0x15U
#define TEST_PLAN_COUNT 9U
#define TEST_PLAN_DIST_CM 123U
#define TEST_PLAN_WIDTH_CM 77U

/* Calibration: a flat wall CAL_WALL_MM straight ahead of the sensor. The
 * expected range at an angle is the wall distance divided by cos(angle),
 * so the table below is only valid for a 300 mm wall.
 */
#define CAL_WALL_MM 300
#define CAL_TOL_MM 25
#define CAL_HOLD_MSEC 4000
#define CAL_PINGS 3U

/** @brief Print one PASS or FAIL line. */
static void report (char const * p_name, bool b_pass);
/** @brief Fill a profile with one range at every scan angle. */
static void fill (scan_plan_point_t * p_points, int32_t range_mm);

#if OBS_TEST_SERVO_CAL
typedef struct
{
    int32_t angle_deg;
    int32_t expect_mm;
} cal_step_t;

/** @brief One ping, with a failed ping reported as OBS_FAR_MM. */
static int32_t cal_ping (void);
/** @brief Check measured range against the wall at three angles. */
static void cal_range_check (void);
/** @brief Hold known angles forever so the pointer can be measured. */
static void cal_visual_sweep (void);
#endif

#if OBS_TEST_FIND_LIMITS
/** @brief Step outwards in 5 degrees to find where the servo is blocked. */
static void cal_find_limits (void);
#endif

void test_obs_run (void)
{
    scan_plan_point_t  points[OBS_SCAN_POINTS];
    scan_plan_bypass_t plan = {0U, 0U};
    int32_t const      zero = OBS_SCAN_RIGHT_DEG / OBS_SCAN_STEP_DEG;
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

    /* A narrow object straight ahead: three samples around 0 degrees. */
    fill(points, 1500);
    points[zero - 1].range_mm = 254;
    points[zero].range_mm     = 250;
    points[zero + 1].range_mm = 254;
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
#endif

#if OBS_TEST_SERVO_CAL
    ultrasonic_init();
    servo_init();
    cal_range_check();
    cal_visual_sweep();
#endif

#if OBS_TEST_FIND_LIMITS
    servo_init();
    cal_find_limits();
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

    for (int32_t angle = -OBS_SCAN_RIGHT_DEG; angle <= OBS_SCAN_LEFT_DEG;
         angle += OBS_SCAN_STEP_DEG)
    {
        p_points[index].angle_deg = angle;
        p_points[index].range_mm  = range_mm;
        index++;
    }
}

#if OBS_TEST_SERVO_CAL
static int32_t cal_ping (void)
{
    int32_t                   range_mm = 0;
    ultrasonic_result_t const result   = ultrasonic_ping(&range_mm);

    tk_dly_tsk(OBS_PING_PERIOD_MSEC);
    return (ULTRASONIC_OK == result) ? range_mm : (int32_t)OBS_FAR_MM;
}

static void cal_range_check (void)
{
    /* Wall at 300 mm: expected range is 300 / cos(angle). */
    static cal_step_t const steps[] = {{-30, 346}, {0, 300}, {30, 346}};

    for (uint32_t index = 0U; index < (sizeof(steps) / sizeof(steps[0]));
         index++)
    {
        int32_t pings[CAL_PINGS];

        tk_dly_tsk((RELTIM)servo_set_angle(steps[index].angle_deg));

        for (uint32_t ping = 0U; ping < CAL_PINGS; ping++)
        {
            pings[ping] = cal_ping();
        }

        int32_t const median =
            ultrasonic_median_of_three(pings[0], pings[1], pings[2]);
        int32_t const error = median - steps[index].expect_mm;
        bool const    b_ok  = (error >= -CAL_TOL_MM) && (error <= CAL_TOL_MM);

        tm_printf((UB *)"[CAL] angle=%d expect=%d measured=%d error=%d %s\n",
                  (INT)steps[index].angle_deg, (INT)steps[index].expect_mm,
                  (INT)median, (INT)error, b_ok ? (UB *)"PASS" : (UB *)"FAIL");
    }

    tk_dly_tsk((RELTIM)servo_set_angle(0));
}

static void cal_visual_sweep (void)
{
    static int32_t const angles[] = {0, 30,
                                     0, -30,
                                     0, OBS_SERVO_LEFT_LIMIT_DEG,
                                     0, -OBS_SERVO_RIGHT_LIMIT_DEG};

    for (;;)
    {
        for (uint32_t index = 0U; index < (sizeof(angles) / sizeof(angles[0]));
             index++)
        {
            tk_dly_tsk((RELTIM)servo_set_angle(angles[index]));
            tm_printf((UB *)"[CAL] holding %d deg\n", (INT)angles[index]);
            tk_dly_tsk(CAL_HOLD_MSEC);
        }
    }
}
#endif

#if OBS_TEST_FIND_LIMITS
static void cal_find_limits (void)
{
    /* Set both servo limits to 80 in obs_config.h while using this, or the
     * servo clamps before it reaches the blocked position.
     */
    for (;;)
    {
        for (int32_t angle = 0; angle <= 80; angle += 5)
        {
            tk_dly_tsk((RELTIM)servo_set_angle(angle));
            tm_printf((UB *)"[CAL] left %d\n", (INT)angle);
            tk_dly_tsk(2000);
        }

        for (int32_t angle = 0; angle >= -80; angle -= 5)
        {
            tk_dly_tsk((RELTIM)servo_set_angle(angle));
            tm_printf((UB *)"[CAL] right %d\n", (INT)angle);
            tk_dly_tsk(2000);
        }
    }
}
#endif

/*** end of file ***/

