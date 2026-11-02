/* Host-only tests for the Buddy 5 geometry, servo mapping and planner.
 *
 * Built twice by run_buddy5_clearance.py: with the robot's configuration
 * and with a measured-looking mount and 75 degree servo limits, so the
 * frame transform and the wider sweep are checked as well as the defaults.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "board/board_config.h"
#include "modules/obstacle/obs_config.h"
#include "modules/obstacle/obs_geometry.h"
#include "modules/obstacle/scan_plan.h"
#include "modules/obstacle/servo.h"

#define HALF_PATH_MM                                                           \
    ((OBS_ROBOT_WIDTH_MM / 2) + OBS_SAFETY_MARGIN_MM + OBS_OUTWARD_EXTRA_MM)
#define MAX_POINTS OBS_SCAN_MAX_POINTS

static UW g_pulse;
static int g_checks;

/* BSP doubles for servo.c. */
ER pwm_set_pin(UINT pin) { (void)pin; return E_OK; }
ER pwm_set_wrap(UINT pin, UW wrap) { (void)pin; (void)wrap; return E_OK; }
ER pwm_set_enabled(UINT pin, BOOL on) { (void)pin; (void)on; return E_OK; }
ER pwm_set_cc(UINT pin, UW compare)
{
    if (BOARD_SERVO_PIN == pin)
    {
        g_pulse = compare;
    }
    return E_OK;
}
void out_w(UW address, UW value) { (void)address; (void)value; }

static void pass(const char *name)
{
    g_checks++;
    printf("PASS %s\n", name);
}

static int near_value(int32_t value, int32_t expect, int32_t tol)
{
    return abs(value - expect) <= tol;
}

/* The sweep the obstacle task takes: far right, profile, far left. */
static int32_t sweep(scan_plan_point_t *points, int32_t range)
{
    int32_t count = 0;
    int32_t const right = (OBS_SCAN_FAR_DEG < OBS_SERVO_RIGHT_LIMIT_DEG)
                              ? OBS_SCAN_FAR_DEG : OBS_SERVO_RIGHT_LIMIT_DEG;
    int32_t const left = (OBS_SCAN_FAR_DEG < OBS_SERVO_LEFT_LIMIT_DEG)
                             ? OBS_SCAN_FAR_DEG : OBS_SERVO_LEFT_LIMIT_DEG;

    points[count].angle_deg = -right;
    points[count++].range_mm = range;
    for (int32_t angle = -OBS_SCAN_RIGHT_DEG; angle <= OBS_SCAN_LEFT_DEG;
         angle += OBS_SCAN_STEP_DEG)
    {
        points[count].angle_deg = angle;
        points[count++].range_mm = range;
    }
    points[count].angle_deg = left;
    points[count++].range_mm = range;
    assert(count == MAX_POINTS);
    return count;
}

/* Index of a bearing in the sweep. */
static int32_t at(scan_plan_point_t const *points, int32_t angle)
{
    for (int32_t i = 0; i < MAX_POINTS; i++)
    {
        if (points[i].angle_deg == angle)
        {
            return i;
        }
    }
    assert(0);
    return -1;
}

/* A box straight ahead, seen at 0 and +/-10 degrees, as a 150 mm box. */
static void centre_box(scan_plan_point_t *points)
{
    points[at(points, -10)].range_mm = 354;
    points[at(points, 0)].range_mm = 350;
    points[at(points, 10)].range_mm = 354;
}

static void test_geometry(void)
{
    int32_t fwd = 0;
    int32_t lat = 0;

    assert(0 == obs_geometry_sin_milli(0));
    assert(1000 == obs_geometry_sin_milli(90));
    assert(-1000 == obs_geometry_sin_milli(-90));
    assert(500 == obs_geometry_sin_milli(30));
    assert(-500 == obs_geometry_sin_milli(390 - 420));
    assert(-1000 == obs_geometry_cos_milli(180));
    assert(866 == obs_geometry_cos_milli(-30));
    assert(obs_geometry_sin_milli(75) == 966);
    pass("integer sine and cosine at any whole degree");

    /* Mounting transform: axis = face at FRONT minus the arm, and the
     * face swings round the axis with the bearing.
     */
    obs_geometry_sensor_origin(0, &fwd, &lat);
    assert(fwd == OBS_SENSOR_TO_AXLE_MM);
    assert(lat == OBS_SENSOR_LATERAL_MM);
    obs_geometry_sensor_origin(90, &fwd, &lat);
    assert(fwd == (OBS_SENSOR_TO_AXLE_MM - OBS_SERVO_ARM_MM));
    assert(lat == (OBS_SENSOR_LATERAL_MM + OBS_SERVO_ARM_MM));
    obs_geometry_sensor_origin(-90, &fwd, &lat);
    assert(lat == (OBS_SENSOR_LATERAL_MM - OBS_SERVO_ARM_MM));
    pass("sensor origin rotates about the servo axis, not the axle");

    obs_geometry_echo_point(0, 300, &fwd, &lat);
    assert(fwd == (OBS_SENSOR_TO_AXLE_MM + 300));
    assert(lat == OBS_SENSOR_LATERAL_MM);
    obs_geometry_echo_point(70, 200, &fwd, &lat);
    assert(near_value(lat, OBS_SENSOR_LATERAL_MM +
                               ((OBS_SERVO_ARM_MM * 940) / 1000) + 188, 2));
    assert(near_value(fwd, OBS_SENSOR_TO_AXLE_MM - OBS_SERVO_ARM_MM +
                               ((OBS_SERVO_ARM_MM * 342) / 1000) + 68, 2));
    pass("echo points are placed from the transducer, in the vehicle frame");

    obs_geometry_pose_t const start = {0, 0, 0};
    obs_geometry_pose_t pose = {0, 0, 0};
    int32_t const half_track = OBS_TRACK_WIDTH_MM / 2;

    obs_geometry_pivot(&start, 1, 90, &pose);
    assert(near_value(pose.x_mm, half_track, 1));
    assert(near_value(pose.y_mm, half_track, 1));
    assert(90 == pose.heading_deg);
    obs_geometry_pivot(&start, -1, -90, &pose);
    assert(near_value(pose.x_mm, half_track, 1));
    assert(near_value(pose.y_mm, -half_track, 1));
    obs_geometry_advance(&pose, 100, &pose);
    assert(near_value(pose.y_mm, -half_track - 100, 1));
    pass("pivots about a stopped wheel move the axle on the wheel's circle");

    obs_geometry_box_t ahead = {OBS_ROBOT_FRONT_MM + 50,
                                OBS_ROBOT_FRONT_MM + 80, -20, 20};
    assert(obs_geometry_body_hits_box(&start, &ahead, 60));
    assert(!obs_geometry_body_hits_box(&start, &ahead, 40));
    obs_geometry_box_t beside = {0, 50, (OBS_ROBOT_WIDTH_MM / 2) + 30, 400};
    assert(obs_geometry_body_hits_box(&start, &beside, 31));
    assert(!obs_geometry_body_hits_box(&start, &beside, 29));
    obs_geometry_pose_t const turned = {0, 0, 90};
    /* Turned left, the nose points at +y: the box beside is now ahead. */
    assert(obs_geometry_body_hits_box(&turned, &beside, 0));
    obs_geometry_box_t behind_corner = {-OBS_ROBOT_REAR_MM - 100,
                                        -OBS_ROBOT_REAR_MM - 60, 0, 10};
    assert(!obs_geometry_body_hits_box(&start, &behind_corner, 50));
    pass("whole-body clearance test against boxes at any heading");
}

static void test_servo(void)
{
    int32_t const left = servo_left_limit_deg();
    int32_t const right = servo_right_limit_deg();

    assert(E_OK == servo_init());
    assert(OBS_SERVO_FRONT_USEC == (int32_t)g_pulse);
    assert(0 == servo_angle_get());
    pass("servo starts at the calibrated FRONT pulse, angle zero");

    (void)servo_set_angle(30);
    assert((int32_t)g_pulse ==
           OBS_SERVO_FRONT_USEC + (OBS_SERVO_LEFT_SIGN * 30 *
                                   OBS_SERVO_USEC_PER_DEG));
    (void)servo_set_angle(-30);
    assert((int32_t)g_pulse ==
           OBS_SERVO_FRONT_USEC - (OBS_SERVO_LEFT_SIGN * 30 *
                                   OBS_SERVO_USEC_PER_DEG));
    pass("positive bearings are left and negative right, by LEFT_SIGN");

    assert(left == OBS_SERVO_LEFT_LIMIT_DEG);
    assert(right == OBS_SERVO_RIGHT_LIMIT_DEG);
    (void)servo_set_angle(200);
    assert(left == servo_angle_get());
    (void)servo_set_angle(-200);
    assert(-right == servo_angle_get());
    assert((int32_t)g_pulse >= OBS_SERVO_MIN_USEC);
    assert((int32_t)g_pulse <= OBS_SERVO_MAX_USEC);
    printf("     servo limits L=%d R=%d deg, pulses %d..%d usec\n",
           (int)left, (int)right,
           OBS_SERVO_FRONT_USEC - (right * OBS_SERVO_USEC_PER_DEG),
           OBS_SERVO_FRONT_USEC + (left * OBS_SERVO_USEC_PER_DEG));
    pass("bearings clamp to the measured physical limits inside the PWM range");

    uint32_t const small = servo_set_angle(-right + 10);
    uint32_t const large = servo_set_angle(left);
    assert(large > small);
    assert(servo_set_angle(left) == (uint32_t)OBS_SERVO_SETTLE_BASE_MSEC);
    pass("settle time grows with travel and is never zero");
}

static void test_planner(void)
{
    scan_plan_point_t points[MAX_POINTS];
    scan_plan_bypass_t plan = {0U, 0U};
    int32_t count = sweep(points, 1500);

    assert(OBS_SCAN_MAX_POINTS == OBS_SCAN_POINTS + 2);
    assert(points[0].angle_deg <= -70);
    assert(points[MAX_POINTS - 1].angle_deg >= 70);
    printf("     sweep %d..%d deg, %d bearings\n", (int)points[0].angle_deg,
           (int)points[MAX_POINTS - 1].angle_deg, (int)count);
    pass("one sweep covers far right, the profile and far left");

    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_CLEAR == scan_plan_result(&plan));
    pass("empty scan is clear");

    count = sweep(points, 1500);
    centre_box(points);
    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    /* The offset reserves half the body, the margin and the allowance. */
    assert((scan_plan_lateral_cm(&plan) * 10) >= (HALF_PATH_MM - 5));
    /* The near face is from the axle: the range plus the sensor offset. */
    assert(near_value(scan_plan_near_cm(&plan) * 10,
                      OBS_SENSOR_TO_AXLE_MM + 345, 15));
    pass("object ahead, both sides open: route with full body offset");

    scan_plan_build_checked(points, count, 350, false, true, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    assert(!scan_plan_side_is_left(&plan));
    scan_plan_build_checked(points, count, 350, true, false, &plan);
    assert(scan_plan_side_is_left(&plan));
    scan_plan_build_checked(points, count, 350, false, false, &plan);
    assert(SCAN_PLAN_NO_ROUTE == scan_plan_result(&plan));
    pass("caller vetoes still select the other side or none");

    /* Left blocked by a second object beside the first. */
    count = sweep(points, 1500);
    centre_box(points);
    points[at(points, 30)].range_mm = 380;
    points[at(points, 40)].range_mm = 360;
    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    assert(!scan_plan_side_is_left(&plan));
    pass("object ahead, left blocked: bypass right");

    count = sweep(points, 1500);
    centre_box(points);
    points[at(points, -30)].range_mm = 380;
    points[at(points, -40)].range_mm = 360;
    scan_plan_build(points, count, &plan);
    assert(scan_plan_side_is_left(&plan));
    pass("object ahead, right blocked: bypass left");

    /* Only the far bearing sees something in the left departure corridor. */
    count = sweep(points, 1500);
    centre_box(points);
    points[MAX_POINTS - 1].range_mm = 300;
    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    assert(!scan_plan_side_is_left(&plan));
    count = sweep(points, 1500);
    centre_box(points);
    points[0].range_mm = 300;
    scan_plan_build(points, count, &plan);
    assert(scan_plan_side_is_left(&plan));
    pass("far-bearing obstacle in the outward corridor vetoes that side");

    /* A distant wall out at the far bearing leaves room. */
    count = sweep(points, 1500);
    centre_box(points);
    points[0].range_mm = 900;
    points[MAX_POINTS - 1].range_mm = 900;
    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    assert(scan_plan_slack_cm(&plan) > 0);
    pass("distant side surfaces leave a route and checked slack");

    /* No echo at all is never free space. */
    count = sweep(points, OBS_FAR_MM);
    centre_box(points);
    points[MAX_POINTS - 1].range_mm = SCAN_PLAN_RANGE_UNKNOWN;
    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    assert(!scan_plan_side_is_left(&plan));
    points[0].range_mm = SCAN_PLAN_RANGE_UNKNOWN;
    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_NO_ROUTE == scan_plan_result(&plan));
    pass("missing echoes leave a side unverified; beyond-range is not proof");

    /* Beyond range everywhere around the object: a usable route. */
    count = sweep(points, OBS_FAR_MM);
    centre_box(points);
    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    pass("beyond-range outer bearings with a healthy sensor give a route");

    /* An unknown sample next to the object is more of the object. */
    count = sweep(points, 1500);
    centre_box(points);
    scan_plan_build(points, count, &plan);
    int32_t const width_known = scan_plan_width_cm(&plan);
    points[at(points, 20)].range_mm = SCAN_PLAN_RANGE_UNKNOWN;
    scan_plan_build(points, count, &plan);
    assert(scan_plan_width_cm(&plan) > width_known);
    pass("a missing echo beside the object widens it, never narrows it");

    /* An object off to the left: the right needs clearly less offset. */
    count = sweep(points, 1500);
    points[at(points, 0)].range_mm = 350;
    points[at(points, 10)].range_mm = 352;
    points[at(points, 20)].range_mm = 360;
    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    assert(!scan_plan_side_is_left(&plan));
    pass("both sides open: the shorter way round is chosen");

    count = sweep(points, 1500);
    scan_plan_build_confirmed(points, count, 350, &plan);
    assert(points[at(points, 0)].range_mm == 350);
    assert(SCAN_PLAN_CLEAR != scan_plan_result(&plan));
    count = sweep(points, 1500);
    points[at(points, 0)].range_mm = SCAN_PLAN_RANGE_UNKNOWN;
    scan_plan_build_confirmed(points, count, 350, &plan);
    assert(SCAN_PLAN_CLEAR != scan_plan_result(&plan));
    count = sweep(points, 1500);
    scan_plan_build_confirmed(points, count, 4095, &plan);
    assert(SCAN_PLAN_CLEAR == scan_plan_result(&plan));
    pass("a confirmed front object survives a missed or lost centre echo");

    /* The profile is filled but the far bearings see nothing near. 230 mm
     * keeps clear of the marginal case where a 90 degree pivot only just
     * fails its margin; that depends on the mount.
     */
    count = sweep(points, 230);
    points[0].range_mm = OBS_FAR_MM;
    points[MAX_POINTS - 1].range_mm = OBS_FAR_MM;
    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    assert((scan_plan_lateral_cm(&plan) * 10) >= OBS_LATERAL_UNKNOWN_MM);
    pass("an object filling the profile uses the unknown-edge offset");

    /* Walls 200 mm away at both far bearings leave no way round. */
    count = sweep(points, 200);
    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_NO_ROUTE == scan_plan_result(&plan));
    pass("close surfaces at both far bearings give no route");

    /* Too close to pivot without the body reaching the object. */
    count = sweep(points, 1500);
    points[at(points, -10)].range_mm = 45;
    points[at(points, 0)].range_mm = 40;
    points[at(points, 10)].range_mm = 45;
    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_NO_ROUTE == scan_plan_result(&plan));
    pass("no departure angle whose swept body clears: no route");

    count = sweep(points, 1500);
    points[1].angle_deg = points[2].angle_deg;
    scan_plan_build(points, count, &plan);
    assert(SCAN_PLAN_NO_ROUTE == scan_plan_result(&plan));
    pass("unordered bearings are rejected");

    /* The nine-bearing profile alone, as the on-target self-test uses. */
    scan_plan_point_t profile[OBS_SCAN_POINTS];
    int32_t n = 0;

    for (int32_t angle = -OBS_SCAN_RIGHT_DEG; angle <= OBS_SCAN_LEFT_DEG;
         angle += OBS_SCAN_STEP_DEG)
    {
        profile[n].angle_deg = angle;
        profile[n++].range_mm = 1500;
    }
    profile[3].range_mm = 254;
    profile[4].range_mm = 250;
    profile[5].range_mm = 254;
    scan_plan_build(profile, n, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    /* Objects beside it at +/-20 degrees: the way round all three fits
     * within the widest offset, so it is taken rather than stopping.
     */
    profile[2].range_mm = 400;
    profile[6].range_mm = 400;
    scan_plan_build(profile, n, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    assert((scan_plan_lateral_cm(&plan) * 10) > (HALF_PATH_MM + 100));
    assert((scan_plan_lateral_cm(&plan) * 10) <= OBS_LATERAL_UNKNOWN_MM);
    /* Further out, the way round would be wider than ever allowed. */
    profile[2].range_mm = 1500;
    profile[6].range_mm = 1500;
    profile[0].range_mm = 400;
    profile[1].range_mm = 400;
    profile[7].range_mm = 400;
    profile[8].range_mm = 400;
    scan_plan_build(profile, n, &plan);
    assert(SCAN_PLAN_NO_ROUTE == scan_plan_result(&plan));
    profile[0].range_mm = 1500;
    profile[1].range_mm = 1500;
    profile[7].range_mm = 1500;
    profile[8].range_mm = 1500;
    for (int32_t i = 0; i < n; i++)
    {
        profile[i].range_mm = 230;
    }
    scan_plan_build(profile, n, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    assert((scan_plan_lateral_cm(&plan) * 10) >= OBS_LATERAL_UNKNOWN_MM);
    pass("the nine-bearing profile still plans as before");
}

int main(void)
{
    test_geometry();
    test_servo();
    test_planner();
    printf("All %d geometry, servo and planner checks passed "
           "(arm %d mm, lateral %d mm, limits L%d R%d).\n",
           g_checks, OBS_SERVO_ARM_MM, OBS_SENSOR_LATERAL_MM,
           OBS_SERVO_LEFT_LIMIT_DEG, OBS_SERVO_RIGHT_LIMIT_DEG);
    return 0;
}
