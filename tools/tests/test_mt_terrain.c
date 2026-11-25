/** @file test_mt_terrain.c
 * @brief Host test (FR8 hump/tilt, FR3 level reference): the real
 *        tilt_math.c angle and heading maths at known vectors, and the
 *        real terrain.c axis mapping, calibration, relative tilt with
 *        angle wrap, magnitude, snapshot store and kernel error paths.
 *
 * Usage: test_mt_terrain          (reference roll +10 degrees)
 *        test_mt_terrain wrap     (reference roll -10 degrees, other wrap)
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app/tasks.h"
#include "modules/terrain/task_imu.h"
#include "modules/terrain/terrain.h"
#include "modules/terrain/tilt_math.h"

/* Angle comparison tolerance, degrees. */
#define T_EPS_DEG 0.05f
/* Gravity, mg. */
#define T_G 1000.0f
/* Degrees to radians. */
#define T_RAD 0.0174532925f

static ER       g_cre_result[2];  /* Results of the next two creates. */
static uint32_t g_cre_calls;
static uint32_t g_del_calls;
static ER       g_loc_result;
static ER       g_unl_result;
static ID       g_task_result;
static uint32_t g_sta_calls;

/** @brief Host stub for tk_cre_mtx(): scripted handle or error. */
ID tk_cre_mtx (T_CMTX const * p_config)
{
    assert(NULL != p_config);
    ID const result = (g_cre_calls < 2U) ? g_cre_result[g_cre_calls] : 99;

    g_cre_calls++;
    return result;
}

/** @brief Host stub for tk_del_mtx(): count deletions. */
ER tk_del_mtx (ID mutex)
{
    (void)mutex;
    g_del_calls++;
    return E_OK;
}

/** @brief Host stub for tk_loc_mtx(): scripted result. */
ER tk_loc_mtx (ID mutex, TMO timeout)
{
    (void)mutex;
    (void)timeout;
    return g_loc_result;
}

/** @brief Host stub for tk_unl_mtx(): scripted result. */
ER tk_unl_mtx (ID mutex)
{
    (void)mutex;
    return g_unl_result;
}

/** @brief Host stub for tasks_create(): scripted handle or error. */
ID tasks_create (tasks_entry_t p_entry, PRI priority)
{
    assert(NULL != p_entry);
    (void)priority;
    return g_task_result;
}

/** @brief Host stub for tk_sta_tsk(): count starts. */
ER tk_sta_tsk (ID task, INT code)
{
    (void)task;
    (void)code;
    g_sta_calls++;
    return E_OK;
}

/** @brief Host stub: the IMU task is not run by this test. */
void task_imu_task (INT start_code, void * p_context)
{
    (void)start_code;
    (void)p_context;
}

/** @brief True when two angles agree within T_EPS_DEG. */
static bool near (float a, float b)
{
    return fabsf(a - b) < T_EPS_DEG;
}

/** @brief Axis value, mg, of gravity at @p deg from the reference. */
static int16_t g_sin (float deg)
{
    return (int16_t)lroundf(T_G * sinf(deg * T_RAD));
}

/** @brief Axis value, mg, of gravity at @p deg from the reference. */
static int16_t g_cos (float deg)
{
    return (int16_t)lroundf(T_G * cosf(deg * T_RAD));
}

/** @brief tilt_math_angles at 0, +-30, +-90, upside down and invalid. */
static void test_tilt_math_angles (void)
{
    float roll  = 0.0f;
    float pitch = 0.0f;

    assert(tilt_math_angles(0, 0, 1000, &roll, &pitch));
    assert(near(roll, 0.0f) && near(pitch, 0.0f));
    assert(tilt_math_angles(0, 500, 866, &roll, &pitch));
    assert(near(roll, 30.0f) && near(pitch, 0.0f));
    assert(tilt_math_angles(0, -500, 866, &roll, &pitch));
    assert(near(roll, -30.0f));
    assert(tilt_math_angles(-500, 0, 866, &roll, &pitch));
    assert(near(pitch, 30.0f) && near(roll, 0.0f));
    assert(tilt_math_angles(500, 0, 866, &roll, &pitch));
    assert(near(pitch, -30.0f));
    assert(tilt_math_angles(0, 1000, 0, &roll, &pitch));
    assert(near(roll, 90.0f));
    assert(tilt_math_angles(0, -1000, 0, &roll, &pitch));
    assert(near(roll, -90.0f));
    assert(tilt_math_angles(0, 0, -1000, &roll, &pitch));
    assert(near(fabsf(roll), 180.0f));
    /* Combined: pitch uses the whole Y-Z gravity, not Z alone. */
    assert(tilt_math_angles(-500, 433, 750, &roll, &pitch));
    assert(near(roll, 30.0f) && near(pitch, 30.0f));
    puts("PASS tilt_math_angles: 0, +-30 roll/pitch, +-90 and 180 roll");

    /* Full-scale int16 axes: no integer overflow, finite angles. */
    assert(tilt_math_angles(INT16_MIN, INT16_MIN, INT16_MIN, &roll,
                            &pitch));
    assert(isfinite(roll) && isfinite(pitch) && near(roll, -135.0f));
    puts("PASS tilt_math_angles: full-scale axes stay finite");

    roll  = 7.0f;
    pitch = 7.0f;
    assert(!tilt_math_angles(0, 0, 0, &roll, &pitch));
    /* Straight nose-down: Y and Z zero, roll undefined. */
    assert(!tilt_math_angles(1000, 0, 0, &roll, &pitch));
    assert((7.0f == roll) && (7.0f == pitch)); /* outputs untouched */
    assert(!tilt_math_angles(0, 0, 1000, NULL, &pitch));
    assert(!tilt_math_angles(0, 0, 1000, &roll, NULL));
    puts("PASS tilt_math_angles: zero vector, Y=Z=0 and NULL rejected, "
         "outputs untouched");
}

/** @brief Hump height from roll, and the 0..360 magnetic heading. */
static void test_tilt_math_height_heading (void)
{
    float heading = -1.0f;

    assert(0.0f == tilt_math_hump_height(0.0f, 110.0f));
    assert(fabsf(tilt_math_hump_height(30.0f, 110.0f) - 55.0f) < 0.01f);
    assert(tilt_math_hump_height(-30.0f, 110.0f) ==
           tilt_math_hump_height(30.0f, 110.0f));
    assert(fabsf(tilt_math_hump_height(90.0f, 110.0f) - 110.0f) < 0.01f);
    puts("PASS tilt_math_hump_height: 0, 55 mm at +-30 deg, 110 at 90");

    assert(tilt_math_heading_level(0, -100, &heading));
    assert(near(heading, 0.0f));
    assert(tilt_math_heading_level(100, 0, &heading));
    assert(near(heading, 90.0f));
    assert(tilt_math_heading_level(0, 100, &heading));
    assert(near(heading, 180.0f));
    /* atan2 gives -90 here: must be brought into 0..360 as 270. */
    assert(tilt_math_heading_level(-100, 0, &heading));
    assert(near(heading, 270.0f));
    assert(tilt_math_heading_level(-1, -1000, &heading));
    assert((heading >= 359.0f) && (heading < 360.0f));
    heading = 5.0f;
    assert(!tilt_math_heading_level(0, 0, &heading));
    assert(5.0f == heading);
    assert(!tilt_math_heading_level(10, 10, NULL));
    puts("PASS tilt_math_heading_level: 0/90/180/270, just under 360, "
         "zero field and NULL rejected");
}

/** @brief Calls before terrain_init() fail closed. */
static void test_before_init (void)
{
    terrain_snapshot_t snap = {0};
    float              roll  = 0.0f;
    float              pitch = 0.0f;

    assert(!terrain_is_calibrated());
    assert(E_OBJ == terrain_calibrate(0, 0, 1000));
    assert(E_OBJ == terrain_relative_tilt(0, 0, 1000, &roll, &pitch));
    assert(E_OBJ == terrain_publish_snapshot(&snap));
    assert(E_OBJ == terrain_get_snapshot(&snap));
    assert(E_PAR == terrain_publish_snapshot(NULL));
    assert(E_PAR == terrain_get_snapshot(NULL));
    assert(E_PAR == terrain_relative_tilt(0, 0, 1000, NULL, &pitch));
    puts("PASS terrain before init: E_OBJ, never a stale reference");
}

/** @brief Mutex creation failures, then a good init and no re-init. */
static void test_init (void)
{
    g_cre_result[0] = E_OBJ - 1;
    g_cre_calls     = 0U;
    assert((E_OBJ - 1) == terrain_init());
    g_cre_result[0] = E_OK;
    g_cre_calls     = 0U;
    assert(E_OBJ == terrain_init()); /* a zero handle is a failure */
    g_cre_result[0] = 5;
    g_cre_result[1] = E_TMOUT;
    g_cre_calls     = 0U;
    g_del_calls     = 0U;
    assert(E_TMOUT == terrain_init());
    assert(1U == g_del_calls); /* the first mutex is not leaked */
    g_cre_result[1] = E_OK;
    g_cre_calls     = 0U;
    assert(E_OBJ == terrain_init());
    assert(2U == g_del_calls);
    assert(!terrain_is_calibrated());
    g_cre_result[0] = 5;
    g_cre_result[1] = 6;
    g_cre_calls     = 0U;
    assert(E_OK == terrain_init());
    assert(E_OBJ == terrain_init()); /* only once */
    puts("PASS terrain_init: create errors returned, partial mutex "
         "deleted, second init refused");

    g_task_result = E_OK;
    assert(E_OBJ == terrain_start());
    g_task_result = E_TMOUT;
    assert(E_TMOUT == terrain_start());
    g_task_result = 8;
    g_sta_calls   = 0U;
    assert(E_OK == terrain_start());
    assert(1U == g_sta_calls);
    puts("PASS terrain_start: zero/negative task handle fails, good one "
         "starts the IMU task");
}

/** @brief Sensor-to-robot axis mapping and magnitude. */
static void test_axes_and_magnitude (void)
{
    float roll  = 0.0f;
    float pitch = 0.0f;

    /* Sensor X is the robot's sideways axis: it rolls the robot. */
    assert(E_OK == terrain_tilt(500, 0, 866, &roll, &pitch));
    assert(near(roll, 30.0f) && near(pitch, 0.0f));
    /* Sensor Y is the robot's backward axis: +Y is nose-up pitch. */
    assert(E_OK == terrain_tilt(0, 500, 866, &roll, &pitch));
    assert(near(roll, 0.0f) && near(pitch, 30.0f));
    assert(E_PAR == terrain_tilt(0, 0, 0, &roll, &pitch));
    assert(E_PAR == terrain_tilt(0, 1000, 0, &roll, &pitch));
    puts("PASS terrain_tilt: sensor X -> roll, sensor Y -> pitch, "
         "undefined roll E_PAR");

    assert(0 == terrain_magnitude(0, 0, 0));
    assert(1000 == terrain_magnitude(0, 0, 1000));
    assert(1000 == terrain_magnitude(600, -800, 0));
    assert(1 == terrain_magnitude(1, 1, 1)); /* floor of 1.73 */
    assert(3545 == terrain_magnitude(2047, 2047, 2047));
    assert(3547 == terrain_magnitude(-2048, -2048, -2048));
    puts("PASS terrain_magnitude: exact integer roots to the 12-bit "
         "extreme 3547 mg");

    float heading = 0.0f;

    assert(E_OK == terrain_heading_level(-100, 0, &heading));
    assert(near(heading, 270.0f));
    assert(E_PAR == terrain_heading_level(0, 0, &heading));
    assert(fabsf(terrain_hump_height(30.0f) - 55.0f) < 0.01f);
    puts("PASS terrain heading/hump height: 270 deg, zero field E_PAR, "
         "55 mm at 30 deg on 110 mm spacing");
}

/** @brief Calibrate once at @p ref_roll and check relative angles. */
static void test_calibration (float ref_roll)
{
    float roll  = 0.0f;
    float pitch = 0.0f;
    int16_t const ref_x = g_sin(ref_roll);
    int16_t const ref_z = g_cos(ref_roll);

    assert(E_OBJ == terrain_relative_tilt(ref_x, 0, ref_z, &roll, &pitch));
    assert(E_PAR == terrain_calibrate(0, 0, 0));
    assert(!terrain_is_calibrated());
    g_loc_result = E_TMOUT;
    assert(E_TMOUT == terrain_calibrate(ref_x, 0, ref_z));
    assert(!terrain_is_calibrated());
    g_loc_result = E_OK;
    assert(E_OK == terrain_calibrate(ref_x, 0, ref_z));
    assert(terrain_is_calibrated());
    /* Once only: a second, different reference is refused. */
    assert(E_OBJ == terrain_calibrate(0, 0, 1000));
    assert(E_OK == terrain_relative_tilt(ref_x, 0, ref_z, &roll, &pitch));
    assert(near(roll, 0.0f) && near(pitch, 0.0f));
    assert(E_OK == terrain_relative_tilt(g_sin(ref_roll + 20.0f), 0,
                                         g_cos(ref_roll + 20.0f),
                                         &roll, &pitch));
    assert(near(roll, 20.0f));
    puts("PASS terrain_calibrate: bad vector and lock error leave it "
         "uncalibrated; once only; relative roll 0 then +20");

    /* 185 degrees the other way round is 175 degrees, wrapped. */
    float const reading = (ref_roll > 0.0f) ? (ref_roll - 185.0f)
                                            : (ref_roll + 185.0f);
    float const want    = (ref_roll > 0.0f) ? 175.0f : -175.0f;

    assert(E_OK == terrain_relative_tilt(g_sin(reading), 0,
                                         g_cos(reading), &roll, &pitch));
    assert(near(roll, want));
    printf("PASS terrain_relative_tilt: reference %+.0f, reading %+.0f "
           "wraps to %+.0f (not %+.0f)\n", (double)ref_roll,
           (double)reading, (double)want, (double)(reading - ref_roll));

    g_loc_result = E_TMOUT;
    assert(!terrain_is_calibrated());
    assert(E_TMOUT == terrain_relative_tilt(0, 0, 1000, &roll, &pitch));
    g_loc_result = E_OK;
    g_unl_result = E_TMOUT;
    assert(!terrain_is_calibrated());
    assert(E_TMOUT == terrain_relative_tilt(0, 0, 1000, &roll, &pitch));
    g_unl_result = E_OK;
    assert(E_PAR == terrain_relative_tilt(0, 0, 0, &roll, &pitch));
    puts("PASS terrain_relative_tilt: lock/unlock errors returned, not "
         "reported as calibrated");
}

/** @brief Snapshot publish and copy, with lock errors. */
static void test_snapshot (void)
{
    terrain_snapshot_t in  = {0};
    terrain_snapshot_t out = {0};

    in.roll_deg            = 4.5f;
    in.last_hump_height_mm = 9U;
    in.lift_height_mm      = 1022U;
    in.b_lifted            = 1U;
    in.timestamp_msec      = 1234U;
    assert(E_OK == terrain_publish_snapshot(&in));
    assert(E_OK == terrain_get_snapshot(&out));
    assert((4.5f == out.roll_deg) && (9U == out.last_hump_height_mm) &&
           (1022U == out.lift_height_mm) && (1U == out.b_lifted) &&
           (1234U == out.timestamp_msec));
    g_loc_result = E_TMOUT;
    in.timestamp_msec = 99U;
    assert(E_TMOUT == terrain_publish_snapshot(&in));
    assert(E_TMOUT == terrain_get_snapshot(&out));
    g_loc_result = E_OK;
    assert(E_OK == terrain_get_snapshot(&out));
    assert(1234U == out.timestamp_msec); /* failed publish wrote nothing */
    puts("PASS terrain snapshot: copy round-trips; lock error writes "
         "nothing");
}

/** @brief Run the cases; "wrap" calibrates at -10 degrees instead. */
int main (int argc, char ** argv)
{
    bool const b_wrap = (argc > 1) && (0 == strcmp(argv[1], "wrap"));

    g_loc_result = E_OK;
    g_unl_result = E_OK;
    test_tilt_math_angles();
    test_tilt_math_height_heading();
    test_before_init();
    test_init();
    test_axes_and_magnitude();
    test_calibration(b_wrap ? -10.0f : 10.0f);
    test_snapshot();
    return 0;
}

/*** end of file ***/

