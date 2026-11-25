/** @file test_mt_downhill.c
 * @brief Host test (FR18 downhill speed cap): boundaries not covered by
 *        test_downhill_governor.c / test_ramp_motion.c, on the real
 *        downhill.c and motion.c (with steering.c and wheel_speed.c).
 *
 * Pure governor: the 40 RPM runaway limit is strictly "over" (40.0 is
 * not a runaway), the 5 RPM target floor, a right-wheel-only runaway, an
 * interrupted streak restarting the 10-period count, and the count
 * clearing at full speed. Production motion.c: only a confirmed,
 * fresh, valid hump DESCENDING phase engages the 65 % cap (200 ms fresh
 * boundary), a wheel over 40 RPM is cut to the -25 % brake at once while
 * a wheel under it is only ramped, 9 runaway periods keep driving and the
 * 10th latches the overspeed stop, which outlasts later commands.
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app/app_config.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "modules/motion/downhill.h"
#include "modules/motion/motion.h"

/* Controller period, us. */
#define T_PERIOD_USEC 50000U
/* Encoder counts per 50 ms: 17 = 19.3 RPM (near the 65 % target of
 * 30 RPM), 35 = 39.8 RPM (under the runaway limit), 36 = 40.9 RPM. */
#define T_NEAR_TARGET 17
#define T_UNDER_LIMIT 35
#define T_OVER_LIMIT 36
/* -25 % brake as a PWM compare count (12499 * 25 / 100, truncated). */
#define T_BRAKE_PWM 3124
/* A wheel target well above the 5 RPM floor, RPM. */
#define T_TARGET 9.75f

static uint32_t g_now_usec = 1000000U;
static int32_t  g_count[ENCODER_WHEEL_COUNT];
static int32_t  g_reg_left;
static int32_t  g_reg_right;
static uint32_t g_reg_calls;
static uint32_t g_stop_calls;

/** @brief Host stub: outputs off. */
void motor_stop (void)
{
    g_stop_calls++;
    g_reg_left  = 0;
    g_reg_right = 0;
}

/** @brief Host stub: short brake counts as outputs off. */
void motor_brake (void)
{
    motor_stop();
}

/** @brief Host stub for motor_init(). */
void motor_init (void)
{
    motor_stop();
}

/** @brief Host stub: not used by these paths. */
void motor_set_forward (uint32_t left_pwm, uint32_t right_pwm)
{
    g_reg_left  = (int32_t)left_pwm;
    g_reg_right = (int32_t)right_pwm;
}

/** @brief Host stub: record signed regulated duty (negative = brake). */
void motor_set_regulated (int32_t left_pwm, int32_t right_pwm)
{
    g_reg_calls++;
    g_reg_left  = left_pwm;
    g_reg_right = right_pwm;
}

/** @brief Host stub: not used by these paths. */
void motor_set_directional (int32_t left_pwm, int32_t right_pwm)
{
    g_reg_left  = left_pwm;
    g_reg_right = right_pwm;
}

/** @brief Host stub for encoder_init(). */
ER encoder_init (void)
{
    return E_OK;
}

/** @brief Host stub: latest edge at the current time. */
void encoder_read (encoder_wheel_t wheel, encoder_sample_t * p_sample)
{
    p_sample->count     = g_count[wheel];
    p_sample->edge_usec = g_now_usec;
}

/** @brief Host stub: the simulated microsecond clock. */
uint32_t encoder_now_usec (void)
{
    return g_now_usec;
}

/** @brief Host stub: no barcode capture. */
uint8_t bc_wave_capture_active (void)
{
    return 0U;
}

/** @brief Host stub for tk_cre_mtx(). */
ID tk_cre_mtx (T_CMTX const * p_config)
{
    (void)p_config;
    return 42;
}

/** @brief Host stub for tk_loc_mtx(). */
ER tk_loc_mtx (ID mutex, TMO timeout)
{
    (void)mutex;
    (void)timeout;
    return E_OK;
}

/** @brief Host stub for tk_unl_mtx(). */
ER tk_unl_mtx (ID mutex)
{
    (void)mutex;
    return E_OK;
}

/** @brief Host stub for tm_printf(). */
INT tm_printf (UB const * p_format, ...)
{
    (void)p_format;
    return 0;
}

/** @brief A governor engaged by a confirmed descent (scale 95 %). */
static void engaged (downhill_governor_t * p_gov)
{
    downhill_reset(p_gov);
    downhill_sample_terrain(p_gov, 0.0f, 0.0f, true, true, true);
    assert(p_gov->engaged && (p_gov->scale_percent < 100U));
}

/** @brief 40 RPM is the limit, not a runaway; 5 RPM target floor. */
static void test_runaway_limit (void)
{
    downhill_governor_t gov;

    engaged(&gov);
    assert(!downhill_wheel_over_target(&gov, T_TARGET, 40.0f));
    assert(downhill_wheel_over_target(&gov, T_TARGET, 40.01f));
    assert(!downhill_wheel_over_target(&gov, 4.99f, 80.0f));
    assert(downhill_wheel_over_target(&gov, 5.0f, 40.01f));
    assert(!downhill_wheel_over_target(NULL, T_TARGET, 80.0f));

    for (uint32_t i = 0U; i < 30U; i++)
    {
        assert(!downhill_observe_speed(&gov, T_TARGET, T_TARGET,
                                       40.0f, 40.0f));
    }

    assert(!gov.fault);
    /* Not on a descent (full speed): never a runaway. */
    downhill_reset(&gov);
    assert(!downhill_wheel_over_target(&gov, T_TARGET, 80.0f));

    for (uint32_t i = 0U; i < 30U; i++)
    {
        assert(!downhill_observe_speed(&gov, T_TARGET, T_TARGET,
                                       80.0f, 80.0f));
    }

    puts("PASS downhill runaway limit: 40.00 RPM is not over, 40.01 is; "
         "target < 5 RPM ignored; full speed never counts");
}

/** @brief Right wheel alone; an interrupted streak starts again. */
static void test_streak (void)
{
    downhill_governor_t gov;
    float               l = 1.0f;
    float               r = 1.0f;

    engaged(&gov);

    for (uint32_t i = 0U; i < 9U; i++)
    {
        assert(!downhill_observe_speed(&gov, T_TARGET, T_TARGET,
                                       10.0f, 45.0f));
    }

    /* One period at exactly 40 RPM breaks the streak. */
    assert(!downhill_observe_speed(&gov, T_TARGET, T_TARGET, 10.0f, 40.0f));

    for (uint32_t i = 0U; i < 9U; i++)
    {
        assert(!downhill_observe_speed(&gov, T_TARGET, T_TARGET,
                                       10.0f, 45.0f));
    }

    assert(!gov.fault);
    assert(downhill_observe_speed(&gov, T_TARGET, T_TARGET, 10.0f, 45.0f));
    /* Latched: normal speed later still reports the fault, zero targets. */
    assert(downhill_observe_speed(&gov, T_TARGET, T_TARGET, 0.0f, 0.0f));
    downhill_scale_pair(&gov, 20.0f, 20.0f, &l, &r);
    assert((0.0f == l) && (0.0f == r));
    puts("PASS downhill streak: right wheel alone counts; 9 over, one at "
         "40.0, 9 over = no stop; 10 in a row latches");

    /* Back at full speed clears a partial count before the next slope. */
    engaged(&gov);

    for (uint32_t i = 0U; i < 9U; i++)
    {
        (void)downhill_observe_speed(&gov, T_TARGET, T_TARGET, 45.0f, 0.0f);
    }

    for (uint32_t i = 0U; i < 20U; i++)
    {
        downhill_sample_terrain(&gov, 0.0f, 0.0f, true, true, false);
    }

    assert(100U == gov.scale_percent);
    assert(!downhill_observe_speed(&gov, T_TARGET, T_TARGET, 45.0f, 0.0f));
    downhill_sample_terrain(&gov, 0.0f, 0.0f, true, true, true);

    for (uint32_t i = 0U; i < 9U; i++)
    {
        assert(!downhill_observe_speed(&gov, T_TARGET, T_TARGET,
                                       45.0f, 0.0f));
    }

    assert(!gov.fault);
    puts("PASS downhill streak: count cleared at full speed, next descent "
         "gets a full 10 periods");

    /* NULL arguments are refused safely. */
    assert(!downhill_observe_speed(NULL, T_TARGET, T_TARGET, 80.0f, 80.0f));
    downhill_sample_terrain(NULL, 0.0f, 0.0f, true, true, true);
    downhill_reset(NULL);
    downhill_scale_pair(NULL, 20.0f, 10.0f, &l, &r);
    assert((20.0f == l) && (10.0f == r));
    downhill_scale_pair(&gov, 20.0f, 10.0f, NULL, &r);
    puts("PASS downhill NULL handling: no state = no derating, no crash");
}

/** @brief One controller period with each wheel advancing @p counts. */
static void period (terrain_snapshot_t const * p_snap, int32_t left,
                    int32_t right)
{
    g_now_usec += T_PERIOD_USEC;
    g_count[ENCODER_LEFT] += left;
    g_count[ENCODER_RIGHT] += right;

    if (NULL != p_snap)
    {
        motion_observe_terrain(p_snap, g_now_usec / 1000U);
    }

    motion_apply(LINE_FORWARD);
}

/** @brief A fresh snapshot in hump phase @p phase. */
static terrain_snapshot_t snapshot (uint8_t phase, uint32_t age_msec)
{
    terrain_snapshot_t s = {0};

    s.b_valid        = 1U;
    s.height_valid   = 1U;
    s.onHump         = (TERRAIN_HUMP_LEVEL != phase) ? 1U : 0U;
    s.hump_phase     = phase;
    s.timestamp_msec = (g_now_usec / 1000U) + 50U - age_msec;
    return s;
}

/** @brief Run @p n periods with a fresh snapshot of @p phase. */
static motion_status_t run_phase (uint8_t phase, uint32_t n,
                                  int32_t left, int32_t right)
{
    motion_status_t status = {0};

    for (uint32_t i = 0U; i < n; i++)
    {
        terrain_snapshot_t const s = snapshot(phase, 0U);

        period(&s, left, right);
    }

    assert(E_OK == motion_get_status(&status));
    return status;
}

/** @brief Only a fresh, valid, on-hump DESCENDING phase engages. */
static void test_descent_gate (void)
{
    motion_status_t    status = {0};
    terrain_snapshot_t s      = {0};

    motion_reset_run();
    status = run_phase(TERRAIN_HUMP_CLIMBING, 20U, T_NEAR_TARGET,
                       T_NEAR_TARGET);
    assert(!status.downhill_engaged && (100U == status.downhill_percent));

    for (uint32_t i = 0U; i < 20U; i++)
    {
        s = snapshot(TERRAIN_HUMP_DESCENDING, 0U);
        s.height_valid = 0U;
        period(&s, T_NEAR_TARGET, T_NEAR_TARGET);
        s = snapshot(TERRAIN_HUMP_DESCENDING, 0U);
        s.onHump = 0U;
        period(&s, T_NEAR_TARGET, T_NEAR_TARGET);
    }

    assert(E_OK == motion_get_status(&status));
    assert(!status.downhill_engaged && (0U == status.downhill_fault));
    puts("PASS motion descent gate: climbing, invalid height or off-hump "
         "DESCENDING never caps the speed");

    /* 200 ms old is fresh and engages; 201 ms is stale. */
    s = snapshot(TERRAIN_HUMP_DESCENDING, 200U);
    period(&s, T_NEAR_TARGET, T_NEAR_TARGET);
    assert(E_OK == motion_get_status(&status));
    assert(status.downhill_engaged && (95U == status.downhill_percent));
    motion_reset_run();

    for (uint32_t i = 0U; i < 19U; i++)
    {
        s = snapshot(TERRAIN_HUMP_DESCENDING, 201U);
        period(&s, T_NEAR_TARGET, T_NEAR_TARGET);
    }

    assert(E_OK == motion_get_status(&status));
    assert(!status.downhill_engaged && (0U == motion_downhill_fault()));
    s = snapshot(TERRAIN_HUMP_DESCENDING, 201U);
    period(&s, T_NEAR_TARGET, T_NEAR_TARGET);
    assert(1U == motion_downhill_fault()); /* IMU stale */
    puts("PASS motion descent gate: 200 ms old snapshot engages, 201 ms "
         "is stale (20 stale periods = IMU fault)");
    motion_downhill_reset();
}

/** @brief Runaway cut and the 10-period overspeed stop on motion.c. */
static void test_runaway_cut (void)
{
    motion_status_t status = {0};

    motion_reset_run();
    motion_stop_now();
    status = run_phase(TERRAIN_HUMP_DESCENDING, 30U, T_NEAR_TARGET,
                       T_NEAR_TARGET);
    assert(65U == status.downhill_percent);
    assert((status.left_target_rpm > 19.4f) &&
           (status.left_target_rpm < 19.6f));
    assert((g_reg_left > 0) && (g_reg_right > 0));

    /* Left over the limit, right just under it. */
    int32_t const prior_right = g_reg_right;

    status = run_phase(TERRAIN_HUMP_DESCENDING, 1U, T_OVER_LIMIT,
                       T_UNDER_LIMIT);
    assert(status.left_rpm > 40.0f && status.right_rpm < 40.0f);
    assert(-T_BRAKE_PWM == g_reg_left);       /* cut at once */
    assert(g_reg_right > -T_BRAKE_PWM);       /* only ramped down */
    assert((prior_right - g_reg_right) <= 251); /* 2 % of 12499 + 1 */
    puts("PASS motion runaway cut: wheel at 40.9 RPM jumps to the -25 % "
         "brake, wheel at 39.8 RPM only ramps 2 %");

    for (uint32_t i = 2U; i <= 9U; i++)
    {
        status = run_phase(TERRAIN_HUMP_DESCENDING, 1U, T_OVER_LIMIT,
                           T_UNDER_LIMIT);
        assert(0U == motion_downhill_fault());
        assert(status.left_target_rpm > 0.0f);
    }

    uint32_t const stops = g_stop_calls;

    status = run_phase(TERRAIN_HUMP_DESCENDING, 1U, T_OVER_LIMIT,
                       T_UNDER_LIMIT);
    assert(2U == motion_downhill_fault()); /* overspeed */
    assert((0U == status.left_pwm) && (0U == status.right_pwm));
    assert((0.0f == status.left_target_rpm) &&
           (0.0f == status.right_target_rpm));
    assert((g_stop_calls > stops) && (0 == g_reg_left) &&
           (0 == g_reg_right));
    puts("PASS motion overspeed: 9 runaway periods keep driving, the 10th "
         "latches the stop (fault 2)");

    uint32_t const regs = g_reg_calls;

    (void)run_phase(TERRAIN_HUMP_LEVEL, 30U, 0, 0);
    motion_drive(20.0f, 20.0f);
    assert((regs == g_reg_calls) && (0 == g_reg_left));
    assert(2U == motion_downhill_fault());
    motion_downhill_reset();
    assert(0U == motion_downhill_fault());
    puts("PASS motion overspeed: latched through 30 periods level and a "
         "bypass drive; cleared only by reset");
}

/** @brief Run the governor and production-motion cases. */
int main (void)
{
    test_runaway_limit();
    test_streak();
    assert(E_OK == motion_init());
    test_descent_gate();
    test_runaway_cut();
    return 0;
}

/*** end of file ***/

