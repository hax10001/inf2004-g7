/** @file motion.c
 * @brief Line-action steering with per-wheel encoder speed control.
 *
 * Steering turns the line action into a speed ratio between the wheels
 * instead of stopping one. A shared base speed ramps up from rest and the
 * steer is rate limited, so the wheels never jump, yet a turn is applied
 * from the first period after a start. A bounded PI trim on top of a scaled
 * feedforward duty makes each wheel reach its target, so mismatched motors
 * still turn at the commanded speeds.
 */
#include "motion.h"
#include "steering.h"
#include "wheel_speed.h"
#include "app/app_config.h"
#include "board/board_config.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include <tm/tmonitor.h>

/* Open-loop duty percent that gives MOTION_CAL_RPM. Measured on the track:
 * the speed loop settled near 21 and 22 percent at 40 RPM, so the trim
 * keeps headroom for steering instead of sitting at its limit.
 */
#define MOTION_LEFT_SPEED 21.0f
#define MOTION_RIGHT_SPEED 22.0f
/* The RPM the two duties above were measured at. Feedforward for any other
 * speed is interpolated from it, so MOTION_TARGET_RPM can change freely.
 */
#define MOTION_CAL_RPM 40.0f
/* Deadband: the duty a wheel needs before it turns at all.
 * WARNING: Uncalibrated. With APP_SPEED_CONTROL_ENABLED 0, lower the
 * matching speed above until that wheel stops, then use the last duty
 * that still turned it. Must stay below that wheel's speed.
 */
#define MOTION_LEFT_MIN_DUTY 8.0f
#define MOTION_RIGHT_MIN_DUTY 8.0f
#define MOTION_MAX_PERCENT 100.0f

/* Driving speed on a straight. 40 RPM is about 140 mm/s. */
#define MOTION_TARGET_RPM 20.0f
/* Largest change in the base speed per period, when starting or stopping. */
#define MOTION_RAMP_RPM 8.0f
/* Largest growth in steer per period; lower is smoother, but too low lags
 * on tight curves. Easing off or reversing is never delayed.
 */
#define MOTION_STEER_STEP 0.3f
/* Fraction of the speed shed at full steer: the inner wheel slows more than
 * the outer one speeds up, giving the sensors more time on a curve.
 */
#define MOTION_CURVE_SLOWDOWN 0.6f
/* Targets below this stop the wheel: a fifth of the driving speed, so only
 * the inner wheel of a turn near full steer gets this low. Driving it there
 * gave about 15 RPM instead of 4 at 40 RPM.
 */
#define MOTION_STOP_RPM (0.2f * MOTION_TARGET_RPM)
/* Duty percent per RPM of error, and per RPM-second of accumulated error. */
#define MOTION_KP 0.1f
#define MOTION_KI 0.5f
/* Bounds the trim so a miscalibrated target cannot run away. */
#define MOTION_TRIM_MAX_PERCENT 12.0f
/* Below this target the deadband dominates and the inner wheel of a hard turn
 * is dragged along, so the learned offset would be wrong at normal speed.
 */
#define MOTION_INTEGRATE_MIN_RPM 15.0f
#define MOTION_PERIOD_SEC ((float)APP_CONTROLLER_PERIOD_MSEC / 1000.0f)
#define MOTION_LOG_EVERY 10U
#define MOTION_PI 3.14159265f

typedef struct
{
    wheel_speed_t speed;
    float         integral;   /**< Learned duty offset, kept while stopped. */
    float         target_rpm; /**< Target actually commanded. */
    bool          b_running;
} motion_wheel_t;

static void  drive_wheels (INT action_code);
static float ramp_target (float current, float desired);

static motion_wheel_t g_wheels[ENCODER_WHEEL_COUNT];
static steering_t     g_steering;
static float          g_base_rpm = 0.0f; /**< Ramped speed of both wheels. */
static float          g_steer    = 0.0f; /**< Rate-limited steer applied. */
static uint32_t       g_log_countdown = 0U;

/* Only this module can directly access these variables. */
static ID volatile gh_status_mutex       = 0;
static motion_status_t volatile g_status = {0};

/** @brief Feedforward plus bounded PI trim for one wheel.
 * @param base_percent Open-loop duty; zero stops the wheel.
 * @return Duty percent to apply.
 */
static float wheel_command (motion_wheel_t * p_wheel, float base_percent,
                            float target_rpm, float measured_rpm);
/** @brief Open-loop duty for a target: deadband plus a line through the
 * calibrated duty at MOTION_CAL_RPM; zero below MOTION_STOP_RPM.
 */
static float feedforward (float target_rpm, float min_duty, float target_duty);
/** @brief Zero both targets and steering at once; restarts ramp from rest. */
static void stop_wheels (void);
/** @brief Move a value towards the desired value by at most one step. */
static float step_towards (float current, float desired, float step);
/** @brief Rate limit a growing turn; ease off or reverse at once. */
static float limit_steer (float current, float desired);
/** @brief Bound a value to plus or minus a limit. */
static float clamp_symmetric (float value, float limit);
/** @brief Convert a bounded percentage to the tested PWM compare count. */
static uint32_t speed_to_pwm (float speed_percent);
/** @brief Publish the latest status; failure only affects telemetry. */
static void publish_status (motion_status_t const * p_status);

ER motion_init (void)
{
    /* Motors are configured and zeroed first, whatever fails afterwards. */
    motor_init();

    T_CMTX mutex_config = {0};

    mutex_config.mtxatr = TA_INHERIT;

    ID h_mutex = tk_cre_mtx(&mutex_config);

    if (h_mutex <= E_OK)
    {
        return (E_OK == h_mutex) ? E_OBJ : h_mutex;
    }

    gh_status_mutex = h_mutex;

    ER error = encoder_init();

    if (error < E_OK)
    {
        return error;
    }

    for (uint32_t wheel = 0U; wheel < (uint32_t)ENCODER_WHEEL_COUNT; wheel++)
    {
        encoder_sample_t sample = {0};

        encoder_read((encoder_wheel_t)wheel, &sample);
        wheel_speed_reset(&g_wheels[wheel].speed, sample.count,
                          sample.edge_usec);
    }

    return E_OK;
}

ER motion_start (void)
{
    return E_OK;
}

void motion_apply (line_follow_action_t action)
{
    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[MOTOR REQUEST] Action=%s\n", line_follow_text(action));

    switch (action)
    {
        case LINE_FORWARD:
        case LINE_CORRECT_LEFT:
        case LINE_CORRECT_RIGHT:
            g_steer = limit_steer(g_steer, steering_update(&g_steering, action,
                                                           MOTION_PERIOD_SEC));
            g_base_rpm =
                step_towards(g_base_rpm, MOTION_TARGET_RPM, MOTION_RAMP_RPM);
        break;

        case LINE_UNCERTAIN:
            /* Stopping is not ramped: it is the safe response. */
            stop_wheels();
        break;

        default:
            stop_wheels();
            motor_stop();
            /* UB views the same bytes; the API does not modify them. */
            tm_printf((UB *)"[MOTOR] Uncertain action, stopping motors.\n");
            return;
    }

    motion_wheel_t * const p_left  = &g_wheels[ENCODER_LEFT];
    motion_wheel_t * const p_right = &g_wheels[ENCODER_RIGHT];

    /* Positive steer turns left by slowing the left wheel. Scaling the base
     * keeps the turn's sharpness the same while the speed changes.
     */
    float const magnitude = (g_steer < 0.0f) ? -g_steer : g_steer;
    float const speed_rpm =
        g_base_rpm * (1.0f - (MOTION_CURVE_SLOWDOWN * magnitude));

    p_left->target_rpm  = speed_rpm * (1.0f - g_steer);
    p_right->target_rpm = speed_rpm * (1.0f + g_steer);

    drive_wheels((INT)action);
}

static void drive_wheels (INT action_code)
{
    motion_wheel_t * const p_left  = &g_wheels[ENCODER_LEFT];
    motion_wheel_t * const p_right = &g_wheels[ENCODER_RIGHT];

    /* Measure the period that just ended, under the previous command. */
    uint32_t const   now    = encoder_now_usec();
    encoder_sample_t left   = {0};
    encoder_sample_t right  = {0};
    motion_status_t  status = {0};

    encoder_read(ENCODER_LEFT, &left);
    encoder_read(ENCODER_RIGHT, &right);

    status.left_rpm =
        wheel_speed_update(&p_left->speed, left.count, left.edge_usec, now);
    status.right_rpm =
        wheel_speed_update(&p_right->speed, right.count, right.edge_usec, now);
    status.left_target_rpm  = p_left->target_rpm;
    status.right_target_rpm = p_right->target_rpm;
    status.steer            = g_steer;

    float const left_feedforward = feedforward(
        p_left->target_rpm, MOTION_LEFT_MIN_DUTY, MOTION_LEFT_SPEED);
    float const right_feedforward = feedforward(
        p_right->target_rpm, MOTION_RIGHT_MIN_DUTY, MOTION_RIGHT_SPEED);

    status.left_pwm  = speed_to_pwm(wheel_command(
        p_left, left_feedforward, p_left->target_rpm, status.left_rpm));
    status.right_pwm = speed_to_pwm(wheel_command(
        p_right, right_feedforward, p_right->target_rpm, status.right_rpm));

    motor_set_forward(status.left_pwm, status.right_pwm);

    /* PWM counts are at most 12499, within INT; UB preserves string bytes. */
    tm_printf((UB *)"[MOTOR REQUEST] LEFT=%d RIGHT=%d\n", (INT)status.left_pwm,
              (INT)status.right_pwm);

    float const mm_per_rev = MOTION_PI * BOARD_WHEEL_DIAMETER_MM;

    status.speed_mm_per_sec =
        ((status.left_rpm + status.right_rpm) * 0.5f * mm_per_rev) / 60.0f;
    status.left_count     = left.count;
    status.right_count    = right.count;
    status.timestamp_usec = now;
    status.b_valid        = true;
    publish_status(&status);

#if APP_MOTION_DEBUG_CSV
    /* Columns: CSV,ms,action,steer%,targetL,targetR,rpmL x10,rpmR x10,
     * pwmL,pwmR. INT truncation is for display; UB preserves the bytes.
     */
    tm_printf((UB *)"CSV,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", (INT)(now / 1000U),
              (INT)action, (INT)(g_steer * 100.0f), (INT)status.left_target_rpm,
              (INT)status.right_target_rpm, (INT)(status.left_rpm * 10.0f),
              (INT)(status.right_rpm * 10.0f), (INT)status.left_pwm,
              (INT)status.right_pwm);
#endif

    if (0U == g_log_countdown)
    {
        g_log_countdown = MOTION_LOG_EVERY;
        /* STEER is in percent. Truncation to INT is for display only;
         * UB preserves the string bytes.
         */
        tm_printf((UB *)"[MOTOR SPEED] RPM L=%d R=%d TARGET L=%d R=%d "
                        "STEER=%d COUNT L=%d R=%d MM/S=%d\n",
                  (INT)status.left_rpm, (INT)status.right_rpm,
                  (INT)status.left_target_rpm, (INT)status.right_target_rpm,
                  (INT)(g_steer * 100.0f), (INT)left.count, (INT)right.count,
                  (INT)status.speed_mm_per_sec);
    }

    g_log_countdown--;
}

void motion_drive (float left_rpm, float right_rpm)
{
    motion_wheel_t * const p_left  = &g_wheels[ENCODER_LEFT];
    motion_wheel_t * const p_right = &g_wheels[ENCODER_RIGHT];

    p_left->target_rpm  = ramp_target(p_left->target_rpm, left_rpm);
    p_right->target_rpm = ramp_target(p_right->target_rpm, right_rpm);

    g_base_rpm = 0.5f * (p_left->target_rpm + p_right->target_rpm);
    g_steer    = 0.0f;
    steering_reset(&g_steering);

    drive_wheels(-1);
}

static float ramp_target (float current, float desired)
{
    return (desired > current) ? step_towards(current, desired, MOTION_RAMP_RPM)
                               : desired;
}

ER motion_get_status (motion_status_t * p_output)
{
    if (NULL == p_output)
    {
        return E_PAR;
    }

    if (gh_status_mutex <= E_OK)
    {
        return E_OBJ;
    }

    ER error = tk_loc_mtx(gh_status_mutex, TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    /* Consumers receive their own copy, never a view of live state. */
    *p_output = g_status;

    return tk_unl_mtx(gh_status_mutex);
}

static float wheel_command (motion_wheel_t * p_wheel, float base_percent,
                            float target_rpm, float measured_rpm)
{
    if (base_percent <= 0.0f)
    {
        /* Keep the integral: it is this motor's learned offset. */
        p_wheel->b_running = false;
        return 0.0f;
    }

    if (0 == APP_SPEED_CONTROL_ENABLED)
    {
        return base_percent;
    }

    if (!p_wheel->b_running)
    {
        /* The last period measured a stopped wheel; do not react to it. */
        p_wheel->b_running = true;
        return base_percent + p_wheel->integral;
    }

    float const error = target_rpm - measured_rpm;

    if (target_rpm >= MOTION_INTEGRATE_MIN_RPM)
    {
        p_wheel->integral = clamp_symmetric(
            p_wheel->integral + (MOTION_KI * error * MOTION_PERIOD_SEC),
            MOTION_TRIM_MAX_PERCENT);
    }

    return base_percent +
           clamp_symmetric((MOTION_KP * error) + p_wheel->integral,
                           MOTION_TRIM_MAX_PERCENT);
}

static float feedforward (float target_rpm, float min_duty, float target_duty)
{
    if (target_rpm < MOTION_STOP_RPM)
    {
        /* Too slow to hold: the inner wheel of a hard turn is dragged
         * along at about 15 RPM instead, so stop it and pivot instead.
         */
        return 0.0f;
    }

    return min_duty +
           (((target_duty - min_duty) * target_rpm) / MOTION_CAL_RPM);
}

static void stop_wheels (void)
{
    g_base_rpm = 0.0f;
    g_steer    = 0.0f;
    steering_reset(&g_steering);
}

static float step_towards (float current, float desired, float step)
{
    return current + clamp_symmetric(desired - current, step);
}

static float limit_steer (float current, float desired)
{
    float const current_size = (current < 0.0f) ? -current : current;
    float const desired_size = (desired < 0.0f) ? -desired : desired;

    if (desired_size <= current_size)
    {
        /* Easing off: a late recentre is what overshoots the line. */
        return desired;
    }

    if ((desired * current) < 0.0f)
    {
        /* Reversing: drop the old turn at once, then build the new one. */
        return clamp_symmetric(desired, MOTION_STEER_STEP);
    }

    return step_towards(current, desired, MOTION_STEER_STEP);
}

static float clamp_symmetric (float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

static uint32_t speed_to_pwm (float speed_percent)
{
    if (speed_percent <= 0.0f)
    {
        return 0U;
    }

    if (speed_percent > MOTION_MAX_PERCENT)
    {
        speed_percent = MOTION_MAX_PERCENT;
    }

    /* Truncation matches the original tested PWM counts. */
    return (uint32_t)(((float)MOTOR_PWM_WRAP * speed_percent) /
                      MOTION_MAX_PERCENT);
}

static void publish_status (motion_status_t const * p_status)
{
    if (gh_status_mutex <= E_OK)
    {
        return;
    }

    if (tk_loc_mtx(gh_status_mutex, TMO_FEVR) < E_OK)
    {
        return;
    }

    g_status = *p_status;
    (void)tk_unl_mtx(gh_status_mutex);
}

/*** end of file ***/

