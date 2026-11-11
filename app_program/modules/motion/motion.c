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
#include "downhill.h"
#include "steering.h"
#include "wheel_speed.h"
#include "app/app_config.h"
#include "board/board_config.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "modules/barcode_scanning/bc_wave.h"
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
#define MOTION_TARGET_RPM 15.0f
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
/* Bound drive/brake effort changes on descent and during its handoff. */
#define MOTION_DESCENT_DUTY_STEP 2.0f
/* Bounds the trim so a miscalibrated target cannot run away. */
#define MOTION_TRIM_MAX_PERCENT 12.0f
/* Below this target the deadband dominates and the inner wheel of a hard turn
 * is dragged along, so the learned offset would be wrong at normal speed.
 */
#define MOTION_INTEGRATE_MIN_RPM 15.0f
#define MOTION_PERIOD_SEC ((float)APP_CONTROLLER_PERIOD_MSEC / 1000.0f)
/* Stall rescue (see wheel_command): a wheel commanded to turn but measured
 * below MOTION_STALL_RPM gets this much more duty every period (1 % per
 * 50 ms, so +10 % after 0.5 s), up to the maximum; once it turns the boost
 * is removed at the decay rate. 15 % on top of ~13 % stays well short of
 * full power, so a jammed wheel is not driven hard. */
#define MOTION_STALL_RPM 1.0f
#define MOTION_STALL_STEP_PERCENT 1.0f
#define MOTION_STALL_MAX_PERCENT 15.0f
#define MOTION_STALL_DECAY_PERCENT 0.5f
#define MOTION_LOG_EVERY 10U
#define MOTION_PI 3.14159265f
/* Junction pivots: wheels in opposite directions, each wheel's duty
 * stepped every period so it moves MOTION_PIVOT_COUNTS per period whatever
 * the floor grip (16 counts per 50 ms is ~70 degrees/s: a U-turn ~2.6 s,
 * slow enough for the line sensors to catch the 18 mm tape). Angle, line
 * reacquisition and time limits are nav_core's. PWM counts of 12499. */
#define MOTION_PIVOT_COUNTS 16
#define MOTION_PIVOT_PWM_START 1250U /* 10 %, just above the deadband */
#define MOTION_PIVOT_PWM_STEP 125U
#define MOTION_PIVOT_PWM_MIN 750U
#define MOTION_PIVOT_PWM_MAX 3750U   /* 30 % */
/* Nonzero while pivoting: the larger wheel duty (motion_brake reads it). */
static uint32_t g_turn_pwm = 0U;
static uint32_t g_pivot_pwm[ENCODER_WHEEL_COUNT];
static int32_t g_pivot_last[ENCODER_WHEEL_COUNT];
static uint8_t g_pivot_action = 0U;
/* Stop periods left to hold the short brake before releasing to coast. */
#define MOTION_BRAKE_CYCLES \
    (APP_JUNCTION_BRAKE_MSEC / APP_CONTROLLER_PERIOD_MSEC)
static uint32_t g_brake_cycles = 0U;

typedef struct
{
    wheel_speed_t speed;
    float         integral;   /**< Learned duty offset, kept while stopped. */
    float         stall_boost; /**< Extra duty while commanded but stuck. */
    float         target_rpm; /**< Target actually commanded. */
    /* Controller-owned. Signed PWM -3124..12499 needs 15 bits. These
     * fields occupy the old boolean's word, adding no wheel storage. */
    signed int    effort_pwm:15;
    unsigned int  b_running:1;
    unsigned int  b_regulating:1;
} motion_wheel_t;

static motion_wheel_t g_wheels[ENCODER_WHEEL_COUNT];
static steering_t     g_steering;
static float          g_base_rpm = 0.0f; /**< Ramped speed of both wheels. */
static float          g_steer    = 0.0f; /**< Rate-limited steer applied. */
static uint32_t       g_log_countdown = 0U;
/* Controller task is the sole writer; no shared RMW or extra mutex. */
static downhill_governor_t g_downhill = {0};
/* A bounded brake pulse, never indefinite H-bridge shorting on a fault. */
static struct
{
    unsigned int remaining:3;
} g_downhill_brake = {0};
#if MOTION_BRAKE_CYCLES > 7U
#error Downhill brake counter requires at most 3 bits
#endif

/* Only this module can directly access these variables. */
static ID volatile              gh_status_mutex = 0;
static motion_status_t volatile g_status        = {0};

/** @brief Feedforward plus bounded PI trim for one wheel.
 * @param base_percent Open-loop duty; zero stops the wheel.
 * @return Duty percent to apply.
 */
static float wheel_command (motion_wheel_t * p_wheel, float base_percent,
                            float target_rpm, float measured_rpm);
/** @brief Open-loop duty for a target: deadband plus a line through the
 * calibrated duty at MOTION_CAL_RPM; zero below MOTION_STOP_RPM.
 */
static float feedforward (float target_rpm, float min_duty,
                          float target_duty);
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
/** @brief Apply wheel targets through the existing speed controller. */
static void drive_wheels (line_follow_action_t action);
/** @brief Timed brake then coast while the governor fault remains latched. */
static void downhill_fault_stop (void);

ER motion_init (void)
{
    downhill_reset(&g_downhill);
    g_downhill_brake.remaining = 0U;
    /* Motors are configured and zeroed first, whatever fails afterwards. */
    motor_init();
#if APP_DOWNHILL_NOSE_DOWN_SIGN == 0
    tm_printf((UB *)"[RAMP] IMU descent control ON; pitch sign unverified\n");
#else
    tm_printf((UB *)"[RAMP] Governor enabled, nose-down sign=%d\n",
              (INT)APP_DOWNHILL_NOSE_DOWN_SIGN);
#endif

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
    if (g_downhill.fault)
    {
        downhill_fault_stop();
        return;
    }
/* Barcode processing never owns the motor; only an explicit STOP,
 * safety fault, or controller navigation state may inhibit movement. */
    g_turn_pwm = 0U;
#if !APP_MOTOR_MOVEMENT_ENABLED
    /* Keep targets and telemetry stopped as well as the driver outputs. */
    action = LINE_UNCERTAIN;
#endif
    /* UB views the same string bytes; tm_printf does not modify them. */
    //tm_printf((UB *)"[MOTOR REQUEST] Action=%s\n", line_follow_text(action));

    /* Keep the original improve-movement 15 RPM control target. Barcode
     * waveform diagnostics must not silently derate the drive to 10 RPM.
     * Sensor sampling uses encoder distance, not this velocity setting. */
    switch (action)
    {
        case LINE_FORWARD:
        case LINE_CORRECT_LEFT:
        case LINE_CORRECT_RIGHT:
            g_brake_cycles = 0U;
            g_steer = limit_steer(
                g_steer,
                steering_update(&g_steering, action, MOTION_PERIOD_SEC));
            g_base_rpm =
                step_towards(g_base_rpm,
                    bc_wave_capture_active() ? APP_BARCODE_SCAN_RPM :
                    MOTION_TARGET_RPM, MOTION_RAMP_RPM);
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

    /* Equal multiplicative governor after steering preserves left/right
     * wheel proportions. No other task can write the motor duty. */
    downhill_scale_pair(&g_downhill, p_left->target_rpm,
                        p_right->target_rpm, &p_left->target_rpm,
                        &p_right->target_rpm);

    drive_wheels(action);
}

void motion_brake (void)
{
#if APP_MOTOR_MOVEMENT_ENABLED
    /* Only a robot that was driving needs braking; a repeat request while
     * already braking or stopped must not restart the hold. */
    if ((g_base_rpm > 0.0f) || (0U != g_turn_pwm))
    {
        g_brake_cycles = MOTION_BRAKE_CYCLES;
    }
#endif
    motion_apply(LINE_UNCERTAIN);
}

void motion_stop_now(void)
{
    /* Operator STOP has priority over a previous timed brake pulse too. */
    motor_stop();
    g_brake_cycles = 0U;
    g_downhill_brake.remaining = 0U;
    stop_wheels();
    g_wheels[ENCODER_LEFT].target_rpm = 0.0f;
    g_wheels[ENCODER_RIGHT].target_rpm = 0.0f;
    g_turn_pwm = 0U;
    /* Measure coasting wheels after the zero write; do not invent a zero
     * measured speed merely because the commanded target became zero. */
    drive_wheels(LINE_UNCERTAIN);
}

void motion_drive (float left_rpm, float right_rpm)
{
    if (g_downhill.fault)
    {
        downhill_fault_stop();
        return;
    }
    motion_wheel_t * const p_left = &g_wheels[ENCODER_LEFT];
    motion_wheel_t * const p_right = &g_wheels[ENCODER_RIGHT];
#if !APP_MOTOR_MOVEMENT_ENABLED
    motion_apply(LINE_UNCERTAIN);
    return;
#endif
    /* Bypass forward motion receives the same downhill restriction. */
    downhill_scale_pair(&g_downhill, left_rpm, right_rpm,
                        &left_rpm, &right_rpm);
    /* Obstacle arcs use forward motion. Braking is never ramped. */
    if (left_rpm < 0.0f) left_rpm = 0.0f;
    if (right_rpm < 0.0f) right_rpm = 0.0f;
    if ((left_rpm > 0.0f) || (right_rpm > 0.0f)) g_brake_cycles = 0U;
    p_left->target_rpm = (left_rpm > p_left->target_rpm) ?
        step_towards(p_left->target_rpm, left_rpm, MOTION_RAMP_RPM) : left_rpm;
    p_right->target_rpm = (right_rpm > p_right->target_rpm) ?
        step_towards(p_right->target_rpm, right_rpm, MOTION_RAMP_RPM) :
        right_rpm;
    g_base_rpm = 0.5f * (p_left->target_rpm + p_right->target_rpm);
    g_steer = 0.0f;
    g_turn_pwm = 0U;
    steering_reset(&g_steering);
    drive_wheels((left_rpm == 0.0f && right_rpm == 0.0f) ?
                 LINE_UNCERTAIN : LINE_FORWARD);
}

static void drive_wheels (line_follow_action_t action)
{
    motion_wheel_t * const p_left = &g_wheels[ENCODER_LEFT];
    motion_wheel_t * const p_right = &g_wheels[ENCODER_RIGHT];

    /* Measure the period that just ended, under the previous command. */
    uint32_t const   now     = encoder_now_usec();
    encoder_sample_t left    = {0};
    encoder_sample_t right   = {0};
    motion_status_t  status  = {0};

    encoder_read(ENCODER_LEFT, &left);
    encoder_read(ENCODER_RIGHT, &right);

    status.left_rpm  = wheel_speed_update(&p_left->speed, left.count,
                                          left.edge_usec, now);
    status.right_rpm = wheel_speed_update(&p_right->speed, right.count,
                                          right.edge_usec, now);
    status.left_target_rpm  = p_left->target_rpm;
    status.right_target_rpm = p_right->target_rpm;
    status.steer            = g_steer;

    if (action != LINE_UNCERTAIN &&
        downhill_observe_speed(&g_downhill,
                               p_left->target_rpm, p_right->target_rpm,
                               status.left_rpm, status.right_rpm))
    {
        /* Fail closed before PI or stall rescue can increase duty. */
        g_downhill_brake.remaining = MOTION_BRAKE_CYCLES;
        downhill_fault_stop();
        p_left->target_rpm = 0.0f;
        p_right->target_rpm = 0.0f;
        status.left_target_rpm = 0.0f;
        status.right_target_rpm = 0.0f;
        status.left_pwm = 0U;
        status.right_pwm = 0U;
        status.timestamp_usec = now;
        status.left_count = left.count;
        status.right_count = right.count;
        status.b_valid = true;
        publish_status(&status);
        tm_printf((UB *)"[RAMP] Overspeed -> latched STOP\n");
        return;
    }

    if (action == LINE_UNCERTAIN)
    {
        /* The PI loop is not an emergency-stop implementation.  Apply
         * physical zero immediately and publish *commanded* zero duty,
         * even when an encoder still measures coasting motion. */
        status.left_pwm = 0U;
        status.right_pwm = 0U;

        if (g_brake_cycles > 0U)
        {
            g_brake_cycles--;
            motor_brake();
        }
        else
        {
            motor_stop();
        }
    }
    else
    {
        float const left_feedforward = feedforward(
            p_left->target_rpm, MOTION_LEFT_MIN_DUTY, MOTION_LEFT_SPEED);
        float const right_feedforward = feedforward(
            p_right->target_rpm, MOTION_RIGHT_MIN_DUTY, MOTION_RIGHT_SPEED);
        float lp = wheel_command(p_left, left_feedforward,
                                p_left->target_rpm, status.left_rpm);
        float rp = wheel_command(p_right, right_feedforward,
                                p_right->target_rpm, status.right_rpm);
        /* Preserve the existing gross-runaway inhibit before its latch. */
        if (downhill_wheel_over_target(&g_downhill,
                    p_left->target_rpm, status.left_rpm)) lp = -25.0f;
        if (downhill_wheel_over_target(&g_downhill,
                    p_right->target_rpm, status.right_rpm)) rp = -25.0f;
        status.left_pwm = speed_to_pwm(lp < 0.0f ? -lp : lp);
        status.right_pwm = speed_to_pwm(rp < 0.0f ? -rp : rp);
        p_left->effort_pwm = lp < 0.0f ? -(int)status.left_pwm :
                                         (int)status.left_pwm;
        p_right->effort_pwm = rp < 0.0f ? -(int)status.right_pwm :
                                          (int)status.right_pwm;
        /* Positive regulated PWM is the existing forward pin pattern.
         * Keep bounded braking available while returning to level too. */
        motor_set_regulated(lp < 0.0f ? -(int32_t)status.left_pwm :
                                              (int32_t)status.left_pwm,
                                rp < 0.0f ? -(int32_t)status.right_pwm :
                                              (int32_t)status.right_pwm);
    }

    /* PWM counts are at most 12499, within INT; UB preserves string bytes.
    tm_printf((UB *)"[MOTOR REQUEST] LEFT=%d RIGHT=%d\n", (INT)status.left_pwm,
               (INT)status.right_pwm);*/

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
              (INT)action, (INT)(g_steer * 100.0f),
              (INT)status.left_target_rpm, (INT)status.right_target_rpm,
              (INT)(status.left_rpm * 10.0f), (INT)(status.right_rpm * 10.0f),
              (INT)status.left_pwm, (INT)status.right_pwm);
#endif

    if (0U == g_log_countdown)
    {
        g_log_countdown = MOTION_LOG_EVERY;
        /* STEER is in percent. Truncation to INT is for display only;
         * UB preserves the string bytes.
         */
        /*
        tm_printf((UB *)"[MOTOR SPEED] RPM L=%d R=%d TARGET L=%d R=%d "
                        "STEER=%d COUNT L=%d R=%d MM/S=%d\n",
                  (INT)status.left_rpm, (INT)status.right_rpm,
                  (INT)status.left_target_rpm, (INT)status.right_target_rpm,
                  (INT)(g_steer * 100.0f), (INT)left.count, (INT)right.count,
                  (INT)status.speed_mm_per_sec);
                  */
    }

    g_log_countdown--;
}

void motion_apply_maneuver (uint8_t mapped_action)
{
    if (g_downhill.fault)
    {
        downhill_fault_stop();
        return;
    }
    motion_status_t status = {0};
    encoder_sample_t left = {0};
    encoder_sample_t right = {0};
    int32_t left_pwm = 0;
    int32_t right_pwm = 0;
    stop_wheels();
    g_brake_cycles = 0U;
#if !APP_MOTOR_MOVEMENT_ENABLED
    g_turn_pwm = 0U;
    motor_stop();
    return;
#endif
    if (mapped_action < 1U || mapped_action > 3U)
    {
        g_turn_pwm = 0U;
        motor_stop();
        return;
    }
    encoder_read(ENCODER_LEFT, &left);
    encoder_read(ENCODER_RIGHT, &right);
    if ((0U == g_turn_pwm) || (mapped_action != g_pivot_action))
    {
        /* New pivot: start both wheels just above the deadband. */
        g_pivot_action = mapped_action;
        g_pivot_pwm[ENCODER_LEFT] = MOTION_PIVOT_PWM_START;
        g_pivot_pwm[ENCODER_RIGHT] = MOTION_PIVOT_PWM_START;
    }
    else
    {
        /* Speed each wheel up or down towards the target movement. */
        int32_t const moved[ENCODER_WHEEL_COUNT] = {
            left.count - g_pivot_last[ENCODER_LEFT],
            right.count - g_pivot_last[ENCODER_RIGHT]};
        for (uint32_t w = 0U; w < (uint32_t)ENCODER_WHEEL_COUNT; w++)
        {
            int32_t const m = (moved[w] < 0) ? -moved[w] : moved[w];
            if (m < MOTION_PIVOT_COUNTS)
                g_pivot_pwm[w] += MOTION_PIVOT_PWM_STEP;
            else if (m > MOTION_PIVOT_COUNTS + (MOTION_PIVOT_COUNTS / 4))
                g_pivot_pwm[w] -= MOTION_PIVOT_PWM_STEP;
            if (g_pivot_pwm[w] > MOTION_PIVOT_PWM_MAX)
                g_pivot_pwm[w] = MOTION_PIVOT_PWM_MAX;
            if (g_pivot_pwm[w] < MOTION_PIVOT_PWM_MIN)
                g_pivot_pwm[w] = MOTION_PIVOT_PWM_MIN;
        }
    }
    g_pivot_last[ENCODER_LEFT] = left.count;
    g_pivot_last[ENCODER_RIGHT] = right.count;
    left_pwm = (int32_t)g_pivot_pwm[ENCODER_LEFT];
    right_pwm = (int32_t)g_pivot_pwm[ENCODER_RIGHT];
    g_turn_pwm = (left_pwm > right_pwm) ? (uint32_t)left_pwm :
                                           (uint32_t)right_pwm;
    /* Left and U-turn spin anticlockwise (left wheel back), right turns
     * clockwise (right wheel back). */
    if (mapped_action == 2U) right_pwm = -right_pwm;
    else left_pwm = -left_pwm;
    motor_set_directional(left_pwm, right_pwm);
    status.left_count = left.count;
    status.right_count = right.count;
    status.left_pwm = (uint32_t)(left_pwm < 0 ? -left_pwm : left_pwm);
    status.right_pwm = (uint32_t)(right_pwm < 0 ? -right_pwm : right_pwm);
    status.timestamp_usec = encoder_now_usec();
    status.b_valid = true;
    publish_status(&status);
}

static void report_downhill_change (unsigned int before,
                                    unsigned int fault_before)
{
    if (!before && g_downhill.engaged)
    {
        tm_printf((UB *)"[RAMP] DOWNHILL -> 65%% wheel targets\n");
    }
    else if (before && !g_downhill.engaged)
    {
        tm_printf((UB *)"[RAMP] LEVEL -> restoring speed gradually\n");
    }
    if (!fault_before && g_downhill.fault)
    {
        /* Controller will enter a latched safety stop this same cycle. */
        tm_printf((UB *)"[RAMP] Invalid IMU -> latched STOP\n");
    }
}

void motion_downhill_sample (float pitch_deg, float roll_deg,
                             bool fresh, bool driving)
{
    unsigned int const before = g_downhill.engaged;
    unsigned int const fault_before = g_downhill.fault;
    downhill_sample(&g_downhill, pitch_deg, roll_deg, fresh, driving);
    report_downhill_change(before, fault_before);
}

bool motion_downhill_faulted (void)
{
    return g_downhill.fault != 0U;
}

void motion_downhill_reset (void)
{
    downhill_reset(&g_downhill);
    g_downhill_brake.remaining = 0U;
}

void motion_reset_run(void)
{
    motion_downhill_reset();
    for (uint32_t w = 0U; w < ENCODER_WHEEL_COUNT; ++w)
    {
        g_wheels[w].integral = 0.0f;
        g_wheels[w].stall_boost = 0.0f;
        g_wheels[w].b_running = false;
        g_wheels[w].b_regulating = false;
        g_wheels[w].effort_pwm = 0;
    }
}

void motion_observe_terrain(terrain_snapshot const *s, uint32_t now)
{
    bool fresh = s->b_valid &&
        (uint32_t)(now - s->timestamp_msec) <= 200U;
    unsigned int const before = g_downhill.engaged;
    unsigned int const fault_before = g_downhill.fault;
    downhill_sample_terrain(&g_downhill, s->pitch_deg, s->roll_deg,
        fresh, true, s->height_valid && s->onHump &&
                     s->hump_phase == TERRAIN_HUMP_DESCENDING);
    report_downhill_change(before, fault_before);
}

uint8_t motion_downhill_fault(void)
{
    return g_downhill.fault ? (g_downhill.overspeed >= 10U ? 2U : 1U) : 0U;
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
    bool const ramp_control = g_downhill.scale_percent < 100U;
    if (target_rpm < MOTION_STOP_RPM)
    {
        /* Keep the integral: it is this motor's learned offset. */
        p_wheel->b_running = false;
        p_wheel->b_regulating = false;
        p_wheel->stall_boost = 0.0f;
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
        return base_percent + p_wheel->integral + p_wheel->stall_boost;
    }

    /* Stall rescue. On a tight curve the outer wheel is asked for ~12 RPM
     * with the inner wheel stopped; that is below the integrating range,
     * so its ~13 % duty could sit there forever without turning it (logged:
     * both encoders frozen ~7 s on a curve, still "following"). While a
     * commanded wheel is not turning, raise its duty step by step until it
     * moves; once it turns, the boost is taken back off gradually. */
    if (measured_rpm < MOTION_STALL_RPM && target_rpm >= MOTION_STOP_RPM)
    {
        p_wheel->stall_boost += MOTION_STALL_STEP_PERCENT;
        if (p_wheel->stall_boost >= MOTION_STALL_MAX_PERCENT)
        {
            if (p_wheel->stall_boost < (MOTION_STALL_MAX_PERCENT +
                                        MOTION_STALL_STEP_PERCENT))
            {
                /* Once per stall: the boost has just reached its limit. */
                tm_printf((UB *)"[MOTOR] Wheel stalled: target %d RPM, "
                          "boost at max %d %%\n", (INT)target_rpm,
                          (INT)MOTION_STALL_MAX_PERCENT);
            }
            p_wheel->stall_boost = MOTION_STALL_MAX_PERCENT;
        }
    }
    else if (p_wheel->stall_boost > 0.0f)
    {
        p_wheel->stall_boost -= MOTION_STALL_DECAY_PERCENT;
        if (p_wheel->stall_boost < 0.0f)
        {
            p_wheel->stall_boost = 0.0f;
        }
    }

    float const error = target_rpm - measured_rpm;

    if (ramp_control)
    {
        /* At 65% of 15 RPM the old >=15 RPM gate disabled integration.
         * Negative effort uses bounded short-brake/coast PWM, not reverse.
         * Conditional integration prevents saturation windup. */
        float candidate = p_wheel->integral +
                          MOTION_KI * error * MOTION_PERIOD_SEC;
        float effort = base_percent + MOTION_KP * error + candidate;
        if ((effort >= -25.0f && effort <= 100.0f) ||
            (effort < -25.0f && error > 0.0f) ||
            (effort > 100.0f && error < 0.0f))
            p_wheel->integral = clamp_symmetric(candidate, 40.0f);
        effort = base_percent + MOTION_KP * error + p_wheel->integral;
        if (effort < -25.0f) effort = -25.0f;
        if (effort > 100.0f) effort = 100.0f;
        p_wheel->b_regulating = true;
        return step_towards((float)p_wheel->effort_pwm * 100.0f /
                             (float)MOTOR_PWM_WRAP,
                             effort, MOTION_DESCENT_DUTY_STEP);
    }

    if (target_rpm >= MOTION_INTEGRATE_MIN_RPM)
    {
        p_wheel->integral = clamp_symmetric(
            p_wheel->integral + (MOTION_KI * error * MOTION_PERIOD_SEC),
            MOTION_TRIM_MAX_PERCENT);
    }

    float const effort = base_percent +
           clamp_symmetric((MOTION_KP * error) + p_wheel->integral,
                           MOTION_TRIM_MAX_PERCENT) +
           p_wheel->stall_boost;
    if (p_wheel->b_regulating)
    {
        float const prior = (float)p_wheel->effort_pwm * 100.0f /
                             (float)MOTOR_PWM_WRAP;
        float const change = effort - prior;
        p_wheel->b_regulating = change > MOTION_DESCENT_DUTY_STEP ||
                                change < -MOTION_DESCENT_DUTY_STEP;
        return step_towards(prior, effort, MOTION_DESCENT_DUTY_STEP);
    }
    return effort;
}

static float feedforward (float target_rpm, float min_duty,
                          float target_duty)
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

static void downhill_fault_stop (void)
{
    stop_wheels();
    if (g_downhill_brake.remaining > 0U)
    {
        --g_downhill_brake.remaining;
        motor_brake();
    }
    else
    {
        motor_stop();
    }
}

static void stop_wheels (void)
{
    g_base_rpm = 0.0f;
    g_steer    = 0.0f;
    steering_reset(&g_steering);
    for (uint32_t w = 0U; w < ENCODER_WHEEL_COUNT; ++w)
    {
        g_wheels[w].effort_pwm = 0;
        g_wheels[w].b_regulating = false;
    }
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

    motion_status_t snapshot = *p_status;
    snapshot.downhill_percent = g_downhill.scale_percent;
    snapshot.downhill_engaged = g_downhill.engaged;
    snapshot.downhill_fault = motion_downhill_fault();
    snapshot.downhill_pitch_x10 = g_downhill.filtered_pitch_x10;
    g_status = snapshot;
    (void)tk_unl_mtx(gh_status_mutex);
}

/*** end of file ***/

