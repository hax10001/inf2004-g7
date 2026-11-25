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
#include <tm/tmonitor.h>

/* Open-loop duty percent that gives MOTION_CAL_RPM, as measured on the
 * track, so the trim keeps headroom for steering.
 */
#define MOTION_LEFT_SPEED 21.0f
#define MOTION_RIGHT_SPEED 22.0f
/* The RPM the two duties above were measured at. Feedforward for any other
 * speed is interpolated from it, so MOTION_TARGET_RPM can change freely.
 */
#define MOTION_CAL_RPM 40.0f
/* Deadband: duty percent a wheel needs before it turns at all. Must stay
 * below that wheel's MOTION_*_SPEED.
 */
#define MOTION_LEFT_MIN_DUTY 8.0f
#define MOTION_RIGHT_MIN_DUTY 8.0f
#define MOTION_MAX_PERCENT 100.0f /* Full duty, percent. */

/* Driving speed on a straight, RPM (40 RPM is about 140 mm/s). Steering
 * scales with it (STEERING_REF_RPM). Higher speeds miss barcodes.
 */
#ifndef MOTION_TARGET_RPM
#define MOTION_TARGET_RPM 30.0f
#endif
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
 * the inner wheel near full steer gets this low. The motor cannot hold
 * such a low speed and would be dragged along instead.
 */
#define MOTION_STOP_RPM (0.2f * MOTION_TARGET_RPM)
/* Duty percent per RPM of error, and per RPM-second of accumulated error. */
#define MOTION_KP 0.1f
#define MOTION_KI 0.5f
/* Largest duty change per period (percent) on a descent and its handoff. */
#define MOTION_DESCENT_DUTY_STEP 2.0f
/* Trim bound, percent, so a miscalibrated target cannot run away. */
#define MOTION_TRIM_MAX_PERCENT 12.0f
/* Below this target the deadband dominates and the inner wheel of a hard turn
 * is dragged along, so the learned offset would be wrong at normal speed.
 */
#define MOTION_INTEGRATE_MIN_RPM 15.0f
#define MOTION_PERIOD_SEC ((float)APP_CONTROLLER_PERIOD_MSEC / 1000.0f)
/* Stall rescue (see wheel_command): a wheel commanded to turn but measured
 * below MOTION_STALL_RPM gains MOTION_STALL_STEP_PERCENT duty per period up
 * to the maximum; once it turns the boost decays. The 15 % cap keeps a
 * jammed wheel well short of full power.
 */
#define MOTION_STALL_RPM 1.0f
#define MOTION_STALL_STEP_PERCENT 1.0f
#define MOTION_STALL_MAX_PERCENT 15.0f
#define MOTION_STALL_DECAY_PERCENT 0.5f
#define MOTION_LOG_EVERY 10U /* Periods between status log slots. */
#define MOTION_PI 3.14159265f
/* Wheel travel per encoder count: a 70 mm wheel over 1056 counts. */
#define MOTION_MM_PER_COUNT \
    ((MOTION_PI * BOARD_WHEEL_DIAMETER_MM) / (float)BOARD_ENC_COUNTS_PER_REV)
/* Junction pivots: wheels in opposite directions, each wheel's duty
 * stepped every period to hold MOTION_PIVOT_COUNTS per period whatever the
 * floor grip (~70 degrees/s, slow enough for the line sensors to catch the
 * 18 mm tape). Angle and time limits are nav_core's. PWM counts of 12499.
 */
#define MOTION_PIVOT_COUNTS 16
#define MOTION_PIVOT_PWM_START 1250U /* 10 %, just above the deadband */
#define MOTION_PIVOT_PWM_STEP 125U /* 1 % per period. */
#define MOTION_PIVOT_PWM_MIN 750U  /* 6 %. */
#define MOTION_PIVOT_PWM_MAX 3750U   /* 30 % */
/* Ramp (downhill) wheel effort limits, percent of full PWM: negative
 * effort is bounded short-brake, never reverse. The integral is clamped
 * so it cannot wind up beyond what the limits can use. */
#define MOTION_RAMP_EFFORT_MIN (-25.0f)
#define MOTION_RAMP_EFFORT_MAX 100.0f
#define MOTION_RAMP_INTEGRAL_MAX 40.0f
/* An IMU snapshot older than this does not drive the downhill governor. */
#define MOTION_IMU_FRESH_MSEC 200U
/* Nonzero while pivoting: the larger wheel duty (motion_brake reads it).
 * Pivot duties stay within MOTION_PIVOT_PWM_MIN..MAX, so 16 bits hold them.
 */
#if (MOTION_PIVOT_PWM_MAX + MOTION_PIVOT_PWM_STEP) > 65535U
#error Pivot PWM no longer fits 16 bits
#endif
#if MOTION_PIVOT_PWM_MIN < MOTION_PIVOT_PWM_STEP
#error Pivot PWM minimum must be at least one step
#endif
static uint16_t g_turn_pwm = 0U;
static uint16_t g_pivot_pwm[ENCODER_WHEEL_COUNT];
static int32_t g_pivot_last[ENCODER_WHEEL_COUNT];
static uint8_t g_pivot_action = 0U;
/* Stop periods left to hold the short brake before releasing to coast. */
#define MOTION_BRAKE_CYCLES \
    (APP_JUNCTION_BRAKE_MSEC / APP_CONTROLLER_PERIOD_MSEC)
#if MOTION_BRAKE_CYCLES > 255U
#error Brake hold no longer fits its 8-bit counter
#endif
static uint8_t g_brake_cycles = 0U;

/* Lost-line search. Two sensors that straddle the line both read white
 * whether the line is between them or has been left behind, so travel
 * tells them apart: after MOTION_SEARCH_AFTER_MM with no sensor on the
 * line, the robot pivots towards where it last saw it, up to
 * MOTION_SEARCH_SWEEP_DEG, then the same past the start the other way, and
 * stops if it still has not found it. A sharp corner is the usual cause:
 * the inside sensor crosses the new arm for only one tape width. Uses the
 * junction pivot (MOTION_PIVOT_*).
 */
#define MOTION_SEARCH_AFTER_MM 100U
#define MOTION_SEARCH_SWEEP_DEG 120U
/* Rolling check before that pivot. On a straight the line is usually
 * still between the sensors (26 mm apart over 18 mm tape, ~4 mm clear each
 * side), so stopping to pivot would only make the robot jitter. After
 * MOTION_SEARCH_ROLL_MM (mm) with no sensor on the line the robot keeps
 * driving and steers by MOTION_SEARCH_ROLL_STEER towards the line's last
 * side; only at MOTION_SEARCH_AFTER_MM does it stop and pivot.
 */
#ifndef MOTION_SEARCH_ROLL_MM
#define MOTION_SEARCH_ROLL_MM 50U
#endif
#ifndef MOTION_SEARCH_ROLL_STEER
#define MOTION_SEARCH_ROLL_STEER 0.2f
#endif
#if MOTION_SEARCH_ROLL_MM >= MOTION_SEARCH_AFTER_MM
#error The rolling check must start before the pivot search
#endif
/* Both wheels' counts: 1056 per revolution over 219.8 mm (app_config.h). */
#define MOTION_SEARCH_AFTER_COUNTS \
    ((MOTION_SEARCH_AFTER_MM * BOARD_ENC_COUNTS_PER_REV * 20U) / 2198U)
#define MOTION_SEARCH_ROLL_COUNTS \
    ((MOTION_SEARCH_ROLL_MM * BOARD_ENC_COUNTS_PER_REV * 20U) / 2198U)
#define MOTION_SEARCH_SWEEP_COUNTS \
    ((APP_NAV_TURN90_COUNTS * MOTION_SEARCH_SWEEP_DEG) / 90U)
#if (MOTION_SEARCH_AFTER_COUNTS > 65535U) || \
    ((2U * MOTION_SEARCH_SWEEP_COUNTS) > 65535U)
#error Lost-line search counts no longer fit 16 bits
#endif
/* Lost-line search phases (line_search_t.phase). */
#define SEARCH_FOLLOWING 0U
#define SEARCH_FIRST_SWEEP 1U
#define SEARCH_SWEEP_BACK 2U
#define SEARCH_GAVE_UP 3U
#define SEARCH_ROLLING 4U
/* Pivot directions for pivot_step(). */
#define PIVOT_LEFT 1U
#define PIVOT_RIGHT 2U
#define PIVOT_UTURN 3U
/* Pivot duty eases off above the target count plus a quarter of it. */
#define MOTION_PIVOT_SLACK_DIV 4
/* Degrees in one APP_NAV_TURN90_COUNTS pivot. */
#define MOTION_TURN90_DEG 90U
/* Largest uint16_t value. */
#define MOTION_U16_MAX 65535U
/* Unit conversions for status and the debug CSV. */
#define MOTION_SEC_PER_MIN 60.0f
#define MOTION_USEC_PER_MSEC 1000U
#define MOTION_PERCENT_SCALE 100.0f
#define MOTION_X10 10.0f
/* motion_downhill_fault() values: stale/invalid IMU, overspeed. */
#define MOTION_DOWNHILL_FAULT_IMU 1U
#define MOTION_DOWNHILL_FAULT_OVERSPEED 2U

/* Controller-task only, like the rest of this module's state. */
typedef struct
{
    int32_t      last_count[ENCODER_WHEEL_COUNT]; /**< At the last period. */
    uint16_t     white_counts; /**< Travel with no line seen (saturates). */
    uint16_t     swept_counts; /**< Pivoted in the current sweep. */
    int32_t      side:2;       /**< Line last seen: +1 left, -1 right, 0 no. */
    int32_t      seen_now:2;   /**< This period: one sensor's side, else 0. */
    uint32_t     b_black:1;    /**< A sensor saw black this period. */
    uint32_t     b_observed:1; /**< motion_line_seen() ran this period. */
    uint32_t     phase:3;      /**< SEARCH_*. */
    uint32_t     b_counts:1;   /**< last_count holds a reading. */
} line_search_t;

static line_search_t g_search = {{0, 0}, 0U, 0U, 0, 0, 0U, 0U, 0U, 0U};

typedef struct
{
    wheel_speed_t speed;
    float         integral;   /**< Learned duty offset, kept while stopped. */
    float         stall_boost; /**< Extra duty while commanded but stuck. */
    float         target_rpm; /**< Target actually commanded. */
    /* Controller-owned. Signed PWM -3124..12499 needs 15 bits; the flags
     * share the same word. */
    int32_t       effort_pwm:15;
    uint32_t      b_running:1;
    uint32_t      b_regulating:1;
} motion_wheel_t;

static motion_wheel_t g_wheels[ENCODER_WHEEL_COUNT];
static steering_t     g_steering;
/* Line observer for STEERING_OBSERVER; controller task only. */
static line_track_t   g_track = {0.0f, 0.0f, 0.0f, 0.0f};
/* Each wheel's encoder counts over the last period (search_travel()). At
 * most ~60 per 50 ms period at 65 RPM, so 16 bits hold them. */
static int16_t        g_period_counts[ENCODER_WHEEL_COUNT] = {0, 0};
static float          g_base_rpm = 0.0f; /**< Ramped speed of both wheels. */
static float          g_steer    = 0.0f; /**< Rate-limited steer applied. */
static uint8_t        g_log_countdown = 0U; /**< 0..MOTION_LOG_EVERY. */
/* Controller task is the sole writer; no shared RMW or extra mutex. */
static downhill_governor_t g_downhill = {0};
/* A bounded brake pulse, never indefinite H-bridge shorting on a fault. */
static struct
{
    uint32_t     remaining:3;
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
/** @brief One period of an on-the-spot pivot (PIVOT_LEFT, PIVOT_RIGHT, or 3
 *  for a U-turn), each wheel's duty stepped to hold MOTION_PIVOT_COUNTS. */
static void pivot_step (uint8_t mapped_action);
/** @brief Run the lost-line search for this period.
 * @return true if it drove the wheels (pivoting, or stopped after giving
 * up), so motion_apply() must not.
 */
static bool search_step (line_follow_action_t action);
/** @brief Both wheels' encoder travel since the previous period. */
static uint32_t search_travel (void);
/** @brief Forget the search: another manoeuvre is driving the wheels. */
static void search_forget (void);
/** @brief Add, saturating at the 16-bit limit. */
static uint16_t add_saturating (uint16_t total, uint32_t amount);

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

void motion_line_seen (bool b_left, bool b_right)
{
    g_search.b_observed = 1U;
    g_search.b_black    = (b_left || b_right) ? 1U : 0U;
    g_search.seen_now   = (b_left == b_right) ? 0 : (b_left ? 1 : -1);
}

void motion_line_hint (int32_t side)
{
    if (0 != side)
    {
        /* After a bypass the line is beside the robot, not ahead of it:
         * with neither sensor on it, pivot towards it straight away
         * (as far as MOTION_SEARCH_SWEEP_DEG, then back the other way)
         * instead of driving on along the white first. */
        g_search.side         = (side > 0) ? 1 : -1;
        g_search.phase        = SEARCH_FOLLOWING;
        g_search.white_counts = MOTION_SEARCH_AFTER_COUNTS;
    }
}

void motion_apply (line_follow_action_t action)
{
    if (0U != g_downhill.fault)
    {
        downhill_fault_stop();
        return;
    }
#if !APP_MOTOR_MOVEMENT_ENABLED
    /* Keep targets and telemetry stopped as well as the driver outputs. */
    action = LINE_UNCERTAIN;
#endif

    bool const b_searching = search_step(action);
    int32_t const seen_now = g_search.seen_now;
    bool const b_observed  = 0U != g_search.b_observed;
    bool const b_both      = (0U != g_search.b_black) && (0 == seen_now);

    /* Only the observer steering reads these. */
    (void)b_observed;
    (void)b_both;

    /* Consumed: the controller reports again next period. */
    g_search.b_observed = 0U;
    g_search.b_black    = 0U;
    g_search.seen_now   = 0;

    if (b_searching)
    {
        return;
    }

    /* Barcode processing never owns the motor; only an explicit STOP, a
     * safety fault or the navigation state may inhibit movement. */
    g_turn_pwm = 0U;

    /* Drive at MOTION_TARGET_RPM. Barcode capture and diagnostics never
     * change it: sensor sampling uses encoder distance, not velocity. */
    switch (action)
    {
        case LINE_FORWARD:
        case LINE_CORRECT_LEFT:
        case LINE_CORRECT_RIGHT:
            g_brake_cycles = 0U;
#if STEERING_OBSERVER
            {
                /* What the sensors saw: the controller's report when line
                 * following, else what the action implies. */
                bool const b_left  = b_observed
                                         ? ((seen_now > 0) || b_both)
                                         : (LINE_CORRECT_LEFT == action);
                bool const b_right = b_observed
                                         ? ((seen_now < 0) || b_both)
                                         : (LINE_CORRECT_RIGHT == action);

                g_steer = limit_steer(
                    g_steer,
                    line_track_update(
                        &g_track, b_left, b_right,
                        (float)g_period_counts[ENCODER_LEFT] *
                            MOTION_MM_PER_COUNT,
                        (float)g_period_counts[ENCODER_RIGHT] *
                            MOTION_MM_PER_COUNT));
            }
#else
            /* PI steering, one period at a time, no speed scaling. */
            g_steer = limit_steer(
                g_steer,
                steering_update(&g_steering, action, MOTION_PERIOD_SEC,
                                1.0f));
#endif
            if (SEARCH_ROLLING == g_search.phase)
            {
                /* Rolling check (search_step()): lean gently towards the
                 * line's last side while still driving. */
                g_steer = limit_steer(g_steer, (float)g_search.side *
                                                   MOTION_SEARCH_ROLL_STEER);
            }
            /* Barcodes are read at the driving speed: a capture neither
             * slows nor brakes the robot (the scanner measures distance
             * from the encoders, so it needs no particular speed). */
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

    /* The governor scales both targets equally after steering, keeping
     * the left/right ratio. No other task writes the motor duty. */
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
        g_brake_cycles = (uint8_t)MOTION_BRAKE_CYCLES;
    }
#endif
    motion_apply(LINE_UNCERTAIN);
}

void motion_stop_now(void)
{
    /* Operator STOP also overrides a timed brake pulse in progress. */
    motor_stop();
    search_forget();
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
    if (0U != g_downhill.fault)
    {
        downhill_fault_stop();
        return;
    }
    motion_wheel_t * const p_left = &g_wheels[ENCODER_LEFT];
    motion_wheel_t * const p_right = &g_wheels[ENCODER_RIGHT];
    search_forget();
#if !APP_MOTOR_MOVEMENT_ENABLED
    motion_apply(LINE_UNCERTAIN);
    return;
#endif
    /* Bypass forward motion receives the same downhill restriction. */
    downhill_scale_pair(&g_downhill, left_rpm, right_rpm,
                        &left_rpm, &right_rpm);
    /* Obstacle arcs use forward motion. Braking is never ramped. */
    if (left_rpm < 0.0f)
    {
        left_rpm = 0.0f;
    }
    if (right_rpm < 0.0f)
    {
        right_rpm = 0.0f;
    }
    if ((left_rpm > 0.0f) || (right_rpm > 0.0f))
    {
        g_brake_cycles = 0U;
    }
    p_left->target_rpm = (left_rpm > p_left->target_rpm) ?
        step_towards(p_left->target_rpm, left_rpm, MOTION_RAMP_RPM) : left_rpm;
    p_right->target_rpm = (right_rpm > p_right->target_rpm) ?
        step_towards(p_right->target_rpm, right_rpm, MOTION_RAMP_RPM) :
        right_rpm;
    g_base_rpm = 0.5f * (p_left->target_rpm + p_right->target_rpm);
    g_steer = 0.0f;
    g_turn_pwm = 0U;
    steering_reset(&g_steering);
    line_track_reset(&g_track);
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
        /* A wheel well over target brakes before the overspeed latch. */
        if (downhill_wheel_over_target(&g_downhill,
                    p_left->target_rpm, status.left_rpm))
        {
            lp = MOTION_RAMP_EFFORT_MIN;
        }
        if (downhill_wheel_over_target(&g_downhill,
                    p_right->target_rpm, status.right_rpm))
        {
            rp = MOTION_RAMP_EFFORT_MIN;
        }
        status.left_pwm = speed_to_pwm(lp < 0.0f ? -lp : lp);
        status.right_pwm = speed_to_pwm(rp < 0.0f ? -rp : rp);
        p_left->effort_pwm = lp < 0.0f ? -(int32_t)status.left_pwm :
                                         (int32_t)status.left_pwm;
        p_right->effort_pwm = rp < 0.0f ? -(int32_t)status.right_pwm :
                                          (int32_t)status.right_pwm;
        /* Positive PWM drives forward; negative is bounded braking,
         * also available while returning to level. */
        motor_set_regulated(lp < 0.0f ? -(int32_t)status.left_pwm :
                                              (int32_t)status.left_pwm,
                                rp < 0.0f ? -(int32_t)status.right_pwm :
                                              (int32_t)status.right_pwm);
    }

    float const mm_per_rev = MOTION_PI * BOARD_WHEEL_DIAMETER_MM;

    status.speed_mm_per_sec =
        ((status.left_rpm + status.right_rpm) * 0.5f * mm_per_rev) /
        MOTION_SEC_PER_MIN;
    status.left_count     = left.count;
    status.right_count    = right.count;
    status.timestamp_usec = now;
    status.b_valid        = true;
    publish_status(&status);

#if APP_MOTION_DEBUG_CSV
    /* Columns: CSV,ms,action,steer%,targetL,targetR,rpmL x10,rpmR x10,
     * pwmL,pwmR. INT truncation is for display; UB preserves the bytes.
     */
    tm_printf((UB *)"CSV,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
              (INT)(now / MOTION_USEC_PER_MSEC),
              (INT)action, (INT)(g_steer * MOTION_PERCENT_SCALE),
              (INT)status.left_target_rpm, (INT)status.right_target_rpm,
              (INT)(status.left_rpm * MOTION_X10),
              (INT)(status.right_rpm * MOTION_X10),
              (INT)status.left_pwm, (INT)status.right_pwm);
#endif

    if (0U == g_log_countdown)
    {
        g_log_countdown = (uint8_t)MOTION_LOG_EVERY;
    }

    g_log_countdown--;
}

void motion_apply_maneuver (uint8_t mapped_action)
{
    /* Navigation's turn: whatever the search knew is out of date after it. */
    search_forget();
    pivot_step(mapped_action);
}

static void pivot_step (uint8_t mapped_action)
{
    if (0U != g_downhill.fault)
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
    if (mapped_action < PIVOT_LEFT || mapped_action > PIVOT_UTURN)
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
            {
                g_pivot_pwm[w] =
                    (uint16_t)(g_pivot_pwm[w] + MOTION_PIVOT_PWM_STEP);
            }
            else if (m > MOTION_PIVOT_COUNTS +
                     (MOTION_PIVOT_COUNTS / MOTION_PIVOT_SLACK_DIV))
            {
                g_pivot_pwm[w] =
                    (uint16_t)(g_pivot_pwm[w] - MOTION_PIVOT_PWM_STEP);
            }
            if (g_pivot_pwm[w] > MOTION_PIVOT_PWM_MAX)
            {
                g_pivot_pwm[w] = MOTION_PIVOT_PWM_MAX;
            }
            if (g_pivot_pwm[w] < MOTION_PIVOT_PWM_MIN)
            {
                g_pivot_pwm[w] = MOTION_PIVOT_PWM_MIN;
            }
        }
    }
    g_pivot_last[ENCODER_LEFT] = left.count;
    g_pivot_last[ENCODER_RIGHT] = right.count;
    left_pwm = (int32_t)g_pivot_pwm[ENCODER_LEFT];
    right_pwm = (int32_t)g_pivot_pwm[ENCODER_RIGHT];
    g_turn_pwm = (left_pwm > right_pwm) ? (uint16_t)left_pwm :
                                           (uint16_t)right_pwm;
    /* Left and U-turn spin anticlockwise (left wheel back), right turns
     * clockwise (right wheel back). */
    if (mapped_action == PIVOT_RIGHT)
    {
        right_pwm = -right_pwm;
    }
    else
    {
        left_pwm = -left_pwm;
    }
    motor_set_directional(left_pwm, right_pwm);
    status.left_count = left.count;
    status.right_count = right.count;
    status.left_pwm = (uint32_t)(left_pwm < 0 ? -left_pwm : left_pwm);
    status.right_pwm = (uint32_t)(right_pwm < 0 ? -right_pwm : right_pwm);
    status.timestamp_usec = encoder_now_usec();
    status.b_valid = true;
    publish_status(&status);
}

static bool search_step (line_follow_action_t action)
{
    uint32_t const moved = search_travel();

    if (0 == STEERING_OBSERVER)
    {
        /* PI steering has no lost-line search. */
        return false;
    }
    bool const     b_turning =
        (LINE_CORRECT_LEFT == action) || (LINE_CORRECT_RIGHT == action);

    if (LINE_UNCERTAIN == action)
    {
        /* Stopped: where the line was is no longer known. */
        search_forget();
        return false;
    }

    if (b_turning)
    {
        g_search.side = (LINE_CORRECT_LEFT == action) ? 1 : -1;
    }
    else if (0 != g_search.seen_now)
    {
        g_search.side = g_search.seen_now;
    }
    else
    {
        /* Both or neither: the last single sensor still says where. */
    }

    if (0U == g_search.b_observed)
    {
        /* Not line following this period (the controller reports the line
         * only then): never search, and end a search in progress so the
         * normal path drives the wheels. */
        g_search.phase        = SEARCH_FOLLOWING;
        g_search.white_counts = 0U;
        g_search.swept_counts = 0U;
        return false;
    }

    if (b_turning || (0U != g_search.b_black))
    {
        if ((SEARCH_FOLLOWING != g_search.phase) &&
            (SEARCH_ROLLING != g_search.phase) && b_turning)
        {
            /* Found it: drive on from rest, steering from scratch. */
            g_base_rpm = 0.0f;
            /* Both wheels' counts over APP_NAV_TURN90_COUNTS per 90; the
             * sweep is at most 2 x 120 degrees, so UINT holds it. */
            tm_printf((UB *)"[MOTOR] Line found after a %u deg search\n",
                      (UINT)((g_search.swept_counts * MOTION_TURN90_DEG) /
                             APP_NAV_TURN90_COUNTS));
        }

        g_search.phase        = SEARCH_FOLLOWING;
        g_search.white_counts = 0U;
        g_search.swept_counts = 0U;
        return false;
    }

    if (0 == g_search.side)
    {
        /* The line has not been seen since the last stop. */
        return false;
    }

    if ((SEARCH_FOLLOWING == g_search.phase) ||
        (SEARCH_ROLLING == g_search.phase))
    {
        g_search.white_counts = add_saturating(g_search.white_counts, moved);

        if (g_search.white_counts < MOTION_SEARCH_ROLL_COUNTS)
        {
            return false;
        }

        if (g_search.white_counts < MOTION_SEARCH_AFTER_COUNTS)
        {
            /* Keep driving; motion_apply() leans the steer towards the
             * line's last side (no stop, no pivot). */
            g_search.phase = SEARCH_ROLLING;
            return false;
        }

        g_search.phase        = SEARCH_FIRST_SWEEP;
        g_search.swept_counts = 0U;
        /* UB preserves the string bytes. */
        tm_printf((UB *)"[MOTOR] Line lost %u mm ago: pivoting %s to find "
                        "it\n",
                  (UINT)MOTION_SEARCH_AFTER_MM,
                  (g_search.side > 0) ? (UB *)"left" : (UB *)"right");
    }
    else
    {
        g_search.swept_counts = add_saturating(g_search.swept_counts, moved);
    }

    if ((SEARCH_FIRST_SWEEP == g_search.phase) &&
        (g_search.swept_counts >= MOTION_SEARCH_SWEEP_COUNTS))
    {
        g_search.phase        = SEARCH_SWEEP_BACK;
        g_search.swept_counts = 0U;
        tm_printf((UB *)"[MOTOR] Not there: pivoting back the other way\n");
    }

    if ((SEARCH_SWEEP_BACK == g_search.phase) &&
        (g_search.swept_counts >= (2U * MOTION_SEARCH_SWEEP_COUNTS)))
    {
        g_search.phase = SEARCH_GAVE_UP;
        /* UINT matches %u; UB preserves the string bytes. */
        tm_printf((UB *)"[MOTOR] Line not found within %u deg either way: "
                        "stopped\n",
                  (UINT)MOTION_SEARCH_SWEEP_DEG);
    }

    if (SEARCH_GAVE_UP == g_search.phase)
    {
        /* Stay put until a sensor finds the line (or the robot is moved
         * onto it), rather than drive off the course. */
        stop_wheels();
        g_turn_pwm = 0U;
        drive_wheels(LINE_UNCERTAIN);
        return true;
    }

    /* First sweep towards the line's last side, the second back past it. */
    bool const b_left = (SEARCH_FIRST_SWEEP == g_search.phase)
                            ? (g_search.side > 0)
                            : (g_search.side < 0);

    pivot_step(b_left ? PIVOT_LEFT : PIVOT_RIGHT);
    return true;
}

static uint32_t search_travel (void)
{
    encoder_sample_t samples[ENCODER_WHEEL_COUNT] = {{0, 0U}, {0, 0U}};
    uint32_t         moved                        = 0U;

    encoder_read(ENCODER_LEFT, &samples[ENCODER_LEFT]);
    encoder_read(ENCODER_RIGHT, &samples[ENCODER_RIGHT]);

    for (uint32_t w = 0U; w < (uint32_t)ENCODER_WHEEL_COUNT; w++)
    {
        /* Unsigned subtraction is wrap-safe; either direction counts. */
        uint32_t const delta =
            (uint32_t)samples[w].count - (uint32_t)g_search.last_count[w];

        if (0U != g_search.b_counts)
        {
            uint32_t const size =
                (delta > (uint32_t)INT32_MAX) ? (0U - delta) : delta;

            moved += size;
            /* Size only: the left encoder counts down when driving
             * forward, and line following only drives forward, so a
             * signed count would look like spinning. Bounded to 16 bits. */
            g_period_counts[w] =
                (int16_t)((size > (uint32_t)INT16_MAX) ? INT16_MAX : size);
        }
        else
        {
            g_period_counts[w] = 0;
        }

        g_search.last_count[w] = samples[w].count;
    }

    g_search.b_counts = 1U;
    return moved;
}

static void search_forget (void)
{
    g_search.phase        = SEARCH_FOLLOWING;
    g_search.white_counts = 0U;
    g_search.swept_counts = 0U;
    g_search.side         = 0;
    g_search.b_counts     = 0U;
}

static uint16_t add_saturating (uint16_t total, uint32_t amount)
{
    uint32_t const sum = (uint32_t)total + amount;

    /* Bounded to the 16-bit limit before narrowing. */
    return (sum > MOTION_U16_MAX) ? MOTION_U16_MAX : (uint16_t)sum;
}

/** @brief Log downhill engage, release and fault changes. */
static void report_downhill_change (uint32_t before,
                                    uint32_t fault_before)
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
    uint32_t const before = g_downhill.engaged;
    uint32_t const fault_before = g_downhill.fault;
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

/** @brief Clear speed-controller state for a new run. */
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

/** @brief Feed a terrain snapshot to the downhill governor. */
void motion_observe_terrain(terrain_snapshot_t const *s, uint32_t now)
{
    bool fresh = s->b_valid &&
        (uint32_t)(now - s->timestamp_msec) <= MOTION_IMU_FRESH_MSEC;
    uint32_t const before = g_downhill.engaged;
    uint32_t const fault_before = g_downhill.fault;
    downhill_sample_terrain(&g_downhill, s->pitch_deg, s->roll_deg,
        fresh, true, s->height_valid && s->onHump &&
                     s->hump_phase == TERRAIN_HUMP_DESCENDING);
    report_downhill_change(before, fault_before);
}

/** @brief Return the downhill fault: 0 none, 1 IMU, 2 overspeed. */
uint8_t motion_downhill_fault(void)
{
    return g_downhill.fault ?
        (g_downhill.overspeed >= DOWNHILL_OVERSPEED_COUNT ?
         MOTION_DOWNHILL_FAULT_OVERSPEED : MOTION_DOWNHILL_FAULT_IMU) : 0U;
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
    bool const ramp_control = g_downhill.scale_percent < DOWNHILL_FULL_PERCENT;
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

    /* Stall rescue. On a tight curve the outer wheel may be asked for a
     * speed below the integrating range, where its feedforward duty alone
     * may not turn it. While a commanded wheel is not turning, raise its
     * duty step by step; once it turns, take the boost off gradually. */
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
        /* Integrates at any target speed on a descent. Negative effort is
         * bounded short-brake, never reverse. Integrating only when not
         * saturated (or when unwinding) prevents windup. */
        float candidate = p_wheel->integral +
                          MOTION_KI * error * MOTION_PERIOD_SEC;
        float effort = base_percent + MOTION_KP * error + candidate;
        if (((effort >= MOTION_RAMP_EFFORT_MIN) &&
             (effort <= MOTION_RAMP_EFFORT_MAX)) ||
            ((effort < MOTION_RAMP_EFFORT_MIN) && (error > 0.0f)) ||
            ((effort > MOTION_RAMP_EFFORT_MAX) && (error < 0.0f)))
        {
            p_wheel->integral = clamp_symmetric(candidate,
                                                MOTION_RAMP_INTEGRAL_MAX);
        }
        effort = base_percent + MOTION_KP * error + p_wheel->integral;
        if (effort < MOTION_RAMP_EFFORT_MIN)
        {
            effort = MOTION_RAMP_EFFORT_MIN;
        }
        if (effort > MOTION_RAMP_EFFORT_MAX)
        {
            effort = MOTION_RAMP_EFFORT_MAX;
        }
        p_wheel->b_regulating = true;
        return step_towards((float)p_wheel->effort_pwm * MOTION_PERCENT_SCALE /
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
        float const prior = (float)p_wheel->effort_pwm * MOTION_PERCENT_SCALE /
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
        /* Too slow to hold: the wheel would be dragged along instead. */
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
    line_track_reset(&g_track);
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

    /* Truncation gives the calibrated PWM counts. */
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

