/** @file task_imu.c
 * @brief Calibrate the stationary robot, then sample and report IMU data.
 */
#include "task_imu.h"
#include "terrain.h"
#include "lift_math.h"
#include "drivers/lsm303.h"
#include "app/app_config.h"
#include "platform/console.h"
#include <stdbool.h>
#include <stdint.h>
#include <tm/tmonitor.h>

#define IMU_SAMPLE_PERIOD_MSEC 20
#define IMU_LOG_PERIOD_MSEC 100U
#define IMU_SENSOR_SETTLE_MSEC 100
#define IMU_CALIBRATION_SAMPLES 50
#define IMU_CALIBRATION_MIN_MG 800
#define IMU_CALIBRATION_MAX_MG 1200
#define IMU_CALIBRATION_CHANGE_MG 50
#define IMU_CALIBRATION_RETRY_MSEC 500U
/* Trial limit from turn/flick logs; validate normal braking and humps too. */
#define IMU_HORIZONTAL_JERK_THRESHOLD_MG_PER_SEC 12000

/* Fraction of each new reading kept by the smoothing filter. Lower is
 * smoother but lags more and trims the peak; motor vibration needs some.
 */
#define IMU_FILTER_ALPHA 0.2F
/* A 1 cm hump under one wheel rolls the robot about 5.2 degrees
 * (asin(10 / 110)). Starting and ending at different angles stops noise
 * near one value from flickering.
 */
/* Readings whose total acceleration is this far from the calibrated
 * gravity are being shoved, not just tilted, so they are left out. Without
 * a gyroscope this is the only way to keep a jolt from reading as tilt.
 */
#define IMU_GRAVITY_TOLERANCE_MG 50
#define HUMP_START_DEG 3.0F
#define HUMP_LEVEL_DEG 1.5F
/* Samples in a row above HUMP_START_DEG that count as a real tilt, so a
 * single vibration spike neither starts a hump nor cancels its ending.
 */
#define HUMP_TILT_CONFIRM_SAMPLES 3U
/* Level samples needed to end a hump. Noise between the two angles above
 * neither counts nor resets them; only a sustained tilt starts over.
 */
#define HUMP_LEVEL_CONFIRM_SAMPLES 15U
/* Drop below the peak that counts as descending, ignoring jitter. */
#define HUMP_DESCENT_MM 1.5F
#if (HUMP_LEVEL_CONFIRM_SAMPLES > 15U) || (HUMP_TILT_CONFIRM_SAMPLES > 3U)
#error Hump confirmation counts exceed their bit-field widths
#endif
/* Hump heights are published in seven bits (terrain_snapshot). */
#define HUMP_PUBLISH_MAX_MM 127U
#if WHEEL_SPACING_MM > 127
#error Hump height fields hold at most 127 mm; widen terrain_snapshot
#endif
#if (APP_LIFT_STOP_HEIGHT_MM < 1) || (APP_LIFT_STOP_HEIGHT_MM > 1022)
#error APP_LIFT_STOP_HEIGHT_MM must be 1..1022 mm
#endif

typedef struct
{
    float    filtered[3]; /**< Smoothed axes, mg. */
    float    tilt_deg;    /**< Size of the roll, degrees. */
    float    height_mm;
    float    peak_mm;     /**< Highest point of this hump. */
    int16_t  gravity_mg;  /**< Total acceleration at rest: 800..1200. */
    uint16_t rejected;    /**< Shoved samples since last log. */
    /* IMU-task owned, so packing needs no lock. Phase is 0..2; the counts
     * stop at HUMP_LEVEL_CONFIRM_SAMPLES and HUMP_TILT_CONFIRM_SAMPLES. */
    unsigned int b_filter_ready:1;
    unsigned int b_on_hump:1;
    unsigned int phase:2;        /**< A terrain_hump_phase value. */
    unsigned int level_count:4;
    unsigned int tilted_count:2; /**< Samples in a row above start. */
    uint8_t            last_peak_mm; /**< 0..127, as published. */
    uint8_t            max_peak_mm;  /**< 0..127, as published. */
    uint8_t            hump_count;
} hump_state_t;

static hump_state_t g_hump = {0};
/* Owned by the IMU task alone. */
static lift_math_state_t g_lift = {0};
/* Nonzero while the lifted state has been logged, so it prints once. */
static uint8_t g_lift_reported = 0U;
/* Samples in a row still and level, up to IMU_REST_SAMPLES. */
#define IMU_REST_SAMPLES (APP_LIFT_REST_MSEC / IMU_SAMPLE_PERIOD_MSEC)
#if (IMU_REST_SAMPLES < 1) || (IMU_REST_SAMPLES > 65535)
#error APP_LIFT_REST_MSEC must be 20..1310700 ms
#endif
static uint16_t g_rest_samples = 0U;
/* Last valid relative tilt, for the level test (only roll, pitch and
 * b_valid are used). IMU task only. */
static terrain_snapshot g_rest_tilt = {0};

typedef struct
{
    int32_t  previous_magnitude;
    int16_t previous_x;
    int16_t previous_y;
    int16_t previous_z;
    bool     b_have_previous;
    bool     b_collision_enabled; /**< Previous sample was in an active run. */
    uint32_t previous_time_msec;
    uint32_t last_log_msec;
} imu_history_t;

typedef struct
{
    int16_t axis_x;
    int16_t axis_y;
    int16_t axis_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    int32_t magnitude;
    /* Signed rate of change of acceleration magnitude, mg/s. */
    int32_t jerk;
    /* Signed sensor-axis vector jerk components, mg/s. */
    int32_t jerk_x;
    int32_t jerk_y;
    int32_t jerk_z;
    bool    b_mag_ok;
} imu_sample_t;

/** @brief Retrieve the same low 32 bits of kernel operating time. */
static bool operating_time (uint32_t * p_msec);
/** @brief Read one sample and update history after processing it. */
static void process_sample (ID h_device, imu_history_t * p_history);
/** @brief Print one successful accelerometer sample in the original format. */
static void log_sample (uint32_t now_msec, imu_sample_t const * p_sample);
/** @brief Average a stationary one-second window and save its angles. */
static bool calibrate_sensor (ID h_device);
/** @brief Reject acceleration far from gravity or changing from startup. */
static bool calibration_sample_valid (int16_t const * p_axes,
                                      int16_t const * p_first_axes);
/** @brief Report angles relative to the startup level reference. */
static void log_tilt(terrain_snapshot const *p_snapshot);
/** @brief Low-pass filter the accelerometer axes against vibration. */
static void smooth_axes (imu_sample_t const * p_sample, int16_t * p_axes);
/** @brief Update hump start, climb, descent and end from a valid tilt. */
static void update_hump (terrain_snapshot const * p_snapshot);
/** @brief Round a non-negative hump height to 0..127 whole millimetres. */
static uint8_t round_mm (float height_mm);
/** @brief Advance the lifted-height estimate and publish it. */
static void update_lift (imu_sample_t const * p_sample,
                         uint32_t elapsed_msec, terrain_snapshot * p_snapshot);

void task_imu_task (INT start_code, void * p_context)
{
    /* Casting unused arguments to void cannot alter their values. */
    (void)start_code;
    (void)p_context;

    /* The kernel device name is read-only; UB preserves its string bytes. */
    ID h_device = tk_opn_dev((UB *)"iica", TD_UPDATE);

    if (h_device <= E_OK)
    {
        /* UB preserves string bytes; the API does not modify the string. */
        tm_printf((UB *)"IMU: unable to open I2C0 (er=%d)\n", h_device);
        tk_slp_tsk(TMO_FEVR); /* never return from a task function */
        return;
    }

    for (INT attempt = 0;
         (attempt < APP_USB_WAIT_ATTEMPTS) && (console_state() < 2U); ++attempt)
    {
        tk_dly_tsk(APP_USB_WAIT_PERIOD_MSEC);
    }

    tk_dly_tsk(APP_USB_SETTLE_MSEC);
    /* UB preserves string bytes; the API does not modify the string. */
    tm_printf((UB *)"\n=== RoboPico LSM303DLHC microT test: GP16/GP17 ===\n");

    if (!lsm303_init(h_device))
    {
        /* UB preserves string bytes; the API does not modify the string. */
        tm_printf((UB *)"IMU: failed to initialize accelerometer or "
                        "magnetometer\n");
        tk_cls_dev(h_device, 0);
        tk_slp_tsk(TMO_FEVR); /* never return from a task function */
        return;
    }

    /* UB preserves string bytes; the API does not modify the string. */
    tm_printf((UB *)"IMU initialized: accelerometer 0x19; magnetometer 0x1E\n");

    /* UB views read-only string bytes; the monitor does not modify them. */
    tm_printf((UB *)"IMU: keep robot still and level for calibration\n");
    tk_dly_tsk(IMU_SENSOR_SETTLE_MSEC);

    while (!terrain_is_calibrated())
    {
        if (calibrate_sensor(h_device))
        {
            break;
        }

        /* Each attempt starts a fresh calibration sample window. */
        tm_printf((UB *)"IMU: calibration failed; retrying in %u ms. "
                        "Keep robot still and level.\n",
                (UINT)IMU_CALIBRATION_RETRY_MSEC);

        tk_dly_tsk(IMU_CALIBRATION_RETRY_MSEC);
    }

    /* UB views read-only string bytes; the monitor does not modify them. */
    tm_printf((UB *)"IMU: level reference saved in RAM\n");

    imu_history_t history = {0};

    if (!operating_time(&history.previous_time_msec))
    {
        /* UB preserves string bytes; the API does not modify the string. */
        tm_printf((UB *)"IMU: cannot read kernel operating time\n");
        tk_cls_dev(h_device, 0);
        tk_slp_tsk(TMO_FEVR); /* never return from a task function */
        return;
    }

    history.last_log_msec = history.previous_time_msec - IMU_LOG_PERIOD_MSEC;

    for (;;)
    {
        process_sample(h_device, &history);
        /* Allow new samples and let other tasks run. */
        tk_dly_tsk(IMU_SAMPLE_PERIOD_MSEC);
    }
}

static bool calibrate_sensor (ID h_device)
{
    // This is 3 for x,y,z axes sumof
    int32_t sums[3] = {0};
    int16_t first_axes[3] = {0};

    for (INT sample = 0; sample < IMU_CALIBRATION_SAMPLES; ++sample)
    {
        int16_t axes[3] = {0};

        if (!lsm303_read_accel(h_device, &axes[0], &axes[1], &axes[2]))
        {
            tm_printf((UB *)"IMU: calibration I2C read failed at sample %d\n",
                      sample + 1);
            return false;
        }
        // This is a for loop of 3 iterations, one for each axis (X, Y, Z).
        // It accumulates the sum of the axis readings
        // and stores the first sample's values for later comparison.
        for (INT axis = 0; axis < 3; ++axis)
        {
            if (0 == sample)
            {
                first_axes[axis] = axes[axis];
            }

            /* 50 signed 12-bit samples fit int32_t without overflow. */
            sums[axis] += axes[axis];
        }

        if (!calibration_sample_valid(axes, first_axes))
        {
            tm_printf((UB *)"IMU: calibration rejected sample %d "
                            "XYZ=(%d,%d,%d) magnitude=%d\n",
                      sample + 1, (INT)axes[0], (INT)axes[1], (INT)axes[2],
                      (INT)terrain_magnitude(axes[0], axes[1], axes[2]));
            return false;
        }

        tk_dly_tsk(IMU_SAMPLE_PERIOD_MSEC);
    }

    /* Each average stays in the signed 12-bit range and fits int16_t. */
    int16_t avg_x = (int16_t)(sums[0] / IMU_CALIBRATION_SAMPLES);
    int16_t avg_y = (int16_t)(sums[1] / IMU_CALIBRATION_SAMPLES);
    int16_t avg_z = (int16_t)(sums[2] / IMU_CALIBRATION_SAMPLES);

    ER error = terrain_calibrate(avg_x, avg_y, avg_z);

    /* Start the smoothing filter from the level reference, not from the
     * first noisy reading, so it cannot begin with a false hump.
     */
    g_hump.filtered[0]    = (float)avg_x;
    g_hump.filtered[1]    = (float)avg_y;
    g_hump.filtered[2]    = (float)avg_z;
    g_hump.b_filter_ready = true;
    /* Calibration accepted only 800..1200 mg, so this fits int16_t. */
    g_hump.gravity_mg     = (int16_t)terrain_magnitude(avg_x, avg_y, avg_z);

    if (error < E_OK)
    {
        return false;
    }

    float roll_zero = 0.0F;
    float pitch_zero = 0.0F;

    // Terrain_tilt is called again to compute once for the printf readings.
    // Note that terrain_calibrate already calls terrain_tilt
    // internally to save the reference angles.
    error = terrain_tilt(
        avg_x,
        avg_y,
        avg_z,
        &roll_zero,
        &pitch_zero
    );

    if (error < E_OK)
    {
        return false;
    }

    tm_printf(
        (UB *)"IMU CALIBRATION:\n"
            "AVG XYZ = (%d, %d, %d)\n"
            "ROLL_ZERO_mdeg = %d\n"
            "PITCH_ZERO_mdeg = %d\n",
        (INT)avg_x,
        (INT)avg_y,
        (INT)avg_z,
        (INT)(roll_zero * 1000.0F),
        (INT)(pitch_zero * 1000.0F)
    );

    return true;
}

static bool calibration_sample_valid (int16_t const * p_axes,
                                      int16_t const * p_first_axes)
{
    int32_t magnitude = terrain_magnitude(p_axes[0], p_axes[1], p_axes[2]);

    if ((magnitude < IMU_CALIBRATION_MIN_MG) ||
        (magnitude > IMU_CALIBRATION_MAX_MG))
    {
        return false;
    }

    for (INT axis = 0; axis < 3; ++axis)
    {
        int32_t difference = (int32_t)p_axes[axis] - p_first_axes[axis];

        if ((difference < -IMU_CALIBRATION_CHANGE_MG) ||
            (difference > IMU_CALIBRATION_CHANGE_MG))
        {
            return false;
        }
    }

    return true;
}

static bool operating_time (uint32_t * p_msec)
{
    SYSTIM time = {0};

    if (tk_get_otm(&time) < E_OK)
    {
        return false;
    }

    *p_msec = time.lo;
    return true;
}

static void process_sample (ID h_device, imu_history_t * p_history)
{
    uint32_t now_msec = 0U;

    if (!operating_time(&now_msec))
    {
        /* UB preserves string bytes; the API does not modify the string. */
        tm_printf((UB *)"IMU: operating-time read failed\n");
        return;
    }

    // This is reading the raw readings of accelerometer and magnetometer
    imu_sample_t sample = {0};
    bool b_accel_ok = lsm303_read_accel(
        h_device,
        &sample.axis_x,
        &sample.axis_y,
        &sample.axis_z
    );

    sample.b_mag_ok = lsm303_read_mag(
        h_device,
        &sample.mag_x,
        &sample.mag_y,
        &sample.mag_z
    );

    // This is for writing the processed readings into the terrain snapshot
    terrain_snapshot snapshot = {0};
    snapshot.timestamp_msec = now_msec;
    snapshot.b_valid = false;

    /* Compute the magnetic heading here */
    if (sample.b_mag_ok)
    {
        ER heading_error = terrain_heading_level(
            sample.mag_x,
            sample.mag_y,
            &snapshot.heading_deg
        );

        if(heading_error == E_OK)
        {
            snapshot.heading_valid = true;
        }
    }

    /* Compute motion measurements only after a successful read. */
    if (b_accel_ok)
    {
        sample.magnitude = terrain_magnitude(
            sample.axis_x,
            sample.axis_y,
            sample.axis_z
        );

        if (p_history->b_have_previous)
        {
            uint32_t elapsed_msec =
                now_msec - p_history->previous_time_msec;
            update_lift(&sample, elapsed_msec, &snapshot);
            /* Rate of change of the total acceleration magnitude. */
            sample.jerk = terrain_jerk(sample.magnitude,
                p_history->previous_magnitude,
                elapsed_msec);

            /* Rate of change of each acceleration axis. */
            terrain_jerk_vector vector = terrain_vector_jerk(
                sample.axis_x, sample.axis_y, sample.axis_z,
                p_history->previous_x, p_history->previous_y,
                p_history->previous_z, elapsed_msec);
            sample.jerk_x = vector.x;
            sample.jerk_y = vector.y;
            sample.jerk_z = vector.z;
        }
        /* Sensor X/Y are sideways/forward. Keep Z in the diagnostic log;
         * vertical jerk alone does not trigger this horizontal-impact rule.
         */
        bool const collision_enabled = APP_COLLISION_ENABLED != 0 &&
                                        terrain_collision_enabled();
        /* The first run sample establishes a fresh impact baseline. Handling
         * the car in STOP must neither count nor become a START-time impact. */
        if (collision_enabled && p_history->b_collision_enabled &&
            (sample.jerk_x > IMU_HORIZONTAL_JERK_THRESHOLD_MG_PER_SEC ||
            sample.jerk_x < -IMU_HORIZONTAL_JERK_THRESHOLD_MG_PER_SEC ||
            sample.jerk_y > IMU_HORIZONTAL_JERK_THRESHOLD_MG_PER_SEC ||
            sample.jerk_y < -IMU_HORIZONTAL_JERK_THRESHOLD_MG_PER_SEC))
        {
            tm_printf((UB *)"[IMU] Collision detected! "
                            "XYZ jerk=(%d,%d,%d) mg/s\n",
                      (INT)sample.jerk_x, (INT)sample.jerk_y,
                      (INT)sample.jerk_z);
            /* Publish one event; the terrain store owns the latched state.
             * STOP acknowledges it, not a separate unsynchronised IMU flag. */
            snapshot.b_collision = true;
        }
        p_history->b_collision_enabled = collision_enabled;
        // Compute the tilt from smoothed readings: raw ones swing by
        // several degrees with motor vibration.
        int16_t smoothed[3] = {0};
        smooth_axes(&sample, smoothed);

        ER tilt_error = terrain_relative_tilt(
            smoothed[0], smoothed[1],
            smoothed[2],
            &snapshot.roll_deg,
            &snapshot.pitch_deg
        );
        if (tilt_error == E_OK)
        {
            snapshot.b_valid = true;
        }
        g_rest_tilt.roll_deg  = snapshot.roll_deg;
        g_rest_tilt.pitch_deg = snapshot.pitch_deg;
        g_rest_tilt.b_valid   = snapshot.b_valid;

        if (snapshot.b_valid)
        {
            update_hump(&snapshot);
        }
        else
        {
            // Invalid measurements cannot confirm completion
            g_hump.level_count = 0U;
        }

        // Assign the values to the snapshot for publishing
        snapshot.onHump = g_hump.b_on_hump;
        snapshot.hump_phase = g_hump.phase;
        snapshot.height_valid = snapshot.b_valid;
        snapshot.hump_height_mm =
            snapshot.b_valid ? round_mm(g_hump.height_mm) : 0U;
        snapshot.last_hump_height_mm = g_hump.last_peak_mm;
        snapshot.max_hump_height_mm = g_hump.max_peak_mm;
        snapshot.hump_count = g_hump.hump_count;

        // This is for logging (printfs) the readings to the console every 100ms
        if ((0 != APP_IMU_DEBUG_LOG) &&
            ((now_msec - p_history->last_log_msec) >= IMU_LOG_PERIOD_MSEC))
        {
            log_sample(now_msec, &sample);
            log_tilt(&snapshot);
            /* Tenths keep sub-millimetre detail without float printing. */
#ifndef B1_BARCODE_TEST /* quiet barcode test log */
            tm_printf((UB *)"HUMP: on=%u phase=%u tilt_mdeg=%d "
                            "height_x10mm=%d peak_x10mm=%d count=%u "
                            "rejected=%u MAGNETOMETER: DEGREE: %d\n",
                      (UINT)g_hump.b_on_hump, (UINT)g_hump.phase,
                      (INT)(g_hump.tilt_deg * 1000.0F),
                      (INT)(g_hump.height_mm * 10.0F),
                      (INT)(g_hump.peak_mm * 10.0F),
                      (UINT)g_hump.hump_count, (UINT)g_hump.rejected,
                      (INT)(snapshot.heading_deg * 1000.0F));
            tm_printf((UB *)"LIFT: height_mm=%u lifted=%u excess_mg=%d "
                            "bias_mg=%d\n",
                      (UINT)snapshot.lift_height_mm,
                      (UINT)snapshot.b_lifted,
                      (INT)(sample.magnitude - g_hump.gravity_mg),
                      (INT)g_lift.bias_mg);
#endif
            g_hump.rejected = 0U;
            p_history->last_log_msec = now_msec;
        }

        p_history->previous_magnitude = sample.magnitude;
        p_history->previous_x = sample.axis_x;
        p_history->previous_y = sample.axis_y;
        p_history->previous_z = sample.axis_z;
        p_history->previous_time_msec = now_msec;
        p_history->b_have_previous    = true;
    }
    else if ((now_msec - p_history->last_log_msec) >= IMU_LOG_PERIOD_MSEC)
    {
        /* UINT preserves 32-bit time; UB preserves read-only string bytes. */
        tm_printf((UB *)"t=%u ms ACCEL=I2C_ERROR MAG=%s\n", (UINT)now_msec,
                  sample.b_mag_ok ? "OK" : "I2C_ERROR");
        p_history->last_log_msec = now_msec;
    }

    ER publish_error = terrain_publish_snapshot(&snapshot);
    if (publish_error < E_OK)
    {
        tm_printf(
            (UB *)"IMU: snapshot publish failed: %d\n",
            (INT)publish_error
        );
    }
}

static void smooth_axes (imu_sample_t const * p_sample, int16_t * p_axes)
{
    float const raw[3] = {(float)p_sample->axis_x, (float)p_sample->axis_y,
                          (float)p_sample->axis_z};

    int32_t const excess_mg = p_sample->magnitude - g_hump.gravity_mg;
    bool const    b_shoved  = g_hump.b_filter_ready &&
                           ((excess_mg > IMU_GRAVITY_TOLERANCE_MG) ||
                            (excess_mg < -IMU_GRAVITY_TOLERANCE_MG));

    if (b_shoved && (g_hump.rejected < UINT16_MAX))
    {
        g_hump.rejected++;
    }

    for (INT axis = 0; axis < 3; ++axis)
    {
        if (!g_hump.b_filter_ready)
        {
            g_hump.filtered[axis] = raw[axis];
        }
        else if (!b_shoved)
        {
            g_hump.filtered[axis] +=
                IMU_FILTER_ALPHA * (raw[axis] - g_hump.filtered[axis]);
        }
        else
        {
            /* Hold the last trustworthy estimate. */
        }

        /* Averages of 12-bit readings stay within int16_t. */
        p_axes[axis] = (int16_t)g_hump.filtered[axis];
    }

    g_hump.b_filter_ready = true;
}

static void update_hump (terrain_snapshot const * p_snapshot)
{
    g_hump.tilt_deg = (p_snapshot->roll_deg < 0.0F) ? -p_snapshot->roll_deg
                                                     : p_snapshot->roll_deg;
    g_hump.height_mm = terrain_hump_height(p_snapshot->roll_deg);

    if (g_hump.tilt_deg <= HUMP_START_DEG)
    {
        g_hump.tilted_count = 0U;
    }
    else if (g_hump.tilted_count < HUMP_TILT_CONFIRM_SAMPLES)
    {
        g_hump.tilted_count++;
    }
    else
    {
        /* Already confirmed; the count only needs to reach the limit. */
    }

    bool const b_tilted = (g_hump.tilted_count >= HUMP_TILT_CONFIRM_SAMPLES);

    if (!g_hump.b_on_hump)
    {
        if (b_tilted)
        {
            g_hump.b_on_hump   = true;
            g_hump.phase       = TERRAIN_HUMP_CLIMBING;
            g_hump.level_count = 0U;
            g_hump.peak_mm     = g_hump.height_mm;
            /* UB views read-only string bytes. */
#ifndef B1_BARCODE_TEST /* quiet barcode test log */
            tm_printf((UB *)"HUMP STARTED: climbing\n");
#endif
        }

        return;
    }

    if (g_hump.height_mm > g_hump.peak_mm)
    {
        /* A new high point: still on the way up. */
        g_hump.peak_mm = g_hump.height_mm;

        if (TERRAIN_HUMP_CLIMBING != g_hump.phase)
        {
            /* UB views read-only string bytes. */
#ifndef B1_BARCODE_TEST /* quiet barcode test log */
            tm_printf((UB *)"HUMP: climbing\n");
#endif
        }

        g_hump.phase = TERRAIN_HUMP_CLIMBING;
    }
    else if (g_hump.height_mm < (g_hump.peak_mm - HUMP_DESCENT_MM))
    {
        /* Clearly below the top: past it and on the way down. */
        if (TERRAIN_HUMP_DESCENDING != g_hump.phase)
        {
            /* UB views read-only string bytes. */
#ifndef B1_BARCODE_TEST /* quiet barcode test log */
            tm_printf((UB *)"HUMP: descending\n");
#endif
        }

        g_hump.phase = TERRAIN_HUMP_DESCENDING;
    }
    else
    {
        /* Near the top: keep the current phase. */
    }

    if (b_tilted)
    {
        /* Tilt detected again; reset confirmation */
        g_hump.level_count = 0U;
        return;
    }

    if (g_hump.tilt_deg >= HUMP_LEVEL_DEG)
    {
        /* Between the thresholds, or a brief spike: hold the count. */
        return;
    }

    g_hump.level_count++;

    if (g_hump.level_count < HUMP_LEVEL_CONFIRM_SAMPLES)
    {
        return;
    }

    uint8_t const peak_mm = round_mm(g_hump.peak_mm);

    g_hump.b_on_hump    = false;
    g_hump.phase        = TERRAIN_HUMP_LEVEL;
    g_hump.level_count  = 0U;
    g_hump.last_peak_mm = peak_mm;
    g_hump.hump_count++;

    if (peak_mm > g_hump.max_peak_mm)
    {
        g_hump.max_peak_mm = peak_mm;
    }

    /* UB views read-only string bytes; tenths avoid float printing. */
#ifndef B1_BARCODE_TEST /* quiet barcode test log */
    tm_printf((UB *)"HUMP DONE: peak_x10mm=%d count=%u\n",
              (INT)(g_hump.peak_mm * 10.0F), (UINT)g_hump.hump_count);
#endif
}

static uint8_t round_mm (float height_mm)
{
    float const rounded = height_mm + 0.5F;

    if (rounded >= (float)HUMP_PUBLISH_MAX_MM)
    {
        return (uint8_t)HUMP_PUBLISH_MAX_MM;
    }

    /* Non-negative and below 127 here, so the value fits uint8_t. */
    return (uint8_t)rounded;
}

static void update_lift (imu_sample_t const * p_sample,
                         uint32_t elapsed_msec, terrain_snapshot * p_snapshot)
{
    bool const b_ir_floor = terrain_floor_contact();
    /* Previous sample's tilt: this sample's is computed after. A robot
     * sitting still and level on a junction (both line sensors black, as
     * in the air) is standing on the floor; a hand is never that steady
     * for long, and a real pickup breaks the stillness at once. */
    bool const b_level = terrain_is_level(&g_rest_tilt);
    if (lift_math_is_still(&g_lift) && b_level)
    {
        if (g_rest_samples < (uint16_t)IMU_REST_SAMPLES)
        {
            g_rest_samples++;
        }
    }
    else
    {
        g_rest_samples = 0U;
    }
    bool const b_resting = (g_rest_samples >= (uint16_t)IMU_REST_SAMPLES);
    bool const b_floor = b_ir_floor || b_resting;
    /* 12-bit axes bound the magnitude to 3547 mg, so the difference from
     * the calibrated gravity fits int16_t. */
    int16_t const excess_mg =
        (int16_t)(p_sample->magnitude - g_hump.gravity_mg);
    uint16_t const height_mm =
        lift_math_step(&g_lift, excess_mg, elapsed_msec, b_floor);
    bool const b_lifted = lift_math_is_active(&g_lift) && (!b_floor) &&
                          (height_mm >= (uint16_t)APP_LIFT_STOP_HEIGHT_MM);

    if (b_lifted && (0U == g_lift_reported))
    {
        /* UB views read-only string bytes; UINT matches %u. */
        tm_printf((UB *)"[IMU] Lifted: about %u mm off the floor\n",
                  (UINT)height_mm);
    }

    g_lift_reported            = b_lifted ? 1U : 0U;
    p_snapshot->lift_height_mm = height_mm;
    p_snapshot->b_lifted       = b_lifted ? 1U : 0U;
    p_snapshot->b_resting      = b_resting ? 1U : 0U;
}

static void log_tilt(terrain_snapshot const *p_snapshot)
{
    if ((NULL == p_snapshot) || (!p_snapshot->b_valid))
    {
        tm_printf((UB *)"TILT=UNAVAILABLE\n");
        return;
    }

    tm_printf(
        (UB *)"TILT_REL_mdeg ROLL=%d PITCH=%d\n",
        (INT)(p_snapshot->roll_deg * 1000.0F),
        (INT)(p_snapshot->pitch_deg * 1000.0F)
    );
}

static void log_sample (uint32_t now_msec, imu_sample_t const * p_sample)
{
    /* UB preserves read-only string bytes. UINT preserves 32-bit time;
     * int16_t axes, 12-bit magnitude, and jerk all fit the 32-bit INT ABI.
     */
    if (p_sample->b_mag_ok)
    {
        tm_printf((UB *)"t=%u ms ACCEL_mg=(%d,%d,%d) MAGNITUDE_mg=%d "
                        "JERK_mg_s=%d JERK_XYZ_mg_s=(%d,%d,%d) "
                        "MAG=(%d,%d,%d)\n",
                  (UINT)now_msec, (INT)p_sample->axis_x, (INT)p_sample->axis_y,
                  (INT)p_sample->axis_z, (INT)p_sample->magnitude,
                  (INT)p_sample->jerk, (INT)p_sample->jerk_x,
                  (INT)p_sample->jerk_y, (INT)p_sample->jerk_z,
                  (INT)p_sample->mag_x,
                  (INT)p_sample->mag_y, (INT)p_sample->mag_z);
    }
    else
    {
        tm_printf((UB *)"t=%u ms ACCEL_mg=(%d,%d,%d) MAGNITUDE_mg=%d "
                        "JERK_mg_s=%d JERK_XYZ_mg_s=(%d,%d,%d) "
                        "MAG=I2C_ERROR\n",
                  (UINT)now_msec, (INT)p_sample->axis_x, (INT)p_sample->axis_y,
                  (INT)p_sample->axis_z, (INT)p_sample->magnitude,
                  (INT)p_sample->jerk, (INT)p_sample->jerk_x,
                  (INT)p_sample->jerk_y, (INT)p_sample->jerk_z);
    }
}

/*** end of file ***/

