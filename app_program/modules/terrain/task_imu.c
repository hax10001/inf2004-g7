/** @file task_imu.c
 * @brief Calibrate the stationary robot, then sample and report IMU data.
 */
#include "task_imu.h"
#include "terrain.h"
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

typedef struct
{
    float              filtered[3]; /**< Smoothed axes, mg. */
    bool               b_filter_ready;
    int32_t            gravity_mg; /**< Total acceleration at rest. */
    uint16_t           rejected;   /**< Shoved samples since last log. */
    bool               b_on_hump;
    terrain_hump_phase phase;
    uint8_t            level_count;
    uint8_t            tilted_count; /**< Samples in a row above start. */
    float              tilt_deg;     /**< Size of the roll, degrees. */
    float              height_mm;
    float              peak_mm; /**< Highest point of this hump. */
    uint16_t           last_peak_mm;
    uint16_t           max_peak_mm;
    uint8_t            hump_count;
} hump_state_t;

static hump_state_t g_hump = {0};

typedef struct
{
    int32_t  previous_magnitude;
    bool     b_have_previous;
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
    int32_t jerk;
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
static void log_tilt (terrain_snapshot const * p_snapshot);
/** @brief Low-pass filter the accelerometer axes against vibration. */
static void smooth_axes (imu_sample_t const * p_sample, int16_t * p_axes);
/** @brief Update hump start, climb, descent and end from a valid tilt. */
static void update_hump (terrain_snapshot const * p_snapshot);
/** @brief Round a non-negative height to whole millimetres. */
static uint16_t round_mm (float height_mm);

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
        return;
    }

    /* UB preserves string bytes; the API does not modify the string. */
    tm_printf((UB *)"IMU initialized: accelerometer 0x19; magnetometer 0x1E\n");

    /* UB views read-only string bytes; the monitor does not modify them. */
    tm_printf((UB *)"IMU: keep robot still and level for calibration\n");
    tk_dly_tsk(IMU_SENSOR_SETTLE_MSEC);

    if (!calibrate_sensor(h_device))
    {
        /* UB views read-only string bytes. Restart to retry calibration. */
        tm_printf((UB *)"IMU: calibration failed; motors remain stopped\n");
        tk_cls_dev(h_device, 0);
        return;
    }

    /* UB views read-only string bytes; the monitor does not modify them. */
    tm_printf((UB *)"IMU: level reference saved in RAM\n");

    imu_history_t history = {0, false, 0U, 0U};

    if (!operating_time(&history.previous_time_msec))
    {
        /* UB preserves string bytes; the API does not modify the string. */
        tm_printf((UB *)"IMU: cannot read kernel operating time\n");
        tk_cls_dev(h_device, 0);
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
    int32_t sums[3]       = {0};
    int16_t first_axes[3] = {0};

    for (INT sample = 0; sample < IMU_CALIBRATION_SAMPLES; ++sample)
    {
        int16_t axes[3] = {0};

        if (!lsm303_read_accel(h_device, &axes[0], &axes[1], &axes[2]))
        {
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
    g_hump.gravity_mg     = terrain_magnitude(avg_x, avg_y, avg_z);

    if (error < E_OK)
    {
        return false;
    }

    float roll_zero  = 0.0F;
    float pitch_zero = 0.0F;

    // Terrain_tilt is called again to compute once for the printf readings.
    // Note that terrain_calibrate already calls terrain_tilt
    // internally to save the reference angles.
    error = terrain_tilt(avg_x, avg_y, avg_z, &roll_zero, &pitch_zero);

    if (error < E_OK)
    {
        return false;
    }

    tm_printf((UB *)"IMU CALIBRATION:\n"
                    "AVG XYZ = (%d, %d, %d)\n"
                    "ROLL_ZERO_mdeg = %d\n"
                    "PITCH_ZERO_mdeg = %d\n",
              (INT)avg_x, (INT)avg_y, (INT)avg_z, (INT)(roll_zero * 1000.0F),
              (INT)(pitch_zero * 1000.0F));

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
    imu_sample_t sample     = {0};
    bool         b_accel_ok = lsm303_read_accel(h_device, &sample.axis_x,
                                                &sample.axis_y, &sample.axis_z);

    sample.b_mag_ok =
        lsm303_read_mag(h_device, &sample.mag_x, &sample.mag_y, &sample.mag_z);

    // This is for writing the processed readings into the terrain snapshot
    terrain_snapshot snapshot = {0};
    snapshot.timestamp_msec   = now_msec;
    snapshot.b_valid          = false;

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
        sample.magnitude =
            terrain_magnitude(sample.axis_x, sample.axis_y, sample.axis_z);

        if (p_history->b_have_previous)
        {
            sample.jerk =
                terrain_jerk(sample.magnitude, p_history->previous_magnitude,
                             now_msec - p_history->previous_time_msec);
        }
        // Compute the tilt from smoothed readings: raw ones swing by
        // several degrees with motor vibration.
        int16_t smoothed[3] = {0};
        smooth_axes(&sample, smoothed);

        ER tilt_error =
            terrain_relative_tilt(smoothed[0], smoothed[1], smoothed[2],
                                  &snapshot.roll_deg, &snapshot.pitch_deg);
        if (tilt_error == E_OK)
        {
            snapshot.b_valid = true;
        }

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
        snapshot.onHump       = g_hump.b_on_hump;
        snapshot.hump_phase   = g_hump.phase;
        snapshot.height_valid = snapshot.b_valid;
        snapshot.hump_height_mm =
            snapshot.b_valid ? round_mm(g_hump.height_mm) : 0U;
        snapshot.last_hump_height_mm = g_hump.last_peak_mm;
        snapshot.max_hump_height_mm  = g_hump.max_peak_mm;
        snapshot.hump_count          = g_hump.hump_count;

        // This is for logging (printfs) the readings to the console every 100ms
        if ((0 != APP_IMU_DEBUG_LOG) &&
            ((now_msec - p_history->last_log_msec) >= IMU_LOG_PERIOD_MSEC))
        {
            log_sample(now_msec, &sample);
            log_tilt(&snapshot);
            /* Tenths keep sub-millimetre detail without float printing. */
            tm_printf((UB *)"HUMP: on=%u phase=%u tilt_mdeg=%d "
                            "height_x10mm=%d peak_x10mm=%d count=%u "
                            "rejected=%u MAGNETOMETER: DEGREE: %d\n",
                      (UINT)g_hump.b_on_hump, (UINT)g_hump.phase,
                      (INT)(g_hump.tilt_deg * 1000.0F),
                      (INT)(g_hump.height_mm * 10.0F),
                      (INT)(g_hump.peak_mm * 10.0F),
                      (UINT)g_hump.hump_count, (UINT)g_hump.rejected,
                      (INT)(snapshot.heading_deg * 1000.0F));
            g_hump.rejected = 0U;
            p_history->last_log_msec = now_msec;
        }

        p_history->previous_magnitude = sample.magnitude;
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
        tm_printf((UB *)"IMU: snapshot publish failed: %d\n",
                  (INT)publish_error);
    }
}

static void smooth_axes (imu_sample_t const * p_sample, int16_t * p_axes)
{
    float const raw[3] = {(float)p_sample->axis_x, (float)p_sample->axis_y,
                          (float)p_sample->axis_z};

    int32_t const excess_mg = p_sample->magnitude - g_hump.gravity_mg;
    bool const    b_shoved =
        g_hump.b_filter_ready && ((excess_mg > IMU_GRAVITY_TOLERANCE_MG) ||
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
    g_hump.tilt_deg  = (p_snapshot->roll_deg < 0.0F) ? -p_snapshot->roll_deg
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
            tm_printf((UB *)"HUMP STARTED: climbing\n");
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
            tm_printf((UB *)"HUMP: climbing\n");
        }

        g_hump.phase = TERRAIN_HUMP_CLIMBING;
    }
    else if (g_hump.height_mm < (g_hump.peak_mm - HUMP_DESCENT_MM))
    {
        /* Clearly below the top: past it and on the way down. */
        if (TERRAIN_HUMP_DESCENDING != g_hump.phase)
        {
            /* UB views read-only string bytes. */
            tm_printf((UB *)"HUMP: descending\n");
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

    uint16_t const peak_mm = round_mm(g_hump.peak_mm);

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
    tm_printf((UB *)"HUMP DONE: peak_x10mm=%d count=%u\n",
              (INT)(g_hump.peak_mm * 10.0F), (UINT)g_hump.hump_count);
}

static uint16_t round_mm (float height_mm)
{
    return (uint16_t)(height_mm + 0.5F);
}

static void log_tilt (terrain_snapshot const * p_snapshot)
{
    if ((NULL == p_snapshot) || (!p_snapshot->b_valid))
    {
        tm_printf((UB *)"TILT=UNAVAILABLE\n");
        return;
    }

    tm_printf((UB *)"TILT_REL_mdeg ROLL=%d PITCH=%d\n",
              (INT)(p_snapshot->roll_deg * 1000.0F),
              (INT)(p_snapshot->pitch_deg * 1000.0F));
}

static void log_sample (uint32_t now_msec, imu_sample_t const * p_sample)
{
    /* UB preserves read-only string bytes. UINT preserves 32-bit time;
     * int16_t axes, 12-bit magnitude, and jerk all fit the 32-bit INT ABI.
     */
    if (p_sample->b_mag_ok)
    {
        tm_printf((UB *)"t=%u ms ACCEL_mg=(%d,%d,%d) MAGNITUDE_mg=%d "
                        "JERK_mg_s=%d MAG=(%d,%d,%d)\n",
                  (UINT)now_msec, (INT)p_sample->axis_x, (INT)p_sample->axis_y,
                  (INT)p_sample->axis_z, (INT)p_sample->magnitude,
                  (INT)p_sample->jerk, (INT)p_sample->mag_x,
                  (INT)p_sample->mag_y, (INT)p_sample->mag_z);
    }
    else
    {
        tm_printf((UB *)"t=%u ms ACCEL_mg=(%d,%d,%d) MAGNITUDE_mg=%d "
                        "JERK_mg_s=%d MAG=I2C_ERROR\n",
                  (UINT)now_msec, (INT)p_sample->axis_x, (INT)p_sample->axis_y,
                  (INT)p_sample->axis_z, (INT)p_sample->magnitude,
                  (INT)p_sample->jerk);
    }
}

/*** end of file ***/

