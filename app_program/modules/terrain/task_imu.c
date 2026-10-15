/** @file task_imu.c
 * @brief Original IMU sampling and console reporting, dormant by default.
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
        tk_dly_tsk(IMU_SAMPLE_PERIOD_MSEC);
    }
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

    imu_sample_t sample     = {0};
    bool         b_accel_ok = lsm303_read_accel(h_device, &sample.axis_x,
                                                &sample.axis_y, &sample.axis_z);

    sample.b_mag_ok =
        lsm303_read_mag(h_device, &sample.mag_x, &sample.mag_y, &sample.mag_z);

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

        if ((now_msec - p_history->last_log_msec) >= IMU_LOG_PERIOD_MSEC)
        {
            log_sample(now_msec, &sample);
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
    else
    {
        /* Keep the previous history until another accelerometer sample. */
    }
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

