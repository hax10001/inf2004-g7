/*
 * GY-511 / LSM303DLHC test, ported from the working Pico SDK imu.c.
 * Uses the pin map of the uploaded vehicle main.c: I2C0 SDA=GP4, SCL=GP5.
 * The other robot components are deliberately not initialized or started.
 */
#include <stdint.h>
#include <stdbool.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "../device/include/dev_i2c.h"
#include "usb_console_compat.h"
#include "imu_task.h"

#define ADDR_ACCEL 0x19U
#define ADDR_MAG   0x1EU

#define CTRL_REG1_A 0x20U
#define CTRL_REG4_A 0x23U
#define OUT_X_L_A   0x28U
#define MR_REG_M    0x02U
#define OUT_X_H_M   0x03U

#define SAMPLE_INTERVAL_MS 20U
#define LOG_INTERVAL_MS    100U

/* Equivalent to Pico SDK's 2-byte register write; checks the byte count. */
LOCAL bool write_register(ID dd, uint8_t address, uint8_t reg, uint8_t value)
{
    UB config[2] = { reg, value };
    SZ actual = 0;
    ER er = tk_swri_dev(dd, (W)address, config, (SZ)sizeof(config), &actual);
    return er >= E_OK && actual == (SZ)sizeof(config);
}

/* Equivalent to a register-address write + repeated-start burst read. */
LOCAL bool read_registers(ID dd, uint8_t address, uint8_t reg,
                          uint8_t *data, SZ length)
{
    if (data == NULL || length <= 0) return false;

    T_I2C_EXEC txn = {
        .sadr     = (UW)address,
        .snd_size = 1,
        .snd_data = &reg,
        .rcv_size = length,
        .rcv_data = data,
    };
    SZ actual = 0;
    ER er = tk_swri_dev(dd, TDN_I2C_EXEC, &txn, (SZ)sizeof(txn), &actual);
    return er >= E_OK && actual == (SZ)sizeof(txn);
}

LOCAL bool lsm303_init(ID dd)
{
    /* 0x57: normal operation, 100 Hz data rate, XYZ accelerometer enabled. */
    if (!write_register(dd, ADDR_ACCEL, CTRL_REG1_A, 0x57U)) return false;

    /* Enable high-resolution 12-bit data and block-data-update, +/-2 g. */
    if (!write_register(dd, ADDR_ACCEL, CTRL_REG4_A, 0x88U)) return false;

    /* Magnetometer in continuous-conversion mode (same as original imu.c). */
    if (!write_register(dd, ADDR_MAG, MR_REG_M, 0x00U)) return false;
    return true;
}

LOCAL bool read_accelerometer(ID dd, int16_t *ax, int16_t *ay, int16_t *az)
{
    UB data[6];
    if (!read_registers(dd, ADDR_ACCEL, OUT_X_L_A | 0x80U, data, 6)) return false;

    /* Accelerometer: little-endian, signed, left-justified 12-bit values. */
    *ax = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8)) >> 4;
    *ay = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8)) >> 4;
    *az = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8)) >> 4;
    return true;
}

LOCAL bool read_magnetometer(ID dd, int16_t *mx, int16_t *my, int16_t *mz)
{
    UB data[6];
    if (!read_registers(dd, ADDR_MAG, OUT_X_H_M, data, 6)) return false;

    /* Magnetometer: big-endian; its register sequence is X, Z, then Y. */
    *mx = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    *mz = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    *my = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    return true;
}

/* Integer square root avoids requiring -lm for sqrtf() in the Make build. */
LOCAL uint32_t isqrt_u32(uint32_t n)
{
    uint32_t result = 0;
    uint32_t bit = (uint32_t)1U << 30;
    while (bit > n) bit >>= 2;
    while (bit != 0U) {
        if (n >= result + bit) {
            n -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

LOCAL bool operating_time_ms(uint32_t *ms)
{
    SYSTIM t;
    if (tk_get_otm(&t) < E_OK) return false;
    *ms = (uint32_t)t.lo;
    return true;
}

EXPORT void imu_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    /* 'iica' is the RP2040 port's registered I2C0 device name. */
    ID dd = tk_opn_dev((UB *)"iica", TD_UPDATE);
    if (dd <= E_OK) {
        tm_printf((UB *)"IMU: unable to open I2C0 (er=%d)\n", (INT)dd);
        return;
    }

    INT i;
    for (i = 0; i < 200 && tm_usb_state() < 2; ++i) tk_dly_tsk(100);
    tk_dly_tsk(500);

    tm_printf((UB *)"\n=== RoboPico LSM303DLHC microT test: GP4/GP5 ===\n");
    if (!lsm303_init(dd)) {
        tm_printf((UB *)"IMU: failed to initialize accelerometer or magnetometer\n");
        (void)tk_cls_dev(dd, 0);
        return;
    }
    tm_printf((UB *)"IMU initialized: accelerometer 0x19; magnetometer 0x1E\n");

    int16_t ax = 0, ay = 0, az = 0;
    int16_t mx = 0, my = 0, mz = 0;
    int32_t previous_magnitude = 0;
    bool have_previous = false;
    uint32_t previous_time_ms = 0;
    uint32_t last_log_ms = 0;

    if (!operating_time_ms(&previous_time_ms)) {
        tm_printf((UB *)"IMU: cannot read kernel operating time\n");
        (void)tk_cls_dev(dd, 0);
        return;
    }
    last_log_ms = previous_time_ms - LOG_INTERVAL_MS;

    while (1) {
        uint32_t now_ms;
        if (!operating_time_ms(&now_ms)) {
            tm_printf((UB *)"IMU: operating-time read failed\n");
            tk_dly_tsk(SAMPLE_INTERVAL_MS);
            continue;
        }

        bool accel_ok = read_accelerometer(dd, &ax, &ay, &az);
        bool mag_ok = read_magnetometer(dd, &mx, &my, &mz);

        if (accel_ok) {
            int32_t x = ax, y = ay, z = az;
            uint32_t sum_sq = (uint32_t)(x*x + y*y + z*z);
            int32_t magnitude = (int32_t)isqrt_u32(sum_sq);
            uint32_t elapsed_ms = now_ms - previous_time_ms;
            int32_t jerk = (have_previous && elapsed_ms != 0U)
                ? (magnitude - previous_magnitude) * 1000 / (int32_t)elapsed_ms
                : 0;

            if ((uint32_t)(now_ms - last_log_ms) >= LOG_INTERVAL_MS) {
                if (mag_ok) {
                    tm_printf((UB *)"t=%u ms ACCEL_mg=(%d,%d,%d) MAGNITUDE_mg=%d JERK_mg_s=%d MAG=(%d,%d,%d)\n",
                              (UW)now_ms, (INT)ax, (INT)ay, (INT)az,
                              (INT)magnitude, (INT)jerk,
                              (INT)mx, (INT)my, (INT)mz);
                } else {
                    tm_printf((UB *)"t=%u ms ACCEL_mg=(%d,%d,%d) MAGNITUDE_mg=%d JERK_mg_s=%d MAG=I2C_ERROR\n",
                              (UW)now_ms, (INT)ax, (INT)ay, (INT)az,
                              (INT)magnitude, (INT)jerk);
                }
                last_log_ms = now_ms;
            }
            previous_magnitude = magnitude;
            previous_time_ms = now_ms;
            have_previous = true;
        } else if ((uint32_t)(now_ms - last_log_ms) >= LOG_INTERVAL_MS) {
            tm_printf((UB *)"t=%u ms ACCEL=I2C_ERROR MAG=%s\n",
                      (UW)now_ms, mag_ok ? (UB *)"OK" : (UB *)"I2C_ERROR");
            last_log_ms = now_ms;
        }
        tk_dly_tsk(SAMPLE_INTERVAL_MS);
    }
}
