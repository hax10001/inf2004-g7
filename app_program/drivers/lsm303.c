/** @file lsm303.c
 * @brief Preserve the existing LSM303DLHC I2C transactions and register values.
 * @note Register definitions follow the existing robot IMU implementation.
 * Refer to STMicroelectronics LSM303DLHC datasheet, register descriptions.
 */
#include "lsm303.h"
#include "dev_i2c.h"

#define LSM303_ACCEL_ADDRESS 0x19U
#define LSM303_MAG_ADDRESS 0x1EU
#define LSM303_CTRL_REG1 0x20U
#define LSM303_CTRL_REG4 0x23U
#define LSM303_ACCEL_OUT 0x28U
#define LSM303_MAG_MODE 0x02U
#define LSM303_MAG_OUT 0x03U
#define LSM303_AXIS_BYTES 6
#define LSM303_SIGN_BIT 0x8000U
#define LSM303_WORD_RANGE 65536
#define LSM303_ACCEL_RANGE 4096

/** @brief Write one byte to a device register. */
static bool write_register (ID h_device, uint8_t address, uint8_t reg,
                            uint8_t value);
/** @brief Perform the original repeated-start register read. */
static bool read_registers (ID h_device, uint8_t address, uint8_t reg,
                            uint8_t * p_data, SZ length);
/** @brief Convert an unsigned register word without shifting signed data. */
static int16_t decode_axis (uint32_t raw, bool b_accel);

bool lsm303_init (ID h_device)
{
    if (!write_register(h_device, LSM303_ACCEL_ADDRESS, LSM303_CTRL_REG1,
                        0x57U))
    {
        return false;
    }

    if (!write_register(h_device, LSM303_ACCEL_ADDRESS, LSM303_CTRL_REG4,
                        0x88U))
    {
        return false;
    }

    return write_register(h_device, LSM303_MAG_ADDRESS, LSM303_MAG_MODE, 0x00U);
}

bool lsm303_read_accel (ID h_device, int16_t * p_x, int16_t * p_y,
                        int16_t * p_z)
{
    uint8_t data[LSM303_AXIS_BYTES] = {0U};

    if (!read_registers(h_device, LSM303_ACCEL_ADDRESS,
                        LSM303_ACCEL_OUT | 0x80U, data, LSM303_AXIS_BYTES))
    {
        return false;
    }

    /* Each byte fits uint32_t; unsigned shifts assemble the register word. */
    *p_x = decode_axis(data[0] | ((uint32_t)data[1] << 8U), true);
    *p_y = decode_axis(data[2] | ((uint32_t)data[3] << 8U), true);
    *p_z = decode_axis(data[4] | ((uint32_t)data[5] << 8U), true);
    return true;
}

bool lsm303_read_mag (ID h_device, int16_t * p_x, int16_t * p_y, int16_t * p_z)
{
    uint8_t data[LSM303_AXIS_BYTES] = {0U};

    if (!read_registers(h_device, LSM303_MAG_ADDRESS, LSM303_MAG_OUT, data,
                        LSM303_AXIS_BYTES))
    {
        return false;
    }

    /* Each byte fits uint32_t; the device register order is X, Z, then Y. */
    *p_x = decode_axis(((uint32_t)data[0] << 8U) | data[1], false);
    *p_z = decode_axis(((uint32_t)data[2] << 8U) | data[3], false);
    *p_y = decode_axis(((uint32_t)data[4] << 8U) | data[5], false);
    return true;
}

static bool write_register (ID h_device, uint8_t address, uint8_t reg,
                            uint8_t value)
{
    uint8_t config[2] = {reg, value};
    SZ      actual    = 0;

    /* The 7-bit address fits W; this fixed two-byte buffer fits SZ. */
    ER error =
        tk_swri_dev(h_device, (W)address, config, (SZ)sizeof(config), &actual);

    /* The fixed two-byte buffer size is representable by SZ. */
    return (error >= E_OK) && (actual == (SZ)sizeof(config));
}

static bool read_registers (ID h_device, uint8_t address, uint8_t reg,
                            uint8_t * p_data, SZ length)
{
    if ((NULL == p_data) || (length <= 0))
    {
        return false;
    }

    T_I2C_EXEC transaction = {.sadr     = address,
                              .snd_size = 1,
                              .snd_data = &reg,
                              .rcv_size = length,
                              .rcv_data = p_data};
    SZ         actual      = 0;

    /* T_I2C_EXEC is a small fixed BSP structure, representable by SZ. */
    ER error = tk_swri_dev(h_device, TDN_I2C_EXEC, &transaction,
                           (SZ)sizeof(transaction), &actual);

    /* The same fixed structure size is representable by SZ. */
    return (error >= E_OK) && (actual == (SZ)sizeof(transaction));
}

static int16_t decode_axis (uint32_t raw, bool b_accel)
{
    /* raw is assembled from exactly two bytes, so it is at most 65535. */
    int32_t value = (int32_t)raw;

    if (b_accel)
    {
        /* Unsigned shift gives 0..4095, exactly representable by int32_t. */
        value = (int32_t)(raw >> 4U);

        if (0U != (raw & LSM303_SIGN_BIT))
        {
            value -= LSM303_ACCEL_RANGE;
        }
    }
    else if (0U != (raw & LSM303_SIGN_BIT))
    {
        value -= LSM303_WORD_RANGE;
    }
    else
    {
        /* A positive magnetometer word already has its signed value. */
    }

    /* Sign extension bounds value to int16_t, including negative extrema. */
    return (int16_t)value;
}

/*** end of file ***/

