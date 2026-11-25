/** @file lsm303.c
 * @brief LSM303DLHC accelerometer and magnetometer I2C access.
 * @note Register values follow the ST LSM303DLHC datasheet.
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
#define LSM303_WORD_SIGN_THRESHOLD 32768
#define LSM303_WORD_RANGE 65536
#define LSM303_ACCEL_SIGN_THRESHOLD 2048
#define LSM303_ACCEL_RANGE 4096
/* CTRL_REG1_A: 100 Hz output, X/Y/Z enabled. */
#define LSM303_ACCEL_CTRL1_VALUE 0x57U
/* CTRL_REG4_A: block data update, high resolution, +/-2 g. */
#define LSM303_ACCEL_CTRL4_VALUE 0x88U
/* MR_REG_M: continuous conversion. */
#define LSM303_MAG_CONTINUOUS 0x00U
/* Sub-address MSB: auto-increment over a multi-byte read. */
#define LSM303_AUTO_INCREMENT 0x80U
/* Byte offsets of the second and third axis words in an output block. */
#define LSM303_AXIS1_BYTE0 2
#define LSM303_AXIS1_BYTE1 3
#define LSM303_AXIS2_BYTE0 4
#define LSM303_AXIS2_BYTE1 5
/* A register write sends the register address and one value byte. */
#define LSM303_WRITE_BYTES 2
/* Bits in a byte, to join the high and low bytes of a word. */
#define LSM303_BYTE_BITS 8U
/* Accelerometer words are left-justified: four padding bits. */
#define LSM303_ACCEL_PAD_BITS 4U

/** @brief Write one byte to a device register. */
static bool write_register (ID h_device, uint8_t address, uint8_t reg,
                            uint8_t value);
/** @brief Read consecutive registers with a repeated-start transfer. */
static bool read_registers (ID h_device, uint8_t address, uint8_t reg,
                            uint8_t * p_data, SZ length);
/** @brief Decode low-byte-first acceleration; discard four padding bits. */
static int16_t decode_accel (uint8_t low_byte, uint8_t high_byte);
/** @brief Decode high-byte-first magnetic data; no padding to discard. */
static int16_t decode_mag (uint8_t high_byte, uint8_t low_byte);

bool lsm303_init (ID h_device)
{
    if (!write_register(h_device, LSM303_ACCEL_ADDRESS, LSM303_CTRL_REG1,
                        LSM303_ACCEL_CTRL1_VALUE))
    {
        return false;
    }

    if (!write_register(h_device, LSM303_ACCEL_ADDRESS, LSM303_CTRL_REG4,
                        LSM303_ACCEL_CTRL4_VALUE))
    {
        return false;
    }

    return write_register(h_device, LSM303_MAG_ADDRESS, LSM303_MAG_MODE,
                          LSM303_MAG_CONTINUOUS);
}

bool lsm303_read_accel (ID h_device, int16_t * p_x, int16_t * p_y,
                        int16_t * p_z)
{
    uint8_t data[LSM303_AXIS_BYTES] = {0U};

    if (!read_registers(h_device, LSM303_ACCEL_ADDRESS,
                        LSM303_ACCEL_OUT | LSM303_AUTO_INCREMENT, data,
                        LSM303_AXIS_BYTES))
    {
        return false;
    }

    /* Register order: X low/high, Y low/high, Z low/high. */
    *p_x = decode_accel(data[0], data[1]);
    *p_y = decode_accel(data[LSM303_AXIS1_BYTE0], data[LSM303_AXIS1_BYTE1]);
    *p_z = decode_accel(data[LSM303_AXIS2_BYTE0], data[LSM303_AXIS2_BYTE1]);
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

    /* Register order: X high/low, Z high/low, Y high/low. */
    *p_x = decode_mag(data[0], data[1]);
    *p_z = decode_mag(data[LSM303_AXIS1_BYTE0], data[LSM303_AXIS1_BYTE1]);
    *p_y = decode_mag(data[LSM303_AXIS2_BYTE0], data[LSM303_AXIS2_BYTE1]);
    return true;
}

static bool write_register (ID h_device, uint8_t address, uint8_t reg,
                            uint8_t value)
{
    uint8_t config[LSM303_WRITE_BYTES] = {reg, value};
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
    SZ actual = 0;

    /* T_I2C_EXEC is a small fixed BSP structure, representable by SZ. */
    ER error = tk_swri_dev(h_device, TDN_I2C_EXEC, &transaction,
                           (SZ)sizeof(transaction), &actual);

    /* The same fixed structure size is representable by SZ. */
    return (error >= E_OK) && (actual == (SZ)sizeof(transaction));
}

static int16_t decode_accel (uint8_t low_byte, uint8_t high_byte)
{
    /* Join two bytes using an unsigned shift; raw is at most 65535. */
    uint32_t raw = ((uint32_t)high_byte << LSM303_BYTE_BITS) | low_byte;

    /* Bits 15..4 hold the reading. Bits 3..0 are padding.
     * After shifting, 0..4095 fits int32_t exactly.
     */
    int32_t value = (int32_t)(raw >> LSM303_ACCEL_PAD_BITS);

    /* In 12-bit two's complement, 2048..4095 means -2048..-1. */
    if (value >= LSM303_ACCEL_SIGN_THRESHOLD)
    {
        value -= LSM303_ACCEL_RANGE;
    }

    /* The decoded range -2048..2047 fits int16_t. */
    return (int16_t)value;
}

static int16_t decode_mag (uint8_t high_byte, uint8_t low_byte)
{
    /* Join two bytes using an unsigned shift; raw is at most 65535. */
    uint32_t raw = ((uint32_t)high_byte << LSM303_BYTE_BITS) | low_byte;

    /* The sensor already extends the sign to 16 bits: do not shift.
     * The unsigned word 0..65535 fits int32_t exactly.
     */
    int32_t value = (int32_t)raw;

    /* In 16-bit two's complement, 32768..65535 means -32768..-1. */
    if (value >= LSM303_WORD_SIGN_THRESHOLD)
    {
        value -= LSM303_WORD_RANGE;
    }

    /* Any decoded 16-bit word fits int16_t, including sensor status values. */
    return (int16_t)value;
}

/*** end of file ***/

