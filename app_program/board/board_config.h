/** @file board_config.h
 * @brief Wiring used by the tested robot application.
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define BOARD_IR_LEFT_PIN 6U
#define BOARD_IR_RIGHT_PIN 2U
/* WARNING: Preserve GP27; GP26 is not used by the working IR task. */
#define BOARD_IR_BARCODE_PIN 27U
#define BOARD_MOTOR_LEFT_A_PIN 8U
#define BOARD_MOTOR_LEFT_B_PIN 9U
#define BOARD_MOTOR_RIGHT_A_PIN 10U
#define BOARD_MOTOR_RIGHT_B_PIN 11U

/* WARNING: GP0/GP1 are UART0 at boot; the encoder takes them over, so a
 * CONSOLE=uart build loses its console. Use CONSOLE=usb_cdc.
 */
#define BOARD_ENC_LEFT_A_PIN 0U
#define BOARD_ENC_LEFT_B_PIN 1U
#define BOARD_ENC_RIGHT_A_PIN 7U
/* WARNING: GP28 is ADC2 at boot and must stay free of the VBAT-sense jumper. */
#define BOARD_ENC_RIGHT_B_PIN 28U
/* Counts per wheel revolution: both edges of channel A only (2 x PPR x gear).
 * WARNING: Unverified. Turn a wheel exactly once by hand and read COUNT from
 * the [MOTOR SPEED] log, then set this value.
 */
#define BOARD_ENC_COUNTS_PER_REV 1056U
/* WARNING: Unverified; measure the driven wheel's outer diameter. */
#define BOARD_WHEEL_DIAMETER_MM 65.0f

/* I2C0 pin configuration remains owned by the supplied device driver. */

#endif /* BOARD_CONFIG_H */

/*** end of file ***/

