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

/* I2C0 pin configuration remains owned by the supplied device driver. */

#endif /* BOARD_CONFIG_H */

/*** end of file ***/

