/** @file servo.h
 * @brief Servo PWM interface for the obstacle subsystem.
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <tk/tkernel.h>

/** @brief Configure the servo PWM and move to FRONT. */
ER servo_init (void);

/** @brief Command an angle relative to FRONT; positive is left.
 *  @return Settling time in milliseconds.
 */
uint32_t servo_set_angle (int32_t angle_deg);

#endif /* SERVO_H */

/*** end of file ***/

