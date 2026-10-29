/** @file servo.h
 *
 * @brief SG90 pan servo on a 50 Hz PWM channel.
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <tk/tkernel.h>

/** @brief Configure the PWM slice for 50 Hz and centre on FRONT.
 *  @return E_OK, or a BSP error code.
 */
ER servo_init (void);

/** @brief Move to an angle; positive is left. The angle is clamped.
 *  @param angle_deg Requested angle in degrees from FRONT.
 *  @return Milliseconds to wait before pinging.
 */
uint32_t servo_set_angle (int32_t angle_deg);

#endif /* SERVO_H */

/*** end of file ***/

