/** @file servo.h
 * @brief SG90 pan servo on a 50 Hz PWM channel.
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <tk/tkernel.h>

/** @brief Configure the PWM slice for 50 Hz and centre on FRONT. */
ER servo_init (void);
/** @brief Drive to FRONT whatever the last position was.
 * @return Milliseconds to wait: a full turn, as the start position is unknown.
 */
uint32_t servo_home (void);
/** @brief Move to an angle (positive is left), clamped to the usable sweep.
 * @return Milliseconds to wait before pinging.
 */
uint32_t servo_set_angle (int32_t angle_deg);
/** @brief Angle last commanded, after clamping; zero is FRONT. */
int32_t servo_angle_get (void);
/** @brief Usable sweep to the left: the configured limit, reduced if the
 *  pulse for it would fall outside the servo's pulse range.
 */
int32_t servo_left_limit_deg (void);
/** @brief Usable sweep to the right, reduced in the same way. */
int32_t servo_right_limit_deg (void);

#endif /* SERVO_H */

/*** end of file ***/

