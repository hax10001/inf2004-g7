/** @file motor.h
 * @brief Motor device access through the supplied BSP PWM API.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/** @brief Configure the tested PWM settings and zero all outputs. */
void motor_init (void);
/** @brief Zero all motor outputs. */
void motor_stop (void);
/** @brief Apply forward duty counts, preserving the left motor inversion.
 * @param left_pwm Left compare count, between zero and MOTOR_PWM_WRAP.
 * @param right_pwm Right compare count, between zero and MOTOR_PWM_WRAP.
 */
void motor_set_forward (uint32_t left_pwm, uint32_t right_pwm);

#define MOTOR_PWM_WRAP 12499U

#endif /* MOTOR_H */

/*** end of file ***/

