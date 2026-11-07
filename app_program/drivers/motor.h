/** @file motor.h
 * @brief Motor device access through the supplied BSP PWM API.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/** @brief Configure the tested PWM settings and zero all outputs. */
void motor_init (void);
/** @brief Zero all motor outputs (wheels coast). */
void motor_stop (void);
/** @brief Drive both inputs of each H-bridge high: short brake.
 * @note Robo Pico's driver brakes with both inputs high; hold it briefly,
 * then call motor_stop() to release.
 */
void motor_brake (void);
/** @brief Apply forward duty counts, preserving the left motor inversion.
 * @param left_pwm Left compare count, between zero and MOTOR_PWM_WRAP.
 * @param right_pwm Right compare count, between zero and MOTOR_PWM_WRAP.
 */
void motor_set_forward (uint32_t left_pwm, uint32_t right_pwm);
/** Signed per-wheel PWM for bounded, controller-owned turns. */
void motor_set_directional (int32_t left_pwm, int32_t right_pwm);
/** Last PWM compare counts requested by the motor owner, not wheel motion. */
void motor_last_command (uint16_t * left, uint16_t * right);

#define MOTOR_PWM_WRAP 12499U

#endif /* MOTOR_H */

/*** end of file ***/

