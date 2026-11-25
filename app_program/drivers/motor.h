/** @file motor.h
 * @brief Motor device access through the supplied BSP PWM API.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/** @brief Configure motor PWM and zero all outputs. */
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
/** @brief Positive = forward PWM; negative = proportional short-brake or
 * coast PWM, never reverse drive. Controller task only. */
void motor_set_regulated(int32_t left_pwm, int32_t right_pwm);
/** @brief Signed per-wheel PWM for bounded, controller-owned turns. */
void motor_set_directional (int32_t left_pwm, int32_t right_pwm);
/** @brief Last requested PWM compare counts (not measured wheel motion). */
void motor_last_command (uint16_t * p_left, uint16_t * p_right);

/* PWM counter top: duty counts run 0..MOTOR_PWM_WRAP. */
#define MOTOR_PWM_WRAP 12499U

#endif /* MOTOR_H */

/*** end of file ***/

