/** @file ultrasonic.h
 * @brief HC-SR04 ranging; the echo width is counted by PWM hardware.
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
    US_OK = 0,
    US_NO_ECHO,     /**< Echo never rose: wiring, power or sensor fault. */
    US_OUT_OF_RANGE /**< Echo too long or beyond OBS_FAR_MM. */
} us_result_t;

/** @brief Claim TRIG and ECHO and start the gated pulse counter. */
ER ultrasonic_init (void);
/** @brief One ping, sleeping in 1 ms steps; call from a task only.
 * @param p_mm Receives millimetres when US_OK is returned.
 */
us_result_t ultrasonic_ping (uint16_t * p_mm);
/** @brief Median of three values. */
uint16_t ultrasonic_median3 (uint16_t a, uint16_t b, uint16_t c);

#endif /* ULTRASONIC_H */

/*** end of file ***/

