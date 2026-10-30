/** @file ultrasonic.h
 * @brief HC-SR04 ranging; the echo width is counted by PWM hardware.
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
    ULTRASONIC_OK = 0,
    ULTRASONIC_NO_ECHO,     /**< No target return, or sensor/wiring fault. */
    ULTRASONIC_OUT_OF_RANGE /**< Echo too long or beyond OBS_FAR_MM. */
} ultrasonic_result_t;

/** @brief Claim TRIG and ECHO and start the gated pulse counter. */
ER ultrasonic_init (void);
/** @brief One ping with a hardware-timed deadline; call from a task only.
 * Sleeps between polls; a 1 ms request rounds to the RTOS tick.
 * @param p_mm Receives millimetres when ULTRASONIC_OK is returned.
 */
ultrasonic_result_t ultrasonic_ping (int32_t * p_mm);
/** @brief Median of three values. */
int32_t ultrasonic_median_of_three (int32_t first, int32_t second,
                                    int32_t third);

#endif /* ULTRASONIC_H */

/*** end of file ***/

