/** @file ultrasonic.h
 * @brief HC-SR04 interface for the obstacle subsystem.
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
    ULTRASONIC_OK = 0,
    ULTRASONIC_OUT_OF_RANGE,
    ULTRASONIC_NO_ECHO
} ultrasonic_result_t;

/** @brief Configure TRIG as an output and ECHO as a PWM-gated counter. */
ER ultrasonic_init (void);

/** @brief Send one trigger and measure the echo.
 *  @param[out] p_mm Receives the range in mm on ULTRASONIC_OK.
 *  @return ULTRASONIC_OK, ULTRASONIC_OUT_OF_RANGE or ULTRASONIC_NO_ECHO.
 */
ultrasonic_result_t ultrasonic_ping (int32_t * p_mm);

/** @brief Median of three values. */
int32_t ultrasonic_median_of_three (int32_t first, int32_t second,
                                    int32_t third);

#endif /* ULTRASONIC_H */

/*** end of file ***/

