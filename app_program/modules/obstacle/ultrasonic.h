/** @file ultrasonic.h
 * @brief HC-SR04 pulse capture and median helper.
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
    ULTRASONIC_OK = 0,
    ULTRASONIC_NO_ECHO,
    ULTRASONIC_OUT_OF_RANGE
} ultrasonic_result_t;

ER ultrasonic_init (void);
ultrasonic_result_t ultrasonic_ping (int32_t * p_mm);
int32_t ultrasonic_median_of_three (int32_t first, int32_t second,
                                    int32_t third);

#endif /* ULTRASONIC_H */

/*** end of file ***/

