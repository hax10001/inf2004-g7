/** @file wheel_speed.h
 * @brief Wheel RPM estimation from encoder counts and edge timestamps.
 */
#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>

/* A wheel slower than one edge per this interval is reported as stopped. */
#define WHEEL_SPEED_STOP_USEC 200000U

typedef struct
{
    int32_t  last_count;
    uint32_t last_edge_usec;
    float    rpm;
} wheel_speed_t;

/** @brief Start estimation from a sample, reporting zero speed. */
void wheel_speed_reset (wheel_speed_t * p_speed, int32_t count,
                        uint32_t edge_usec);
/** @brief Update the estimate; direction is ignored.
 * @param count Encoder count now.
 * @param edge_usec Time of the latest encoder edge.
 * @param now_usec Current time, used to decay the estimate without edges.
 * @return Wheel speed in revolutions per minute, never negative.
 */
float wheel_speed_update (wheel_speed_t * p_speed, int32_t count,
                          uint32_t edge_usec, uint32_t now_usec);

#endif /* WHEEL_SPEED_H */

/*** end of file ***/

