/** @file terrain.h
 * @brief IMU subsystem lifecycle and integer motion measurements.
 */
#ifndef TERRAIN_H
#define TERRAIN_H

#include <stdint.h>
#include <tk/tkernel.h>

/** @brief Initialise subsystem resources; device open remains task-owned. */
ER terrain_init (void);
/** @brief Create the IMU task; leave it dormant in the tested configuration. */
ER terrain_start (void);
/** @brief Compute magnitude for signed 12-bit accelerometer axes. */
int32_t terrain_magnitude (int16_t axis_x, int16_t axis_y, int16_t axis_z);
/** @brief Compute the existing jerk measurement for elapsed milliseconds. */
int32_t terrain_jerk (int32_t magnitude, int32_t previous,
                      uint32_t elapsed_msec);

#endif /* TERRAIN_H */

/*** end of file ***/

