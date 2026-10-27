/** @file terrain.h
 * @brief IMU lifecycle, startup level reference, and motion measurements.
 */
#ifndef TERRAIN_H
#define TERRAIN_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

/** @brief Calibration values for roll and pitch angles. */
typedef struct
{
    float roll_zero_deg;
    float pitch_zero_deg;
    bool  b_valid;
} terrain_calibration;

/** @brief A snapshot of the current terrain state. */
typedef struct
{
    float roll_deg;
    float pitch_deg;
    // Future variables
    // uint8_t terrain_state;
    // uint16_t hump_height_mm;
    uint32_t timestamp_msec;
    bool b_valid;
} terrain_snapshot;

/** @brief Initialise subsystem resources; device open remains task-owned. */
ER terrain_init (void);
/** @brief Create the IMU task and start it when APP_IMU_ENABLED is set. */
ER terrain_start (void);
/** @brief Compute magnitude for signed 12-bit accelerometer axes. */
int32_t terrain_magnitude (int16_t axis_x, int16_t axis_y, int16_t axis_z);
/** @brief Compute the existing jerk measurement for elapsed milliseconds. */
int32_t terrain_jerk (int32_t magnitude, int32_t previous,
                      uint32_t elapsed_msec);
/** @brief Compute sensor-axis roll/pitch; reject an undefined roll vector.
 * @note X is forward, Y sideways, Z up after mounting-axis mapping.
 * Verify angle signs on the robot. Outputs are degrees, not radians.
 */
ER terrain_tilt (int16_t axis_x, int16_t axis_y, int16_t axis_z,
                 float *p_roll_deg, float *p_pitch_deg);
/** @brief Save averaged stationary axes as the startup reference, once.
 * @note Call only from the IMU task before motion. Reference is RAM-only.
 */
ER terrain_calibrate (int16_t axis_x, int16_t axis_y, int16_t axis_z);
/** @brief Return false until calibration is saved, or if locking fails. */
bool terrain_is_calibrated (void);
/** @brief Compute current angles relative to the saved startupreference. */
ER terrain_relative_tilt (int16_t axis_x, int16_t axis_y, int16_t axis_z,
                          float *p_roll_deg, float *p_pitch_deg);
/** @brief Publish a new terrain snapshot.
 * @note This function is thread-safe and should be called from the IMU task.
 */
ER terrain_publish_snapshot (terrain_snapshot const *p_snapshot);

/** @brief Get the current terrain snapshot.
 * @note This function is thread-safe and should be called from the IMU task.
 */
ER terrain_get_snapshot (terrain_snapshot *p_snapshot);
#endif /* TERRAIN_H */

/*** end of file ***/

