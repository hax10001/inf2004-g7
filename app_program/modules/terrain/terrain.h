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
    unsigned char b_valid:1; /**< 1-bit validity under calibration mutex. */
} terrain_calibration;

/** @brief Where the robot is on a hump. */
typedef enum
{
    TERRAIN_HUMP_LEVEL,
    TERRAIN_HUMP_CLIMBING,   /**< Estimated height rising. */
    TERRAIN_HUMP_DESCENDING  /**< Estimated height falling. */
} terrain_hump_phase;

/** @brief A snapshot of the current terrain state. */
typedef struct
{
    float roll_deg;
    float pitch_deg;
    /* One 32-bit word. A hump is at most the wheel spacing (110 mm), so
     * seven bits (0..127) hold each height; the lifted height is
     * 0..1022 mm in ten bits, as on the telemetry wire. */
    unsigned int hump_height_mm:7;      /**< Current estimate. */
    unsigned int last_hump_height_mm:7; /**< Peak of the last hump. */
    unsigned int max_hump_height_mm:7;  /**< Highest hump this run. */
    unsigned int lift_height_mm:10;     /**< Estimated height off floor. */
    /** At or above APP_LIFT_STOP_HEIGHT_MM with no floor under the line
     *  sensors. Not latched here: the controller latches its stop. */
    unsigned int b_lifted:1;
    uint32_t timestamp_msec;
    uint16_t heading_centideg;    /**< 0..36000 when heading_valid. */
    /* Three phases and five flags in seven bits. The terrain task
     * publishes and the consumers read copies under the terrain mutex. */
    unsigned char hump_phase:2;
    unsigned char onHump:1;
    unsigned char height_valid:1;
    unsigned char b_valid:1;
    unsigned char heading_valid:1;
    /** A fresh IR sample has at least one white line sensor. */
    unsigned char b_floor_contact:1;
} terrain_snapshot;

/** @brief Initialise subsystem resources; device open remains task-owned. */
ER terrain_init (void);
/** @brief Create the IMU task and start it when APP_IMU_ENABLED is set. */
ER terrain_start (void);
/** @brief Compute magnitude for signed 12-bit accelerometer axes. */
int32_t terrain_magnitude (int16_t axis_x, int16_t axis_y, int16_t axis_z);
/** @brief Compute sensor-axis roll/pitch; reject an undefined roll vector.
 * @note X is forward, Y sideways, Z up after mounting-axis mapping.
 * Verify angle signs on the robot. Outputs are degrees, not radians.
 */
ER terrain_tilt (int16_t axis_x, int16_t axis_y, int16_t axis_z,
                 float *p_roll_deg, float *p_pitch_deg);
/** @brief Compute current angles relative to the saved startupreference. */
ER terrain_relative_tilt (int16_t axis_x, int16_t axis_y, int16_t axis_z,
                          float *p_roll_deg, float *p_pitch_deg);
/** @brief Save averaged stationary axes as the startup reference, once.
 * @note Call only from the IMU task before motion. Reference is RAM-only.
 */
ER terrain_calibrate (int16_t axis_x, int16_t axis_y, int16_t axis_z);
/** @brief Return false until calibration is saved, or if locking fails. */
bool terrain_is_calibrated (void);
/** @brief Height of one wheel above the other, from the roll angle. */
float terrain_hump_height (float roll_deg);
/** @brief Compute the magnetic heading based on the magnetometer readings. */
ER terrain_heading_level(int16_t mag_x, int16_t mag_y, float *p_heading_deg);
/** @brief Publish a new terrain snapshot.
 * @note Thread-safe. Called by the IMU task.
 */
ER terrain_publish_snapshot (terrain_snapshot const *p_snapshot);

/** @brief Get the current terrain snapshot.
 * @note Thread-safe. Called by snapshot consumers.
 */
ER terrain_get_snapshot (terrain_snapshot *p_snapshot);
#endif /* TERRAIN_H */

/*** end of file ***/

