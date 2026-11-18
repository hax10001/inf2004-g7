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

/** @brief Signed sensor-axis jerk components, in mg/s. */
typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
} terrain_jerk_vector;

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
    uint8_t hump_count;           /**< Completed humps this run. */
    /* Three phases + six booleans in eight bits. The terrain task
     * publishes and the consumers read copies under the terrain mutex. */
    unsigned char hump_phase:2;
    unsigned char onHump:1;
    unsigned char height_valid:1;
    unsigned char b_valid:1;
    unsigned char heading_valid:1;
    unsigned char b_collision:1;
    /** Still and level for APP_LIFT_REST_MSEC: standing on a surface even
     *  where both line sensors read black (a junction). */
    unsigned char b_resting:1;
    uint32_t timestamp_msec;
    // magnetic heading
    float heading_deg;
    uint32_t collision_sequence;
} terrain_snapshot;
/* Acknowledges only the event that the controller observed. */
ER terrain_ack_collision(uint32_t sequence);
/** @brief Controller enables impact detection only for an authorized run.
 * Disabling prevents new events; an existing run fault still needs STOP ACK.
 */
ER terrain_set_collision_enabled(bool enabled);
bool terrain_collision_enabled(void);
/** @brief Controller reports whether a line sensor sees the floor.
 * @note The IR sensors read black in the air, so a white reading means
 * the robot is standing on the floor. Used only by the lift estimate.
 */
ER terrain_set_floor_contact (bool b_contact);
/** @brief Latest floor report; false before any report or if locking fails. */
bool terrain_floor_contact (void);

/** @brief Initialise subsystem resources; device open remains task-owned. */
ER terrain_init (void);
/** @brief Create the IMU task and start it when APP_IMU_ENABLED is set. */
ER terrain_start (void);
/** @brief Compute magnitude for signed 12-bit accelerometer axes. */
int32_t terrain_magnitude (int16_t axis_x, int16_t axis_y, int16_t axis_z);
/** @brief Compute the signed rate of change of acceleration magnitude, mg/s. */
int32_t terrain_jerk (int32_t magnitude, int32_t previous,
                      uint32_t elapsed_msec);
/** @brief Compute XYZ jerk from current/previous acceleration axes in mg.
 * @note Returns zero components when elapsed time is zero. Keeps sensor axes.
 */
terrain_jerk_vector terrain_vector_jerk (
    int16_t current_x, int16_t current_y, int16_t current_z,
    int16_t previous_x, int16_t previous_y, int16_t previous_z,
    uint32_t elapsed_msec);
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
/** @brief Check if the robot is level. */
bool terrain_is_level(terrain_snapshot const *p_snapshot);
/** @brief Compute the magnetic heading based on the magnetometer readings. */
ER terrain_heading_level(int16_t mag_x, int16_t mag_y, float *p_heading_deg);
/** @brief Publish a new terrain snapshot.
 * @note This function is thread-safe and should be called from the IMU task.
 */
ER terrain_publish_snapshot (terrain_snapshot const *p_snapshot);

/** @brief Get the current terrain snapshot.
 * @note This function is thread-safe and should be called from the IMU task.
 */
ER terrain_get_snapshot (terrain_snapshot *p_snapshot);
/** @brief Clear an impact event following an explicit operator STOP/rearm.
 * @note Protected by the same mutex as the IMU snapshot writer.
 */
ER terrain_acknowledge_collision (void);
#endif /* TERRAIN_H */

/*** end of file ***/

