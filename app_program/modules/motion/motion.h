/** @file motion.h
 * @brief Motion subsystem lifecycle and line-following movement interface.
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "modules/line_barcode/line_follow.h"
#include "modules/terrain/terrain.h"

typedef struct
{
    float    left_rpm;         /**< Measured; direction is not reported. */
    float    right_rpm;        /**< Measured; direction is not reported. */
    float    left_target_rpm;  /**< Ramped target; zero when stopped. */
    float    right_target_rpm; /**< Ramped target; zero when stopped. */
    float    steer;            /**< -0.8 to 0.8; positive turns left. */
    float    speed_mm_per_sec; /**< Vehicle speed: mean of both wheels. */
    int32_t  left_count;       /**< Raw encoder count, for distance. */
    int32_t  right_count;      /**< Raw encoder count, for distance. */
    uint32_t left_pwm;         /**< Applied compare count. */
    uint32_t right_pwm;        /**< Applied compare count. */
    uint32_t timestamp_usec;   /**< Encoder timer time of this update. */
    /* Published and copied under the motion mutex, not ISR-shared. */
    uint32_t     b_valid:1;   /**< False until the first update. */
    uint32_t     downhill_percent:7; /**< 65..100, or 0 before init. */
    uint32_t     downhill_engaged:1;
    uint32_t     downhill_fault:2; /**< 0 none, 1 stale, 2 runaway. */
    int32_t    downhill_pitch_x10:11; /**< -900..900 nose-down deci-deg. */
} motion_status_t;

/** @brief Initialise motors, encoders and status storage; outputs stopped. */
ER motion_init (void);
/** @brief Complete startup; this subsystem has no task of its own. */
ER motion_start (void);
/** @brief Steer from the line action and drive each wheel to its target.
 * After MOTION_SEARCH_AFTER_MM of travel with neither sensor on the line, a
 * forward action pivots on the spot to find it again: towards where it was
 * last seen, then the other way, then stops (see motion.c).
 * @note Call once per controller period; the integral assumes that rate.
 */
void motion_apply (line_follow_action_t action);
/** @brief What the line sensors saw this controller period, including
 *  touches latched between reads. Call before motion_apply() whenever line
 *  following: it is how the lost-line search knows where the line was last
 *  seen while navigation drives straight over a both-black run.
 */
void motion_line_seen (bool b_left, bool b_right);
/** @brief Where the line should be when following resumes with neither
 *  sensor on it (after a bypass): +1 left, -1 right. Lets the lost-line
 *  check lean and pivot that way first instead of driving straight on. */
void motion_line_hint (int32_t side);
/** @brief Stop like motion_apply(LINE_UNCERTAIN), but if the robot was
 * driving, short-brake for APP_JUNCTION_BRAKE_MSEC before coasting. Later
 * stop requests keep the brake until that time is used up.
 */
void motion_brake (void);
/** @brief Operator STOP: zero all motor pins before status/RTOS work. */
void motion_stop_now(void);
/** @brief Controller-owned forward wheel RPM targets for obstacle bypass. */
void motion_drive (float left_rpm, float right_rpm);
/** @brief Controller-only update of a fresh, calibrated terrain snapshot. */
void motion_downhill_sample (float pitch_deg, float roll_deg,
                             bool fresh, bool driving);
/** @brief Latched invalid-IMU/overspeed safety stop. */
bool motion_downhill_faulted (void);
/** @brief Clear only after an explicit STOP; never auto-restart. */
void motion_downhill_reset (void);
/** @brief Clear speed-controller state for a new run. */
void motion_reset_run(void);
/** @brief Feed a terrain snapshot to the downhill governor. */
void motion_observe_terrain(terrain_snapshot_t const *p_snapshot, uint32_t now);
/** @brief Downhill fault: 0 none, 1 IMU, 2 overspeed. */
uint8_t motion_downhill_fault(void);
/** @brief One period of a junction pivot (left, right or U-turn). */
void motion_apply_maneuver (uint8_t mapped_action);
/** @brief Copy the latest wheel speeds under their mutex, for telemetry.
 * @return E_OK, E_PAR for NULL, E_OBJ before motion_init, or a lock error.
 */
ER motion_get_status (motion_status_t * p_output);

#endif /* MOTION_H */

/*** end of file ***/

