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
    unsigned int b_valid:1;   /**< False until the first update. */
    unsigned int downhill_percent:7; /**< 65..100, or 0 before init. */
    unsigned int downhill_engaged:1;
    unsigned int downhill_fault:2; /**< 0 none, 1 stale, 2 runaway. */
    signed int downhill_pitch_x10:11; /**< -900..900 nose-down deci-deg. */
} motion_status_t;

/** @brief Initialise motors, encoders and status storage; outputs stopped. */
ER motion_init (void);
/** @brief Complete startup; this subsystem has no independent task yet. */
ER motion_start (void);
/** @brief Steer from the line action and drive each wheel to its target.
 * @note Call once per controller period; the integral assumes that rate.
 */
void motion_apply (line_follow_action_t action);
/** @brief Stop like motion_apply(LINE_UNCERTAIN), but if the robot was
 * driving, short-brake for APP_JUNCTION_BRAKE_MSEC before coasting. Later
 * stop requests keep the brake until that time is used up.
 */
void motion_brake (void);
/* Operator STOP: write zero to all motor pins before status/RTOS work. */
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
void motion_reset_run(void);
void motion_observe_terrain(terrain_snapshot const *snapshot, uint32_t now);
uint8_t motion_downhill_fault(void);
/** Constant bounded command for the autonomous junction controller only. */
void motion_apply_maneuver (uint8_t mapped_action);
/** @brief Copy the latest wheel speeds under their mutex, for telemetry.
 * @return E_OK, E_PAR for NULL, E_OBJ before motion_init, or a lock error.
 */
ER motion_get_status (motion_status_t * p_output);

#endif /* MOTION_H */

/*** end of file ***/

