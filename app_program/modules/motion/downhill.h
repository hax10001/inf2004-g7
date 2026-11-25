/** @file downhill.h
 * @brief Controller-owned downhill target governor (no direct motor writes).
 */
#ifndef DOWNHILL_H
#define DOWNHILL_H

#include <stdbool.h>
#include <stdint.h>

/* Consecutive periods over DOWNHILL_RUNAWAY_RPM on a descent before the
 * overspeed fault latches; shorter overshoots are left to the PI loop. */
#define DOWNHILL_OVERSPEED_COUNT 10U
/* Full speed: no derating, in percent of the commanded target. */
#define DOWNHILL_FULL_PERCENT 100U

/* Stored states use minimum bit widths; one controller task owns writes. */
typedef struct
{
    int32_t    filtered_pitch_x10:11; /**< -900..+900 deci-degrees. */
    uint32_t     scale_percent:7;    /**< 65..100, integer percent. */
    uint32_t     engaged:1;
    uint32_t     initialized:1;
    uint32_t     fault:1;
    uint32_t     confirm:2;          /**< 0..3 consecutive samples. */
    uint32_t     release:3;          /**< 0..4 consecutive samples. */
    uint32_t     stale:5;            /**< 0..20 missing control periods. */
    uint32_t     overspeed:4;        /**< 0..10 runaway periods. */
} downhill_governor_t;

/** @brief Clear the governor to full speed, no fault. */
void downhill_reset (downhill_governor_t * p_state);
/** @brief Update the governor from calibrated pitch and roll. */
void downhill_sample (downhill_governor_t * p_state,
                      float pitch_deg, float roll_deg,
                      bool fresh, bool moving);
/** @brief As downhill_sample(), plus the IMU's confirmed hump descent,
 *  which engages the governor whatever the pitch sign. */
void downhill_sample_terrain (downhill_governor_t * p_state,
                              float pitch_deg, float roll_deg,
                              bool fresh, bool moving, bool descending);
/** @brief Scale both wheel targets by the governor (zero on a fault). */
void downhill_scale_pair (downhill_governor_t const * p_state,
                          float target_left, float target_right,
                          float * p_left, float * p_right);
/** @brief Count overspeed periods; return true once the fault latches. */
bool downhill_observe_speed (downhill_governor_t * p_state,
                             float target_left, float target_right,
                             float measured_left, float measured_right);
/** @brief True if a wheel exceeds the runaway speed on a descent. */
bool downhill_wheel_over_target (downhill_governor_t const * p_state,
                                 float target, float measured);

#endif /* DOWNHILL_H */

/*** end of file ***/

