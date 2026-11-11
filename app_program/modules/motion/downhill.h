/** @file downhill.h
 * @brief Controller-owned downhill target governor (no direct motor writes).
 */
#ifndef DOWNHILL_H
#define DOWNHILL_H

#include <stdbool.h>
#include <stdint.h>

/* Stored states use minimum bit widths; one controller task owns writes. */
typedef struct
{
    signed int filtered_pitch_x10:11; /**< -900..+900 deci-degrees. */
    unsigned int scale_percent:7;    /**< 65..100, integer percent. */
    unsigned int engaged:1;
    unsigned int initialized:1;
    unsigned int fault:1;
    unsigned int confirm:2;          /**< 0..3 consecutive samples. */
    unsigned int release:3;          /**< 0..4 consecutive samples. */
    unsigned int stale:5;            /**< 0..20 missing control periods. */
    unsigned int overspeed:4;        /**< 0..10 runaway periods. */
} downhill_governor_t;

void downhill_reset (downhill_governor_t * p_state);
void downhill_sample (downhill_governor_t * p_state,
                      float pitch_deg, float roll_deg,
                      bool fresh, bool moving);
/* The IMU's already-confirmed hump descent is independent of pitch sign. */
void downhill_sample_terrain (downhill_governor_t * p_state,
                              float pitch_deg, float roll_deg,
                              bool fresh, bool moving, bool descending);
void downhill_scale_pair (downhill_governor_t const * p_state,
                          float target_left, float target_right,
                          float * p_left, float * p_right);
bool downhill_observe_speed (downhill_governor_t * p_state,
                             float target_left, float target_right,
                             float measured_left, float measured_right);
bool downhill_wheel_over_target (downhill_governor_t const * p_state,
                                 float target, float measured);

#endif /* DOWNHILL_H */

/*** end of file ***/

