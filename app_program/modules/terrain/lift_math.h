/** @file lift_math.h
 * @brief Pure lifted-height estimate from vertical acceleration.
 *
 * The robot has no downward range sensor, so the height above the floor
 * is an inertial ESTIMATE: the acceleration beyond gravity is integrated
 * twice while a lift is in progress. It is independent of kernel headers
 * so the same code runs in the host tests.
 */
#ifndef LIFT_MATH_H
#define LIFT_MATH_H

#include <stdbool.h>
#include <stdint.h>

/* Largest height reported; fits the 10-bit telemetry field (1023 there
 * means unavailable). */
#define LIFT_MATH_MAX_MM 1022U
/* Upward acceleration beyond rest, in mg, that starts a lift when held
 * for LIFT_MATH_START_SAMPLES samples in a row. Motor vibration stays
 * mostly within 50 mg; a hand picking the robot up exceeds it. */
#define LIFT_MATH_START_MG 50
#define LIFT_MATH_START_SAMPLES 3U
/* Changes smaller than this are sensor noise and are not integrated. */
#define LIFT_MATH_DEADBAND_MG 10
/* Acceleration staying inside this band for this many samples means the
 * robot is at rest (on the floor or held still): the speed is zeroed and
 * the resting offset is learned again, which stops the estimate drifting.
 */
#define LIFT_MATH_STILL_BAND_MG 16
#define LIFT_MATH_STILL_SAMPLES 25U
/* While a line sensor sees the floor the robot cannot be lifted, so the
 * height is held at zero; the speed is kept for this many samples (100 ms)
 * so a pickup is measured from its first push, then the estimate resets.
 * Longer let driving vibration build up a false height that showed the
 * moment both sensors went black on a junction (logged: lift stop 10 ms
 * after stopping on a junction). */
#define LIFT_MATH_FLOOR_SAMPLES 5U
/* A longer gap between samples than this restarts the estimate. */
#define LIFT_MATH_MAX_STEP_MSEC 100U

#if (LIFT_MATH_START_SAMPLES > 3U) || (LIFT_MATH_STILL_SAMPLES > 31U) || \
    (LIFT_MATH_FLOOR_SAMPLES > 31U)
#error Lift sample counts exceed their bit-field widths
#endif

/** @brief Estimator state; owned by one task, never shared with an ISR. */
typedef struct
{
    float   velocity_mm_s; /**< Upward speed. */
    float   height_mm;     /**< 0..LIFT_MATH_MAX_MM. */
    int16_t bias_mg;       /**< Resting offset from calibrated gravity. */
    int16_t still_min_mg;  /**< Lowest reading of the rest window. */
    int16_t still_max_mg;  /**< Highest reading of the rest window. */
    /* Counts saturate at their limits: 0..3, 0..25 and 0..25. */
    unsigned int start_count:2;
    unsigned int still_count:5;
    unsigned int floor_count:5;
    unsigned int b_active:1; /**< A lift is being integrated. */
} lift_math_state_t;

/** @brief Forget any lift: height zero, at rest. */
void lift_math_reset (lift_math_state_t * p_state);
/** @brief Advance the estimate by one accelerometer sample.
 * @param excess_mg Total acceleration minus calibrated gravity, in mg.
 * @param elapsed_msec Time since the previous sample.
 * @param b_floor A line sensor currently sees the floor.
 * @return Estimated height in whole millimetres, 0..LIFT_MATH_MAX_MM.
 */
uint16_t lift_math_step (lift_math_state_t * p_state, int16_t excess_mg,
                         uint32_t elapsed_msec, bool b_floor);
/** @brief Whether a lift is currently being measured. */
bool lift_math_is_active (lift_math_state_t const * p_state);
/** @brief Whether the readings have stayed within LIFT_MATH_STILL_BAND_MG
 *  for LIFT_MATH_STILL_SAMPLES samples (at rest, or held very still). */
bool lift_math_is_still (lift_math_state_t const * p_state);

#endif /* LIFT_MATH_H */

/*** end of file ***/

