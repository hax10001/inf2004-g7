/** @file tilt_math.h
 * @brief Pure tilt calculations independent of kernel headers.
 */
#ifndef TILT_MATH_H
#define TILT_MATH_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Convert sensor axes to degrees; reject undefined roll or outputs. */
bool tilt_math_angles (int16_t axis_x, int16_t axis_y, int16_t axis_z,
                       float * p_roll_deg, float * p_pitch_deg);
/** @brief Height of one wheel above the other, from the roll angle. */
float tilt_math_hump_height (float roll_deg, float wheel_spacing_mm);
/** @brief Calculate the heading level based on magnetometer readings. */
bool tilt_math_heading_level(
    int16_t mag_x,
    int16_t mag_y,
    float *p_heading_deg
);
#endif /* TILT_MATH_H */

/*** end of file ***/

