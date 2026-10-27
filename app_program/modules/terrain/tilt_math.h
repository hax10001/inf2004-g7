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

#endif /* TILT_MATH_H */

/*** end of file ***/

