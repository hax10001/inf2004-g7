/** @file tilt_math.c
 * @brief Gravity-based roll and pitch using the toolchain math library.
 * @note Keep math.h separate from T-Kernel's incompatible size_t typedef.
 */
#include "tilt_math.h"
#include <math.h>
#include <stddef.h>

#define TILT_MATH_RAD_TO_DEG 57.2957795F

bool tilt_math_angles (int16_t axis_x, int16_t axis_y, int16_t axis_z,
                       float * p_roll_deg, float * p_pitch_deg)
{
    // Checks for null pointers and invalid axis values to
    // prevent division by zero or undefined behavior.
    if ((NULL == p_roll_deg) || (NULL == p_pitch_deg) ||
        ((0 == axis_y) && (0 == axis_z)))
    {
        return false;
    }

    /* Every int16_t is exactly representable by float on this target.
     * Convert before multiplication to avoid integer overflow.
     */
    float accel_x = (float)axis_x;
    float accel_y = (float)axis_y;
    float accel_z = (float)axis_z;

    *p_roll_deg = atan2f(accel_y, accel_z) * TILT_MATH_RAD_TO_DEG;
    *p_pitch_deg = atan2f(-accel_x,
        sqrtf(accel_y * accel_y + accel_z * accel_z))
        * TILT_MATH_RAD_TO_DEG;
    return true;
}

float tilt_math_hump_height (float roll_deg, float wheel_spacing_mm)
{
    float roll_rad = fabsf(roll_deg) * 0.0174532925F;

    return wheel_spacing_mm * sinf(roll_rad);
}
/*** end of file ***/

