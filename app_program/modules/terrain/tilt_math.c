/** @file tilt_math.c
 * @brief Gravity-based roll and pitch using the toolchain math library.
 * @note Keep math.h separate from T-Kernel's incompatible size_t typedef.
 */
#include "tilt_math.h"
#include <math.h>
#include <stddef.h>

/* Radians to degrees. */
#define TILT_MATH_RAD_TO_DEG 57.2957795F
/* Degrees to radians. */
#define TILT_MATH_DEG_TO_RAD 0.0174532925F
/* One full turn, degrees, to bring a heading into 0..360. */
#define TILT_MATH_FULL_TURN_DEG 360.0F

bool tilt_math_angles (int16_t axis_x, int16_t axis_y, int16_t axis_z,
                       float * p_roll_deg, float * p_pitch_deg)
{
    /* Roll is undefined when Y and Z are both zero. */
    if ((NULL == p_roll_deg) || (NULL == p_pitch_deg) ||
        ((0 == axis_y) && (0 == axis_z)))
    {
        return false;
    }

    /* Convert before multiplying to avoid integer overflow; every
     * int16_t is exact in float. */
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
    float roll_rad = fabsf(roll_deg) * TILT_MATH_DEG_TO_RAD;

    return wheel_spacing_mm * sinf(roll_rad);
}

bool tilt_math_heading_level (int16_t mag_x, int16_t mag_y,
                              float * p_heading_deg)
{
    if(p_heading_deg == NULL)
    {
        return false;
    }
    /* Magnetometer axes to robot axes. */
    float mag_forward = -(float)mag_y;
    float mag_left = (float)mag_x;

    if ((mag_forward == 0.0F) && (mag_left == 0.0F))
    {
        return false;
    }

    float heading = atan2f(mag_left, mag_forward) * TILT_MATH_RAD_TO_DEG;
    /* Bring the heading into 0..360 degrees. */
    if (heading < 0.0F)
    {
        heading += TILT_MATH_FULL_TURN_DEG;
    }

    *p_heading_deg = heading;

    return true;
}
/*** end of file ***/

