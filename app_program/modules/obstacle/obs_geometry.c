/** @file obs_geometry.c
 *
 * @brief Sensor mounting transform and swept-body tests in integer maths.
 *
 * The servo turns the transducer about its own rotation axis. With the
 * servo at bearing b the range starts at
 *
 *     origin = axis + arm * (cos b, sin b)
 *
 * where the axis sits OBS_SERVO_ARM_MM behind the transducer face measured
 * at FRONT. The body test separates the robot rectangle from a box on the
 * four rectangle axes, so it is exact for the rectangle and box used.
 */
#include "obs_geometry.h"
#include "obs_config.h"
#include <stddef.h>

#define TABLE_SIZE 91U
#define QUARTER_DEG 90
#define HALF_TURN_DEG 180
#define THREE_QUARTER_DEG 270
#define FULL_TURN_DEG 360
#define HALF 2
#define CORNER_COUNT 4U

/* The servo rotation axis, from the transducer position measured at FRONT
 * and the arm that carries the transducer ahead of that axis.
 */
#define AXIS_FWD_MM (OBS_SENSOR_TO_AXLE_MM - OBS_SERVO_ARM_MM)
#define AXIS_LAT_MM (OBS_SENSOR_LATERAL_MM)
#define BODY_HALF_WIDTH_MM (OBS_ROBOT_WIDTH_MM / HALF)
#define WHEEL_HALF_TRACK_MM (OBS_TRACK_WIDTH_MM / HALF)

/** @brief Sine of 0 to 90 degrees in 1 degree steps, times 1000. */
static int16_t const g_sine_milli[TABLE_SIZE] = {
    0,   17,  35,  52,  70,  87,  105, 122, 139, 156, 174, 191, 208,
    225, 242, 259, 276, 292, 309, 326, 342, 358, 375, 391, 407, 423,
    438, 454, 469, 485, 500, 515, 530, 545, 559, 574, 588, 602, 616,
    629, 643, 656, 669, 682, 695, 707, 719, 731, 743, 755, 766, 777,
    788, 799, 809, 819, 829, 839, 848, 857, 866, 875, 883, 891, 899,
    906, 914, 921, 927, 934, 940, 946, 951, 956, 961, 966, 970, 974,
    978, 982, 985, 988, 990, 993, 995, 996, 998, 999, 999, 1000, 1000};

/** @brief Look up the table for 0 to 90 degrees. */
static int32_t table_sine (int32_t angle_deg);
/** @brief Smaller of two values. */
static int32_t min_int (int32_t first, int32_t second);
/** @brief Larger of two values. */
static int32_t max_int (int32_t first, int32_t second);

int32_t obs_geometry_sin_milli (int32_t angle_deg)
{
    int32_t angle  = angle_deg % FULL_TURN_DEG;
    int32_t result = 0;

    if (angle < 0)
    {
        angle += FULL_TURN_DEG;
    }

    if (angle <= QUARTER_DEG)
    {
        result = table_sine(angle);
    }
    else if (angle <= HALF_TURN_DEG)
    {
        result = table_sine(HALF_TURN_DEG - angle);
    }
    else if (angle <= THREE_QUARTER_DEG)
    {
        result = -table_sine(angle - HALF_TURN_DEG);
    }
    else
    {
        result = -table_sine(FULL_TURN_DEG - angle);
    }

    return result;
}

int32_t obs_geometry_cos_milli (int32_t angle_deg)
{
    return obs_geometry_sin_milli(angle_deg + QUARTER_DEG);
}

void obs_geometry_sensor_origin (int32_t bearing_deg, int32_t * p_fwd_mm,
                                 int32_t * p_lat_mm)
{
    if ((NULL != p_fwd_mm) && (NULL != p_lat_mm))
    {
        *p_fwd_mm = AXIS_FWD_MM + ((OBS_SERVO_ARM_MM *
                                    obs_geometry_cos_milli(bearing_deg)) /
                                   OBS_GEOMETRY_MILLI);
        *p_lat_mm = AXIS_LAT_MM + ((OBS_SERVO_ARM_MM *
                                    obs_geometry_sin_milli(bearing_deg)) /
                                   OBS_GEOMETRY_MILLI);
    }
}

void obs_geometry_echo_point (int32_t bearing_deg, int32_t range_mm,
                              int32_t * p_fwd_mm, int32_t * p_lat_mm)
{
    int32_t origin_fwd = 0;
    int32_t origin_lat = 0;

    if ((NULL != p_fwd_mm) && (NULL != p_lat_mm))
    {
        obs_geometry_sensor_origin(bearing_deg, &origin_fwd, &origin_lat);
        *p_fwd_mm = origin_fwd + ((range_mm *
                                   obs_geometry_cos_milli(bearing_deg)) /
                                  OBS_GEOMETRY_MILLI);
        *p_lat_mm = origin_lat + ((range_mm *
                                   obs_geometry_sin_milli(bearing_deg)) /
                                  OBS_GEOMETRY_MILLI);
    }
}

void obs_geometry_pivot (obs_geometry_pose_t const * p_start,
                         int32_t pivot_side, int32_t turn_deg,
                         obs_geometry_pose_t * p_out)
{
    if ((NULL != p_start) && (NULL != p_out))
    {
        int32_t const sine   = obs_geometry_sin_milli(p_start->heading_deg);
        int32_t const cosine = obs_geometry_cos_milli(p_start->heading_deg);
        /* From the stopped wheel to the axle midpoint. */
        int32_t const arm_x =
            (pivot_side * WHEEL_HALF_TRACK_MM * sine) / OBS_GEOMETRY_MILLI;
        int32_t const arm_y =
            -(pivot_side * WHEEL_HALF_TRACK_MM * cosine) / OBS_GEOMETRY_MILLI;
        int32_t const turn_sin = obs_geometry_sin_milli(turn_deg);
        int32_t const turn_cos = obs_geometry_cos_milli(turn_deg);
        int32_t const pivot_x  = p_start->x_mm - arm_x;
        int32_t const pivot_y  = p_start->y_mm - arm_y;

        p_out->x_mm = pivot_x + (((arm_x * turn_cos) - (arm_y * turn_sin)) /
                                 OBS_GEOMETRY_MILLI);
        p_out->y_mm = pivot_y + (((arm_x * turn_sin) + (arm_y * turn_cos)) /
                                 OBS_GEOMETRY_MILLI);
        p_out->heading_deg = p_start->heading_deg + turn_deg;
    }
}

void obs_geometry_advance (obs_geometry_pose_t const * p_start,
                           int32_t distance_mm, obs_geometry_pose_t * p_out)
{
    if ((NULL != p_start) && (NULL != p_out))
    {
        int32_t const heading = p_start->heading_deg;

        p_out->x_mm = p_start->x_mm + ((distance_mm *
                                        obs_geometry_cos_milli(heading)) /
                                       OBS_GEOMETRY_MILLI);
        p_out->y_mm = p_start->y_mm + ((distance_mm *
                                        obs_geometry_sin_milli(heading)) /
                                       OBS_GEOMETRY_MILLI);
        p_out->heading_deg = heading;
    }
}

bool obs_geometry_body_hits_box (obs_geometry_pose_t const * p_pose,
                                 obs_geometry_box_t const *  p_box,
                                 int32_t                     clearance_mm)
{
    static int32_t const corner_fwd[CORNER_COUNT] = {
        OBS_ROBOT_FRONT_MM, OBS_ROBOT_FRONT_MM, -OBS_ROBOT_REAR_MM,
        -OBS_ROBOT_REAR_MM};
    static int32_t const corner_lat[CORNER_COUNT] = {
        BODY_HALF_WIDTH_MM, -BODY_HALF_WIDTH_MM, BODY_HALF_WIDTH_MM,
        -BODY_HALF_WIDTH_MM};
    bool b_hit = false;

    if ((NULL != p_pose) && (NULL != p_box))
    {
        int32_t const sine   = obs_geometry_sin_milli(p_pose->heading_deg);
        int32_t const cosine = obs_geometry_cos_milli(p_pose->heading_deg);
        int32_t const x_low  = p_box->x_low_mm - clearance_mm;
        int32_t const x_high = p_box->x_high_mm + clearance_mm;
        int32_t const y_low  = p_box->y_low_mm - clearance_mm;
        int32_t const y_high = p_box->y_high_mm + clearance_mm;
        int32_t const box_x[CORNER_COUNT] = {x_low, x_high, x_low, x_high};
        int32_t const box_y[CORNER_COUNT] = {y_low, y_low, y_high, y_high};
        int32_t       body_x_min = INT32_MAX;
        int32_t       body_x_max = INT32_MIN;
        int32_t       body_y_min = INT32_MAX;
        int32_t       body_y_max = INT32_MIN;
        int32_t       along_min  = INT32_MAX;
        int32_t       along_max  = INT32_MIN;
        int32_t       across_min = INT32_MAX;
        int32_t       across_max = INT32_MIN;

        for (uint32_t index = 0U; index < CORNER_COUNT; index++)
        {
            int32_t const world_x =
                p_pose->x_mm + (((corner_fwd[index] * cosine) -
                                 (corner_lat[index] * sine)) /
                                OBS_GEOMETRY_MILLI);
            int32_t const world_y =
                p_pose->y_mm + (((corner_fwd[index] * sine) +
                                 (corner_lat[index] * cosine)) /
                                OBS_GEOMETRY_MILLI);
            int32_t const rel_x = box_x[index] - p_pose->x_mm;
            int32_t const rel_y = box_y[index] - p_pose->y_mm;
            int32_t const along =
                ((rel_x * cosine) + (rel_y * sine)) / OBS_GEOMETRY_MILLI;
            int32_t const across =
                ((rel_y * cosine) - (rel_x * sine)) / OBS_GEOMETRY_MILLI;

            body_x_min = min_int(body_x_min, world_x);
            body_x_max = max_int(body_x_max, world_x);
            body_y_min = min_int(body_y_min, world_y);
            body_y_max = max_int(body_y_max, world_y);
            along_min  = min_int(along_min, along);
            along_max  = max_int(along_max, along);
            across_min = min_int(across_min, across);
            across_max = max_int(across_max, across);
        }

        /* Overlap needs every one of the four axes to overlap. */
        b_hit = (body_x_max >= x_low) && (body_x_min <= x_high) &&
                (body_y_max >= y_low) && (body_y_min <= y_high) &&
                (along_max >= -OBS_ROBOT_REAR_MM) &&
                (along_min <= OBS_ROBOT_FRONT_MM) &&
                (across_max >= -BODY_HALF_WIDTH_MM) &&
                (across_min <= BODY_HALF_WIDTH_MM);
    }

    return b_hit;
}

static int32_t table_sine (int32_t angle_deg)
{
    /* The callers keep the angle within 0 to 90, so the index is valid. */
    return (int32_t)g_sine_milli[(uint32_t)angle_deg];
}

static int32_t min_int (int32_t first, int32_t second)
{
    return (first < second) ? first : second;
}

static int32_t max_int (int32_t first, int32_t second)
{
    return (first > second) ? first : second;
}

/*** end of file ***/

