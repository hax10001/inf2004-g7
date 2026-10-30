/** @file obs_geom.c
 * @brief Bound every corner rather than treating the car as a point.
 */
#include "obs_geom.h"
#include "obs_config.h"

/** @brief Upper bound on cos(angle), rounding down to a five-degree entry. */
static int32_t cos_upper (int32_t angle_deg);

static int32_t cos_upper (int32_t angle_deg)
{
    static int32_t const table[19] = {
        1000, 997, 985, 966, 940, 907, 867, 820, 767, 708,
        643, 574, 500, 423, 343, 259, 174, 88, 0};

    if (angle_deg < 0)
    {
        angle_deg = 0;
    }
    else if (angle_deg > 90)
    {
        angle_deg = 90;
    }

    return table[angle_deg / 5];
}

bool obs_geom_is_valid (void)
{
    return (0 != OBS_GEOMETRY_VERIFIED) && (OBS_TRACK_WIDTH_MM > 0) &&
           (OBS_FRONT_OVERHANG_MM >= 0) && (OBS_REAR_OVERHANG_MM >= 0) &&
           (OBS_SENSOR_FORWARD_MM <= OBS_FRONT_OVERHANG_MM) &&
           (OBS_SENSOR_FORWARD_MM >= -OBS_REAR_OVERHANG_MM) &&
           ((2 * OBS_SENSOR_LEFT_MM) <= OBS_ROBOT_WIDTH_MM) &&
           ((2 * OBS_SENSOR_LEFT_MM) >= -OBS_ROBOT_WIDTH_MM);
}

int32_t obs_geom_radius_mm (void)
{
    int32_t const half = OBS_BODY_HALF_WIDTH_MM;
    int32_t const depth = (OBS_FRONT_OVERHANG_MM > OBS_REAR_OVERHANG_MM)
                              ? OBS_FRONT_OVERHANG_MM : OBS_REAR_OVERHANG_MM;
    int32_t radius = 0;

    while ((radius * radius) < ((half * half) + (depth * depth)))
    {
        radius++;
    }

    return radius;
}

int32_t obs_geom_padding_mm (void)
{
    return OBS_SAFETY_MARGIN_MM + OBS_POSITION_UNCERTAINTY_MM +
           OBS_BRAKING_ALLOWANCE_MM + OBS_GUARD_TRAVEL_MM;
}

int32_t obs_geom_clearance_mm (int32_t angle_deg)
{
    int32_t const half = OBS_BODY_HALF_WIDTH_MM;
    int32_t forward = OBS_FRONT_OVERHANG_MM - OBS_SENSOR_FORWARD_MM;

    if (0 != angle_deg)
    {
        int32_t const lateral = (angle_deg > 0)
                                    ? half - OBS_SENSOR_LEFT_MM
                                    : half + OBS_SENSOR_LEFT_MM;
        /* Ceil the projection at +/-70 degrees, never overstate room. */
        forward = ((forward * 343) + (lateral * 940) + 999) / 1000;
    }

    int32_t const angular = (obs_geom_radius_mm() *
        cos_upper(90 - OBS_ANGLE_UNCERTAINTY_DEG) + 999) / 1000;

    return forward + obs_geom_padding_mm() + angular;
}

int32_t obs_geom_rear_clear_mm (void)
{
    /* The 70-degree ray looks forward. Its blind wedge can hide an edge
     * ahead, increased by heading and servo uncertainty. Allow the rear to
     * sweep during turn-back, rather than using only the front sensor.
     */
    int32_t const angle = OBS_SIDE_LOOK_DEG -
                         ((OBS_ALIGN_TOL_DDEG + 9) / 10) -
                         OBS_ANGLE_UNCERTAINTY_DEG;

    return OBS_SENSOR_FORWARD_MM + obs_geom_radius_mm() +
           ((OBS_SIDE_NEAR_MM * cos_upper(angle) + 999) / 1000) +
           OBS_PASS_MARGIN_MM + obs_geom_padding_mm();
}

/*** end of file ***/

