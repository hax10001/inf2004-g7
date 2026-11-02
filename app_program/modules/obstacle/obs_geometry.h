/** @file obs_geometry.h
 *
 * @brief Integer geometry shared by the scan planner and the bypass
 * sequencer: trigonometry, the servo-mounted sensor origin and swept-body
 * clearance tests. No I/O is done.
 *
 * Vehicle frame: origin at the midpoint of the driven-wheel axle, x
 * forward, y to the left, angles positive to the left. A range from the
 * HC-SR04 starts at the transducer face, which the servo carries round its
 * own rotation axis, so it never starts at the axle.
 */
#ifndef OBS_GEOMETRY_H
#define OBS_GEOMETRY_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Scale of the sine and cosine results. */
#define OBS_GEOMETRY_MILLI 1000

/** @brief A pose in millimetres and whole degrees. */
typedef struct
{
    int32_t x_mm;
    int32_t y_mm;
    int32_t heading_deg; /**< Positive to the left of +x. */
} obs_geometry_pose_t;

/** @brief An axis-aligned box in the caller's frame. */
typedef struct
{
    int32_t x_low_mm;
    int32_t x_high_mm;
    int32_t y_low_mm;
    int32_t y_high_mm;
} obs_geometry_box_t;

/** @brief Sine of any whole-degree angle, times OBS_GEOMETRY_MILLI. */
int32_t obs_geometry_sin_milli (int32_t angle_deg);
/** @brief Cosine of any whole-degree angle, times OBS_GEOMETRY_MILLI. */
int32_t obs_geometry_cos_milli (int32_t angle_deg);

/** @brief Where a range starts, in the vehicle frame.
 *  @param bearing_deg Actual servo bearing from FRONT; positive is left.
 *  @param[out] p_fwd_mm Forward of the axle midpoint.
 *  @param[out] p_lat_mm Left of the axle midpoint.
 */
void obs_geometry_sensor_origin (int32_t bearing_deg, int32_t * p_fwd_mm,
                                 int32_t * p_lat_mm);

/** @brief The point an echo came from, in the vehicle frame, on the beam
 *  axis. The servo pans in the horizontal plane, so the bearing is a yaw.
 */
void obs_geometry_echo_point (int32_t bearing_deg, int32_t range_mm,
                              int32_t * p_fwd_mm, int32_t * p_lat_mm);

/** @brief Move the axle by a pivot about one stopped wheel.
 *  @param pivot_side 1 pivots on the left wheel, -1 on the right wheel.
 *  @param turn_deg Signed heading change; positive is a left turn.
 */
void obs_geometry_pivot (obs_geometry_pose_t const * p_start,
                         int32_t pivot_side, int32_t turn_deg,
                         obs_geometry_pose_t * p_out);

/** @brief Move the axle straight on along its heading. */
void obs_geometry_advance (obs_geometry_pose_t const * p_start,
                           int32_t distance_mm, obs_geometry_pose_t * p_out);

/** @brief True if the whole robot body at a pose comes within a clearance
 *  of a box. The body is the rectangle set by OBS_ROBOT_FRONT_MM,
 *  OBS_ROBOT_REAR_MM and OBS_ROBOT_WIDTH_MM about the axle midpoint.
 */
bool obs_geometry_body_hits_box (obs_geometry_pose_t const * p_pose,
                                 obs_geometry_box_t const *  p_box,
                                 int32_t                     clearance_mm);

#endif /* OBS_GEOMETRY_H */

/*** end of file ***/

