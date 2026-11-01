/** @file obs_geom.h
 * @brief Conservative horizontal footprint and clearance calculations.
 */
#ifndef OBS_GEOM_H
#define OBS_GEOM_H

#include <stdbool.h>
#include <stdint.h>

/** @brief True only for explicitly calibrated, internally valid geometry. */
bool obs_geom_is_valid (void);
/** @brief Approximate sine times 1000, interpolated from a 5-degree table.
 * @param heading_ddeg Signed angle in tenths of a degree; wraps each turn.
 */
int32_t obs_geom_sine_milli (int32_t heading_ddeg);
/** @brief Ceiling of the axle-to-farthest-corner radius, in mm. */
int32_t obs_geom_radius_mm (void);
/** @brief Bound the combined displacement of two stopped-wheel turns. */
int32_t obs_geom_turn_offset_mm (void);
/** @brief Body projection plus uncertainty, braking and movement reserve. */
int32_t obs_geom_clearance_mm (int32_t angle_deg);
/** @brief Distance after a confirmed side edge, including turn-back sweep. */
int32_t obs_geom_rear_clear_mm (void);
/** @brief Static footprint inflation used to evaluate a bypass corridor. */
int32_t obs_geom_padding_mm (void);
/** @brief Required forward sonar depth for a bounded 90-degree detour.
 * @param lateral_mm Rounded-up sideways run. Heading must stay within 2 deg.
 */
int32_t obs_geom_sideways_depth_mm (int32_t lateral_mm);

#endif /* OBS_GEOM_H */

/*** end of file ***/

