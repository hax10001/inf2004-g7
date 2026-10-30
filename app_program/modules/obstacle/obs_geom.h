/** @file obs_geom.h
 * @brief Conservative horizontal footprint and clearance calculations.
 */
#ifndef OBS_GEOM_H
#define OBS_GEOM_H

#include <stdbool.h>
#include <stdint.h>

/** @brief True only for explicitly calibrated, internally valid geometry. */
bool obs_geom_is_valid (void);
/** @brief Ceiling of the axle-to-farthest-corner radius, in mm. */
int32_t obs_geom_radius_mm (void);
/** @brief Body projection plus uncertainty, braking and movement reserve. */
int32_t obs_geom_clearance_mm (int32_t angle_deg);
/** @brief Distance after a confirmed side edge, including turn-back sweep. */
int32_t obs_geom_rear_clear_mm (void);
/** @brief Static footprint inflation used to evaluate a bypass corridor. */
int32_t obs_geom_padding_mm (void);

#endif /* OBS_GEOM_H */

/*** end of file ***/

