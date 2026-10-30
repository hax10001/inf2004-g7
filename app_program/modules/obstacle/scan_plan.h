/** @file scan_plan.h
 *
 * @brief Turns a servo range profile into a bypass plan; no I/O is done.
 */
#ifndef SCAN_PLAN_H
#define SCAN_PLAN_H

#include <stdbool.h>
#include <stdint.h>

#define SCAN_PLAN_CLEAR 0U
#define SCAN_PLAN_ROUTE 1U
#define SCAN_PLAN_NO_ROUTE 2U

/** @brief Compact sample: 4-bit angle index and 9-bit distance in cm.
 * Zero index is -70 degrees; index 14 is +70. Range 511 is invalid.
 */
typedef struct
{
    uint16_t packed;
} scan_plan_point_t;

/** @brief Encode a scan sample. Invalid readings use a sentinel. */
void scan_plan_point_set (scan_plan_point_t * p_point, int32_t angle_deg,
                          int32_t range_mm);
/** @brief Decode the sample's discrete angle in degrees. */
int32_t scan_plan_point_angle_deg (scan_plan_point_t const * p_point);
/** @brief Decode the conservative cm quantisation into mm; -1 invalid. */
int32_t scan_plan_point_range_mm (scan_plan_point_t const * p_point);

/** @brief A bypass plan packed into two words, with distances in cm. */
typedef struct
{
    uint32_t word_a;
    uint32_t word_b;
} scan_plan_bypass_t;

/** @brief Bypass heading change for an angle index.
 *  @param angle_index Index from scan_plan_angle_index().
 *  @return The angle in degrees.
 */
int32_t scan_plan_angle_deg (uint32_t angle_index);

/** @brief Find the object ahead, its edges, the clearer side and the path.
 *  @param[in] p_points Samples ordered from right to left.
 *  @param count Number of samples.
 *  @param[out] p_plan Receives the packed plan.
 */
void scan_plan_build (scan_plan_point_t const * p_points, int32_t count,
                      scan_plan_bypass_t * p_plan);

/** @brief SCAN_PLAN_CLEAR, SCAN_PLAN_ROUTE or SCAN_PLAN_NO_ROUTE. */
uint32_t scan_plan_result (scan_plan_bypass_t const * p_plan);
/** @brief True when the bypass goes round the left side. */
bool scan_plan_side_is_left (scan_plan_bypass_t const * p_plan);
/** @brief Index of the chosen bypass angle. */
uint32_t scan_plan_angle_index (scan_plan_bypass_t const * p_plan);
/** @brief Approximate object width in cm. */
int32_t scan_plan_width_cm (scan_plan_bypass_t const * p_plan);
/** @brief Distance to the nearest object face in cm. */
int32_t scan_plan_near_cm (scan_plan_bypass_t const * p_plan);
/** @brief Lateral path offset in cm. */
int32_t scan_plan_lateral_cm (scan_plan_bypass_t const * p_plan);
/** @brief Length of the diagonal run in cm. */
int32_t scan_plan_diag_cm (scan_plan_bypass_t const * p_plan);
/** @brief Sampled left radial clearance (0..62 cm; 63 unknown). */
int32_t scan_plan_left_clear_cm (scan_plan_bypass_t const * p_plan);
/** @brief Sampled right radial clearance (0..62 cm; 63 unknown). */
int32_t scan_plan_right_clear_cm (scan_plan_bypass_t const * p_plan);
/** @brief Closest point direction: -1 right, 0 front, 1 left. */
int32_t scan_plan_near_direction (scan_plan_bypass_t const * p_plan);
#endif /* SCAN_PLAN_H */

/*** end of file ***/

