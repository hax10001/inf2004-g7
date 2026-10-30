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

/** @brief One scan sample; positive angles look left of FRONT. */
typedef struct
{
    int32_t angle_deg;
    int32_t range_mm;
} scan_plan_point_t;

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
/** @brief Distance to pass the object by dead reckoning, in cm. */
int32_t scan_plan_fallback_cm (scan_plan_bypass_t const * p_plan);

#endif /* SCAN_PLAN_H */

/*** end of file ***/

