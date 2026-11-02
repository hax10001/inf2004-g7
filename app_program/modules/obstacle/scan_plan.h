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

/** @brief Range value of a sample whose pings gave no echo at all. */
#define SCAN_PLAN_RANGE_UNKNOWN (-1)

/** @brief One scan sample; positive angles look left of FRONT.
 *  The range is from the transducer face. A range of OBS_FAR_MM or more
 *  means nothing came back in range; SCAN_PLAN_RANGE_UNKNOWN means no
 *  echo pulse at all, which is never taken as free space.
 */
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
 *  @param[in] p_points Samples ordered from right to left: optionally a far
 *  right bearing, the profile, then optionally a far left bearing.
 *  @param count Number of samples.
 *  @param[out] p_plan Receives the packed plan.
 */
void scan_plan_build (scan_plan_point_t const * p_points, int32_t count,
                      scan_plan_bypass_t * p_plan);
/** @brief Preserve a confirmed front obstacle when later scan misses it.
 *  @param[in,out] p_points One stationary scan, ordered right to left.
 *  @param count Number of samples.
 *  @param confirmed_front_mm Earlier forward measurement or invalid marker.
 *  @param[out] p_plan Result. A missed centre echo cannot cancel detection.
 */
void scan_plan_build_confirmed (scan_plan_point_t * p_points, int32_t count,
                                int32_t confirmed_front_mm,
                                scan_plan_bypass_t * p_plan);

/** @brief Scan plan with independent outward-side corridor vetoes.
 *  Outer servo readings check both departure sides before any turn.
 *  The original front obstacle is retained when a later centre echo fails.
 */
void scan_plan_build_checked (scan_plan_point_t * p_points, int32_t count,
                              int32_t confirmed_front_mm,
                              bool b_left_clear, bool b_right_clear,
                              scan_plan_bypass_t * p_plan);

/** @brief SCAN_PLAN_CLEAR, SCAN_PLAN_ROUTE or SCAN_PLAN_NO_ROUTE. */
uint32_t scan_plan_result (scan_plan_bypass_t const * p_plan);
/** @brief True when the bypass goes round the left side. */
bool scan_plan_side_is_left (scan_plan_bypass_t const * p_plan);
/** @brief Index of the chosen bypass angle. */
uint32_t scan_plan_angle_index (scan_plan_bypass_t const * p_plan);
/** @brief Approximate object width in cm. */
int32_t scan_plan_width_cm (scan_plan_bypass_t const * p_plan);
/** @brief Distance from the axle midpoint to the nearest face in cm. */
int32_t scan_plan_near_cm (scan_plan_bypass_t const * p_plan);
/** @brief Lateral path offset in cm. */
int32_t scan_plan_lateral_cm (scan_plan_bypass_t const * p_plan);
/** @brief Length of the diagonal run in cm. */
int32_t scan_plan_diag_cm (scan_plan_bypass_t const * p_plan);
/** @brief Distance to pass the object by dead reckoning, in cm. */
int32_t scan_plan_fallback_cm (scan_plan_bypass_t const * p_plan);
/** @brief Verified room beyond the body margin on the outer side of the
 *  passing path, in cm; it bounds offset corrections while passing.
 */
int32_t scan_plan_slack_cm (scan_plan_bypass_t const * p_plan);

#endif /* SCAN_PLAN_H */

/*** end of file ***/

