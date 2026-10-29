/** @file scan_plan.h
 * @brief Turn a range profile into a bypass plan. Pure functions, no I/O.
 */
#ifndef SCAN_PLAN_H
#define SCAN_PLAN_H

#include <stdbool.h>
#include <stdint.h>

/** @brief One scan sample; positive angles look left of FRONT. */
typedef struct
{
    int16_t  angle_deg;
    uint16_t range_mm;
} scan_point_t;

typedef enum
{
    PLAN_CLEAR = 0, /**< Nothing in front any more. */
    PLAN_ROUTE,     /**< Bypass planned. */
    PLAN_NO_ROUTE   /**< No safe side. */
} plan_result_t;

/** @brief Packed plan (49 bits, centimetre units); one writer, then flagged. */
typedef struct
{
    uint32_t result : 2;
    uint32_t side_left : 1;
    uint32_t angle_idx : 2; /**< 0..2 selects 30, 45 or 60 degrees. */
    uint32_t width_cm : 8;
    uint32_t near_cm : 9;
    uint32_t lateral_cm : 9;
    uint32_t diag_cm : 9;
    uint32_t fallback_cm : 9;
} bypass_plan_t;

/** @brief Bypass heading change for a plan's angle index. */
float scan_plan_angle_deg (uint32_t angle_idx);
/** @brief Find the object ahead, its edges, the clearer side, angle and run. */
void scan_plan_build (scan_point_t const * p_pts, uint8_t count,
                      bypass_plan_t * p_plan);

#endif /* SCAN_PLAN_H */

/*** end of file ***/

