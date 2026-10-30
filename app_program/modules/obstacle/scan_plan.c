/** @file scan_plan.c
 *
 * @brief Pure geometry from a servo scan to a bypass plan; no I/O here.
 *
 * The scan is a set of (angle, range) samples from right to left. An object
 * is a run of samples closer than OBS_SCAN_RANGE_MM. The plan picks the side
 * with the wider clear gap, converts the object edges to a lateral offset and
 * a diagonal run, and reports a conservative fallback distance.
 */
#include "scan_plan.h"
#include "obs_config.h"
#include <stdbool.h>

#define MM_PER_CM 10
#define PERMILLE 1000
/* cos(45 deg) x 1000, used to project a diagonal run onto the forward axis. */
#define COS45_PERMILLE 707

/** @brief One contiguous closer-than-range run in the scan. */
typedef struct
{
    int32_t first;
    int32_t last;
    int32_t near_mm;
} scan_object_t;

/** @brief Find the single closest object run in the scan. */
static bool find_object (scan_plan_point_t const * p_points, int32_t count,
                         scan_object_t * p_object);
/** @brief Measure the clear gap on one side of the object. */
static int32_t side_gap_mm (scan_plan_point_t const * p_points, int32_t count,
                            scan_object_t const * p_object, bool b_left);
/** @brief Build the plan for the chosen side. */
static void build_plan (scan_plan_bypass_t * p_plan, bool b_left,
                        scan_object_t const *     p_object,
                        scan_plan_point_t const * p_points, int32_t count);

void scan_plan_build (scan_plan_point_t const * p_points, int32_t count,
                      scan_plan_bypass_t * p_plan)
{
    scan_object_t object  = {0, 0, 0};
    bool const    b_found = find_object(p_points, count, &object);

    p_plan->word_a = 0U;
    p_plan->word_b = 0U;

    if (!b_found)
    {
        /* No object inside the scan range: the way ahead is clear. */
        p_plan->word_a |= (uint32_t)SCAN_PLAN_CLEAR;
        return;
    }

    int32_t const left_gap  = side_gap_mm(p_points, count, &object, true);
    int32_t const right_gap = side_gap_mm(p_points, count, &object, false);
    /* A gap must fit the robot width plus a safety margin on both sides. */
    int32_t const need_mm = OBS_ROBOT_WIDTH_MM + (2 * OBS_SAFETY_MARGIN_MM);

    if ((left_gap < need_mm) && (right_gap < need_mm))
    {
        p_plan->word_a |= (uint32_t)SCAN_PLAN_NO_ROUTE;
        return;
    }

    /* Tie-break: prefer left when both sides fit. */
    bool const b_left = (left_gap >= right_gap);

    build_plan(p_plan, b_left, &object, p_points, count);
}

static bool find_object (scan_plan_point_t const * p_points, int32_t count,
                         scan_object_t * p_object)
{
    bool b_found = false;

    for (int32_t index = 0; index < count; index++)
    {
        if (p_points[index].range_mm < OBS_SCAN_RANGE_MM)
        {
            if (!b_found)
            {
                p_object->first   = index;
                p_object->near_mm = p_points[index].range_mm;
                b_found           = true;
            }
            else
            {
                p_object->last = index;
            }

            if (p_points[index].range_mm < p_object->near_mm)
            {
                p_object->near_mm = p_points[index].range_mm;
            }
        }
        else if (b_found)
        {
            /* The first run is the object; later runs are ignored. */
            break;
        }
        else
        {
            /* Still before the object. */
        }
    }

    return b_found;
}

static int32_t side_gap_mm (scan_plan_point_t const * p_points, int32_t count,
                            scan_object_t const * p_object, bool b_left)
{
    int32_t const edge = b_left ? p_object->last : p_object->first;
    int32_t const step = b_left ? 1 : -1;
    int32_t       gap  = 0;

    for (int32_t index = edge + step; (index >= 0) && (index < count);
         index += step)
    {
        if (p_points[index].range_mm < OBS_SCAN_RANGE_MM)
        {
            /* Another object starts here; the gap ends before it. */
            break;
        }

        gap += (OBS_SCAN_STEP_DEG * p_points[index].range_mm) / 100;
    }

    return gap;
}

static void build_plan (scan_plan_bypass_t * p_plan, bool b_left,
                        scan_object_t const *     p_object,
                        scan_plan_point_t const * p_points, int32_t count)
{
    int32_t const edge = b_left ? p_object->last : p_object->first;
    /* Bearing of the object edge from FRONT, in degrees. */
    int32_t const edge_deg = p_points[edge].angle_deg;
    /* Turn further out on the chosen side to clear the object. */
    int32_t const lateral_mm =
        (edge_deg * p_object->near_mm) / 100 + OBS_SAFETY_MARGIN_MM;
    /* Diagonal run long enough to pass the object's near face. */
    int32_t const diag_mm =
        (p_object->near_mm * COS45_PERMILLE) / PERMILLE + OBS_FRONT_CLEAR_MM;
    /* Conservative fallback if the side look never sees the far edge. */
    int32_t const fallback_mm =
        (p_object->near_mm * COS45_PERMILLE) / PERMILLE + OBS_ROBOT_LENGTH_MM +
        OBS_PASS_MARGIN_MM;

    uint32_t word = (uint32_t)SCAN_PLAN_ROUTE;

    word |= b_left ? (1U << 2) : 0U;
    /* Angle index: 0 is straight ahead, 1 is 10 deg left, and so on. */
    uint32_t const angle_index =
        (uint32_t)((edge_deg + OBS_SCAN_RIGHT_DEG) / OBS_SCAN_STEP_DEG);
    word |= (angle_index & 0xFU) << 3;
    /* Width in cm, clamped to 8 bits. */
    uint32_t const width_cm =
        (uint32_t)((edge_deg * p_object->near_mm) / 100 / MM_PER_CM);
    word |= (width_cm & 0xFFU) << 7;
    /* Lateral in cm, clamped to 8 bits. */
    word |= (((uint32_t)lateral_mm / MM_PER_CM) & 0xFFU) << 15;
    /* Diagonal in cm, clamped to 8 bits. */
    word |= (((uint32_t)diag_mm / MM_PER_CM) & 0xFFU) << 23;

    p_plan->word_a = word;
    /* Fallback in cm, clamped to 16 bits. */
    p_plan->word_b = ((uint32_t)fallback_mm / MM_PER_CM) & 0xFFFFU;
}

int32_t scan_plan_angle_deg (uint32_t angle_index)
{
    return (int32_t)angle_index * OBS_SCAN_STEP_DEG - OBS_SCAN_RIGHT_DEG;
}

uint32_t scan_plan_result (scan_plan_bypass_t const * p_plan)
{
    return p_plan->word_a & 0x3U;
}

bool scan_plan_side_is_left (scan_plan_bypass_t const * p_plan)
{
    return (0U != (p_plan->word_a & (1U << 2)));
}

uint32_t scan_plan_angle_index (scan_plan_bypass_t const * p_plan)
{
    return (p_plan->word_a >> 3) & 0xFU;
}

int32_t scan_plan_width_cm (scan_plan_bypass_t const * p_plan)
{
    return (int32_t)((p_plan->word_a >> 7) & 0xFFU);
}

int32_t scan_plan_near_cm (scan_plan_bypass_t const * p_plan)
{
    /* Near distance is not stored; the caller uses the diagonal instead. */
    (void)p_plan;
    return 0;
}

int32_t scan_plan_lateral_cm (scan_plan_bypass_t const * p_plan)
{
    return (int32_t)((p_plan->word_a >> 15) & 0xFFU);
}

int32_t scan_plan_diag_cm (scan_plan_bypass_t const * p_plan)
{
    return (int32_t)((p_plan->word_a >> 23) & 0xFFU);
}

int32_t scan_plan_fallback_cm (scan_plan_bypass_t const * p_plan)
{
    return (int32_t)(p_plan->word_b & 0xFFFFU);
}

/*** end of file ***/

