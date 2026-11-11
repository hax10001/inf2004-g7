/** @file scan_plan.c
 *
 * @brief Edge finding and bypass geometry from a servo range profile.
 *
 * Each sample becomes a lateral offset (left is positive) and a forward
 * depth. The object is the run of near samples around the front. Its outer
 * edges, padded by half a scan step, set the lateral offset that a path
 * needs: edge plus half the robot width plus a safety margin. If the object
 * reaches the end of the sweep on a side, that edge was never seen, so the
 * side needs at least OBS_LATERAL_UNKNOWN_MM. The smallest candidate angle
 * whose turn and run still clear the nearest face is used; a 90 degree turn
 * always fits, so only another object beside the path makes a side unusable.
 * Both extreme bearings are included. The clear corridor needing the
 * smaller offset wins; two blocked corridors produce NO_ROUTE.
 * Integer maths is used because the RP2040 has no floating point unit.
 */
#include "scan_plan.h"
#include "obs_config.h"
#include <stddef.h>

#if ((OBS_SCAN_STEP_DEG % 10) != 0)
#error "OBS_SCAN_STEP_DEG must be a multiple of 10 degrees."
#endif

#define TRIG_STEP_DEG 5
#define TRIG_RIGHT_ANGLE_DEG 90
#define TRIG_TABLE_SIZE 19U
#define MILLI 1000
#define HALF 2
#define ANGLE_CHOICES 4U
#define MM_PER_CM 10
#define ROUND_MM 5
#define WIDTH_LIMIT_CM 254
#define DIST_LIMIT_CM 510
#define SEED_NONE_DEG 32767

/* Packed plan layout: word A holds the first six fields, word B the rest. */
#define RESULT_SHIFT 0U
#define RESULT_MASK 0x3U
#define SIDE_SHIFT 2U
#define SIDE_MASK 0x1U
#define ANGLE_SHIFT 3U
#define ANGLE_MASK 0x3U
#define WIDTH_SHIFT 5U
#define WIDTH_MASK 0xFFU
#define NEAR_SHIFT 13U
#define NEAR_MASK 0x1FFU
#define LATERAL_SHIFT 22U
#define LATERAL_MASK 0x1FFU
#define DIAG_SHIFT 0U
#define DIAG_MASK 0x1FFU
#define FALLBACK_SHIFT 9U
#define FALLBACK_MASK 0x1FFU

/** @brief Sine times 1000 for 0 to 90 degrees in steps of 5 degrees. */
static int32_t const g_sine_milli[TRIG_TABLE_SIZE] = {
    0,   87,  174, 259, 342, 423, 500, 574, 643, 707,
    766, 819, 866, 906, 940, 966, 985, 996, 1000};

static int32_t const g_angle_choice_deg[ANGLE_CHOICES] = {30, 45, 60, 90};


typedef struct
{
    int32_t  need_mm;
    uint32_t angle_index;
    bool     b_ok;
} side_option_t;

typedef struct
{
    uint32_t result;
    bool     b_left;
    uint32_t angle_index;
    uint32_t width_cm;
    uint32_t near_cm;
    uint32_t lateral_cm;
    uint32_t diag_cm;
    uint32_t fallback_cm;
} plan_fields_t;

/** @brief Sine times 1000 of a multiple of 5 degrees from -90 to 90. */
static int32_t sine_milli (int32_t angle_deg);
/** @brief Cosine times 1000 of a multiple of 5 degrees from -90 to 90. */
static int32_t cosine_milli (int32_t angle_deg);
/** @brief Absolute value of a signed integer. */
static int32_t abs_int (int32_t value);
/** @brief True if a sample is close enough to belong to an object. */
static bool is_near (scan_plan_point_t const * p_point);
/** @brief Lateral offset of a sample in mm; left is positive. */
static int32_t point_x (scan_plan_point_t const * p_point);
/** @brief Forward depth of a sample in mm. */
static int32_t point_y (scan_plan_point_t const * p_point);
/** @brief Half a scan step of lateral uncertainty at the sample range. */
static int32_t edge_pad (scan_plan_point_t const * p_point);
/** @brief Outward lateral extent of an edge sample: the object may reach as
 *  far as the next, empty, ray at the edge sample's depth.
 *  @param dir 1 towards the left end of the samples, -1 towards the right.
 */
static int32_t edge_extent (scan_plan_point_t const * p_points, int32_t count,
                            int32_t index, int32_t dir);
/** @brief Convert millimetres to a clamped, rounded, non-negative cm. */
static uint32_t to_cm (int32_t length_mm, int32_t limit_cm);
/** @brief Find the run of near samples around the front. */
static bool find_cluster (scan_plan_point_t const * p_points, int32_t count,
                          int32_t * p_low, int32_t * p_high);
/** @brief True if another object sits in the corridor beside the edge. */
static bool corridor_blocked (scan_plan_point_t const * p_points, int32_t count,
                              int32_t low, int32_t high, int32_t outward_edge,
                              int32_t limit_mm, int32_t sign);
/** @brief Pick the smallest angle whose run clears the near face. */
static uint32_t pick_angle (int32_t near_depth, int32_t need_mm);
/** @brief Build a plan; prefer is 1 left, -1 right or 0 for the nearer edge.
 *  @param b_strict Never switch away from a non-zero preferred side.
 */
static void build_plan (scan_plan_point_t const * p_points, int32_t count,
                        int32_t prefer, bool b_strict,
                        scan_plan_bypass_t * p_plan);
/** @brief Work out the offset and angle needed on one side.
 *  @param b_edge_seen False if the object reaches the end of the sweep.
 */
static void evaluate_side (scan_plan_point_t const * p_points, int32_t count,
                           int32_t low, int32_t high, int32_t outward_edge,
                           bool b_edge_seen, int32_t near_depth, int32_t sign,
                           side_option_t * p_option);
/** @brief Choose the side and fill in the plan fields. */
static void choose_side (side_option_t const * p_left,
                         side_option_t const * p_right, int32_t width_mm,
                         int32_t prefer, bool b_strict,
                         plan_fields_t * p_fields);
/** @brief Measure the object and plan a path round it. */
static void plan_object (scan_plan_point_t const * p_points, int32_t count,
                         int32_t low, int32_t high, int32_t prefer,
                         bool b_strict, plan_fields_t * p_fields);
/** @brief Pack the plan fields into two words. */
static void pack_fields (plan_fields_t const * p_fields,
                         scan_plan_bypass_t *  p_plan);

int32_t scan_plan_angle_deg (uint32_t angle_index)
{
    int32_t angle = g_angle_choice_deg[0];

    if (angle_index < ANGLE_CHOICES)
    {
        angle = g_angle_choice_deg[angle_index];
    }

    return angle;
}

void scan_plan_build (scan_plan_point_t const * p_points, int32_t count,
                      scan_plan_bypass_t * p_plan)
{
    build_plan(p_points, count, OBS_PREFERRED_SIDE, false, p_plan);
}

static void build_plan (scan_plan_point_t const * p_points, int32_t count,
                        int32_t prefer, bool b_strict,
                        scan_plan_bypass_t * p_plan)
{
    plan_fields_t fields = {SCAN_PLAN_CLEAR, false, 0U, 0U, 0U, 0U, 0U, 0U};
    int32_t       low    = 0;
    int32_t       high   = 0;

    if (find_cluster(p_points, count, &low, &high))
    {
        plan_object(p_points, count, low, high, prefer, b_strict, &fields);
    }

    pack_fields(&fields, p_plan);
}

void scan_plan_build_confirmed (scan_plan_point_t * p_points, int32_t count,
                                int32_t confirmed_front_mm,
                                scan_plan_bypass_t * p_plan)
{
    scan_plan_build_side(p_points, count, confirmed_front_mm, 0, p_plan);
}

void scan_plan_build_side (scan_plan_point_t * p_points, int32_t count,
                           int32_t confirmed_front_mm, int32_t required_side,
                           scan_plan_bypass_t * p_plan)
{
    if (NULL == p_plan)
    {
        return;
    }

    if ((NULL == p_points) || (count <= 0) ||
        (count > OBS_SCAN_POINTS))
    {
        p_plan->word_a = SCAN_PLAN_NO_ROUTE;
        p_plan->word_b = 0U;
        return;
    }

    if ((confirmed_front_mm >= 20) &&
        (confirmed_front_mm <= OBS_STOP_MM))
    {
        for (int32_t index = 0; index < count; index++)
        {
            if ((0 == p_points[index].angle_deg) &&
                (p_points[index].range_mm > confirmed_front_mm))
            {
                p_points[index].range_mm = confirmed_front_mm;
            }
        }
    }

    if (0 == required_side)
    {
        build_plan(p_points, count, OBS_PREFERRED_SIDE, false, p_plan);
    }
    else
    {
        build_plan(p_points, count, (required_side > 0) ? 1 : -1, true,
                   p_plan);
    }
}

uint32_t scan_plan_result (scan_plan_bypass_t const * p_plan)
{
    return (p_plan->word_a >> RESULT_SHIFT) & RESULT_MASK;
}

bool scan_plan_side_is_left (scan_plan_bypass_t const * p_plan)
{
    return 0U != ((p_plan->word_a >> SIDE_SHIFT) & SIDE_MASK);
}

uint32_t scan_plan_angle_index (scan_plan_bypass_t const * p_plan)
{
    return (p_plan->word_a >> ANGLE_SHIFT) & ANGLE_MASK;
}

int32_t scan_plan_width_cm (scan_plan_bypass_t const * p_plan)
{
    /* The field is at most 8 bits wide, so the cast is exact. */
    return (int32_t)((p_plan->word_a >> WIDTH_SHIFT) & WIDTH_MASK);
}

int32_t scan_plan_near_cm (scan_plan_bypass_t const * p_plan)
{
    /* The field is at most 9 bits wide, so the cast is exact. */
    return (int32_t)((p_plan->word_a >> NEAR_SHIFT) & NEAR_MASK);
}

int32_t scan_plan_lateral_cm (scan_plan_bypass_t const * p_plan)
{
    /* The field is at most 9 bits wide, so the cast is exact. */
    return (int32_t)((p_plan->word_a >> LATERAL_SHIFT) & LATERAL_MASK);
}

int32_t scan_plan_diag_cm (scan_plan_bypass_t const * p_plan)
{
    /* The field is at most 9 bits wide, so the cast is exact. */
    return (int32_t)((p_plan->word_b >> DIAG_SHIFT) & DIAG_MASK);
}

int32_t scan_plan_fallback_cm (scan_plan_bypass_t const * p_plan)
{
    /* The field is at most 9 bits wide, so the cast is exact. */
    return (int32_t)((p_plan->word_b >> FALLBACK_SHIFT) & FALLBACK_MASK);
}

static int32_t sine_milli (int32_t angle_deg)
{
    int32_t const magnitude = abs_int(angle_deg);
    int32_t const value     = g_sine_milli[magnitude / TRIG_STEP_DEG];

    return (angle_deg < 0) ? -value : value;
}

static int32_t cosine_milli (int32_t angle_deg)
{
    int32_t const magnitude = abs_int(angle_deg);

    return g_sine_milli[(TRIG_RIGHT_ANGLE_DEG - magnitude) / TRIG_STEP_DEG];
}

static int32_t abs_int (int32_t value)
{
    return (value < 0) ? -value : value;
}

static bool is_near (scan_plan_point_t const * p_point)
{
    return p_point->range_mm < OBS_SCAN_RANGE_MM;
}

static int32_t point_x (scan_plan_point_t const * p_point)
{
    return (p_point->range_mm * sine_milli(p_point->angle_deg)) / MILLI;
}

static int32_t point_y (scan_plan_point_t const * p_point)
{
    return (p_point->range_mm * cosine_milli(p_point->angle_deg)) / MILLI;
}

static int32_t edge_extent (scan_plan_point_t const * p_points, int32_t count,
                            int32_t index, int32_t dir)
{
    int32_t const pad    = point_x(&p_points[index]) +
                           (dir * edge_pad(&p_points[index]));
    int32_t       extent = pad;
    int32_t const next   = index + dir;

    if ((next >= 0) && (next < count))
    {
        int32_t const angle = p_points[next].angle_deg;
        int32_t const cos_m = cosine_milli(angle);

        if (cos_m > 0)
        {
            int32_t const reach =
                (point_y(&p_points[index]) * sine_milli(angle)) / cos_m;

            if ((dir * reach) > (dir * extent))
            {
                extent = reach;
            }
        }
    }

    return extent;
}

static int32_t edge_pad (scan_plan_point_t const * p_point)
{
    return (p_point->range_mm * sine_milli(OBS_SCAN_STEP_DEG / HALF)) / MILLI;
}

static uint32_t to_cm (int32_t length_mm, int32_t limit_cm)
{
    int32_t centimetres = 0;

    if (length_mm > 0)
    {
        centimetres = (length_mm + ROUND_MM) / MM_PER_CM;
    }

    if (centimetres > limit_cm)
    {
        centimetres = limit_cm;
    }

    /* The value is clamped between zero and a positive limit. */
    return (uint32_t)centimetres;
}

static bool find_cluster (scan_plan_point_t const * p_points, int32_t count,
                          int32_t * p_low, int32_t * p_high)
{
    int32_t seed    = -1;
    int32_t best    = SEED_NONE_DEG;
    bool    b_found = false;

    for (int32_t index = 0; index < count; index++)
    {
        int32_t const offset = abs_int(p_points[index].angle_deg);

        if (is_near(&p_points[index]) && (offset < best))
        {
            best = offset;
            seed = index;
        }
    }

    if ((seed >= 0) && (best <= OBS_FRONT_WINDOW_DEG))
    {
        int32_t low  = seed;
        int32_t high = seed;

        while ((low > 0) && is_near(&p_points[low - 1]) &&
               (abs_int(p_points[low - 1].range_mm - p_points[low].range_mm) <
                OBS_EDGE_JUMP_MM))
        {
            low--;
        }

        while (((high + 1) < count) && is_near(&p_points[high + 1]) &&
               (abs_int(p_points[high + 1].range_mm - p_points[high].range_mm) <
                OBS_EDGE_JUMP_MM))
        {
            high++;
        }

        *p_low  = low;
        *p_high = high;
        b_found = true;
    }

    return b_found;
}

static bool corridor_blocked (scan_plan_point_t const * p_points, int32_t count,
                              int32_t low, int32_t high, int32_t outward_edge,
                              int32_t limit_mm, int32_t sign)
{
    bool b_blocked = false;

    for (int32_t index = 0; index < count; index++)
    {
        bool const b_outside = (index < low) || (index > high);

        if (b_outside && is_near(&p_points[index]))
        {
            int32_t const outward_x = sign * point_x(&p_points[index]);
            int32_t const depth_y   = point_y(&p_points[index]);

            if ((outward_x >= outward_edge) && (outward_x <= limit_mm) &&
                (depth_y <= OBS_LOOKAHEAD_MM))
            {
                b_blocked = true;
            }
        }
    }

    return b_blocked;
}

static uint32_t pick_angle (int32_t near_depth, int32_t need_mm)
{
    bool     b_fit       = false;
    /* The steepest angle is the fallback: turning square on adds no forward
     * travel, so it suits an object that is already close.
     */
    uint32_t angle_index = ANGLE_CHOICES - 1U;

    for (uint32_t choice = 0U; (choice < ANGLE_CHOICES) && (!b_fit); choice++)
    {
        int32_t const angle = g_angle_choice_deg[choice];
        /* Pivoting on one wheel also carries the robot forward. */
        int32_t const pivot =
            ((OBS_TRACK_WIDTH_MM / HALF) * sine_milli(angle)) / MILLI;
        int32_t const advance =
            (need_mm * cosine_milli(angle)) / sine_milli(angle);

        /* The body reaches past the sensor, which the range is from. */
        int32_t const overhang = OBS_ROBOT_FRONT_MM - OBS_SENSOR_TO_AXLE_MM;

        if ((advance + pivot + overhang + OBS_FRONT_CLEAR_MM) <= near_depth)
        {
            b_fit       = true;
            angle_index = choice;
        }
    }

    return angle_index;
}

static void evaluate_side (scan_plan_point_t const * p_points, int32_t count,
                           int32_t low, int32_t high, int32_t outward_edge,
                           bool b_edge_seen, int32_t near_depth, int32_t sign,
                           side_option_t * p_option)
{
    int32_t const half_mm = (OBS_ROBOT_WIDTH_MM / HALF) + OBS_SAFETY_MARGIN_MM;
    int32_t       need_mm = outward_edge + half_mm;

    if (need_mm < half_mm)
    {
        need_mm = half_mm;
    }

    if ((!b_edge_seen) && (need_mm < OBS_LATERAL_UNKNOWN_MM))
    {
        need_mm = OBS_LATERAL_UNKNOWN_MM;
    }

    /* The floor is only for an edge the scan may have cut short: a flat
     * face stops echoing at steep angles, so an echo boundary out there may
     * not be the real edge. A boundary near the front is a real edge.
     */
    int32_t const edge_angle =
        abs_int(p_points[(sign > 0) ? high : low].angle_deg);

    if ((edge_angle >= OBS_MIN_LATERAL_FROM_DEG) &&
        (need_mm < OBS_MIN_LATERAL_MM))
    {
        need_mm = OBS_MIN_LATERAL_MM;
    }

    p_option->need_mm = need_mm;
    p_option->b_ok = !corridor_blocked(p_points, count, low, high, outward_edge,
                                       need_mm + half_mm, sign);
    p_option->angle_index      = pick_angle(near_depth, need_mm);
}

static void choose_side (side_option_t const * p_left,
                         side_option_t const * p_right, int32_t width_mm,
                         int32_t prefer, bool b_strict,
                         plan_fields_t * p_fields)
{
    bool b_pick_left = true;

    if (0 == prefer)
    {
        /* The side needing the smaller move; ties go left. */
        if (p_left->b_ok || p_right->b_ok)
        {
            b_pick_left = p_left->b_ok &&
                          ((!p_right->b_ok) ||
                           (p_left->need_mm <= p_right->need_mm));
        }
        else
        {
            p_fields->result = SCAN_PLAN_NO_ROUTE;
            return;
        }
    }
    else
    {
        bool const                  b_pref_left = prefer > 0;
        side_option_t const * const p_pref  = b_pref_left ? p_left : p_right;
        side_option_t const * const p_other = b_pref_left ? p_right : p_left;

        /* A rescan must keep its side; a blocked fixed side stops. */
        b_pick_left = (p_pref->b_ok || b_strict || (!p_other->b_ok))
                          ? b_pref_left
                          : (!b_pref_left);
    }

    side_option_t const * const p_pick = b_pick_left ? p_left : p_right;
    if (!p_pick->b_ok)
    {
        p_fields->result = SCAN_PLAN_NO_ROUTE;
        return;
    }
    int32_t const  need_mm = p_pick->need_mm;
    uint32_t const index   = p_pick->angle_index;
    int32_t const  angle   = g_angle_choice_deg[index];
    int32_t const  diag_mm = (need_mm * MILLI) / sine_milli(angle);

    p_fields->result      = SCAN_PLAN_ROUTE;
    p_fields->b_left      = b_pick_left;
    p_fields->angle_index = index;
    p_fields->lateral_cm  = to_cm(need_mm, DIST_LIMIT_CM);
    p_fields->diag_cm     = to_cm(diag_mm, DIST_LIMIT_CM);
    p_fields->fallback_cm = to_cm(width_mm + OBS_ROBOT_FRONT_MM +
                                      OBS_ROBOT_REAR_MM +
                                      (HALF * OBS_SAFETY_MARGIN_MM),
                                  DIST_LIMIT_CM);
}

static void plan_object (scan_plan_point_t const * p_points, int32_t count,
                         int32_t low, int32_t high, int32_t prefer,
                         bool b_strict, plan_fields_t * p_fields)
{
    int32_t near_depth = point_y(&p_points[low]);

    for (int32_t index = low; index <= high; index++)
    {
        int32_t const depth = point_y(&p_points[index]);

        if (depth < near_depth)
        {
            near_depth = depth;
        }
    }

    int32_t const left_x  = edge_extent(p_points, count, high, 1);
    int32_t const right_x = edge_extent(p_points, count, low, -1);
    int32_t const width   = left_x - right_x;
    /* Samples are ordered right to left, so the ends are the sweep limits. */
    bool const    b_left_seen  = high < (count - 1);
    bool const    b_right_seen = low > 0;
    side_option_t left         = {0, 0U, false};
    side_option_t right        = {0, 0U, false};

    evaluate_side(p_points, count, low, high, left_x, b_left_seen, near_depth,
                  1, &left);
    evaluate_side(p_points, count, low, high, -right_x, b_right_seen,
                  near_depth, -1, &right);

    if ((!b_left_seen) && (!b_right_seen) && left.b_ok && right.b_ok &&
        (left.need_mm == right.need_mm))
    {
        /* Neither edge was seen: go where the outermost sample is farther,
         * as that end of the object is more likely to be turning away.
         */
        if (p_points[0].range_mm > p_points[count - 1].range_mm)
        {
            left.need_mm++;
        }
    }
    p_fields->width_cm = to_cm(width, WIDTH_LIMIT_CM);
    p_fields->near_cm  = to_cm(near_depth, DIST_LIMIT_CM);
    choose_side(&left, &right, width, prefer, b_strict, p_fields);
}

static void pack_fields (plan_fields_t const * p_fields,
                         scan_plan_bypass_t *  p_plan)
{
    uint32_t const side_bit = p_fields->b_left ? 1U : 0U;

    p_plan->word_a = ((p_fields->result & RESULT_MASK) << RESULT_SHIFT) |
                     ((side_bit & SIDE_MASK) << SIDE_SHIFT) |
                     ((p_fields->angle_index & ANGLE_MASK) << ANGLE_SHIFT) |
                     ((p_fields->width_cm & WIDTH_MASK) << WIDTH_SHIFT) |
                     ((p_fields->near_cm & NEAR_MASK) << NEAR_SHIFT) |
                     ((p_fields->lateral_cm & LATERAL_MASK) << LATERAL_SHIFT);
    p_plan->word_b =
        ((p_fields->diag_cm & DIAG_MASK) << DIAG_SHIFT) |
        ((p_fields->fallback_cm & FALLBACK_MASK) << FALLBACK_SHIFT);
}

/*** end of file ***/

