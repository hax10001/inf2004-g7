/** @file scan_plan.c
 *
 * @brief Edge finding and bypass geometry from a servo range profile.
 *
 * Each sample becomes a lateral offset (left is positive) and a forward
 * depth. The object is the run of near samples around the front. Its outer
 * edges, padded by half a scan step, set the lateral offset that a path
 * needs: edge plus half the robot width plus a safety margin. The smallest
 * candidate angle whose diagonal still clears the nearest face is used.
 * Integer maths is used because the RP2040 has no floating point unit.
 */
#include "scan_plan.h"
#include "obs_config.h"
#include "obs_geom.h"
#include <stddef.h>

#if ((OBS_SCAN_STEP_DEG % 10) != 0)
#error "OBS_SCAN_STEP_DEG must be a multiple of 10 degrees."
#endif

#define TRIG_STEP_DEG 5
#define TRIG_RIGHT_ANGLE_DEG 90
#define TRIG_TABLE_SIZE 19U
#define MILLI 1000
#define HALF 2
#define ANGLE_CHOICES 3U
#define MM_PER_CM 10
#define ROUND_MM 5
#define WIDTH_LIMIT_CM 254
#define DIST_LIMIT_CM 510
#define SEED_NONE_DEG 32767
#define POINT_ANGLE_MASK 0xFU
#define POINT_RANGE_SHIFT 4U
#define POINT_RANGE_MASK 0x1FFU
#define POINT_INVALID_CM 511U


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
#define LEFT_CLEAR_SHIFT 18U
#define RIGHT_CLEAR_SHIFT 24U
#define CLEAR_MASK 0x3FU
#define NEAR_DIR_SHIFT 30U
#define NEAR_DIR_MASK 0x3U

/** @brief Sine times 1000 for 0 to 90 degrees in steps of 5 degrees. */
static int32_t const g_sine_milli[TRIG_TABLE_SIZE] = {
    0,   87,  174, 259, 342, 423, 500, 574, 643, 707,
    766, 819, 866, 906, 940, 966, 985, 996, 1000};

static int32_t const g_angle_choice_deg[ANGLE_CHOICES] = {30, 45, 60};

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
    uint32_t left_clear_cm;
    uint32_t right_clear_cm;
    uint32_t near_direction;
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
/** @brief Convert millimetres to a clamped, rounded, non-negative cm. */
static uint32_t to_cm (int32_t length_mm, int32_t limit_cm);
/** @brief Find the run of near samples around the front. */
static bool find_cluster (scan_plan_point_t const * p_points, int32_t count,
                          int32_t * p_low, int32_t * p_high);
/** @brief True if another object sits in the corridor beside the edge. */
static bool corridor_blocked (scan_plan_point_t const * p_points, int32_t count,
                              int32_t low, int32_t high, int32_t outward_edge,
                              int32_t limit_mm, int32_t sign);
/** @brief Pick the smallest angle whose diagonal clears the near face. */
static void pick_angle (int32_t near_depth, side_option_t * p_option);
/** @brief Work out the offset and angle needed on one side. */
static void evaluate_side (scan_plan_point_t const * p_points, int32_t count,
                           int32_t low, int32_t high, int32_t outward_edge,
                           int32_t near_depth, int32_t sign,
                           side_option_t * p_option);
/** @brief Choose the better side and fill in the plan fields. */
static void choose_side (side_option_t const * p_left,
                         side_option_t const * p_right, int32_t width_mm,
                         plan_fields_t * p_fields);
/** @brief Measure the object and plan a path round it. */
static void plan_object (scan_plan_point_t const * p_points, int32_t count,
                         int32_t low, int32_t high, plan_fields_t * p_fields);
/** @brief Pack the plan fields into two words. */
static void pack_fields (plan_fields_t const * p_fields,
                         scan_plan_bypass_t *  p_plan);

void scan_plan_point_set (scan_plan_point_t * p_point, int32_t angle_deg,
                          int32_t range_mm)
{
    if (NULL == p_point)
    {
        return;
    }

    uint32_t index = 0U;
    uint32_t centimetres = POINT_INVALID_CM;

    bool const b_angle_valid =
        (angle_deg >= -OBS_SCAN_LIMIT_DEG) &&
        (angle_deg <= OBS_SCAN_LIMIT_DEG) &&
        (0 == ((angle_deg + OBS_SCAN_LIMIT_DEG) % OBS_SCAN_STEP_DEG));

    if (b_angle_valid)
    {
        index = (uint32_t)((angle_deg + OBS_SCAN_LIMIT_DEG) /
                           OBS_SCAN_STEP_DEG);
    }

    if (b_angle_valid && (range_mm >= 20) && (range_mm <= OBS_FAR_MM))
    {
        /* Round down, never exaggerating available free space. */
        centimetres = (uint32_t)(range_mm / 10);
    }

    p_point->packed = (uint16_t)((index & POINT_ANGLE_MASK) |
                ((centimetres & POINT_RANGE_MASK) << POINT_RANGE_SHIFT));
}

int32_t scan_plan_point_angle_deg (scan_plan_point_t const * p_point)
{
    return -OBS_SCAN_LIMIT_DEG +
           ((int32_t)(p_point->packed & POINT_ANGLE_MASK) *
            OBS_SCAN_STEP_DEG);
}

int32_t scan_plan_point_range_mm (scan_plan_point_t const * p_point)
{
    uint32_t const centimetres =
        (p_point->packed >> POINT_RANGE_SHIFT) & POINT_RANGE_MASK;

    return (centimetres == POINT_INVALID_CM) ? -1 :
           ((int32_t)centimetres * 10);
}

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
    if (NULL == p_plan)
    {
        return;
    }

    plan_fields_t fields = {SCAN_PLAN_CLEAR, false, 0U, 0U, 0U, 0U,
                            0U, 0U, 63U, 63U, 1U};
    int32_t       low    = 0;
    int32_t       high   = 0;

    /* An incomplete scan must never be interpreted as clear road. */
    if ((NULL == p_points) || (count != OBS_SCAN_POINTS) ||
        (!obs_geom_is_valid()))
    {
        fields.result = SCAN_PLAN_NO_ROUTE;
    }
    else
    {
        bool all_valid = true;

        for (int32_t index = 0; index < count; index++)
        {
            if ((scan_plan_point_range_mm(&p_points[index]) < 20) ||
                (scan_plan_point_range_mm(&p_points[index]) > OBS_FAR_MM) ||
                (scan_plan_point_angle_deg(&p_points[index]) !=
                 (-OBS_SCAN_LIMIT_DEG + (index * OBS_SCAN_STEP_DEG))))
            {
                all_valid = false;
            }
        }

        if (!all_valid)
        {
            fields.result = SCAN_PLAN_NO_ROUTE;
        }
        else if (find_cluster(p_points, count, &low, &high))
        {
            plan_object(p_points, count, low, high, &fields);
        }
    }

    pack_fields(&fields, p_plan);
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

int32_t scan_plan_left_clear_cm (scan_plan_bypass_t const * p_plan)
{
    return (int32_t)((p_plan->word_b >> LEFT_CLEAR_SHIFT) & CLEAR_MASK);
}

int32_t scan_plan_right_clear_cm (scan_plan_bypass_t const * p_plan)
{
    return (int32_t)((p_plan->word_b >> RIGHT_CLEAR_SHIFT) & CLEAR_MASK);
}

int32_t scan_plan_near_direction (scan_plan_bypass_t const * p_plan)
{
    return (int32_t)((p_plan->word_b >> NEAR_DIR_SHIFT) & NEAR_DIR_MASK) - 1;
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
    return scan_plan_point_range_mm(p_point) < OBS_SCAN_RANGE_MM;
}

static int32_t point_x (scan_plan_point_t const * p_point)
{
    int32_t const range = scan_plan_point_range_mm(p_point);
    int32_t const angle = scan_plan_point_angle_deg(p_point);

    return (range * sine_milli(angle)) / MILLI;
}

static int32_t point_y (scan_plan_point_t const * p_point)
{
    int32_t const range = scan_plan_point_range_mm(p_point);
    int32_t const angle = scan_plan_point_angle_deg(p_point);

    return (range * cosine_milli(angle)) / MILLI;
}

static int32_t edge_pad (scan_plan_point_t const * p_point)
{
    int32_t const range = scan_plan_point_range_mm(p_point);

    int32_t const angle = (OBS_SCAN_STEP_DEG / HALF) +
                          OBS_ANGLE_UNCERTAINTY_DEG;
    /* Round angle uncertainty up to a supported five-degree table entry. */
    int32_t const rounded = ((angle + 4) / 5) * 5;

    return (range * sine_milli(rounded) + 999) / MILLI;
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
        int32_t const offset =
            abs_int(scan_plan_point_angle_deg(&p_points[index]));

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
               (abs_int(scan_plan_point_range_mm(&p_points[low - 1]) -
                        scan_plan_point_range_mm(&p_points[low])) <
                OBS_EDGE_JUMP_MM))
        {
            low--;
        }

        while (((high + 1) < count) && is_near(&p_points[high + 1]) &&
               (abs_int(scan_plan_point_range_mm(&p_points[high + 1]) -
                        scan_plan_point_range_mm(&p_points[high])) <
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

        if (b_outside)
        {
            int32_t const outward_x = sign * point_x(&p_points[index]);
            int32_t const depth_y   = point_y(&p_points[index]);
            int32_t const pad = edge_pad(&p_points[index]) +
                                OBS_POSITION_UNCERTAINTY_MM;

            if (((outward_x + pad) >= outward_edge) &&
                ((outward_x - pad) <= limit_mm) &&
                ((depth_y - pad) <= OBS_LOOKAHEAD_MM))
            {
                b_blocked = true;
            }
        }
    }

    return b_blocked;
}

static void pick_angle (int32_t near_depth, side_option_t * p_option)
{
    bool b_fit = false;

    for (uint32_t choice = 0U; (choice < ANGLE_CHOICES) && (!b_fit); choice++)
    {
        int32_t const angle = g_angle_choice_deg[choice];
        int32_t const advance =
            (p_option->need_mm * cosine_milli(angle)) / sine_milli(angle);
        int32_t const turns =
            (OBS_TRACK_WIDTH_MM * sine_milli(angle) + 999) / MILLI;
        int32_t const footprint = obs_geom_radius_mm() +
                                  obs_geom_padding_mm() -
                                  OBS_SENSOR_FORWARD_MM;

        if ((advance + turns + footprint + OBS_FRONT_CLEAR_MM) <= near_depth)
        {
            b_fit                 = true;
            p_option->angle_index = choice;
        }
    }

    p_option->b_ok = b_fit;
}

static void evaluate_side (scan_plan_point_t const * p_points, int32_t count,
                           int32_t low, int32_t high, int32_t outward_edge,
                           int32_t near_depth, int32_t sign,
                           side_option_t * p_option)
{
    int32_t const half_mm = obs_geom_radius_mm() + obs_geom_padding_mm();
    int32_t       need_mm = outward_edge + half_mm +
                           (sign * OBS_SENSOR_LEFT_MM);

    if (need_mm < (half_mm + (sign * OBS_SENSOR_LEFT_MM)))
    {
        need_mm = half_mm + (sign * OBS_SENSOR_LEFT_MM);
    }

    p_option->need_mm     = need_mm;
    p_option->angle_index = 0U;
    p_option->b_ok = !corridor_blocked(p_points, count, low, high, outward_edge,
                                       need_mm + half_mm + OBS_TRACK_WIDTH_MM +
                                           abs_int(OBS_SENSOR_LEFT_MM), sign);

    if (p_option->b_ok)
    {
        pick_angle(near_depth, p_option);
    }
}

static void choose_side (side_option_t const * p_left,
                         side_option_t const * p_right, int32_t width_mm,
                         plan_fields_t * p_fields)
{
    bool const b_pick_left =
        p_left->b_ok &&
        ((!p_right->b_ok) || (p_left->need_mm <= p_right->need_mm));
    side_option_t const * const p_pick = b_pick_left ? p_left : p_right;

    if (p_left->b_ok || p_right->b_ok)
    {
        int32_t const angle   = g_angle_choice_deg[p_pick->angle_index];
        int32_t const diag_mm = (p_pick->need_mm * MILLI) / sine_milli(angle);

        p_fields->result      = SCAN_PLAN_ROUTE;
        p_fields->b_left      = b_pick_left;
        p_fields->angle_index = p_pick->angle_index;
        p_fields->lateral_cm  = to_cm(p_pick->need_mm, DIST_LIMIT_CM);
        p_fields->diag_cm     = to_cm(diag_mm, DIST_LIMIT_CM);
        p_fields->fallback_cm = to_cm(width_mm + OBS_ROBOT_LENGTH_MM +
                                          (HALF * OBS_SAFETY_MARGIN_MM),
                                      DIST_LIMIT_CM);
    }
    else
    {
        p_fields->result = SCAN_PLAN_NO_ROUTE;
    }
}

static void plan_object (scan_plan_point_t const * p_points, int32_t count,
                         int32_t low, int32_t high, plan_fields_t * p_fields)
{
    int32_t near_depth = point_y(&p_points[low]);
    int32_t near_angle = scan_plan_point_angle_deg(&p_points[low]);
    int32_t left_clear = OBS_SCAN_RANGE_MM;
    int32_t right_clear = OBS_SCAN_RANGE_MM;

    /* Report radial return depths, not verified navigable lane widths. */
    for (int32_t index = 0; index < count; index++)
    {
        int32_t const angle = scan_plan_point_angle_deg(&p_points[index]);
        int32_t const range = scan_plan_point_range_mm(&p_points[index]);

        if ((angle >= 40) && (range < left_clear))
        {
            left_clear = range;
        }
        else if ((angle <= -40) && (range < right_clear))
        {
            right_clear = range;
        }
    }

    p_fields->left_clear_cm = to_cm(left_clear, 62);
    p_fields->right_clear_cm = to_cm(right_clear, 62);

    for (int32_t index = low; index <= high; index++)
    {
        int32_t const depth = point_y(&p_points[index]);

        if (depth < near_depth)
        {
            near_depth = depth;
            near_angle = scan_plan_point_angle_deg(&p_points[index]);
        }
    }

    p_fields->near_direction = (near_angle < -10) ? 0U :
                               ((near_angle > 10) ? 2U : 1U);

    int32_t const left_x = point_x(&p_points[high]) + edge_pad(&p_points[high]);
    int32_t const right_x = point_x(&p_points[low]) - edge_pad(&p_points[low]);
    int32_t const width   = left_x - right_x;
    side_option_t left    = {0, 0U, false};
    side_option_t right   = {0, 0U, false};

    evaluate_side(p_points, count, low, high, left_x, near_depth, 1, &left);
    evaluate_side(p_points, count, low, high, -right_x, near_depth, -1, &right);
    p_fields->width_cm = to_cm(width, WIDTH_LIMIT_CM);
    p_fields->near_cm  = to_cm(near_depth, DIST_LIMIT_CM);
    choose_side(&left, &right, width, p_fields);
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
        ((p_fields->fallback_cm & FALLBACK_MASK) << FALLBACK_SHIFT) |
        ((p_fields->left_clear_cm & CLEAR_MASK) << LEFT_CLEAR_SHIFT) |
        ((p_fields->right_clear_cm & CLEAR_MASK) << RIGHT_CLEAR_SHIFT) |
        ((p_fields->near_direction & NEAR_DIR_MASK) << NEAR_DIR_SHIFT);
}

/*** end of file ***/

