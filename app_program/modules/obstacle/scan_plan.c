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
 * smaller offset wins; exactly equal offsets go to the side of least
 * resistance (least_resistance()). The comparison uses the measured needs:
 * the OBS_MIN_LATERAL_MM floor only sets how far the chosen side drives.
 * Two blocked corridors produce NO_ROUTE.
 * Integer maths is used because the RP2040 has no floating point unit.
 */
#include "scan_plan.h"
#include "obs_config.h"
#include <stddef.h>

#if ((OBS_SCAN_DIRECTION < -1) || (OBS_SCAN_DIRECTION > 1))
#error "OBS_SCAN_DIRECTION must be 0 (both ways), 1 or -1."
#endif

#if ((OBS_SCAN_STEP_DEG % 10) != 0)
#error "OBS_SCAN_STEP_DEG must be a multiple of 10 degrees."
#endif

#if ((OBS_SCAN_FINE_STEP_DEG <= 0) || ((OBS_SCAN_FINE_STEP_DEG % 5) != 0) || \
     ((OBS_SCAN_STEP_DEG % OBS_SCAN_FINE_STEP_DEG) != 0))
#error "OBS_SCAN_FINE_STEP_DEG must be a multiple of 5 dividing the step."
#endif

#if ((OBS_SCAN_FINE_DEG < 0) || \
     ((OBS_SCAN_FINE_DEG % OBS_SCAN_STEP_DEG) != 0) || \
     (OBS_SCAN_FINE_DEG > OBS_SCAN_LEFT_DEG) || \
     (OBS_SCAN_FINE_DEG > OBS_SCAN_RIGHT_DEG))
#error "OBS_SCAN_FINE_DEG must be a step multiple within the sweep."
#endif

#if ((OBS_BEAM_HALF_DEG < 0) || (OBS_BEAM_HALF_DEG > 30) || \
     ((OBS_BEAM_HALF_DEG % 5) != 0))
#error "OBS_BEAM_HALF_DEG must be a multiple of 5 from 0 to 30."
#endif

#if ((OBS_CORNER_MAX_DEG < OBS_SCAN_FINE_DEG) || \
     ((OBS_CORNER_MAX_DEG % OBS_SCAN_STEP_DEG) != 0) || \
     (OBS_CORNER_MAX_DEG > OBS_SCAN_LEFT_DEG) || \
     (OBS_CORNER_MAX_DEG > OBS_SCAN_RIGHT_DEG))
#error "OBS_CORNER_MAX_DEG must be a step multiple from FINE_DEG to the sweep."
#endif

#if ((OBS_CORNER_BEAM_DEG < 0) || \
     (OBS_CORNER_BEAM_DEG >= OBS_SCAN_FINE_DEG) || \
     ((OBS_CORNER_BEAM_DEG % 5) != 0))
#error "OBS_CORNER_BEAM_DEG must be a multiple of 5 below OBS_SCAN_FINE_DEG."
#endif

#if ((OBS_CORNER_DEPTH_MM <= 0) || (OBS_CORNER_DEPTH_MM > OBS_SCAN_RANGE_MM))
#error "OBS_CORNER_DEPTH_MM must be 1..OBS_SCAN_RANGE_MM."
#endif

#if ((OBS_SIDE_CORNER_ECHO != 0) && (OBS_SIDE_CORNER_ECHO != 1))
#error "OBS_SIDE_CORNER_ECHO must be 0 or 1."
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
/* Largest value of the 12-bit need fields in scan_plan_sides_t. */
#define NEED_LIMIT_MM 4095
/* Range of the 12-bit signed edge field in scan_plan_side_info_t. */
#define EDGE_MIN_MM (-2048)
#define EDGE_MAX_MM 2047

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
    uint32_t reason;  /**< SCAN_PLAN_NEED_*. */
    int32_t  blocker; /**< Nearest corridor blocker index, or -1. */
    /** The need before the OBS_MIN_LATERAL_MM floor, used to compare the
     *  sides: the floor is a minimum drive, not a measurement, and one
     *  sample crossing OBS_MIN_LATERAL_FROM_DEG must not decide the side. */
    int32_t  compare_mm;
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
/** @brief Half the gap to the next sample, as lateral uncertainty at the
 *  sample range.
 *  @param gap_deg Bearing gap to that sample, a multiple of 5 degrees.
 */
static int32_t edge_pad (scan_plan_point_t const * p_point, int32_t gap_deg);
/** @brief Outward lateral extent of an edge sample: the object may reach as
 *  far as the next, empty, ray at the edge sample's depth.
 *  @param dir 1 towards the left end of the samples, -1 towards the right.
 */
static int32_t edge_extent (scan_plan_point_t const * p_points, int32_t count,
                            int32_t index, int32_t dir);
/** @brief Convert millimetres to a clamped, rounded, non-negative cm. */
static uint32_t to_cm (int32_t length_mm, int32_t limit_cm);
/** @brief Lateral offset, left positive, of a bearing at a forward depth.
 *  @param angle_x2 Twice the bearing in degrees, a multiple of 5 (so
 *  2.5 degree steps), clamped to +-80 degrees.
 */
static int32_t lateral_mm (int32_t depth_mm, int32_t angle_x2);
/** @brief Best estimate of the object's width for reporting: each edge
 *  midway to the next empty bearing, moved in by OBS_BEAM_HALF_DEG, at the
 *  face depth. Never more than the conservative planning width.
 */
static int32_t measured_width (scan_plan_point_t const * p_points,
                               int32_t count, int32_t low, int32_t high,
                               int32_t near_depth, int32_t planning_width);
/** @brief Beam-corrected edge positions (lateral mm at the face depth,
 *  left positive): each edge midway to the next empty bearing, moved in by
 *  OBS_BEAM_HALF_DEG. Edges that cross (narrower than the beam resolves)
 *  meet in the middle.
 */
static void measured_edges (scan_plan_point_t const * p_points,
                            int32_t count, int32_t low, int32_t high,
                            int32_t near_depth, int32_t * p_left_mm,
                            int32_t * p_right_mm);
/** @brief Nearest echo beyond one edge (samples first..last), OBS_FAR_MM
 *  if none: how much room there is on that side past the object. */
static int32_t open_beyond (scan_plan_point_t const * p_points, int32_t first,
                            int32_t last);
/** @brief Integer square root, rounded down. */
static int32_t isqrt (int32_t value);
/** @brief Sideways offset of the object's corner on one side, from a corner
 *  band echo (beyond +-OBS_SCAN_FINE_DEG, up to OBS_CORNER_MAX_DEG) within
 *  OBS_CORNER_DEPTH_MM of the face range: sqrt(range^2 - face^2), the
 *  farthest out if several. It uses the range, not the bearing, so the
 *  beam width does not blur it.
 *  @param face_mm Nearest range of the object's face.
 *  @param sign 1 for the left side, -1 for the right.
 *  @return The offset in mm (outward), or -1 if no corner echo was seen.
 */
static int32_t corner_offset (scan_plan_point_t const * p_points,
                              int32_t count, int32_t face_mm, int32_t sign);
/** @brief Find the run of near samples around the front. */
static bool find_cluster (scan_plan_point_t const * p_points, int32_t count,
                          int32_t * p_low, int32_t * p_high);
/** @brief Nearest other object in the corridor beside the edge.
 *  @return Its sample index, or -1 if the corridor is clear.
 */
static int32_t corridor_blocker (scan_plan_point_t const * p_points,
                                 int32_t count, int32_t low, int32_t high,
                                 int32_t outward_edge, int32_t limit_mm,
                                 int32_t sign);
/** @brief Pick the smallest angle whose run clears the near face. */
static uint32_t pick_angle (int32_t near_depth, int32_t need_mm);
/** @brief Clamp a sideways need or a range to a 12-bit report field. */
static uint32_t need_field (int32_t need_mm);
/** @brief Fill one side of the debug report.
 *  @param edge_index Outermost object sample on that side.
 *  @param first, last Samples beyond the edge on that side, inclusive.
 */
static void report_side (scan_plan_point_t const * p_points,
                         int32_t edge_index, int32_t edge_mm, bool b_seen,
                         int32_t first, int32_t last,
                         side_option_t const * p_option,
                         scan_plan_side_info_t * p_info);
/** @brief Build a plan; prefer is 1 left, -1 right or 0 for the nearer edge.
 *  @param b_strict Never switch away from a non-zero preferred side.
 *  @param[out] p_sides Side comparison for the log; may be NULL.
 */
static void build_plan (scan_plan_point_t const * p_points, int32_t count,
                        int32_t prefer, bool b_strict,
                        scan_plan_bypass_t * p_plan,
                        scan_plan_sides_t * p_sides);
/** @brief Work out the offset and angle needed on one side.
 *  @param b_edge_seen False if the object reaches the end of the sweep.
 */
static void evaluate_side (scan_plan_point_t const * p_points, int32_t count,
                           int32_t low, int32_t high, int32_t outward_edge,
                           bool b_edge_seen, int32_t near_depth, int32_t sign,
                           side_option_t * p_option);
/** @brief Choose the side and fill in the plan fields.
 *  @param tie_side Side when both moves are exactly equal: 1 left, -1
 *  right (from least_resistance()).
 *  @param[out] p_b_tie Set when both sides were clear and their compared
 *  moves exactly equal.
 */
static void choose_side (side_option_t const * p_left,
                         side_option_t const * p_right, int32_t width_mm,
                         int32_t prefer, bool b_strict, int32_t tie_side,
                         plan_fields_t * p_fields, bool * p_b_tie);
/** @brief Side of least resistance when both sideways moves are equal: the
 *  first of these that differs: more open room past the object, the
 *  smaller drive out, the more open half of the sweep (sum of its ranges).
 *  A perfectly mirror-symmetric scan, where none differ, goes by the parity
 *  of the summed ranges: a coin flip taken from the readings, not a fixed
 *  side.
 *  @return 1 for left, -1 for right.
 */
static int32_t least_resistance (scan_plan_point_t const * p_points,
                                 int32_t count, int32_t low, int32_t high,
                                 bool b_left_seen, bool b_right_seen,
                                 side_option_t const * p_left,
                                 side_option_t const * p_right);
/** @brief Measure the object and plan a path round it. */
static void plan_object (scan_plan_point_t const * p_points, int32_t count,
                         int32_t low, int32_t high, int32_t prefer,
                         bool b_strict, plan_fields_t * p_fields,
                         scan_plan_sides_t * p_sides);
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

int32_t scan_plan_next_angle (int32_t angle_deg)
{
    bool const b_fine = (angle_deg >= -OBS_SCAN_FINE_DEG) &&
                        (angle_deg < OBS_SCAN_FINE_DEG);

    return angle_deg + (b_fine ? OBS_SCAN_FINE_STEP_DEG : OBS_SCAN_STEP_DEG);
}

void scan_plan_build (scan_plan_point_t const * p_points, int32_t count,
                      scan_plan_bypass_t * p_plan)
{
    build_plan(p_points, count, OBS_PREFERRED_SIDE, false, p_plan, NULL);
}

static void build_plan (scan_plan_point_t const * p_points, int32_t count,
                        int32_t prefer, bool b_strict,
                        scan_plan_bypass_t * p_plan,
                        scan_plan_sides_t * p_sides)
{
    plan_fields_t fields = {SCAN_PLAN_CLEAR, false, 0U, 0U, 0U, 0U, 0U, 0U};
    int32_t       low    = 0;
    int32_t       high   = 0;

    if (NULL != p_sides)
    {
        *p_sides = (scan_plan_sides_t){0};
    }

    if (find_cluster(p_points, count, &low, &high))
    {
        plan_object(p_points, count, low, high, prefer, b_strict, &fields,
                    p_sides);
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
    scan_plan_build_side_report(p_points, count, confirmed_front_mm,
                                required_side, p_plan, NULL);
}

void scan_plan_build_side_report (scan_plan_point_t * p_points, int32_t count,
                                  int32_t confirmed_front_mm,
                                  int32_t required_side,
                                  scan_plan_bypass_t * p_plan,
                                  scan_plan_sides_t * p_sides)
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
        build_plan(p_points, count, OBS_PREFERRED_SIDE, false, p_plan,
                   p_sides);
    }
    else
    {
        build_plan(p_points, count, (required_side > 0) ? 1 : -1, true,
                   p_plan, p_sides);
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
    int32_t const next = index + dir;
    /* Gap to the next sample outward; past the sweep end, a full step. */
    int32_t const gap_deg =
        ((next >= 0) && (next < count))
            ? abs_int(p_points[next].angle_deg - p_points[index].angle_deg)
            : OBS_SCAN_STEP_DEG;
    int32_t const pad = point_x(&p_points[index]) +
                        (dir * edge_pad(&p_points[index], gap_deg));
    int32_t       extent = pad;

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

static int32_t lateral_mm (int32_t depth_mm, int32_t angle_x2)
{
    int32_t const limit_x2 =
        HALF * (TRIG_RIGHT_ANGLE_DEG - (2 * TRIG_STEP_DEG));
    int32_t       a_x2     = angle_x2;

    if (a_x2 > limit_x2)
    {
        a_x2 = limit_x2;
    }
    else if (a_x2 < -limit_x2)
    {
        a_x2 = -limit_x2;
    }
    else
    {
        /* Within the table's useful range. */
    }

    if (0 == (a_x2 % (HALF * TRIG_STEP_DEG)))
    {
        int32_t const angle = a_x2 / HALF;

        return (depth_mm * sine_milli(angle)) / cosine_milli(angle);
    }

    /* Half step: average of the two neighbouring table angles. Both are
     * exact multiples of 5 degrees because a_x2 is an odd multiple of 5. */
    int32_t const below = (a_x2 - TRIG_STEP_DEG) / HALF;
    int32_t const above = below + TRIG_STEP_DEG;

    return (((depth_mm * sine_milli(below)) / cosine_milli(below)) +
            ((depth_mm * sine_milli(above)) / cosine_milli(above))) / HALF;
}

static void measured_edges (scan_plan_point_t const * p_points,
                            int32_t count, int32_t low, int32_t high,
                            int32_t near_depth, int32_t * p_left_mm,
                            int32_t * p_right_mm)
{
    int32_t const hi_angle = p_points[high].angle_deg;
    int32_t const lo_angle = p_points[low].angle_deg;
    int32_t const hi_next  = (high + 1 < count) ? p_points[high + 1].angle_deg
                                               : hi_angle + OBS_SCAN_STEP_DEG;
    int32_t const lo_next  = (low > 0) ? p_points[low - 1].angle_deg
                                      : lo_angle - OBS_SCAN_STEP_DEG;
    /* Twice the edge bearings: midway to the next empty ray, less beam. */
    int32_t left_x2  = hi_angle + hi_next - (HALF * OBS_BEAM_HALF_DEG);
    int32_t right_x2 = lo_angle + lo_next + (HALF * OBS_BEAM_HALF_DEG);

    if (left_x2 < right_x2)
    {
        /* Narrower than the beam can resolve: both edges at the middle,
         * rounded to the 2.5 degree grid lateral_mm() accepts. */
        int32_t const mid_x2 =
            ((left_x2 + right_x2) / (HALF * TRIG_STEP_DEG)) * TRIG_STEP_DEG;

        left_x2  = mid_x2;
        right_x2 = mid_x2;
    }

    *p_left_mm  = lateral_mm(near_depth, left_x2);
    *p_right_mm = lateral_mm(near_depth, right_x2);
}

static int32_t measured_width (scan_plan_point_t const * p_points,
                               int32_t count, int32_t low, int32_t high,
                               int32_t near_depth, int32_t planning_width)
{
    int32_t left_mm  = 0;
    int32_t right_mm = 0;

    measured_edges(p_points, count, low, high, near_depth, &left_mm,
                   &right_mm);

    int32_t width = left_mm - right_mm;

    /* Narrower than the beam can resolve: report what can be measured,
     * never a wider object than the planner itself assumed. */
    if (width < 0)
    {
        width = 0;
    }

    return (width < planning_width) ? width : planning_width;
}

static int32_t open_beyond (scan_plan_point_t const * p_points, int32_t first,
                            int32_t last)
{
    int32_t open_mm = OBS_FAR_MM;

    for (int32_t index = first; index <= last; index++)
    {
        if (p_points[index].range_mm < open_mm)
        {
            open_mm = p_points[index].range_mm;
        }
    }

    return open_mm;
}

static int32_t isqrt (int32_t value)
{
    uint32_t remainder = (value > 0) ? (uint32_t)value : 0U;
    uint32_t root      = 0U;
    uint32_t bit       = 1UL << 30;

    while (bit > remainder)
    {
        bit >>= 2;
    }

    while (0U != bit)
    {
        if (remainder >= (root + bit))
        {
            remainder -= root + bit;
            root = (root >> 1) + bit;
        }
        else
        {
            root >>= 1;
        }

        bit >>= 2;
    }

    /* The root of a non-negative int32_t fits int32_t. */
    return (int32_t)root;
}

static int32_t corner_offset (scan_plan_point_t const * p_points,
                              int32_t count, int32_t face_mm, int32_t sign)
{
    int32_t corner = -1;

    for (int32_t index = 0; index < count; index++)
    {
        int32_t const outward = sign * p_points[index].angle_deg;
        int32_t const range   = p_points[index].range_mm;

        if ((outward > OBS_SCAN_FINE_DEG) && (outward <= OBS_CORNER_MAX_DEG) &&
            is_near(&p_points[index]) && (range >= face_mm) &&
            (range <= (face_mm + OBS_CORNER_DEPTH_MM)))
        {
            /* Ranges are below OBS_SCAN_RANGE_MM, so the squares fit. */
            int32_t const offset =
                isqrt((range * range) - (face_mm * face_mm));
            /* A corner this bearing can hear is at most OBS_CORNER_BEAM_DEG
             * off it. Nearer in, the echo is the face itself still
             * answering (a face not square on), not a corner. Bearings and
             * OBS_CORNER_BEAM_DEG are multiples of 5, so the doubled angle
             * is a multiple of 5 as lateral_mm() requires. */
            int32_t const least =
                lateral_mm(face_mm, HALF * (outward - OBS_CORNER_BEAM_DEG));

            if ((offset >= least) && (offset > corner))
            {
                corner = offset;
            }
        }
    }

    return corner;
}

static int32_t edge_pad (scan_plan_point_t const * p_point, int32_t gap_deg)
{
    int32_t const half_deg = gap_deg / HALF;
    /* The sine table is in 5 degree steps. Half of a 5 degree gap is not
     * in it, and sin(2.5) is within 0.1% of sin(5) / 2. */
    int32_t const sine_m =
        (0 == (half_deg % TRIG_STEP_DEG))
            ? sine_milli(half_deg)
            : (sine_milli(gap_deg) / HALF);

    return (p_point->range_mm * sine_m) / MILLI;
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

static uint32_t need_field (int32_t need_mm)
{
    int32_t clamped = need_mm;

    if (clamped < 0)
    {
        clamped = 0;
    }

    if (clamped > NEED_LIMIT_MM)
    {
        clamped = NEED_LIMIT_MM;
    }

    /* The value is clamped to 0..4095, so it fits the 12-bit field. */
    return (uint32_t)clamped;
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

static int32_t corridor_blocker (scan_plan_point_t const * p_points,
                                 int32_t count, int32_t low, int32_t high,
                                 int32_t outward_edge, int32_t limit_mm,
                                 int32_t sign)
{
    int32_t blocker = -1;

    for (int32_t index = 0; index < count; index++)
    {
        bool const b_outside = (index < low) || (index > high);

        if (b_outside && is_near(&p_points[index]))
        {
            int32_t const outward_x = sign * point_x(&p_points[index]);
            int32_t const depth_y   = point_y(&p_points[index]);

            if ((outward_x >= outward_edge) && (outward_x <= limit_mm) &&
                (depth_y <= OBS_LOOKAHEAD_MM) &&
                ((blocker < 0) ||
                 (p_points[index].range_mm < p_points[blocker].range_mm)))
            {
                blocker = index;
            }
        }
    }

    return blocker;
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
    uint32_t      reason  = SCAN_PLAN_NEED_MEASURED;

    if (need_mm < half_mm)
    {
        need_mm = half_mm;
        reason  = SCAN_PLAN_NEED_BODY;
    }

    if ((!b_edge_seen) && (need_mm < OBS_LATERAL_UNKNOWN_MM))
    {
        need_mm = OBS_LATERAL_UNKNOWN_MM;
        reason  = SCAN_PLAN_NEED_UNSEEN;
    }

    p_option->compare_mm = need_mm;

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
        reason  = SCAN_PLAN_NEED_MIN;
    }

    p_option->need_mm     = need_mm;
    p_option->reason      = reason;
    p_option->blocker     = corridor_blocker(p_points, count, low, high,
                                             outward_edge, need_mm + half_mm,
                                             sign);
    p_option->b_ok        = p_option->blocker < 0;
    p_option->angle_index = pick_angle(near_depth, need_mm);
}

static void choose_side (side_option_t const * p_left,
                         side_option_t const * p_right, int32_t width_mm,
                         int32_t prefer, bool b_strict, int32_t tie_side,
                         plan_fields_t * p_fields, bool * p_b_tie)
{
    bool b_pick_left = true;

    *p_b_tie = false;

    if (0 == prefer)
    {
        if (p_left->b_ok && p_right->b_ok)
        {
            /* Both clear: the side needing the smaller move, however small
             * the difference. Needs are a few thousand mm at most, so this
             * cannot overflow.
             */
            int32_t const diff_mm =
                p_left->compare_mm - p_right->compare_mm;

            if (0 == diff_mm)
            {
                *p_b_tie    = true;
                b_pick_left = (tie_side > 0);
            }
            else
            {
                b_pick_left = (diff_mm < 0);
            }
        }
        else if (p_left->b_ok || p_right->b_ok)
        {
            b_pick_left = p_left->b_ok;
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
                         bool b_strict, plan_fields_t * p_fields,
                         scan_plan_sides_t * p_sides)
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
    side_option_t left         = {0, 0U, false, 0U, -1, 0};
    side_option_t right        = {0, 0U, false, 0U, -1, 0};
    int32_t const half_mm = (OBS_ROBOT_WIDTH_MM / HALF) + OBS_SAFETY_MARGIN_MM;
    /* Corner echoes: the object's real extent each side, or -1. Only for an
     * object whose ends are both inside the sweep (a 15 cm box). */
    int32_t       left_corner  = -1;
    int32_t       right_corner = -1;

    if ((0 != OBS_SIDE_CORNER_ECHO) && b_left_seen && b_right_seen)
    {
        /* Face depth: the nearest range. Square on, every echo of the face
         * is the perpendicular one, whatever the bearing. */
        int32_t face_mm = p_points[low].range_mm;

        for (int32_t index = low; index <= high; index++)
        {
            if ((int32_t)p_points[index].range_mm < face_mm)
            {
                face_mm = p_points[index].range_mm;
            }
        }

        left_corner  = corner_offset(p_points, count, face_mm, 1);
        right_corner = corner_offset(p_points, count, face_mm, -1);
    }

    /* Drive out far enough to clear a seen corner, as well as the edges.
     * The width (and so the default straight leg) is not changed. */
    int32_t const left_out  = (left_corner > left_x) ? left_corner : left_x;
    int32_t const right_out =
        (right_corner > -right_x) ? right_corner : -right_x;

    evaluate_side(p_points, count, low, high, left_out, b_left_seen,
                  near_depth, 1, &left);
    evaluate_side(p_points, count, low, high, right_out, b_right_seen,
                  near_depth, -1, &right);

#if OBS_SIDE_COMPARE_MEASURED
    if (b_left_seen && b_right_seen)
    {
        /* Choose the side on the beam-corrected edges, which do not depend
         * on the bearing spacing; the conservative edges above still set
         * how far out the robot drives (need_mm). */
        int32_t left_mm  = 0;
        int32_t right_mm = 0;

        measured_edges(p_points, count, low, high, near_depth, &left_mm,
                       &right_mm);
        left.compare_mm  = left_mm + half_mm;
        right.compare_mm = -right_mm + half_mm;

        if (left.compare_mm < half_mm)
        {
            left.compare_mm = half_mm;
        }

        if (right.compare_mm < half_mm)
        {
            right.compare_mm = half_mm;
        }
    }
#endif

    /* With a corner seen, the edges above are only where the sensor stops
     * seeing the face square on (about +-30 degrees whatever the box's
     * position), so compare the real extent. Offsets are outward from the
     * line on each side, so a box wholly to one side has a negative offset
     * on the other. */
    bool    b_centre  = false;
    int32_t centre_mm = 0;

    if ((left_corner >= 0) || (right_corner >= 0))
    {
        /* An unseen corner: if the face stopped echoing inside the
         * square-on zone, that edge is real (beam-corrected); otherwise the
         * far edge of the widest object. */
        int32_t left_mm  = 0;
        int32_t right_mm = 0;

        measured_edges(p_points, count, low, high, near_depth, &left_mm,
                       &right_mm);

        bool const    b_left_real  =
            p_points[high].angle_deg < OBS_SCAN_FINE_DEG;
        bool const    b_right_real =
            p_points[low].angle_deg > -OBS_SCAN_FINE_DEG;
        int32_t const left_edge  = (left_corner >= 0) ? left_corner
                                   : b_left_real
                                       ? left_mm
                                       : (OBS_OBJECT_MAX_MM - right_corner);
        int32_t const right_edge = (right_corner >= 0) ? right_corner
                                   : b_right_real
                                       ? -right_mm
                                       : (OBS_OBJECT_MAX_MM - left_corner);

        left.compare_mm  = left_edge + half_mm;
        right.compare_mm = right_edge + half_mm;

        if (left.compare_mm < half_mm)
        {
            left.compare_mm = half_mm;
        }

        if (right.compare_mm < half_mm)
        {
            right.compare_mm = half_mm;
        }

        b_centre  = true;
        centre_mm = (left_edge - right_edge) / HALF;
    }

    /* Equal moves: the side of least resistance by the next measure that
     * differs. There is no fixed default side. */
    int32_t const tie_side =
        least_resistance(p_points, count, low, high, b_left_seen,
                         b_right_seen, &left, &right);
    bool          b_tie    = false;

    /* Reported width only; sides and offsets use the conservative edges. */
    int32_t const reported =
        (b_left_seen && b_right_seen)
            ? measured_width(p_points, count, low, high, near_depth, width)
            : width;

    p_fields->width_cm = to_cm(reported, WIDTH_LIMIT_CM);
    p_fields->near_cm  = to_cm(near_depth, DIST_LIMIT_CM);
    choose_side(&left, &right, width, prefer, b_strict, tie_side, p_fields,
                &b_tie);

    if (NULL != p_sides)
    {
        p_sides->left_need_mm  = need_field(left.need_mm);
        p_sides->right_need_mm = need_field(right.need_mm);
        p_sides->b_left_ok     = left.b_ok ? 1U : 0U;
        p_sides->b_right_ok    = right.b_ok ? 1U : 0U;
        p_sides->b_tie         = b_tie ? 1U : 0U;
        p_sides->b_object      = 1U;
        /* Bearings are -70..70 degrees, so they fit the 8-bit fields. */
        p_sides->low_deg  = p_points[low].angle_deg;
        p_sides->high_deg = p_points[high].angle_deg;
        p_sides->near_mm  = need_field(near_depth);
        p_sides->width_mm = need_field(reported);
        report_side(p_points, high, left_out, b_left_seen, high + 1,
                    count - 1, &left, &p_sides->left);
        report_side(p_points, low, right_out, b_right_seen, 0, low - 1,
                    &right, &p_sides->right);
        /* Offsets are below OBS_SCAN_RANGE_MM: they fit the 12-bit fields. */
        p_sides->b_centre        = b_centre ? 1U : 0U;
        p_sides->centre_mm       = centre_mm;
        p_sides->left.b_corner   = (left_corner >= 0) ? 1U : 0U;
        p_sides->left.corner_mm  = (left_corner >= 0) ? left_corner : 0;
        p_sides->right.b_corner  = (right_corner >= 0) ? 1U : 0U;
        p_sides->right.corner_mm = (right_corner >= 0) ? right_corner : 0;
    }
}

static int32_t least_resistance (scan_plan_point_t const * p_points,
                                 int32_t count, int32_t low, int32_t high,
                                 bool b_left_seen, bool b_right_seen,
                                 side_option_t const * p_left,
                                 side_option_t const * p_right)
{
    /* Room past the object: the nearest echo beyond each edge; with an
     * edge unseen there is no room known past it. */
    int32_t const left_open =
        b_left_seen ? open_beyond(p_points, high + 1, count - 1) : 0;
    int32_t const right_open =
        b_right_seen ? open_beyond(p_points, 0, low - 1) : 0;
    /* Openness of each half of the sweep; 0..4095 mm each, a few dozen
     * samples, so the sums fit easily. */
    int32_t left_sum  = 0;
    int32_t right_sum = 0;

    for (int32_t index = 0; index < count; index++)
    {
        if (p_points[index].angle_deg > 0)
        {
            left_sum += (int32_t)p_points[index].range_mm;
        }
        else if (p_points[index].angle_deg < 0)
        {
            right_sum += (int32_t)p_points[index].range_mm;
        }
        else
        {
            /* Straight ahead belongs to neither side. */
        }
    }

    int32_t side = 0;

    if (left_open != right_open)
    {
        side = (left_open > right_open) ? 1 : -1;
    }
    else if (p_left->need_mm != p_right->need_mm)
    {
        side = (p_left->need_mm < p_right->need_mm) ? 1 : -1;
    }
    else if (left_sum != right_sum)
    {
        side = (left_sum > right_sum) ? 1 : -1;
    }
    else
    {
        /* Mirror-symmetric readings: nothing to prefer either side. */
        side = (0 != ((left_sum + right_sum) & 1)) ? 1 : -1;
    }

    return side;
}

static void report_side (scan_plan_point_t const * p_points,
                         int32_t edge_index, int32_t edge_mm, bool b_seen,
                         int32_t first, int32_t last,
                         side_option_t const * p_option,
                         scan_plan_side_info_t * p_info)
{
    int32_t edge    = edge_mm;
    int32_t open_mm = OBS_FAR_MM;

    if (edge < EDGE_MIN_MM)
    {
        edge = EDGE_MIN_MM;
    }

    if (edge > EDGE_MAX_MM)
    {
        edge = EDGE_MAX_MM;
    }

    for (int32_t index = first; index <= last; index++)
    {
        if (p_points[index].range_mm < open_mm)
        {
            open_mm = p_points[index].range_mm;
        }
    }

    *p_info = (scan_plan_side_info_t){0};
    /* Bearings fit 8 bits, edge is clamped to the 12-bit signed field. */
    p_info->edge_deg    = p_points[edge_index].angle_deg;
    p_info->edge_mm     = edge;
    p_info->b_seen      = b_seen ? 1U : 0U;
    p_info->reason      = p_option->reason & 0x3U;
    p_info->open_mm     = need_field(open_mm);
    p_info->angle_index = p_option->angle_index & 0x3U;
    p_info->compare_mm  = need_field(p_option->compare_mm);

    if (p_option->blocker >= 0)
    {
        p_info->b_blocked   = 1U;
        p_info->blocker_deg = p_points[p_option->blocker].angle_deg;
        p_info->blocker_mm  = need_field(p_points[p_option->blocker].range_mm);
    }
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

