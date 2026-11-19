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
 * resistance (least_resistance()). Corner evidence can locate the box in
 * the robot frame and support a tighter clearance target.
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

#if ((OBS_SIDE_PREFERENCE_THRESHOLD_MM < 0) || \
     (OBS_SIDE_PREFERENCE_THRESHOLD_MM > 255))
#error "OBS_SIDE_PREFERENCE_THRESHOLD_MM must be 0..255 mm."
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

#if ((OBS_CORNER_STEP_DEG <= 0) || ((OBS_CORNER_STEP_DEG % 5) != 0) || \
     (((OBS_CORNER_MAX_DEG - OBS_SCAN_FINE_DEG) % OBS_CORNER_STEP_DEG) != 0))
#error "OBS_CORNER_STEP_DEG must be a multiple of 5 dividing the corner band."
#endif

#if ((OBS_CORNER_FROM_DEG < OBS_SCAN_FINE_DEG) || \
     (OBS_CORNER_FROM_DEG > OBS_CORNER_MAX_DEG) || \
     (OBS_CORNER_FROM_DEG <= OBS_CORNER_BEAM_DEG))
#error "OBS_CORNER_FROM_DEG must be FINE_DEG..CORNER_MAX_DEG, above the beam."
#endif

#if ((OBS_FACE_BOWL_MM <= 0) || (OBS_FACE_BOWL_MM > 100) || \
     (OBS_FACE_TILT_MAX_DEG < 0) || (OBS_FACE_TILT_MAX_DEG > 30))
#error "OBS_FACE_BOWL_MM must be 1..100 and OBS_FACE_TILT_MAX_DEG 0..30."
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
    int32_t left_mm;  /**< Left edge in the robot frame, left positive. */
    int32_t right_mm; /**< Right edge in the same frame. */
} front_span_t;

typedef struct
{
    int32_t stop_mm; /**< Earlier confirmed forward range, or 0. */
    int32_t tie_side; /**< Earlier plan's side if the fresh scan ties. */
    bool    b_measured; /**< The centre sample came from this sweep. */
} scan_anchor_t;

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
                               int32_t near_depth, int32_t tilt_x2,
                               int32_t planning_width);
/** @brief Beam-corrected edge positions (lateral mm at the face depth,
 *  left positive): each edge midway to the next empty bearing, moved in by
 *  OBS_BEAM_HALF_DEG. Edges that cross (narrower than the beam resolves)
 *  meet in the middle. Measured from the face's normal (tilt_x2, half
 *  degrees from face_tilt_x2()), so a robot that stopped turned to one
 *  side does not make the box look longer on that side.
 */
static void measured_edges (scan_plan_point_t const * p_points,
                            int32_t count, int32_t low, int32_t high,
                            int32_t near_depth, int32_t tilt_x2,
                            int32_t * p_left_mm, int32_t * p_right_mm);
/** @brief Nearest echo beyond one edge (samples first..last), OBS_FAR_MM
 *  if none: how much room there is on that side past the object. */
static int32_t open_beyond (scan_plan_point_t const * p_points, int32_t first,
                            int32_t last);
/** @brief Integer square root, rounded down. */
static int32_t isqrt (int32_t value);
/** @brief Bearing of the face's normal (where the robot is square to it),
 *  in half degrees, + = left, a multiple of 5 (2.5 degree steps): the
 *  centre of the bearings within OBS_FACE_BOWL_MM of the nearest echo,
 *  each weighted by how much nearer than that it is. Off-axis echoes come
 *  back a little late, so the range profile is a bowl centred on it. At
 *  most OBS_FACE_TILT_MAX_DEG either way.
 *  @param low, high The object's samples.
 *  @param face_mm Nearest range among them.
 */
static int32_t face_tilt_x2 (scan_plan_point_t const * p_points, int32_t low,
                             int32_t high, int32_t face_mm);
/** @brief Sideways offset of the object's corner on one side, from a corner
 *  band echo (beyond +-OBS_SCAN_FINE_DEG, up to OBS_CORNER_MAX_DEG, and at
 *  least OBS_CORNER_FROM_DEG off the face normal) within
 *  OBS_CORNER_DEPTH_MM of the face range: sqrt(range^2 - face^2), the
 *  farthest out if several. It uses the range, not the bearing, so the
 *  beam width does not blur it.
 *  @param face_mm Nearest range of the object's face.
 *  @param tilt_x2 Face normal from face_tilt_x2().
 *  @param sign 1 for the left side, -1 for the right.
 *  @return The offset in mm (outward), or -1 if no corner echo was seen.
 */
static int32_t corner_offset (scan_plan_point_t const * p_points,
                              int32_t count, int32_t face_mm,
                              int32_t tilt_x2, int32_t sign);
/** @brief True if a range could be the object's corner: within
 *  OBS_CORNER_DEPTH_MM beyond the face range and inside the scan range. */
static bool corner_window (int32_t range_mm, int32_t face_mm);
/** @brief The corner echo range at one corner band sample: its confirmed
 *  reading if that is in the corner window, else its lone echo if the next
 *  corner bearing on the same side heard the same range (within
 *  OBS_CORNER_NEIGHBOUR_MM), else -1. */
static int32_t corner_range (scan_plan_point_t const * p_points,
                             int32_t count, int32_t index, int32_t face_mm,
                             int32_t sign);
/** @brief Find the run of near samples around the front. */
static bool find_cluster (scan_plan_point_t const * p_points, int32_t count,
                          int32_t * p_low, int32_t * p_high);
/** @brief Nearest other object in the corridor beside the edge.
 *  @param face_mm When positive, corner band echoes near this face range
 *  are placed sideways by their range, as corners are.
 *  @return Its sample index, or -1 if the corridor is clear.
 */
static int32_t corridor_blocker (scan_plan_point_t const * p_points,
                                 int32_t count, int32_t low, int32_t high,
                                 int32_t outward_edge, int32_t limit_mm,
                                 int32_t face_mm, int32_t sign);
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
                        scan_anchor_t anchor,
                        scan_plan_bypass_t * p_plan,
                        scan_plan_sides_t * p_sides);
/** @brief Work out the offset and angle needed on one side.
 *  @param b_edge_seen False if the object reaches the end of the sweep.
 *  @param face_mm Nearest face range for corner band echoes (see
 *  corridor_blocker()), or 0 when corners are not used.
 */
static void evaluate_side (scan_plan_point_t const * p_points, int32_t count,
                           int32_t low, int32_t high, int32_t outward_edge,
                           bool b_edge_seen, int32_t near_depth,
                           int32_t face_mm, int32_t sign,
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
/** @brief Side for an about-centred box: the first of these that differs:
 *  more room past the object (the nearest echo beyond each edge), more room
 *  on average beyond each edge, an off-centre terminal echo, then the
 *  smaller estimated move. A mirror-symmetric scan goes by
 *  the parity of the summed ranges: a coin flip taken from the readings,
 *  not a fixed side. Nothing measured from straight ahead is used: a robot
 *  that stopped turned to one side would bias it.
 *  @return 1 for left, -1 for right.
 */
static int32_t least_resistance (scan_plan_point_t const * p_points,
                                 int32_t count, int32_t low, int32_t high,
                                 bool b_left_seen, bool b_right_seen,
                                 side_option_t const * p_left,
                                 side_option_t const * p_right,
                                 int32_t edge_preference);
/** @brief The tilt to compare sides in: the face normal, or 0 if unseen. */
static int32_t compare_tilt_x2 (scan_plan_point_t const * p_points,
                                int32_t low, int32_t high, int32_t tilt_x2);
/** @brief Measure the object and plan a path round it. */
static void plan_object (scan_plan_point_t const * p_points, int32_t count,
                         int32_t low, int32_t high, int32_t prefer,
                         bool b_strict, scan_anchor_t anchor,
                         plan_fields_t * p_fields,
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
    bool const b_fine   = (angle_deg >= -OBS_SCAN_FINE_DEG) &&
                          (angle_deg < OBS_SCAN_FINE_DEG);
    bool const b_corner = (angle_deg >= -OBS_CORNER_MAX_DEG) &&
                          (angle_deg < OBS_CORNER_MAX_DEG);

    return angle_deg + (b_fine     ? OBS_SCAN_FINE_STEP_DEG
                        : b_corner ? OBS_CORNER_STEP_DEG
                                   : OBS_SCAN_STEP_DEG);
}

void scan_plan_build (scan_plan_point_t const * p_points, int32_t count,
                      scan_plan_bypass_t * p_plan)
{
    scan_anchor_t const anchor = {0, 0, true};
    build_plan(p_points, count, OBS_PREFERRED_SIDE, false, anchor, p_plan,
               NULL);
}

static void build_plan (scan_plan_point_t const * p_points, int32_t count,
                        int32_t prefer, bool b_strict,
                        scan_anchor_t anchor,
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
        plan_object(p_points, count, low, high, prefer, b_strict, anchor,
                    &fields, p_sides);
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

    scan_anchor_t anchor = {0, 0, false};

    if ((confirmed_front_mm >= 20) && (confirmed_front_mm <= OBS_STOP_MM))
    {
        anchor.stop_mm = confirmed_front_mm;
    }

    for (int32_t index = 0; index < count; index++)
    {
        if (0 == p_points[index].angle_deg)
        {
            int32_t const fresh_mm = p_points[index].range_mm;
            anchor.b_measured = is_near(&p_points[index]);

            if ((anchor.stop_mm > 0) && (fresh_mm > anchor.stop_mm))
            {
                if ((0 == OBS_FRESH_SCAN_CENTRE) || (!anchor.b_measured))
                {
                    /* The stop range still protects a missing centre echo. */
                    p_points[index].range_mm = anchor.stop_mm;
                    anchor.b_measured = false;
                }
                else if ((0 == required_side) && (0 == OBS_PREFERRED_SIDE) &&
                         (fresh_mm - anchor.stop_mm >= OBS_EDGE_SPLIT_JUMP_MM))
                {
                    /* Keep the earlier side only as a tie-break when the
                     * fresh scan finds two clear corridors. */
                    scan_plan_bypass_t earlier = {0U, 0U};
                    scan_anchor_t const previous = {anchor.stop_mm, 0, false};
                    p_points[index].range_mm = anchor.stop_mm;
                    build_plan(p_points, count, 0, false, previous, &earlier,
                               NULL);
                    p_points[index].range_mm = fresh_mm;
                    if (SCAN_PLAN_ROUTE == scan_plan_result(&earlier))
                    {
                        anchor.tie_side = scan_plan_side_is_left(&earlier)
                                              ? 1 : -1;
                    }
                }
                else
                {
                    /* The fresh reading supplies the centre geometry. */
                }
            }

            break;
        }
    }

    int32_t const prefer = (0 == required_side) ? OBS_PREFERRED_SIDE
                         : (required_side > 0) ? 1 : -1;
    build_plan(p_points, count, prefer, 0 != required_side, anchor, p_plan,
               p_sides);
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
                            int32_t near_depth, int32_t tilt_x2,
                            int32_t * p_left_mm, int32_t * p_right_mm)
{
    int32_t const hi_angle = p_points[high].angle_deg;
    int32_t const lo_angle = p_points[low].angle_deg;
    int32_t const hi_next  = (high + 1 < count) ? p_points[high + 1].angle_deg
                                               : hi_angle + OBS_SCAN_STEP_DEG;
    int32_t const lo_next  = (low > 0) ? p_points[low - 1].angle_deg
                                      : lo_angle - OBS_SCAN_STEP_DEG;
    /* Twice the edge bearings: midway to the next empty ray, less beam. */
    /* Bearings are multiples of 5 and the tilt of 2.5 degrees, so these
     * stay multiples of 5 half degrees, as lateral_mm() requires. */
    int32_t left_x2  = hi_angle + hi_next - (HALF * OBS_BEAM_HALF_DEG) -
                      tilt_x2;
    int32_t right_x2 = lo_angle + lo_next + (HALF * OBS_BEAM_HALF_DEG) -
                       tilt_x2;

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
                               int32_t near_depth, int32_t tilt_x2,
                               int32_t planning_width)
{
    int32_t left_mm  = 0;
    int32_t right_mm = 0;

    measured_edges(p_points, count, low, high, near_depth, tilt_x2,
                   &left_mm, &right_mm);

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

/* The face normal only says where the line runs when the face's nearest
 * echo lies inside the echo span, ranges rising towards both ends (logged
 * 19 Nov 12:32: 250 mm at -15, 211 at 25, 227 at 40 deg: a face turned 20
 * degrees, the robot stopped turned on the line). When the nearest echo is
 * an end of the span (a box wholly to one side, square on) the normal is
 * not seen, so measure from sonar zero instead. */
static int32_t compare_tilt_x2 (scan_plan_point_t const * p_points,
                                int32_t low, int32_t high, int32_t tilt_x2)
{
    int32_t nearest = (int32_t)p_points[low].range_mm;

    for (int32_t index = low; index <= high; index++)
    {
        if ((int32_t)p_points[index].range_mm < nearest)
        {
            nearest = (int32_t)p_points[index].range_mm;
        }
    }

    bool const b_bowl =
        (((int32_t)p_points[low].range_mm - nearest) >= OBS_TILT_BOWL_MM) &&
        (((int32_t)p_points[high].range_mm - nearest) >= OBS_TILT_BOWL_MM);

    return b_bowl ? tilt_x2 : 0;
}

static int32_t face_tilt_x2 (scan_plan_point_t const * p_points, int32_t low,
                             int32_t high, int32_t face_mm)
{
    /* The whole object: a robot stopped 20 degrees off square has its
     * nearest echoes out past 25 degrees (logged 22:27). */
    int32_t const reach = OBS_CORNER_MAX_DEG;
    int32_t const unit  = TRIG_STEP_DEG; /* half degrees: 2.5 degrees */
    int32_t const limit = HALF * OBS_FACE_TILT_MAX_DEG;
    int32_t       num   = 0;
    int32_t       den   = 0;

    for (int32_t index = low; index <= high; index++)
    {
        int32_t const angle  = p_points[index].angle_deg;
        int32_t const weight =
            OBS_FACE_BOWL_MM - ((int32_t)p_points[index].range_mm - face_mm);

        if ((angle >= -reach) && (angle <= reach) && (weight > 0))
        {
            /* At most 13 bearings of 25 degrees x 2 x 100: no overflow. */
            num += HALF * angle * weight;
            den += weight;
        }
    }

    int32_t tilt = 0;

    if (den > 0)
    {
        /* Round the weighted centre to the nearest 2.5 degrees. */
        int32_t const span = den * unit;

        tilt = ((num >= 0) ? ((num + (span / HALF)) / span)
                           : -(((-num) + (span / HALF)) / span)) *
               unit;
    }

    if (tilt > limit)
    {
        tilt = limit;
    }

    if (tilt < -limit)
    {
        tilt = -limit;
    }

    return tilt;
}

static int32_t corner_offset (scan_plan_point_t const * p_points,
                              int32_t count, int32_t face_mm,
                              int32_t tilt_x2, int32_t sign)
{
    int32_t corner = -1;

    for (int32_t index = 0; index < count; index++)
    {
        int32_t const outward = sign * p_points[index].angle_deg;
        /* Twice the bearing off the face normal, outward. Bearings are
         * multiples of 5 and the tilt of 2.5, so this is a multiple of 5
         * as lateral_mm() requires. */
        int32_t const off_x2  = (HALF * outward) - (sign * tilt_x2);
        int32_t const range   =
            ((outward > OBS_SCAN_FINE_DEG) && (outward <= OBS_CORNER_MAX_DEG))
                ? corner_range(p_points, count, index, face_mm, sign)
                : -1;

        if ((range >= 0) && (off_x2 >= (HALF * OBS_CORNER_FROM_DEG)))
        {
            /* Ranges are below OBS_SCAN_RANGE_MM, so the squares fit. */
            int32_t const offset =
                isqrt((range * range) - (face_mm * face_mm));
            /* A corner this bearing can hear is at most OBS_CORNER_BEAM_DEG
             * off it. Nearer in, the echo is the face itself still
             * answering (a face not square on), not a corner. */
            int32_t const least =
                lateral_mm(face_mm, off_x2 - (HALF * OBS_CORNER_BEAM_DEG));

            if ((offset >= least) && (offset > corner))
            {
                corner = offset;
            }
        }
    }

    return corner;
}

static bool corner_window (int32_t range_mm, int32_t face_mm)
{
    return (range_mm >= face_mm) &&
           (range_mm <= (face_mm + OBS_CORNER_DEPTH_MM)) &&
           (range_mm < OBS_SCAN_RANGE_MM);
}

static int32_t corner_range (scan_plan_point_t const * p_points,
                             int32_t count, int32_t index, int32_t face_mm,
                             int32_t sign)
{
    int32_t const confirmed = p_points[index].range_mm;
    int32_t const lone      = p_points[index].lone_mm;

    if (corner_window(confirmed, face_mm))
    {
        return confirmed;
    }

    if ((0 == lone) || (!corner_window(lone, face_mm)))
    {
        return -1;
    }

    /* A lone echo: the next corner bearing either way must have heard the
     * same corner, confirmed or lone. */
    for (int32_t step = -1; step <= 1; step += HALF)
    {
        int32_t const other = index + step;

        if ((other >= 0) && (other < count))
        {
            int32_t const outward = sign * p_points[other].angle_deg;
            int32_t const heard   = p_points[other].range_mm;
            int32_t const single  = p_points[other].lone_mm;
            bool const    b_band  = (outward > OBS_SCAN_FINE_DEG) &&
                                    (outward <= OBS_CORNER_MAX_DEG);

            if (b_band &&
                ((corner_window(heard, face_mm) &&
                  (abs_int(heard - lone) <= OBS_CORNER_NEIGHBOUR_MM)) ||
                 ((0 != single) && corner_window(single, face_mm) &&
                  (abs_int(single - lone) <= OBS_CORNER_NEIGHBOUR_MM))))
            {
                return lone;
            }
        }
    }

    return -1;
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
                                 int32_t face_mm, int32_t sign)
{
    int32_t blocker = -1;

    for (int32_t index = 0; index < count; index++)
    {
        bool const b_outside = (index < low) || (index > high);

        if (b_outside && is_near(&p_points[index]))
        {
            int32_t const angle     = p_points[index].angle_deg;
            int32_t const range     = p_points[index].range_mm;
            int32_t const outward   = sign * angle;
            int32_t       outward_x = sign * point_x(&p_points[index]);
            int32_t const depth_y   = point_y(&p_points[index]);
            bool          b_own     = false;

            if ((face_mm > 0) && (outward > OBS_SCAN_FINE_DEG) &&
                (outward <= OBS_CORNER_MAX_DEG) && (range >= face_mm) &&
                (range <= (face_mm + OBS_CORNER_DEPTH_MM)))
            {
                /* An echo near the face's depth in the corner band, apart
                 * from the face only because the bearing between did not
                 * echo: where it is sideways follows from its range, as for
                 * a corner; the bearing overstates it by up to a beam
                 * width. Ranges are below OBS_SCAN_RANGE_MM: no overflow.
                 * Out to the object's edge, it is the object's own corner
                 * (the edge includes seen corners), not something else. */
                outward_x = isqrt((range * range) - (face_mm * face_mm));
                b_own     = outward_x <= outward_edge;
            }

            if ((!b_own) && (outward_x >= outward_edge) &&
                (outward_x <= limit_mm) &&
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
                           bool b_edge_seen, int32_t near_depth,
                           int32_t face_mm, int32_t sign,
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
                                             face_mm, sign);
    p_option->b_ok        = p_option->blocker < 0;
    p_option->angle_index = pick_angle(near_depth, need_mm);
}

/* Linear interpolation for the 2.5-degree grid of the face-normal fit. */
static int32_t sine_half_milli (int32_t angle_x2)
{
    int32_t const step_x2 = HALF * TRIG_STEP_DEG;
    int32_t const angle = (angle_x2 / step_x2) * TRIG_STEP_DEG;

    if (0 == (angle_x2 % step_x2))
    {
        return sine_milli(angle);
    }

    int32_t const next = angle + ((angle_x2 < 0) ? -TRIG_STEP_DEG
                                                : TRIG_STEP_DEG);
    return (sine_milli(angle) + sine_milli(next)) / HALF;
}

static int32_t round_milli (int32_t value)
{
    int32_t const half = MILLI / HALF;
    return (value + ((value >= 0) ? half : -half)) / MILLI;
}

/* Rotate a span along the box face into the robot frame. Sonar zero is
 * still straight ahead. The face normal only rotates and translates it. */
static front_span_t project_span (int32_t face_mm, int32_t tilt_x2,
                                  int32_t left_corner, int32_t right_corner,
                                  int32_t width_mm)
{
    int32_t const left = (left_corner >= 0) ? left_corner
                                            : width_mm - right_corner;
    int32_t const right = (right_corner >= 0) ? -right_corner
                                              : left_corner - width_mm;
    int32_t const sin_m = sine_half_milli(tilt_x2);
    int32_t const cos_m = sine_half_milli(
        (HALF * TRIG_RIGHT_ANGLE_DEG) - abs_int(tilt_x2));
    front_span_t const span = {
        round_milli((face_mm * sin_m) + (left * cos_m)),
        round_milli((face_mm * sin_m) + (right * cos_m))};

    return span;
}

/* Retain the original corridor check and also check the narrower path in
 * the robot frame. Round the reduced target outward before packing it. */
static int32_t tighten_clearance (scan_plan_point_t const * p_points,
                                  int32_t count, int32_t low, int32_t high,
                                  int32_t outward, int32_t near_depth,
                                  int32_t sign, int32_t original_edge,
                                  side_option_t * p_side)
{
    int32_t const half_mm = (OBS_ROBOT_WIDTH_MM / HALF) + OBS_SAFETY_MARGIN_MM;
    int32_t const padded = outward + OBS_EDGE_PROJECTION_PAD_MM;
    int32_t const edge = (padded > 0) ? padded : 0;
    int32_t const need = (((edge + half_mm) + MM_PER_CM - 1) / MM_PER_CM) *
                         MM_PER_CM;

    if (p_side->b_ok && (need < p_side->need_mm) &&
        (corridor_blocker(p_points, count, low, high, edge, need + half_mm,
                          0, sign) < 0))
    {
        p_side->need_mm = need;
        p_side->angle_index = pick_angle(near_depth, need);
        p_side->reason = SCAN_PLAN_NEED_MEASURED;
        return edge;
    }

    return original_edge;
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

            if (abs_int(diff_mm) <= OBS_SIDE_PREFERENCE_THRESHOLD_MM)
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
                         bool b_strict, scan_anchor_t anchor,
                         plan_fields_t * p_fields,
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
    int32_t       tilt_x2      = 0;
    /* Face range for corner band echoes; 0 while corners are not used. */
    int32_t       face_mm      = 0;

    if (b_left_seen && b_right_seen)
    {
        /* Face depth: the nearest range. Square on, every echo of the face
         * is the perpendicular one, whatever the bearing. */
        int32_t nearest = p_points[low].range_mm;

        for (int32_t index = low; index <= high; index++)
        {
            if ((int32_t)p_points[index].range_mm < nearest)
            {
                nearest = p_points[index].range_mm;
            }
        }

        /* Edges and corners are judged from where the robot is square to
         * the face, not from straight ahead. */
        tilt_x2 = face_tilt_x2(p_points, low, high, nearest);

        if (0 != OBS_SIDE_CORNER_ECHO)
        {
            face_mm      = nearest;
            left_corner  = corner_offset(p_points, count, face_mm, tilt_x2,
                                         1);
            right_corner = corner_offset(p_points, count, face_mm, tilt_x2,
                                         -1);
        }
    }

    /* Drive out far enough to clear a seen corner, as well as the edges.
     * The width (and so the default straight leg) is not changed. */
    int32_t left_out  = (left_corner > left_x) ? left_corner : left_x;
    int32_t right_out =
        (right_corner > -right_x) ? right_corner : -right_x;

    int32_t const turn_depth = ((anchor.stop_mm > 0) &&
                                (anchor.stop_mm < near_depth))
                                   ? anchor.stop_mm : near_depth;
    evaluate_side(p_points, count, low, high, left_out, b_left_seen,
                  turn_depth, face_mm, 1, &left);
    evaluate_side(p_points, count, low, high, right_out, b_right_seen,
                  turn_depth, face_mm, -1, &right);

#if OBS_SIDE_COMPARE_MEASURED
    if (b_left_seen && b_right_seen)
    {
        /* Choose the side on the beam-corrected edges, which do not depend
         * on the bearing spacing; the conservative edges above still set
         * how far out the robot drives (need_mm). Measured from the face
         * normal, not sonar zero: the box sits square to the line, and a
         * robot that stopped turned on the line (logged 19 Nov 12:32: face
         * 20 degrees off, heading still swinging) would otherwise pick the
         * side that is shorter only in its own skewed frame. */
        int32_t left_mm  = 0;
        int32_t right_mm = 0;

        measured_edges(p_points, count, low, high, near_depth,
                       compare_tilt_x2(p_points, low, high, tilt_x2),
                       &left_mm, &right_mm);
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

        measured_edges(p_points, count, low, high, near_depth, tilt_x2,
                       &left_mm, &right_mm);

        /* Inside the square-on zone, counted from the face's normal. */
        bool const    b_left_real  =
            ((HALF * p_points[high].angle_deg) - tilt_x2) <
            (HALF * OBS_SCAN_FINE_DEG);
        bool const    b_right_real =
            ((HALF * p_points[low].angle_deg) - tilt_x2) >
            -(HALF * OBS_SCAN_FINE_DEG);
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

    int32_t const reported =
        (b_left_seen && b_right_seen)
            ? measured_width(p_points, count, low, high, near_depth, tilt_x2,
                             width)
            : width;
    front_span_t front = {0, 0};
    int32_t edge_preference = 0;
    bool b_front_estimate = true;

    if ((0 != OBS_FORWARD_EDGE_SPLIT) && (face_mm > 0) && anchor.b_measured)
    {
        if (b_centre)
        {
            front = project_span(face_mm, tilt_x2, left_corner, right_corner,
                                 reported);

            b_front_estimate = (left_corner < 0) || (right_corner < 0);

            if ((!b_strict) && (!b_front_estimate))
            {
                /* Real corners anchor both edges. The rear of a tilted
                 * box can protrude farther than its front. */
                int32_t const box_mm = (reported > OBS_OBJECT_MAX_MM)
                                           ? reported : OBS_OBJECT_MAX_MM;
                int32_t const rear =
                    (box_mm * sine_half_milli(tilt_x2)) / MILLI;
                int32_t const l = front.left_mm + ((rear > 0) ? rear : 0);
                int32_t const r = -front.right_mm + ((rear < 0) ? -rear : 0);

                left_out = tighten_clearance(p_points, count, low, high, l,
                                             turn_depth, 1, left_out, &left);
                right_out = tighten_clearance(p_points, count, low, high, r,
                                               turn_depth, -1, right_out,
                                               &right);
                left.compare_mm = half_mm + ((l > 0) ? l : 0);
                right.compare_mm = half_mm + ((r > 0) ? r : 0);
            }
        }
        else
        {
            /* The scan did not resolve the split. */
        }

    }

    int32_t tie_side =
        least_resistance(p_points, count, low, high, b_left_seen,
                         b_right_seen, &left, &right, edge_preference);
    if ((0 != anchor.tie_side) && (!b_centre) && (0 == edge_preference) &&
        b_left_seen && b_right_seen &&
        (open_beyond(p_points, high + 1, count - 1) >= OBS_LOOKAHEAD_MM) &&
        (open_beyond(p_points, 0, low - 1) >= OBS_LOOKAHEAD_MM))
    {
        tie_side = anchor.tie_side;
    }
    bool          b_tie    = false;

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
        /* Within +-2 * OBS_FACE_TILT_MAX_DEG (60): fits the 7-bit field. */
        p_sides->tilt_x2         = tilt_x2;
        p_sides->centre_mm       = centre_mm;
        /* Dashboard split: exactly the extents the side choice compared
         * (beam-corrected, from the face normal, i.e. the line), so the
         * reading and the decision always agree. A side the box does not
         * reach shows 0. One-sided echo guesses are no longer reported:
         * logged 19 Nov 12:33 one stray echo showed 173/12 mm for a box
         * that was about centred. */
        int32_t const split_left  = left.compare_mm - half_mm;
        int32_t const split_right = right.compare_mm - half_mm;
        int32_t const front_width = split_left + split_right;

        (void)front;
        if (b_left_seen && b_right_seen && (front_width > 0))
        {
            p_sides->front_width_mm   = need_field(front_width);
            p_sides->front_left_mm    = need_field(split_left);
            p_sides->b_front_estimate = b_front_estimate ? 1U : 0U;
        }
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
                                 side_option_t const * p_right,
                                 int32_t edge_preference)
{
    /* Room past the object: the nearest echo beyond each edge; with an
     * edge unseen there is no room known past it. */
    int32_t const left_open =
        b_left_seen ? open_beyond(p_points, high + 1, count - 1) : 0;
    int32_t const right_open =
        b_right_seen ? open_beyond(p_points, 0, low - 1) : 0;
    /* Average room past each edge (the surroundings, not the object). At
     * most 4095 mm on a few dozen samples, so the sums fit easily. */
    int32_t left_sum  = 0;
    int32_t right_sum = 0;
    int32_t left_n    = 0;
    int32_t right_n   = 0;

    for (int32_t index = 0; index < count; index++)
    {
        if (index > high)
        {
            left_sum += (int32_t)p_points[index].range_mm;
            left_n++;
        }
        else if (index < low)
        {
            right_sum += (int32_t)p_points[index].range_mm;
            right_n++;
        }
        else
        {
            /* The object itself. */
        }
    }

    int32_t const left_mean  = (left_n > 0) ? (left_sum / left_n) : 0;
    int32_t const right_mean = (right_n > 0) ? (right_sum / right_n) : 0;
    int32_t       side       = 0;

    if (left_open != right_open)
    {
        side = (left_open > right_open) ? 1 : -1;
    }
    else if (left_mean != right_mean)
    {
        side = (left_mean > right_mean) ? 1 : -1;
    }
    else if (0 != edge_preference)
    {
        side = edge_preference;
    }
    else if (p_left->compare_mm != p_right->compare_mm)
    {
        /* Same room either way: the side the box sticks out less on, even
         * by a little (measured from the face's normal). */
        side = (p_left->compare_mm < p_right->compare_mm) ? 1 : -1;
    }
    else if (p_left->need_mm != p_right->need_mm)
    {
        /* Equal at the beam's resolution: the smaller drive-out. */
        side = (p_left->need_mm < p_right->need_mm) ? 1 : -1;
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

