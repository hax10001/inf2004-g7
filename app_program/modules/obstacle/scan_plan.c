/** @file scan_plan.c
 *
 * @brief Edge finding and bypass geometry from a servo range profile.
 *
 * Each sample becomes an echo point in the vehicle frame, starting from
 * where the servo really carries the transducer, not from the axle. The
 * object is the run of near profile samples around the front; a profile
 * sample with no echo next to that run is assumed to be more of the object,
 * never free space. The object's outer edges, padded by half a scan step,
 * set the passing offset: edge plus half the body width plus the body
 * margin plus the position allowance. An edge that reaches the end of the
 * profile was never seen, so that side needs at least
 * OBS_LATERAL_UNKNOWN_MM.
 *
 * For each side the planner then plays the whole manoeuvre through: the
 * pivot out on one wheel, the diagonal run, the pivot back to parallel on
 * the other wheel and the run past the object. A departure angle is used
 * only if the full body keeps its margin from every scanned surface at
 * every step. Valid echoes outside the object, including the far bearings,
 * are other obstacles in the corridor. A side whose outer bearings gave no
 * echo at all is not verified and is not used. Of two usable sides the
 * one needing clearly less offset wins; otherwise the one with more room
 * beyond its path. Integer maths is used because the RP2040 has no FPU.
 */
#include "scan_plan.h"
#include "obs_config.h"
#include "obs_geometry.h"
#include <stddef.h>

#if ((OBS_SCAN_STEP_DEG % 10) != 0)
#error "OBS_SCAN_STEP_DEG must be a multiple of 10 degrees."
#endif

#define HALF 2
#define ANGLE_CHOICES 4U
#define MM_PER_CM 10
#define ROUND_MM 5
#define WIDTH_LIMIT_CM 254
#define DIST_LIMIT_CM 510
#define SLACK_LIMIT_CM 63
#define SLACK_LIMIT_MM (SLACK_LIMIT_CM * MM_PER_CM)
#define SEED_NONE_DEG 32767
#define TURN_STEP_DEG 5
#define RUN_STEP_MM 20
#define MAX_BEARING_DEG 90
#define CONFIRMED_MIN_MM 20

#define SAMPLE_VALID 0U
#define SAMPLE_BEYOND 1U
#define SAMPLE_NONE 2U

/* Passing offset beyond an object edge: half the body, the body margin
 * and the allowance for position error.
 */
#define HALF_PATH_MM                                                           \
    ((OBS_ROBOT_WIDTH_MM / HALF) + OBS_SAFETY_MARGIN_MM + OBS_OUTWARD_EXTRA_MM)
/* Gap the swept body keeps from every scanned surface. */
#define SWEEP_CLEAR_MM (OBS_SAFETY_MARGIN_MM + OBS_RANGE_UNCERT_MM)
#define HALF_TRACK_MM (OBS_TRACK_WIDTH_MM / HALF)
#define HALF_BODY_MM (OBS_ROBOT_WIDTH_MM / HALF)

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
#define SLACK_SHIFT 18U
#define SLACK_MASK 0x3FU

static int32_t const g_angle_choice_deg[ANGLE_CHOICES] = {30, 45, 60, 90};

/** @brief One sample with its echo point in the vehicle frame. */
typedef struct
{
    int16_t range_mm;
    int16_t fwd_mm;
    int16_t lat_mm;
    int8_t  angle_deg;
    uint8_t kind;
    bool    b_assumed; /**< No echo, taken as more of the object. */
} sample_t;

typedef struct
{
    sample_t samples[OBS_SCAN_MAX_POINTS];
    int32_t  count;
    int32_t  profile_low;  /**< First profile sample, the rightmost. */
    int32_t  profile_high; /**< Last profile sample, the leftmost. */
    int32_t  low;          /**< Rightmost sample of the object. */
    int32_t  high;         /**< Leftmost sample of the object. */
    int32_t  left_mm;      /**< Left edge, padded and extended if unseen. */
    int32_t  right_mm;     /**< Right edge, padded and extended if unseen. */
    int32_t  near_mm;      /**< Nearest face, forward of the axle. */
    int32_t  width_mm;     /**< Width between the padded edges seen. */
    bool     b_left_seen;
    bool     b_right_seen;
} scan_view_t;

typedef struct
{
    int32_t  need_mm;
    int32_t  slack_mm;
    int32_t  diag_mm;
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
    uint32_t slack_cm;
} plan_fields_t;

/** @brief Absolute value of a signed integer. */
static int32_t abs_int (int32_t value);
/** @brief Smaller of two values. */
static int32_t min_int (int32_t first, int32_t second);
/** @brief Larger of two values. */
static int32_t max_int (int32_t first, int32_t second);
/** @brief Convert millimetres to a clamped, rounded, non-negative cm. */
static uint32_t to_cm (int32_t length_mm, int32_t limit_cm);
/** @brief Give a sample a range and work out its echo point. */
static void set_range (sample_t * p_sample, int32_t range_mm);
/** @brief Copy and check the samples; false if they are unusable. */
static bool load_view (scan_plan_point_t const * p_points, int32_t count,
                       scan_view_t * p_view);
/** @brief True if a sample is a real echo close enough to be the object. */
static bool is_near (sample_t const * p_sample);
/** @brief True if the next profile sample continues the object. */
static bool joins (scan_view_t * p_view, int32_t next, int32_t current);
/** @brief Find the run of near samples around the front. */
static bool find_cluster (scan_view_t * p_view);
/** @brief Set the object's edges, width and nearest face. */
static void measure_object (scan_view_t * p_view);
/** @brief Box of one object sample, joined to its left neighbour. */
static void object_box (scan_view_t const * p_view, int32_t index,
                        obs_geometry_box_t * p_box);
/** @brief True if a valid sample outside the object is another obstacle. */
static bool is_other (scan_view_t const * p_view, int32_t index);
/** @brief True if the body at a pose comes too close to anything seen. */
static bool pose_hits (scan_view_t const * p_view,
                       obs_geometry_pose_t const * p_pose);
/** @brief Play one departure through; true if the body stays clear.
 *  @param[out] p_end_mm Axle x once back parallel to the line.
 */
static bool path_clear (scan_view_t const * p_view, int32_t side,
                        int32_t need_mm, int32_t angle_deg,
                        int32_t * p_diag_mm, int32_t * p_end_mm);
/** @brief Where the side beam first meets the object's side, along x, when
 *  the car is back parallel at a given axle x.
 */
static int32_t side_view_mm (int32_t side, int32_t end_mm, int32_t gap_mm);
/** @brief True if no sample on a side lacks an echo, so it is verified. */
static bool side_verified (scan_view_t const * p_view, int32_t side);
/** @brief Room beyond the body margin outside the passing path. */
static int32_t side_slack (scan_view_t const * p_view, int32_t side,
                           int32_t need_mm);
/** @brief Offset that also passes the other surfaces seen on a side. */
static int32_t wider_need (scan_view_t const * p_view, int32_t side,
                           int32_t need_mm);
/** @brief Find the departure angle for one offset; false if none fits. */
static bool pick_angle (scan_view_t const * p_view, int32_t side,
                        int32_t edge_mm, side_option_t * p_option);
/** @brief Work out the offset, angle and room on one side. */
static void evaluate_side (scan_view_t const * p_view, int32_t side,
                           side_option_t * p_option);
/** @brief Choose the better side and fill in the plan fields. */
static void choose_side (scan_view_t const * p_view,
                         side_option_t const * p_left,
                         side_option_t const * p_right,
                         plan_fields_t * p_fields);
/** @brief Internal planner using outer side-corridor vetoes. */
static void build_with_limits (scan_plan_point_t const * p_points,
                               int32_t count, bool b_left_clear,
                               bool b_right_clear,
                               scan_plan_bypass_t * p_plan);
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
    if (NULL != p_plan)
    {
        build_with_limits(p_points, count, true, true, p_plan);
    }
}

void scan_plan_build_confirmed (scan_plan_point_t * p_points, int32_t count,
                                int32_t confirmed_front_mm,
                                scan_plan_bypass_t * p_plan)
{
    scan_plan_build_checked(p_points, count, confirmed_front_mm, true, true,
                            p_plan);
}

void scan_plan_build_checked (scan_plan_point_t * p_points, int32_t count,
                              int32_t confirmed_front_mm, bool b_left_clear,
                              bool b_right_clear, scan_plan_bypass_t * p_plan)
{
    if (NULL == p_plan)
    {
        return;
    }

    if ((NULL == p_points) || (count <= 0) || (count > OBS_SCAN_MAX_POINTS))
    {
        p_plan->word_a = SCAN_PLAN_NO_ROUTE;
        p_plan->word_b = 0U;
        return;
    }

    if ((confirmed_front_mm >= CONFIRMED_MIN_MM) &&
        (confirmed_front_mm <= OBS_STOP_MM))
    {
        for (int32_t index = 0; index < count; index++)
        {
            /* A missed or farther centre echo cannot cancel the object. */
            if ((0 == p_points[index].angle_deg) &&
                ((p_points[index].range_mm > confirmed_front_mm) ||
                 (p_points[index].range_mm < 0)))
            {
                p_points[index].range_mm = confirmed_front_mm;
            }
        }
    }

    build_with_limits(p_points, count, b_left_clear, b_right_clear, p_plan);
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

int32_t scan_plan_slack_cm (scan_plan_bypass_t const * p_plan)
{
    /* The field is at most 6 bits wide, so the cast is exact. */
    return (int32_t)((p_plan->word_b >> SLACK_SHIFT) & SLACK_MASK);
}

static int32_t abs_int (int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t min_int (int32_t first, int32_t second)
{
    return (first < second) ? first : second;
}

static int32_t max_int (int32_t first, int32_t second)
{
    return (first > second) ? first : second;
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

static void set_range (sample_t * p_sample, int32_t range_mm)
{
    int32_t fwd_mm = 0;
    int32_t lat_mm = 0;

    obs_geometry_echo_point(p_sample->angle_deg, range_mm, &fwd_mm, &lat_mm);
    /* Ranges are at most OBS_FAR_MM, so every value fits in 16 bits. */
    p_sample->range_mm = (int16_t)range_mm;
    p_sample->fwd_mm   = (int16_t)fwd_mm;
    p_sample->lat_mm   = (int16_t)lat_mm;
}

static bool load_view (scan_plan_point_t const * p_points, int32_t count,
                       scan_view_t * p_view)
{
    bool b_ok = (NULL != p_points) && (count > 0) &&
                (count <= OBS_SCAN_MAX_POINTS);

    p_view->count        = count;
    p_view->profile_low  = -1;
    p_view->profile_high = -1;

    for (int32_t index = 0; b_ok && (index < count); index++)
    {
        int32_t const angle = p_points[index].angle_deg;
        int32_t const range = p_points[index].range_mm;
        sample_t *    p_one = &p_view->samples[index];

        /* Bearings must rise from right to left and stay within reach. */
        b_ok = (abs_int(angle) <= MAX_BEARING_DEG) &&
               ((0 == index) || (angle > p_points[index - 1].angle_deg));

        if (b_ok)
        {
            /* The bearing is checked to be within +/-90 just above. */
            p_one->angle_deg = (int8_t)angle;
            p_one->b_assumed = false;

            if (range < 0)
            {
                p_one->kind = SAMPLE_NONE;
                set_range(p_one, 0);
            }
            else if (range >= OBS_FAR_MM)
            {
                p_one->kind = SAMPLE_BEYOND;
                set_range(p_one, OBS_FAR_MM);
            }
            else
            {
                p_one->kind = SAMPLE_VALID;
                set_range(p_one, range);
            }

            if ((angle >= -OBS_SCAN_RIGHT_DEG) &&
                (angle <= OBS_SCAN_LEFT_DEG))
            {
                p_view->profile_low =
                    (p_view->profile_low < 0) ? index : p_view->profile_low;
                p_view->profile_high = index;
            }
        }
    }

    return b_ok && (p_view->profile_low >= 0);
}

static bool is_near (sample_t const * p_sample)
{
    return (SAMPLE_VALID == p_sample->kind) &&
           (p_sample->range_mm < OBS_SCAN_RANGE_MM);
}

static bool joins (scan_view_t * p_view, int32_t next, int32_t current)
{
    sample_t * const       p_next    = &p_view->samples[next];
    sample_t const * const p_current = &p_view->samples[current];
    bool                   b_joins   = false;

    if (is_near(p_next) &&
        (abs_int(p_next->range_mm - p_current->range_mm) < OBS_EDGE_JUMP_MM))
    {
        b_joins = true;
    }
    else if (SAMPLE_NONE == p_next->kind)
    {
        /* No echo beside the object: assume the object goes on there. */
        p_next->b_assumed = true;
        set_range(p_next, p_current->range_mm);
        b_joins = true;
    }
    else
    {
        /* An edge: a jump in range, or nothing near. */
    }

    return b_joins;
}

static bool find_cluster (scan_view_t * p_view)
{
    int32_t seed    = -1;
    int32_t best    = SEED_NONE_DEG;
    bool    b_found = false;

    for (int32_t index = p_view->profile_low; index <= p_view->profile_high;
         index++)
    {
        int32_t const offset = abs_int(p_view->samples[index].angle_deg);

        if (is_near(&p_view->samples[index]) && (offset < best))
        {
            best = offset;
            seed = index;
        }
    }

    if ((seed >= 0) && (best <= OBS_FRONT_WINDOW_DEG))
    {
        int32_t low  = seed;
        int32_t high = seed;

        while ((low > p_view->profile_low) && joins(p_view, low - 1, low))
        {
            low--;
        }

        while ((high < p_view->profile_high) && joins(p_view, high + 1, high))
        {
            high++;
        }

        p_view->low  = low;
        p_view->high = high;
        b_found      = true;
    }

    return b_found;
}

static void measure_object (scan_view_t * p_view)
{
    int32_t const pad_sine = obs_geometry_sin_milli(OBS_SCAN_STEP_DEG / HALF);
    int32_t const unseen   = OBS_LATERAL_UNKNOWN_MM - HALF_PATH_MM;
    int32_t       left     = INT32_MIN;
    int32_t       right    = INT32_MAX;
    int32_t       near     = INT32_MAX;

    for (int32_t index = p_view->low; index <= p_view->high; index++)
    {
        sample_t const * const p_one = &p_view->samples[index];
        int32_t const          pad =
            (p_one->range_mm * pad_sine) / OBS_GEOMETRY_MILLI;

        left  = max_int(left, p_one->lat_mm + pad);
        right = min_int(right, p_one->lat_mm - pad);
        near  = min_int(near, p_one->fwd_mm);
    }

    p_view->width_mm     = left - right;
    p_view->near_mm      = near;
    p_view->b_left_seen  = p_view->high < p_view->profile_high;
    p_view->b_right_seen = p_view->low > p_view->profile_low;

    /* An unseen edge is assumed no nearer than the unknown-edge offset
     * allows, so the swept check models the same object the offset does.
     */
    p_view->left_mm  = p_view->b_left_seen ? left : max_int(left, unseen);
    p_view->right_mm = p_view->b_right_seen ? right : min_int(right, -unseen);
}

static void object_box (scan_view_t const * p_view, int32_t index,
                        obs_geometry_box_t * p_box)
{
    sample_t const * const p_one = &p_view->samples[index];
    int32_t                fwd_low  = p_one->fwd_mm;
    int32_t                fwd_high = p_one->fwd_mm;
    int32_t                lat_low  = p_one->lat_mm;
    int32_t                lat_high = p_one->lat_mm;

    if (index < p_view->high)
    {
        sample_t const * const p_next = &p_view->samples[index + 1];

        fwd_low  = min_int(fwd_low, p_next->fwd_mm);
        fwd_high = max_int(fwd_high, p_next->fwd_mm);
        lat_low  = min_int(lat_low, p_next->lat_mm);
        lat_high = max_int(lat_high, p_next->lat_mm);
    }

    if (index == p_view->low)
    {
        lat_low = min_int(lat_low, p_view->right_mm);
    }

    if (index == p_view->high)
    {
        lat_high = max_int(lat_high, p_view->left_mm);
    }

    /* The face is seen; the depth behind it is assumed. */
    p_box->x_low_mm  = fwd_low;
    p_box->x_high_mm = fwd_high + OBS_DEFAULT_DEPTH_MM;
    p_box->y_low_mm  = lat_low;
    p_box->y_high_mm = lat_high;
}

static bool is_other (scan_view_t const * p_view, int32_t index)
{
    sample_t const * const p_one = &p_view->samples[index];

    return (SAMPLE_VALID == p_one->kind) && (!p_one->b_assumed) &&
           ((index < p_view->low) || (index > p_view->high));
}

static bool pose_hits (scan_view_t const * p_view,
                       obs_geometry_pose_t const * p_pose)
{
    bool b_hit = false;

    for (int32_t index = p_view->low; (index <= p_view->high) && (!b_hit);
         index++)
    {
        obs_geometry_box_t box = {0, 0, 0, 0};

        object_box(p_view, index, &box);
        b_hit = obs_geometry_body_hits_box(p_pose, &box, SWEEP_CLEAR_MM);
    }

    for (int32_t index = 0; (index < p_view->count) && (!b_hit); index++)
    {
        if (is_other(p_view, index))
        {
            sample_t const * const   p_one = &p_view->samples[index];
            obs_geometry_box_t const box   = {p_one->fwd_mm, p_one->fwd_mm,
                                              p_one->lat_mm, p_one->lat_mm};

            b_hit = obs_geometry_body_hits_box(p_pose, &box, SWEEP_CLEAR_MM);
        }
    }

    return b_hit;
}

static bool path_clear (scan_view_t const * p_view, int32_t side,
                        int32_t need_mm, int32_t angle_deg,
                        int32_t * p_diag_mm, int32_t * p_end_mm)
{
    obs_geometry_pose_t const origin = {0, 0, 0};
    obs_geometry_pose_t       pose   = origin;
    obs_geometry_pose_t       start  = origin;
    /* Each pivot on a wheel carries the axle outwards by this much. */
    int32_t const gain =
        (HALF_TRACK_MM * (OBS_GEOMETRY_MILLI -
                          obs_geometry_cos_milli(angle_deg))) /
        OBS_GEOMETRY_MILLI;
    int32_t const diag_mm =
        (max_int(0, need_mm - (HALF * gain)) * OBS_GEOMETRY_MILLI) /
        obs_geometry_sin_milli(angle_deg);
    bool b_hit = pose_hits(p_view, &origin);

    /* 1. Pivot out on the wheel on the bypass side. */
    for (int32_t turn = TURN_STEP_DEG; (turn <= angle_deg) && (!b_hit);
         turn += TURN_STEP_DEG)
    {
        obs_geometry_pivot(&origin, side, side * turn, &pose);
        b_hit = pose_hits(p_view, &pose);
    }

    /* 2. Run out on the diagonal. */
    start = pose;

    for (int32_t run = RUN_STEP_MM; (run < (diag_mm + RUN_STEP_MM)) && (!b_hit);
         run += RUN_STEP_MM)
    {
        obs_geometry_advance(&start, min_int(run, diag_mm), &pose);
        b_hit = pose_hits(p_view, &pose);
    }

    /* 3. Pivot back to parallel on the wheel nearer the object. */
    start = pose;

    for (int32_t turn = TURN_STEP_DEG; (turn <= angle_deg) && (!b_hit);
         turn += TURN_STEP_DEG)
    {
        obs_geometry_pivot(&start, -side, -side * turn, &pose);
        b_hit = pose_hits(p_view, &pose);
    }

    /* 4. Run past the object until the rear clears its assumed depth. */
    start     = pose;
    *p_end_mm = pose.x_mm;

    int32_t const run_mm = (p_view->near_mm + OBS_DEFAULT_DEPTH_MM +
                            OBS_ROBOT_REAR_MM + SWEEP_CLEAR_MM) -
                           start.x_mm;

    for (int32_t run = RUN_STEP_MM; (run < (run_mm + RUN_STEP_MM)) && (!b_hit);
         run += RUN_STEP_MM)
    {
        obs_geometry_advance(&start, min_int(run, run_mm), &pose);
        b_hit = pose_hits(p_view, &pose);
    }

    *p_diag_mm = diag_mm;

    return !b_hit;
}

static int32_t side_view_mm (int32_t side, int32_t end_mm, int32_t gap_mm)
{
    int32_t const reach =
        (side > 0) ? OBS_SERVO_RIGHT_LIMIT_DEG : OBS_SERVO_LEFT_LIMIT_DEG;
    int32_t const look =
        (reach < OBS_SIDE_LOOK_DEG) ? reach : OBS_SIDE_LOOK_DEG;
    int32_t origin_fwd = 0;
    int32_t origin_lat = 0;

    /* The object is on the other side, so the beam looks that way. */
    obs_geometry_sensor_origin(-side * look, &origin_fwd, &origin_lat);

    return end_mm + origin_fwd +
           ((gap_mm * obs_geometry_cos_milli(look)) /
            obs_geometry_sin_milli(look));
}

static bool side_verified (scan_view_t const * p_view, int32_t side)
{
    int32_t const outer    = (side > 0) ? (p_view->count - 1) : 0;
    bool          b_known  = true;
    bool          b_beside = (side * p_view->samples[outer].angle_deg) > 0;

    for (int32_t index = 0; index < p_view->count; index++)
    {
        sample_t const * const p_one = &p_view->samples[index];
        bool const b_outside = (index < p_view->low) || (index > p_view->high);

        if (b_outside && ((side * p_one->angle_deg) > 0) &&
            (SAMPLE_NONE == p_one->kind))
        {
            b_known = false;
        }
    }

    /* The outermost bearing on the side must exist and must have spoken. */
    return b_known && b_beside &&
           (SAMPLE_NONE != p_view->samples[outer].kind);
}

static int32_t side_slack (scan_view_t const * p_view, int32_t side,
                           int32_t need_mm)
{
    int32_t const edge   = (side > 0) ? p_view->left_mm : -p_view->right_mm;
    int32_t const outer  = need_mm + HALF_BODY_MM + OBS_SAFETY_MARGIN_MM;
    int32_t const reach  = p_view->near_mm + OBS_DEFAULT_DEPTH_MM +
                          OBS_LOOKAHEAD_MM;
    int32_t       slack  = SLACK_LIMIT_MM;

    for (int32_t index = 0; index < p_view->count; index++)
    {
        if (is_other(p_view, index))
        {
            sample_t const * const p_one   = &p_view->samples[index];
            int32_t const          outward = side * p_one->lat_mm;

            if ((outward > edge) && (p_one->fwd_mm >= -OBS_ROBOT_REAR_MM) &&
                (p_one->fwd_mm <= reach))
            {
                slack = min_int(slack, outward - outer);
            }
        }
    }

    return max_int(0, slack);
}

static int32_t wider_need (scan_view_t const * p_view, int32_t side,
                           int32_t need_mm)
{
    int32_t const edge  = (side > 0) ? p_view->left_mm : -p_view->right_mm;
    int32_t const reach = p_view->near_mm + OBS_DEFAULT_DEPTH_MM +
                          OBS_LOOKAHEAD_MM;
    int32_t       need  = need_mm;

    for (int32_t index = 0; index < p_view->count; index++)
    {
        if (is_other(p_view, index))
        {
            sample_t const * const p_one   = &p_view->samples[index];
            int32_t const          outward = side * p_one->lat_mm;

            if ((outward > edge) && (p_one->fwd_mm <= reach))
            {
                need = max_int(need, outward + HALF_PATH_MM);
            }
        }
    }

    return need;
}

static bool pick_angle (scan_view_t const * p_view, int32_t side,
                        int32_t edge_mm, side_option_t * p_option)
{
    int32_t const view_limit = p_view->near_mm + OBS_MIN_DEPTH_MM;
    int32_t       best_view  = INT32_MAX;
    bool          b_fit      = false;
    bool          b_seen     = false;

    /* The gentlest angle whose whole manoeuvre keeps clear and whose side
     * beam still meets the object's side is used. Failing the second, the
     * clear angle that looks back furthest is used.
     */
    for (uint32_t choice = 0U; (choice < ANGLE_CHOICES) && (!b_seen);
         choice++)
    {
        int32_t diag_mm = 0;
        int32_t end_mm  = 0;

        if (path_clear(p_view, side, p_option->need_mm,
                       g_angle_choice_deg[choice], &diag_mm, &end_mm))
        {
            int32_t const view_mm =
                side_view_mm(side, end_mm, p_option->need_mm - edge_mm);

            if ((!b_fit) || (view_mm < best_view))
            {
                best_view             = view_mm;
                p_option->angle_index = choice;
                p_option->diag_mm     = diag_mm;
            }

            b_fit  = true;
            b_seen = view_mm <= view_limit;
        }
    }

    return b_fit;
}

static void evaluate_side (scan_view_t const * p_view, int32_t side,
                           side_option_t * p_option)
{
    int32_t const edge  = (side > 0) ? p_view->left_mm : -p_view->right_mm;
    bool          b_fit = false;

    p_option->need_mm     = max_int(edge, 0) + HALF_PATH_MM;
    p_option->angle_index = ANGLE_CHOICES - 1U;
    p_option->diag_mm     = 0;
    p_option->slack_mm    = 0;
    p_option->b_ok        = side_verified(p_view, side);

    if (p_option->b_ok)
    {
        b_fit = pick_angle(p_view, side, edge, p_option);

        if (!b_fit)
        {
            /* Another surface on this side is in the way: go round it as
             * well, but never wider than the widest offset ever planned.
             */
            int32_t const wider = wider_need(p_view, side, p_option->need_mm);

            if ((wider > p_option->need_mm) &&
                (wider <= OBS_LATERAL_UNKNOWN_MM))
            {
                p_option->need_mm = wider;
                b_fit = pick_angle(p_view, side, edge, p_option);
            }
        }
    }

    p_option->b_ok = p_option->b_ok && b_fit;

    if (p_option->b_ok)
    {
        p_option->slack_mm = side_slack(p_view, side, p_option->need_mm);
    }
}

static void choose_side (scan_view_t const * p_view,
                         side_option_t const * p_left,
                         side_option_t const * p_right,
                         plan_fields_t * p_fields)
{
    bool b_pick_left = p_left->b_ok;

    if (p_left->b_ok && p_right->b_ok)
    {
        int32_t const left_far =
            p_view->samples[p_view->profile_high].range_mm;
        int32_t const right_far =
            p_view->samples[p_view->profile_low].range_mm;

        if ((p_left->need_mm + OBS_SIDE_PREFER_MM) < p_right->need_mm)
        {
            b_pick_left = true; /* Clearly the shorter way round. */
        }
        else if ((p_right->need_mm + OBS_SIDE_PREFER_MM) < p_left->need_mm)
        {
            b_pick_left = false;
        }
        else if (p_left->slack_mm != p_right->slack_mm)
        {
            b_pick_left = p_left->slack_mm > p_right->slack_mm;
        }
        else if ((!p_view->b_left_seen) && (!p_view->b_right_seen) &&
                 (left_far != right_far))
        {
            /* Neither edge seen: go where the outermost profile sample is
             * farther, as that end of the object is more likely to turn
             * away.
             */
            b_pick_left = left_far > right_far;
        }
        else
        {
            b_pick_left = p_left->need_mm <= p_right->need_mm;
        }
    }

    if (p_left->b_ok || p_right->b_ok)
    {
        side_option_t const * const p_pick = b_pick_left ? p_left : p_right;

        p_fields->result      = SCAN_PLAN_ROUTE;
        p_fields->b_left      = b_pick_left;
        p_fields->angle_index = p_pick->angle_index;
        p_fields->lateral_cm  = to_cm(p_pick->need_mm, DIST_LIMIT_CM);
        p_fields->diag_cm     = to_cm(p_pick->diag_mm, DIST_LIMIT_CM);
        p_fields->slack_cm    = to_cm(p_pick->slack_mm, SLACK_LIMIT_CM);
        p_fields->fallback_cm = to_cm(p_view->width_mm + OBS_ROBOT_FRONT_MM +
                                          OBS_ROBOT_REAR_MM +
                                          (HALF * OBS_SAFETY_MARGIN_MM),
                                      DIST_LIMIT_CM);
    }
    else
    {
        p_fields->result = SCAN_PLAN_NO_ROUTE;
    }
}

static void build_with_limits (scan_plan_point_t const * p_points,
                               int32_t count, bool b_left_clear,
                               bool b_right_clear,
                               scan_plan_bypass_t * p_plan)
{
    plan_fields_t fields = {SCAN_PLAN_CLEAR, false, 0U, 0U, 0U,
                            0U,              0U,    0U, 0U};
    scan_view_t   view;

    if (!load_view(p_points, count, &view))
    {
        fields.result = SCAN_PLAN_NO_ROUTE;
    }
    else if (find_cluster(&view))
    {
        side_option_t left  = {0, 0, 0, 0U, false};
        side_option_t right = {0, 0, 0, 0U, false};

        measure_object(&view);
        evaluate_side(&view, 1, &left);
        evaluate_side(&view, -1, &right);
        /* Callers may still veto a side from their own outer checks. */
        left.b_ok  = left.b_ok && b_left_clear;
        right.b_ok = right.b_ok && b_right_clear;
        fields.width_cm = to_cm(view.width_mm, WIDTH_LIMIT_CM);
        fields.near_cm  = to_cm(view.near_mm, DIST_LIMIT_CM);
        choose_side(&view, &left, &right, &fields);
    }
    else
    {
        /* Nothing near ahead: the way is clear. */
    }

    pack_fields(&fields, p_plan);
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
        ((p_fields->slack_cm & SLACK_MASK) << SLACK_SHIFT);
}

/*** end of file ***/

