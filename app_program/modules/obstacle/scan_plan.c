/** @file scan_plan.c
 * @brief Edge finding and bypass geometry from a servo range profile.
 *
 * Each sample becomes x = r sin(a) (left positive) and y = r cos(a). The
 * object is the run of near samples around FRONT. Its outer edges, padded by
 * half a scan step, give the lateral offset a path needs: edge plus half the
 * robot width plus a margin. The smallest of 30, 45, 60 degrees whose
 * diagonal run still clears the nearest face is used.
 */
#include "scan_plan.h"
#include "obs_config.h"
#include <math.h>
#include <string.h>

#define DEG_TO_RAD 0.01745329252F
#define ANGLE_CHOICES 3U
#define WIDTH_MAX_CM 254U
#define DIST_MAX_CM 510U

static float const g_angles[ANGLE_CHOICES] = {30.0F, 45.0F, 60.0F};

typedef struct
{
    float    need_mm;
    uint32_t angle_idx;
    bool     b_ok;
} side_option_t;

static bool     is_near (scan_point_t const * p_pt);
static float    point_x (scan_point_t const * p_pt);
static float    point_y (scan_point_t const * p_pt);
static float    edge_pad (scan_point_t const * p_pt);
static uint16_t diff_u16 (uint16_t a, uint16_t b);
static uint32_t to_cm (float mm, uint32_t limit);
static void     evaluate_side (scan_point_t const * p_pts, uint8_t count,
                               uint8_t lo, uint8_t hi, float outward_edge,
                               float y_near, float sign, side_option_t * p_opt);

float scan_plan_angle_deg (uint32_t angle_idx)
{
    return g_angles[(angle_idx < ANGLE_CHOICES) ? angle_idx : 0U];
}

void scan_plan_build (scan_point_t const * p_pts, uint8_t count,
                      bypass_plan_t * p_plan)
{
    (void)memset(p_plan, 0, sizeof(*p_plan));
    p_plan->result = (uint32_t)PLAN_CLEAR;

    int32_t seed = -1;
    int32_t best = 0x7FFF;

    for (uint8_t i = 0U; i < count; i++)
    {
        int32_t const a =
            (p_pts[i].angle_deg < 0) ? -p_pts[i].angle_deg : p_pts[i].angle_deg;

        if (is_near(&p_pts[i]) && (a < best))
        {
            best = a;
            seed = (int32_t)i;
        }
    }

    if ((seed < 0) || (best > (2 * OBS_SCAN_STEP_DEG)))
    {
        return;
    }

    uint8_t lo = (uint8_t)seed;
    uint8_t hi = (uint8_t)seed;

    while ((lo > 0U) && is_near(&p_pts[lo - 1U]) &&
           (diff_u16(p_pts[lo - 1U].range_mm, p_pts[lo].range_mm) <
            OBS_EDGE_JUMP_MM))
    {
        lo--;
    }

    while (((uint8_t)(hi + 1U) < count) && is_near(&p_pts[hi + 1U]) &&
           (diff_u16(p_pts[hi + 1U].range_mm, p_pts[hi].range_mm) <
            OBS_EDGE_JUMP_MM))
    {
        hi++;
    }

    float y_near = point_y(&p_pts[lo]);

    for (uint8_t i = lo; i <= hi; i++)
    {
        float const y = point_y(&p_pts[i]);
        y_near        = (y < y_near) ? y : y_near;
    }

    float const x_left  = point_x(&p_pts[hi]) + edge_pad(&p_pts[hi]);
    float const x_right = point_x(&p_pts[lo]) - edge_pad(&p_pts[lo]);
    float const width   = x_left - x_right;

    side_option_t left  = {0.0F, 0U, false};
    side_option_t right = {0.0F, 0U, false};

    evaluate_side(p_pts, count, lo, hi, x_left, y_near, 1.0F, &left);
    evaluate_side(p_pts, count, lo, hi, -x_right, y_near, -1.0F, &right);

    p_plan->width_cm = to_cm(width, WIDTH_MAX_CM);
    p_plan->near_cm  = to_cm(y_near, DIST_MAX_CM);

    bool b_left = false;

    if (left.b_ok && right.b_ok)
    {
        b_left = (left.need_mm <= right.need_mm); /* Tie goes left. */
    }
    else if (left.b_ok)
    {
        b_left = true;
    }
    else if (right.b_ok)
    {
        b_left = false;
    }
    else
    {
        p_plan->result = (uint32_t)PLAN_NO_ROUTE;
        return;
    }

    side_option_t const * const p_pick = b_left ? &left : &right;
    float const sin_a = sinf(g_angles[p_pick->angle_idx] * DEG_TO_RAD);

    p_plan->result     = (uint32_t)PLAN_ROUTE;
    p_plan->side_left  = b_left ? 1U : 0U;
    p_plan->angle_idx  = p_pick->angle_idx;
    p_plan->lateral_cm = to_cm(p_pick->need_mm, DIST_MAX_CM);
    p_plan->diag_cm    = to_cm(p_pick->need_mm / sin_a, DIST_MAX_CM);
    p_plan->fallback_cm =
        to_cm(width + OBS_ROBOT_LENGTH_MM + (2.0F * OBS_SAFETY_MARGIN_MM),
              DIST_MAX_CM);
}

static void evaluate_side (scan_point_t const * p_pts, uint8_t count,
                           uint8_t lo, uint8_t hi, float outward_edge,
                           float y_near, float sign, side_option_t * p_opt)
{
    float const half = (0.5F * OBS_ROBOT_WIDTH_MM) + OBS_SAFETY_MARGIN_MM;
    float       need = outward_edge + half;

    need             = (need < half) ? half : need;
    p_opt->need_mm   = need;
    p_opt->b_ok      = false;
    p_opt->angle_idx = 0U;

    for (uint8_t i = 0U; i < count; i++)
    {
        if ((i >= lo) && (i <= hi))
        {
            continue;
        }

        if (is_near(&p_pts[i]))
        {
            float const ox = sign * point_x(&p_pts[i]);
            float const oy = point_y(&p_pts[i]);

            if ((ox >= outward_edge) && (ox <= (need + half)) &&
                (oy <= (y_near + OBS_BYPASS_LOOKAHEAD_MM)))
            {
                return; /* Another object sits in the corridor. */
            }
        }
    }

    for (uint32_t k = 0U; k < ANGLE_CHOICES; k++)
    {
        float const advance = need / tanf(g_angles[k] * DEG_TO_RAD);

        if ((advance + OBS_FRONT_CLEAR_MM) <= y_near)
        {
            p_opt->b_ok      = true;
            p_opt->angle_idx = k;
            return;
        }
    }
}

static bool is_near (scan_point_t const * p_pt)
{
    return p_pt->range_mm < OBS_SCAN_RANGE_MM;
}

static float point_x (scan_point_t const * p_pt)
{
    return (float)p_pt->range_mm * sinf((float)p_pt->angle_deg * DEG_TO_RAD);
}

static float point_y (scan_point_t const * p_pt)
{
    return (float)p_pt->range_mm * cosf((float)p_pt->angle_deg * DEG_TO_RAD);
}

static float edge_pad (scan_point_t const * p_pt)
{
    return (float)p_pt->range_mm *
           sinf(0.5F * (float)OBS_SCAN_STEP_DEG * DEG_TO_RAD);
}

static uint16_t diff_u16 (uint16_t a, uint16_t b)
{
    return (a > b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint32_t to_cm (float mm, uint32_t limit)
{
    if (mm <= 0.0F)
    {
        return 0U;
    }

    uint32_t const cm = (uint32_t)((mm + 5.0F) / 10.0F);
    return (cm > limit) ? limit : cm;
}

/*** end of file ***/

