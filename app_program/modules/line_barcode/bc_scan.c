/** @file bc_scan.c
 * @brief Encoder-distance acquisition of one Code 39 letter per frame.
 */
#include "bc_scan.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>

#define BC_QUIET_MULTIPLE 10U
#define BC_CONFIRM_SAMPLES 2U

static int64_t count_delta (int32_t current, int32_t previous);
static bool skew_exceeded (bc_scan_t const * p_scan, uint64_t left,
                            uint64_t right);
static bool motion_valid (bc_scan_t * p_scan, uint64_t left, uint64_t right);
static void reject_scan (bc_scan_t * p_scan, bc_status_t status);
static bool finish_scan (bc_scan_t * p_scan);
static bool transition (bc_scan_t * p_scan, uint64_t edge_position,
                         uint32_t edge_usec, uint32_t from_level);

bool bc_scan_init (bc_scan_t * p_scan, bc_scan_config_t const * p_config)
{
    if ((NULL == p_scan) || (NULL == p_config))
    {
        return false;
    }

    memset(p_scan, 0, sizeof(*p_scan));
    p_scan->config = *p_config;
    return (p_config->black_level <= 1U) &&
           ((1 == p_config->left_sign) || (-1 == p_config->left_sign)) &&
           ((1 == p_config->right_sign) || (-1 == p_config->right_sign)) &&
           (p_config->max_skew_percent < 100U) &&
           (0U < p_config->max_gap_usec) &&
           (p_config->max_gap_usec < p_config->stop_usec) &&
           (p_config->stop_usec < p_config->timeout_usec) &&
           (p_config->timeout_usec < (UINT32_MAX / 2U));
}

void bc_scan_reset (bc_scan_t * p_scan)
{
    bc_scan_config_t const config = p_scan->config;

    (void)bc_scan_init(p_scan, &config);
}

bool bc_scan_feed (bc_scan_t * p_scan, uint32_t raw, int32_t left,
                   int32_t right, uint32_t now_usec)
{
    if (BC_COMPLETE == p_scan->state)
    {
        return false;
    }

    if (raw > 1U)
    {
        reject_scan(p_scan, BC_ERR_SAMPLE);
        return false;
    }

    if (!p_scan->b_seeded)
    {
        p_scan->last_left = left;
        p_scan->last_right = right;
        p_scan->last_usec = now_usec;
        p_scan->moved_usec = now_usec;
        p_scan->stable_level = raw;
        p_scan->white_start = p_scan->position;
        p_scan->white_max_step = 0U;
        p_scan->motion_left_units = 0U;
        p_scan->motion_right_units = 0U;
        p_scan->b_seeded = true;
        return false;
    }

    int64_t const dl = count_delta(left, p_scan->last_left) *
                       p_scan->config.left_sign;
    int64_t const dr = count_delta(right, p_scan->last_right) *
                       p_scan->config.right_sign;
    uint32_t const gap = now_usec - p_scan->last_usec;

    p_scan->last_left = left;
    p_scan->last_right = right;
    p_scan->last_usec = now_usec;

    if (gap > p_scan->config.max_gap_usec)
    {
        reject_scan(p_scan, BC_ERR_SAMPLE);
        return false;
    }
    if ((dl < 0) || (dr < 0))
    {
        reject_scan(p_scan, BC_ERR_REVERSE);
        return false;
    }
    if ((dl + dr) > UINT32_MAX)
    {
        reject_scan(p_scan, BC_ERR_OVERFLOW);
        return false;
    }

    uint32_t const step = (uint32_t)(dl + dr);
    if (!motion_valid(p_scan, (uint64_t)dl, (uint64_t)dr))
    {
        reject_scan(p_scan, BC_ERR_SKEW);
        return false;
    }

    p_scan->position += step;
    if (((BC_WAIT_WHITE == p_scan->state) || (BC_ERROR == p_scan->state)) &&
        (step > p_scan->white_max_step))
    {
        p_scan->white_max_step = step;
    }
    if (step > 0U)
    {
        p_scan->moved_usec = now_usec;
    }

    if ((BC_SCANNING == p_scan->state) ||
        (BC_TRAILING_WHITE == p_scan->state))
    {
        uint32_t * const p_max = (BC_SCANNING == p_scan->state) ?
            &p_scan->barcode_max_step : &p_scan->trailing_max_step;

        if (step > *p_max)
        {
            *p_max = step;
        }
        if (step > p_scan->max_step_units)
        {
            p_scan->max_step_units = step;
        }
        if ((now_usec - p_scan->start_usec) > p_scan->config.timeout_usec)
        {
            reject_scan(p_scan, BC_ERR_TIMEOUT);
            return false;
        }
        if ((now_usec - p_scan->moved_usec) > p_scan->config.stop_usec)
        {
            reject_scan(p_scan, BC_ERR_STOPPED);
            return false;
        }
        if ((p_scan->position - p_scan->run_start) > UINT32_MAX)
        {
            reject_scan(p_scan, BC_ERR_OVERFLOW);
            return false;
        }
    }

    if (raw == p_scan->stable_level)
    {
        if ((p_scan->candidate_samples > 0U) &&
            ((BC_SCANNING == p_scan->state) ||
             (BC_TRAILING_WHITE == p_scan->state)))
        {
            p_scan->unconfirmed_edges++;
        }
        p_scan->candidate_samples = 0U;
    }
    else
    {
        if ((0U == p_scan->candidate_samples) ||
            (raw != p_scan->candidate_level))
        {
            p_scan->candidate_level = raw;
            p_scan->candidate_position = p_scan->position;
            p_scan->candidate_usec = now_usec;
            p_scan->candidate_samples = 1U;
            p_scan->candidate_step_units = step;
        }
        else
        {
            p_scan->candidate_samples++;
            if (step > p_scan->candidate_step_units)
            {
                p_scan->candidate_step_units = step;
            }
        }

        if (p_scan->candidate_samples >= BC_CONFIRM_SAMPLES)
        {
            uint32_t const from_level = p_scan->stable_level;
            uint32_t const confirm_usec = now_usec - p_scan->candidate_usec;
            uint32_t const confirm_units =
                (uint32_t)(p_scan->position - p_scan->candidate_position);

            if (confirm_usec > p_scan->max_confirm_usec)
            {
                p_scan->max_confirm_usec = confirm_usec;
            }
            if (confirm_units > p_scan->max_confirm_units)
            {
                p_scan->max_confirm_units = confirm_units;
            }
            p_scan->stable_level = raw;
            p_scan->candidate_samples = 0U;
            if ((BC_WAIT_WHITE == p_scan->state) ||
                (BC_ERROR == p_scan->state))
            {
                p_scan->max_step_units = p_scan->candidate_step_units;
                if (p_scan->white_max_step > p_scan->max_step_units)
                {
                    p_scan->max_step_units = p_scan->white_max_step;
                }
            }
            if (transition(p_scan, p_scan->candidate_position,
                           p_scan->candidate_usec, from_level))
            {
                return true;
            }
        }
    }

    if ((BC_TRAILING_WHITE == p_scan->state) &&
        (0U == p_scan->candidate_samples) &&
        ((p_scan->position - p_scan->run_start) >= p_scan->quiet_units))
    {
        return finish_scan(p_scan);
    }

    return false;
}

static int64_t count_delta (int32_t current, int32_t previous)
{
    uint32_t const delta = (uint32_t)current - (uint32_t)previous;

    return (delta <= INT32_MAX) ? (int64_t)delta :
                                (int64_t)delta - INT64_C(4294967296);
}

static bool skew_exceeded (bc_scan_t const * p_scan, uint64_t left,
                            uint64_t right)
{
    uint64_t const difference = (left >= right) ? left - right : right - left;
    uint64_t const allowance =
        ((uint64_t)p_scan->config.max_skew_units * 100U) +
        ((left + right) * p_scan->config.max_skew_percent);

    return (difference * 100U) > allowance;
}

static bool motion_valid (bc_scan_t * p_scan, uint64_t left, uint64_t right)
{
    if (skew_exceeded(p_scan, left, right))
    {
        return false;
    }

    p_scan->motion_left_units += left;
    p_scan->motion_right_units += right;

    /* Accumulate enough counts to distinguish pulse timing from a pivot.
     * A one-wheel move of only two counts fits the per-sample floor, but
     * repeating it must not permit a complete stationary-wheel scan.
     * The window and one final sample fit safely in uint64_t arithmetic.
     */
    uint64_t window_units =
        ((uint64_t)p_scan->config.max_skew_units * 4U) + 4U;
    uint64_t const pivot_units =
        (((uint64_t)p_scan->config.max_skew_units * 100U) /
         (100U - p_scan->config.max_skew_percent)) + 1U;

    if (pivot_units > window_units)
    {
        window_units = pivot_units;
    }

    if ((p_scan->motion_left_units + p_scan->motion_right_units) >=
        window_units)
    {
        bool const b_valid = !skew_exceeded(p_scan,
            p_scan->motion_left_units, p_scan->motion_right_units);

        p_scan->motion_left_units = 0U;
        p_scan->motion_right_units = 0U;
        return b_valid;
    }

    return true;
}

static void reject_scan (bc_scan_t * p_scan, bc_status_t status)
{
    p_scan->state = BC_ERROR;
    p_scan->status = status;
    p_scan->b_seeded = false;
    p_scan->candidate_samples = 0U;
    p_scan->motion_left_units = 0U;
    p_scan->motion_right_units = 0U;
}

static bool finish_scan (bc_scan_t * p_scan)
{
    if (skew_exceeded(p_scan, p_scan->motion_left_units,
                      p_scan->motion_right_units))
    {
        reject_scan(p_scan, BC_ERR_SKEW);
        return false;
    }

    /* A jump across the quiet zone must not hide unobserved elements. */
    bc_status_t const status = bc_decode_inspect(p_scan->runs,
        p_scan->max_step_units, &p_scan->decoded, &p_scan->decode_report);

    if (BC_OK != status)
    {
        reject_scan(p_scan, status);
        return false;
    }

    p_scan->state = BC_COMPLETE;
    return true;
}

static bool transition (bc_scan_t * p_scan, uint64_t edge_position,
                         uint32_t edge_usec, uint32_t from_level)
{
    bool const b_black = p_scan->stable_level == p_scan->config.black_level;

    if ((BC_WAIT_WHITE == p_scan->state) || (BC_ERROR == p_scan->state))
    {
        if (!b_black)
        {
            p_scan->white_start = edge_position;
            p_scan->white_max_step = p_scan->candidate_step_units;
        }
        else
        {
            uint64_t const white = edge_position - p_scan->white_start;

            if (white > 0U)
            {
                p_scan->leading_units = (white > UINT32_MAX) ?
                                         UINT32_MAX : (uint32_t)white;
                p_scan->run_start = edge_position;
                p_scan->start_usec = edge_usec;
                p_scan->run_count = 0U;
                p_scan->decode_report = (bc_decode_report_t){0};
                p_scan->decode_report.failed_index = BC_RUN_COUNT;
                p_scan->approach_max_step = p_scan->white_max_step;
                p_scan->barcode_max_step = p_scan->candidate_step_units;
                p_scan->trailing_max_step = 0U;
                p_scan->start_level = p_scan->stable_level;
                p_scan->start_from_level = from_level;
                p_scan->max_confirm_usec = p_scan->last_usec - edge_usec;
                p_scan->max_confirm_units =
                    (uint32_t)(p_scan->position - edge_position);
                p_scan->unconfirmed_edges = 0U;
                p_scan->status = BC_OK;
                p_scan->state = BC_SCANNING;
            }
        }
        return false;
    }

    uint64_t const width = edge_position - p_scan->run_start;

    if (width > UINT32_MAX)
    {
        reject_scan(p_scan, BC_ERR_OVERFLOW);
        return false;
    }

    p_scan->last_run_units = (uint32_t)width;
    if (BC_TRAILING_WHITE == p_scan->state)
    {
        if (width >= p_scan->quiet_units)
        {
            return finish_scan(p_scan);
        }
        /* Extra elements/multiple data letters do not fit this format. */
        reject_scan(p_scan, BC_ERR_OVERFLOW);
        return false;
    }

    if (p_scan->run_count >= BC_RUN_COUNT)
    {
        reject_scan(p_scan, BC_ERR_OVERFLOW);
        return false;
    }
    p_scan->runs[p_scan->run_count] = (uint32_t)width;
    p_scan->run_count++;
    p_scan->run_start = edge_position;

    if (BC_RUN_COUNT == p_scan->run_count)
    {
        bc_status_t const status = bc_decode_inspect(p_scan->runs,
            p_scan->max_step_units, &p_scan->decoded, &p_scan->decode_report);

        if (BC_OK != status)
        {
            reject_scan(p_scan, status);
        }
        else if (p_scan->decoded.narrow_units >
                 (UINT32_MAX / BC_QUIET_MULTIPLE))
        {
            reject_scan(p_scan, BC_ERR_OVERFLOW);
        }
        else
        {
            p_scan->quiet_units =
                p_scan->decoded.narrow_units * BC_QUIET_MULTIPLE;
            if (p_scan->leading_units < p_scan->quiet_units)
            {
                reject_scan(p_scan, BC_ERR_QUIET);
            }
            else
            {
                p_scan->state = BC_TRAILING_WHITE;
            }
        }
    }

    return false;
}

/*** end of file ***/

