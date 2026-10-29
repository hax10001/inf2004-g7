/** @file bc_scan.c
 * @brief Distance-bounded barcode start, runs and trailing-zone capture.
 */
#include "bc_scan.h"

static uint8_t bc_scan_reject (bc_scan_t * p_scan, uint8_t error,
                               uint32_t now_msec)
{
    p_scan->state = BC_SCAN_INVALID;
    p_scan->error = error;
    p_scan->result_msec = now_msec;
    return BC_SCAN_EVENT_REJECTED;
}

uint8_t bc_scan_init (bc_scan_t * p_scan,
                      bc_scan_config_t const * p_config)
{
    if ((0 == p_scan) || (0 == p_config) ||
        (0U == p_config->min_run_units) ||
        (p_config->leading_units <= p_config->min_run_units) ||
        (p_config->min_run_units > UINT32_MAX / 5U) ||
        (p_config->trailing_units <= p_config->min_run_units * 4U) ||
        (p_config->max_barcode_units >
         UINT32_MAX - p_config->trailing_units) ||
        (p_config->min_barcode_units <
         p_config->min_run_units * BC_CODE_ELEMENTS) ||
        (p_config->max_barcode_units <
         p_config->min_barcode_units) ||
        (0U == p_config->timeout_msec))
    {
        return 0U;
    }

    *p_scan = (bc_scan_t){0};
    p_scan->cfg = *p_config;
    return 1U;
}

void bc_scan_interrupt (bc_scan_t * p_scan, uint32_t now_msec)
{
    if ((0 != p_scan) &&
        ((BC_SCAN_CAPTURING == p_scan->state) ||
         (BC_SCAN_END_CANDIDATE == p_scan->state)))
    {
        (void)bc_scan_reject(p_scan, BC_SCAN_ERROR_INTERRUPTED,
                             now_msec);
    }
}

uint8_t bc_scan_feed (bc_scan_t * p_scan, uint8_t b_black,
                      uint32_t travel_units, uint32_t now_msec,
                      uint8_t b_valid)
{
    uint32_t width;

    if (0 == p_scan)
    {
        return BC_SCAN_EVENT_REJECTED;
    }

    if (BC_SCAN_COMPLETE == p_scan->state)
    {
        return BC_SCAN_EVENT_READY;
    }

    if (BC_SCAN_INVALID == p_scan->state)
    {
        return BC_SCAN_EVENT_REJECTED;
    }

    if ((b_black > 1U) || (b_valid > 1U) || (0U == b_valid))
    {
        return bc_scan_reject(p_scan, BC_SCAN_ERROR_INPUT, now_msec);
    }

    if ((BC_SCAN_IDLE != p_scan->state) &&
        (travel_units < p_scan->last_edge_units))
    {
        return bc_scan_reject(p_scan, BC_SCAN_ERROR_DISTANCE,
                              now_msec);
    }

    if (BC_SCAN_IDLE == p_scan->state)
    {
        if (0U == b_black)
        {
            p_scan->state = BC_SCAN_START_CANDIDATE;
            p_scan->first_white_units = travel_units;
            p_scan->last_edge_units = travel_units;
        }
        return BC_SCAN_EVENT_NONE;
    }

    if (BC_SCAN_START_CANDIDATE == p_scan->state)
    {
        if (0U != b_black)
        {
            if ((travel_units - p_scan->first_white_units) >=
                p_scan->cfg.leading_units)
            {
                p_scan->state = BC_SCAN_CAPTURING;
                p_scan->run_count = 0U;
                p_scan->last_black = 1U;
                p_scan->start_units = travel_units;
                p_scan->start_msec = now_msec;
                p_scan->last_edge_units = travel_units;
            }
            else
            {
                p_scan->state = BC_SCAN_IDLE;
            }
        }
        return BC_SCAN_EVENT_NONE;
    }

    if ((uint32_t)(now_msec - p_scan->start_msec) >
        p_scan->cfg.timeout_msec)
    {
        return bc_scan_reject(p_scan, BC_SCAN_ERROR_TIMEOUT, now_msec);
    }

    if ((travel_units - p_scan->start_units) >
        p_scan->cfg.max_barcode_units + p_scan->cfg.trailing_units)
    {
        return bc_scan_reject(p_scan, BC_SCAN_ERROR_DISTANCE,
                              now_msec);
    }

    if (BC_SCAN_END_CANDIDATE == p_scan->state)
    {
        if (0U != b_black)
        {
            return bc_scan_reject(p_scan, BC_SCAN_ERROR_OVERFLOW,
                                  now_msec);
        }
        if ((travel_units - p_scan->last_edge_units) >=
            p_scan->cfg.trailing_units)
        {
            p_scan->state = BC_SCAN_COMPLETE;
            p_scan->result_msec = now_msec;
            return BC_SCAN_EVENT_READY;
        }
        return BC_SCAN_EVENT_NONE;
    }

    if (b_black == p_scan->last_black)
    {
        return BC_SCAN_EVENT_NONE;
    }

    width = travel_units - p_scan->last_edge_units;

    if (width < p_scan->cfg.min_run_units)
    {
        return bc_scan_reject(p_scan, BC_SCAN_ERROR_WIDTH, now_msec);
    }

    if (p_scan->run_count >= BC_CODE_ELEMENTS)
    {
        return bc_scan_reject(p_scan, BC_SCAN_ERROR_OVERFLOW,
                              now_msec);
    }

    p_scan->widths[p_scan->run_count] = width;
    ++p_scan->run_count;
    p_scan->last_edge_units = travel_units;
    p_scan->last_black = b_black;

    if (BC_CODE_ELEMENTS == p_scan->run_count)
    {
        uint32_t length = travel_units - p_scan->start_units;

        if ((0U != b_black) ||
            (length < p_scan->cfg.min_barcode_units) ||
            (length > p_scan->cfg.max_barcode_units))
        {
            return bc_scan_reject(p_scan, BC_SCAN_ERROR_DISTANCE,
                                  now_msec);
        }
        p_scan->state = BC_SCAN_END_CANDIDATE;
    }

    return BC_SCAN_EVENT_NONE;
}

uint8_t bc_scan_take (bc_scan_t * p_scan, uint32_t * p_width,
                      uint8_t * p_error, uint32_t * p_timestamp)
{
    uint8_t event = BC_SCAN_EVENT_NONE;

    if ((0 == p_scan) || (0 == p_width) || (0 == p_error) ||
        (0 == p_timestamp))
    {
        return event;
    }

    if (BC_SCAN_COMPLETE == p_scan->state)
    {
        for (uint8_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
        {
            p_width[i] = p_scan->widths[i];
        }
        event = BC_SCAN_EVENT_READY;
    }
    else if (BC_SCAN_INVALID == p_scan->state)
    {
        event = BC_SCAN_EVENT_REJECTED;
    }

    if (BC_SCAN_EVENT_NONE != event)
    {
        *p_error = p_scan->error;
        *p_timestamp = p_scan->result_msec;
        /* Require a new uninterrupted leading white region after take. */
        p_scan->state = BC_SCAN_IDLE;
        p_scan->run_count = 0U;
        p_scan->error = BC_SCAN_ERROR_NONE;
    }

    return event;
}

/*** end of file ***/

