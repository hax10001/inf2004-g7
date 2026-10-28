/** @file bc_capture.c
 * @brief Barcode boundaries confirmed by star pattern and travel quiet zone.
 */
#include "bc_capture.h"
#include <stddef.h>
#include <stdint.h>

#define BC_PHASE_QUIET 0U
#define BC_PHASE_SCANNING 1U
#define BC_PHASE_TRAILING 2U

void bc_capture_cancel (bc_capture_t * p_capture)
{
    if (NULL != p_capture)
    {
        p_capture->phase = BC_PHASE_QUIET;
        p_capture->run_count = 0U;
        p_capture->b_have_last = 0U;
    }
}

bool bc_capture_init (bc_capture_t * p_capture,
                      bc_capture_config_t const * p_config)
{
    if ((NULL == p_capture) || (NULL == p_config) ||
        (0U == p_config->min_run_ticks) ||
        (p_config->quiet_ticks <= p_config->min_run_ticks) ||
        (p_config->max_scan_ticks <= p_config->quiet_ticks) ||
        (0U == p_config->timeout_msec) ||
        (p_config->min_run_ticks > (UINT32_MAX / 4U)) ||
        (p_config->quiet_ticks <=
         (p_config->min_run_ticks * 4U)) ||
        (p_config->max_scan_ticks > (UINT32_MAX / 3U)) ||
        (0U == p_config->junction_min_ticks) ||
        (p_config->pending_expiry_ticks <=
         p_config->max_scan_ticks))
    {
        return false;
    }
    *p_capture = (bc_capture_t){0};
    p_capture->config = *p_config;
    return true;
}

static bc_capture_event_t restart_quiet (bc_capture_t * p_capture,
                                         uint32_t ticks, bool b_black)
{
    bc_capture_cancel(p_capture);
    p_capture->last_ticks = ticks;
    p_capture->last_black = b_black ? 1U : 0U;
    p_capture->b_have_last = 1U;
    p_capture->quiet_start_ticks = ticks;
    return BC_REJECTED;
}

bc_capture_event_t bc_capture_step (bc_capture_t * p_capture,
                                    bool b_black, uint32_t ticks,
                                    uint32_t msec, bool b_encoder_valid,
                                    bc_decode_result_t * p_result)
{
    uint32_t width = 0U;

    if ((NULL == p_capture) || (NULL == p_result))
    {
        return BC_REJECTED;
    }
    *p_result = (bc_decode_result_t){BC_INVALID_LETTER, 0U,
                                     false, false};
    if (!b_encoder_valid)
    {
        return restart_quiet(p_capture, ticks, b_black);
    }
    if (0U == p_capture->b_have_last)
    {
        p_capture->last_ticks = ticks;
        p_capture->quiet_start_ticks = ticks;
        p_capture->last_black = b_black ? 1U : 0U;
        p_capture->b_have_last = 1U;
        return BC_PENDING;
    }
    /* An external producer must reset on wrap, direction change or reset. */
    if (ticks < p_capture->last_ticks)
    {
        return restart_quiet(p_capture, ticks, b_black);
    }
    if (BC_PHASE_QUIET == p_capture->phase)
    {
        if (!b_black)
        {
            if (0U != p_capture->last_black)
            {
                p_capture->quiet_start_ticks = ticks;
            }
        }
        else if ((0U == p_capture->last_black) &&
                 ((ticks - p_capture->quiet_start_ticks) >=
                   p_capture->config.quiet_ticks))
        {
            p_capture->phase = BC_PHASE_SCANNING;
            p_capture->start_ticks = ticks;
            p_capture->start_msec = msec;
            p_capture->last_ticks = ticks;
        }
        p_capture->last_black = b_black ? 1U : 0U;
        return BC_PENDING;
    }

    if (((msec - p_capture->start_msec) >
         p_capture->config.timeout_msec) ||
        ((ticks - p_capture->start_ticks) >
         p_capture->config.max_scan_ticks))
    {
        return restart_quiet(p_capture, ticks, b_black);
    }
    if (BC_PHASE_TRAILING == p_capture->phase)
    {
        if (b_black)
        {
            return restart_quiet(p_capture, ticks, b_black);
        }
        if ((ticks - p_capture->last_ticks) >=
            p_capture->config.quiet_ticks)
        {
            *p_result = bc_decode_runs(p_capture->runs,
                                       p_capture->config.min_run_ticks);
            bc_capture_cancel(p_capture);
            p_capture->last_black = 0U;
            p_capture->last_ticks = ticks;
            p_capture->quiet_start_ticks = ticks;
            p_capture->b_have_last = 1U;
            return p_result->b_valid ? BC_COMPLETE : BC_REJECTED;
        }
        return BC_PENDING;
    }
    if (b_black == (0U != p_capture->last_black))
    {
        return BC_PENDING;
    }
    width = ticks - p_capture->last_ticks;
    if ((width < p_capture->config.min_run_ticks) ||
        (p_capture->run_count >= BC_RUN_COUNT))
    {
        return restart_quiet(p_capture, ticks, b_black);
    }
    p_capture->runs[p_capture->run_count] = width;
    ++p_capture->run_count;
    p_capture->last_ticks = ticks;
    p_capture->last_black = b_black ? 1U : 0U;
    if (BC_RUN_COUNT == p_capture->run_count)
    {
        if (b_black)
        {
            return restart_quiet(p_capture, ticks, b_black);
        }
        p_capture->phase = BC_PHASE_TRAILING;
    }
    return BC_PENDING;
}

/*** end of file ***/

