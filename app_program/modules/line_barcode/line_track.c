/** @file line_track.c
 * @brief Conservative bounded line-loss and cross-junction classification.
 */
#include "line_track.h"
#include <stddef.h>

#define LT_JUNCTION_SAMPLES 4U
#define LT_JUNCTION_HOLDOFF_MSEC 250U
#define LT_RECOVERY_TIMEOUT_MSEC 600U
#define LT_RECOVERY_STABLE_SAMPLES 3U

void line_track_init (line_track_t * p_track)
{
    if (NULL != p_track)
    {
        *p_track = (line_track_t){0};
    }
}

void line_track_recovery (line_track_t * p_track,
                           bool b_recovery, uint32_t now_msec)
{
    if (NULL != p_track)
    {
        p_track->b_recovering = b_recovery ? 1U : 0U;
        p_track->recovery_count = 0U;
        p_track->recovery_start_msec = now_msec;
    }
}

line_track_state_t line_track_step_distance (line_track_t * p_track,
                                             uint8_t bits, uint32_t now_msec,
                                             uint32_t travel_ticks,
                                             uint32_t min_junction_ticks,
                                             bool b_encoder_valid,
                                             bool * p_b_junction_event)
{
    if (NULL != p_b_junction_event)
    {
        *p_b_junction_event = false;
    }
    if ((NULL == p_track) || (NULL == p_b_junction_event) ||
        (0U != (bits & 0xFCU)))
    {
        return LT_FAULT;
    }
    if (0U != p_track->b_recovering)
    {
        if ((now_msec - p_track->recovery_start_msec) >
            LT_RECOVERY_TIMEOUT_MSEC)
        {
            p_track->state = LT_FAULT;
            return LT_FAULT;
        }
        if ((1U == bits) || (2U == bits))
        {
            if (p_track->recovery_count < LT_RECOVERY_STABLE_SAMPLES)
            {
                ++p_track->recovery_count;
            }
            if (p_track->recovery_count == LT_RECOVERY_STABLE_SAMPLES)
            {
                p_track->b_recovering = 0U;
                p_track->state = (1U == bits) ? LT_LEFT : LT_RIGHT;
                p_track->last_side = bits;
                return (line_track_state_t)p_track->state;
            }
        }
        else
        {
            p_track->recovery_count = 0U;
        }
        p_track->state = LT_SEARCH;
        return LT_SEARCH;
    }
    if (3U == bits)
    {
        if (p_track->junction_count < LT_JUNCTION_SAMPLES)
        {
            ++p_track->junction_count;
        }
        if ((p_track->junction_count == 1U) && b_encoder_valid)
        {
            p_track->junction_start_ticks = travel_ticks;
            p_track->b_have_junction_start = 1U;
        }
        if ((p_track->junction_count >= LT_JUNCTION_SAMPLES) &&
            (0U == p_track->b_junction_latched) &&
            b_encoder_valid && (min_junction_ticks > 0U) &&
            (0U != p_track->b_have_junction_start) &&
            (travel_ticks >= p_track->junction_start_ticks) &&
            ((travel_ticks - p_track->junction_start_ticks) >=
              min_junction_ticks) &&
            ((0U == p_track->last_junction_msec) ||
             ((now_msec - p_track->last_junction_msec) >
               LT_JUNCTION_HOLDOFF_MSEC)))
        {
            *p_b_junction_event = true;
            p_track->b_junction_latched = 1U;
            p_track->last_junction_msec = now_msec;
        }
        p_track->state = (p_track->junction_count <
                          LT_JUNCTION_SAMPLES) ? LT_SEARCH : LT_JUNCTION;
        return (line_track_state_t)p_track->state;
    }
    p_track->junction_count = 0U;
    p_track->b_junction_latched = 0U;
    p_track->b_have_junction_start = 0U;
    if (1U == bits)
    {
        p_track->state = LT_LEFT;
        p_track->last_side = 1U;
    }
    else if (2U == bits)
    {
        p_track->state = LT_RIGHT;
        p_track->last_side = 2U;
    }
    else
    {
        /* White/white can mean centred or lost; cannot disambiguate. */
        p_track->state = LT_FORWARD;
    }
    return (line_track_state_t)p_track->state;
}

line_track_state_t line_track_step (line_track_t * p_track,
                                    uint8_t bits, uint32_t now_msec,
                                    bool * p_b_junction_event)
{
    /* Compatibility wrapper does not assert a movement-confirmed junction. */
    return line_track_step_distance(p_track, bits, now_msec,
                                     0U, 1U, false, p_b_junction_event);
}

/*** end of file ***/

