/** @file nav_core.c
 * @brief Pure bounded navigation state transitions, no hardware access.
 */
#include "nav_core.h"
#include <stddef.h>

void nav_init(nav_controller_t *nav, const nav_config_t *cfg)
{
    if (nav == NULL || cfg == NULL) return;
    *nav = (nav_controller_t){0};
    nav->cfg = *cfg;
    nav->state = NAV_WAIT;
}
void nav_stop(nav_controller_t *nav)
{
    if (nav == NULL) return;
    nav->state = NAV_WAIT;
    nav->pending = 0U;
    nav->black_since = 0U;
    nav->black_from_counts = 0U;
    nav->edge_since = 0U;
    nav->edge_seen = 0U;
}
void nav_start(nav_controller_t *nav, uint32_t now_msec)
{
    if (nav == NULL || nav->state == NAV_FAULT) return;
    nav->state = NAV_FOLLOW;
    nav->pending = 0U;
    nav->black_since = 0U;
    nav->black_from_counts = 0U;
    nav->edge_seen = 0U;
    nav->turn_announced = 0U;
    nav->resume_announced = 0U;
    nav->edge_since = 0U;
    nav->state_since = now_msec;
}
int nav_barcode(nav_controller_t *nav, uint8_t letter, uint8_t action,
                uint16_t seq, uint32_t ms, uint32_t traveled)
{
    if (nav == NULL || letter > 25U || action > 3U) return -2;
    if (nav->state != NAV_FOLLOW && nav->state != NAV_APPROACH)
        return -2;
    if (nav->last_barcode_seq == seq && seq != 0U) return 0;
    /* Reject duplicate readings of the same physical printed card until
     * there is a measured displacement. Suppression is deliberately small
     * and needs calibration against real barcode/track geometry. */
    if (nav->last_barcode_seq != 0U && nav->letter == letter &&
        (uint32_t)(ms - nav->last_barcode_ms) < 1500U &&
        (uint32_t)(traveled - nav->last_barcode_counts) < 120U)
        return 0;
    if (nav->pending != 0U) return -1;
    nav->pending = 1U;
    nav->letter = letter;
    nav->action = action;
    nav->barcode_counts = traveled;
    nav->last_barcode_counts = traveled;
    nav->last_barcode_ms = ms;
    nav->last_barcode_seq = seq;
    nav->state = NAV_APPROACH;
    nav->black_since = 0U;
    nav->black_from_counts = 0U;
    return 1;
}
nav_result_t nav_step(nav_controller_t *nav, const nav_sample_t *s)
{
    nav_result_t result = {NAV_OUTPUT_STOP, NAV_EVENT_NONE, 0U};
    uint8_t both, edge;
    if (nav == NULL || s == NULL) return result;
    if (nav->state == NAV_WAIT || nav->state == NAV_FAULT) return result;
    both = (uint8_t)(s->left_black && s->right_black);
    edge = (uint8_t)((s->left_black != s->right_black) && !both);
    if (nav->state == NAV_FOLLOW || nav->state == NAV_APPROACH)
    {
        result.drive = NAV_OUTPUT_FOLLOW;
        if (nav->resume_announced)
        {
            nav->resume_announced = 0U;
            result.event = NAV_EVENT_FOLLOWING;
            result.value = nav->action;
        }
        if (both)
        {
            uint32_t elapsed;
            uint32_t travel;
            if (nav->black_since == 0U)
            {
                nav->black_since = s->now_msec;
                nav->black_from_counts = s->travel_counts;
            }
            elapsed = (uint32_t)(s->now_msec - nav->black_since);
            travel = (s->travel_counts >= nav->black_from_counts) ?
                (s->travel_counts - nav->black_from_counts) : UINT32_MAX;
            /* The barcode/line sensors can overlap at a printed symbol.
             * A scan attempt (including one that FAILS decoding) may NOT
             * become a junction fault. Cross that ambiguous stripe only
             * while the scanner says a capture is in progress/recent, and
             * only inside BOTH time and encoder distance limits. Beyond
             * either limit we stop safely; never blindly cross a junction.
             */
            if (s->barcode_capturing &&
                elapsed < nav->cfg.scan_cross_max_msec &&
                travel < nav->cfg.scan_cross_max_counts)
            {
                result.drive = NAV_OUTPUT_FORWARD;
                return result;
            }
            if (elapsed >= nav->cfg.confirm_msec ||
                (s->barcode_capturing &&
                 (elapsed >= nav->cfg.scan_cross_max_msec ||
                  travel >= nav->cfg.scan_cross_max_counts)))
            {
                if (!nav->pending || s->travel_counts < nav->barcode_counts ||
                    (uint32_t)(s->travel_counts - nav->barcode_counts) <
                    nav->cfg.min_after_barcode_counts || s->barcode_capturing)
                {
                    nav->state = NAV_FAULT;
                    nav->fault_reason = 1U;
                    result.drive = NAV_OUTPUT_STOP;
                    result.event = NAV_EVENT_FAULT;
                    result.value = 1U;
                }
                else
                {
                    nav->state = (nav->action == NAV_STRAIGHT) ?
                        NAV_CROSS : NAV_TURN;
                    nav->state_since = s->now_msec;
                    nav->edge_seen = 0U;
                    nav->turn_announced = 0U;
                    nav->edge_since = 0U;
                    result.drive = (nav->state == NAV_CROSS) ?
                        NAV_OUTPUT_FORWARD :
                        (nav_drive_t)(NAV_OUTPUT_LEFT + nav->action - 1U);
                    result.event = NAV_EVENT_JUNCTION;
                    result.value = nav->action;
                }
            }
        }
        else
        {
            nav->black_since = 0U;
            nav->black_from_counts = 0U;
        }
        return result;
    }
    if (nav->state == NAV_CROSS)
    {
        result.drive = NAV_OUTPUT_FORWARD;
        if ((uint32_t)(s->now_msec - nav->state_since) >
            nav->cfg.max_cross_msec)
        {
            nav->state = NAV_FAULT;
            nav->fault_reason = 3U;
            result.drive = NAV_OUTPUT_STOP;
            result.event = NAV_EVENT_FAULT;
            result.value = 3U;
        }
        else if (!both && (uint32_t)(s->now_msec - nav->state_since) >=
                 nav->cfg.min_turn_msec)
        {
            if (nav->edge_since == 0U) nav->edge_since = s->now_msec;
            if ((uint32_t)(s->now_msec - nav->edge_since) >=
                nav->cfg.confirm_msec)
            {
                nav->pending = 0U;
                nav->state = NAV_FOLLOW;
                nav->black_since = 0U;
                result.drive = NAV_OUTPUT_FOLLOW;
                result.event = NAV_EVENT_FOLLOWING;
                result.value = NAV_STRAIGHT;
            }
        }
        else nav->edge_since = 0U;
        return result;
    }
    if (nav->state == NAV_TURN || nav->state == NAV_REACQUIRE)
    {
        result.drive = (nav_drive_t)(NAV_OUTPUT_LEFT + nav->action - 1U);
        if (!nav->turn_announced)
        {
            nav->turn_announced = 1U;
            result.event = NAV_EVENT_TURN_STARTED;
            result.value = nav->action;
        }
        if ((uint32_t)(s->now_msec - nav->state_since) >
            nav->cfg.max_turn_msec)
        {
            nav->state = NAV_FAULT;
            nav->fault_reason = 2U;
            result.drive = NAV_OUTPUT_STOP;
            result.event = NAV_EVENT_FAULT;
            result.value = 2U;
        }
        else if ((uint32_t)(s->now_msec - nav->state_since) >=
                 nav->cfg.min_turn_msec && edge)
        {
            if (nav->edge_since == 0U) nav->edge_since = s->now_msec;
            if ((uint32_t)(s->now_msec - nav->edge_since) >=
                nav->cfg.confirm_msec)
            {
                nav->state = NAV_RESUME;
                nav->pending = 0U;
                nav->black_since = 0U;
                result.drive = NAV_OUTPUT_FOLLOW;
                result.event = NAV_EVENT_TURN_COMPLETED;
                result.value = nav->action;
            }
            else nav->state = NAV_REACQUIRE;
        }
        else nav->edge_since = 0U;
        return result;
    }
    if (nav->state == NAV_RESUME)
    {
        nav->state = NAV_FOLLOW;
        nav->resume_announced = 1U;
        result.drive = NAV_OUTPUT_FOLLOW;
        result.event = NAV_EVENT_LINE_REACQUIRED;
        result.value = nav->action;
        return result;
    }
    nav->state = NAV_FAULT;
    result.event = NAV_EVENT_FAULT;
    result.value = 4U;
    return result;
}

/*** end of file ***/

