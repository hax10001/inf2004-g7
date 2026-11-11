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
    nav->decode_hold = 0U;
}
void nav_forget(nav_controller_t *nav)
{
    if (nav == NULL) return;
    nav_stop(nav);
    nav->held = 0U;
    nav->fault_reason = 0U;
    nav->hold_letter = 0U;
    nav->last_barcode_seq = 0U;
    nav->have_barcode = 0U;
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
    nav->decode_hold = 0U;
    nav->state_since = now_msec;
    /* Repeat mode: the action before an obstacle bypass carries on. Its
     * barcode is long passed, so no barcode clearance applies to it. */
    if (nav->cfg.repeat_action && nav->held)
    {
        nav->pending = 1U;
        nav->letter = nav->held_letter;
        nav->action = nav->held_action;
        nav->barcode_counts = 0U;
    }
}
/* A junction action finished. Single mode: it is used up. Repeat mode:
 * it stays queued for the next junction until a new barcode replaces it. */
static void nav_action_done(nav_controller_t *nav)
{
    if (!(nav->cfg.repeat_action && nav->held)) nav->pending = 0U;
}
int nav_barcode(nav_controller_t *nav, uint8_t letter, uint8_t action,
                uint16_t seq, uint32_t ms, uint32_t traveled)
{
    if (nav == NULL || letter > 25U || action > 3U) return -2;
    if (nav->state != NAV_FOLLOW && nav->state != NAV_APPROACH)
        return -2;
    /* A result consumed after the hard deadline cannot rescue that wait.
     * At the boundary itself, a valid result wins before nav_step. */
    if (nav->decode_hold &&
        (int32_t)(ms - (nav->hold_since + nav->cfg.decode_wait_msec)) > 0)
        return -3;
    if (nav->have_barcode && nav->last_barcode_seq == seq) return 0;
    /* Reject duplicate readings of the same physical printed card until
     * there is a measured displacement. Suppression is deliberately small
     * and needs calibration against real barcode/track geometry. */
    if (nav->have_barcode && nav->letter == letter &&
        (uint32_t)(ms - nav->last_barcode_ms) < 1500U &&
        (uint32_t)(traveled - nav->last_barcode_counts) < 120U)
        return 0;
    /* Single mode keeps the first queued letter; repeat mode replaces the
     * remembered action with the newest barcode. */
    if (nav->pending != 0U && !nav->cfg.repeat_action) return -1;
    nav->pending = 1U;
    if (nav->decode_hold) nav->hold_letter = 1U;
    if (nav->cfg.repeat_action)
    {
        nav->held = 1U;
        nav->held_letter = letter;
        nav->held_action = action;
    }
    nav->letter = letter;
    nav->action = action;
    nav->barcode_counts = traveled;
    nav->last_barcode_counts = traveled;
    nav->last_barcode_ms = ms;
    nav->last_barcode_seq = seq;
    nav->have_barcode = 1U;
    nav->state = NAV_APPROACH;
    /* Stopped at a junction waiting for this letter: keep where its bar
     * began, the turn's drive to the centre is measured from there. */
    if (!nav->decode_hold && nav->black_since == 0U)
    {
        nav->black_since = 0U;
        nav->black_from_counts = 0U;
    }
    return 1;
}
/* Pivot output for the queued turn (left, right or U-turn). */
static nav_drive_t nav_turn_drive(const nav_controller_t *nav)
{
    return (nav_drive_t)(NAV_OUTPUT_LEFT + nav->action - 1U);
}
/* Start pivoting from here; the angle is measured from turn_counts. */
static void nav_begin_turn(nav_controller_t *nav, const nav_sample_t *s,
                           nav_result_t *result)
{
    nav->state = NAV_TURN;
    nav->state_since = s->now_msec;
    nav->turn_from = s->turn_counts;
    nav->turn_announced = 1U;
    nav->edge_since = 0U;
    result->drive = nav_turn_drive(nav);
    result->event = NAV_EVENT_TURN_STARTED;
    result->value = nav->action;
}
/* Confirmed junction with a letter queued: start the mapped manoeuvre.
 * Straight drives over the cross. A turn first drives on to the junction
 * centre (when configured) so the pivot lands the sensors on the new arm.
 */
static void nav_take_junction(nav_controller_t *nav, uint32_t now,
                              nav_result_t *result)
{
    if (nav->action == NAV_STRAIGHT) nav->state = NAV_CROSS;
    else if (nav->cfg.centre_counts != 0U) nav->state = NAV_CENTRE;
    else nav->state = NAV_TURN;
    nav->centre_from = nav->black_from_counts;
    nav->state_since = now;
    nav->edge_seen = 0U;
    nav->turn_announced = 0U;
    nav->edge_since = 0U;
    result->drive = (nav->state == NAV_TURN) ? nav_turn_drive(nav) :
        NAV_OUTPUT_FORWARD;
    result->event = NAV_EVENT_JUNCTION;
    result->value = nav->action;
}
static void nav_fault(nav_controller_t *nav, uint8_t reason,
                      nav_result_t *result)
{
    nav->state = NAV_FAULT;
    nav->fault_reason = reason;
    nav->decode_hold = 0U;
    result->drive = NAV_OUTPUT_STOP;
    result->event = NAV_EVENT_FAULT;
    result->value = reason;
}
/* Stopped at a junction while the barcode before it is still decoding.
 * The letter arriving starts the manoeuvre from here; it was decoded from
 * a symbol already passed, so no barcode-to-junction clearance applies.
 * Bounded: the decode must finish and deliver within the configured time.
 */
static void nav_decode_hold(nav_controller_t *nav, const nav_sample_t *s,
                            nav_result_t *result)
{
    result->drive = NAV_OUTPUT_STOP;
    if (nav->hold_letter)
    {
        nav->decode_hold = 0U;
        nav->hold_letter = 0U;
        nav->black_since = 0U;
        nav_take_junction(nav, s->now_msec, result);
        return;
    }
    if (s->barcode_decoding) nav->decode_idle_since = 0U;
    else if (nav->decode_idle_since == 0U)
        nav->decode_idle_since = (s->now_msec != 0U) ? s->now_msec : 1U;
    if ((uint32_t)(s->now_msec - nav->hold_since) >=
            nav->cfg.decode_wait_msec ||
        (nav->decode_idle_since != 0U &&
         (uint32_t)(s->now_msec - nav->decode_idle_since) >=
            nav->cfg.decode_grace_msec))
    {
        nav->decode_hold = 0U;
        if (nav->cfg.repeat_action && nav->pending)
        {
            /* Decode failed: carry on with the remembered action. */
            nav->black_since = 0U;
            nav_take_junction(nav, s->now_msec, result);
        }
        else nav_fault(nav, 5U, result); /* decode gave no usable letter */
    }
}
nav_result_t nav_step(nav_controller_t *nav, const nav_sample_t *s)
{
    nav_result_t result = {NAV_OUTPUT_STOP, NAV_EVENT_NONE, 0U};
    uint8_t both, edge;
    if (nav == NULL || s == NULL) return result;
    if (nav->state == NAV_WAIT || nav->state == NAV_FAULT) return result;
    both = (uint8_t)(s->left_black && s->right_black);
    edge = (uint8_t)((s->left_black != s->right_black) && !both);
    if ((nav->state == NAV_FOLLOW || nav->state == NAV_APPROACH) &&
        nav->decode_hold)
    {
        nav_decode_hold(nav, s, &result);
        return result;
    }
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
            /* A run starts at the first both-black reading, and starts
             * again if the 1 ms sampler saw white in between (barcode
             * bars, or a curve swept under both sensors twice). */
            if (nav->black_since == 0U || s->both_interrupted)
            {
                nav->black_since = (s->now_msec != 0U) ? s->now_msec : 1U;
                nav->black_from_counts = s->travel_counts;
            }
            elapsed = (uint32_t)(s->now_msec - nav->black_since);
            travel = s->travel_counts - nav->black_from_counts;
            /* Keep rolling straight while measuring the run. On a curve
             * the line crosses diagonally under both sensors for only a
             * few mm and never reaches the confirmation distance; a
             * junction square does. Stopping on the first reading instead
             * would freeze the curve under both sensors and "confirm" it.
             * Bounded by distance and by time: never blind motion. */
            if (travel < nav->cfg.junction_confirm_counts &&
                elapsed < nav->cfg.junction_confirm_max_msec)
            {
                result.drive = NAV_OUTPUT_FORWARD;
                return result;
            }
            {
                if ((!nav->pending || nav->cfg.repeat_action) &&
                    s->barcode_decoding && nav->cfg.decode_wait_msec != 0U)
                {
                    /* Letter not ready yet: stop here and wait for it. In
                     * repeat mode too: the barcode just passed may replace
                     * the remembered action. */
                    nav->decode_hold = 1U;
                    nav->hold_letter = 0U;
                    nav->hold_since = s->now_msec;
                    nav->decode_idle_since = 0U;
                    result.drive = NAV_OUTPUT_STOP;
                    result.event = NAV_EVENT_DECODE_WAIT;
                    result.value = 0U;
                }
                /* A capture running here does not block a queued action:
                 * barcodes sit beside the line, so both line sensors black
                 * is not barcode ink; the capture is the junction tape
                 * passing the barcode sensor (logged), and is ignored. The
                 * clearance is measured from where the barcode ended. */
                else if (!nav->pending ||
                    (uint32_t)(s->travel_counts - nav->barcode_counts) <
                    nav->cfg.min_after_barcode_counts)
                {
                    nav_fault(nav, 1U, &result);
                }
                else
                {
                    nav_take_junction(nav, s->now_msec, &result);
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
                nav_action_done(nav);
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
    if (nav->state == NAV_CENTRE)
    {
        /* Follow the arm straight ahead until the axle is over the centre.
         * Still on the crossing bar (both black) the follower would stop,
         * so drive straight there. Bounded by time: never blind motion. */
        uint32_t const travel = s->travel_counts - nav->centre_from;
        result.drive = both ? NAV_OUTPUT_FORWARD : NAV_OUTPUT_FOLLOW;
        if ((uint32_t)(s->now_msec - nav->state_since) >
            nav->cfg.centre_max_msec)
        {
            nav_fault(nav, 3U, &result);
        }
        else if (travel >= nav->cfg.centre_counts)
        {
            nav_begin_turn(nav, s, &result);
        }
        return result;
    }
    if (nav->state == NAV_TURN && nav->cfg.turn90_counts != 0U)
    {
        /* Pivot on the spot by encoder angle. Lines crossed early (the arm
         * ahead, and for a U-turn the side arm) are ignored until the gate;
         * the first black after it is the arm being turned onto. */
        uint32_t const q = nav->cfg.turn90_counts;
        uint8_t const u = (uint8_t)(nav->action == NAV_UTURN);
        uint32_t const gate = u ? (q * 5U) / 3U : (q * 2U) / 3U;
        uint32_t const limit = u ? (q * 5U) / 2U : (q * 3U) / 2U;
        uint32_t const turned = s->turn_counts - nav->turn_from;
        result.drive = nav_turn_drive(nav);
        if (!nav->turn_announced)
        {
            nav->turn_announced = 1U;
            result.event = NAV_EVENT_TURN_STARTED;
            result.value = nav->action;
        }
        /* The line is checked first: reaching it on the same reading as
         * the limit is a finished turn, not a fault (logged U-turn: the
         * sensor was on the line just as the count limit tripped). */
        if (turned >= gate && (s->left_black || s->right_black))
        {
            /* Brake this one period to kill the spin, then follow. */
            nav->state = NAV_RESUME;
            nav_action_done(nav);
            nav->black_since = 0U;
            result.drive = NAV_OUTPUT_STOP;
            result.event = NAV_EVENT_TURN_COMPLETED;
            result.value = nav->action;
        }
        else if (turned > limit || (uint32_t)(s->now_msec - nav->state_since)
                 > nav->cfg.max_turn_msec)
        {
            nav_fault(nav, 2U, &result); /* new arm never found */
        }
        return result;
    }
    if (nav->state == NAV_TURN || nav->state == NAV_REACQUIRE)
    {
        result.drive = nav_turn_drive(nav);
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
                nav_action_done(nav);
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

