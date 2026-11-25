/** @file nav_core.c
 * @brief Pure bounded navigation state transitions, no hardware access.
 */
#include "nav_core.h"
#include <stddef.h>

/* A capture still recording when a junction confirms can only finish if
 * the robot keeps moving: roll on up to this much more (100 mm) before
 * deciding. Stopping would stall the capture into a failed read and time
 * the crossing out. Centring counts from where the bar was first seen, so
 * the extra roll does not shift a turn. */
#ifndef NAV_CAPTURE_ROLL_COUNTS
#define NAV_CAPTURE_ROLL_COUNTS ((100U * 1056U * 10U) / 2198U)
#endif
/* A straight crossing may drive on over a barcode being captured, at most
 * this far (400 mm) past where the bar was first seen. */
#ifndef NAV_CROSS_CAPTURE_COUNTS
#define NAV_CROSS_CAPTURE_COUNTS ((400U * 1056U * 10U) / 2198U)
#endif
/* And never longer than this (a stalled robot does not travel). */
#define NAV_CAPTURE_ROLL_MSEC 3000U
#ifndef NAV_TAPE_LEAD_COUNTS
/* Highest barcode letter index (A..Z = 0..25) and action code (2 bits). */
#define NAV_LETTER_MAX 25U
#define NAV_ACTION_MAX 3U
/* The same letter read again within this time and travel is the same
 * printed card, not a new barcode (needs track calibration). */
#define NAV_DUP_WINDOW_MSEC 1500U
#define NAV_DUP_WINDOW_COUNTS 120U
/* A capture starting no more than this far before a both-black run (or
 * after it) began is the crossing tape: 40 mm at 1056 counts per 219.8 mm.
 * Real barcodes sit well before the junction (~300 mm). */
#define NAV_TAPE_LEAD_COUNTS ((40U * 1056U * 10U) / 2198U)
#endif
/* nav_barcode() refusals: not following, or the decode deadline passed. */
#define NAV_BARCODE_NOT_RUNNING (-2)
#define NAV_BARCODE_TOO_LATE    (-3)
/* Safe-stop reasons latched in fault_reason and sent as the FAULT value. */
#define NAV_FAULT_UNMAPPED      1U /* junction with no usable action */
#define NAV_FAULT_TURN          2U /* new arm never found */
#define NAV_FAULT_CROSS         3U /* crossing or centring timed out */
#define NAV_FAULT_STATE         4U /* unknown navigation state */
#define NAV_FAULT_NO_LETTER     5U /* decode gave no usable letter */
/* Pivot gate and limit as fractions of a 90-degree turn (num / den). */
#define NAV_TURN_GATE_NUM       2U /* turn: arm accepted after 2/3 */
#define NAV_TURN_GATE_DEN       3U
#define NAV_UTURN_GATE_NUM      5U /* U-turn: arm accepted after 5/3 */
#define NAV_UTURN_GATE_DEN      3U
#define NAV_TURN_LIMIT_NUM      3U /* turn: fault past 3/2 */
#define NAV_TURN_LIMIT_DEN      2U
#define NAV_UTURN_LIMIT_NUM     5U /* U-turn: fault past 5/2 */
#define NAV_UTURN_LIMIT_DEN     2U

/** @brief Reset the navigation controller and load its configuration. */
void nav_init(nav_controller_t *p_nav, const nav_config_t *p_cfg)
{
    if (p_nav == NULL || p_cfg == NULL)
    {
        return;
    }
    *p_nav = (nav_controller_t){0};
    p_nav->cfg = *p_cfg;
    p_nav->state = NAV_WAIT;
}
/** @brief Stop navigation and wait for the next START. */
void nav_stop(nav_controller_t *p_nav)
{
    if (p_nav == NULL)
    {
        return;
    }
    p_nav->state = NAV_WAIT;
    p_nav->pending = 0U;
    p_nav->black_since = 0U;
    p_nav->black_from_counts = 0U;
    p_nav->edge_since = 0U;
    p_nav->edge_seen = 0U;
    p_nav->decode_hold = 0U;
}
/** @brief Stop and forget the remembered action (STOP command). */
void nav_forget(nav_controller_t *p_nav)
{
    if (p_nav == NULL)
    {
        return;
    }
    nav_stop(p_nav);
    p_nav->held = 0U;
    p_nav->fault_reason = 0U;
    p_nav->hold_letter = 0U;
    p_nav->last_barcode_seq = 0U;
    p_nav->have_barcode = 0U;
}
/** @brief Start line following (a latched fault stays latched). */
void nav_start(nav_controller_t *p_nav, uint32_t now_msec)
{
    if (p_nav == NULL || p_nav->state == NAV_FAULT)
    {
        return;
    }
    p_nav->state = NAV_FOLLOW;
    p_nav->pending = 0U;
    p_nav->black_since = 0U;
    p_nav->black_from_counts = 0U;
    p_nav->edge_seen = 0U;
    p_nav->turn_announced = 0U;
    p_nav->resume_announced = 0U;
    p_nav->edge_since = 0U;
    p_nav->decode_hold = 0U;
    p_nav->state_since = now_msec;
    /* An action not yet used (single mode) or still repeating survives an
     * obstacle bypass. Its barcode is long passed, so no barcode clearance
     * applies to it. */
    if (0U != p_nav->held)
    {
        p_nav->pending = 1U;
        p_nav->letter = p_nav->held_letter;
        p_nav->action = p_nav->held_action;
        p_nav->barcode_counts = 0U;
    }
}
/* A junction action finished. Single mode: it is used up and forgotten,
 * so the next junction needs a new barcode. Repeat mode: it stays queued
 * for the next junction until a new barcode replaces it. */
static void nav_action_done(nav_controller_t *p_nav)
{
    if (!(p_nav->cfg.repeat_action && p_nav->held))
    {
        p_nav->pending = 0U;
        p_nav->held = 0U;
    }
}
/** @brief Queue a decoded barcode's action for the next junction. */
int32_t nav_barcode(nav_controller_t *p_nav, uint8_t letter, uint8_t action,
                uint16_t seq, uint32_t ms, uint32_t traveled)
{
    if (p_nav == NULL || letter > NAV_LETTER_MAX || action > NAV_ACTION_MAX)
    {
        return NAV_BARCODE_NOT_RUNNING;
    }
    if (p_nav->state != NAV_FOLLOW && p_nav->state != NAV_APPROACH)
    {
        return NAV_BARCODE_NOT_RUNNING;
    }
    /* A result consumed after the hard deadline cannot rescue that wait.
     * At the boundary itself, a valid result wins before nav_step. */
    if (p_nav->decode_hold &&
        (int32_t)(ms - (p_nav->hold_since + p_nav->cfg.decode_wait_msec)) > 0)
    {
        return NAV_BARCODE_TOO_LATE;
    }
    if (p_nav->have_barcode && p_nav->last_barcode_seq == seq)
    {
        return 0;
    }
    /* Reject a repeat read of the same printed card within the duplicate
     * window. The window is deliberately small and needs calibration
     * against real barcode/track geometry. */
    if (p_nav->have_barcode && p_nav->letter == letter &&
        (uint32_t)(ms - p_nav->last_barcode_ms) < NAV_DUP_WINDOW_MSEC &&
        (uint32_t)(traveled - p_nav->last_barcode_counts) <
            NAV_DUP_WINDOW_COUNTS)
    {
        return 0;
    }
    /* Single mode keeps the first queued letter; repeat mode replaces the
     * remembered action with the newest barcode. */
    if (p_nav->pending != 0U && !p_nav->cfg.repeat_action)
    {
        return -1;
    }
    p_nav->pending = 1U;
    if (0U != p_nav->decode_hold)
    {
        p_nav->hold_letter = 1U;
    }
    /* Remembered until used (single) or replaced (repeat), so an obstacle
     * bypass between the barcode and its junction does not lose it. */
    p_nav->held = 1U;
    p_nav->held_letter = letter;
    p_nav->held_action = action;
    p_nav->letter = letter;
    p_nav->action = action;
    p_nav->barcode_counts = traveled;
    p_nav->last_barcode_counts = traveled;
    p_nav->last_barcode_ms = ms;
    p_nav->last_barcode_seq = seq;
    p_nav->have_barcode = 1U;
    p_nav->state = NAV_APPROACH;
    /* Stopped at a junction waiting for this letter: keep where its bar
     * began, the turn's drive to the centre is measured from there. */
    if (!p_nav->decode_hold && p_nav->black_since == 0U)
    {
        p_nav->black_since = 0U;
        p_nav->black_from_counts = 0U;
    }
    return 1;
}
/* Pivot output for the queued turn (left, right or U-turn). */
static nav_drive_t nav_turn_drive(const nav_controller_t *p_nav)
{
    return (nav_drive_t)(NAV_OUTPUT_LEFT + p_nav->action - 1U);
}
/* Start pivoting from here; the angle is measured from turn_counts. */
static void nav_begin_turn(nav_controller_t *p_nav,
    const nav_sample_t *p_sample,
                           nav_result_t *p_result)
{
    p_nav->state = NAV_TURN;
    p_nav->state_since = p_sample->now_msec;
    p_nav->turn_from = p_sample->turn_counts;
    p_nav->turn_announced = 1U;
    p_nav->edge_since = 0U;
    p_result->drive = nav_turn_drive(p_nav);
    p_result->event = NAV_EVENT_TURN_STARTED;
    p_result->value = p_nav->action;
}
/* Confirmed junction with a letter queued: start the mapped manoeuvre.
 * Straight drives over the cross. A turn first drives on to the junction
 * centre (when configured) so the pivot lands the sensors on the new arm.
 */
static void nav_take_junction(nav_controller_t *p_nav, uint32_t now,
                              nav_result_t *p_result)
{
    if (p_nav->action == NAV_STRAIGHT)
    {
        p_nav->state = NAV_CROSS;
    }
    else if (p_nav->cfg.centre_counts != 0U)
    {
        p_nav->state = NAV_CENTRE;
    }
    else
    {
        p_nav->state = NAV_TURN;
    }
    p_nav->centre_from = p_nav->black_from_counts;
    p_nav->state_since = now;
    p_nav->edge_seen = 0U;
    p_nav->turn_announced = 0U;
    p_nav->edge_since = 0U;
    p_result->drive = (p_nav->state == NAV_TURN) ? nav_turn_drive(p_nav) :
        NAV_OUTPUT_FORWARD;
    p_result->event = NAV_EVENT_JUNCTION;
    p_result->value = p_nav->action;
}
/* Confirmed junction with no usable action: cross it straight on. Nothing
 * is queued, so the remembered repeat action (none) is left untouched. */
static void nav_take_straight(nav_controller_t *p_nav, uint32_t now,
                              nav_result_t *p_result)
{
    p_nav->action = NAV_STRAIGHT;
    nav_take_junction(p_nav, now, p_result);
}
/** @brief Latch a navigation fault and command a stop. */
static void nav_fault(nav_controller_t *p_nav, uint8_t reason,
                      nav_result_t *p_result)
{
    p_nav->state = NAV_FAULT;
    p_nav->fault_reason = reason;
    p_nav->decode_hold = 0U;
    p_result->drive = NAV_OUTPUT_STOP;
    p_result->event = NAV_EVENT_FAULT;
    p_result->value = reason;
}
/* Stopped at a junction while the barcode before it is still decoding.
 * The letter arriving starts the manoeuvre from here; it was decoded from
 * a symbol already passed, so no barcode-to-junction clearance applies.
 * Bounded: the decode must finish and deliver within the configured time.
 */
static void nav_decode_hold(nav_controller_t *p_nav,
    const nav_sample_t *p_sample,
                            nav_result_t *p_result)
{
    p_result->drive = NAV_OUTPUT_STOP;
    if (0U != p_nav->hold_letter)
    {
        p_nav->decode_hold = 0U;
        p_nav->hold_letter = 0U;
        p_nav->black_since = 0U;
        nav_take_junction(p_nav, p_sample->now_msec, p_result);
        return;
    }
    if (0U != p_sample->barcode_decoding)
    {
        p_nav->decode_idle_since = 0U;
    }
    else if (p_nav->decode_idle_since == 0U)
    {
        p_nav->decode_idle_since = (p_sample->now_msec != 0U)
            ? p_sample->now_msec : 1U;
    }
    if ((uint32_t)(p_sample->now_msec - p_nav->hold_since) >=
            p_nav->cfg.decode_wait_msec ||
        (p_nav->decode_idle_since != 0U &&
         (uint32_t)(p_sample->now_msec - p_nav->decode_idle_since) >=
            p_nav->cfg.decode_grace_msec))
    {
        p_nav->decode_hold = 0U;
        if (p_nav->cfg.repeat_action && p_nav->pending)
        {
            /* Decode failed: carry on with the remembered action. */
            p_nav->black_since = 0U;
            nav_take_junction(p_nav, p_sample->now_msec, p_result);
        }
        else if (p_nav->cfg.straight_default)
        {
            /* No letter and nothing remembered: go straight on. */
            p_nav->black_since = 0U;
            nav_take_straight(p_nav, p_sample->now_msec, p_result);
        }
        else
        {
            nav_fault(p_nav, NAV_FAULT_NO_LETTER, p_result);
        }
    }
}
/** @brief Run one navigation period: junctions, turns and decode waits. */
nav_result_t nav_step(nav_controller_t *p_nav, const nav_sample_t *p_sample)
{
    nav_result_t result = {NAV_OUTPUT_STOP, NAV_EVENT_NONE, 0U};
    uint8_t both;
    uint8_t edge;
    if (p_nav == NULL || p_sample == NULL)
    {
        return result;
    }
    if (p_nav->state == NAV_WAIT || p_nav->state == NAV_FAULT)
    {
        return result;
    }
    both = (uint8_t)(p_sample->left_black && p_sample->right_black);
    edge = (uint8_t)((p_sample->left_black != p_sample->right_black) && !both);
    if ((p_nav->state == NAV_FOLLOW || p_nav->state == NAV_APPROACH) &&
        p_nav->capture_roll)
    {
        /* Past the bar while letting a capture finish: still this junction. */
        both = 1U;
    }
    if ((p_nav->state == NAV_FOLLOW || p_nav->state == NAV_APPROACH) &&
        p_nav->decode_hold)
    {
        nav_decode_hold(p_nav, p_sample, &result);
        return result;
    }
    if (p_nav->state == NAV_FOLLOW || p_nav->state == NAV_APPROACH)
    {
        result.drive = NAV_OUTPUT_FOLLOW;
        if (0U != p_nav->resume_announced)
        {
            p_nav->resume_announced = 0U;
            result.event = NAV_EVENT_FOLLOWING;
            result.value = p_nav->action;
        }
        if (0U != both)
        {
            uint32_t elapsed;
            uint32_t travel;
            /* A run starts at the first both-black reading, and starts
             * again if the 1 ms sampler saw white in between (barcode
             * bars, or a curve swept under both sensors twice). */
            if (p_nav->black_since == 0U ||
                (p_sample->both_interrupted && !p_nav->capture_roll))
            {
                p_nav->black_since = (p_sample->now_msec != 0U)
                    ? p_sample->now_msec : 1U;
                p_nav->black_from_counts = p_sample->travel_counts;
            }
            elapsed = (uint32_t)(p_sample->now_msec - p_nav->black_since);
            travel = p_sample->travel_counts - p_nav->black_from_counts;
            /* Keep rolling straight while measuring the run. On a curve
             * the line crosses diagonally under both sensors for only a
             * few mm and never reaches the confirmation distance; a
             * junction square does. Stopping on the first reading instead
             * would freeze the curve under both sensors and "confirm" it.
             * Bounded by distance and by time: never blind motion. */
            if (travel < p_nav->cfg.junction_confirm_counts &&
                elapsed < p_nav->cfg.junction_confirm_max_msec)
            {
                result.drive = NAV_OUTPUT_FORWARD;
                return result;
            }
            {
                /* A capture that began within NAV_TAPE_LEAD_COUNTS of this
                 * both-black run is the crossing tape under the barcode
                 * sensor, not a barcode (barcodes come before the
                 * junction). Waiting for it would stop the robot on the
                 * tape, so the stripe never ends and the crossing times
                 * out. Ignore it; the caller drops it. */
                uint8_t b_tape = (uint8_t)(p_sample->barcode_capturing &&
                    p_sample->capture_start_known &&
                    ((int32_t)(p_sample->capture_start_counts -
                               p_nav->black_from_counts) >=
                     -(int32_t)NAV_TAPE_LEAD_COUNTS));
                if (!b_tape && p_sample->capture_active)
                {
                    if ((travel < (p_nav->cfg.junction_confirm_counts +
                                   NAV_CAPTURE_ROLL_COUNTS)) &&
                        (elapsed < (p_nav->cfg.junction_confirm_max_msec +
                                    NAV_CAPTURE_ROLL_MSEC)))
                    {
                        /* Let the capture finish before stopping for it. */
                        p_nav->capture_roll = 1U;
                        result.drive = NAV_OUTPUT_FORWARD;
                        return result;
                    }
                    /* Still recording 100 mm on: a long dark area, not a
                     * barcode. Drop it like the tape and carry on. */
                    b_tape = 1U;
                }
                p_nav->capture_roll = 0U;
                if (b_tape)
                {
                    p_nav->tape_capture = 1U;
                }
                if ((!p_nav->pending || p_nav->cfg.repeat_action) &&
                    p_sample->barcode_decoding && !b_tape &&
                    p_nav->cfg.decode_wait_msec != 0U)
                {
                    /* Letter not ready yet: stop here and wait for it. In
                     * repeat mode too: the barcode just passed may replace
                     * the remembered action. */
                    p_nav->decode_hold = 1U;
                    p_nav->hold_letter = 0U;
                    p_nav->hold_since = p_sample->now_msec;
                    p_nav->decode_idle_since = 0U;
                    result.drive = NAV_OUTPUT_STOP;
                    result.event = NAV_EVENT_DECODE_WAIT;
                    result.value = 0U;
                }
                /* A capture running here does not block a queued action:
                 * barcodes sit beside the line, so both line sensors black
                 * is not barcode ink; the capture is the junction tape
                 * passing the barcode sensor, and is ignored. The
                 * clearance is measured from where the barcode ended. */
                else if (!p_nav->pending && p_nav->cfg.straight_default)
                {
                    /* No barcode action for this junction: go straight. */
                    nav_take_straight(p_nav, p_sample->now_msec, &result);
                }
                else if (p_nav->pending && p_nav->cfg.clearance_straight &&
                    (uint32_t)(
                        p_sample->travel_counts - p_nav->barcode_counts) <
                    p_nav->cfg.min_after_barcode_counts)
                {
                    /* Too close to have been meant for this junction (most
                     * likely the crossing tape read as a barcode): drop it
                     * and cross straight on rather than stop. */
                    p_nav->pending = 0U;
                    p_nav->held = 0U;
                    p_nav->clearance_dropped = 1U;
                    nav_take_straight(p_nav, p_sample->now_msec, &result);
                }
                else if (!p_nav->pending ||
                    (uint32_t)(
                        p_sample->travel_counts - p_nav->barcode_counts) <
                    p_nav->cfg.min_after_barcode_counts)
                {
                    nav_fault(p_nav, NAV_FAULT_UNMAPPED, &result);
                }
                else
                {
                    nav_take_junction(p_nav, p_sample->now_msec, &result);
                }
            }
        }
        else
        {
            p_nav->black_since = 0U;
            p_nav->black_from_counts = 0U;
        }
        return result;
    }
    if (p_nav->state == NAV_CROSS)
    {
        result.drive = NAV_OUTPUT_FORWARD;
        if (p_sample->capture_active &&
            (uint32_t)(p_sample->travel_counts - p_nav->centre_from) <
                NAV_CROSS_CAPTURE_COUNTS)
        {
            /* Crossing a barcode laid across the line: its bars keep both
             * sensors black with gaps too short to read as the far side.
             * While the capture records, keep driving straight over it;
             * the cross timeout counts from where it ends. */
            p_nav->state_since = p_sample->now_msec;
            p_nav->edge_since = 0U;
            return result;
        }
        if ((uint32_t)(p_sample->now_msec - p_nav->state_since) >
            p_nav->cfg.max_cross_msec)
        {
            p_nav->state = NAV_FAULT;
            p_nav->fault_reason = NAV_FAULT_CROSS;
            result.drive = NAV_OUTPUT_STOP;
            result.event = NAV_EVENT_FAULT;
            result.value = NAV_FAULT_CROSS;
        }
        else if (!both && (uint32_t)(p_sample->now_msec - p_nav->state_since) >=
                 p_nav->cfg.min_turn_msec)
        {
            if (p_nav->edge_since == 0U)
            {
                p_nav->edge_since = p_sample->now_msec;
            }
            if ((uint32_t)(p_sample->now_msec - p_nav->edge_since) >=
                p_nav->cfg.confirm_msec)
            {
                nav_action_done(p_nav);
                p_nav->state = NAV_FOLLOW;
                p_nav->black_since = 0U;
                result.drive = NAV_OUTPUT_FOLLOW;
                result.event = NAV_EVENT_FOLLOWING;
                result.value = NAV_STRAIGHT;
            }
        }
        else
        {
            p_nav->edge_since = 0U;
        }
        return result;
    }
    if (p_nav->state == NAV_CENTRE)
    {
        /* Follow the arm straight ahead until the axle is over the centre.
         * Still on the crossing bar (both black) the follower would stop,
         * so drive straight there. Bounded by time: never blind motion. */
        uint32_t const travel = p_sample->travel_counts - p_nav->centre_from;
        result.drive = both ? NAV_OUTPUT_FORWARD : NAV_OUTPUT_FOLLOW;
        if ((uint32_t)(p_sample->now_msec - p_nav->state_since) >
            p_nav->cfg.centre_max_msec)
        {
            nav_fault(p_nav, NAV_FAULT_CROSS, &result);
        }
        else if (travel >= p_nav->cfg.centre_counts)
        {
            nav_begin_turn(p_nav, p_sample, &result);
        }
        return result;
    }
    if (p_nav->state == NAV_TURN && p_nav->cfg.turn90_counts != 0U)
    {
        /* Pivot on the spot by encoder angle. Lines crossed early (the arm
         * ahead, and for a U-turn the side arm) are ignored until the gate;
         * the first black after it is the arm being turned onto. */
        uint32_t const q = p_nav->cfg.turn90_counts;
        uint8_t const u = (uint8_t)(p_nav->action == NAV_UTURN);
        uint32_t const gate = u ?
            (q * NAV_UTURN_GATE_NUM) / NAV_UTURN_GATE_DEN :
            (q * NAV_TURN_GATE_NUM) / NAV_TURN_GATE_DEN;
        uint32_t const limit = u ?
            (q * NAV_UTURN_LIMIT_NUM) / NAV_UTURN_LIMIT_DEN :
            (q * NAV_TURN_LIMIT_NUM) / NAV_TURN_LIMIT_DEN;
        uint32_t const turned = p_sample->turn_counts - p_nav->turn_from;
        result.drive = nav_turn_drive(p_nav);
        if (0U == p_nav->turn_announced)
        {
            p_nav->turn_announced = 1U;
            result.event = NAV_EVENT_TURN_STARTED;
            result.value = p_nav->action;
        }
        /* The line is checked first: reaching it on the same reading as
         * the limit is a finished turn, not a fault. */
        if (turned >= gate && (p_sample->left_black || p_sample->right_black))
        {
            /* Brake this one period to kill the spin, then follow. */
            p_nav->state = NAV_RESUME;
            nav_action_done(p_nav);
            p_nav->black_since = 0U;
            result.drive = NAV_OUTPUT_STOP;
            result.event = NAV_EVENT_TURN_COMPLETED;
            result.value = p_nav->action;
        }
        else if (turned > limit || (uint32_t)(
            p_sample->now_msec - p_nav->state_since)
                 > p_nav->cfg.max_turn_msec)
        {
            nav_fault(p_nav, NAV_FAULT_TURN, &result);
        }
        return result;
    }
    if (p_nav->state == NAV_TURN || p_nav->state == NAV_REACQUIRE)
    {
        result.drive = nav_turn_drive(p_nav);
        if (0U == p_nav->turn_announced)
        {
            p_nav->turn_announced = 1U;
            result.event = NAV_EVENT_TURN_STARTED;
            result.value = p_nav->action;
        }
        if ((uint32_t)(p_sample->now_msec - p_nav->state_since) >
            p_nav->cfg.max_turn_msec)
        {
            p_nav->state = NAV_FAULT;
            p_nav->fault_reason = NAV_FAULT_TURN;
            result.drive = NAV_OUTPUT_STOP;
            result.event = NAV_EVENT_FAULT;
            result.value = NAV_FAULT_TURN;
        }
        else if ((uint32_t)(p_sample->now_msec - p_nav->state_since) >=
                 p_nav->cfg.min_turn_msec && edge)
        {
            if (p_nav->edge_since == 0U)
            {
                p_nav->edge_since = p_sample->now_msec;
            }
            if ((uint32_t)(p_sample->now_msec - p_nav->edge_since) >=
                p_nav->cfg.confirm_msec)
            {
                p_nav->state = NAV_RESUME;
                nav_action_done(p_nav);
                p_nav->black_since = 0U;
                result.drive = NAV_OUTPUT_FOLLOW;
                result.event = NAV_EVENT_TURN_COMPLETED;
                result.value = p_nav->action;
            }
            else
            {
                p_nav->state = NAV_REACQUIRE;
            }
        }
        else
        {
            p_nav->edge_since = 0U;
        }
        return result;
    }
    if (p_nav->state == NAV_RESUME)
    {
        p_nav->state = NAV_FOLLOW;
        p_nav->resume_announced = 1U;
        result.drive = NAV_OUTPUT_FOLLOW;
        result.event = NAV_EVENT_LINE_REACQUIRED;
        result.value = p_nav->action;
        return result;
    }
    p_nav->state = NAV_FAULT;
    result.event = NAV_EVENT_FAULT;
    result.value = NAV_FAULT_STATE;
    return result;
}

/*** end of file ***/

