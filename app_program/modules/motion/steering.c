/** @file steering.c
 * @brief Turn harder the longer the line stays off-centre, then ease off.
 *
 * Two digital IR sensors give only left, centre or right, so the error is
 * -1, 0 or +1. A derivative term would see only steps; the integral is what
 * grades the response, and its decay avoids snapping straight on the line.
 */
#include "steering.h"
#include "app/app_config.h"

/* PI steering (STEERING_OBSERVER 0): steer for any off-centre reading,
 * and its growth per second.
 */
#define STEERING_KP 0.3f
#define STEERING_KI 2.0f
/* Fraction of the integral kept per centred period: lower recentres faster. */
#define STEERING_DECAY 0.85f
/* A touch that built less integral than this is a nudge on a straight, so
 * the turn ends with it; keeping it would overshoot to the other edge. A
 * longer touch is a curve, and its turn is kept through the bend. At
 * STEERING_KI 2.0 and 50 ms periods, 0.35 is about four periods.
 */
#define STEERING_CURVE_INTEGRAL 0.35f
/* Inside a bend the touches are short but keep coming from the same side.
 * A touch on the same side within this many centred periods of the last
 * one is a curve too, however short. Straight-line nudges come about a
 * second apart or from alternating sides.
 */
#define STEERING_REPEAT_PERIODS 8U
/* A touch that drove the turn to full strength is a bend too tight to have
 * been rounded yet: when the sensor clears, the line may have slipped round
 * beside the robot, where driving on would never see it again. The full
 * turn is held this many periods, about 0.7 s, or until a sensor finds the
 * line. If the line was in fact centred, the other sensor finds it instead.
 */
#define STEERING_HOLD_PERIODS 12U
/* After a full-strength turn the robot is in a tight bend, and a brief
 * touch on the other side as it swings past the line must not end it. A
 * touch on the full turn's side within this many periods, about 1.5 s,
 * resumes the full turn and holds it.
 */
#define STEERING_BEND_MEMORY_PERIODS 25U
/* Only a full turn whose sensor stayed on the line this many periods in a
 * row, about 0.3 s, restarts that window. Straight-line nudges last two or
 * three periods, so after a bend the window closes.
 */
#define STEERING_BEND_TOUCH_PERIODS 5U

/* ---- Line observer (STEERING_OBSERVER) ---------------------------------
 * The IR pair sits APP_NAV_IR_TO_AXLE_MM ahead of the axle, so a turn of
 * the robot swings it sideways by that lever; the wheels are
 * APP_NAV_TRACK_MM apart. LINE_TRACK_SENSOR_GAP_MM is centre to centre of
 * the two IR spots; the model tolerates 20 to 34 mm.
 */
#define LINE_TRACK_SENSOR_GAP_MM 26U
#define LINE_TRACK_TAPE_MM 18U /* Line tape width, mm. */
#define LINE_TRACK_LEAD_MM ((float)APP_NAV_IR_TO_AXLE_MM)
#define LINE_TRACK_TRACK_MM ((float)APP_NAV_TRACK_MM)
/* Where each sensor sees the line: the line's centre within the tape's
 * half width of the sensor. Between the inner edges it is in the gap. */
#define LINE_TRACK_GAP_EDGE_MM \
    ((float)(LINE_TRACK_SENSOR_GAP_MM - LINE_TRACK_TAPE_MM) / 2.0f)
#define LINE_TRACK_OUTER_EDGE_MM \
    ((float)(LINE_TRACK_SENSOR_GAP_MM + LINE_TRACK_TAPE_MM) / 2.0f)
/* PID gains: steer per mm of offset, per radian of heading across the line,
 * and per mm-mm of offset travelled (curves). Tuned in a closed-loop model
 * over 35-65 RPM, 20-34 mm sensor gaps and 110-135 mm tracks.
 */
#define LINE_TRACK_KP 0.06f
#define LINE_TRACK_KD 0.5f
#define LINE_TRACK_KI 0.0002f
/* A sensor's correction of the offset also corrects the heading, spread
 * over the distance it built up across (at least LINE_TRACK_SPAN_MM). */
#define LINE_TRACK_BETA 0.5f
#define LINE_TRACK_SPAN_MM 20.0f
/* Heading estimate bound: about 45 degrees, beyond any real line angle. */
#define LINE_TRACK_HEADING_MAX 0.8f

#if (LINE_TRACK_SENSOR_GAP_MM <= LINE_TRACK_TAPE_MM)
#error The line observer needs the sensors wider apart than the tape
#endif

/** @brief Bound a value to plus or minus STEERING_MAX. */
static float clamp_steer (float value);
/** @brief Bound a value to plus or minus a limit. */
static float clamp_float (float value, float limit);
/** @brief Bound the speed scale to 0..STEERING_SCALE_MAX. */
static float clamp_scale (float scale);

void steering_reset (steering_t * p_steering)
{
    p_steering->integral      = 0.0f;
    p_steering->b_off_centre  = false;
    p_steering->last_side     = 0;
    p_steering->centred_count = 0.0f;
    p_steering->hold_count    = 0.0f;
    p_steering->bend_side     = 0;
    p_steering->bend_age      = (float)STEERING_BEND_MEMORY_PERIODS + 1.0f;
    p_steering->touch_count   = 0.0f;
}

float steering_update (steering_t * p_steering, line_follow_action_t action,
                       float period_sec, float scale)
{
    /* Each period advances everything by the distance it covered, in
     * reference periods; so does the decay, to first order (it stays above
     * zero for every allowed scale). */
    float const step  = clamp_scale(scale);
    float const decay = 1.0f - ((1.0f - STEERING_DECAY) * step);
    float       error = 0.0f;

    if (LINE_CORRECT_LEFT == action)
    {
        error = 1.0f;
    }
    else if (LINE_CORRECT_RIGHT == action)
    {
        error = -1.0f;
    }
    else
    {
        /* Centred, or not steering: no error. */
    }

    if (p_steering->bend_age <= (float)STEERING_BEND_MEMORY_PERIODS)
    {
        p_steering->bend_age += step;
    }

    if (0.0f == error)
    {
        if (p_steering->b_off_centre)
        {
            /* A touch just ended; the integral's sign is its side. */
            float const side = (p_steering->integral < 0.0f) ? -1.0f : 1.0f;
            float const size = side * p_steering->integral;
            bool const  b_repeat =
                (side == (float)p_steering->last_side) &&
                (p_steering->centred_count <= (float)STEERING_REPEAT_PERIODS);
            bool const  b_bend =
                (side == (float)p_steering->bend_side) &&
                (p_steering->bend_age <= (float)STEERING_BEND_MEMORY_PERIODS);

            if (size >= STEERING_MAX)
            {
                /* Full-strength turn: keep it while the line is found. */
                p_steering->hold_count = (float)STEERING_HOLD_PERIODS;

                if (p_steering->touch_count >=
                    (float)STEERING_BEND_TOUCH_PERIODS)
                {
                    p_steering->bend_side = (side > 0.0f) ? 1 : -1;
                    p_steering->bend_age  = 0.0f;
                }
            }
            else if (b_bend)
            {
                /* The tight bend goes on: resume the full turn. */
                p_steering->integral   = side * STEERING_MAX;
                p_steering->hold_count = (float)STEERING_HOLD_PERIODS;
            }
            else if ((size < STEERING_CURVE_INTEGRAL) && !b_repeat)
            {
                p_steering->integral = 0.0f;
            }
            else
            {
                p_steering->integral *= decay;
            }

            p_steering->last_side     = (side > 0.0f) ? 1 : -1;
            p_steering->centred_count = 0.0f;
        }
        else if (p_steering->hold_count > 0.0f)
        {
            p_steering->hold_count -= step;
        }
        else
        {
            p_steering->integral *= decay;

            /* Only compared against STEERING_REPEAT_PERIODS: stop counting
             * well past it, so the float never loses precision. */
            if (p_steering->centred_count <=
                (float)STEERING_REPEAT_PERIODS + 1.0f)
            {
                p_steering->centred_count += step;
            }
        }

        p_steering->b_off_centre = false;
        return clamp_steer(p_steering->integral);
    }

    bool const b_crossing = ((p_steering->integral * error) < 0.0f);

    if ((!p_steering->b_off_centre) || b_crossing)
    {
        /* A new touch, on either side: start timing it. */
        p_steering->touch_count = 0.0f;
    }

    /* Only compared against STEERING_BEND_TOUCH_PERIODS: stop counting well
     * past it. */
    if (p_steering->touch_count <= (float)STEERING_BEND_TOUCH_PERIODS + 1.0f)
    {
        p_steering->touch_count += step;
    }

    p_steering->b_off_centre = true;
    /* A sensor has the line again; any hold has done its job. */
    p_steering->hold_count = 0.0f;

    /* Crossing the line: drop the opposite turn rather than unwind it. */
    if (b_crossing)
    {
        p_steering->integral = 0.0f;
    }

    p_steering->integral = clamp_steer(
        p_steering->integral + (STEERING_KI * error * period_sec * step));

    return clamp_steer((STEERING_KP * error) + p_steering->integral);
}

void line_track_reset (line_track_t * p_track)
{
    p_track->offset_mm   = 0.0f;
    p_track->heading_rad = 0.0f;
    p_track->integral    = 0.0f;
    p_track->since_mm    = 0.0f;
}

float line_track_update (line_track_t * p_track, bool b_left, bool b_right,
                         float left_mm, float right_mm)
{
    float const travel = 0.5f * (left_mm + right_mm);
    float const turn   = (right_mm - left_mm) / LINE_TRACK_TRACK_MM;

    /* Predict (small angles): turning left swings the sensors left, and
     * driving with the heading left of the line carries them left too, so
     * the line moves right relative to them. On a straight line the robot's
     * turn is its heading change across the line. */
    p_track->offset_mm -= (LINE_TRACK_LEAD_MM * turn) +
                          (travel * p_track->heading_rad);
    p_track->heading_rad += turn;
    p_track->since_mm += travel;

    /* Correct: the reading says which band the line centre is in. */
    bool  b_measured = true;
    float lowest     = 0.0f;
    float highest    = 0.0f;

    if (b_left && !b_right)
    {
        lowest  = LINE_TRACK_GAP_EDGE_MM;
        highest = LINE_TRACK_OUTER_EDGE_MM;
    }
    else if (b_right && !b_left)
    {
        lowest  = -LINE_TRACK_OUTER_EDGE_MM;
        highest = -LINE_TRACK_GAP_EDGE_MM;
    }
    else if ((!b_left) && (!b_right) &&
             (p_track->offset_mm <= LINE_TRACK_OUTER_EDGE_MM) &&
             (p_track->offset_mm >= -LINE_TRACK_OUTER_EDGE_MM))
    {
        /* Neither sees it and it was not left outside a sensor: in the
         * gap between them. */
        lowest  = -LINE_TRACK_GAP_EDGE_MM;
        highest = LINE_TRACK_GAP_EDGE_MM;
    }
    else
    {
        /* Both black (a junction or a bar), or lost outside a sensor: the
         * reading does not say where, so keep the prediction. */
        b_measured = false;
    }

    if (b_measured)
    {
        float const now_mm = p_track->offset_mm;
        float const seen   = (now_mm < lowest)    ? lowest
                             : (now_mm > highest) ? highest
                                                  : now_mm;
        float const error = seen - p_track->offset_mm;

        if (0.0f != error)
        {
            /* Further than predicted by error over the distance it built up
             * across: the heading was off by about that much. */
            float const span = (p_track->since_mm > LINE_TRACK_SPAN_MM)
                                   ? p_track->since_mm
                                   : LINE_TRACK_SPAN_MM;

            p_track->heading_rad -= (LINE_TRACK_BETA * error) / span;
            p_track->offset_mm = seen;
            p_track->since_mm  = 0.0f;
        }
    }

    p_track->heading_rad =
        clamp_float(p_track->heading_rad, LINE_TRACK_HEADING_MAX);
    p_track->integral = clamp_steer(
        p_track->integral + (LINE_TRACK_KI * p_track->offset_mm * travel));

    /* Steer towards the line, damped by the heading across it. */
    return clamp_steer((LINE_TRACK_KP * p_track->offset_mm) -
                       (LINE_TRACK_KD * p_track->heading_rad) +
                       p_track->integral);
}

static float clamp_float (float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

static float clamp_scale (float scale)
{
    if (scale < 0.0f)
    {
        return 0.0f;
    }

    if (scale > STEERING_SCALE_MAX)
    {
        return STEERING_SCALE_MAX;
    }

    return scale;
}

static float clamp_steer (float value)
{
    if (value > STEERING_MAX)
    {
        return STEERING_MAX;
    }

    if (value < -STEERING_MAX)
    {
        return -STEERING_MAX;
    }

    return value;
}

/*** end of file ***/

