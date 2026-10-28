/** @file steering.c
 * @brief Turn harder the longer the line stays off-centre, then ease off.
 *
 * Two digital IR sensors give only left, centre or right, so the error is
 * -1, 0 or +1. A derivative term would see only steps; the integral is what
 * grades the response, and its decay avoids snapping straight on the line.
 */
#include "steering.h"

/* Steer for any off-centre reading, and its growth per second. */
#define STEERING_KP 0.3f
#define STEERING_KI 2.0f
/* Fraction of the integral kept per centred period: lower recentres faster. */
#define STEERING_DECAY 0.85f
/* A touch that built less integral than this was a nudge on a straight, so
 * the turn ends with it; keeping it would overshoot to the other edge. A
 * longer touch is a curve, and its turn is kept to carry through the bend.
 * At STEERING_KI 2.0 and 50 ms periods, 0.35 is about four periods.
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
 * three periods, so after a bend the window closes; letting resumed turns
 * restart it kept straights pivoting at every nudge.
 */
#define STEERING_BEND_TOUCH_PERIODS 5U

/** @brief Bound a value to plus or minus STEERING_MAX. */
static float clamp_steer (float value);

void steering_reset (steering_t * p_steering)
{
    p_steering->integral      = 0.0f;
    p_steering->b_off_centre  = false;
    p_steering->last_side     = 0.0f;
    p_steering->centred_count = 0U;
    p_steering->hold_count    = 0U;
    p_steering->bend_side     = 0.0f;
    p_steering->bend_age      = STEERING_BEND_MEMORY_PERIODS + 1U;
    p_steering->touch_count   = 0U;
}

float steering_update (steering_t * p_steering, line_follow_action_t action,
                       float period_sec)
{
    float error = 0.0f;

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

    if (p_steering->bend_age <= STEERING_BEND_MEMORY_PERIODS)
    {
        p_steering->bend_age++;
    }

    if (0.0f == error)
    {
        if (p_steering->b_off_centre)
        {
            /* A touch just ended; the integral's sign is its side. */
            float const side = (p_steering->integral < 0.0f) ? -1.0f : 1.0f;
            float const size = side * p_steering->integral;
            bool const  b_repeat =
                (side == p_steering->last_side) &&
                (p_steering->centred_count <= STEERING_REPEAT_PERIODS);
            bool const  b_bend =
                (side == p_steering->bend_side) &&
                (p_steering->bend_age <= STEERING_BEND_MEMORY_PERIODS);

            if (size >= STEERING_MAX)
            {
                /* Full-strength turn: keep it while the line is found. */
                p_steering->hold_count = STEERING_HOLD_PERIODS;

                if (p_steering->touch_count >= STEERING_BEND_TOUCH_PERIODS)
                {
                    p_steering->bend_side = side;
                    p_steering->bend_age  = 0U;
                }
            }
            else if (b_bend)
            {
                /* The tight bend goes on: resume the full turn. */
                p_steering->integral   = side * STEERING_MAX;
                p_steering->hold_count = STEERING_HOLD_PERIODS;
            }
            else if ((size < STEERING_CURVE_INTEGRAL) && !b_repeat)
            {
                p_steering->integral = 0.0f;
            }
            else
            {
                p_steering->integral *= STEERING_DECAY;
            }

            p_steering->last_side     = side;
            p_steering->centred_count = 0U;
        }
        else if (p_steering->hold_count > 0U)
        {
            p_steering->hold_count--;
        }
        else
        {
            p_steering->integral *= STEERING_DECAY;

            if (p_steering->centred_count < UINT32_MAX)
            {
                p_steering->centred_count++;
            }
        }

        p_steering->b_off_centre = false;
        return clamp_steer(p_steering->integral);
    }

    bool const b_crossing = ((p_steering->integral * error) < 0.0f);

    if ((!p_steering->b_off_centre) || b_crossing)
    {
        /* A new touch, on either side: start timing it. */
        p_steering->touch_count = 0U;
    }

    if (p_steering->touch_count < UINT32_MAX)
    {
        p_steering->touch_count++;
    }

    p_steering->b_off_centre = true;
    /* A sensor has the line again; any hold has done its job. */
    p_steering->hold_count = 0U;

    /* Crossing the line: drop the opposite turn rather than unwind it. */
    if (b_crossing)
    {
        p_steering->integral = 0.0f;
    }

    p_steering->integral =
        clamp_steer(p_steering->integral + (STEERING_KI * error * period_sec));

    return clamp_steer((STEERING_KP * error) + p_steering->integral);
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

