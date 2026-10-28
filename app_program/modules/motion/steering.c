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

/** @brief Bound a value to plus or minus STEERING_MAX. */
static float clamp_steer (float value);

void steering_reset (steering_t * p_steering)
{
    p_steering->integral     = 0.0f;
    p_steering->b_off_centre = false;
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

    if (0.0f == error)
    {
        float const size = (p_steering->integral < 0.0f)
                               ? -p_steering->integral
                               : p_steering->integral;

        if (p_steering->b_off_centre && (size < STEERING_CURVE_INTEGRAL))
        {
            p_steering->integral = 0.0f;
        }
        else
        {
            p_steering->integral *= STEERING_DECAY;
        }

        p_steering->b_off_centre = false;
        return clamp_steer(p_steering->integral);
    }

    p_steering->b_off_centre = true;

    /* Crossing the line: drop the opposite turn rather than unwind it. */
    if ((p_steering->integral * error) < 0.0f)
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

