/** @file lift_math.c
 * @brief Double integration of vertical acceleration, bounded against drift.
 * @note Uses no math library and no kernel types.
 */
#include "lift_math.h"
#include <stddef.h>

/* One mg of acceleration, in mm/s^2. */
#define LIFT_MATH_MM_S2_PER_MG 9.80665F
#define LIFT_MATH_MSEC_PER_SEC 1000.0F

/** @brief Track the rest window; true once the readings have stayed still. */
static bool update_still (lift_math_state_t * p_state, int16_t excess_mg);
/** @brief Integrate one sample of net acceleration into speed and height. */
static void integrate (lift_math_state_t * p_state, int32_t net_mg,
                       uint32_t elapsed_msec);

void lift_math_reset (lift_math_state_t * p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    /* The learned resting offset is kept: it describes the sensor. */
    p_state->velocity_mm_s = 0.0F;
    p_state->height_mm     = 0.0F;
    p_state->start_count   = 0U;
    p_state->still_count   = 0U;
    p_state->floor_count   = 0U;
}

uint16_t lift_math_step (lift_math_state_t * p_state, int16_t excess_mg,
                         uint32_t elapsed_msec, bool b_floor)
{
    if (NULL == p_state)
    {
        return 0U;
    }

    if ((0U == elapsed_msec) || (elapsed_msec > LIFT_MATH_MAX_STEP_MSEC))
    {
        /* Missed samples: the speed in between is unknown. */
        lift_math_reset(p_state);
        return 0U;
    }

    if (!b_floor)
    {
        p_state->floor_count = 0U;
    }
    else if (p_state->floor_count < LIFT_MATH_FLOOR_SAMPLES)
    {
        p_state->floor_count++;
    }
    else
    {
        /* Already confirmed; the count only needs to reach the limit. */
    }

    bool const b_still = update_still(p_state, excess_mg);
    /* int16_t minus int16_t always fits int32_t. */
    int32_t net_mg = (int32_t)excess_mg - (int32_t)p_state->bias_mg;

    if (!lift_math_is_active(p_state))
    {
        if (net_mg > LIFT_MATH_START_MG)
        {
            /* Integrate from the first sample of the push, so the speed
             * gained before the lift is confirmed is not lost. */
            integrate(p_state, net_mg, elapsed_msec);
            p_state->start_count++;

            if (p_state->start_count >= LIFT_MATH_START_SAMPLES)
            {
                /* Floor seen before the lift does not end it at once. */
                p_state->floor_count = 0U;
            }
        }
        else
        {
            p_state->start_count   = 0U;
            p_state->velocity_mm_s = 0.0F;
            p_state->height_mm     = 0.0F;
        }
    }
    else if (p_state->floor_count >= LIFT_MATH_FLOOR_SAMPLES)
    {
        /* Standing on the floor again (or it never left it). */
        uint8_t const floor_count = (uint8_t)p_state->floor_count;

        lift_math_reset(p_state);
        p_state->floor_count = floor_count;
    }
    else
    {
        if ((net_mg > -LIFT_MATH_DEADBAND_MG) &&
            (net_mg < LIFT_MATH_DEADBAND_MG))
        {
            net_mg = 0;
        }

        integrate(p_state, net_mg, elapsed_msec);

        if (b_still)
        {
            /* Held still: any remaining speed is integration error. */
            p_state->velocity_mm_s = 0.0F;
        }
    }

    if (b_floor)
    {
        /* A line sensor sees the floor: whatever the integration says, the
         * robot is not up. Speed is kept (see LIFT_MATH_FLOOR_SAMPLES). */
        p_state->height_mm = 0.0F;
    }

    /* The height is bounded to 0..LIFT_MATH_MAX_MM, so it fits uint16_t. */
    return (uint16_t)(p_state->height_mm + 0.5F);
}

bool lift_math_is_still (lift_math_state_t const * p_state)
{
    return (NULL != p_state) &&
           (p_state->still_count >= LIFT_MATH_STILL_SAMPLES);
}

bool lift_math_is_active (lift_math_state_t const * p_state)
{
    return (NULL != p_state) &&
           (p_state->start_count >= LIFT_MATH_START_SAMPLES);
}

static bool update_still (lift_math_state_t * p_state, int16_t excess_mg)
{
    if (0U == p_state->still_count)
    {
        p_state->still_min_mg = excess_mg;
        p_state->still_max_mg = excess_mg;
        p_state->still_count  = 1U;
        return false;
    }

    if (excess_mg < p_state->still_min_mg)
    {
        p_state->still_min_mg = excess_mg;
    }

    if (excess_mg > p_state->still_max_mg)
    {
        p_state->still_max_mg = excess_mg;
    }

    int32_t const spread =
        (int32_t)p_state->still_max_mg - (int32_t)p_state->still_min_mg;

    if (spread > LIFT_MATH_STILL_BAND_MG)
    {
        /* Moved: start a new window from this reading. */
        p_state->still_min_mg = excess_mg;
        p_state->still_max_mg = excess_mg;
        p_state->still_count  = 1U;
        return false;
    }

    if (p_state->still_count < LIFT_MATH_STILL_SAMPLES)
    {
        p_state->still_count++;
        return false;
    }

    /* The middle of two int16_t values fits int16_t. */
    p_state->bias_mg = (int16_t)(((int32_t)p_state->still_min_mg +
                                  (int32_t)p_state->still_max_mg) / 2);
    return true;
}

static void integrate (lift_math_state_t * p_state, int32_t net_mg,
                       uint32_t elapsed_msec)
{
    /* Both conversions are exact: |net_mg| < 2^16 and elapsed <= 100. */
    float const step_sec = (float)elapsed_msec / LIFT_MATH_MSEC_PER_SEC;
    float const accel    = (float)net_mg * LIFT_MATH_MM_S2_PER_MG;

    p_state->velocity_mm_s += accel * step_sec;
    p_state->height_mm     += p_state->velocity_mm_s * step_sec;

    if (p_state->height_mm <= 0.0F)
    {
        /* The floor: it cannot sink, and cannot keep falling. */
        p_state->height_mm = 0.0F;

        if (p_state->velocity_mm_s < 0.0F)
        {
            p_state->velocity_mm_s = 0.0F;
        }
    }
    else if (p_state->height_mm > (float)LIFT_MATH_MAX_MM)
    {
        p_state->height_mm = (float)LIFT_MATH_MAX_MM;
    }
    else
    {
        /* Within range. */
    }
}

/*** end of file ***/

