/** @file downhill.c
 * @brief Deterministic calibrated-pitch governor and overspeed latch.
 */
#include "downhill.h"
#include "app/app_config.h"
#include <stddef.h>

#define DOWNHILL_ENGAGE_X10 50
#define DOWNHILL_RELEASE_X10 30
#define DOWNHILL_MAX_ROLL_X10 80
#define DOWNHILL_SCALE_STEP 5U
#define DOWNHILL_ENGAGE_COUNT 3U
#define DOWNHILL_RELEASE_COUNT 4U
/* 20 controller cycles ~= 1s. Brief IMU gaps retain the restricted
 * target instead of causing repeated stop/restart during ramp travel. */
#define DOWNHILL_STALE_COUNT 20U
/* A true runaway is not an ordinary overshoot above the derated target.
 * The PI loop handles ordinary speed error without PWM zero/on pulsing. */
#define DOWNHILL_OVERSPEED_COUNT 10U
#define DOWNHILL_RUNAWAY_RPM APP_DOWNHILL_RUNAWAY_RPM

void downhill_reset (downhill_governor_t * p_state)
{
    if (p_state != NULL)
    {
        *p_state = (downhill_governor_t){0};
        p_state->scale_percent = 100U;
    }
}

static void sample_terrain (downhill_governor_t * p_state,
                            float pitch_deg, float roll_deg,
                            bool fresh, bool moving, bool descending,
                            bool terrain_input)
{
    if (p_state == NULL || p_state->fault)
    {
        return;
    }
    /* Sign 0 is intentional: no ramp evidence, no unverified derating. */
    if (APP_DOWNHILL_NOSE_DOWN_SIGN == 0 && !terrain_input)
    {
        downhill_reset(p_state);
        return;
    }
    if (!fresh || !(pitch_deg >= -90.0f && pitch_deg <= 90.0f) ||
        !(roll_deg >= -90.0f && roll_deg <= 90.0f))
    {
        if (moving)
        {
            if (p_state->stale < DOWNHILL_STALE_COUNT)
            {
                ++p_state->stale;
            }
            if (p_state->stale >= DOWNHILL_STALE_COUNT)
            {
                p_state->fault = 1U;
            }
        }
        else
        {
            p_state->stale = 0U;
        }
        return;
    }
    p_state->stale = 0U;
    /* The input is checked before converting to the 11-bit signed value. */
    int32_t const sample =
        (int32_t)(pitch_deg * (float)(10 * APP_DOWNHILL_NOSE_DOWN_SIGN));
    int32_t const roll = (int32_t)(roll_deg * 10.0f);
    if (!p_state->initialized)
    {
        p_state->filtered_pitch_x10 = (int)sample;
        p_state->initialized = 1U;
    }
    else
    {
        int32_t const prior = p_state->filtered_pitch_x10;
        p_state->filtered_pitch_x10 =
            (int)(prior + ((sample - prior) / 4));
    }
    if (!p_state->engaged)
    {
        /* One-wheel hump roll does not establish a downhill ramp. */
        if (descending || (APP_DOWNHILL_NOSE_DOWN_SIGN != 0 &&
            p_state->filtered_pitch_x10 >= DOWNHILL_ENGAGE_X10 &&
            roll > -DOWNHILL_MAX_ROLL_X10 &&
            roll < DOWNHILL_MAX_ROLL_X10))
        {
            if (p_state->confirm < DOWNHILL_ENGAGE_COUNT)
            {
                ++p_state->confirm;
            }
            /* The hump state already includes its own confirmation. */
            if (descending || p_state->confirm >= DOWNHILL_ENGAGE_COUNT)
            {
                p_state->engaged = 1U;
                p_state->release = 0U;
            }
        }
        else
        {
            p_state->confirm = 0U;
        }
    }
    else
    {
        if (!descending && (APP_DOWNHILL_NOSE_DOWN_SIGN == 0 ||
            p_state->filtered_pitch_x10 <= DOWNHILL_RELEASE_X10))
        {
            if (p_state->release < DOWNHILL_RELEASE_COUNT)
            {
                ++p_state->release;
            }
            if (p_state->release >= DOWNHILL_RELEASE_COUNT)
            {
                p_state->engaged = 0U;
                p_state->confirm = 0U;
            }
        }
        else
        {
            p_state->release = 0U;
        }
    }
    /* Both entering and leaving a ramp are continuous reductions of
     * forward RPM; never convert a slope transition into a stop pulse. */
    unsigned int desired = p_state->engaged ?
        APP_DOWNHILL_REDUCED_PERCENT : 100U;
    if (p_state->scale_percent > desired)
    {
        unsigned int const gap = p_state->scale_percent - desired;
        p_state->scale_percent -= (gap > DOWNHILL_SCALE_STEP) ?
            DOWNHILL_SCALE_STEP : gap;
    }
    else if (p_state->scale_percent < desired)
    {
        unsigned int const gap = desired - p_state->scale_percent;
        p_state->scale_percent += (gap > DOWNHILL_SCALE_STEP) ?
            DOWNHILL_SCALE_STEP : gap;
    }
}

void downhill_sample (downhill_governor_t * p_state,
                      float pitch_deg, float roll_deg,
                      bool fresh, bool moving)
{
    sample_terrain(p_state, pitch_deg, roll_deg, fresh, moving, false, false);
}

void downhill_sample_terrain (downhill_governor_t * p_state,
                              float pitch_deg, float roll_deg,
                              bool fresh, bool moving, bool descending)
{
    sample_terrain(p_state, pitch_deg, roll_deg, fresh, moving,
                   descending, true);
}

void downhill_scale_pair (downhill_governor_t const * p_state,
                          float target_left, float target_right,
                          float * p_left, float * p_right)
{
    if (p_left == NULL || p_right == NULL)
    {
        return;
    }
    float const scale = (p_state == NULL) ? 1.0f :
        (p_state->fault ? 0.0f :
         ((float)p_state->scale_percent / 100.0f));
    *p_left = target_left * scale;
    *p_right = target_right * scale;
}

bool downhill_wheel_over_target (downhill_governor_t const * p_state,
                                 float target, float measured)
{
    /* Zero duty only at a genuine runaway rate. Minor above-target
     * readings are handled by the PI trim; a 1 RPM error must not make
     * the motors alternate OFF/ON every controller tick. */
    return p_state != NULL && p_state->scale_percent < 100U &&
           target >= 5.0f && measured > DOWNHILL_RUNAWAY_RPM;
}

bool downhill_observe_speed (downhill_governor_t * p_state,
                             float target_left, float target_right,
                             float measured_left, float measured_right)
{
    if (p_state == NULL || p_state->fault)
    {
        return p_state != NULL && p_state->fault;
    }
    if (p_state->scale_percent == 100U)
    {
        p_state->overspeed = 0U;
        return false;
    }
    bool const left = target_left >= 5.0f &&
        measured_left > DOWNHILL_RUNAWAY_RPM;
    bool const right = target_right >= 5.0f &&
        measured_right > DOWNHILL_RUNAWAY_RPM;
    if (left || right)
    {
        if (p_state->overspeed < DOWNHILL_OVERSPEED_COUNT)
        {
            ++p_state->overspeed;
        }
        if (p_state->overspeed >= DOWNHILL_OVERSPEED_COUNT)
        {
            p_state->fault = 1U;
        }
    }
    else
    {
        p_state->overspeed = 0U;
    }
    return p_state->fault != 0U;
}

/*** end of file ***/

