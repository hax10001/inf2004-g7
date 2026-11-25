/** @file downhill.c
 * @brief Deterministic calibrated-pitch governor and overspeed latch.
 */
#include "downhill.h"
#include "app/app_config.h"
#include <stddef.h>

/* Filtered nose-down pitch (deci-degrees) that engages / releases the
 * governor; the gap is hysteresis. */
#define DOWNHILL_ENGAGE_X10 50
#define DOWNHILL_RELEASE_X10 30
/* Roll beyond +/- 8.0 degrees is a one-wheel hump, not a ramp. */
#define DOWNHILL_MAX_ROLL_X10 80
/* Largest change in the speed scale per sample, percent. */
#define DOWNHILL_SCALE_STEP 5U
/* Consecutive samples needed to engage / release. */
#define DOWNHILL_ENGAGE_COUNT 3U
#define DOWNHILL_RELEASE_COUNT 4U
/* Missing IMU samples while moving before the fault latches (20 controller
 * periods, ~1 s). Brief gaps keep the reduced target instead of stopping. */
#define DOWNHILL_STALE_COUNT 20U
#define DOWNHILL_RUNAWAY_RPM APP_DOWNHILL_RUNAWAY_RPM /* Overspeed, RPM. */
/* Percent to fraction divisor. */
#define DOWNHILL_PERCENT_SCALE 100.0f
/* Pitch or roll outside +/- this many degrees is not a valid reading. */
#define DOWNHILL_TILT_LIMIT_DEG 90.0f
/* Degrees to tenths of a degree (integer and float). */
#define DOWNHILL_X10 10
#define DOWNHILL_X10_F 10.0f
/* Pitch filter: move a quarter of the way to each new sample. */
#define DOWNHILL_FILTER_DIV 4
/* Wheel targets below this RPM cannot show a runaway. */
#define DOWNHILL_MIN_TARGET_RPM 5.0f

/** @brief Reset the downhill governor. */
void downhill_reset (downhill_governor_t * p_state)
{
    if (p_state != NULL)
    {
        *p_state = (downhill_governor_t){0};
        p_state->scale_percent = DOWNHILL_FULL_PERCENT;
    }
}

/** @brief Update the governor from one terrain sample. */
static void sample_terrain (downhill_governor_t * p_state,
                            float pitch_deg, float roll_deg,
                            bool fresh, bool moving, bool descending,
                            bool terrain_input)
{
    if (p_state == NULL || p_state->fault)
    {
        return;
    }
    /* Sign 0 means unverified pitch sign: pitch alone never derates. */
    if (APP_DOWNHILL_NOSE_DOWN_SIGN == 0 && !terrain_input)
    {
        downhill_reset(p_state);
        return;
    }
    if (!fresh || !(pitch_deg >= -DOWNHILL_TILT_LIMIT_DEG &&
                    pitch_deg <= DOWNHILL_TILT_LIMIT_DEG) ||
        !(roll_deg >= -DOWNHILL_TILT_LIMIT_DEG &&
          roll_deg <= DOWNHILL_TILT_LIMIT_DEG))
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
        (int32_t)(pitch_deg *
                  (float)(DOWNHILL_X10 * APP_DOWNHILL_NOSE_DOWN_SIGN));
    int32_t const roll = (int32_t)(roll_deg * DOWNHILL_X10_F);
    if (!p_state->initialized)
    {
        p_state->filtered_pitch_x10 = (int32_t)sample;
        p_state->initialized = 1U;
    }
    else
    {
        int32_t const prior = p_state->filtered_pitch_x10;
        p_state->filtered_pitch_x10 =
            (int32_t)(prior + ((sample - prior) / DOWNHILL_FILTER_DIV));
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
    /* Entering and leaving a ramp change the speed gradually; a slope
     * transition never becomes a stop pulse. */
    uint32_t desired = p_state->engaged ?
        APP_DOWNHILL_REDUCED_PERCENT : DOWNHILL_FULL_PERCENT;
    if (p_state->scale_percent > desired)
    {
        uint32_t const gap = p_state->scale_percent - desired;
        p_state->scale_percent -= (gap > DOWNHILL_SCALE_STEP) ?
            DOWNHILL_SCALE_STEP : gap;
    }
    else if (p_state->scale_percent < desired)
    {
        uint32_t const gap = desired - p_state->scale_percent;
        p_state->scale_percent += (gap > DOWNHILL_SCALE_STEP) ?
            DOWNHILL_SCALE_STEP : gap;
    }
}

/** @brief Update the governor from pitch and roll. */
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

/** @brief Scale both wheel targets by the governor. */
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
         ((float)p_state->scale_percent / DOWNHILL_PERCENT_SCALE));
    *p_left = target_left * scale;
    *p_right = target_right * scale;
}

/** @brief Return true if a wheel runs over its target. */
bool downhill_wheel_over_target (downhill_governor_t const * p_state,
                                 float target, float measured)
{
    /* Only a genuine runaway brakes; small overshoots are left to the PI
     * trim so the motors do not pulse on and off every period. */
    return p_state != NULL && p_state->scale_percent < DOWNHILL_FULL_PERCENT &&
           target >= DOWNHILL_MIN_TARGET_RPM && measured > DOWNHILL_RUNAWAY_RPM;
}

/** @brief Count overspeed periods on a descent. */
bool downhill_observe_speed (downhill_governor_t * p_state,
                             float target_left, float target_right,
                             float measured_left, float measured_right)
{
    if (p_state == NULL || p_state->fault)
    {
        return p_state != NULL && p_state->fault;
    }
    if (p_state->scale_percent == DOWNHILL_FULL_PERCENT)
    {
        p_state->overspeed = 0U;
        return false;
    }
    bool const left = target_left >= DOWNHILL_MIN_TARGET_RPM &&
        measured_left > DOWNHILL_RUNAWAY_RPM;
    bool const right = target_right >= DOWNHILL_MIN_TARGET_RPM &&
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

