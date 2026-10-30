/** @file servo.c
 *
 * @brief 1 MHz PWM tick, 20 msec period, pulse width set by compare value.
 */
#include "servo.h"
#include "obs_config.h"
#include "obs_state.h"
#include <bsp/libbsp.h>

#ifndef PWM_BASE
#define PWM_BASE 0x40050000U
#endif

#define SERVO_SLICE_MASK 7U
#define SERVO_SLICE_STRIDE 0x14U
#define SERVO_SLICE_BASE                                                       \
    (PWM_BASE +                                                                \
     (((BOARD_SERVO_PIN >> 1U) & SERVO_SLICE_MASK) * SERVO_SLICE_STRIDE))
#define SERVO_REG_CSR (SERVO_SLICE_BASE + 0x00U)
#define SERVO_REG_DIV (SERVO_SLICE_BASE + 0x04U)
#define SERVO_WRAP 19999U
#define SERVO_DIV_INT_SHIFT 4U

/* Commanded range is -80..80 degrees; no physical position feedback. */
static int8_t g_angle_deg = 0;

/** @brief Bound a value between a low and a high limit. */
static int32_t clamp_int (int32_t value, int32_t low, int32_t high);

ER servo_init (void)
{
    ER error = pwm_set_pin(BOARD_SERVO_PIN);

    if (error >= E_OK)
    {
        out_w(SERVO_REG_CSR, 0U);
        out_w(SERVO_REG_DIV, OBS_PWM_CLK_MHZ << SERVO_DIV_INT_SHIFT);
        error = pwm_set_wrap(BOARD_SERVO_PIN, SERVO_WRAP);
    }

    if (error >= E_OK)
    {
        error = pwm_set_cc(BOARD_SERVO_PIN, OBS_SERVO_FRONT_USEC);

        if (error >= E_OK)
        {
            g_angle_deg = 0;
            error = pwm_set_enabled(BOARD_SERVO_PIN, TRUE);
        }
    }

    if (error < E_OK)
    {
        obs_state_faults_update(OBS_FAULT_SERVO, 0U);
    }

    return error;
}

uint32_t servo_set_angle (int32_t angle_deg)
{
    int32_t const angle =
        clamp_int(angle_deg, -OBS_SERVO_LIMIT_DEG, OBS_SERVO_LIMIT_DEG);
    int32_t const raw_pulse =
        OBS_SERVO_FRONT_USEC +
        (OBS_SERVO_LEFT_SIGN * angle * OBS_SERVO_USEC_PER_DEG);
    int32_t const pulse =
        clamp_int(raw_pulse, OBS_SERVO_MIN_USEC, OBS_SERVO_MAX_USEC);
    int32_t const moved = clamp_int(
        angle - g_angle_deg, -2 * OBS_SERVO_LIMIT_DEG, 2 * OBS_SERVO_LIMIT_DEG);
    int32_t const travel = (moved < 0) ? -moved : moved;
    int32_t const settle =
        OBS_SERVO_SETTLE_BASE_MSEC + (travel * OBS_SERVO_SETTLE_PER_DEG_MSEC);

    /* The pulse is clamped to a positive range, so the cast is exact. */
    if (pwm_set_cc(BOARD_SERVO_PIN, (UW)pulse) < E_OK)
    {
        obs_state_faults_update(OBS_FAULT_SERVO, 0U);
    }
    else
    {
        /* Angle is clamped to +/-80, within the signed eight-bit range. */
        g_angle_deg = (int8_t)angle;
    }

    /* The settle time is a sum of non-negative terms, so the cast is exact. */
    return (uint32_t)settle;
}

static int32_t clamp_int (int32_t value, int32_t low, int32_t high)
{
    int32_t result = value;

    if (value < low)
    {
        result = low;
    }
    else if (value > high)
    {
        result = high;
    }
    else
    {
        /* The value is already inside the limits. */
    }

    return result;
}

/*** end of file ***/

