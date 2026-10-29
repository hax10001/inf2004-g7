/** @file servo.c
 * @brief 1 MHz PWM tick, 20 ms period, pulse width set by compare value.
 */
#include "servo.h"
#include "obs_config.h"
#include <bsp/libbsp.h>

#ifndef PWM_BASE
#define PWM_BASE 0x40050000U
#endif

#define SERVO_SLICE ((UW)((BOARD_SERVO_PIN >> 1U) & 7U))
#define SERVO_REG(offset) (PWM_BASE + (SERVO_SLICE * 0x14U) + (offset))
#define SERVO_REG_CSR 0x00U
#define SERVO_REG_DIV 0x04U
#define SERVO_WRAP 19999U /* 20 ms at a 1 us tick. */
#define SERVO_DIV_INT_SHIFT 4U

static int16_t g_angle_deg = 0;

ER servo_init (void)
{
    ER error = pwm_set_pin(BOARD_SERVO_PIN);

    if (error < E_OK)
    {
        return error;
    }

    out_w(SERVO_REG(SERVO_REG_CSR), 0U);
    out_w(SERVO_REG(SERVO_REG_DIV), (UW)(OBS_PWM_CLK_MHZ << SERVO_DIV_INT_SHIFT));
    error = pwm_set_wrap(BOARD_SERVO_PIN, SERVO_WRAP);

    if (error < E_OK)
    {
        return error;
    }

    (void)servo_set_angle(0);
    return pwm_set_enabled(BOARD_SERVO_PIN, TRUE);
}

uint32_t servo_set_angle (int16_t angle_deg)
{
    if (angle_deg > OBS_SERVO_LIMIT_DEG)
    {
        angle_deg = OBS_SERVO_LIMIT_DEG;
    }

    if (angle_deg < -OBS_SERVO_LIMIT_DEG)
    {
        angle_deg = -OBS_SERVO_LIMIT_DEG;
    }

    int32_t pulse = OBS_SERVO_FRONT_US + ((int32_t)OBS_SERVO_LEFT_SIGN *
                                          (int32_t)angle_deg *
                                          (int32_t)OBS_SERVO_US_PER_DEG);

    if (pulse < OBS_SERVO_MIN_US)
    {
        pulse = OBS_SERVO_MIN_US;
    }

    if (pulse > OBS_SERVO_MAX_US)
    {
        pulse = OBS_SERVO_MAX_US;
    }

    (void)pwm_set_cc(BOARD_SERVO_PIN, (UW)pulse);

    int32_t delta = (int32_t)angle_deg - (int32_t)g_angle_deg;

    if (delta < 0)
    {
        delta = -delta;
    }

    g_angle_deg = angle_deg;
    return OBS_SERVO_SETTLE_BASE_MS +
           ((uint32_t)delta * OBS_SERVO_SETTLE_PER_DEG_MS);
}

/*** end of file ***/