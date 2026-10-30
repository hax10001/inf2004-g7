/** @file ultrasonic.c
 * @brief PWM echo capture with an elapsed-time timeout.
 */
#include "ultrasonic.h"
#include "obs_config.h"
#include "drivers/encoder.h"
#include <stdbool.h>
#include <bsp/libbsp.h>

#ifndef PWM_BASE
#define PWM_BASE 0x40050000U
#endif

#define US_SLICE_BASE \
    (PWM_BASE + (((BOARD_US_ECHO_PIN >> 1U) & 7U) * 0x14U))
#define US_CSR (US_SLICE_BASE + 0x00U)
#define US_DIV (US_SLICE_BASE + 0x04U)
#define US_CTR (US_SLICE_BASE + 0x08U)
#define US_TOP (US_SLICE_BASE + 0x10U)
#define US_LEVEL_MODE (1U << 4)
#define US_ENABLE 1U

/** @brief Generate the existing bounded trigger pulse. */
static void trigger (void);

ER ultrasonic_init (void)
{
    ER error = gpio_set_pin(BOARD_US_TRIG_PIN, GPIO_MODE_OUT);

    if (error >= E_OK)
    {
        gpio_set_val(BOARD_US_TRIG_PIN, 0U);
        error = pwm_set_pin(BOARD_US_ECHO_PIN);
    }
    if (error >= E_OK)
    {
        out_w(US_CSR, 0U);
        out_w(US_DIV, OBS_PWM_CLK_MHZ << 4U);
        out_w(US_TOP, 0xFFFFU);
        out_w(US_CTR, 0U);
        out_w(US_CSR, US_ENABLE | US_LEVEL_MODE);
    }
    return error;
}

ultrasonic_result_t ultrasonic_ping (int32_t * p_mm)
{
    if (p_mm == 0)
    {
        return ULTRASONIC_NO_ECHO;
    }
    *p_mm = 0;
    /* Do not attribute a preceding stuck-high echo to a new trigger. */
    if (gpio_get_val(BOARD_US_ECHO_PIN) != 0U)
    {
        return ULTRASONIC_OUT_OF_RANGE;
    }
    out_w(US_CTR, 0U);
    trigger();
    uint32_t const started = encoder_now_usec();
    uint32_t ticks = 0U;
    bool ended = false;

    while (((uint32_t)(encoder_now_usec() - started) <
            (OBS_ECHO_TIMEOUT_MSEC * 1000U)) && !ended)
    {
        tk_dly_tsk(1);
        ticks = in_w(US_CTR) & 0xFFFFU;
        ended = (ticks != 0U) &&
                (gpio_get_val(BOARD_US_ECHO_PIN) == 0U);
    }
    if (ticks == 0U)
    {
        return ULTRASONIC_NO_ECHO;
    }
    int32_t const mm = (int32_t)((ticks * 343U) / 2000U);

    if (!ended || (mm >= OBS_FAR_MM))
    {
        return ULTRASONIC_OUT_OF_RANGE;
    }
    *p_mm = mm;
    return ULTRASONIC_OK;
}

int32_t ultrasonic_median_of_three (int32_t first, int32_t second,
                                    int32_t third)
{
    if (first > second)
    {
        int32_t const value = first;
        first = second;
        second = value;
    }
    if (second > third)
    {
        second = third;
    }
    return (first > second) ? first : second;
}

static void trigger (void)
{
    gpio_set_val(BOARD_US_TRIG_PIN, 1U);
    uint32_t const started = encoder_now_usec();

    while ((uint32_t)(encoder_now_usec() - started) < OBS_TRIG_PULSE_USEC)
    {
        /* Existing bounded microsecond trigger; never wait for echo here. */
    }
    gpio_set_val(BOARD_US_TRIG_PIN, 0U);
}

/*** end of file ***/

