/** @file ultrasonic.c
 * @brief ECHO is gated into a PWM counter, so no GPIO interrupt is used and
 * the encoder driver's shared interrupt stays untouched.
 *
 * Distance (mm) = pulse (us) x 343 / 2000, i.e. 343 m/s, out and back.
 */
#include "ultrasonic.h"
#include "obs_config.h"
#include "drivers/encoder.h"
#include <stdbool.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#ifndef PWM_BASE
#define PWM_BASE 0x40050000U /* RP2040 PWM block, if the BSP header lacks it. */
#endif

#define US_SLICE ((UW)((BOARD_US_ECHO_PIN >> 1U) & 7U))
#define US_REG(offset) (PWM_BASE + (US_SLICE * 0x14U) + (offset))
#define US_REG_CSR 0x00U
#define US_REG_DIV 0x04U
#define US_REG_CTR 0x08U
#define US_REG_TOP 0x10U
#define US_CSR_EN 0x01U
#define US_CSR_DIVMODE_LEVEL (1U << 4)
#define US_DIV_INT_SHIFT 4U
#define US_COUNT_MASK 0xFFFFU
#define US_SOUND_NUM 343U
#define US_SOUND_DEN 2000U

/** @brief Raise TRIG for OBS_TRIG_PULSE_USEC; the only busy wait (12 us). */
static void send_trigger (void);
/** @brief Convert counted microseconds to millimetres. */
static uint32_t ticks_to_mm (UW ticks);

ER ultrasonic_init (void)
{
    ER error = gpio_set_pin(BOARD_US_TRIG_PIN, GPIO_MODE_OUT);

    if (error < E_OK)
    {
        return error;
    }

    (void)gpio_set_val(BOARD_US_TRIG_PIN, 0U);
    error = pwm_set_pin(BOARD_US_ECHO_PIN);

    if (error < E_OK)
    {
        return error;
    }

    out_w(US_REG(US_REG_CSR), 0U);
    out_w(US_REG(US_REG_DIV), (UW)(OBS_PWM_CLK_MHZ << US_DIV_INT_SHIFT));
    out_w(US_REG(US_REG_TOP), US_COUNT_MASK);
    out_w(US_REG(US_REG_CTR), 0U);
    /* Level mode: the counter only advances while ECHO is high. */
    out_w(US_REG(US_REG_CSR), US_CSR_EN | US_CSR_DIVMODE_LEVEL);
    return E_OK;
}

us_result_t ultrasonic_ping (uint16_t * p_mm)
{
    UW ticks = 0U;

    out_w(US_REG(US_REG_CTR), 0U);
    send_trigger();

    for (uint32_t waited = 0U; waited < OBS_ECHO_TIMEOUT_MS; waited++)
    {
        (void)tk_dly_tsk(1);
        ticks = in_w(US_REG(US_REG_CTR)) & US_COUNT_MASK;

        if ((ticks > 0U) && (0U == gpio_get_val(BOARD_US_ECHO_PIN)))
        {
            uint32_t const mm = ticks_to_mm(ticks);

            if (mm >= OBS_FAR_MM)
            {
                return US_OUT_OF_RANGE;
            }

            *p_mm = (uint16_t)mm;
            return US_OK;
        }
    }

    return (ticks > 0U) ? US_OUT_OF_RANGE : US_NO_ECHO;
}

uint16_t ultrasonic_median3 (uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b)
    {
        uint16_t const t = a;
        a                = b;
        b                = t;
    }

    if (b > c)
    {
        b = c;
    }

    return (a > b) ? a : b;
}

static void send_trigger (void)
{
    (void)gpio_set_val(BOARD_US_TRIG_PIN, 1U);
    uint32_t const start = encoder_now_usec();

    while ((uint32_t)(encoder_now_usec() - start) < OBS_TRIG_PULSE_USEC)
    {
    }

    (void)gpio_set_val(BOARD_US_TRIG_PIN, 0U);
}

static uint32_t ticks_to_mm (UW ticks)
{
    return ((uint32_t)ticks * US_SOUND_NUM) / US_SOUND_DEN;
}

/*** end of file ***/