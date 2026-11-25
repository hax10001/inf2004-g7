/** @file ultrasonic.c
 *
 * @brief ECHO is gated into a PWM counter, so no GPIO interrupt is used.
 *
 * NOTE: Distance in mm is the pulse width in usec times 343, divided by
 * 2000, because sound travels out and back.
 */
#include "ultrasonic.h"
#include "obs_config.h"
#include "drivers/encoder.h"
#include <stdbool.h>
#include <bsp/libbsp.h>

#ifndef PWM_BASE
#define PWM_BASE 0x40050000U
#endif

#define US_SLICE_MASK 7U
#define US_SLICE_STRIDE 0x14U
#define US_SLICE_BASE                                                          \
    (PWM_BASE + (((BOARD_US_ECHO_PIN >> 1U) & US_SLICE_MASK) * US_SLICE_STRIDE))
#define US_REG_CSR (US_SLICE_BASE + 0x00U)
#define US_REG_DIV (US_SLICE_BASE + 0x04U)
#define US_REG_CTR (US_SLICE_BASE + 0x08U)
#define US_REG_TOP (US_SLICE_BASE + 0x10U)
#define US_CSR_EN 0x01U
#define US_CSR_DIVMODE_LEVEL (1U << 4)
#define US_DIV_INT_SHIFT 4U
#define US_COUNT_MASK 0xFFFFU
#define US_SOUND_NUM 343
#define US_SOUND_DEN 2000

/** @brief Wait up to OBS_ECHO_CLEAR_MSEC for ECHO to be low.
 *  @return false if ECHO is still high: the last pulse has not ended.
 */
static bool wait_echo_low (void);
/* Kernel ms of the last trigger, for ping spacing. Ultrasonic owner task
 * only (the obstacle worker), so no lock is needed. */
static uint32_t g_last_trigger_msec = 0U;
static uint8_t  g_last_trigger_valid = 0U;

/** @brief Raise TRIG for OBS_TRIG_PULSE_USEC; the only busy wait. */
static void send_trigger (void);
/** @brief Convert counted microseconds to millimetres. */
static int32_t ticks_to_mm (UW ticks);

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
        out_w(US_REG_CSR, 0U);
        out_w(US_REG_DIV, OBS_PWM_CLK_MHZ << US_DIV_INT_SHIFT);
        out_w(US_REG_TOP, US_COUNT_MASK);
        out_w(US_REG_CTR, 0U);
        /* Level mode: the counter only advances while ECHO is high. */
        out_w(US_REG_CSR, US_CSR_EN | US_CSR_DIVMODE_LEVEL);
    }

    return error;
}

ultrasonic_result_t ultrasonic_ping (int32_t * p_mm)
{
    ultrasonic_result_t result  = ULTRASONIC_NO_ECHO;
    UW                  ticks   = 0U;
    bool                b_ended = false;
    /* Some HC-SR04 clones hold ECHO high long after a missed echo; a
     * trigger then would time the tail of the old pulse. */
    bool const          b_clear = wait_echo_low();

    if (b_clear)
    {
        out_w(US_REG_CTR, 0U);
        send_trigger();
        {
            SYSTIM now = {0};

            g_last_trigger_valid = (tk_get_otm(&now) >= E_OK) ? 1U : 0U;
            g_last_trigger_msec  = now.lo;
        }

        for (uint32_t waited = 0U;
             (waited < OBS_ECHO_TIMEOUT_MSEC) && (!b_ended); waited++)
        {
            tk_dly_tsk(1);
            ticks = in_w(US_REG_CTR) & US_COUNT_MASK;

            if ((ticks > 0U) && (0U == gpio_get_val(BOARD_US_ECHO_PIN)))
            {
                b_ended = true;
            }
        }
    }

    if (!b_clear)
    {
        /* ECHO stuck high: the sensor is still busy or faulty. */
        result = ULTRASONIC_NO_ECHO;
    }
    else if (ticks > 0U)
    {
        int32_t const range_mm = ticks_to_mm(ticks);

        if (b_ended && (range_mm < OBS_FAR_MM))
        {
            *p_mm  = range_mm;
            result = ULTRASONIC_OK;
        }
        else
        {
            result = ULTRASONIC_OUT_OF_RANGE;
        }
    }
    else
    {
        /* No echo pulse was seen: wiring, power or sensor fault. */
        result = ULTRASONIC_NO_ECHO;
    }

    return result;
}

bool ultrasonic_last_trigger (uint32_t * p_msec)
{
    if ((NULL == p_msec) || (0U == g_last_trigger_valid))
    {
        return false;
    }

    *p_msec = g_last_trigger_msec;
    return true;
}

int32_t ultrasonic_median_of_three (int32_t first, int32_t second,
                                    int32_t third)
{
    int32_t low  = first;
    int32_t mid  = second;
    int32_t high = third;
    int32_t swap = 0;

    /* A three-element sorting network leaves the median in mid. */
    if (low > mid)
    {
        swap = low;
        low  = mid;
        mid  = swap;
    }

    if (mid > high)
    {
        swap = mid;
        mid  = high;
        high = swap;
    }

    if (low > mid)
    {
        mid = low;
    }

    return mid;
}

static bool wait_echo_low (void)
{
    bool b_low = (0U == gpio_get_val(BOARD_US_ECHO_PIN));

    for (uint32_t waited = 0U; (waited < OBS_ECHO_CLEAR_MSEC) && (!b_low);
         waited++)
    {
        tk_dly_tsk(1);
        b_low = (0U == gpio_get_val(BOARD_US_ECHO_PIN));
    }

    return b_low;
}

/** @brief Send the ultrasonic trigger pulse. */
static void send_trigger (void)
{
    gpio_set_val(BOARD_US_TRIG_PIN, 1U);
    uint32_t const start_usec = encoder_now_usec();

    while ((encoder_now_usec() - start_usec) < OBS_TRIG_PULSE_USEC)
    {
        /* Busy wait: the trigger pulse is only a few usec long. */
    }

    gpio_set_val(BOARD_US_TRIG_PIN, 0U);
}

static int32_t ticks_to_mm (UW ticks)
{
    /* The counter is 16 bits wide, so the product fits in 32 bits. */
    int32_t const count = (int32_t)ticks;

    return (count * US_SOUND_NUM) / US_SOUND_DEN;
}

/*** end of file ***/

