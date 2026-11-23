/** @file ir.c
 * @brief Digital line inputs and optional GP27 analog barcode capture.
 */
#include "ir.h"
#include "board/board_config.h"
#include "app/app_config.h"
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#if APP_BARCODE_ANALOG_ENABLED && defined(_PICO_RP2040_)
#include "../../device/adc/sysdepend/rp2040/adc_sysdep.h"

#if (APP_BARCODE_ADC_HYSTERESIS > APP_BARCODE_ADC_THRESHOLD) || \
    ((APP_BARCODE_ADC_THRESHOLD + APP_BARCODE_ADC_HYSTERESIS) > 4095U)
#error Barcode ADC thresholds must stay inside the 12-bit ADC range
#endif
#if BOARD_IR_BARCODE_PIN != 27U
#error Barcode analog capture requires GP27 / ADC channel 1
#endif

static volatile uint32_t g_barcode_level;
static volatile uint8_t gb_adc_ready;
static uint8_t gb_adc_classified;

static ER ir_barcode_adc_init (void)
{
    /* The barcode exclusively owns ADC1 conversions. The generic ADC
     * device is not opened by this application; do not open it concurrently.
     * Register definitions match the repository's RP2040 ADC driver.
     */
    clr_w(RESETS_RESET, RESETS_RESET_ADC);
    for (uint32_t i = 0U; i < 10000U; ++i)
    {
        if (0U != (in_w(RESETS_RESET_DONE) & RESETS_RESET_ADC))
        {
            out_w(GPIO_CTRL(BOARD_IR_BARCODE_PIN), GPIO_CTRL_FUNCSEL_NULL);
            clr_w(GPIO(BOARD_IR_BARCODE_PIN), GPIO_IE | GPIO_PUE | GPIO_PDE);
            out_w(ADC_INTE, 0U);
            out_w(ADC_FCS, 0U);
            out_w(ADC_DIV, 0U);
            out_w(ADC_CS, (1U << ADC_CS_AINSEL_POS) | ADC_CS_EN);
            for (uint32_t ready = 0U; ready < 10000U; ++ready)
            {
                if (0U != (in_w(ADC_CS) & ADC_CS_READY))
                {
                    /* Free-running ADC: ISR only reads the latest result. */
                    set_w(ADC_CS, ADC_CS_START_MANY);
                    gb_adc_ready = 1U;
                    return E_OK;
                }
            }
            return E_IO;
        }
    }
    return E_IO;
}
#endif

ER ir_init (bool * p_b_sleep_on_error)
{
    *p_b_sleep_on_error = true;

    ER error = gpio_set_pin(BOARD_IR_LEFT_PIN, GPIO_MODE_IN);

    if (error < E_OK)
    {
        /* UB views the same bytes; the API does not modify the string. */
        tm_printf((UB *)"IR LEFT initialization failed: %d\n", error);
        return error;
    }

#if APP_BARCODE_ANALOG_ENABLED && defined(_PICO_RP2040_)
    error = ir_barcode_adc_init();
#else
    error = gpio_set_pin(BOARD_IR_BARCODE_PIN, GPIO_MODE_IN);
#endif

    if (error < E_OK)
    {
        *p_b_sleep_on_error = false;
        /* UB views the same bytes; the API does not modify the string. */
        tm_printf((UB *)"IR BARCODE initialization failed: %d\n", error);
        return error;
    }

    error = gpio_set_pin(BOARD_IR_RIGHT_PIN, GPIO_MODE_IN);

    if (error < E_OK)
    {
        /* UB views the same bytes; the API does not modify the string. */
        tm_printf((UB *)"IR RIGHT initialization failed: %d\n", error);
    }

    return error;
}

void ir_read (uint32_t * p_left, uint32_t * p_right, uint32_t * p_barcode)
{
    *p_left    = gpio_get_val(BOARD_IR_LEFT_PIN);
    *p_right   = gpio_get_val(BOARD_IR_RIGHT_PIN);
#if APP_BARCODE_ANALOG_ENABLED && defined(_PICO_RP2040_)
    *p_barcode = g_barcode_level;
#else
    *p_barcode = gpio_get_val(BOARD_IR_BARCODE_PIN);
#endif
}

bool ir_barcode_capture (uint16_t * p_raw, uint8_t * p_level)
{
#if APP_BARCODE_ANALOG_ENABLED && defined(_PICO_RP2040_)
    if ((0U == gb_adc_ready) || (0U != (in_w(ADC_CS) & ADC_CS_ERR)))
    {
        return false;
    }
    uint16_t raw = (uint16_t)(in_w(ADC_RESULT) & 0x0FFFU);
    uint8_t high = APP_BARCODE_ADC_BLACK_HIGH ? 1U : 0U;
    uint8_t low = APP_BARCODE_ADC_BLACK_HIGH ? 0U : 1U;
    if (0U == gb_adc_classified)
    {
        g_barcode_level = (raw >= APP_BARCODE_ADC_THRESHOLD) ? high : low;
        gb_adc_classified = 1U;
    }
    else if (raw >= APP_BARCODE_ADC_THRESHOLD + APP_BARCODE_ADC_HYSTERESIS)
    {
        g_barcode_level = high;
    }
    else if (raw <= APP_BARCODE_ADC_THRESHOLD - APP_BARCODE_ADC_HYSTERESIS)
    {
        g_barcode_level = low;
    }
    *p_raw = raw;
    *p_level = (uint8_t)g_barcode_level;
#else
    *p_level = (uint8_t)gpio_get_val(BOARD_IR_BARCODE_PIN);
    *p_raw = *p_level;
#endif
    return true;
}

/*** end of file ***/

