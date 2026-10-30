/** @file ir.c
 * @brief IR GPIO access with the robot's tested wiring and error behaviour.
 */
#include "ir.h"
#include "board/board_config.h"
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

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

    error = gpio_set_pin(BOARD_IR_BARCODE_PIN, GPIO_MODE_IN);

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
    *p_barcode = gpio_get_val(BOARD_IR_BARCODE_PIN);
}

/*** end of file ***/

