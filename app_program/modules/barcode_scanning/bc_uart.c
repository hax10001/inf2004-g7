/** @file bc_uart.c
 * @brief Throttled barcode status messages through existing tm_printf.
 */
#include "bc_uart.h"
#include "bc_port.h"
#include <tm/tmonitor.h>

#ifndef BC_UART_DEBUG
#define BC_UART_DEBUG 1
#endif

#if BC_UART_DEBUG
static uint8_t gb_reported_input_error;
static uint32_t g_input_error_msec;
#endif

ER bc_uart_poll (void)
{
#if BC_UART_DEBUG
    bc_port_result_t result = {0U, 0U, 0U};
    ER error = bc_port_diag_pop(&result);

    if (E_OBJ == error)
    {
        return E_OK;
    }
    if (error < E_OK)
    {
        return error;
    }

    if (0U != (result.info & BC_PORT_VALID_FLAG))
    {
        uint8_t letter = (uint8_t)(result.info & BC_PORT_LETTER_MASK);

        if (letter < 26U)
        {
            tm_printf((UB *)"[BARCODE] Detected: %c\n",
                      (INT)('A' + letter));
        }
    }
    else
    {
        uint8_t reason = (uint8_t)
            ((result.info & BC_PORT_ERROR_MASK) >>
             BC_PORT_ERROR_SHIFT);
        /* Repeated input errors must not amplify console-induced delays.
         * Counters still record every failed sample in [BC INPUT].
         */
        if ((1U != reason) || (0U == gb_reported_input_error) ||
            ((uint32_t)(result.timestamp_msec - g_input_error_msec) >=
             1000U))
        {
            tm_printf((UB *)"[BARCODE] Rejected: code %d\n",
                      (INT)reason);
            if (1U == reason)
            {
                gb_reported_input_error = 1U;
                g_input_error_msec = result.timestamp_msec;
            }
        }
    }
    return E_OK;
#else
    return E_OK;
#endif
}

/*** end of file ***/

