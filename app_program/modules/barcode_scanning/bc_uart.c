/** @file bc_uart.c
 * @brief Throttled barcode status messages through existing tm_printf.
 */
#include "bc_uart.h"
#include "bc_port.h"
#include <tm/tmonitor.h>

#ifndef BC_UART_DEBUG
#define BC_UART_DEBUG 1
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
        tm_printf((UB *)"[BARCODE] Rejected: code %d\n",
                  (INT)reason);
    }
    return E_OK;
#else
    return E_OK;
#endif
}

/*** end of file ***/

