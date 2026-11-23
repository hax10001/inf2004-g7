/** @file bc_uart.h
 * @brief Deferred UART diagnostics for barcode result events.
 */
#ifndef BC_UART_H
#define BC_UART_H

#include <tk/tkernel.h>

/** @brief Print at most once per pending result sequence.
 * @note Call from a lower-priority task, never an ISR or IR sampler.
 * This function may block inside tm_printf; it never acknowledges a result.
 */
ER bc_uart_poll (void);

#endif /* BC_UART_H */

/*** end of file ***/

