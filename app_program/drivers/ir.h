/** @file ir.h
 * @brief Raw IR device operations through the supplied BSP GPIO API.
 */
#ifndef IR_H
#define IR_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

/** @brief Configure left, barcode, then right input in the tested order.
 * @param p_b_sleep_on_error Reports the original task's failure disposition.
 * @return E_OK or the first BSP error; the task owns sleeping or returning.
 */
ER ir_init (bool * p_b_sleep_on_error);
/** @brief Read each sensor once, preserving sampling order. */
void ir_read (uint32_t * p_left, uint32_t * p_right, uint32_t * p_barcode);

#endif /* IR_H */

/*** end of file ***/

