/** @file ir.h
 * @brief Digital line sensors and optional analog barcode device access.
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
/** @brief Read line inputs and barcode level (cached in analog mode). */
void ir_read (uint32_t * p_left, uint32_t * p_right, uint32_t * p_barcode);

/** @brief Capture barcode ADC and update hysteresis, from barcode ISR only.
 * No waiting or RTOS calls. In digital mode raw receives the GPIO level.
 * Task reads use the last classified barcode level, avoiding two writers.
 */
bool ir_barcode_capture (uint16_t * p_raw, uint8_t * p_level);

#endif /* IR_H */

/*** end of file ***/

