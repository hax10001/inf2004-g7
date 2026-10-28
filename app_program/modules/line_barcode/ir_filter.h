/** @file ir_filter.h
 * @brief Debounced, bit-packed IR signals with configured digital polarity.
 */
#ifndef IR_FILTER_H
#define IR_FILTER_H

#include <stdbool.h>
#include <stdint.h>

#define IR_FILTER_BITS 0x07U
#define IR_FILTER_STABLE_SAMPLES 3U

typedef struct
{
    uint8_t polarity_bits;
    uint8_t stable_bits;
    uint8_t previous_bits;
    uint8_t counts[3];
    bool b_valid;
} ir_filter_t;

/** @brief Set input polarity (one=GPIO high means black) for each sensor.
 * @note Calibration of digital modules is physical threshold adjustment;
 *       this API only configures polarity after on-track measurements.
 */
bool ir_filter_init (ir_filter_t * p_filter, uint8_t polarity_bits);
/** @brief Observe raw GPIO bits (left=0, right=1, barcode=2).
 * @return True when three consecutive identical samples have been seen.
 */
bool ir_filter_step (ir_filter_t * p_filter, uint8_t raw_bits,
                     uint8_t * p_black_bits);

#endif /* IR_FILTER_H */

/*** end of file ***/

