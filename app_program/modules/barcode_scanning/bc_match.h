/** @file bc_match.h
 * @brief Analog Code 39 template matching on a frozen waveform.
 */
#ifndef BC_MATCH_H
#define BC_MATCH_H

#include <stdint.h>

/** Position relative to the saved buffer; signal is baseline-corrected ADC. */
typedef void (* bc_match_read_t)(uint32_t index, uint32_t * p_units,
                                 int32_t * p_signal);

typedef struct
{
    /* The five boolean conditions fit in one byte. Letter fields retain
     * a 5-bit invalid letter sentinel (31); support stays byte-wide. */
    unsigned char valid:1;
    unsigned char reverse:1;
    unsigned char runner_reverse:1;
    unsigned char positive:1;
    unsigned char eligible:1;
    unsigned char letter:5; /* 0..25; invalid 31 */
    unsigned char runner_letter:5;
    unsigned char support:3; /**< 0..6: one vote per threshold level. */
    int32_t score_z100, margin_z100, runner_score_z100;
} bc_match_result_t;

/** @brief Start a bounded search. Frozen data must remain unchanged. */
void bc_match_begin (uint32_t count, uint32_t noise,
                      bc_match_read_t read_sample, int32_t * p_prefix,
                      uint32_t prefix_capacity);

/** @brief Process at most two letter/orientation jobs; nonzero when done. */
uint8_t bc_match_poll (void);

bc_match_result_t bc_match_result (void);

#endif /* BC_MATCH_H */

/*** end of file ***/

