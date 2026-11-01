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
    uint8_t valid;
    uint8_t letter;
    uint8_t reverse;
    uint8_t support;
    int32_t score_z100;
    int32_t margin_z100;
    uint8_t runner_letter;
    uint8_t runner_reverse;
    uint8_t positive;
    uint8_t eligible;
    int32_t runner_score_z100;
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

