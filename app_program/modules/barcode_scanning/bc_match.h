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
    /* One-bit flags; letters are 5 bits with 31 meaning invalid. */
    uint8_t       valid:1;
    uint8_t       reverse:1;
    uint8_t       runner_reverse:1;
    uint8_t       positive:1;
    uint8_t       eligible:1;
    uint8_t       letter:5; /* 0..25; invalid 31 */
    uint8_t       runner_letter:5;
    uint8_t       support:3; /**< 0..6: one vote per threshold level. */
    int32_t score_z100;
    int32_t margin_z100;
    int32_t runner_score_z100;
} bc_match_result_t;

/** @brief Start a bounded search. Frozen data must remain unchanged. */
void bc_match_begin (uint32_t count, uint32_t noise,
                      bc_match_read_t read_sample, int32_t * p_prefix,
                      uint32_t prefix_capacity);

/** @brief Process at most two letter/orientation jobs; nonzero when done. */
uint8_t bc_match_poll (void);

/** @brief Result of the last finished search. */
bc_match_result_t bc_match_result (void);

#endif /* BC_MATCH_H */

/*** end of file ***/

