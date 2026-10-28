/** @file bc_decode.h
 * @brief Bounded, platform-independent Code 39 letter decoding.
 */
#ifndef BC_DECODE_H
#define BC_DECODE_H

#include <stdbool.h>
#include <stdint.h>

#define BC_RUN_COUNT 29U
#define BC_INVALID_LETTER 31U
#define BC_LETTER_MASK 0x1FU

/** @brief Result uses five bits for A=0 through Z=25; 31 is invalid. */
typedef struct
{
    uint8_t letter;
    uint8_t confidence;
    bool b_reverse;
    bool b_valid;
} bc_decode_result_t;

/** @brief Decode 29 alternating runs: * / letter / * in Code 39.
 * @param p_run_ticks Twenty-nine widths, beginning with black.
 * @param min_ticks Minimum acceptable narrow run in encoder ticks.
 * @return Valid result on exact pattern match; otherwise invalid.
 * @note Confidence is 0 for rejected, 1 for valid narrow/wide separation.
 */
bc_decode_result_t bc_decode_runs (uint32_t const * p_run_ticks,
                                    uint32_t min_ticks);

/** @brief Return Code 39 nine-element pattern for a letter (test utility).
 * @return False for values outside A=0 to Z=25.
 */
bool bc_decode_pattern (uint8_t letter, char * p_pattern);

#endif /* BC_DECODE_H */

/*** end of file ***/

