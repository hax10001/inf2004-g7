/** @file bc_code.h
 * @brief Portable, fixed-size Code 39 A-Z letter pattern decoder.
 */
#ifndef BC_CODE_H
#define BC_CODE_H

#include <stdbool.h>
#include <stdint.h>

#define BC_CODE_ELEMENTS 29U
#define BC_CODE_LETTER_INVALID 31U
#define BC_CODE_LETTER_MASK 0x1FU

/** Nine elements per symbol; a set bit denotes a wide element.
 * First bar is bit 8, last bar is bit 0.
 */
typedef struct
{
    /* Code 39 consists of nine wide/narrow elements, exactly 9 bits.
     * This copied read-only lookup record is not shared with an ISR. */
    uint16_t element_pattern:9;
} bc_code_pattern_t;

typedef struct
{
    /* Decoder local/result-copy record: 5+2+1+1 bits, no shared RMW. */
    unsigned char letter:5;  /**< 0..25, or 31 invalid. */
    unsigned char quality:2; /**< 0 invalid, 1 marginal, 2 good, 3 strong. */
    unsigned char reverse:1; /**< 1 when scanned backwards. */
    unsigned char valid:1;   /**< Exactly 0 or 1. */
} bc_code_result_t;

/** @brief Copy the Code 39 pattern for letter 0..25.
 * @return False for a reserved or invalid letter code.
 */
bool bc_code_get (uint8_t letter, bc_code_pattern_t * p_pattern);

/** @brief Decode 29 alternating black/white element widths.
 * Bars and spaces are classified separately within each nine-run symbol.
 * Wide elements must be at least 1.4x the largest same-colour narrow run.
 * @param p_width 29 encoder travel widths; starts and ends in black.
 * @param min_width Minimum trustworthy width in same travel units.
 * @return Only exact * / A-Z / * matches are accepted.
 */
bc_code_result_t bc_code_decode (uint32_t const * p_width,
                                  uint32_t min_width);

#endif /* BC_CODE_H */

/*** end of file ***/

