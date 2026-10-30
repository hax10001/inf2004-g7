/** @file bc_decode.h
 * @brief Pure, relative-width decoding of one Code 39 uppercase letter.
 */
#ifndef BC_DECODE_H
#define BC_DECODE_H

#include <stdbool.h>
#include <stdint.h>

/* Three nine-element symbols, separated by two narrow white gaps. */
#define BC_RUN_COUNT 29U

typedef enum
{
    BC_OK = 0,
    BC_ERR_PATTERN,
    BC_ERR_RESOLUTION,
    BC_ERR_QUIET,
    BC_ERR_TIMEOUT,
    BC_ERR_OVERFLOW,
    BC_ERR_REVERSE,
    BC_ERR_SKEW,
    BC_ERR_SAMPLE,
    BC_ERR_STOPPED
} bc_status_t;

typedef struct
{
    char     letter;
    uint32_t narrow_units;
    uint32_t width_units;
} bc_decoded_t;

typedef enum
{
    BC_CHECK_NOT_RUN = 0,
    BC_CHECK_OK,
    BC_CHECK_INPUT,
    BC_CHECK_ZERO_WIDTH,
    BC_CHECK_TOTAL_OVERFLOW,
    BC_CHECK_MIN_WIDTH,
    BC_CHECK_SEPARATION,
    BC_CHECK_WIDTH_RATIO,
    BC_CHECK_NARROW_TOLERANCE,
    BC_CHECK_WIDE_TOLERANCE,
    BC_CHECK_GAP_FIRST,
    BC_CHECK_GAP_SECOND,
    BC_CHECK_START_SYMBOL,
    BC_CHECK_STOP_SYMBOL,
    BC_CHECK_LETTER
} bc_check_t;

typedef struct
{
    bc_check_t check;
    uint32_t min_width;
    uint32_t narrow_max;
    uint32_t wide_min;
    uint32_t failed_index;
    uint32_t max_step_units;
    uint64_t required_min;
    uint64_t required_separation;
    uint64_t narrow_sum;
    uint64_t wide_sum;
    uint16_t start_pattern;
    uint16_t letter_pattern;
    uint16_t stop_pattern;
    bool b_metrics_valid;
} bc_decode_report_t;

/** @brief Decode exactly the 29 alternating black/white runs in *letter*.
 * @param runs Encoder-distance widths, starting and ending with black.
 * @param max_step_units Largest distance between samples during this scan.
 * @param p_result Written only on success; letter excludes both stars.
 * @return BC_OK, or the reason the measured widths cannot be accepted.
 *
 * Quiet zones and motion validity belong to the acquisition state machine.
 * Minimum width and per-element tolerance are selected in bc_decode.c.
 * The experimental relaxed threshold accepts fewer samples per narrow run;
 * restore conservative validation after commissioning the optical sensor.
 * Wide and narrow clusters must also be separated by more than
 * 2 * max_step_units + 2 units. No wheel diameter or absolute scale is used.
 */
bc_status_t bc_decode (uint32_t const runs[BC_RUN_COUNT],
                       uint32_t max_step_units, bc_decoded_t * p_result);

/** @brief Decode with the same checks and report the first failed check.
 * A NULL report is allowed. Metric fields are valid when b_metrics_valid
 * is true, including on either resolution failure. required_min is an
 * inclusive lower bound; wide_min - narrow_max must strictly exceed
 * required_separation. failed_index is an original run index, the first
 * run of an invalid symbol, or BC_RUN_COUNT for a check with no one index.
 * p_result is unchanged on failure, as with bc_decode().
 */
bc_status_t bc_decode_inspect (uint32_t const runs[BC_RUN_COUNT],
                               uint32_t max_step_units,
                               bc_decoded_t * p_result,
                               bc_decode_report_t * p_report);
/** @brief Return a stable name for a decoder check; never returns NULL. */
char const * bc_check_name (bc_check_t check);

#endif /* BC_DECODE_H */

/*** end of file ***/

