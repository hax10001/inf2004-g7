/** @file line_barcode.h
 * @brief IR lifecycle and acknowledged, thread-safe barcode results.
 */
#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include <tk/tkernel.h>
#include "bc_scan.h"

typedef struct
{
    char letter;
    bc_status_t status;
    uint32_t sequence;
    uint32_t timestamp_msec;
    uint32_t narrow_units;
    uint32_t width_units;
    float width_mm;
    bool b_distance_valid;
} barcode_result_t;

typedef enum
{
    BARCODE_DISABLED = 0,
    BARCODE_MEASURING,
    BARCODE_ENABLED
} barcode_mode_t;

typedef struct
{
    barcode_mode_t mode;
    bc_state_t state;
    bc_status_t status;
    uint32_t raw;
    int32_t left_count;
    int32_t right_count;
    uint32_t max_gap_usec;
    uint32_t max_step_units;
    uint32_t max_skew_units;
    uint32_t last_run_units;
    uint32_t last_run_level;
    uint32_t rejected;
    uint32_t completed;
    uint32_t failure_sequence;
    bc_status_t failure_status;
    uint32_t failure_reports_dropped;
    uint32_t timestamp_usec;
    uint32_t run_count;
    uint32_t runs[BC_RUN_COUNT];
} barcode_diagnostic_t;

typedef struct
{
    bc_decode_report_t decode;
    bc_status_t status;
    bc_state_t failed_state;
    uint32_t sequence;
    uint32_t timestamp_usec;
    uint32_t start_usec;
    uint32_t run_count;
    uint32_t runs[BC_RUN_COUNT];
    uint32_t max_step_units;
    uint32_t approach_max_step;
    uint32_t barcode_max_step;
    uint32_t trailing_max_step;
    uint32_t max_confirm_usec;
    uint32_t max_confirm_units;
    uint32_t unconfirmed_edges;
    uint32_t start_level;
    uint32_t black_level;
    uint32_t rejected;
    uint32_t completed;
    bool b_started_black;
    bool b_from_white;
    bool b_pending;
} barcode_failure_t;

/** @brief Create protected snapshot storage before any task starts. */
ER line_barcode_init (void);
/** @brief Create and start the IR sampling task. */
ER line_barcode_start (void);

/** @brief Copy an unacknowledged valid result; E_OBJ means none available.
 * Never consumes the result. Its letter is independent of mission actions.
 */
ER line_barcode_get_result (barcode_result_t * p_result);
/** @brief Consume only this sequence; stale acknowledgements return E_OBJ.
 * Rearming requires fresh forward white travel and a new complete frame.
 */
ER line_barcode_ack (uint32_t sequence);
/** @brief Discard partial capture, preserving any unacknowledged result.
 * Enabling uncalibrated settings returns E_PAR, without affecting motors.
 */
ER line_barcode_set_enabled (bool b_enabled);
/** @brief Copy commissioning statistics and captured widths atomically. */
ER line_barcode_get_diagnostic (barcode_diagnostic_t * p_output);

/** @brief Copy the saved failed attempt, independent of later live errors.
 * The first unacknowledged report is never overwritten. E_OBJ means none.
 * Printing acknowledges it; the copy remains readable until a new failure.
 */
ER line_barcode_get_failure (barcode_failure_t * p_output);
/** @brief Release the matching report after inspecting its copied data.
 * With diagnostics off, consumers use this to allow the next saved report.
 */
ER line_barcode_ack_failure (uint32_t sequence);

/** @brief Internal hook after ir_read(); time is captured before ir_read.
 * Rejects delayed IR/count pairs and never prints or waits for a mutex.
 */
void line_barcode_sample (uint32_t barcode_raw, uint32_t sample_usec);
/** @brief Optional low-rate logging outside active capture and mutex. */
void line_barcode_log (void);

#endif /* LINE_BARCODE_H */

/*** end of file ***/

