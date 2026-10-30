/** @file bc_scan.h
 * @brief Fixed-storage, forward-only barcode acquisition state machine.
 */
#ifndef BC_SCAN_H
#define BC_SCAN_H

#include "bc_decode.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BC_WAIT_WHITE = 0,
    BC_SCANNING,
    BC_TRAILING_WHITE,
    BC_COMPLETE,
    BC_ERROR
} bc_state_t;

typedef struct
{
    uint32_t black_level;
    int32_t left_sign;
    int32_t right_sign;
    uint32_t max_gap_usec;
    uint32_t timeout_usec;
    uint32_t stop_usec;
    uint32_t max_skew_units;
    uint32_t max_skew_percent;
} bc_scan_config_t;

typedef struct
{
    bc_scan_config_t config;
    bc_state_t state;
    bc_status_t status;
    bc_decoded_t decoded;
    bc_decode_report_t decode_report;
    uint32_t runs[BC_RUN_COUNT];
    uint32_t run_count;
    uint32_t max_step_units;
    uint32_t last_run_units;
    uint32_t leading_units;
    uint32_t quiet_units;
    uint32_t white_max_step;
    uint32_t approach_max_step;
    uint32_t barcode_max_step;
    uint32_t trailing_max_step;
    uint32_t start_level;
    uint32_t start_from_level;
    uint32_t max_confirm_usec;
    uint32_t max_confirm_units;
    uint32_t unconfirmed_edges;
    uint32_t last_usec;
    uint32_t start_usec;
    uint32_t moved_usec;
    uint32_t candidate_usec;
    int32_t last_left;
    int32_t last_right;
    uint64_t position;
    uint64_t run_start;
    uint64_t white_start;
    uint64_t candidate_position;
    uint64_t motion_left_units;
    uint64_t motion_right_units;
    uint32_t stable_level;
    uint32_t candidate_level;
    uint32_t candidate_samples;
    uint32_t candidate_step_units;
    bool b_seeded;
} bc_scan_t;

/** @brief Validate settings and reset a scanner. No RTOS or hardware calls. */
bool bc_scan_init (bc_scan_t * p_scan, bc_scan_config_t const * p_config);
/** @brief Discard a partial/completed frame; require fresh white travel. */
void bc_scan_reset (bc_scan_t * p_scan);
/** @brief Feed one raw sample. True means a newly completed valid result.
 * Counts use modulo-32-bit deltas; elapsed time handles the usec wrap.
 * Two equal samples confirm an edge at the first sample's position.
 */
bool bc_scan_feed (bc_scan_t * p_scan, uint32_t raw, int32_t left,
                   int32_t right, uint32_t now_usec);

#endif /* BC_SCAN_H */

/*** end of file ***/

