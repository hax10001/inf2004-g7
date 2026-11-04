/** @file bc_scan.h
 * @brief Bounded Code 39 capture state machine, independent of the RTOS.
 */
#ifndef BC_SCAN_H
#define BC_SCAN_H

#include "bc_code.h"
#include <stdint.h>

#define BC_SCAN_EVENT_NONE 0U
#define BC_SCAN_EVENT_READY 1U
#define BC_SCAN_EVENT_REJECTED 2U

#define BC_SCAN_ERROR_NONE 0U
#define BC_SCAN_ERROR_INPUT 1U
#define BC_SCAN_ERROR_WIDTH 2U
#define BC_SCAN_ERROR_OVERFLOW 3U
#define BC_SCAN_ERROR_TIMEOUT 4U
#define BC_SCAN_ERROR_DISTANCE 5U
#define BC_SCAN_ERROR_INTERRUPTED 6U
#define BC_SCAN_ERROR_PATTERN 7U
#define BC_SCAN_ERROR_BUSY 8U

/** Three bits are sufficient for six capture states. */
typedef enum
{
    BC_SCAN_IDLE = 0,
    BC_SCAN_START_CANDIDATE,
    BC_SCAN_CAPTURING,
    BC_SCAN_END_CANDIDATE,
    BC_SCAN_COMPLETE,
    BC_SCAN_INVALID
} bc_scan_state_t;

/** Distances are monotonically increasing calibrated travel units.
 * Configuration is rejected if any threshold is contradictory.
 */
typedef struct
{
    uint32_t min_run_units;
    uint32_t leading_units;
    uint32_t trailing_units;
    uint32_t min_barcode_units;
    uint32_t max_barcode_units;
    uint32_t timeout_msec;
} bc_scan_config_t;

typedef struct
{
    bc_scan_config_t cfg;
    uint32_t widths[BC_CODE_ELEMENTS];
    uint32_t first_white_units;
    uint32_t last_edge_units;
    uint32_t start_units;
    uint32_t start_msec;
    uint32_t result_msec;
    uint8_t  state;      /**< bc_scan_state_t, valid values 0..5. */
    uint8_t  run_count;  /**< Valid 0..29. */
    uint8_t  last_black; /**< Bit 0, unused bits reserved. */
    uint8_t  error;      /**< BC_SCAN_ERROR_*, 0..8. */
} bc_scan_t;

/** @brief Initialize capture; return false for invalid configuration. */
uint8_t bc_scan_init (bc_scan_t * p_scan,
                      bc_scan_config_t const * p_config);

/** @brief Capture one sample in bounded time; never decode or print here.
 * @param travel_units Monotonic, calibrated, forward barcode-sensor travel.
 * @param b_valid Zero on sensor/encoder fault, missed sample or rotation.
 * @return NONE, READY or REJECTED; event held until take is called.
 */
uint8_t bc_scan_feed (bc_scan_t * p_scan, uint8_t b_black,
                      uint32_t travel_units, uint32_t now_msec,
                      uint8_t b_valid);

/** @brief Transfer a completed capture or failure and rearm at white.
 * @param p_width Receives 29 widths for READY; unchanged otherwise.
 * @param p_error Receives failure reason or NONE.
 * @param p_timestamp Receives event time.
 * @return NONE, READY or REJECTED.
 */
uint8_t bc_scan_take (bc_scan_t * p_scan, uint32_t * p_width,
                      uint8_t * p_error, uint32_t * p_timestamp);

/** @brief Invalidate an interrupted scan, e.g. on lost sample. */
void bc_scan_interrupt (bc_scan_t * p_scan, uint32_t now_msec);

#endif /* BC_SCAN_H */

/*** end of file ***/

