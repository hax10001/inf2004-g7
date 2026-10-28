/** @file bc_capture.h
 * @brief Fixed-memory Code 39 edge capture using encoder travel distance.
 */
#ifndef BC_CAPTURE_H
#define BC_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>
#include "bc_decode.h"

/** @brief Units are cumulative (unsigned) encoder travel ticks and ms.
 * @note Caller supplies strictly monotonic *travel* ticks, even in reverse.
 *       Wheel position differences may not be substituted for this measure.
 */
typedef struct
{
    uint32_t min_run_ticks;
    uint32_t quiet_ticks;
    uint32_t max_scan_ticks;
    uint32_t timeout_msec;
    uint32_t junction_min_ticks;
    uint32_t pending_expiry_ticks;
} bc_capture_config_t;

typedef enum
{
    BC_PENDING = 0,
    BC_COMPLETE = 1,
    BC_REJECTED = 2
} bc_capture_event_t;

typedef struct
{
    bc_capture_config_t config;
    uint32_t runs[BC_RUN_COUNT];
    uint32_t last_ticks;
    uint32_t start_ticks;
    uint32_t start_msec;
    uint32_t quiet_start_ticks;
    uint8_t run_count;
    uint8_t last_black;
    uint8_t phase;
    uint8_t b_have_last;
} bc_capture_t;

/** @brief Initialise with *calibrated* width, quiet-zone and travel limits. */
bool bc_capture_init (bc_capture_t * p_capture,
                      bc_capture_config_t const * p_config);
/** @brief Feed one stable sample and synchronous encoder travel position.
 * @param b_black True if barcode IR currently reads black.
 * @param ticks Cumulative encoder travel (monotonic, not signed position).
 * @param msec Monotonic millisecond timestamp; wraps naturally.
 * @param b_encoder_valid False causes scan rejection and re-arm.
 * @param p_result Written only on a complete, valid scan.
 * @return Pending, accepted, or rejected. Does not publish turn requests.
 */
bc_capture_event_t bc_capture_step (bc_capture_t * p_capture,
                                    bool b_black, uint32_t ticks,
                                    uint32_t msec, bool b_encoder_valid,
                                    bc_decode_result_t * p_result);
/** @brief Invalidate pending scan (STOP, fault, obstacle, direction change). */
void bc_capture_cancel (bc_capture_t * p_capture);

#endif /* BC_CAPTURE_H */

/*** end of file ***/

