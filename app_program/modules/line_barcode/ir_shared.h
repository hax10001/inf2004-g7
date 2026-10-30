/** @file ir_shared.h
 * @brief Protected IR readings shared between sampler and controller.
 */
#ifndef IR_SHARED_H
#define IR_SHARED_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

typedef struct
{
    uint32_t left : 1;
    uint32_t right : 1;
    uint32_t barcode : 1;
    uint32_t b_valid : 1;

    uint32_t timestamp_msec;
} ir_shared_snapshot_t;

/** @brief Create snapshot storage before starting any tasks. */
ER ir_shared_init (void);

/** @brief Publish a timestamped snapshot under its mutex. */
ER ir_shared_publish (uint32_t left, uint32_t right, uint32_t barcode);

/** @brief Copy the latest snapshot under its mutex. */
ER ir_shared_get_latest (ir_shared_snapshot_t * p_output);

/** @brief Report whether any sample since the last call was both black,
 * then clear that record. Intended for the single controller consumer.
 */
ER ir_shared_take_junction (bool * p_b_seen);

/** @brief Report whether each sensor saw black in any sample since the last
 * call, then clear those records. Intended for the single controller
 * consumer.
 */
ER ir_shared_take_sides (bool * p_b_left, bool * p_b_right);

/** @brief Internal subsystem lock shared with the barcode mailbox.
 * No hardware, logging, or nested snapshot calls while held. A zero
 * timeout lets barcode sampling skip contention without delaying IR.
 */
ER ir_shared_lock (TMO timeout);
/** @brief Release a successful ir_shared_lock(). */
ER ir_shared_unlock (void);

#endif /* IR_SHARED_H */

/*** end of file ***/

