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
    uint32_t left;
    uint32_t right;
    uint32_t barcode;

    uint32_t timestamp_msec;
    bool     b_valid;
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

#endif /* IR_SHARED_H */

/*** end of file ***/

