/** @file ir_shared.h
 * @brief Backward-compatible IR API backed by packed, mutex-protected data.
 */
#ifndef IR_SHARED_H
#define IR_SHARED_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#define IR_SHARED_LEFT_BIT 0x01U
#define IR_SHARED_RIGHT_BIT 0x02U
#define IR_SHARED_BARCODE_BIT 0x04U
#define IR_SHARED_VALID_BIT 0x08U
#define IR_SHARED_ENCODER_BIT 0x10U

/* ABI preserved: current controller uses the three 32-bit fields. */
typedef struct
{
    uint32_t left;
    uint32_t right;
    uint32_t barcode;
    uint32_t timestamp_msec;
    bool b_valid;
} ir_shared_snapshot_t;

/* Internal compact snapshot can be transmitted without the legacy fields. */
typedef struct
{
    uint32_t timestamp_msec;
    uint32_t travel_ticks;
    uint32_t sample_number;
    uint8_t flags;
} ir_shared_compact_t;

/** @brief Create the mutex; must be called before either IR task starts. */
ER ir_shared_init (void);
/** @brief Publish exactly one GPIO sample (0 or 1 per sensor).
 * @note Never call from interrupt context; bounded mutex is task-only.
 */
ER ir_shared_publish (uint32_t left, uint32_t right, uint32_t barcode);
/** @brief Legacy controller interface, reconstructed from packed flags. */
ER ir_shared_get_latest (ir_shared_snapshot_t * p_output);
/** @brief Copy packed sensor and latest task-provided encoder snapshot. */
ER ir_shared_get_compact (ir_shared_compact_t * p_output);
/** @brief Buddy 2 task-level adapter for cumulative travelled ticks.
 * @note Must be cumulative unsigned travel, not signed position or time.
 *       Must call at least every 40 ms, and revalidate on direction change.
 *       This adapter is not an ISR-safe API. No Buddy 2 calls yet exist.
 */
ER ir_shared_encoder_update (uint32_t ticks, bool b_valid);

#endif /* IR_SHARED_H */

/*** end of file ***/

