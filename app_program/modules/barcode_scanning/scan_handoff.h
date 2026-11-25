/** @file scan_handoff.h
 * @brief One worker, one controller; caller serializes all access.
 * Telemetry has an independent acknowledgement. No motor authority here.
 */
#ifndef SCAN_HANDOFF_H
#define SCAN_HANDOFF_H
#include <stdint.h>

typedef struct {
    uint32_t captured_msec;
    uint32_t completed_msec;
    uint16_t sequence;
    uint16_t letter : 5;   /* 0..25, 31 invalid */
    uint16_t ready : 1;
    uint16_t busy : 1;
    uint16_t enabled : 1;  /* explicit mission authorization */
    uint16_t owned : 1;    /* this capture began in the current mission */
    uint16_t overflow : 1; /* unread result: fail closed */
    uint16_t begin_overflow : 1; /* overflow set by the current capture */
} scan_handoff_t;

/** @brief Enable or disable hand-off for a new run; clears ownership. */
static inline void scan_handoff_enable(scan_handoff_t *p_slot, uint8_t enable)
{
    /* Never transfer an old worker's ownership into a new run. */
    p_slot->enabled = (enable != 0U);
    p_slot->owned = 0U;
    p_slot->busy = 0U;
    p_slot->ready = 0U;
    p_slot->overflow = 0U;
    p_slot->begin_overflow = 0U;
}
/** @brief Mark the start of a capture owned by the current run. */
static inline void scan_handoff_begin(scan_handoff_t *p_slot, uint32_t captured)
{
    p_slot->owned = p_slot->enabled;
    if (!p_slot->owned)
    {
        return;
    }
    p_slot->begin_overflow = (uint16_t)(p_slot->ready && !p_slot->overflow);
    if (p_slot->ready)
    {
        p_slot->overflow = 1U;
    }
    p_slot->captured_msec = captured;
    p_slot->busy = 1U;
}
/** @brief End a capture dropped as not a barcode: no result is produced,
 * navigation stops waiting, and an unread earlier result is not counted
 * as overwritten. */
static inline void scan_handoff_abort(scan_handoff_t *p_slot)
{
    if (!p_slot->owned)
    {
        return;
    }
    if (p_slot->begin_overflow)
    {
        p_slot->overflow = 0U;
    }
    p_slot->begin_overflow = 0U;
    p_slot->busy = 0U;
    p_slot->owned = 0U;
}
/** @brief Publish a finished capture's result, or flag an overflow. */
static inline void scan_handoff_finish(scan_handoff_t *p_slot, uint8_t letter,
                                        uint16_t sequence, uint32_t completed)
{
    if (!p_slot->owned)
    {
        return;
    }
    p_slot->sequence = sequence;
    p_slot->completed_msec = completed;
    p_slot->letter = (letter < 26U && !p_slot->overflow) ? letter : 31U;
    /* Completion and result become visible in the SAME critical section. */
    p_slot->ready = 1U;
    p_slot->busy = 0U;
    p_slot->owned = 0U;
    p_slot->begin_overflow = 0U;
}
#endif

/*** end of file ***/

