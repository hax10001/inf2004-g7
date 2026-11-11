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
} scan_handoff_t;

static inline void scan_handoff_enable(scan_handoff_t *s, uint8_t enable)
{
    /* Never transfer an old worker's ownership into a new run. */
    s->enabled = (enable != 0U);
    s->owned = 0U;
    s->busy = 0U;
    s->ready = 0U;
    s->overflow = 0U;
}
static inline void scan_handoff_begin(scan_handoff_t *s, uint32_t captured)
{
    s->owned = s->enabled;
    if (!s->owned) return;
    if (s->ready) s->overflow = 1U;
    s->captured_msec = captured;
    s->busy = 1U;
}
static inline void scan_handoff_finish(scan_handoff_t *s, uint8_t letter,
                                        uint16_t sequence, uint32_t completed)
{
    if (!s->owned) return;
    s->sequence = sequence;
    s->completed_msec = completed;
    s->letter = (letter < 26U && !s->overflow) ? letter : 31U;
    /* Completion and result become visible in the SAME critical section. */
    s->ready = 1U;
    s->busy = 0U;
    s->owned = 0U;
}
#endif

/*** end of file ***/

