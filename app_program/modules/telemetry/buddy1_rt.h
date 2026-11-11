/** @file buddy1_rt.h
 * @brief Start the Buddy 1 telemetry adapter task.
 */
#ifndef BUDDY1_RT_H
#define BUDDY1_RT_H
int b1_rt_start(void);
#include <stdint.h>
/** Queues actual decoded barcode events and looks up acknowledged map. */
void g7_nav_note_decoded(uint8_t letter, uint16_t sequence);
/** Discard any pre-START or old-session barcode. */
void g7_nav_clear_decoded(void);
/** 1: a mapped event, 0: nothing, -1: not mapped; nonblocking. */
int g7_nav_take_decoded(uint8_t *letter, uint8_t *action, uint16_t *sequence);
/* Atomic worker status/result, independent of telemetry delivery. */
int g7_nav_take_scan(uint8_t *letter, uint8_t *action, uint16_t *sequence,
                     uint32_t *captured_msec, uint32_t *completed_msec,
                     uint8_t *pending);
int g7_nav_begin_run(void);
#endif
/*** end of file ***/

