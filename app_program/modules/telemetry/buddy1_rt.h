/** @file buddy1_rt.h
 * @brief Start the Buddy 1 telemetry adapter task.
 */
#ifndef BUDDY1_RT_H
#define BUDDY1_RT_H
#include <stdint.h>
/** @brief Create and start the telemetry task; 0 on success. */
int32_t b1_rt_start(void);
/** @brief Disable the handoff and cancel any pending scan. */
void g7_nav_clear_decoded(void);
/** @brief 1 mapped letter, 0 nothing, negative as g7_nav_take_scan(). */
int32_t g7_nav_take_decoded(uint8_t *p_letter, uint8_t *p_action,
    uint16_t *p_sequence);
/** @brief Take the scanner result with its timing, independent of
 *  telemetry delivery: 1 mapped, 0 nothing, -1 not mapped, -2 no letter,
 *  -3 fault. Nonblocking. */
int32_t g7_nav_take_scan(uint8_t *p_letter, uint8_t *p_action,
    uint16_t *p_sequence,
                     uint32_t *p_captured_msec, uint32_t *p_completed_msec,
                     uint8_t *p_pending);
/** @brief START only: 1 when the scanner is armed and the handoff is
 *  enabled. */
int32_t g7_nav_begin_run(void);
/** @brief Mid-run (after a bypass or a lift): enable the handoff even while
 *  the scanner is still rearming. 1 on success, 0 if the handoff failed. */
int32_t g7_nav_continue_run(void);
#endif
/*** end of file ***/

