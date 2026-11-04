/** @file bc_wave.h
 * @brief Experimental buffered analog Code 39 decoder and comparison logs.
 */
#ifndef BC_WAVE_H
#define BC_WAVE_H

#include "bc_fastq.h"

/** @brief Consume raw samples in the worker, never in the interrupt. */
void bc_wave_sample (bc_fastq_sample_t const * p_sample);

/** @brief Process a frozen buffer incrementally and publish unique matches. */
void bc_wave_poll (void);

/** @brief Diagnostic motor hold; always clear in live-control builds. */
uint8_t bc_wave_motion_hold (void);
/** Reports whether a completed capture is safe to rearm. */
uint8_t bc_wave_can_rearm (void);
/** Live scan window, used to lower wheel target temporarily. */
uint8_t bc_wave_capture_active (void);
/** Short scan/junction suppression window; bounded in navigation core. */
uint8_t bc_wave_navigation_guard_active(void);
void bc_wave_request_rearm (void);

#endif /* BC_WAVE_H */

/*** end of file ***/

