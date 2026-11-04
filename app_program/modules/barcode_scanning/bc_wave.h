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

/** @brief Hold driving after the one-pass capture; cleared only by reboot. */
uint8_t bc_wave_motion_hold (void);

#endif /* BC_WAVE_H */

/*** end of file ***/

