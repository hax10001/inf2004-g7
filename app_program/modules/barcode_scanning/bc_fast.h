/** @file bc_fast.h
 * @brief Barcode-only 2 ms RP2040 alarm-2 capture without RTOS sleeps.
 */
#ifndef BC_FAST_H
#define BC_FAST_H

#include <stdint.h>
#include <tk/tkernel.h>

/** @brief Reserve alarm 2 after encoder initialization; do not arm yet. */
ER bc_fast_init (void);

/** @brief Arm only after the existing GP27 driver has initialized it.
 * @note Called once from existing 10 ms IR hook; subsequent hooks ignored.
 */
ER bc_fast_arm (void);

/** @brief Stop this module's alarm without changing GPIO or encoders. */
void bc_fast_stop (void);

/** @brief Report a fatal alarm-programming failure to the worker task. */
ER bc_fast_error (void);

/** @brief Drain and process each sample in task context, never in ISR. */
ER bc_fast_drain (void);

/** ADC range over processed samples since the previous diagnostic report.
 * Read/reset only from the same barcode worker that calls drain.
 */
typedef struct
{
    uint32_t samples;
    uint32_t errors;
    /* These are copied from the validated 12-bit ADC queue entry.
     * Only the barcode worker updates/resets this statistics record;
     * the timer ISR only writes to the separate SPSC sample ring. */
    unsigned int latest:12;
    unsigned int minimum:12;
    unsigned int maximum:12;
    uint32_t filtered_spikes; /**< Unconfirmed colour pulses suppressed. */
} bc_fast_adc_stats_t;

void bc_fast_adc_report (bc_fast_adc_stats_t * p_stats);

#endif /* BC_FAST_H */

/*** end of file ***/

