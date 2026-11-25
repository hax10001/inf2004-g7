/** @file bc_fast.h
 * @brief Barcode-only 2 ms RP2040 alarm-2 capture without RTOS sleeps.
 */
#ifndef BC_FAST_H
#define BC_FAST_H

#include <stdint.h>
#include <tk/tkernel.h>

/** @brief Reserve alarm 2 after encoder initialization; do not arm yet. */
ER bc_fast_init (void);

/** @brief Arm only after the GP27 driver has initialized the pin.
 * @note Called from the 10 ms IR hook; calls after the first are ignored.
 */
ER bc_fast_arm (void);

/** @brief Stop this module's alarm without changing GPIO or encoders. */
void bc_fast_stop (void);

/** @brief Report a fatal alarm-programming failure to the worker task. */
ER bc_fast_error (void);

/** @brief Drain and process each sample in task context, never in ISR. */
ER bc_fast_drain (void);

/** @brief ADC range of processed samples since the last diagnostic report.
 * Read/reset only from the same barcode worker that calls drain.
 */
typedef struct
{
    uint32_t samples;
    uint32_t errors;
    /* These are copied from the validated 12-bit ADC queue entry.
     * Only the barcode worker updates/resets this statistics record;
     * the timer ISR only writes to the separate SPSC sample ring. */
    uint32_t     latest:12;
    uint32_t     minimum:12;
    uint32_t     maximum:12;
    uint32_t filtered_spikes; /**< Unconfirmed colour pulses suppressed. */
} bc_fast_adc_stats_t;

void bc_fast_adc_report (bc_fast_adc_stats_t * p_stats);

#endif /* BC_FAST_H */

/*** end of file ***/

