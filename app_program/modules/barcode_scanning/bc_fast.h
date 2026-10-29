/** @file bc_fast.h
 * @brief Barcode-only 4 ms RP2040 alarm-2 capture without RTOS sleeps.
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

/** @brief Drain sampled data in low-priority task, never in ISR. */
ER bc_fast_drain (void);

#endif /* BC_FAST_H */

/*** end of file ***/

