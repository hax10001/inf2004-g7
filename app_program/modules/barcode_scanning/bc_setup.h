/** @file bc_setup.h
 * @brief Minimal activation hooks for review by the team integration owner.
 */
#ifndef BC_SETUP_H
#define BC_SETUP_H

#include <stdint.h>
#include <tk/tkernel.h>

/** @brief Initialise only after motor/encoder initialization.
 * When BC_CAL_READY is zero, this is an inactive, safe no-op.
 */
ER bc_setup_init (void);

/** @brief Optionally create a low-priority diagnostics/decoder task. */
ER bc_setup_start (void);

/** @brief Existing 10 ms sampler calls this once GP27 is ready.
 * Arms dedicated 4 ms timer; subsequent 10 ms calls do not feed it.
 * No printing, sleeps, motor writes, or dynamic allocation.
 */
ER bc_setup_sample (uint8_t raw_gpio, uint32_t now_msec);

#endif /* BC_SETUP_H */

/*** end of file ***/

