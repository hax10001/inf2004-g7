/** @file bc_setup.h
 * @brief Barcode module start-up, worker and IR-sampler hooks.
 */
#ifndef BC_SETUP_H
#define BC_SETUP_H

#include <stdint.h>
#include <tk/tkernel.h>

/** @brief Initialise only after motor/encoder initialization.
 * Invalid calibration selects measurement logging. Barcode failures are
 * reported by the worker and never abort the rest of application startup.
 */
ER bc_setup_init (void);

/** @brief Optionally create a low-priority diagnostics/decoder task. */
ER bc_setup_start (void);

/** @brief Service capture/results once, from the barcode worker only. */
ER bc_setup_poll (void);

/** @brief Called by the 10 ms IR sampler once GP27 is ready.
 * Arms the dedicated 2 ms capture alarm; later calls do not feed it.
 * No printing, sleeps, motor writes, or dynamic allocation.
 */
ER bc_setup_sample (uint8_t raw_gpio, uint32_t now_msec);

#endif /* BC_SETUP_H */

/*** end of file ***/

