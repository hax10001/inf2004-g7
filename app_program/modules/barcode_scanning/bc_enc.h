/** @file bc_enc.h
 * @brief Motor-off, encoder-only calibration session in usermain context.
 */
#ifndef BC_ENC_H
#define BC_ENC_H

#include <tk/tkernel.h>

/** @brief Initialize stopped motors and encoders; log three timed passes.
 * Never starts motion, controller, IR, IMU or barcode capture tasks.
 * After completion, sleeps until reset. Distance is measured by the user.
 */
INT bc_enc_run (void);

#endif /* BC_ENC_H */

/*** end of file ***/

