/** @file motion.h
 * @brief Motion subsystem lifecycle and line-following movement interface.
 */
#ifndef MOTION_H
#define MOTION_H

#include <tk/tkernel.h>
#include "modules/line_barcode/line_follow.h"

/** @brief Initialise the motor device and leave outputs stopped. */
ER motion_init (void);
/** @brief Complete startup; this subsystem has no independent task yet. */
ER motion_start (void);
/** @brief Apply the tested line-following speed policy. */
void motion_apply (line_follow_action_t action);

#endif /* MOTION_H */

/*** end of file ***/

