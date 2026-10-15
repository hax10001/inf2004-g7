/** @file controller.h
 * @brief Robot controller lifecycle.
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <tk/tkernel.h>

/** @brief Prepare controller resources; currently no kernel objects needed. */
ER controller_init (void);
/** @brief Create and start the controller task at its tested priority. */
ER controller_start (void);

#endif /* CONTROLLER_H */

/*** end of file ***/

