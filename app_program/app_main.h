/** @file app_main.h
 * @brief Entry point required by the supplied micro T-Kernel startup.
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <tk/tkernel.h>

/** @brief Initialise subsystem resources, start tasks, then sleep.
 * @return Zero after the initial sleep, or one on startup failure.
 * @note The externally required name usermain is fixed by the kernel ABI.
 */
INT usermain (void);

#endif /* APP_MAIN_H */

/*** end of file ***/

