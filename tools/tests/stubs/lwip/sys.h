/** @file sys.h
 * @brief Host stub of lwIP sys_now(): the test sets the millisecond clock
 *        (FR15 MQTT link-silence timing). Not the real lwIP header.
 */
#ifndef CT_LWIP_SYS_H
#define CT_LWIP_SYS_H
#include "lwip/err.h"

/** @brief Milliseconds since boot, chosen by the test. */
u32_t sys_now(void);

#endif /* CT_LWIP_SYS_H */

/*** end of file ***/

