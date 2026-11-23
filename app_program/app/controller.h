/** @file controller.h
 * @brief Robot controller lifecycle.
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <tk/tkernel.h>
#include <stdint.h>

/** @brief Prepare controller resources, including the live-command mutex. */
ER controller_init (void);
/** @brief Create and start the controller task at its tested priority. */
ER controller_start (void);
#ifdef B1_LIVE_COMMANDS
/* No PWM writes. Validated remote requests are consumed by controller task. */
uint8_t g7_controller_remote_command(uint8_t cmd,void *context);
uint8_t g7_controller_remote_ready(void);
/* Bitmask explaining why remote START is blocked; zero means ready. */
uint8_t g7_controller_remote_blockers(void);
/* True only when STOP has actually left the controller in safe remote hold. */
uint8_t g7_controller_remote_hold_applied(void);
uint8_t g7_controller_mission_state(void);
#endif

#endif /* CONTROLLER_H */

/*** end of file ***/

