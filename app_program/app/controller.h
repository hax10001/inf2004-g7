/** @file controller.h
 * @brief Robot controller lifecycle.
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <tk/tkernel.h>
#include <stdint.h>

/** @brief Prepare controller resources, including the live-command mutex. */
ER controller_init (void);
/** @brief Create and start the controller task. */
ER controller_start (void);
#ifdef B1_LIVE_COMMANDS
/** @brief Queue a validated remote command; the controller task acts on
 * it (no PWM writes here). */
uint8_t g7_controller_remote_command(uint8_t cmd,void *p_context);
/** @brief Return non-zero when a remote START would be accepted. */
uint8_t g7_controller_remote_ready(void);
/** @brief Bitmask of reasons remote START is blocked; zero = ready. */
uint8_t g7_controller_remote_blockers(void);
/** @brief True only once STOP has put the controller in safe hold. */
uint8_t g7_controller_remote_hold_applied(void);
/** @brief Return the mission state code for telemetry. */
uint8_t g7_controller_mission_state(void);
#endif

#endif /* CONTROLLER_H */

/*** end of file ***/

