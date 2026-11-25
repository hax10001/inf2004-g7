/** @file task_controller.h
 * @brief Internal controller task entry point.
 */
#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H

#include <tk/tkernel.h>
#include <stdint.h>

/** @brief Run the robot state machine every APP_CONTROLLER_PERIOD_MSEC. */
void task_controller_task (INT start_code, void * p_context);
#ifdef B1_LIVE_COMMANDS
/** @brief Create the remote-command mutex before tasks start. */
ER g7_controller_remote_init (void);
/** @brief Read the last navigation event for the MQTT mission stream. */
void g7_controller_navigation_report(uint8_t *p_kind, uint8_t *p_value,
                                     uint16_t *p_sequence);
#endif

#endif /* TASK_CONTROLLER_H */

/*** end of file ***/

