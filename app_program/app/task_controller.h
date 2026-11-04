/** @file task_controller.h
 * @brief Internal controller task entry point.
 */
#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H

#include <tk/tkernel.h>
#include <stdint.h>

/** @brief Run the tested robot state machine at its original period. */
void task_controller_task (INT start_code, void * p_context);
#ifdef B1_LIVE_COMMANDS
/** @brief Create controller command mutex before tasks are started. */
ER g7_controller_remote_init (void);
/** Last autonomous navigation event for compact MQTT mission stream. */
void g7_controller_navigation_report(uint8_t *kind, uint8_t *value,
                                     uint16_t *sequence);
#endif

#endif /* TASK_CONTROLLER_H */

/*** end of file ***/

