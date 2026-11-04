/** @file task_controller.h
 * @brief Internal controller task entry point.
 */
#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H

#include <tk/tkernel.h>

/** @brief Run the tested robot state machine at its original period. */
void task_controller_task (INT start_code, void * p_context);

#endif /* TASK_CONTROLLER_H */

/*** end of file ***/

