/** @file task_ir.h
 * @brief Buddy 3 periodic IR sampling task entry point.
 */
#ifndef TASK_IR_H
#define TASK_IR_H

#include <tk/tkernel.h>

/** @brief Request periodic sampling (actual period requires RTOS validation).
 * @note Task exits on invalid sensor or resource error; Mission Coordinator
 *       must stop motion on stale snapshot / explicit fault.
 */
void task_ir_task (INT start_code, void * p_context);

#endif /* TASK_IR_H */

/*** end of file ***/

