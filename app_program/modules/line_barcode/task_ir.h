/** @file task_ir.h
 * @brief Internal IR sampling task entry point.
 */
#ifndef TASK_IR_H
#define TASK_IR_H

#include <tk/tkernel.h>

/** @brief Sample and publish IR inputs using APP_IR_PERIOD_MSEC. */
void task_ir_task (INT start_code, void * p_context);

#endif /* TASK_IR_H */

/*** end of file ***/

