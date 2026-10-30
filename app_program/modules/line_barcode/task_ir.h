/** @file task_ir.h
 * @brief Internal IR sampling task entry point.
 */
#ifndef TASK_IR_H
#define TASK_IR_H

#include <tk/tkernel.h>

/** @brief Sample and publish IR inputs at the tested 100 ms interval. */
void task_ir_task (INT start_code, void * p_context);

#endif /* TASK_IR_H */

/*** end of file ***/

