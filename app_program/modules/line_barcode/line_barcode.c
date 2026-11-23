/** @file line_barcode.c
 * @brief Start only the existing line/IR task, not a duplicate decoder.
 */
#include "line_barcode.h"
#include "ir_shared.h"
#include "task_ir.h"
#include "app/tasks.h"
#include "app/app_config.h"

ER line_barcode_init (void)
{
    return ir_shared_init();
}

ER line_barcode_start (void)
{
    ID h_task = tasks_create(task_ir_task, APP_IR_PRIORITY);
    if (h_task <= E_OK)
    {
        return (E_OK == h_task) ? E_OBJ : h_task;
    }
    return tk_sta_tsk(h_task, 0);
}

/*** end of file ***/

