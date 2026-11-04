/** @file line_barcode.c
 * @brief IR subsystem resource creation and task startup.
 */
#include "line_barcode.h"
#include "ir_shared.h"
#include "task_ir.h"
#include "app/app_config.h"
#include "app/tasks.h"

ER line_barcode_init (void)
{
    return ir_shared_init();
}

ER line_barcode_start (void)
{
    ID h_task = tasks_create(task_ir_task, APP_IR_PRIORITY);

    if (h_task <= E_OK)
    {
        /* Keep the original rejection of a zero task handle. */
        return (E_OK == h_task) ? E_OBJ : h_task;
    }

    return tk_sta_tsk(h_task, 0);
}

/*** end of file ***/

