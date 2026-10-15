/** @file controller.c
 * @brief Controller lifecycle, separate from its periodic task.
 */
#include "controller.h"
#include "app_config.h"
#include "task_controller.h"
#include "tasks.h"
#include <tm/tmonitor.h>

ER controller_init (void)
{
    return E_OK;
}

ER controller_start (void)
{
    ID h_task = tasks_create(task_controller_task, APP_CONTROLLER_PRIORITY);

    if (h_task <= E_OK)
    {
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[MAIN] Controller creation failed: %d\n", h_task);
        /* Keep the original rejection of a zero task handle. */
        return (E_OK == h_task) ? E_OBJ : h_task;
    }

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[MAIN] Before controller start\n");
    ER error = tk_sta_tsk(h_task, 0);

    if (error < E_OK)
    {
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[MAIN] Controller start failed: %d\n", error);
    }

    return error;
}

/*** end of file ***/

