/** @file tasks.c
 * @brief Common task creation; each subsystem owns its start operation.
 */
#include "tasks.h"
#include "app_config.h"

ID tasks_create (tasks_entry_t p_entry, PRI priority)
{
    T_CTSK task_config = {0};

    task_config.itskpri = priority;
    task_config.stksz   = APP_TASK_STACK_BYTES;
    task_config.task    = p_entry;
    task_config.tskatr  = TA_HLNG | TA_RNG3;

    return tk_cre_tsk(&task_config);
}

/*** end of file ***/

