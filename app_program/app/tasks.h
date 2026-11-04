/** @file tasks.h
 * @brief Shared task creation without choosing subsystem scheduling policy.
 */
#ifndef TASKS_H
#define TASKS_H

#include <tk/tkernel.h>

typedef void (*tasks_entry_t)(INT start_code, void * p_context);

/** @brief Create a dormant task with the original stack and attributes.
 * @return A positive task handle, or the kernel error.
 */
ID tasks_create (tasks_entry_t p_entry, PRI priority);

#endif /* TASKS_H */

/*** end of file ***/

