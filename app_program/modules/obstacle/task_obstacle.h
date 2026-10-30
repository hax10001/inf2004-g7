/** @file task_obstacle.h
 * @brief Obstacle task entry point.
 */
#ifndef TASK_OBSTACLE_H
#define TASK_OBSTACLE_H

#include <tk/tkernel.h>

/** @brief Obstacle task body; created by obstacle_start(). */
void task_obstacle_task (INT start_code, void * p_context);

#endif /* TASK_OBSTACLE_H */

/*** end of file ***/

