/** @file task_obstacle.h
 * @brief Obstacle task entry and the link functions implemented in obstacle.c.
 */
#ifndef TASK_OBSTACLE_H
#define TASK_OBSTACLE_H

#include "scan_plan.h"
#include <stdint.h>
#include <tk/tkernel.h>

/* Event flag bits shared by the task and the controller side. */
#define OBS_EVT_BLOCKED (1U << 0)
#define OBS_EVT_SCAN_REQ (1U << 1)
#define OBS_EVT_SCAN_DONE (1U << 2)
#define OBS_EVT_ABORT (1U << 3)
#define OBS_EVT_SIDE_WATCH (1U << 4)
#define OBS_EVT_SIDE_SEEN (1U << 5)
#define OBS_EVT_SIDE_CLEAR (1U << 6)

void task_obstacle_task (INT start_code, void * p_context);

/* Implemented in obstacle.c. */
UINT obstacle_link_flags_read (void);
void obstacle_link_flags_set (UINT bits);
void obstacle_link_flags_clear (UINT bits);
void obstacle_link_publish_distance (uint16_t mm);
void obstacle_link_faults (uint8_t set_mask, uint8_t clear_mask);
void obstacle_link_confirm (void);
void obstacle_link_store_plan (bypass_plan_t const * p_plan);
int8_t obstacle_link_watch_side (void);

#endif /* TASK_OBSTACLE_H */

/*** end of file ***/