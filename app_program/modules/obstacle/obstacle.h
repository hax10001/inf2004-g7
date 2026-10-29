/** @file obstacle.h
 *
 * @brief Obstacle detection, scan, bypass and line reacquisition.
 *
 * The obstacle task owns the ultrasonic sensor and servo. The controller
 * stays the only caller of the motion module and drives the bypass one
 * non-blocking step per period.
 */
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "obs_state.h"
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

/** @brief What the controller should do this period. */
typedef struct
{
    int32_t left_rpm;
    int32_t right_rpm;
    bool    b_stop;        /**< Stop both wheels. */
    bool    b_line_follow; /**< Use the normal line action this period. */
    bool    b_done;        /**< Bypass finished; resume line following. */
    bool    b_fault;       /**< Stop and enter the error state. */
} obstacle_drive_t;

/** @brief Create the shared state, sensor and servo. */
ER obstacle_init (void);
/** @brief Create and start the obstacle task. */
ER obstacle_start (void);
/** @brief True once an obstacle has been confirmed ahead. */
bool obstacle_is_blocked (void);
/** @brief True if a critical sensor fault should stop the robot. */
bool obstacle_has_fault (void);
/** @brief Cancel any scan or bypass and park the servo. */
void obstacle_abort (void);
/** @brief Advance the bypass one step; call once per controller period.
 *  @param line_left Left IR, 1 is black.
 *  @param line_right Right IR, 1 is black.
 *  @param now_msec Kernel time in milliseconds.
 */
obstacle_drive_t obstacle_bypass_step (uint32_t line_left, uint32_t line_right,
                                       uint32_t now_msec);

#endif /* OBSTACLE_H */

/*** end of file ***/

