/** @file obstacle.h
 * @brief Fail-closed obstacle integration interface.
 */
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

typedef struct
{
    int32_t left_rpm;
    int32_t right_rpm;
    bool    b_stop;
    bool    b_line_follow;
    bool    b_done;
    bool    b_fault;
} obstacle_drive_t;

ER               obstacle_init (void);
ER               obstacle_start (void);
bool             obstacle_is_blocked (void);
bool             obstacle_has_fault (void);
void             obstacle_abort (void);
obstacle_drive_t obstacle_bypass_step (uint32_t line_left, uint32_t line_right,
                                       uint32_t now_msec);

#endif /* OBSTACLE_H */

/*** end of file ***/

