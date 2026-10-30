/** @file obstacle.c
 * @brief Fail-closed replacement while bypass calibration is unavailable.
 *
 * This implementation deliberately authorises no movement. It creates no
 * task and accesses no peripheral. It must not be called a working bypass.
 */
#include "obstacle.h"

ER obstacle_init (void)
{
    /* Refuse startup; the caller must propagate this error. */
    return E_OBJ;
}

ER obstacle_start (void)
{
    return E_OBJ;
}

bool obstacle_is_blocked (void)
{
    return true;
}

bool obstacle_has_fault (void)
{
    return true;
}

void obstacle_abort (void)
{
    /* Fault is permanent in this build; no restart can clear it. */
}

obstacle_drive_t obstacle_bypass_step (uint32_t line_left,
                                      uint32_t line_right,
                                      uint32_t now_msec)
{
    obstacle_drive_t result = {0, 0, true, false, false, true};

    (void)line_left;
    (void)line_right;
    (void)now_msec;
    return result;
}

/*** end of file ***/

