/** @file wheel_speed.c
 * @brief Measure RPM between edges, not by counting per control period.
 *
 * Dividing the edges seen by the exact time between the first and last of
 * them avoids the quantisation of counting pulses in a fixed 50 ms window.
 */
#include "wheel_speed.h"
#include "board/board_config.h"

#define USEC_PER_MINUTE 60000000.0f

void wheel_speed_reset (wheel_speed_t * p_speed, int32_t count,
                        uint32_t edge_usec)
{
    p_speed->last_count     = count;
    p_speed->last_edge_usec = edge_usec;
    p_speed->rpm            = 0.0f;
}

float wheel_speed_update (wheel_speed_t * p_speed, int32_t count,
                          uint32_t edge_usec, uint32_t now_usec)
{
    /* Unsigned subtraction keeps differences correct across wraparound. */
    int32_t const delta =
        (int32_t)((uint32_t)count - (uint32_t)p_speed->last_count);
    uint32_t const elapsed = edge_usec - p_speed->last_edge_usec;

    if (0 != delta)
    {
        if ((0U == elapsed) || (elapsed > WHEEL_SPEED_STOP_USEC))
        {
            /* Restarting from rest: no full interval to measure yet. */
            p_speed->rpm = 0.0f;
        }
        else
        {
            float const edges = (float)((delta < 0) ? -delta : delta);

            p_speed->rpm = (edges * USEC_PER_MINUTE) /
                           ((float)elapsed * (float)BOARD_ENC_COUNTS_PER_REV);
        }

        p_speed->last_count     = count;
        p_speed->last_edge_usec = edge_usec;
        return p_speed->rpm;
    }

    uint32_t const idle = now_usec - p_speed->last_edge_usec;

    if (idle > WHEEL_SPEED_STOP_USEC)
    {
        p_speed->rpm = 0.0f;
    }
    else if (0U != idle)
    {
        /* No edge yet: the wheel is at most this fast, so decay towards it. */
        float const ceiling =
            USEC_PER_MINUTE / ((float)idle * (float)BOARD_ENC_COUNTS_PER_REV);

        if (ceiling < p_speed->rpm)
        {
            p_speed->rpm = ceiling;
        }
    }
    else
    {
        /* Same microsecond as the last edge: keep the estimate. */
    }

    return p_speed->rpm;
}

/*** end of file ***/

