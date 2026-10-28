/** @file line_track.h
 * @brief Line and junction perception; never controls motor hardware.
 */
#ifndef LINE_TRACK_H
#define LINE_TRACK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LT_FORWARD = 0,
    LT_LEFT = 1,
    LT_RIGHT = 2,
    LT_SEARCH = 3,
    LT_JUNCTION = 4,
    LT_FAULT = 5
} line_track_state_t;

typedef struct
{
    uint8_t state;
    uint8_t last_side;
    uint8_t junction_count;
    uint8_t b_junction_latched;
    uint32_t lost_since_msec;
    uint32_t last_junction_msec;
    uint8_t b_lost;
    uint8_t b_have_junction_start;
    uint32_t junction_start_ticks;
    uint32_t recovery_start_msec;
    uint8_t b_recovering;
    uint8_t recovery_count;
} line_track_t;

/** @brief Initialise pure line perception and recovery tracking. */
void line_track_init (line_track_t * p_track);
/** @brief Activate bounded recovery explicitly after obstacle bypass. */
void line_track_recovery (line_track_t * p_track,
                           bool b_recovery, uint32_t now_msec);
/** @brief Update line state after a filtered sampling step.
 * @param bits Bit 0 is left-black; bit 1 is right-black.
 * @param now_msec Monotonic timestamp.
 * @param b_junction_event Set true only on a fresh stable junction.
 * @note Both-black is a candidate only; real junction geometry needs
 *       independent calibration and validation on hardware.
 */
line_track_state_t line_track_step (line_track_t * p_track,
                                    uint8_t bits, uint32_t now_msec,
                                    bool * p_b_junction_event);
/** @brief Calibrated version requiring sustained movement across a crossing. */
line_track_state_t line_track_step_distance (line_track_t * p_track,
                                             uint8_t bits, uint32_t now_msec,
                                             uint32_t travel_ticks,
                                             uint32_t min_junction_ticks,
                                             bool b_encoder_valid,
                                             bool * p_b_junction_event);

#endif /* LINE_TRACK_H */

/*** end of file ***/

