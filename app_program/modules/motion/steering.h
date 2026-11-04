/** @file steering.h
 * @brief Proportional-integral steering from the discrete line action.
 */
#ifndef STEERING_H
#define STEERING_H

#include "modules/line_barcode/line_follow.h"
#include <stdbool.h>
#include <stdint.h>

/* Largest steer; at 0.8 the inner wheel keeps 20 percent of the target. */
#define STEERING_MAX 0.8f

typedef struct
{
    float    integral;      /**< Grows while off-centre, decays once centred. */
    bool     b_off_centre;  /**< The previous update saw a correction. */
    float    last_side;     /**< +1 left, -1 right, 0 none: last touch. */
    uint32_t centred_count; /**< Centred periods since that touch ended. */
    uint32_t hold_count;    /**< Periods left holding a full turn. */
    float    bend_side;     /**< Side of the last built full turn. */
    uint32_t bend_age;      /**< Periods since it, capped past memory. */
    uint32_t touch_count;   /**< Periods the current touch has lasted. */
} steering_t;

/** @brief Clear accumulated steering, as after a stop. */
void steering_reset (steering_t * p_steering);
/** @brief Update steering for one controller period.
 * @param action Latest line action; only the two corrections steer.
 * @param period_sec Time since the previous update.
 * @return Steer in [-STEERING_MAX, STEERING_MAX]; positive turns left.
 */
float steering_update (steering_t * p_steering, line_follow_action_t action,
                       float period_sec);

#endif /* STEERING_H */

/*** end of file ***/

