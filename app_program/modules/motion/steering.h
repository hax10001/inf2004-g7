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
    /* Only the controller task updates this flag. */
    unsigned int b_off_centre:1; /**< Previous update saw correction. */
    signed int last_side:2; /**< Exactly -1, 0 or +1: last touch. */
    uint32_t centred_count; /**< Centred periods since that touch ended. */
    uint32_t hold_count;    /**< Periods left holding a full turn. */
    signed int bend_side:2; /**< Exactly -1, 0 or +1: prior bend. */
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

