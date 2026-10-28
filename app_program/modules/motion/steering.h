/** @file steering.h
 * @brief Proportional-integral steering from the discrete line action.
 */
#ifndef STEERING_H
#define STEERING_H

#include "modules/line_barcode/line_follow.h"
#include <stdbool.h>

/* Largest steer; at 0.8 the inner wheel keeps 20 percent of the target. */
#define STEERING_MAX 0.8f

typedef struct
{
    float integral;       /**< Grows while off-centre, decays once centred. */
    bool  b_off_centre;   /**< The previous update saw a correction. */
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

