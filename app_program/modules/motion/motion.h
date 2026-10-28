/** @file motion.h
 * @brief Motion subsystem lifecycle and line-following movement interface.
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "modules/line_barcode/line_follow.h"

typedef struct
{
    float    left_rpm;         /**< Measured; direction is not reported. */
    float    right_rpm;        /**< Measured; direction is not reported. */
    float    left_target_rpm;  /**< Ramped target; zero when stopped. */
    float    right_target_rpm; /**< Ramped target; zero when stopped. */
    float    steer;            /**< -0.8 to 0.8; positive turns left. */
    float    speed_mm_per_sec; /**< Vehicle speed: mean of both wheels. */
    int32_t  left_count;       /**< Raw encoder count, for distance. */
    int32_t  right_count;      /**< Raw encoder count, for distance. */
    uint32_t left_pwm;         /**< Applied compare count. */
    uint32_t right_pwm;        /**< Applied compare count. */
    uint32_t timestamp_usec;   /**< Encoder timer time of this update. */
    bool     b_valid;          /**< False until the first update. */
} motion_status_t;

/** @brief Initialise motors, encoders and status storage; outputs stopped. */
ER motion_init (void);
/** @brief Complete startup; this subsystem has no independent task yet. */
ER motion_start (void);
/** @brief Steer from the line action and drive each wheel to its target.
 * @note Call once per controller period; the integral assumes that rate.
 */
void motion_apply (line_follow_action_t action);
/** @brief Copy the latest wheel speeds under their mutex, for telemetry.
 * @return E_OK, E_PAR for NULL, E_OBJ before motion_init, or a lock error.
 */
ER motion_get_status (motion_status_t * p_output);

#endif /* MOTION_H */

/*** end of file ***/

