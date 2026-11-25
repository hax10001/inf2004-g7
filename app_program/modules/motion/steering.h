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
/* The speed, RPM, the steering gains were tuned at. */
#define STEERING_REF_RPM 35.0f
/* Bound on the speed scale (105 RPM): beyond it the gains are untested. */
#define STEERING_SCALE_MAX 3.0f

typedef struct
{
    float    integral;      /**< Grows while off-centre, decays once centred. */
    /* Only the controller task updates this flag. */
    uint32_t     b_off_centre:1; /**< Previous update saw correction. */
    int32_t    last_side:2; /**< Exactly -1, 0 or +1: last touch. */
    /* The four counts below are in reference periods: one per controller
     * period at STEERING_REF_RPM, more at higher speed (see scale in
     * steering_update()), so they measure distance travelled. */
    float    centred_count; /**< Centred periods since that touch ended. */
    float    hold_count;    /**< Periods left holding a full turn. */
    int32_t    bend_side:2; /**< Exactly -1, 0 or +1: prior bend. */
    float    bend_age;      /**< Periods since it, capped past memory. */
    float    touch_count;   /**< Periods the current touch has lasted. */
} steering_t;

/* 1: steer with the line observer below (PID on where the line is, from
 * the sensors and the wheel encoders). 0: the PI steering above (fallback).
 */
#ifndef STEERING_OBSERVER
#define STEERING_OBSERVER 1
#endif

/** @brief Where the line is, estimated between sensor touches.
 *
 * Two sensors that straddle the line only say "left", "right" or "somewhere
 * in between". Driving blind in between drifts across to the other sensor,
 * giving a zig-zag. The wheel encoders measure how
 * far it drove and turned, so the line's position can be followed through
 * the gap and corrected whenever a sensor sees it. Steering is then a PID
 * on that estimate: P on the line's offset, D on the robot's heading across
 * it (from the encoders), I for curves. Only the controller task uses it.
 */
typedef struct
{
    float offset_mm;   /**< Line centre beside the sensor pair, + = left. */
    float heading_rad; /**< Robot heading relative to the line, + = left. */
    float integral;    /**< Steer held for a curve. */
    float since_mm;    /**< Travel since a sensor last corrected it. */
} line_track_t;

/** @brief Forget the estimate (line centred, robot along it). */
void line_track_reset (line_track_t * p_track);
/** @brief One controller period: move the estimate by what the wheels did,
 *  correct it from what the sensors saw, and return the steer.
 * @param b_left, b_right Each sensor saw the line this period.
 * @param left_mm, right_mm Each wheel's forward travel this period.
 * @return Steer in [-STEERING_MAX, STEERING_MAX]; positive turns left.
 */
float line_track_update (line_track_t * p_track, bool b_left, bool b_right,
                         float left_mm, float right_mm);

/** @brief Clear accumulated steering, as after a stop. */
void steering_reset (steering_t * p_steering);
/** @brief Update steering for one controller period.
 * @param action Latest line action; only the two corrections steer.
 * @param period_sec Time since the previous update.
 * @param scale Driving speed over STEERING_REF_RPM (0..STEERING_SCALE_MAX):
 * the integral, its decay and every period count advance by this much
 * each period, so a correction takes the same distance at any speed.
 * @return Steer in [-STEERING_MAX, STEERING_MAX]; positive turns left.
 */
float steering_update (steering_t * p_steering, line_follow_action_t action,
                       float period_sec, float scale);

#endif /* STEERING_H */

/*** end of file ***/

