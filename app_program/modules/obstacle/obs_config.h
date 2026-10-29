/** @file obs_config.h
 *
 * @brief Tunable values for the obstacle subsystem.
 *
 * NOTE: Each value marked "Measure" must be calibrated on the robot.
 */
#ifndef OBS_CONFIG_H
#define OBS_CONFIG_H

#include "board/board_config.h"

/* NOTE: Assumes a 125 MHz system clock, so a divider of 125 gives 1 usec. */
#define OBS_PWM_CLK_MHZ 125U

/* Ultrasonic ranging. */
#define OBS_TRIG_PULSE_USEC 12U
#define OBS_ECHO_TIMEOUT_MSEC 30U
#define OBS_PING_PERIOD_MSEC 60U
#define OBS_MEDIAN_SAMPLES 3U
#define OBS_FAR_MM 4000
/* Measure: how close an obstacle may get before the robot stops. */
#define OBS_STOP_MM 200
#define OBS_CONFIRM_HITS 3U
#define OBS_NO_ECHO_LIMIT 5U
/* A value of 1 stops the robot when the sensor stays silent. */
#define OBS_FAILSAFE_NO_ECHO 1

/* Servo (SG90). Measure every value in this group. */
#define OBS_SERVO_FRONT_USEC 800
#define OBS_SERVO_USEC_PER_DEG 10
/* A value of 1 means that a longer pulse turns the sensor left. */
#define OBS_SERVO_LEFT_SIGN -1
#define OBS_SERVO_MIN_USEC 600
#define OBS_SERVO_MAX_USEC 2400
/* Measure: how far the servo can turn each side before it is blocked. */
#define OBS_SERVO_LEFT_LIMIT_DEG 40
#define OBS_SERVO_RIGHT_LIMIT_DEG 40
/* The larger side; kept for code that assumes a symmetric sweep. */
#define OBS_SERVO_LIMIT_DEG 40
#define OBS_SERVO_SETTLE_BASE_MSEC 40
#define OBS_SERVO_SETTLE_PER_DEG_MSEC 3

/* Panning scan. The step must be a multiple of 10 degrees. */
#define OBS_SCAN_STEP_DEG 10
/* Sweep each side of FRONT; never more than the servo limits above. */
#define OBS_SCAN_LEFT_DEG 40
#define OBS_SCAN_RIGHT_DEG 40
/* The larger side; kept for code that assumes a symmetric sweep. */
#define OBS_SCAN_LIMIT_DEG 40
/* Equals the left plus right sweep divided by the step, plus one. */
#define OBS_SCAN_POINTS 9
#define OBS_SCAN_PINGS 3U
#define OBS_SCAN_RANGE_MM 600
#define OBS_EDGE_JUMP_MM 80
#define OBS_FRONT_WINDOW_DEG 20
#define OBS_LOOKAHEAD_MM 500

/* Robot geometry. Measure all of these; the track width can change. */
#define OBS_ROBOT_WIDTH_MM 150
#define OBS_ROBOT_LENGTH_MM 200
#define OBS_SAFETY_MARGIN_MM 40
#define OBS_FRONT_CLEAR_MM 400
#define OBS_TRACK_WIDTH_MM 110

/* Bypass motion. Heading values are in tenths of a degree. */
#define OBS_DRIVE_RPM 20
#define OBS_TURN_LEAD_DDEG 40
#define OBS_ALIGN_TOL_DDEG 100
#define OBS_HEADING_TRIM_DIV 70
#define OBS_HEADING_TRIM_MAX 4
#define OBS_STOP_SETTLE_MSEC 300U
#define OBS_SCAN_TIMEOUT_MSEC 12000U
#define OBS_STATE_TIMEOUT_MSEC 15000U

/* Passing the object and finding the line again. */
#define OBS_SIDE_LOOK_DEG 40
#define OBS_SIDE_NEAR_MM 400
#define OBS_SIDE_CLEAR_HITS 3U
#define OBS_PASS_MAX_MM 600
#define OBS_PASS_MARGIN_MM 40
#define OBS_SEARCH_MARGIN_MM 150
#define OBS_LINE_CONFIRM_CYCLES 2U
#define OBS_VERIFY_CYCLES 20U

#endif /* OBS_CONFIG_H */

/*** end of file ***/

