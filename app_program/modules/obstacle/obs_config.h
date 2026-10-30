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
#define OBS_PING_PERIOD_MSEC 30U
#define OBS_MEDIAN_SAMPLES 3U
#define OBS_FAR_MM 4000
/* Measure: how close an obstacle may get before the robot stops. */
#define OBS_STOP_MM 250
#define OBS_CONFIRM_HITS 3U
#define OBS_NO_ECHO_LIMIT 5U
/* A value of 1 stops the robot when the sensor stays silent. */
#define OBS_FAILSAFE_NO_ECHO 1

/* Servo (SG90). Measure every value in this group. */
#define OBS_SERVO_FRONT_USEC 600
#define OBS_SERVO_USEC_PER_DEG 10
/* A value of 1 means that a longer pulse turns the sensor left. */
#define OBS_SERVO_LEFT_SIGN 1
#define OBS_SERVO_MIN_USEC 400
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
#define OBS_SCAN_RANGE_MM 400
#define OBS_EDGE_JUMP_MM 80
#define OBS_FRONT_WINDOW_DEG 20
#define OBS_LOOKAHEAD_MM 300

/* Robot geometry. Measure all of these; the track width can change. */
#define OBS_ROBOT_WIDTH_MM 150
#define OBS_ROBOT_LENGTH_MM 200
#define OBS_SAFETY_MARGIN_MM 40
#define OBS_FRONT_CLEAR_MM 20
#define OBS_TRACK_WIDTH_MM 110

/* Bypass motion. Heading values are in tenths of a degree. */
#define OBS_DRIVE_RPM 20
#define OBS_TURN_LEAD_DDEG 80
#define OBS_ALIGN_TOL_DDEG 300
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
#define OBS_ALIGN_MAX_MSEC 2500U
#define OBS_VERIFY_CYCLES 6U

/* Side look and forward safety while passing the object. */
/* cos(OBS_SIDE_LOOK_DEG) x 1000; update it if the look angle changes. */
#define OBS_SIDE_COS_PERMILLE 766
/* Travel allowed between stops to check
forward while the sensor looks aside. */
#define OBS_FWD_CHECK_MM 100
/* Forward range needed to carry on: the stop range plus the next leg. */
#define OBS_FWD_CLEAR_MM (OBS_STOP_MM + OBS_FWD_CHECK_MM)
#define OBS_FWD_PINGS 3U
/* Measure: front range that stops a moving leg; must stay below OBS_STOP_MM. */
#define OBS_BYPASS_STOP_MM 150
/* A value of 1 lets the plan's dead-reckoned distance replace seeing the
 * object. Seeing the far edge is the default; unseen means halt.
 */
#define OBS_ALLOW_BLIND_PASS 0

/* Handing back to line following. */
/* A value of 1 means the line continues along the heading before the bypass. */
#define OBS_TRACK_STRAIGHT 1
#define OBS_VERIFY_MM 60
#define OBS_VERIFY_MAX_MSEC 4000U

/* Longest wait for the status lock; the lock is held for microseconds. */
#define OBS_LOCK_TIMEOUT_MSEC 20

/* Every servo bearing must fit inside the pulse clamp, otherwise several scan
 * angles would silently share one physical direction. A negative array size
 * makes the build fail when a limit does not fit. Set OBS_SERVO_REACH_CHECK
 * to 0 only for bench builds; the scan and side look are wrong without it.
 */
#define OBS_SERVO_REACH_CHECK 1
#define OBS_SERVO_PULSE_LEFT_USEC                                              \
    (OBS_SERVO_FRONT_USEC + (OBS_SERVO_LEFT_SIGN * OBS_SERVO_LEFT_LIMIT_DEG *  \
                             OBS_SERVO_USEC_PER_DEG))
#define OBS_SERVO_PULSE_RIGHT_USEC                                             \
    (OBS_SERVO_FRONT_USEC - (OBS_SERVO_LEFT_SIGN * OBS_SERVO_RIGHT_LIMIT_DEG * \
                             OBS_SERVO_USEC_PER_DEG))
#define OBS_SERVO_REACH_OK                                                     \
    ((OBS_SERVO_PULSE_LEFT_USEC >= OBS_SERVO_MIN_USEC) &&                      \
     (OBS_SERVO_PULSE_LEFT_USEC <= OBS_SERVO_MAX_USEC) &&                      \
     (OBS_SERVO_PULSE_RIGHT_USEC >= OBS_SERVO_MIN_USEC) &&                     \
     (OBS_SERVO_PULSE_RIGHT_USEC <= OBS_SERVO_MAX_USEC) &&                     \
     (OBS_SCAN_LEFT_DEG <= OBS_SERVO_LEFT_LIMIT_DEG) &&                        \
     (OBS_SCAN_RIGHT_DEG <= OBS_SERVO_RIGHT_LIMIT_DEG) &&                      \
     (OBS_SIDE_LOOK_DEG <= OBS_SERVO_LEFT_LIMIT_DEG) &&                        \
     (OBS_SIDE_LOOK_DEG <= OBS_SERVO_RIGHT_LIMIT_DEG))
#if OBS_SERVO_REACH_CHECK
typedef char obs_config_servo_reach_check_t[OBS_SERVO_REACH_OK ? 1 : -1];
#endif

#endif /* OBS_CONFIG_H */

/*** end of file ***/

