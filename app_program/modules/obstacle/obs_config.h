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
/* How close an obstacle may get before the robot stops. The beam is only
 * about 25 degrees wide, so nearer than this it no longer covers the full
 * body width and an object off to one side slips out of view.
 */
#define OBS_STOP_MM 250
#define OBS_CONFIRM_HITS 3U
#define OBS_NO_ECHO_LIMIT 5U
/* A value of 1 stops the robot when the sensor stays silent. */
#define OBS_FAILSAFE_NO_ECHO 0

/* Servo (SG90). FRONT is the SG90 centre, as first configured and as the
 * hardware diagnostic used. If the sensor is not square to the front at
 * startup, adjust FRONT: each 10 usec turns it about 1 degree, and a longer
 * pulse turns it left.
 */
#define OBS_SERVO_FRONT_USEC 1500
#define OBS_SERVO_USEC_PER_DEG 10
/* A value of 1 means that a longer pulse turns the sensor left. */
#define OBS_SERVO_LEFT_SIGN 1
/* SG90 pulse range; angles needing a pulse outside it are not used. */
#define OBS_SERVO_MIN_USEC 500
#define OBS_SERVO_MAX_USEC 2400
/* Measure: how far the servo can turn each side before it is blocked. The
 * side watch needs about 70; if the mount stops it sooner, set the real
 * reach here so the side-watch geometry stays right.
 */
#define OBS_SERVO_LEFT_LIMIT_DEG 70
#define OBS_SERVO_RIGHT_LIMIT_DEG 70
#define OBS_SERVO_SETTLE_BASE_MSEC 40
#define OBS_SERVO_SETTLE_PER_DEG_MSEC 3
/* Wait at startup: the power-on position is unknown, so allow a full turn. */
#define OBS_SERVO_HOME_MSEC 600U

/* Panning scan. The step must be a multiple of 10 degrees. */
#define OBS_SCAN_STEP_DEG 10
/* Sweep each side of FRONT; never more than the servo limits above. */
#define OBS_SCAN_LEFT_DEG 40
#define OBS_SCAN_RIGHT_DEG 40
/* Equals the left plus right sweep divided by the step, plus one. */
#define OBS_SCAN_POINTS 9
#define OBS_SCAN_PINGS 3U
#define OBS_SCAN_RANGE_MM 600
#define OBS_EDGE_JUMP_MM 80
#define OBS_FRONT_WINDOW_DEG 20
#define OBS_LOOKAHEAD_MM 500

/* Robot geometry, measured from the axle midpoint. The front and rear add
 * up to 200 mm against a 190 mm overall length, so the larger is kept.
 */
#define OBS_ROBOT_WIDTH_MM 153
#define OBS_ROBOT_FRONT_MM 135
#define OBS_ROBOT_REAR_MM 65
#define OBS_SAFETY_MARGIN_MM 40
#define OBS_FRONT_CLEAR_MM 30
#define OBS_TRACK_WIDTH_MM 110
/* Forward from the axle to the ultrasonic sensor, which sits centred. */
#define OBS_SENSOR_TO_AXLE_MM 110
/* Measure: forward from the axle to the line IR sensors. */
#define OBS_IR_TO_AXLE_MM 125

/* Bypass motion. Heading values are in tenths of a degree. */
#define OBS_DRIVE_RPM 25
/* Turns end this early, as the robot keeps turning while it slows. */
#define OBS_TURN_LEAD_DDEG 40
#define OBS_ALIGN_TOL_DDEG 100
#define OBS_HEADING_TRIM_DIV 60
#define OBS_HEADING_TRIM_MAX 5
#define OBS_STOP_SETTLE_MSEC 250U
#define OBS_SCAN_TIMEOUT_MSEC 12000U
#define OBS_STATE_TIMEOUT_MSEC 30000U

/* Sideways offset from the line while passing. The scan sets it; an object
 * that fills the sweep on a side has no visible edge, so that side uses
 * OBS_LATERAL_UNKNOWN_MM.
 */
#define OBS_LATERAL_UNKNOWN_MM 350
/* Object depth assumed when the side watch never sees it. */
#define OBS_DEFAULT_DEPTH_MM 150

/* Watching the object from the side while passing it. Nearly side-on, a
 * flat side echoes back and the watch sees exactly when the object ends;
 * at a shallow angle the echo glances off and the object goes unseen.
 * A side ping counts as the object only if it is nearer than
 * OBS_SIDE_NEAR_MM, further than OBS_SIDE_MIN_MM (closer is the robot's
 * own body), and lies between the robot and the line.
 */
#define OBS_SIDE_LOOK_DEG 70
#define OBS_SIDE_NEAR_MM 500
#define OBS_SIDE_MIN_MM 60
#define OBS_SIDE_LINE_SLACK_MM 30
/* Side pings that must see the object before its end is trusted, and side
 * pings without it before it is taken as passed.
 */
#define OBS_SIDE_SEEN_HITS 2U
#define OBS_SIDE_CLEAR_HITS 3U
/* Holding the passing offset: heading change per mm of offset error, in
 * tenths of a degree, and the most it may turn out or in.
 */
#define OBS_OFFSET_GAIN_DDEG_PER_MM 1
#define OBS_OFFSET_OUT_MAX_DDEG 150
#define OBS_OFFSET_IN_MAX_DDEG 50
/* Fault if the object still has not ended this far past its front face. */
#define OBS_PASS_MAX_MM 1200
/* Least distance the axle passes the object's rear before turning back in.
 * More is added when the robot runs close to the object, so that the body
 * side still clears the rear corner on the way back.
 */
#define OBS_PASS_MARGIN_MM 40

/* Checking ahead while bypassing: stop when something is this near ahead
 * for OBS_GUARD_HITS pings in a row. While passing, the sensor glances
 * ahead once every OBS_GLANCE_EVERY side pings.
 */
#define OBS_GUARD_MM 200
#define OBS_GUARD_HITS 2U
#define OBS_GLANCE_EVERY 4U
/* On the way back, ignore anything this far beyond the line. */
#define OBS_GUARD_BEYOND_LINE_MM 60
/* Black only counts as the line when the IR sensors should be this near
 * it; elsewhere it is a mark or a mistake.
 */
#define OBS_LINE_NEAR_MM 120
/* Stops to look again allowed in one bypass before giving up. */
#define OBS_REPLAN_MAX 3U

/* Finding the line again. The return run heads back at this angle, in
 * degrees. If the line is not found this far beyond where it should be,
 * the robot turns and crosses back over it, this many times, then faults.
 */
#define OBS_RETURN_DEG 60
#define OBS_SEARCH_MARGIN_MM 150
#define OBS_SEARCH_RETRIES 1U
#define OBS_LINE_CONFIRM_CYCLES 2U
#define OBS_VERIFY_CYCLES 10U

#endif /* OBS_CONFIG_H */

/*** end of file ***/

