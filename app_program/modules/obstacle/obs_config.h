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
/* Front trigger is 100 mm closer than the previous 350 mm setting.
 * The confirmed stop is followed by a full side sweep; a narrow front beam
 * alone does not establish clearance for the robot's whole width. */
#define OBS_STOP_MM 250
#define OBS_CONFIRM_HITS 3U
#define OBS_NO_ECHO_LIMIT 5U
/* Persistent missing echoes are unverified rather than free space. */
#define OBS_FAILSAFE_NO_ECHO 1

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
#define OBS_SCAN_LEFT_DEG OBS_SERVO_LEFT_LIMIT_DEG
#define OBS_SCAN_RIGHT_DEG OBS_SERVO_RIGHT_LIMIT_DEG
/* Equals the left plus right sweep divided by the step, plus one. */
#define OBS_SCAN_POINTS \
    ((OBS_SCAN_LEFT_DEG + OBS_SCAN_RIGHT_DEG) / OBS_SCAN_STEP_DEG + 1)
#define OBS_SCAN_PINGS 3U
#define OBS_SCAN_RANGE_MM 600
#define OBS_EDGE_JUMP_MM 80
#define OBS_FRONT_WINDOW_DEG 20
#define OBS_LOOKAHEAD_MM 500

/* Choose the clear corridor needing the smaller sideways move. A blocked
 * corridor cannot be made safe by assuming unseen space beyond the sweep. */
#define OBS_PREFERRED_SIDE 0
/* Never pass closer than this sideways offset from the line, whatever the
 * scan says. A flat face stops echoing once the scan looks more than about
 * 30 degrees across it, so a wide object can look narrower than it is.
 * Set to half the widest obstacle + half the robot width + margin, e.g.
 * a 400 mm wide obstacle: 200 + 77 + 40 = 317. 0 turns it off.
 */
#define OBS_MIN_LATERAL_MM 245
/* OBS_MIN_LATERAL_MM only applies on a side whose last echo was at least
 * this many degrees off centre. A flat face keeps echoing to about 30
 * degrees, so a boundary nearer the front is a real edge and a narrow
 * obstacle keeps its tighter, measured offset.
 */
#define OBS_MIN_LATERAL_FROM_DEG 20
/* Turning parallel pivots on the wheel nearest the object, which should
 * carry the robot up to half a track further out. The stopped wheel drags
 * and slides, so by default none of that is counted: the robot drives the
 * whole offset before pivoting and any pivot gain is extra clearance.
 * 100 counts all of it (the old behaviour, a shorter drive out).
 */
#define OBS_PIVOT_GAIN_PCT 0

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
/* Watching the object from the side while passing it. A nearly side-on
 * view helps track a flat side. Glancing reflections can still be lost,
 * so a missing echo is not by itself proof that the object has ended.
 * Track valid echoes between the robot and the line, including ranges
 * beyond the old 500 mm cap. Closer than OBS_SIDE_MIN_MM is the body.
 */
#define OBS_SIDE_LOOK_DEG 70
#define OBS_SIDE_NEAR_MM OBS_FAR_MM
#define OBS_SIDE_MIN_MM 60
#define OBS_SIDE_LINE_SLACK_MM 30
/* First acquire the object, then require consecutive absent side pings.
 * No echo can also be a poor reflection: absence is only a candidate end.
 * Continue watching while travelling the full body clearance distance.
 */
#define OBS_SIDE_SEEN_HITS 2U
#define OBS_SIDE_CLEAR_HITS 5U
/* Allow the normal front glance and servo travel, but stop if the worker
 * stops publishing side samples. A cached clear sample cannot start a turn.
 */
#define OBS_SIDE_MAX_AGE_MSEC 1000U
/* Holding the passing offset: heading change per mm of offset error, in
 * tenths of a degree, and the most it may turn out or in.
 */
#define OBS_OFFSET_GAIN_DDEG_PER_MM 1
#define OBS_OFFSET_OUT_MAX_DDEG 150
#define OBS_OFFSET_IN_MAX_DDEG 50
/* Fault if the object still has not ended this far past its front face. */
#define OBS_PASS_MAX_MM 1200
/* If the side watch never gets an echo from the object (a smooth face seen
 * at an angle can reflect the sound away), assume it is this deep and turn
 * back once the body has cleared that, as for a seen object. If the object
 * is really longer, the front guard on the way back sees it and the robot
 * runs parallel again. Measure: set it to the deepest obstacle expected.
 */
#define OBS_UNSEEN_DEPTH_MM 300
/* Side-watch summary in the log every this many mm of travel; 0 = off. */
#define OBS_SIDE_LOG_MM 50
/* Return clearance uses the measured side gap and swept body geometry,
 * including OBS_SAFETY_MARGIN_MM. There is no additional fixed pass run.
 * The front-mounted sensor seeing the end is not yet body clearance. */

/* Checking ahead while bypassing: stop when something is this near ahead
 * for OBS_GUARD_HITS pings in a row.
 *
 * From the turn out until the obstacle has been passed, the servo keeps the
 * ultrasonic pointed at the obstacle. With OBS_PASS_FRONT_GLANCE set to 1 it
 * also glances ahead once every OBS_GLANCE_EVERY side pings (the old
 * behaviour); with 0 it never looks away, so nothing checks the path ahead
 * until the robot turns back towards the line.
 */
#define OBS_PASS_FRONT_GLANCE 0
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

/* Going back to the line. Odometry decides the return: the robot heads back
 * at OBS_RETURN_DEG and straightens up where the line should be, whether or
 * not the IR sensors saw it. Black seen near the expected line only
 * corrects the position estimate on the way.
 *
 * After straightening, line following runs for OBS_VERIFY_CYCLES controller
 * periods. If neither IR sensor saw black since the straightening turn began,
 * the robot zig-zags across the expected line at OBS_SEEK_DEG, reversing
 * OBS_SEEK_BAND_MM beyond it, for up to OBS_SEEK_LEGS legs, then stops.
 */
#define OBS_RETURN_DEG 60
#define OBS_LINE_CONFIRM_CYCLES 2U
#define OBS_VERIFY_CYCLES 10U
#define OBS_SEEK_DEG 30
#define OBS_SEEK_BAND_MM 150
#define OBS_SEEK_LEGS 4U
/* Servo steps used while pointing at the obstacle during the turn out. */
#define OBS_AIM_STEP_DEG 5

#endif /* OBS_CONFIG_H */

/*** end of file ***/

