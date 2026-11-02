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
#define OBS_STOP_MM 350
#define OBS_CONFIRM_HITS 3U
#define OBS_NO_ECHO_LIMIT 5U
/* Persistent missing echoes are an unverified path, not free space. */
#define OBS_FAILSAFE_NO_ECHO 1

/* Servo (SG90). FRONT is the pulse at which the transducer beam points
 * straight ahead along the robot's x axis. It is a calibration, not the
 * SG90 mechanical centre: the horn splines and the bracket can leave the
 * beam off square at 1500 usec. Measure with OBS_TEST_SERVO_CAL in
 * test_obs.c; each 10 usec turns the beam about 1 degree.
 */
#define OBS_SERVO_FRONT_USEC 1500
/* SG90 mechanical centre, used only to report the forward shaft angle. */
#define OBS_SERVO_CENTRE_USEC 1500
#define OBS_SERVO_USEC_PER_DEG 10
/* A value of 1 means that a longer pulse turns the sensor left. Measure:
 * a servo mounted upside down reverses this.
 */
#define OBS_SERVO_LEFT_SIGN 1
/* SG90 pulse range; angles needing a pulse outside it are not used. With
 * FRONT at 1500 the pulse range allows 90 degrees left and 100 right.
 */
#define OBS_SERVO_MIN_USEC 500
#define OBS_SERVO_MAX_USEC 2400
/* Measure: how far the beam can turn each side of FRONT before the sensor,
 * its bracket or its wires touch the car. The scan, side watch and return
 * check all stay inside these. 70 is the last value used on the robot; 75
 * gives the 150 degree arc but only once OBS_TEST_FIND_LIMITS confirms it.
 */
#ifndef OBS_SERVO_LEFT_LIMIT_DEG
#define OBS_SERVO_LEFT_LIMIT_DEG 70
#endif
#ifndef OBS_SERVO_RIGHT_LIMIT_DEG
#define OBS_SERVO_RIGHT_LIMIT_DEG 70
#endif
/* Measure: 1 when the servo shaft is vertical, so that the beam pans left
 * and right in the horizontal plane. A shaft mounted horizontally tilts
 * the beam up and down instead; that cannot see left or right, and the
 * build stops with an error rather than misread pitch as bearing.
 */
#define OBS_SERVO_PANS_HORIZONTALLY 1
#define OBS_SERVO_SETTLE_BASE_MSEC 40
#define OBS_SERVO_SETTLE_PER_DEG_MSEC 3
/* Wait at startup: the power-on position is unknown, so allow a full turn. */
#define OBS_SERVO_HOME_MSEC 600U

/* Panning scan. The step must be a multiple of 10 degrees. */
#define OBS_SCAN_STEP_DEG 10
/* Profile each side of FRONT, used to find the object's edges. */
#define OBS_SCAN_LEFT_DEG 40
#define OBS_SCAN_RIGHT_DEG 40
/* Equals the left plus right profile divided by the step, plus one. */
#define OBS_SCAN_POINTS 9
/* The same sweep also reads one far bearing each side, for the departure
 * corridors. It is cut back to the servo limits, so 75 gives the full
 * 150 degree arc only when both limits allow it.
 */
#define OBS_SCAN_FAR_DEG 75
#define OBS_SCAN_MAX_POINTS (OBS_SCAN_POINTS + 2)
/* Up to this many pings per bearing; two that agree within
 * OBS_SCAN_AGREE_MM already bound the median of three, so the third is
 * skipped.
 */
#define OBS_SCAN_PINGS 3U
#define OBS_SCAN_AGREE_MM 30
#define OBS_SCAN_RANGE_MM 600
#define OBS_EDGE_JUMP_MM 80
#define OBS_FRONT_WINDOW_DEG 20
#define OBS_LOOKAHEAD_MM 500

/* Robot geometry. Vehicle frame: origin at the midpoint of the driven-
 * wheel axle, x forward, y left, z up. The front and rear add up to 200 mm
 * against a 190 mm overall length, so the larger is kept.
 */
#define OBS_ROBOT_WIDTH_MM 153
#define OBS_ROBOT_FRONT_MM 135
#define OBS_ROBOT_REAR_MM 65
#define OBS_TRACK_WIDTH_MM 110
/* Gap kept between the car body and an object. The passing offset is the
 * object edge plus half the body width plus this margin plus
 * OBS_OUTWARD_EXTRA_MM; verify on the actual course surface.
 */
#define OBS_SAFETY_MARGIN_MM 65
/* Allowance for where the car really is: the dead-reckoned position drifts
 * over the pivot turns of the departure leg. It is added to the passing
 * offset on top of the body margin.
 */
#define OBS_OUTWARD_EXTRA_MM 40
/* Measure: forward from the axle to the line IR sensors. */
#define OBS_IR_TO_AXLE_MM 125

/* Ultrasonic mounting, in the vehicle frame with the servo at FRONT.
 * OBS_SENSOR_TO_AXLE_MM is the transducer face, as used so far. The others
 * are Measure values: until measured they are 0, which places the servo
 * axis under the transducer and keeps the earlier behaviour.
 */
#define OBS_SENSOR_TO_AXLE_MM 110
/* Measure: transducer centre left of the axle midpoint. */
#ifndef OBS_SENSOR_LATERAL_MM
#define OBS_SENSOR_LATERAL_MM 0
#endif
/* Measure: transducer centre above the floor. It sets which low objects
 * the beam can miss; the planar maths does not use it.
 */
#define OBS_SENSOR_HEIGHT_MM 0
/* Measure: transducer face ahead of the servo rotation axis, with a ruler
 * from the shaft centre. The servo carries the face round that axis, so at
 * bearing b the range starts at axis + arm * (cos b, sin b).
 */
#ifndef OBS_SERVO_ARM_MM
#define OBS_SERVO_ARM_MM 0
#endif
/* Set to 1 once the four values above are measured on the robot. */
#define OBS_GEOMETRY_MEASURED 0

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
#define OBS_LATERAL_UNKNOWN_MM 430
/* With both sides usable, a side needing this much less passing offset
 * wins; closer than this, the side with more checked room beyond wins.
 */
#define OBS_SIDE_PREFER_MM 40
/* During the diagonal departure, block nearby lateral obstructions. */
#define OBS_OUTWARD_GUARD_MM 190
/* Object depth assumed until the side watch has seen the face clear
 * beyond it.
 */
#define OBS_DEFAULT_DEPTH_MM 150
/* The thinnest object the side watch must still see: the planner prefers
 * a departure angle that leaves the side beam looking at the object's side
 * no later than this far behind its front face. An object the side watch
 * never sees is never taken as passed.
 */
#define OBS_MIN_DEPTH_MM 40

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
/* Confirmed body-side gap below this needs a new avoidance decision. */
#define OBS_SIDE_CRITICAL_MM 100
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
/* Most the passing offset may grow over the plan while passing; more needs
 * a new stationary decision, so a correction never runs into anything
 * beyond the corridor the scan checked.
 */
#define OBS_PASS_CORRECT_MAX_MM 60
/* Fault if the object still has not ended this far past its front face. */
#define OBS_PASS_MAX_MM 1200
/* The rear of the car, OBS_ROBOT_REAR_MM behind the axle, must be this far
 * past the object's estimated rear before turning back in. With the rear
 * overhang this is the 110 mm axle margin used before.
 */
#define OBS_REAR_PASS_MARGIN_MM 45
/* Axle travel since the last side hit before an end is credible. */
#define OBS_SIDE_CLEAR_TRAVEL_MM 110
/* Consecutive side pings with no echo at all before the sensor is taken
 * as lost; a missing echo never counts as clear.
 */
#define OBS_SIDE_NO_ECHO_LIMIT 5U
/* The return check is aimed this far before the turn point, so a fresh
 * reading is ready on arrival and the car need not stop to wait.
 */
#define OBS_RETURN_PROBE_LEAD_MM 60
/* With the way back blocked, pass on this far before checking again. It
 * is longer than the probe lead, so the side watch looks at the object
 * again between two looks down the way back.
 */
#define OBS_RETURN_RECHECK_MM 100
/* A forward reading older than this much travel no longer verifies the
 * way ahead while passing.
 */
#define OBS_FRONT_STALE_MM 150
/* Longest wait for a requested servo reading before a servo fault. */
#define OBS_READING_WAIT_MSEC 2000U
/* Range error allowed for: speed of sound and timing. */
#define OBS_RANGE_UNCERT_MM 10

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

#if (1 != OBS_SERVO_PANS_HORIZONTALLY)
#error "The servo must pan the beam horizontally to scan left and right."
#endif

#endif /* OBS_CONFIG_H */

/*** end of file ***/

