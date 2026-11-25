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
/* Before a trigger, wait this long at most for ECHO to fall from the last
 * ping; still high counts as no echo (a stuck ECHO line is a fault). */
#define OBS_ECHO_CLEAR_MSEC 50U
/* Minimum time between two triggers (HC-SR04 guidance: lets the previous
 * ping's echoes die away). Scans space pings by this from trigger to
 * trigger, overlapping the echo and servo settling with it. */
#define OBS_PING_PERIOD_MSEC 60U
#define OBS_MEDIAN_SAMPLES 3U
#define OBS_FAR_MM 4000
/* Front stop range, mm. A confirmed stop is followed by a full side sweep;
 * a narrow front beam alone does not show clearance for the whole width. */
#define OBS_STOP_MM 250
#define OBS_CONFIRM_HITS 3U
#define OBS_NO_ECHO_LIMIT 5U
/* Persistent missing echoes are unverified rather than free space. */
#define OBS_FAILSAFE_NO_ECHO 1

/* Servo (SG90). FRONT is the pulse that aims the sonar along the robot's
 * forward axis; each 10 usec turns it about 1 degree, a longer pulse turns
 * it left. 1500 aims a few degrees right, so 1540 is used. Check: a box
 * centred on the line should log "Square to the face" near 0.
 */
#define OBS_SERVO_FRONT_USEC 1540
#define OBS_SERVO_USEC_PER_DEG 10
/* 1: a longer pulse turns the sensor left. With -1 every bearing is
 * mirrored and the scan and side watch look at the wrong side. Keep 1. */
#define OBS_SERVO_LEFT_SIGN 1
/* SG90 pulse range; angles needing a pulse outside it are not used. */
#define OBS_SERVO_MIN_USEC 500
#define OBS_SERVO_MAX_USEC 2400
/* Measure: how far the servo can turn each side, degrees. The left limit
 * is 60 (2140 usec) because the servo strains beyond it; it is a multiple
 * of OBS_SCAN_STEP_DEG so the sweep ends on the limit. The side watch
 * clamps to these limits.
 */
#define OBS_SERVO_LEFT_LIMIT_DEG 60
#define OBS_SERVO_RIGHT_LIMIT_DEG 70
#define OBS_SERVO_SETTLE_BASE_MSEC 40
#define OBS_SERVO_SETTLE_PER_DEG_MSEC 3
/* Scan moves always arrive moving right: this SG90 turns right smoothly
 * but springs back after turning left. A move right is ramped in
 * OBS_SERVO_RAMP_DEG steps OBS_SERVO_RAMP_MSEC apart; a move left first
 * swings to the left limit, waits OBS_SERVO_SWING_SETTLE_MSEC, then comes
 * right to the bearing. The side watch while passing moves directly. */
#define OBS_SERVO_SWING_SETTLE_MSEC 250U
#define OBS_SERVO_RAMP_DEG 10
#define OBS_SERVO_RAMP_MSEC 20U
/* Wait at startup: the power-on position is unknown, so allow a full turn. */
#define OBS_SERVO_HOME_MSEC 600U

/* Panning scan bearings, listed from the right end to the left end.
 * Inside +-OBS_SCAN_FINE_DEG, where the object usually is, it steps
 * OBS_SCAN_FINE_STEP_DEG (a multiple of 5 that divides the step). In the
 * corner band beyond it, out to +-OBS_CORNER_MAX_DEG, it steps
 * OBS_CORNER_STEP_DEG; outside that, OBS_SCAN_STEP_DEG (a multiple of 10).
 * Each extra point adds about 0.2 s to a scan pass.
 * OBS_SCAN_FINE_DEG: a multiple of OBS_SCAN_STEP_DEG; 0 turns it off.
 */
#define OBS_SCAN_STEP_DEG 10
#define OBS_SCAN_FINE_DEG 30
#define OBS_SCAN_FINE_STEP_DEG 5
/* Sweep each side of FRONT; never more than the servo limits above. */
#define OBS_SCAN_LEFT_DEG OBS_SERVO_LEFT_LIMIT_DEG
#define OBS_SCAN_RIGHT_DEG OBS_SERVO_RIGHT_LIMIT_DEG
/* Fine points across the front zone, the corner band each side, then the
 * coarse ones out to each end. */
#define OBS_SCAN_POINTS \
    (((2 * OBS_SCAN_FINE_DEG) / OBS_SCAN_FINE_STEP_DEG) + 1 + \
     ((2 * (OBS_CORNER_MAX_DEG - OBS_SCAN_FINE_DEG)) / OBS_CORNER_STEP_DEG) + \
     ((OBS_SCAN_LEFT_DEG - OBS_CORNER_MAX_DEG) / OBS_SCAN_STEP_DEG) + \
     ((OBS_SCAN_RIGHT_DEG - OBS_CORNER_MAX_DEG) / OBS_SCAN_STEP_DEG))
/* Sweep direction. The servo settles slightly off each target, the same
 * way for each approach direction, which shifts a one-way sweep's bearings.
 *   0 = both ways: left to right, then right to left over
 *       +-OBS_SCAN_RETURN_DEG; each bearing keeps the nearer reading.
 *   1 = right to left only; -1 = left to right only (default, matches the
 *       right-arriving servo moves above).
 */
#ifndef OBS_SCAN_DIRECTION
#define OBS_SCAN_DIRECTION (-1)
#endif
/* Pings per bearing per pass: two, and a third only when those two differ
 * by more than OBS_SCAN_AGREE_MM (a weak edge or a stray echo); the bearing
 * then reads the median of three. Two that agree read their average. */
#define OBS_SCAN_PINGS 2U
#define OBS_SCAN_AGREE_MM 50
/* Two-way scan only: the return pass re-reads only bearings within
 * +-OBS_SCAN_RETURN_DEG, where the edges are found; outer bearings keep the
 * first pass's reading. Set to the sweep limit for a full return pass. */
#define OBS_SCAN_RETURN_DEG OBS_SCAN_FINE_DEG
/* Corner band: bearings beyond +-OBS_SCAN_FINE_DEG up to this. A flat face
 * square to the sensor acts as a mirror: every bearing within about 30
 * degrees returns the perpendicular distance, so the echo span shows the
 * sensor's limit, not the box's width or offset. Only the box's vertical
 * corners echo further out, and their range gives the corner's true
 * sideways position, sqrt(range^2 - depth^2). Each bearing in this band
 * takes up to OBS_CORNER_PINGS pings until two agree on an echo. A multiple
 * of OBS_SCAN_STEP_DEG, OBS_SCAN_FINE_DEG..sweep limit. */
#define OBS_CORNER_MAX_DEG 50
#define OBS_CORNER_PINGS 4U
/* Corner band bearing step, degrees. A box 3-7 cm off the line has its far
 * corner 26-35 degrees out, so the band needs 5 degree steps. A multiple
 * of 5 that divides OBS_CORNER_MAX_DEG - OBS_SCAN_FINE_DEG. */
#define OBS_CORNER_STEP_DEG 5
/* A corner heard on only one of a bearing's pings is kept when the next
 * corner bearing (OBS_CORNER_STEP_DEG on) heard it too, within this many mm:
 * the same corner seen from both. */
#define OBS_CORNER_NEIGHBOUR_MM 30
/* An echo in the corner band is the box's corner when it is no more than
 * this beyond the face depth (a 15 cm box up to ~25 cm off centre at 22 cm
 * ahead). Farther echoes are something else and are left to the corridor
 * check. */
#define OBS_CORNER_DEPTH_MM 150
/* A bearing hears a corner at most this many degrees off its axis (the
 * sensor's half beam), so a corner band echo nearer the line than
 * face * tan(bearing - this) is the face itself still echoing, not a
 * corner. Multiple of 5, below OBS_SCAN_FINE_DEG. */
#define OBS_CORNER_BEAM_DEG 15
/* Slow edge vote after the scan passes. Box edges echo only some of the
 * time, so each bearing OBS_EDGE_VOTE_MIN_DEG..MAX_DEG either side is
 * visited again, from the centre outwards on both sides, settled
 * OBS_EDGE_VOTE_SETTLE_MSEC longer and pinged OBS_EDGE_VOTE_PINGS times
 * OBS_EDGE_VOTE_GAP_MSEC apart. A ping is "near" when it echoes within
 * OBS_CORNER_DEPTH_MM beyond the face. The bearing is the object when most
 * pings are near (their median), else open (the median of the rest); a
 * minority of near pings is kept as a lone corner echo. About 0.6 s per
 * bearing. 0 turns it off. PINGS: odd, 3..15. Bearings: multiples of 5.
 */
#define OBS_EDGE_VOTE 1
#define OBS_EDGE_VOTE_MIN_DEG 20
#define OBS_EDGE_VOTE_MAX_DEG 45
#define OBS_EDGE_VOTE_PINGS 5U
#define OBS_EDGE_VOTE_SETTLE_MSEC 60U
#define OBS_EDGE_VOTE_GAP_MSEC 70U
/* 1: a run of silent bearings next to the object's echoing face, ended by
 * a real echo and within OBS_CORNER_MAX_DEG, counts as the face seen at a
 * slant, at the edge's range. A flat face mirrors the ping away beyond its
 * square-on zone, so silence there means the box is still in the beam. A
 * silent run that nothing ends may be open space and stays open. 0: off. */
#ifndef OBS_GLANCE_AS_FACE
#define OBS_GLANCE_AS_FACE 1
#endif
/* Silent bearings in a row needed (one alone may be a dropped ping). */
#ifndef OBS_GLANCE_MIN_RUN
#define OBS_GLANCE_MIN_RUN 2
#endif
/* The robot seldom stops square to the box, and where the face leans it
 * keeps echoing further out, which looks like a corner. Each scan measures
 * the face normal and a corner band echo counts only when it is at least
 * OBS_CORNER_FROM_DEG off it. The normal is the range-weighted centre of
 * the bearings within OBS_FACE_BOWL_MM of the nearest echo, limited to
 * +-OBS_FACE_TILT_MAX_DEG (the robot can stop ~20 degrees off square). */
#define OBS_CORNER_FROM_DEG 35
#define OBS_FACE_BOWL_MM 10
#define OBS_FACE_TILT_MAX_DEG 25
/* 1: when a corner is found, choose the side from the object's real
 * extent: the seen corner(s), and for an unseen one the face's own edge if
 * it stopped echoing inside +-OBS_SCAN_FINE_DEG, else the far edge of an
 * OBS_OBJECT_MAX_MM wide object. The robot also drives out at least far
 * enough to clear a seen corner on its side. With no corner echo the edges
 * decide. 0: off. */
#define OBS_SIDE_CORNER_ECHO 1
/* Creep closer before scanning (first stop only): from nearer, more of an
 * off-centre box's corners fall outside the square-on zone and echo more
 * strongly. Creeps at OBS_APPROACH_RPM on fresh front pings until the face
 * is OBS_SCAN_FROM_MM away, at most OBS_APPROACH_MAX_MM or
 * OBS_APPROACH_MAX_MSEC, and stops early after OBS_APPROACH_MISSES pings
 * in a row without the face. 0 turns it off; off by default because the
 * closer start leaves less room to clear the box on the turn out. */
#define OBS_SCAN_FROM_MM 0
#define OBS_APPROACH_RPM 12
#define OBS_APPROACH_MAX_MM 120
#define OBS_APPROACH_MAX_MSEC 4000U
#define OBS_APPROACH_MISSES 3U
#define OBS_SCAN_RANGE_MM 600
#define OBS_EDGE_JUMP_MM 80
#define OBS_FRONT_WINDOW_DEG 20
#define OBS_LOOKAHEAD_MM 500
/* Use a fresh reading at servo zero for the scan geometry. The earlier
 * stop range can still bound turn clearance. */
#ifndef OBS_FRESH_SCAN_CENTRE
#define OBS_FRESH_SCAN_CENTRE 1
#endif

/* 0: choose the clear corridor needing the smaller sideways move (see
 * least resistance below). 1 or -1: always take left or
 * right while it is clear. A blocked corridor cannot be made safe by
 * assuming unseen space beyond the sweep. */
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
 * 0..100 percent of the predicted gain.
 */
#define OBS_PIVOT_GAIN_PCT 0
/* Wide initial passes gain about 45-75 mm in odometry while turning
 * parallel, on top of the planned offset. Count this percentage of the
 * predicted pivot gain for them. Narrow passes and rescans use the
 * regular gain above. 0 drives the full offset out. */
#ifndef OBS_WIDE_PASS_PIVOT_GAIN_PCT
#define OBS_WIDE_PASS_PIVOT_GAIN_PCT 50
#endif

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
#define OBS_HEADING_TRIM_DIV 60
#define OBS_HEADING_TRIM_MAX 5
#define OBS_STOP_SETTLE_MSEC 250U
/* A one-way scan takes about 8-10 s, both ways about 16-20 s. */
#define OBS_SCAN_TIMEOUT_MSEC 30000U
#define OBS_STATE_TIMEOUT_MSEC 30000U

/* Sideways offset from the line while passing. The scan sets it; an object
 * that fills the sweep on a side has no visible edge, so that side uses
 * OBS_LATERAL_UNKNOWN_MM.
 */
#define OBS_LATERAL_UNKNOWN_MM 350
/* Watching the object from the side while passing it. Glancing reflections
 * can be lost, so a missing echo alone does not prove the object has
 * ended. Valid echoes between the robot and the line are tracked; closer
 * than OBS_SIDE_MIN_MM is the robot's own body.
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
 * tenths of a degree, and the most it may turn out. While the object is
 * beside the robot it never turns in towards it, because odometry can
 * overestimate the offset and an inward correction would steer the front
 * corner into the object.
 */
#define OBS_OFFSET_GAIN_DDEG_PER_MM 1
#define OBS_OFFSET_OUT_MAX_DDEG 150
/* Measured push-out: when a side echo shows the gap to the object short
 * of half the robot width plus OBS_SAFETY_MARGIN_MM, turn out by this
 * many tenths of a degree per mm short (capped at OBS_OFFSET_OUT_MAX_DDEG),
 * whatever odometry says. 3 turns 15 degrees out for a 50 mm shortfall. */
#define OBS_SIDE_PUSH_GAIN_DDEG_PER_MM 3

/* ---- Bypass side choice and straight-leg length -------------------------
 * All distances are millimetres. Travel is measured by encoder odometry
 * along the line from where the obstacle was confirmed.
 *
 * Side choice, least resistance: the scan works out how far sideways each
 * side needs the robot to move to clear the object (the object's edge plus
 * half the robot width plus OBS_SAFETY_MARGIN_MM; with a corner echo, its
 * real extent). The clear side needing clearly less wins; within
 * OBS_SIDE_PREFERENCE_THRESHOLD_MM the box is about centred and the side
 * with more room past it wins (scan_plan.c, least_resistance()). There is
 * no fixed default side. OBS_PREFERRED_SIDE, if not 0, overrides all of
 * these.
 */
/* Half-width of the ultrasonic beam, degrees (multiple of 5, 0..30). A
 * bearing echoes while the object is anywhere within it, so edges appear
 * this far outside the real ones. Used ONLY for the reported object width
 * (log, dashboard); bypass offsets still use the conservative edges. */
#define OBS_BEAM_HALF_DEG 10
/* The box counts as about centred when the two sides' moves (measured from
 * the face's normal, or from corner echoes) differ by no more than this; it
 * then goes to the side with more room past it (the nearest echo beyond
 * each edge, then the average). About the sonar's edge resolution, so only
 * truly centred boxes use the room tie-break. 0..255 mm. */
#define OBS_SIDE_PREFERENCE_THRESHOLD_MM 15
/* 1: compare the two sides using the beam-corrected edges (each edge
 * midway to the next empty bearing). The conservative edges, padded out to
 * the next empty bearing, depend on the bearing spacing (5 degrees within
 * +-30, 10 outside), which tilts close calls. The drive offset still uses
 * the conservative edges. 0: compare the conservative edges. */
#define OBS_SIDE_COMPARE_MEASURED 1
/* Project corner evidence onto the sonar's straight-ahead axis. A longer
 * echo at the end of the face can break a tie. Reducing drive clearance
 * requires both corners. 0 disables projected geometry. */
#ifndef OBS_FORWARD_EDGE_SPLIT
#define OBS_FORWARD_EDGE_SPLIT 1
#endif
#define OBS_EDGE_SPLIT_JUMP_MM 25
#define OBS_EDGE_PROJECTION_PAD_MM 25
/* OBS_PASS_DEFAULT_MM: fallback length of the straight leg, as sensor travel
 * past the object's front face. If the side watch has not confirmed the end
 * (OBS_SIDE_CLEAR_HITS absent pings in a row) and has not seen the object in
 * the last OBS_SIDE_CLEAR_HITS pings, the object is assumed to end here and
 * the robot turns back once its body has cleared that point. A confirmed
 * end still finishes the leg earlier; an object still being seen keeps it
 * going. If the object is really longer, the front guard on the way back
 * sees it and the robot runs parallel again. The default end is never
 * shorter than the beam-corrected scanned width (capped at
 * OBS_OBJECT_MAX_MM): depth is not measured, so a box is assumed about as
 * deep as it is wide. 1..OBS_PASS_MAX_MM - 1 mm.
 *
 * OBS_PASS_MAX_MM: hard limit. Stop with a NO_ROUTE fault if the axle gets
 * this far past the front face while still passing, whatever the side watch
 * says. Must exceed OBS_PASS_DEFAULT_MM. 0..32767 mm.
 */
#define OBS_PASS_DEFAULT_MM 40
#define OBS_PASS_MAX_MM 600
/* Largest obstacle on the course (width and depth), mm. Caps the depth
 * assumed when the side watch cannot confirm the end. 15 cm boxes only. */
#define OBS_OBJECT_MAX_MM 150
/* 1 prints the side clearances and choice, and how and where the straight
 * leg ended; 0 turns those lines off.
 */
#define OBS_DEBUG_LOG 1
/* ------------------------------------------------------------------------ */

/* Side-watch summary in the log every this many mm of travel; 0 = off. */
#define OBS_SIDE_LOG_MM 50

/* Checking ahead while bypassing: stop when something is this near ahead
 * for OBS_GUARD_HITS pings in a row.
 *
 * From the turn out until the obstacle has been passed, the servo keeps the
 * ultrasonic pointed at the obstacle. With OBS_PASS_FRONT_GLANCE set to 1 it
 * also glances ahead once every OBS_GLANCE_EVERY side pings; with 0 it never
 * looks away, so nothing checks the path ahead until the robot turns back
 * towards the line.
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

/* Going back to the line. The robot heads back at OBS_RETURN_DEG towards
 * where odometry puts the line (see OBS_RETURN_TO_BLACK for the IR-led
 * return).
 *
 * After straightening, line following runs for OBS_VERIFY_CYCLES controller
 * periods. If neither IR sensor saw black since the straightening turn began,
 * the robot zig-zags across the expected line at OBS_SEEK_DEG, reversing
 * OBS_SEEK_BAND_MM beyond it, for up to OBS_SEEK_LEGS legs, then stops.
 *
 * Straightening up on the main return (a turn of at least
 * OBS_ALIGN_CHECK_DEG) sweeps the IR sensors back over the line. Only when
 * the leading sensor alone crossed it is the line between them. If both
 * did, or neither did after it was seen on the way in, the robot is beside
 * the line, so a seek leg heads back to it (it counts towards
 * OBS_SEEK_LEGS). Seek legs and small turns are not checked.
 *
 * OBS_RETURN_DEG: 30..90. 90 crosses the line square, but turns a full
 * 90 degrees to straighten up and needs more travel past the object first.
 * The pass end and body clearance are derived from it.
 */
#ifndef OBS_RETURN_DEG
#define OBS_RETURN_DEG 75
#endif
#define OBS_ALIGN_CHECK_DEG 30
/* Added to the REPORTED object width only (the "[OBS] Object" log line
 * and the telemetry width), never to the plan. The beam-corrected width
 * of a 150 mm box reads about 107 mm, so +43 mm centres it. 0 = report
 * the measured width. */
#define OBS_WIDTH_REPORT_ADD_MM 43
/* 1: return to the line by the IR sensors, not only by odometry. When the
 * robot stopped at an angle to the line, the odometry line is wrong. With
 * this on:
 *  - on the way back any black under a line sensor (seen
 *    OBS_LINE_CONFIRM_CYCLES periods in a row) is the line;
 *  - the robot keeps heading in until it sees it, then drives on until
 *    the axle is over it (OBS_IR_TO_AXLE_MM);
 *  - if it has not seen the line OBS_RETURN_SEARCH_MM beyond where the
 *    odometry line should be, the seek legs take over;
 *  - straightening up stops as soon as the leading sensor has crossed
 *    the line and both are off it (the line between them), and if neither
 *    crossed by the straight-ahead heading it keeps turning up to
 *    OBS_ALIGN_MORE_DEG further for it. Line following then takes over.
 * 0: odometry-only return. */
#ifndef OBS_RETURN_TO_BLACK
#define OBS_RETURN_TO_BLACK 1
#endif
#define OBS_RETURN_SEARCH_MM 400
#define OBS_ALIGN_MORE_DEG 40
#define OBS_LINE_CONFIRM_CYCLES 2U
#define OBS_VERIFY_CYCLES 10U
/* Resume when the way clears. After the first scan of an obstacle (and
 * after each "both sides blocked" rescan), before turning out, the robot
 * pings straight ahead again: OBS_RESUME_HITS valid echoes in a row
 * beyond OBS_RESUME_CLEAR_MM mean the object was taken away during the
 * scan, so it goes back to line following instead of bypassing nothing.
 * One echo at or inside it, or OBS_RESUME_PINGS pings without the clear
 * run, keeps the plan. Missing echoes never count as clear (see
 * OBS_FAILSAFE_NO_ECHO). Hits 1..3 (2-bit counter), pings hits..7 (3-bit
 * counter). OBS_RESUME_CLEAR_MM 0 turns it off; above OBS_STOP_MM so the
 * front monitor does not stop again at once. */
#define OBS_RESUME_CLEAR_MM 400
#define OBS_RESUME_HITS 3U
#define OBS_RESUME_PINGS 6U
/* Turn-out peek. A flat face echoes about +-25 degrees whatever its offset,
 * so a stopped sonar cannot tell where the box sits. At OBS_PEEK_DEG of the
 * turn out the sensor is about 100 mm out on the chosen side; the robot
 * stops, aims the sonar square at the face, skips OBS_PEEK_SKIP pings and
 * takes OBS_PEEK_PINGS. Three or more echoes within OBS_PEEK_SLACK_MM of
 * the face distance mean the box extends past the sensor, so (first choice
 * only, other corridor clear, face at least OBS_PEEK_MIN_NEAR_MM away) it
 * turns out the other way instead. About 1 s. DEG: multiple of 5, 30..60;
 * PINGS 3..5 (3-bit count with the skipped ones).
 * Off (0): the beam still reaches a box ending ~70 mm short of the sensor
 * at the pivot's reach, so the peek gives false results on this robot. */
#define OBS_PEEK 0
#define OBS_PEEK_DEG 50
#define OBS_PEEK_PINGS 5U
#define OBS_PEEK_SKIP 2U
#define OBS_PEEK_SLACK_MM 60
#define OBS_PEEK_MIN_NEAR_MM 160
/* Scans in a row with both sides blocked before it is a fault (about 7 s
 * each with the settle): the robot waits, stopped, for a side to clear. */
#define OBS_NO_ROUTE_RESCANS 10U
/* Odometry this close to the line after straightening up counts as on it:
 * the sensors straddle 18 mm tape only 4 mm clear each side, so crossing it
 * with both sensors is normal, and seeking from there makes the rejoin
 * worse. Line following finds the tape. */
#define OBS_ALIGN_ON_LINE_MM 30
#define OBS_SEEK_DEG 30
#define OBS_SEEK_BAND_MM 150
#define OBS_SEEK_LEGS 4U
/* Servo steps used while pointing at the obstacle during the turn out. */
#define OBS_AIM_STEP_DEG 5

#endif /* OBS_CONFIG_H */

/*** end of file ***/

