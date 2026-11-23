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
/* 21 Nov: 1500 aimed the sonar a few degrees right of the robot's
 * forward, so boxes read left of where they were. The face normal ("Square
 * to the face") read left of sonar zero in 42 of 48 logged scans over three
 * sessions (4 at 0, 2 at -2.5). With no line heading known (robot put down
 * near the box) the sides are compared from sonar zero, and in session
 * 11:27 a box moved right still went right (4 of 4). There the echo span
 * centre was +5..+12.5 deg for boxes left of the line and +2.5..+5 for
 * boxes right of it, so the split sits near +4. Replaying those 14 scans
 * through this planner (box left / box right going the correct way, two
 * right-box runs forced right by a blocker): 0 deg 8/8, 0/2; 5 deg 6/8,
 * 2/2; 10 deg 4/8, 2/2. 40 usec turns the zero 4 degrees left. Check: a
 * box centred on the line should log "Square to the face" near 0. */
#define OBS_SERVO_FRONT_USEC 1540
#define OBS_SERVO_USEC_PER_DEG 10
/* A value of 1 means that a longer pulse turns the sensor left. Keep 1:
 * with -1 every bearing is mirrored, so the scan reads the box on the wrong
 * side and the side watch looks away from the box while passing (logged
 * 19:2x, -1: no echo from the box on either pass, only the open side 0.6-0.9
 * m out; every logged pass that saw the box ran with 1). */
#define OBS_SERVO_LEFT_SIGN 1
/* SG90 pulse range; angles needing a pulse outside it are not used. */
#define OBS_SERVO_MIN_USEC 500
#define OBS_SERVO_MAX_USEC 2400
/* Measure: how far the servo can turn each side before it is blocked. The
 * side watch needs about 70; if the mount stops it sooner, set the real
 * reach here so the side-watch geometry stays right.
 */
/* 21 Nov: 60, not 70. Moving FRONT to 1540 usec put the 70 degree left
 * end at 2240 usec, 40 past the 2200 it had always used, and the servo was
 * seen straining to turn left. 60 is 2140 usec, inside the old range, and
 * a multiple of OBS_SCAN_STEP_DEG so the sweep still ends on the limit.
 * The side watch clamps to it (looks 60 degrees left, not 70). */
#define OBS_SERVO_LEFT_LIMIT_DEG 60
#define OBS_SERVO_RIGHT_LIMIT_DEG 70
#define OBS_SERVO_SETTLE_BASE_MSEC 40
#define OBS_SERVO_SETTLE_PER_DEG_MSEC 3
/* Scan moves only ever arrive moving right. This SG90 turns right
 * smoothly but struggles turning left and springs back a little when it
 * stops, which left the centre and the left bearings off (seen on the
 * robot, 21 Nov; a 5 or 10 degree hop back did not cure it). So a scan
 * move to the right goes straight there, ramped in OBS_SERVO_RAMP_DEG
 * steps OBS_SERVO_RAMP_MSEC apart; a move to the left first swings to the
 * left limit, waits OBS_SERVO_SWING_SETTLE_MSEC for the spring-back to
 * die, then comes right to the bearing. The scan is one left-to-right
 * sweep (OBS_SCAN_DIRECTION -1) and the edge vote runs the same way, so a
 * scan swings left three times: to start the sweep, to start the vote and
 * to park at centre. The side watch while passing keeps its direct
 * moves. */
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
/* Sweep direction. A servo settles a little short or past each target, the
 * same way every time it is approached from the same side, which shifts
 * every bearing of a one-way sweep and makes objects lean one way.
 *   0 = both ways: left to right first, then back right to left; each
 *       bearing keeps the nearer reading, so the two errors cancel and no
 *       echo is lost. The return pass covers +-OBS_SCAN_RETURN_DEG only.
 *   1 = right to left only; -1 = left to right only. Comparing the two in
 *       the log shows how much the servo shifts the bearings.
 */
/* 21 Nov: -1 (one smooth left-to-right sweep); see the servo note above. */
#ifndef OBS_SCAN_DIRECTION
#define OBS_SCAN_DIRECTION -1
#endif
/* Pings per bearing per pass: two, and a third only when those two differ
 * by more than OBS_SCAN_AGREE_MM (an edge that echoes only some of the
 * time, or a stray echo); the bearing then reads the median of three, as
 * every bearing used to. Two that agree read their average. (Keeping just
 * the larger of two dropped a real edge on 18 Nov: 230 mm / no echo.) */
#define OBS_SCAN_PINGS 2U
#define OBS_SCAN_AGREE_MM 50
/* Two-way scan only: the return pass re-reads only bearings within
 * +-OBS_SCAN_RETURN_DEG, where the object's edges are found and servo lag
 * matters; outer bearings keep the first pass's reading. The servo also
 * stops short of the end of the return sweep. Replayed on the logged
 * two-way scans, the plan was identical. Set to the sweep limit (70) for
 * a full return pass. */
#define OBS_SCAN_RETURN_DEG OBS_SCAN_FINE_DEG
/* Corner band: bearings beyond +-OBS_SCAN_FINE_DEG up to this.
 * The sensor faces the box square on, so its flat front face acts like a
 * mirror: every bearing within about 30 degrees returns the same
 * perpendicular distance, wherever the box sits, and the echo stops at
 * about 30 degrees each side whatever the box's width. Those "edges" are the
 * sensor's limit, not the box's, so a box half off the line still reads as
 * centred (logged 18 Nov: box on the left, edges -30..30, every time). Only
 * the box's vertical corners give a weak echo further out, and its range
 * gives where the corner really is: sqrt(range^2 - depth^2) sideways,
 * which does not depend on the beam or the bearing. So in this band each
 * bearing takes up to OBS_CORNER_PINGS pings until two agree on an echo,
 * instead of letting a median drop it. A multiple of OBS_SCAN_STEP_DEG,
 * OBS_SCAN_FINE_DEG..sweep limit. */
#define OBS_CORNER_MAX_DEG 50
#define OBS_CORNER_PINGS 4U
/* Corner band bearing step. 5: +-35, 40, 45, 50. A box 3-7 cm off the line
 * has its far corner 26-35 degrees out, at the edge of the square-on zone;
 * with 10 degree steps nothing sampled between 30 (where the face's own
 * echo arrives first and hides the corner) and 40. A multiple of 5 that
 * divides OBS_CORNER_MAX_DEG - OBS_SCAN_FINE_DEG. */
#define OBS_CORNER_STEP_DEG 5
/* A corner heard on only one of a bearing's pings is kept when the next
 * corner bearing (OBS_CORNER_STEP_DEG on) heard it too, within this many mm:
 * the same corner seen from both. Logged: 5 of 32 corner band visits that
 * heard something near the face heard it on a single ping (279 mm at +40,
 * twice), and the agreement rule dropped it. */
#define OBS_CORNER_NEIGHBOUR_MM 30
/* An echo in the corner band is the box's corner when it is no more than
 * this beyond the face depth (a 15 cm box up to ~25 cm off centre at 22 cm
 * ahead). Farther echoes are something else and are left to the corridor
 * check. */
#define OBS_CORNER_DEPTH_MM 150
/* A bearing hears a corner at most this many degrees off its axis (the
 * sensor's half beam), so a corner band echo nearer the line than
 * face * tan(bearing - this) is the face itself still echoing (a face not
 * square on, logged 18:59: 227 mm at 40 degrees, face 214 mm), not a
 * corner. Multiple of 5, below OBS_SCAN_FINE_DEG. */
#define OBS_CORNER_BEAM_DEG 15
/* The robot seldom stops square to the box: the logged face normal (the
 * centre of the square-on zone) wanders +-5 degrees from run to run, and
 * sat 3-10 degrees right through a whole session of box-on-left runs (18:47,
 * 11 of 12 went the wrong way). Where it leans, the face keeps echoing
 * further out, which looks like a corner on that side. So each scan
 * measures the normal and corner bearings are judged from it: a corner
 * band echo counts only when it is at least OBS_CORNER_FROM_DEG off the
 * normal (the square-on zone is 27-33 degrees each side of it in the
 * logs). The normal is the range-weighted centre of the bearings within
 * OBS_FACE_BOWL_MM of the nearest echo (off-axis echoes come back a little
 * late, so the profile is a bowl centred on it; matches a full fit to the
 * logged scans within 1.7 degrees rms), at most OBS_FACE_TILT_MAX_DEG
 * either way. */
/* Slow edge vote after the two passes. The box's edges echo only some of
 * the time, and a pass took two or three quick pings 55 ms after the
 * servo stepped, so the same box gave different edges run to run (21 Nov
 * 11:5x, box not moved, 3 runs: -30 deg read 4000/4000 on one pass and
 * 243/260 on the other; +30 heard once of three and dropped; spans
 * -30..25, -35..30, -30..30; widths 153/199/174 mm; sides R/L/R). So
 * each bearing OBS_EDGE_VOTE_MIN_DEG..MAX_DEG either side is visited once
 * more, from the centre outwards on both sides (servo overshoot is then
 * mirror-symmetric), settled OBS_EDGE_VOTE_SETTLE_MSEC longer, and pinged
 * OBS_EDGE_VOTE_PINGS times OBS_EDGE_VOTE_GAP_MSEC apart. A ping is "near"
 * when it echoes within OBS_CORNER_DEPTH_MM beyond the face. The bearing
 * is the object when most pings are near (their median), else open (the
 * median of the rest); a minority of near pings is kept as a lone corner
 * echo. About 0.6 s per bearing, ~8 s for 20..45 both sides; the scan
 * was ~8 s. 21 Nov 12:3x: the votes came back 7/7 or 0/7 on almost every
 * bearing, so the settle went 150 -> 100 ms and the gap 90 -> 70 ms.
 * 0 turns it off. PINGS: odd, 3..15. Bearings: multiples of 5.
 */
#define OBS_EDGE_VOTE 1
#define OBS_EDGE_VOTE_MIN_DEG 20
#define OBS_EDGE_VOTE_MAX_DEG 45
#define OBS_EDGE_VOTE_PINGS 5U
#define OBS_EDGE_VOTE_SETTLE_MSEC 60U
#define OBS_EDGE_VOTE_GAP_MSEC 70U
/* 1: a bearing that hears no echo at all, next to the object (beyond its
 * echoing face, in an unbroken run that a real echo ends, still within
 * OBS_CORNER_MAX_DEG), counts as the object's face seen at a slant, at the
 * edge's range. The flat face is a mirror: aimed past its square-on zone
 * the ping bounces off it and never returns, so "no echo" there means
 * the box is still in the beam; past its corner the background answers.
 * A silent run that nothing ends may be open space and stays open.
 * 21 Nov logs: right-offset boxes went silent at -25..-40 with the room
 * answering at -40 (1.5 m), the left heard the room at once; the planner
 * read the silence as open, the box looked centred, and the tie-break
 * ("more room past it": the open right of this room) chose. Replaying
 * the 10 labelled scans taken at the current trim: 6/10 -> 8/10 correct.
 * Still wrong: one symmetric scan (nothing to go on) and one left-offset
 * box whose silence ran into open room (no echo out to +60). 0: off. */
#ifndef OBS_GLANCE_AS_FACE
#define OBS_GLANCE_AS_FACE 1
#endif
/* Silent bearings in a row needed (one alone may be a dropped ping). */
#ifndef OBS_GLANCE_MIN_RUN
#define OBS_GLANCE_MIN_RUN 2
#endif
#define OBS_CORNER_FROM_DEG 35
#define OBS_FACE_BOWL_MM 10
/* 25: the robot was logged up to ~20 degrees off square (22:27); 10 was
 * too tight and let the face's own echo pass as a corner. */
#define OBS_FACE_TILT_MAX_DEG 25
/* 1: when a corner is found, choose the side from the object's real
 * extent: the seen corner(s), and for an unseen one the face's own edge if
 * it stopped echoing inside +-OBS_SCAN_FINE_DEG, else the far edge of an
 * OBS_OBJECT_MAX_MM wide object. The robot also drives out at least far
 * enough to clear a seen corner on its side. With no corner echo the
 * object's edges are both inside the sensor's blind zone (it is about
 * centred) and the edges decide as before. 0: off. */
#define OBS_SIDE_CORNER_ECHO 1
/* Creep closer before scanning (first stop only). The square-on zone is
 * +-30 degrees about the face's normal, which covers range * tan(30) of the
 * face each side: +-125 mm at the usual ~215 mm stop, so a 15 cm box less
 * than ~5 cm off the line keeps both corners inside it and reads as centred
 * (logged 21:58: every non-extreme placement found no corner). From 150 mm
 * it covers +-87 mm, so a box ~2 cm off already has its far corner outside
 * the zone where it can be heard, and the corners are nearer (stronger
 * echoes). Creeps at OBS_APPROACH_RPM on fresh front pings until the face
 * is OBS_SCAN_FROM_MM away, at most OBS_APPROACH_MAX_MM or
 * OBS_APPROACH_MAX_MSEC, and stops early after OBS_APPROACH_MISSES pings in
 * a row without the face (never drives blind). 0 turns it off. The 90
 * degree turn out still fits: pivot 55 + overhang 25 + clearance 30 mm. */
/* Off (0): on the robot (22:27) the closer start left less room to clear
 * the box than the version that cleared it well, and once lost the face
 * while creeping. The side choice no longer needs it (tilt-corrected edges
 * and open room decide). */
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
 * 100 counts all of it (the old behaviour, a shorter drive out).
 */
#define OBS_PIVOT_GAIN_PCT 0
/* Wide initial passes (either side) gained 44-74 mm in odometry while
 * turning parallel (logged 19 Nov: +73 mm after a 90 degree exit, +44 mm
 * after 60), on top of the planned offset, so the robot passed ~110 mm off
 * the box instead of the 40 mm margin. Count half of the predicted pivot
 * gain (about 27 mm at 90 degrees). Narrow passes and rescans use the
 * regular gain above. 0 restores the full drive-out. */
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
#define OBS_ALIGN_TOL_DDEG 100
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
 * tenths of a degree, and the most it may turn out. While the object is
 * beside the robot it never turns in towards it: encoder odometry can
 * think the robot is further out than it is (logged 18 Nov: 231 mm by the
 * encoders, about 160 mm in the video), and the old inward correction
 * then steered the front corner into the box.
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
 * (log, dashboard); bypass offsets still use the conservative edges. Fit
 * to the logged scans of the 15 cm obstacle: about 9-10 degrees. */
#define OBS_BEAM_HALF_DEG 10
/* The box counts as about centred when the two sides' moves (measured from
 * the face's normal, or from corner echoes) differ by no more than this; it
 * then goes to the side with more room past it (the nearest echo beyond
 * each edge, then the average). Logged 22:27: two centred boxes went right
 * on a few cm of difference while the left was open and the right had
 * something 0.75-0.95 m away. 0..255 mm. */
/* 19 Nov: narrowed from 45 to 15 mm (about the sonar's edge resolution):
 * a box 91 mm left / 71 mm right of the line went left on "more room"
 * though right was less work. Only truly centred boxes fall back to it. */
#define OBS_SIDE_PREFERENCE_THRESHOLD_MM 15
/* 1: compare the two sides using the beam-corrected edges (each edge
 * midway to the next empty bearing). The conservative edges, padded out to
 * the next empty bearing, depend on the bearing spacing (5 degrees within
 * +-30, 10 outside), which tilted close calls. The drive offset still uses
 * the conservative edges. 0: compare the conservative edges (previous). */
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
 * sees it and the robot runs parallel again. Measure: set it to the deepest
 * obstacle expected. 1..OBS_PASS_MAX_MM - 1 mm.
 *
 * OBS_PASS_MAX_MM: hard limit. Stop with a NO_ROUTE fault if the axle gets
 * this far past the front face while still passing, whatever the side watch
 * says. Must exceed OBS_PASS_DEFAULT_MM. 0..32767 mm.
 */
/* The default end is never shorter than the object's beam-corrected
 * scanned width, capped at OBS_OBJECT_MAX_MM: depth is not measured, so a
 * box is assumed about as deep as it is wide. A deeper one is caught by the
 * front guard on the way back. */
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
 */
#define OBS_RETURN_DEG 60
#define OBS_ALIGN_CHECK_DEG 30
/* 23 Nov: return to the line by the IR sensors, not by odometry. When the
 * robot stopped at an angle to the line (on a curve, mid-correction), the
 * line is not along the stop heading, so the odometry line above is wrong:
 * black away from it was ignored, the robot straightened up beside the
 * line and the seek legs zig-zagged away from it. With this on:
 *  - on the way back any black under a line sensor (seen
 *    OBS_LINE_CONFIRM_CYCLES periods in a row) is the line;
 *  - the robot keeps heading in until it sees it, then drives on until
 *    the axle is over it (OBS_IR_TO_AXLE_MM), as before;
 *  - if it has not seen the line OBS_RETURN_SEARCH_MM beyond where the
 *    odometry line should be, the seek legs take over as before;
 *  - straightening up stops as soon as the leading sensor has crossed
 *    the line and both are off it (the line between them), and if neither
 *    crossed by the straight-ahead heading it keeps turning up to
 *    OBS_ALIGN_MORE_DEG further for it. Line following then takes over.
 * 0: the previous odometry return. */
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
/* Turn-out peek. A stopped, rotating sonar cannot tell where a flat box
 * sits: its face is a mirror, so it echoes about +-25 degrees whatever its
 * offset (21 Nov, five sessions of labelled runs: box-left and box-right
 * scans read the same edges, and every scan rule tried chose by the room
 * or by noise). The sensor has to move sideways, and the turn out does
 * that: pivoting on one wheel, at OBS_PEEK_DEG of turn the sensor is about
 * 100 mm out on the chosen side. There the robot stops, aims the sonar
 * square at the face (servo at minus the heading) and pings: skip two,
 * then OBS_PEEK_PINGS; three or more echoes within OBS_PEEK_SLACK_MM of
 * the face's distance mean the box still sticks out past the sensor on
 * this side, so (first choice only, other corridor clear, face at least
 * OBS_PEEK_MIN_NEAR_MM away so the swing back clears it) it turns out the
 * other way instead, peeks there too for the log, and goes. No face: the
 * box ends within ~100 mm on this side; carry on. About 1 s; 0 turns it
 * off. DEG: multiple of 5, 30..60 (servo reach). PINGS 3..5 (3-bit count
 * with the two skipped). */
/* 21 Nov 14:3x, on the robot: OFF. At 111-116 mm out the sensor still
 * heard the face on 7 of 10 first peeks, either side (pings 187-191 mm
 * against 202 expected): the beam still reaches a box ending ~70 mm
 * short of it, and the pivot cannot carry the sensor further (123 mm at
 * the servo's reach). So it changed sides on no evidence, and after the
 * second pivot the odometry was off enough to miss the line on the way
 * back (seek legs, then a NO_ROUTE fault at x=1336). Kept for a sensor
 * that can be carried further out; do not enable as it is. */
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
 * the sensors straddle 18 mm tape only 4 mm clear each side, so crossing
 * it with both sensors on the way in is normal, and seeking from there
 * (stop, turn, drive across) made the rejoin worse (logged 19 Nov 13:05:
 * y = -11..17 mm, then seek legs). Line following finds the tape. */
#define OBS_ALIGN_ON_LINE_MM 30
#define OBS_SEEK_DEG 30
#define OBS_SEEK_BAND_MM 150
#define OBS_SEEK_LEGS 4U
/* Servo steps used while pointing at the obstacle during the turn out. */
#define OBS_AIM_STEP_DEG 5

#endif /* OBS_CONFIG_H */

/*** end of file ***/

