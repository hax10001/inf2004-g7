/** @file app_config.h
 * @brief Preserve the tested scheduling and startup configuration.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_TASK_STACK_BYTES 4096
/* Barcode data-collection build (G7_BARCODE_TEST=1): no motor output and
 * no obstacle task, so the robot is pushed by hand; every capture dumps
 * its raw waveform (with the left wheel's travel) and re-arms itself.
 */
#ifdef B1_BARCODE_TEST
#define APP_OBSTACLE_ENABLED 0
#define APP_MOTOR_MOVEMENT_ENABLED 0
#define APP_BARCODE_WAVE_DUMP_ENABLED 1
#endif
#define APP_IR_PRIORITY 5
#define APP_CONTROLLER_PRIORITY 6
#define APP_IMU_PRIORITY 7
#ifndef APP_OBSTACLE_ENABLED
#define APP_OBSTACLE_ENABLED 1
#endif
#define APP_OBSTACLE_PRIORITY 6
#define APP_STARTUP_DELAY_MSEC 2000
/* Live control: the controller waits at most this long each period for the
 * STOP/START mailbox, which the telemetry task holds briefly. If it still
 * cannot read it this many periods in a row, it stops as a fail-safe. One
 * busy period only delays a STOP by one period (50 ms). */
#define APP_REMOTE_LOCK_WAIT_MSEC 2
#define APP_REMOTE_LOCK_MISS_LIMIT 4
#define APP_CONTROLLER_PERIOD_MSEC 50U
#define APP_IR_MAX_AGE_MSEC 200U
/* The IR task prints "[IR] Task stalled N ms" when its 1 ms loop goes this
 * long without running: tells CPU starvation apart from other stale causes. */
#define APP_IR_STALL_REPORT_MSEC 100U
/* Faster than the controller, so each decision sees a recent reading and a
 * brief touch of the line edge is caught for the controller's latches.
 */
#define APP_IR_PERIOD_MSEC 1
/* Barcode AO -> GP27/ADC1, 0..3.3 V only. Left/right remain digital.
 * ADC codes are 0..4095. Tune using [BC ADC] white/black readings.
 * Set ANALOG_ENABLED to 0 to return to the sensor's DO wiring.
 */
#ifndef APP_BARCODE_ANALOG_ENABLED
#define APP_BARCODE_ANALOG_ENABLED 1
#endif
/* Recorded F trace: 1200 merged white gaps; 2200 recovered *F* offline.
 * Trial boundaries 2150/2250; verify across print sizes and orientations.
 */
#define APP_BARCODE_ADC_THRESHOLD 2200U
#define APP_BARCODE_ADC_HYSTERESIS 50U
#define APP_BARCODE_ADC_BLACK_HIGH 1
/* Reject isolated colour changes using one-sample lookahead. Accepted
 * edges keep the first sample's timestamp and encoder counts.
 * Set to 0 to compare against the unfiltered analog input.
 */
#define APP_BARCODE_EDGE_FILTER_ENABLED 1
/* Experimental waveform decoder. Legacy decoder remains the comparison. */
#ifndef APP_BARCODE_BUFFERED_ENABLED
#define APP_BARCODE_BUFFERED_ENABLED 1
#endif
/* White reference for the barcode sensor:
 *   0 = re-learn white after every barcode (recommended). The logged track
 *       is not one white: learned white varied ~180..370 ADC within a run,
 *       and a fixed reference made floor noise count as bars ("many_bars"
 *       on every pass, 18 Nov runs).
 *   1 = learn white once, at the first arming after boot, and reuse it
 *       only while a rearm's fresh reading is within the startup sigma + 8
 *       of it (well inside the 3 sigma + 8 capture trigger); otherwise that
 *       pass uses its own reading. "Recalibrate barcode" replaces it.
 * Rearming always waits for ~0.5 s of white after each barcode, so a
 * barcode is never read twice. */
#ifndef APP_BARCODE_KEEP_STARTUP_WHITE
#define APP_BARCODE_KEEP_STARTUP_WHITE 0
#endif
/* Start a capture only when the barcode sensor reads at least this many
 * ADC counts darker than the learned white (and the 3 sigma + 8 noise
 * margin). Logged runs: every decoded barcode rose >= 1364 above white,
 * while 184 of 338 failed captures (shadows, floor patches) never rose
 * more than 600. Samples before the trigger are kept (512-sample
 * pretrigger), so the first bar's leading edge is not lost. 0 = previous
 * behaviour (trigger at 3 sigma + 8). */
#ifndef APP_BARCODE_TRIGGER_RISE
#define APP_BARCODE_TRIGGER_RISE 600U
#endif
/* Drop a capture early, without decoding, when its dark part ends (white
 * for the end-of-barcode gap) before spanning this many encoder units: a
 * tape, line or mark crossing, not a barcode. The robot stays armed and
 * returns to cruise speed at once. Logged: the shortest decoded barcode
 * was 1374 units; 1000 drops 234 of 338 failed captures. 0 = off. */
#ifndef APP_BARCODE_MIN_DARK_UNITS
#define APP_BARCODE_MIN_DARK_UNITS 1000U
#endif
/* 1 = no new capture while navigation centres on, pivots at or reacquires
 * the line after a junction: the barcode sensor sweeps across the track
 * lines then. 0 = capture there too (previous behaviour). */
#ifndef APP_BARCODE_SUPPRESS_IN_TURNS
#define APP_BARCODE_SUPPRESS_IN_TURNS 1
#endif
/* Barcodes are read at the driving speed (MOTION_TARGET_RPM in
 * modules/motion/motion.c): a capture does not slow or stop the robot.
 * The decoder was tuned at 15 RPM; reading uses encoder distance, so the
 * speed only matters through vibration and sensor height. */
#define APP_BARCODE_WAVE_MIN_VOTES 3U
#define APP_BARCODE_WAVE_MAX_FIT_PERCENT 30U
/* Wide-bar layout + blurred-model decoder (bc_layout.c) decides. Tuned on
 * saved passes: replay tools/barcode_data with tools/barcode_replay.py.
 * The older segmentation and template decoders then do not run: together
 * they took ~5 s per pass and read small prints poorly.
 */
#ifndef APP_BARCODE_LAYOUT_ENABLED
#define APP_BARCODE_LAYOUT_ENABLED 1
#endif
/* Experimental analog templates, all A-Z and both orientations. Scores
 * use white-noise units; the margin is a heuristic, not a probability.
 */
#ifndef APP_BARCODE_TEMPLATE_ENABLED
#if APP_BARCODE_LAYOUT_ENABLED
#define APP_BARCODE_TEMPLATE_ENABLED 0
#else
#define APP_BARCODE_TEMPLATE_ENABLED 1
#endif
#endif
#define APP_BARCODE_TEMPLATE_MIN_MARGIN_Z100 300
#define APP_BARCODE_TEMPLATE_MIN_SCORE_Z100 450
#define APP_BARCODE_TEMPLATE_MIN_SUPPORT 2U
/* Report rejected template candidates on serial only. Estimates never
 * enter the confirmed-character result queue or bypass capture checks.
 */
#ifndef APP_BARCODE_ESTIMATES_ENABLED
#define APP_BARCODE_ESTIMATES_ENABLED 1
#endif
/* Dump the frozen raw and processed waveform AFTER the result, with motors
 * held. Keep serial connected through [BC WAVE RAW END].
 */
#ifndef APP_BARCODE_WAVE_DUMP_ENABLED
#ifdef B1_LIVE_COMMANDS
/* USB raw waveform dumps are bench diagnostics, not live operation.
 * They can block a captured scan for tens of seconds and spam CDC. */
#define APP_BARCODE_WAVE_DUMP_ENABLED 0
#else
#define APP_BARCODE_WAVE_DUMP_ENABLED 1
#endif
#endif
/* One buffered diagnostic pass per boot; 64 KiB of static sample storage.
 * Prints after 1.2 s without a raw colour change, or at capacity.
 * Does not alter thresholds, filtering, movement or decoding.
 */
#ifndef APP_BARCODE_TRACE_ENABLED
#define APP_BARCODE_TRACE_ENABLED 0
#endif
/* Preserve active movement from the incoming drive-test profile. */
#ifndef APP_MOTOR_MOVEMENT_ENABLED
#define APP_MOTOR_MOVEMENT_ENABLED 1
#endif

/* At a junction: readings ignored while coasting, then how long a line must
 * be seen after being placed back before following resumes. A lift during
 * the hold is not seen, so the hold is kept just long enough to stop.
 */
#define APP_JUNCTION_HOLD_MSEC 250U
#define APP_JUNCTION_SETTLE_MSEC 150U
/* Legacy controller: stop for an ambiguous both-black junction.
 * The live-navigation controller uses confirmed readings and a pending
 * mapped barcode, and stops safely on an unmapped/ambiguous crossing.
 */
#define APP_JUNCTION_STOP_ENABLED 0
/* AUTONOMOUS BENCH DEFAULTS — NOT floor-calibrated turning geometry.
 * Two digital IR sensors cannot distinguish all lifted/black/junction
 * cases; require confirmed black plus a pending mapped barcode action. */
#define APP_NAV_JUNCTION_CONFIRM_MSEC 150U
#define APP_NAV_MIN_TURN_MSEC 250U
/* Time bound on a pivot; a U-turn takes ~2.6 s at MOTION_PIVOT_COUNTS. */
#define APP_NAV_MAX_TURN_MSEC 6000U
#define APP_NAV_MAX_CROSS_MSEC 1800U
#define APP_NAV_BARCODE_TO_JUNCTION_MIN_COUNTS 120U
/* Junction = both line sensors black over this much forward travel, with
 * no white gap seen by the IR sampler. The crossing bar is tape: logged
 * crossings were both-black for only ~15 mm (~73 counts), so this must
 * stay below that. Require 7 mm so a brief both-black reading on a curve
 * is less likely to confirm at 35 RPM. Set to 5 for the previous threshold.
 * Counts: 1056 per rev over a 219.8 mm circumference (70 mm wheel), about
 * 4.8 counts per mm; 7 mm = 33 counts. */
#ifndef APP_NAV_JUNCTION_CONFIRM_MM
#define APP_NAV_JUNCTION_CONFIRM_MM 7U
#endif
#define APP_NAV_JUNCTION_CONFIRM_COUNTS \
    ((APP_NAV_JUNCTION_CONFIRM_MM * 1056U * 10U) / 2198U)
/* If the robot barely moves (stall, lifted), this long continuously
 * both-black also counts; ~20 mm takes ~360 ms at 15 RPM. */
#define APP_NAV_JUNCTION_CONFIRM_MAX_MSEC 1500U
/* IR samples that must be not-both-black before the run counts as broken.
 * The IR task logs ~3 samples per 50 ms (~16 ms apart, not 1 ms), so 2
 * samples is ~30 ms, ~2 mm: one stray sample is ignored as flicker. */
#define APP_IR_BOTH_GAP_SAMPLES 2U
/* Print a [JCT] line per 50 ms period in which either line sensor saw
 * black: sample counts, gap flag, run length and navigation decision.
 * Tuning only: ~20 lines/s of USB output, and heavy console output is
 * when the SMP port's spurious E_CTX on the IR mutex has been seen. */
#define APP_JUNCTION_DEBUG_LOG 0
/* Short-brake the motors (both H-bridge inputs high) for this long when
 * navigation stops at a junction, then release to coast. 0 = coast only. */
#define APP_JUNCTION_BRAKE_MSEC 200U
/* Junction reached while the barcode before it is still decoding (decodes
 * take ~2-3 s): stop and wait up to this long for the letter, then do its
 * mapped action. No letter (failed decode) goes straight on
 * (APP_NAV_STRAIGHT_DEFAULT) or repeats the remembered action. */
/* JUNCTION ACTION MODE
 *   0 = single action: a barcode's action is done at the next junction
 *       only, then discarded; a later junction with no new barcode goes
 *       straight (APP_NAV_STRAIGHT_DEFAULT) or is a safety stop. An action
 *       not yet used survives an obstacle bypass; STOP clears it.
 *   1 = repeat action: the action is done at EVERY junction until another
 *       barcode replaces it. It survives an obstacle bypass; STOP (or MQTT
 *       loss) clears it, so each START begins with no action queued.
 * In both modes a junction reached while a barcode is still decoding waits
 * for that letter; in repeat mode a failed decode falls back to the
 * remembered action instead of a safety stop. */
#define APP_NAV_REPEAT_ACTION 0U
/* Junction with no usable action (no barcode read since START, or a decode
 * that gave no letter with nothing remembered):
 *   1 = drive straight across it and carry on line following.
 *   0 = navigation safety stop (the previous behaviour).
 * A junction confirmed too soon after a queued barcode
 * (APP_NAV_BARCODE_TO_JUNCTION_MIN_COUNTS) is set by
 * APP_NAV_SCAN_FAIL_STOP below: crossed straight (0) or a stop (1). */
#define APP_NAV_STRAIGHT_DEFAULT 1U
/* Barcode problems while running:
 *   - a letter that is not in the route mapping (or no mapping loaded),
 *   - a result lost because an unread one was overwritten,
 *   - a barcode read too close to the junction it would apply to
 *     (APP_NAV_BARCODE_TO_JUNCTION_MIN_COUNTS),
 *   - the barcode handoff to navigation failing to start.
 *   0 = never stop: ignore the barcode (cross that junction straight) and
 *       carry on line following; a failed handoff is retried every period.
 *   1 = navigation safety stop (the previous behaviour): STOP, then START.
 * A barcode that could not be decoded never stops the robot. */
#ifndef APP_NAV_SCAN_FAIL_STOP
#define APP_NAV_SCAN_FAIL_STOP 0
#endif
#define APP_NAV_DECODE_WAIT_MSEC 8000U
#define APP_NAV_DECODE_GRACE_MSEC 1500U
/* Junction turns (left, right, U-turn) pivot on the spot with the wheel
 * axle over the junction centre, so the line sensors, which sit ahead of
 * the axle, swing round onto the new arm. MEASURE: axle to line sensors
 * (same as OBS_IR_TO_AXLE_MM). Centre = that plus half the 18 mm tape,
 * measured from where the crossing bar was first seen. If turns end with
 * the sensors short of the new line, raise it; past it, lower it. */
#define APP_NAV_IR_TO_AXLE_MM 125U
#define APP_NAV_CENTRE_MM (APP_NAV_IR_TO_AXLE_MM + 9U)
#define APP_NAV_CENTRE_COUNTS ((APP_NAV_CENTRE_MM * 1056U * 10U) / 2198U)
/* ~2.4 s at 15 RPM; slower than that means something is wrong. */
#define APP_NAV_CENTRE_MAX_MSEC 5000U
/* 90 degree pivot: each wheel rolls a quarter circle of the track's
 * radius, so both together roll (pi/2) x track. Only gates when the line
 * may end the turn, so some slip is tolerated. EFFECTIVE track, not the
 * 110 mm measured one: pivoting scrubs the wheels, and a logged U-turn
 * took 2107 counts for ~185-200 degrees by the compass (~20 % more than
 * 110 mm predicts). Calibrate from "[NAV] Turn ... counts" log lines:
 * track = 130 x (counts for 180 degrees / 1960). */
#define APP_NAV_TRACK_MM 130U
#define APP_NAV_TURN90_COUNTS \
    ((((APP_NAV_TRACK_MM * 1571U) / 1000U) * 1056U * 10U) / 2198U)
#define APP_USB_WAIT_ATTEMPTS 200
#define APP_USB_WAIT_PERIOD_MSEC 100
#define APP_USB_SETTLE_MSEC 500
/* Distance between the two wheels' floor contacts. A hump under one wheel
 * lifts it this far sideways from the other, so height = this x sin(roll).
 */
#define WHEEL_SPACING_MM 110
/* Calibrate the IMU while the robot is stationary and level at boot. */
#ifdef B1_BARCODE_TEST
#define APP_IMU_ENABLED 0 /* Motor-free barcode capture profile. */
#else
#define APP_IMU_ENABLED 1 /* Normal MQTT/live robot profile. */
#endif
/* LIFT STOP: halt the motors when the robot is picked up.
 *   APP_LIFT_STOP_ENABLED   1 = stop when lifted, 0 = only report height.
 *   APP_LIFT_STOP_HEIGHT_MM height off the floor, in mm (1..1022), at
 *                           which the robot counts as lifted. CHANGE THIS
 *                           to make the stop more or less sensitive.
 * There is no downward range sensor: the height is ESTIMATED from the
 * accelerometer (see modules/terrain/lift_math.h) and shown on the
 * dashboard. Fresh white IR readings suppress lift estimation. Two black
 * readings can mean a junction or air. A slow pickup can evade the IMU.
 * What happens after the stop is set by APP_LIFT_RESUME_ENABLED below. */
#ifndef APP_LIFT_STOP_ENABLED
#define APP_LIFT_STOP_ENABLED 1
#endif
#ifndef APP_LIFT_STOP_HEIGHT_MM
#define APP_LIFT_STOP_HEIGHT_MM 40
#endif
/* After a lift stop:
 *   APP_LIFT_RESUME_ENABLED 1 = carry on line following once the robot is
 *                           put back down; 0 = stay stopped until STOP,
 *                           then START, like the other safety stops.
 *   APP_LIFT_RESUME_MSEC    how long it must stand on the floor (a line
 *                           sensor seeing white) before it moves again.
 * STOP or MQTT loss ends the run. To resume, place the robot where
 * at least one line sensor sees white. */
#ifndef APP_LIFT_RESUME_ENABLED
#define APP_LIFT_RESUME_ENABLED 1
#endif
#ifndef APP_LIFT_RESUME_MSEC
#define APP_LIFT_RESUME_MSEC 1000U
#endif
/* Periodic IMU sample lines, for tuning only. Printing blocks interrupts,
 * and with both cores printing the encoders lose pulses, so wheel speed
 * reads wrong. Hump start, descent and end are always printed.
 */
#define APP_IMU_DEBUG_LOG 0

/* V13 downhill governor: 0 means the Pico pitch sign is NOT calibrated.
 * For a verified nose-down ramp, set +1 when measured pitch is positive,
 * -1 when negative. Do not guess the sign from a drawing.
 * Only enable after stationary pitch checks on the real mounted sensor.
 */
#ifndef APP_DOWNHILL_NOSE_DOWN_SIGN
#define APP_DOWNHILL_NOSE_DOWN_SIGN 0
#endif
#if ((APP_DOWNHILL_NOSE_DOWN_SIGN != -1) && \
     (APP_DOWNHILL_NOSE_DOWN_SIGN != 0) && \
     (APP_DOWNHILL_NOSE_DOWN_SIGN != 1))
#error APP_DOWNHILL_NOSE_DOWN_SIGN must be -1, 0, or 1
#endif
/* Preliminary 65% target. Tune only with supported wheels / ramp trials. */
#define APP_DOWNHILL_REDUCED_PERCENT 65U
/* Initial runaway guard, NOT a calibrated descent speed. Normal RPM
 * feedback errors reduce duty through PI without causing STOP/ON pulsing.
 * Only sustained gross overspeed raises a latched safety STOP. */
#define APP_DOWNHILL_RUNAWAY_RPM 40.0f

/* Zero drives open loop, still logging RPM; use it to calibrate the target. */
#define APP_SPEED_CONTROL_ENABLED 1

/* One CSV line per control period, for tuning: paste into a spreadsheet. */
#define APP_MOTION_DEBUG_CSV 0

#endif /* APP_CONFIG_H */

/*** end of file ***/

