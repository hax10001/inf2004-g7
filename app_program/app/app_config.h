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
/* The proven V7.7 scan-speed baseline is 15 RPM. Change only after
 * measuring print width, encoder counts and small-barcode success rates. */
#define APP_BARCODE_SCAN_RPM 15.0f
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
#define APP_JUNCTION_STOP_ENABLED 1
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
 * stay well below that. 5 mm needs at least two consecutive both-black
 * controller reads (~3 mm each at 15 RPM): a single diagonal brush on a
 * curve does not count. Raise only if curves still stop the robot.
 * Counts: 1056 per rev over a 219.8 mm circumference (70 mm wheel), about
 * 4.8 counts per mm; 5 mm = 24 counts. */
#define APP_NAV_JUNCTION_CONFIRM_MM 5U
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
 * mapped action. No letter (failed decode) still ends in a safety stop. */
/* JUNCTION ACTION MODE
 *   0 = single action: a barcode's action is done at the next junction
 *       only; a later junction with no new barcode is a safety stop.
 *   1 = repeat action: the action is done at EVERY junction until another
 *       barcode replaces it. It survives an obstacle bypass; STOP (or MQTT
 *       loss) clears it, so each START begins with no action queued.
 * In both modes a junction reached while a barcode is still decoding waits
 * for that letter; in repeat mode a failed decode falls back to the
 * remembered action instead of a safety stop. */
#define APP_NAV_REPEAT_ACTION 1U
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
/* V12 bench profile: IMU and existing collision detection enabled.
 * Calibrates only while the robot is stationary/level at boot. Missing or
 * invalid IMU must block motion rather than cause an unverified fallback.
 * The 5000 mg/s jerk threshold is NOT physically validated; keep wheels
 * elevated with physical motor power cutoff for first tests. */
#ifdef B1_BARCODE_TEST
#define APP_IMU_ENABLED 0 /* Motor-free barcode capture profile. */
#else
#define APP_IMU_ENABLED 1 /* Normal MQTT/live robot profile. */
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

