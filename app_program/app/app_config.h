/** @file app_config.h
 * @brief Preserve the tested scheduling and startup configuration.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_TASK_STACK_BYTES 4096
#define APP_IR_PRIORITY 5
#define APP_CONTROLLER_PRIORITY 6
#define APP_IMU_PRIORITY 7
#define APP_STARTUP_DELAY_MSEC 2000
#define APP_CONTROLLER_PERIOD_MSEC 50U
#define APP_IR_MAX_AGE_MSEC 200U
/* Faster than the controller, so each decision sees a recent reading and a
 * brief touch of the line edge is caught for the controller's latches.
 */
#define APP_IR_PERIOD_MSEC 10
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
#define APP_BARCODE_SCAN_RPM 10.0f
#define APP_BARCODE_WAVE_MIN_VOTES 3U
#define APP_BARCODE_WAVE_MAX_FIT_PERCENT 30U
/* Experimental analog templates, all A-Z and both orientations. Scores
 * use white-noise units; the margin is a heuristic, not a probability.
 */
#ifndef APP_BARCODE_TEMPLATE_ENABLED
#define APP_BARCODE_TEMPLATE_ENABLED 1
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
#define APP_BARCODE_WAVE_DUMP_ENABLED 1
#endif
/* One buffered diagnostic pass per boot; 64 KiB of static sample storage.
 * Prints after 1.2 s without a raw colour change, or at capacity.
 * Does not alter thresholds, filtering, movement or decoding.
 */
#ifndef APP_BARCODE_TRACE_ENABLED
#define APP_BARCODE_TRACE_ENABLED 0
#endif
/* At a junction: readings ignored while coasting, then how long a line must
 * be seen after being placed back before following resumes. A lift during
 * the hold is not seen, so the hold is kept just long enough to stop.
 */
#define APP_JUNCTION_HOLD_MSEC 250U
#define APP_JUNCTION_SETTLE_MSEC 150U
/* One stops at a junction until placed back on a line; zero drives straight
 * across it instead. WARNING: with zero, lifting the robot (both sensors
 * read black) no longer stops the motors.
 */
#define APP_JUNCTION_STOP_ENABLED 0
#define APP_USB_WAIT_ATTEMPTS 200
#define APP_USB_WAIT_PERIOD_MSEC 100
#define APP_USB_SETTLE_MSEC 500
/* Distance between the two wheels' floor contacts. A hump under one wheel
 * lifts it this far sideways from the other, so height = this x sin(roll).
 */
#define WHEEL_SPACING_MM 110
/* Enabled IMU calibrates at startup; the controller waits before moving. */
#define APP_IMU_ENABLED 1
/* Periodic IMU sample lines, for tuning only. Printing blocks interrupts,
 * and with both cores printing the encoders lose pulses, so wheel speed
 * reads wrong. Hump start, descent and end are always printed.
 */
#define APP_IMU_DEBUG_LOG 0

/* Zero drives open loop, still logging RPM; use it to calibrate the target. */
#define APP_SPEED_CONTROL_ENABLED 1

/* Stationary barcode ADC calibration. Set to 1 to restore motor movement. */
#define APP_MOTOR_MOVEMENT_ENABLED 1

/* One CSV line per control period, for tuning: paste into a spreadsheet. */
#define APP_MOTION_DEBUG_CSV 0

#endif /* APP_CONFIG_H */

/*** end of file ***/

