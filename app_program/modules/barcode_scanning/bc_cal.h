/** @file bc_cal.h
 * @brief Hardware calibration gate; never guess physical scan thresholds.
 */
#ifndef BC_CAL_H
#define BC_CAL_H

/* 1 boots only the motor-off encoder logger; normal robot tasks are skipped.
 * Set to 0 and rebuild to restore normal startup. Requires USB CDC on Pico.
 */
#ifndef BC_CAL_ENCODER_ONLY
#define BC_CAL_ENCODER_ONLY 0
#endif

/* 0 disables barcode work. With 1, valid settings enable decoding; invalid
 * settings select measurement logging instead of aborting robot startup.
 */
#define BC_CAL_READY 1

/* Force measurement logging even after valid scan settings are entered. */
#define BC_CAL_MEASURE 0

/* Standalone console consumer releases results after reporting them.
 * Set to 0 when a mission task owns bc_port_peek()/bc_port_ack().
 */
#define BC_CAL_AUTO_ACK 0 /* MQTT bridge owns ACK on successful publish. */

/* Temporary testing: accept samples despite capture, timing or encoder
 * validity failures. Diagnostics still count failures. Set to 0 to restore
 * strict input rejection. Width relaxation is controlled separately below.
 */
#ifndef BC_CAL_TEST_IGNORE_INPUT_ERRORS
#define BC_CAL_TEST_IGNORE_INPUT_ERRORS 1
#endif

/* Broad drive-test capture limits. Set to 0 for the earlier capture limits.
 * Both profiles require separate bar/space classification and exact patterns.
 */
#ifndef BC_CAL_TEST_RELAXED_CAPTURE
#define BC_CAL_TEST_RELAXED_CAPTURE 1
#endif

#if BC_CAL_READY
/* Units = sum of normalized wheel encoder increments, NOT millimetres.
 * Print sizes vary; 141 mm is an earlier sample, not a fixed scan size.
 * Corrected printed-span range: 154.6-297 mm (15.46-29.7 cm).
 * Source artwork has multiple sizes; see CALIBRATION_GUIDE.md.
 * Expected bar span = measured print span (mm) * sum encoder units/mm.
 * Nominal narrow = span / 47; wide = 3 * narrow for supplied A/F/Z.
 * Initial drive-test limits below use 12.34 encoder-sum units/mm.
 * Tolerances are engineering starting values, not measured sensor limits.
 */
#if BC_CAL_TEST_RELAXED_CAPTURE
/* Initial trial limits, to be validated with repeat physical passes. */
#define BC_CAL_MIN_RUN_UNITS 3U
#define BC_CAL_LEADING_UNITS 60U
#define BC_CAL_TRAILING_UNITS 60U
#define BC_CAL_MIN_BARCODE_UNITS 1300U
#define BC_CAL_MAX_BARCODE_UNITS 4800U
#define BC_CAL_TIMEOUT_MSEC 30000U
#else
#define BC_CAL_MIN_RUN_UNITS 20U
/* Available white travel: 50 mm each side. Convert using measured units/mm
 * and choose validated thresholds within that travel; these are not mm.
 */
/* 30 mm quiet requirement within the reported 50 mm available each side.
 * Barcode span limits allow approximately 10% around 154.6-297 mm.
 */
#define BC_CAL_LEADING_UNITS 370U
#define BC_CAL_TRAILING_UNITS 370U
#define BC_CAL_MIN_BARCODE_UNITS 1717U
#define BC_CAL_MAX_BARCODE_UNITS 4032U
#define BC_CAL_TIMEOUT_MSEC 15000U
#endif
/* Forward hand-roll logs: left decreases, right increases on all 3 passes.
 * Working sum scale from session 2: 3702 / 300 mm = 12.34 units/mm.
 * Each wheel totals 1851 counts / 300 mm = 6.17 counts/mm.
 * Assumes each pass covered exactly 100 mm; verify drive-test bounds.
 */
#define BC_CAL_LEFT_FORWARD_SIGN -1
#define BC_CAL_RIGHT_FORWARD_SIGN 1
/* Initial skew/jitter tolerances. At most 10 units travel per sample is
 * one quarter of the smallest nominal narrow feature (about 40.59 units).
 */
#define BC_CAL_MAX_STEP_DIFFERENCE 6U
#define BC_CAL_MAX_SAMPLE_UNITS 10U
#define BC_CAL_MAX_SAMPLE_MSEC 20U
/* TCRT5000, sensing face approximately 10 mm above paper (user report).
 * User measured black=1, white=0 on GP27 (2026-10-29).
 * Height is physical setup information, not an automatic correction.
 */
#define BC_CAL_BLACK_LEVEL 1U
#endif

/* Larger task priority numbers have lower precedence in this project. */
#define BC_CAL_DIAG_PRIORITY 9
#define BC_CAL_DIAG_PERIOD_MSEC 5U

/* The actual 2 ms alarm period comes from bc_fastq.h. */

#endif /* BC_CAL_H */

/*** end of file ***/

