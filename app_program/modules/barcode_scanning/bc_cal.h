/** @file bc_cal.h
 * @brief Hardware calibration gate; never guess physical scan thresholds.
 */
#ifndef BC_CAL_H
#define BC_CAL_H

/* Set to 1 ONLY after filling the macros below with real measured values.
 * The delivered firmware keeps barcode capture disabled until calibrated.
 */
#define BC_CAL_READY 0

#if BC_CAL_READY
/* Units = sum of normalized wheel encoder increments, NOT millimetres.
 * Record a measured conversion and the approximately 141 mm sample width.
 */
#define BC_CAL_MIN_RUN_UNITS 0U
#define BC_CAL_LEADING_UNITS 0U
#define BC_CAL_TRAILING_UNITS 0U
#define BC_CAL_MIN_BARCODE_UNITS 0U
#define BC_CAL_MAX_BARCODE_UNITS 0U
#define BC_CAL_TIMEOUT_MSEC 0U
#define BC_CAL_LEFT_FORWARD_SIGN 1
#define BC_CAL_RIGHT_FORWARD_SIGN 1
#define BC_CAL_MAX_STEP_DIFFERENCE 0U
#define BC_CAL_MAX_SAMPLE_UNITS 0U
#define BC_CAL_MAX_SAMPLE_MSEC 0U
#define BC_CAL_BLACK_LEVEL 0U
#endif

/* Larger task priority numbers have lower precedence in this project. */
#define BC_CAL_DIAG_PRIORITY 9
#define BC_CAL_DIAG_PERIOD_MSEC 50U

#endif /* BC_CAL_H */

/*** end of file ***/

