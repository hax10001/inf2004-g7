/** @file bc_cal.h
 * @brief Hardware calibration gate; never guess physical scan thresholds.
 */
#ifndef BC_CAL_H
#define BC_CAL_H

/* 0 disables barcode work. With 1, valid settings enable decoding; invalid
 * settings select measurement logging instead of aborting robot startup.
 */
#define BC_CAL_READY 0

/* Force measurement logging even after valid scan settings are entered. */
#define BC_CAL_MEASURE 0

/* Standalone console consumer releases results after reporting them.
 * Set to 0 when a mission task owns bc_port_peek()/bc_port_ack().
 */
#define BC_CAL_AUTO_ACK 0

#if BC_CAL_READY
/* Units = sum of normalized wheel encoder increments, NOT millimetres.
 * Measure the actual printed bars and margins; see README.md.
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
#define BC_CAL_DIAG_PERIOD_MSEC 10U

/* 4 ms alarm period comes from bc_fastq.h. */

#endif /* BC_CAL_H */

/*** end of file ***/

