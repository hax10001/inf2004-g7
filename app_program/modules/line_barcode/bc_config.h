/** @file bc_config.h
 * @brief Barcode-only commissioning settings; no shared wiring changes.
 */
#ifndef BC_CONFIG_H
#define BC_CONFIG_H

/* Leave zero until polarity and both encoder signs have been measured.
 * An uncalibrated scanner does not prevent normal robot startup.
 */
#ifndef BC_CAL_READY
#define BC_CAL_READY 1
#endif
#ifndef BC_BLACK_LEVEL
#define BC_BLACK_LEVEL 1U
#endif
#ifndef BC_LEFT_FORWARD_SIGN
#define BC_LEFT_FORWARD_SIGN (-1)
#endif
#ifndef BC_RIGHT_FORWARD_SIGN
#define BC_RIGHT_FORWARD_SIGN 1
#endif

/* Counts reserved for encoder quantisation and normal pulse timing skew.
 * Steering adds a proportional allowance below; units are not millimetres.
 */
#ifndef BC_MAX_SKEW_UNITS
#define BC_MAX_SKEW_UNITS 6U
#endif
/* Allow moderate line-following corrections. The wheel difference may be
 * this percentage of summed forward travel, plus BC_MAX_SKEW_UNITS.
 * A short distance window also checks slow pivots that fit the count floor.
 * Zero provides no proportional allowance; values must be below 100.
 */
#ifndef BC_MAX_SKEW_PERCENT
#define BC_MAX_SKEW_PERCENT 60U
#endif
#define BC_MAX_GAP_USEC 50000U
/* Reject preemption that separates the IR value and encoder reads too far. */
#define BC_MAX_READ_USEC 1000U
#define BC_SCAN_TIMEOUT_USEC 15000000U
#define BC_STOP_TIMEOUT_USEC 500000U

/* Optional display scale only. Zero means unknown; decoding uses ratios.
 * Measure (abs(delta_left) + abs(delta_right)) / distance_mm.
 */
#ifndef BC_UNITS_PER_MM
#define BC_UNITS_PER_MM 0.0f
#endif

/* One summary per second, outside capture and outside the shared mutex.
 * Printing can disturb encoder timing; leave off for normal operation.
 */
#ifndef BC_DIAGNOSTICS
#define BC_DIAGNOSTICS 1
#endif
#define BC_LOG_PERIOD_USEC 1000000U

#endif /* BC_CONFIG_H */

/*** end of file ***/

