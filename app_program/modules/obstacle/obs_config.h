/** @file obs_config.h
 * @brief Obstacle subsystem tunables. MEASURE marks values to calibrate.
 */
#ifndef OBS_CONFIG_H
#define OBS_CONFIG_H

#include "board/board_config.h"

/* ASSUMPTION: 125 MHz system clock, so a divider of 125 gives a 1 us tick. */
#define OBS_PWM_CLK_MHZ 125U

/* Ultrasonic. */
#define OBS_TRIG_PULSE_USEC 12U
#define OBS_ECHO_TIMEOUT_MS 30U
#define OBS_PING_PERIOD_MS 60U
#define OBS_FAR_MM 4000U
#define OBS_STOP_MM 200U /* MEASURE: stop distance. */
#define OBS_CONFIRM_HITS 3U
#define OBS_NO_ECHO_LIMIT 5U
#define OBS_FAILSAFE_NO_ECHO 1 /* 1: persistent no-echo stops the robot. */

/* Servo (SG90). */
#define OBS_SERVO_FRONT_US 1500 /* MEASURE: pulse for physical FRONT. */
#define OBS_SERVO_US_PER_DEG 10 /* MEASURE. */
#define OBS_SERVO_LEFT_SIGN 1   /* MEASURE: +1 if a longer pulse looks left. */
#define OBS_SERVO_MIN_US 600
#define OBS_SERVO_MAX_US 2400
#define OBS_SERVO_LIMIT_DEG 80 /* MEASURE: safe sweep each side. */
#define OBS_SERVO_SETTLE_BASE_MS 40U
#define OBS_SERVO_SETTLE_PER_DEG_MS 3U

/* Scan. */
#define OBS_SCAN_STEP_DEG 10
#define OBS_SCAN_LIMIT_DEG 70
#define OBS_SCAN_MAX_POINTS 15U /* 2 * LIMIT / STEP + 1 */
#define OBS_SCAN_PINGS 3U
#define OBS_SCAN_RANGE_MM 600U
#define OBS_EDGE_JUMP_MM 80U
#define OBS_BYPASS_LOOKAHEAD_MM 500.0F

/* Robot geometry. */
#define OBS_ROBOT_WIDTH_MM 150.0F  /* MEASURE. */
#define OBS_ROBOT_LENGTH_MM 200.0F /* MEASURE. */
#define OBS_SAFETY_MARGIN_MM 40.0F
#define OBS_FRONT_CLEAR_MM 30.0F
#define OBS_TRACK_WIDTH_MM 110.0F /* MEASURE: varies; recalibrate. */

/* Bypass motion. */
#define OBS_DRIVE_RPM 20.0F
#define OBS_TURN_LEAD_DEG 4.0F
#define OBS_ALIGN_TOL_DEG 10.0F
#define OBS_HEADING_KP 0.15F
#define OBS_HEADING_TRIM_MAX 4.0F
#define OBS_STOP_SETTLE_MS 300U
#define OBS_SCAN_TIMEOUT_MS 12000U
#define OBS_STATE_TIMEOUT_MS 15000U
#define OBS_SIDE_LOOK_DEG 70
#define OBS_SIDE_NEAR_MM 400U
#define OBS_SIDE_CLEAR_HITS 3U
#define OBS_PASS_MAX_MM 800.0F
#define OBS_PASS_MARGIN_MM 40.0F
#define OBS_SEARCH_MARGIN_MM 150.0F
#define OBS_LINE_CONFIRM_CYCLES 2U
#define OBS_VERIFY_CYCLES 20U

#endif /* OBS_CONFIG_H */

/*** end of file ***/

