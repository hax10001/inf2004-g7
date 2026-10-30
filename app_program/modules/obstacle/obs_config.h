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
#define OBS_PING_PERIOD_MSEC 60U
#define OBS_MEDIAN_SAMPLES 3U
#define OBS_FAR_MM 4000
/* Measure: how close an obstacle may get before the robot stops. */
#define OBS_STOP_MM 800
#define OBS_EMERGENCY_MM 250
#define OBS_CONFIRM_HITS 3U
#define OBS_NO_ECHO_LIMIT 5U

/* Servo (SG90). Measure every value in this group. */
#define OBS_SERVO_FRONT_USEC 1500
#define OBS_SERVO_USEC_PER_DEG 10
/* A value of 1 means that a longer pulse turns the sensor left. */
#define OBS_SERVO_LEFT_SIGN 1
#define OBS_SERVO_MIN_USEC 600
#define OBS_SERVO_MAX_USEC 2400
#define OBS_SERVO_LIMIT_DEG 80
#define OBS_SERVO_SETTLE_BASE_MSEC 40
#define OBS_SERVO_SETTLE_PER_DEG_MSEC 3

/* Panning scan. The step must be a multiple of 10 degrees. */
#define OBS_SCAN_STEP_DEG 10
#define OBS_SCAN_LIMIT_DEG 70
/* Equals two times the limit divided by the step, plus one. */
#define OBS_SCAN_POINTS 15
#define OBS_SCAN_PINGS 3U
#define OBS_SCAN_RANGE_MM 1000
#define OBS_EDGE_JUMP_MM 80
#define OBS_FRONT_WINDOW_DEG 20
#define OBS_LOOKAHEAD_MM 1000

/* Overall measured envelope, including the left IR sensor and wheels. */
#define OBS_ROBOT_WIDTH_MM 153
/* Latest axle measurements are 135 + 65 = 200 mm; use the longer envelope
 * rather than the earlier 190 mm total-length estimate.
 */
#define OBS_ROBOT_LENGTH_MM 200
#define OBS_ROBOT_HEIGHT_MM 160
#define OBS_WHEEL_WIDTH_MM 16
/* User-supplied axle and sonar measurements. Physical trials are pending. */
#ifndef OBS_GEOMETRY_VERIFIED
#define OBS_GEOMETRY_VERIFIED 1
#endif
#define OBS_FRONT_OVERHANG_MM 135
#define OBS_REAR_OVERHANG_MM (OBS_ROBOT_LENGTH_MM - OBS_FRONT_OVERHANG_MM)
#define OBS_SENSOR_FORWARD_MM 90
#define OBS_SENSOR_LEFT_MM 0
#define OBS_SAFETY_MARGIN_MM 40
#define OBS_POSITION_UNCERTAINTY_MM 20
/* Nominal 15-degree full sonar beam, rounded outward, plus servo error.
 * Reflection/material effects still require supervised physical checks.
 */
#define OBS_BEAM_HALF_ANGLE_DEG 8
#define OBS_SERVO_UNCERTAINTY_DEG 5
#define OBS_ANGLE_UNCERTAINTY_DEG \
    (OBS_BEAM_HALF_ANGLE_DEG + OBS_SERVO_UNCERTAINTY_DEG)
#define OBS_BRAKING_ALLOWANCE_MM 15
#define OBS_FRONT_CLEAR_MM 30
#define OBS_TRACK_WIDTH_MM 110
/* Current branch's calibrated barcode settings use these encoder signs.
 * Confirm by rolling each wheel forward before enabling the obstacle build.
 */
#ifndef OBS_LEFT_FORWARD_SIGN
#define OBS_LEFT_FORWARD_SIGN (-1)
#endif
#ifndef OBS_RIGHT_FORWARD_SIGN
#define OBS_RIGHT_FORWARD_SIGN 1
#endif
/* The 153 mm envelope includes an asymmetric IR extension. Wheel outer
 * edges are at +/-63 mm. Allow either body side to extend to 153-63=90 mm.
 */
#define OBS_BODY_HALF_WIDTH_MM \
    (OBS_ROBOT_WIDTH_MM - ((OBS_TRACK_WIDTH_MM + OBS_WHEEL_WIDTH_MM) / 2))

/* Bypass motion. Heading values are in tenths of a degree. */
#define OBS_DRIVE_RPM 12
#define OBS_TURN_LEAD_DDEG 40
#define OBS_ALIGN_TOL_DDEG 100
#define OBS_HEADING_TRIM_DIV 70
#define OBS_HEADING_TRIM_MAX 4
#define OBS_STOP_SETTLE_MSEC 300U
#define OBS_STOP_TIMEOUT_MSEC 2000U
#define OBS_STOP_CONFIRM_CYCLES 2U
#define OBS_PATH_MAX_ERROR_DDEG 150
/* Fail closed on a count jump or uncommanded backward travel. */
#define OBS_ODOM_MAX_STEP_COUNTS 256
#define OBS_SCAN_TIMEOUT_MSEC 12000U
/* One stationary rescan maximum; reversing needs Buddy 2 validation. */
#define OBS_MAX_RESCANS 1U
#define OBS_STATE_TIMEOUT_MSEC 180000U
#define OBS_BYPASS_TIMEOUT_MSEC 900000U

/* Three views must be sampled before each bounded increment of movement.
 * Reserve one control period's overrun in the braking allowance above.
 */
#define OBS_GUARD_TRAVEL_MM 10
#define OBS_GUARD_TURN_DDEG 30
#define OBS_GUARD_TIMEOUT_MSEC 2500U
#define OBS_FRONT_MAX_AGE_MSEC 250U
#define OBS_ODOM_MAX_AGE_USEC 200000U
#define OBS_STALL_DRIVE_MSEC 1000U
#define OBS_GUARD_ALL_VIEWS 7U
#define OBS_GUARD_FRONT 0U
#define OBS_GUARD_INNER 1U
#define OBS_GUARD_OUTER 2U

/* Passing the object and finding the line again. */
#define OBS_SIDE_LOOK_DEG 70
#define OBS_SIDE_NEAR_MM 600
#define OBS_SIDE_CLEAR_HITS 3U
#define OBS_PASS_MAX_MM 1500
#define OBS_PASS_MARGIN_MM 40
#define OBS_SEARCH_MARGIN_MM 150
#define OBS_LINE_CONFIRM_CYCLES 2U

/* These limits match the compact controller handoff and local counters. */
#if ((OBS_DRIVE_RPM + OBS_HEADING_TRIM_MAX) > 31)
#error "Obstacle wheel requests must fit five bits."
#endif
#if ((OBS_LINE_CONFIRM_CYCLES > 3U) || (OBS_MAX_RESCANS > 1U) || \
     (OBS_CONFIRM_HITS > 3U) || (OBS_SIDE_CLEAR_HITS > 3U) || \
     (OBS_NO_ECHO_LIMIT > 7U))
#error "Obstacle confirmation/rescan counters exceed their packed ranges."
#endif
#if (OBS_STOP_CONFIRM_CYCLES > 3U)
#error "Stop confirmation must fit two bits."
#endif
#if (OBS_SERVO_LIMIT_DEG > 127)
#error "Commanded servo angle must fit signed eight bits."
#endif
#if (OBS_STALL_DRIVE_MSEC > 2047U)
#error "Stall drive timer must fit eleven bits."
#endif
#if (((OBS_LEFT_FORWARD_SIGN != 1) && (OBS_LEFT_FORWARD_SIGN != -1)) || \
     ((OBS_RIGHT_FORWARD_SIGN != 1) && (OBS_RIGHT_FORWARD_SIGN != -1)))
#error "Forward encoder signs must be +1 or -1."
#endif

#endif /* OBS_CONFIG_H */

/*** end of file ***/

