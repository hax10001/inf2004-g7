/** @file telemetry_peers.h
 * @brief Bridge subsystem snapshots and consume validated barcode results.
 * Never owns motors, I2C, GPIO or the Mission Coordinator.
 */
#ifndef TELEMETRY_PEERS_H
#define TELEMETRY_PEERS_H
#include "buddy1.h"
#include <stdint.h>
typedef struct {
    /* Owned by the telemetry task alone; no ISR touches these bit-fields. */
    uint32_t     have_motion:1;
    uint32_t     have_ir:1;
    uint32_t     have_imu:1;
    uint32_t     barcode_valid:1;
    uint32_t     have_obstacle:1;
    /* 0 absent, 1 nominal, 2 measured */
    uint32_t     odom_confidence:2;
    /* Letter A-Z as 0..25 (31 none); three IR sensor bits. */
    uint32_t     barcode_letter:5;
    uint32_t     last_ir_bits:3;
    /* Terrain's hump-recorded toggle at the last sample (a change is one
     * recorded hump; discarded humps do not flip it). */
    uint32_t     hump_toggle:1;
    uint16_t obstacle_seq;
    uint16_t obstacle_event_seq;
    /* Wire EVENT ID for crossed humps; 16 bits as on the wire. */
    uint16_t hump_event_seq;
    /* Distance driven in half-millimetres, |L+R| summed (wraps after
     * about 2,100 km): the ODOM checkpoint's cumulative distance. */
    uint32_t odom_travel_x2;
    uint32_t obstacle_last_status;
    uint32_t barcode_last_seq;
    uint32_t last_sensor_report_ms;
    uint32_t last_barcode_report_ms;
    uint16_t motion_seq;
    uint16_t ir_seq;
    uint16_t imu_seq;
    uint32_t motion_last_us;
    uint32_t ir_last_ms;
    uint32_t imu_last_ms;
    int32_t previous_left_count;
    int32_t previous_right_count;
    float left_residual_mm;
    float right_residual_mm;
    /* Wheel scale; zero means odometry unavailable. */
    float left_mm_per_count;
    float right_mm_per_count;
} g7_telemetry_peers_t;
/** @brief Copy subsystem snapshots into the store; returns the B1_SRC_*
 *  bits updated. Telemetry task only, never an ISR. */
uint8_t g7_telemetry_poll(g7_telemetry_peers_t *p_peers,b1_store_t *p_store);
#endif
/*** end of file ***/

