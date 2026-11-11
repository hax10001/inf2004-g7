/** @file telemetry_peers.h
 * @brief Bridge subsystem snapshots and consume validated barcode results.
 * Never owns motors, I2C, GPIO or the Mission Coordinator.
 */
#ifndef TELEMETRY_PEERS_H
#define TELEMETRY_PEERS_H
#include "buddy1.h"
#include <stdint.h>
typedef struct {
    /* Single telemetry-task owner: compact booleans and 3-state quality.
     * Never share packed storage with an ISR without synchronization. */
    unsigned int have_motion:1;
    unsigned int have_ir:1;
    unsigned int have_imu:1;
    unsigned int barcode_valid:1;
    unsigned int have_obstacle:1;
    /* 0 absent, 1 nominal, 2 measured */
    unsigned int odom_confidence:2;
    /* A-Z plus sentinel 31, and three IR GPIO states. The telemetry task
     * owns the entire snapshot; no ISR updates these packed words. */
    unsigned int barcode_letter:5;
    unsigned int last_ir_bits:3;
    uint16_t obstacle_seq,obstacle_event_seq;
    uint32_t obstacle_last_status;
    uint32_t barcode_last_seq;
    uint32_t last_sensor_report_ms;
    uint32_t last_barcode_report_ms;
    uint16_t motion_seq,ir_seq,imu_seq;
    uint32_t motion_last_us,ir_last_ms,imu_last_ms;
    int32_t previous_left_count,previous_right_count;
    float left_residual_mm,right_residual_mm;
    /* Calibration from Buddy 2: zero means odometry unavailable. */
    float left_mm_per_count,right_mm_per_count;
} G7_TelemetryPeers;
/* Called by a lower-priority telemetry task only, never an ISR. */
uint8_t g7_telemetry_poll(G7_TelemetryPeers *peers,B1_Store *store);
#endif
/*** end of file ***/

