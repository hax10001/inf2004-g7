/** @file telemetry_peers.h
 * @brief Bridge subsystem snapshots and consume validated barcode results.
 * Never owns motors, I2C, GPIO or the Mission Coordinator.
 */
#ifndef TELEMETRY_PEERS_H
#define TELEMETRY_PEERS_H
#include "buddy1.h"
#include <stdint.h>
typedef struct {
    uint8_t have_motion,have_ir,have_imu;
    uint8_t barcode_valid, barcode_letter, last_ir_bits;
    uint8_t have_obstacle;
    uint16_t obstacle_seq,obstacle_event_seq;
    uint32_t obstacle_last_status;
    uint32_t barcode_last_seq;
    uint32_t last_sensor_report_ms;
    uint32_t last_barcode_report_ms;
    uint8_t odom_confidence; /* 0=absent, 1=nominal, 2=measured */
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

