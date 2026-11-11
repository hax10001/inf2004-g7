/* V11: native bit widths and safe saturation before packed assignments. */
#include "modules/telemetry/buddy1.h"
#include "modules/telemetry/telemetry_peers.h"
#include "modules/barcode_scanning/bc_wave.h"
#include "modules/barcode_scanning/bc_fast.h"
#include "modules/barcode_scanning/bc_port.h"
#include "modules/obstacle/obstacle.h"
#include "modules/obstacle/scan_plan.h"
#include "modules/motion/motion.h"
#include "modules/motion/steering.h"
#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>

static void noop(void *ctx) { (void)ctx; }
static void boundary_snapshot(void) {
    B1_Store store;
    B1_Motion motion = {0};
    B1_Terrain terrain = {0};
    B1_Obstacle obstacle = {0};
    B1_Frame packet = {0};
    B1_SnapshotPayload received = {0};
    scan_plan_point_t point={0};
    point.angle_deg=-70;
    point.range_mm=4000u;
    assert(point.angle_deg==-70&&point.range_mm==4000u);
    point.angle_deg=70;
    assert(point.angle_deg==70);
    assert(sizeof(scan_plan_point_t)==4u);
    assert(sizeof(B1_OdomRecord)==10u);
    assert(b1_store_init(&store,noop,noop,0,99u)==0);
    motion.seq = 1u; motion.ms = 150u;
    motion.left_speed_mm_s = INT16_MAX;
    motion.right_speed_mm_s = INT16_MIN;
    assert(b1_ingest_motion(&store,&motion)==0);
    terrain.seq = 1u; terrain.ms = 150u;
    terrain.hump_mm = UINT16_MAX;
    terrain.max_hump_mm = UINT16_MAX;
    assert(b1_ingest_terrain(&store,&terrain)==0);
    obstacle.seq=1u; obstacle.ms=150u;
    obstacle.distance_cm=UINT16_MAX;
    assert(b1_ingest_obstacle(&store,&obstacle)==0);
    assert(b1_next_frame(&store,300u,&packet)==1);
    assert(packet.type==B1_TYPE_ODOM);
    assert(packet.time_ms==150u);
    assert(b1_get16(packet.payload+4)==1u);
    assert(b1_next_frame(&store,300u,&packet)==1);
    assert(packet.type==B1_TYPE_SNAPSHOT);
    assert(b1_decode_snapshot(packet.payload,packet.payload_len,&received)==0);
    assert(received.speed_left_mm_s==2047);
    assert(received.speed_right_mm_s==-2047);
    assert(received.hump_mm==1023u);
    assert(received.max_hump_mm==1023u);
    assert(received.obstacle_cm==1023u);
}
static void field_limits(void) {
    B1_Event event={0};
    B1_Terrain terrain={0};
    B1_Mission mission={0};
    B1_Obstacle obstacle={0};
    B1_Line line={0};
    G7_TelemetryPeers peers={0};
    bc_wave_status_t scanner={0};
    bc_fast_adc_stats_t adc={0};
    bc_port_stats_t stats={0};
    obstacle_drive_t drive={0};
    steering_t steering={0};
    motion_status_t motion={0};
    assert(sizeof(event)==9u);
    line.event_kind=20u;
    terrain.quality=1u; terrain.mode=2u; terrain.event_kind=20u;
    mission.mission=7u; mission.event_kind=20u;
    obstacle.side=2u; obstacle.event_kind=20u;
    assert(line.event_kind==20u&&terrain.mode==2u);
    assert(terrain.event_kind==20u&&mission.event_kind==20u);
    assert(obstacle.event_kind==20u);
    peers.barcode_letter=25u; peers.last_ir_bits=7u;
    assert(peers.barcode_letter==25u&&peers.last_ir_bits==7u);
    stats.raw=1u;stats.state=5u;stats.runs=29u;
    stats.last_error=8u;stats.last_runs=29u;
    assert(stats.raw==1u&&stats.state==5u&&stats.runs==29u);
    assert(stats.last_error==8u&&stats.last_runs==29u);
    adc.latest=4095u;adc.minimum=0u;adc.maximum=4095u;
    assert(adc.latest==4095u&&adc.maximum==4095u);
    scanner.state=6u;scanner.last_valid=1u;
    scanner.last_letter=31u;scanner.last_reason=7u;
    scanner.white_mean=4095u;scanner.white_sigma=4095u;
    assert(scanner.state==6u&&scanner.last_valid);
    assert(scanner.last_letter==31u&&scanner.last_reason==7u);
    assert(scanner.white_mean==4095u&&scanner.white_sigma==4095u);
    drive.b_stop=1u; drive.b_line_follow=1u;
    drive.b_done=1u;drive.b_fault=1u;
    steering.b_off_centre=1u;motion.b_valid=1u;
    assert(drive.b_stop&&drive.b_line_follow&&drive.b_done);
    assert(drive.b_fault&&steering.b_off_centre&&motion.b_valid);
    printf("V11_SIZES snapshot=%zu event=%zu odom=%zu line=%zu terrain=%zu "
           "mission=%zu obstacle=%zu store=%zu peers=%zu "
           "wave=%zu drive=%zu steering=%zu motion=%zu adc=%zu stats=%zu\n",
           sizeof(B1_SnapshotPayload),sizeof(B1_Event),
           sizeof(B1_OdomRecord),sizeof(B1_Line),
           sizeof(B1_Terrain),sizeof(B1_Mission),sizeof(B1_Obstacle),
           sizeof(B1_Store),sizeof(G7_TelemetryPeers),
           sizeof(bc_wave_status_t),sizeof(obstacle_drive_t),
           sizeof(steering_t),sizeof(motion_status_t),sizeof(bc_fast_adc_stats_t),
           sizeof(bc_port_stats_t));
}
int main(void) {
    boundary_snapshot();field_limits();
    puts("PASS V11 saturation, state boundaries, and packed semantic fields");
    return 0;
}
