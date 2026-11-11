/* Native host size probe. NEVER substitute for target ARM RAM/stack proof. */
#include "app/nav_core.h"
#include "modules/telemetry/buddy1.h"
#include "modules/telemetry/telemetry_peers.h"
#include "modules/barcode_scanning/bc_fastq.h"
#include "modules/barcode_scanning/bc_fast.h"
#include "modules/barcode_scanning/bc_port.h"
#include "modules/barcode_scanning/bc_wave.h"
#include "modules/barcode_scanning/bc_code.h"
#include "modules/obstacle/scan_plan.h"
#include "modules/obstacle/obstacle.h"
#include "modules/motion/steering.h"
#include "modules/motion/motion.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/terrain/terrain.h"
#include <stdio.h>
#define ROW(X) printf(#X ",%zu\n",sizeof(X))
int main(void) {
    puts("Type,Bytes");
    ROW(B1_Motion); ROW(B1_Event); ROW(B1_Store);
    ROW(B1_SnapshotPayload); ROW(B1_Line); ROW(B1_Terrain);
    ROW(B1_Mission); ROW(B1_Obstacle); ROW(G7_TelemetryPeers);
    ROW(bc_fastq_sample_t); ROW(bc_fast_adc_stats_t); ROW(bc_port_stats_t);
    ROW(bc_wave_status_t); ROW(bc_code_pattern_t); ROW(scan_plan_point_t);
    ROW(obstacle_drive_t); ROW(motion_status_t); ROW(steering_t);
    ROW(ir_shared_snapshot_t); ROW(terrain_snapshot);
    ROW(nav_controller_t);
    return 0;
}
