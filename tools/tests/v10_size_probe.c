/* Native (not ARM) build comparison for the selected application records. */
#include <stdio.h>
#include "app/nav_core.h"
#include "modules/barcode_scanning/bc_code.h"
#include "modules/barcode_scanning/bc_scan.h"
#include "modules/barcode_scanning/bc_fastq.h"
#include "modules/barcode_scanning/bc_match.h"
#include "modules/barcode_scanning/bc_layout.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/terrain/terrain.h"
#include "modules/telemetry/buddy1.h"
#include "modules/telemetry/telemetry_peers.h"
#define MEASURE(name,type) printf("%s,%zu\n",name,sizeof(type))
int main(void){
 puts("Type,Bytes");
 MEASURE("nav_config_t",nav_config_t);
 MEASURE("nav_sample_t",nav_sample_t);
 MEASURE("nav_controller_t",nav_controller_t);
 MEASURE("nav_result_t",nav_result_t);
 MEASURE("bc_code_result_t",bc_code_result_t);
 MEASURE("bc_scan_t",bc_scan_t);
 MEASURE("bc_fastq_sample_t",bc_fastq_sample_t);
 printf("bc_fastq_ring_bytes,%zu\n",sizeof(bc_fastq_sample_t)*BC_FASTQ_CAPACITY);
 MEASURE("bc_match_result_t",bc_match_result_t);
 MEASURE("bc_layout_result_t",bc_layout_result_t);
 MEASURE("ir_shared_snapshot_t",ir_shared_snapshot_t);
 MEASURE("terrain_snapshot",terrain_snapshot);
 MEASURE("B1_Store",B1_Store);
 MEASURE("G7_TelemetryPeers",G7_TelemetryPeers);
 return 0;
}
