/* Compile-only ARM layout probe. No firmware source or native-size claims. */
#include "app/nav_core.h"
#include "modules/telemetry/buddy1.h"
#include "modules/telemetry/telemetry_peers.h"
#include "modules/motion/downhill.h"
#include "modules/motion/motion.h"
#include "modules/barcode_scanning/bc_fastq.h"
#include "modules/terrain/terrain.h"
#include <stdint.h>
#define SIZE_RECORD(TYPE) const uint8_t size_##TYPE[sizeof(TYPE)] = {0}
SIZE_RECORD(nav_controller_t);
SIZE_RECORD(nav_sample_t);
SIZE_RECORD(nav_result_t);
SIZE_RECORD(B1_Store);
SIZE_RECORD(B1_Motion);
SIZE_RECORD(B1_OdomRecord);
SIZE_RECORD(B1_Event);
SIZE_RECORD(B1_Mission);
SIZE_RECORD(B1_Terrain);
SIZE_RECORD(G7_TelemetryPeers);
SIZE_RECORD(downhill_governor_t);
SIZE_RECORD(motion_status_t);
SIZE_RECORD(terrain_snapshot);
SIZE_RECORD(bc_fastq_sample_t);
#ifdef V15_PROBE
#include "modules/barcode_scanning/scan_handoff.h"
SIZE_RECORD(scan_handoff_t);
#endif
