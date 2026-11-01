#!/usr/bin/env bash
# Host-only tests. Never flashes or moves the robot.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SRC="$ROOT/app_program/modules/barcode_scanning"
BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT
cd "$ROOT"
python3 tools/check_app_standard.py
python3 "$SRC/integration_audit.py"
: "${CC:=gcc}"
"$CC" -std=c99 -Wall -Wextra -Werror \
  -I tools/tests/stubs -I app_program -DBC_CAL_ENCODER_ONLY=1 \
  "$ROOT/app_program/app_main.c" "$SRC/bc_enc.c" \
  "$SRC/host/test_encoder.c" -o "$BUILD/barcode_encoder"
for scenario in normal zero moving failure; do
  "$BUILD/barcode_encoder" "$scenario"
done
"$CC" -std=c99 -Wall -Wextra -Werror -pedantic \
  -DBC_BARCODE_TEST_MAIN -DBC_CAL_TEST_RELAXED_CAPTURE=0 -I app_program \
  "$SRC/bc_code.c" "$SRC/bc_scan.c" "$SRC/test_barc.c" \
  -o "$BUILD/barcode_core"
"$BUILD/barcode_core"
"$CC" -std=c99 -Wall -Wextra -Werror -pthread \
  -DBC_PORT_HOST_TEST_MAIN -DBC_CAL_TEST_IGNORE_INPUT_ERRORS=0 \
  -DBC_CAL_TEST_RELAXED_CAPTURE=0 \
  -I tools/tests/stubs -I app_program \
  -include "$SRC/host/barcode_kernel.h" \
  "$SRC/bc_code.c" "$SRC/bc_scan.c" "$SRC/bc_port.c" \
  "$SRC/bc_uart.c" "$SRC/test_port.c" -o "$BUILD/barcode_adapter"
"$BUILD/barcode_adapter"
"$CC" -std=c99 -Wall -Wextra -Werror -pthread \
  -DBC_FAST_HOST_TEST_MAIN -I app_program \
  "$SRC/bc_fastq.c" "$SRC/test_fast.c" -o "$BUILD/barcode_fast"
"$BUILD/barcode_fast"
build_live () {
  local output="$1"
  shift
  "$CC" -std=c99 -Wall -Wextra -Werror \
    -DBC_LIVE_TEST_MAIN -D_PICO_RP2040_ "$@" \
    -include "${BC_TEST_HW:-$SRC/host/barcode_hw.h}" \
    -I tools/tests/stubs -I app_program \
    "$SRC/bc_code.c" "$SRC/bc_scan.c" "$SRC/bc_port.c" \
    "$SRC/bc_uart.c" "$SRC/bc_fastq.c" "$SRC/bc_fast.c" \
    "$SRC/bc_setup.c" "$SRC/test_live.c" -o "$output"
}
build_live "$BUILD/barcode_live"
for scenario in scan wrap late alarm conflict worker backlog arm_conflict stuck; do
  "$BUILD/barcode_live" "$scenario"
done
build_live "$BUILD/barcode_invalid" -DBC_TEST_INVALID_CAL
"$BUILD/barcode_invalid"
build_live "$BUILD/barcode_measure" -DBC_TEST_MEASURE
"$BUILD/barcode_measure"
build_live "$BUILD/barcode_off" -DBC_CAL_READY=0
"$BUILD/barcode_off"
build_live "$BUILD/barcode_manual" -DBC_CAL_AUTO_ACK=0
"$BUILD/barcode_manual"
BC_TEST_HW="$SRC/host/drive_hw.h" build_live "$BUILD/barcode_drive" \
  -DBC_TEST_DRIVE_CAL
"$BUILD/barcode_drive" drive_small
"$BUILD/barcode_drive" drive_large
"$BUILD/barcode_drive" stuck
echo 'Host barcode tests passed; 4 ms hardware timing unverified.' 
