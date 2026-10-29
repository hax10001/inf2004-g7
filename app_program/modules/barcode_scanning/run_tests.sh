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
"$CC" -std=c99 -Wall -Wextra -Werror -pedantic \
  -DBC_BARCODE_TEST_MAIN -I app_program \
  "$SRC/bc_code.c" "$SRC/bc_scan.c" "$SRC/test_barc.c" \
  -o "$BUILD/barcode_core"
"$BUILD/barcode_core"
"$CC" -std=c99 -Wall -Wextra -Werror -pthread \
  -DBC_PORT_HOST_TEST_MAIN -I tools/tests/stubs -I app_program \
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
    -include "$SRC/host/barcode_hw.h" \
    -I tools/tests/stubs -I app_program \
    "$SRC/bc_code.c" "$SRC/bc_scan.c" "$SRC/bc_port.c" \
    "$SRC/bc_uart.c" "$SRC/bc_fastq.c" "$SRC/bc_fast.c" \
    "$SRC/bc_setup.c" "$SRC/test_live.c" -o "$output"
}
build_live "$BUILD/barcode_live"
for scenario in scan wrap late alarm conflict worker; do
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
echo 'Host barcode tests passed; 4 ms hardware timing unverified.' 
