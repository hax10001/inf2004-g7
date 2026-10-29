#!/usr/bin/env bash
# Host-only tests. Never flashes or moves the robot.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SRC="$ROOT/app_program/modules/barcode_scanning"
BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT
cd "$ROOT"
python3 tools/check_app_standard.py
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
echo 'Host barcode tests passed; 4 ms hardware timing unverified.' 
