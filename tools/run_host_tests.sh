#!/usr/bin/env bash
# Complete host (native, not ARM) regression suite. Physical Pico/robot checks
# remain separate. Formerly split across run_v13/v14/v15 test scripts, which
# called each other in this same order.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# --- Terrain collision latch (uses the caller's CC, else clang on macOS) ---
python3 tools/test_v14_terrain_host.py

# --- Downhill, motion, standards, network, precision, minbits, obstacle,
#     navigation and dashboard. These run with CC exported (default clang). ---
export CC="${CC:-clang}"
TMP="$(mktemp -d "${TMPDIR:-/tmp}/g7-host-XXXXXX")"
trap 'rm -rf "$TMP"' EXIT
for SIGN in 0 1 -1; do
  "$CC" -std=c99 -O2 -Wall -Wextra -Werror \
    -DAPP_DOWNHILL_NOSE_DOWN_SIGN="$SIGN" \
    -Iapp_program app_program/modules/motion/downhill.c \
    tools/tests/test_v13_downhill.c -lm -o "$TMP/slope-$SIGN"
  "$TMP/slope-$SIGN"
done
# Test the production speed-controller path with each calibrated sign.
for SIGN in 1 -1; do
  "$CC" -std=c99 -O2 -Wall -Wextra -Werror \
    -DAPP_DOWNHILL_NOSE_DOWN_SIGN="$SIGN" \
    -Itools/tests/stubs -I. -Iapp_program \
    tools/tests/obstacle_motion.c \
    app_program/modules/motion/motion.c \
    app_program/modules/motion/downhill.c \
    app_program/modules/motion/steering.c \
    app_program/modules/motion/wheel_speed.c \
    -lm -o "$TMP/motion-$SIGN"
  "$TMP/motion-$SIGN"
done
python3 tools/check_app_standard.py
python3 tools/test_network_owner_boundary.py
bash tools/test_precision_ranges.sh
python3 tools/test_v11_minbits.py
python3 tools/test_obstacle_integration.py --cc "$CC"
bash tools/test_autonav.sh
(cd app_program/dashboard && node --test --test-concurrency=1 tests/*.test.js)
printf '\nCore host tests passed: collision rearm, STOP/START, governor, navigation, dashboard.\n'

# --- Scan handoff, production controller, scanner readiness, ramp plant ---
mkdir -p tmp/v15
GC=-Wl,--gc-sections
if [[ "$(uname -s)" == Darwin ]]; then GC=-Wl,-dead_strip; fi
cc -std=c99 -O2 -Wall -Wextra -Werror -ffunction-sections \
  -fdata-sections "$GC" -Itools/tests/stubs -Iapp_program \
  -include tools/tests/v11_sem_stub.h \
  tools/test_scan_handoff.c -o tmp/v15/scan_handoff
tmp/v15/scan_handoff
cc -std=c99 -O2 -Wall -Wextra -Werror -ffunction-sections \
  -fdata-sections "$GC" -Itools/tests/stubs -I. -Iapp_program \
  tools/test_v15_controller.c app_program/app/nav_core.c \
  app_program/modules/line_barcode/line_follow.c \
  -o tmp/v15/controller
tmp/v15/controller
cc -std=c99 -O2 -Wall -Wextra -Werror -ffunction-sections \
  -fdata-sections "$GC" -Itools/tests/stubs -Iapp_program \
  tools/test_scanner_ready.c -o tmp/v15/scanner_ready
tmp/v15/scanner_ready
for SIGN in 0 1 -1; do
  cc -std=c99 -O2 -Wall -Wextra -Werror -Itools/tests/stubs -Iapp_program \
    -DAPP_DOWNHILL_NOSE_DOWN_SIGN="$SIGN" \
    tools/test_ramp_motion.c app_program/modules/motion/motion.c \
    app_program/modules/motion/downhill.c app_program/drivers/motor.c \
    app_program/modules/motion/steering.c \
    app_program/modules/motion/wheel_speed.c -lm -o tmp/v15/ramp_motion
  tmp/v15/ramp_motion
done
mkdir -p docs/V15/evidence
cp tmp/v15/plant-sign-1.csv tmp/v15/plant-sign--1.csv docs/V15/evidence/
echo 'HOST PASS; ARM build and physical acceptance are still required.'
