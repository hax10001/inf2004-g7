#!/usr/bin/env bash
# Complete host (native, not ARM) regression suite. Physical Pico/robot checks
# remain separate. Formerly split across run_v13/v14/v15 test scripts, which
# called each other in this same order.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# --- Downhill, motion, standards, network, precision, minbits, obstacle,
#     navigation and dashboard. These run with CC exported (default clang). ---
export CC="${CC:-clang}"
TMP="$(mktemp -d "${TMPDIR:-/tmp}/g7-host-XXXXXX")"
GC_FLAG=-Wl,--gc-sections
if [[ "$(uname -s)" == Darwin ]]; then GC_FLAG=-Wl,-dead_strip; fi
trap 'rm -rf "$TMP"' EXIT
for SIGN in 0 1 -1; do
  "$CC" -std=c99 -O2 -Wall -Wextra -Werror \
    -DAPP_DOWNHILL_NOSE_DOWN_SIGN="$SIGN" \
    -Iapp_program app_program/modules/motion/downhill.c \
    tools/tests/test_downhill_governor.c -lm -o "$TMP/slope-$SIGN"
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
# Closed-loop line following: the real motion, steering, IR latches and
# navigation on a simulated robot and track, at the default speed and a
# faster one (curves and 90 degree corners must hold).
for RPM in 30 35 55; do
  "$CC" -std=c99 -O2 -Wall -Wextra -Werror \
    -DMOTION_TARGET_RPM="${RPM}.0f" -DSIM_RPM="$RPM" \
    -Itools/tests/stubs -I. -Iapp_program \
    tools/tests/line_follow_sim.c \
    app_program/modules/motion/motion.c \
    app_program/modules/motion/steering.c \
    app_program/modules/motion/wheel_speed.c \
    app_program/modules/motion/downhill.c \
    app_program/modules/line_barcode/ir_shared.c \
    app_program/modules/line_barcode/line_follow.c \
    app_program/app/nav_core.c \
    -lm -o "$TMP/line-follow-$RPM"
  "$TMP/line-follow-$RPM"
done
# Lifted-height estimator (pure arithmetic).
"$CC" -std=c99 -O2 -Wall -Wextra -Werror -Iapp_program \
  app_program/modules/terrain/lift_math.c tools/tests/test_lift_math.c \
  -o "$TMP/lift-math"
"$TMP/lift-math"
# Route-map telemetry: ODOM checkpoints, hump EVENTs, mission EVENT IDs
# and bounded offline rings, on the real telemetry_peers.c and buddy1.c.
"$CC" -std=c99 -O2 -Wall -Wextra -Werror -Itools/tests/stubs -I. \
  -Iapp_program -Iapp_program/modules/telemetry \
  -include tools/tests/sem_stub.h tools/tests/test_telemetry_map.c \
  app_program/modules/telemetry/telemetry_peers.c \
  app_program/modules/telemetry/buddy1.c \
  app_program/modules/telemetry/buddy1_protocol.c -lm -o "$TMP/telemetry-map"
"$TMP/telemetry-map"
# Hump climb check (23 Nov): the real detector against today's humps.
"$CC" -std=c99 -O2 -Wall -Wextra -Werror -Itools/tests/stubs -I. \
  -Iapp_program -include tools/tests/sem_stub.h -ffunction-sections \
  -fdata-sections "$GC_FLAG" tools/tests/test_hump_climb.c \
  app_program/modules/terrain/tilt_math.c -lm -o "$TMP/hump-climb"
"$TMP/hump-climb"
# Requirement traceability (docs/tribunal2/requirements_traceability.md):
# FR2/FR13/NFR13 behaviour on the real telemetry store and codec, and every
# numeric value in the requirement table traced to its #define.
"$CC" -std=c99 -O2 -Wall -Wextra -Werror -pthread \
  -Iapp_program/modules/telemetry tools/tests/test_requirements_trace.c \
  app_program/modules/telemetry/buddy1.c \
  app_program/modules/telemetry/buddy1_protocol.c -o "$TMP/req-trace"
"$TMP/req-trace"
# FR4 lost-line search on the real motion.c.
"$CC" -std=c99 -O2 -Wall -Wextra -Werror -Itools/tests/stubs -I. \
  -Iapp_program tools/tests/test_line_search.c \
  app_program/modules/motion/motion.c \
  app_program/modules/motion/downhill.c \
  app_program/modules/motion/steering.c \
  app_program/modules/motion/wheel_speed.c -lm -o "$TMP/line-search"
"$TMP/line-search"
# FR3 IMU boot calibration on the real task_imu.c (50 samples, retries,
# movement and gravity-magnitude rejection).
"$CC" -std=c99 -O2 -Wall -Wextra -Werror -Itools/tests/stubs -I. \
  -Iapp_program -include tools/tests/sem_stub.h \
  app_program/modules/terrain/host/test_calibration.c \
  app_program/modules/terrain/task_imu.c \
  app_program/modules/terrain/terrain.c \
  app_program/modules/terrain/tilt_math.c \
  app_program/modules/terrain/lift_math.c \
  tools/tests/encoder_read_stub.c -lm -o "$TMP/imu-calibration"
for CASE in steady read_failure movement magnitude persistent; do
  "$TMP/imu-calibration" "$CASE"
done
python3 tools/test_no_heap.py
python3 tools/test_requirement_constants.py
python3 tools/check_app_standard.py
python3 tools/test_network_owner_boundary.py
bash tools/test_precision_ranges.sh
python3 tools/test_minbits_all.py
python3 tools/test_obstacle_integration.py --cc "$CC"
bash tools/test_autonav.sh
(cd app_program/dashboard && node --test --test-concurrency=1 tests/*.test.js)
printf '\nCore host tests passed: lift estimation, STOP/START, governor, navigation, dashboard.\n'

# --- Scan handoff, production controller, scanner readiness, ramp plant ---
mkdir -p "$TMP/v15"
GC=-Wl,--gc-sections
if [[ "$(uname -s)" == Darwin ]]; then GC=-Wl,-dead_strip; fi
cc -std=c99 -O2 -Wall -Wextra -Werror -ffunction-sections \
  -fdata-sections "$GC" -Itools/tests/stubs -Iapp_program \
  -include tools/tests/sem_stub.h \
  tools/test_scan_handoff.c -o "$TMP"/v15/scan_handoff
"$TMP"/v15/scan_handoff
cc -std=c99 -O2 -Wall -Wextra -Werror -ffunction-sections \
  -fdata-sections "$GC" -Itools/tests/stubs -I. -Iapp_program \
  tools/test_controller_handoff.c app_program/app/nav_core.c \
  app_program/modules/line_barcode/line_follow.c \
  -o "$TMP"/v15/controller
"$TMP"/v15/controller
# Same production controller with the previous stopping behaviour: lift
# stop latched, and unmapped/lost barcodes as navigation safety stops.
cc -std=c99 -O2 -Wall -Wextra -Werror -ffunction-sections \
  -fdata-sections "$GC" -Itools/tests/stubs -I. -Iapp_program \
  -DAPP_LIFT_RESUME_ENABLED=0 -DAPP_NAV_SCAN_FAIL_STOP=1 \
  tools/test_controller_handoff.c app_program/app/nav_core.c \
  app_program/modules/line_barcode/line_follow.c \
  -o "$TMP"/v15/controller_lift_latched
"$TMP"/v15/controller_lift_latched
cc -std=c99 -O2 -Wall -Wextra -Werror -ffunction-sections \
  -fdata-sections "$GC" -Itools/tests/stubs -Iapp_program \
  tools/test_scanner_ready.c -o "$TMP"/v15/scanner_ready
"$TMP"/v15/scanner_ready
# Same real scanner reusing the startup white (setting on).
cc -std=c99 -O2 -Wall -Wextra -Werror -ffunction-sections \
  -fdata-sections "$GC" -Itools/tests/stubs -Iapp_program \
  -DAPP_BARCODE_KEEP_STARTUP_WHITE=1 \
  tools/test_scanner_ready.c -o "$TMP"/v15/scanner_ready_keep
"$TMP"/v15/scanner_ready_keep
# Same real scanner with the capture gate off (previous trigger, no drop).
cc -std=c99 -O2 -Wall -Wextra -Werror -ffunction-sections \
  -fdata-sections "$GC" -Itools/tests/stubs -Iapp_program \
  -DAPP_BARCODE_TRIGGER_RISE=0U -DAPP_BARCODE_MIN_DARK_UNITS=0U \
  tools/test_scanner_ready.c -o "$TMP"/v15/scanner_ready_gate_off
"$TMP"/v15/scanner_ready_gate_off
for SIGN in 0 1 -1; do
  cc -std=c99 -O2 -Wall -Wextra -Werror -Itools/tests/stubs -Iapp_program \
    -DAPP_DOWNHILL_NOSE_DOWN_SIGN="$SIGN" \
    tools/test_ramp_motion.c app_program/modules/motion/motion.c \
    app_program/modules/motion/downhill.c app_program/drivers/motor.c \
    app_program/modules/motion/steering.c \
    app_program/modules/motion/wheel_speed.c -lm -o "$TMP"/v15/ramp_motion
  G7_TEST_TMP="$TMP/v15" "$TMP"/v15/ramp_motion
done
echo 'HOST PASS; ARM build and physical acceptance are still required.'
