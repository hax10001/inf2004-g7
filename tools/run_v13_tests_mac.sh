#!/usr/bin/env bash
# Host-only tests; use ARM build and physical ramp checks separately.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
CC="${CC:-clang}"
TMP="$(mktemp -d "${TMPDIR:-/tmp}/g7-v13-XXXXXX")"
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
printf '\nV13 host tests passed. ARM build and physical downhill sign PENDING.\n'
