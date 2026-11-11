#!/usr/bin/env bash
# V15 extends every V14 acceptance suite; hardware checks remain separate.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
bash tools/run_v14_tests_mac.sh
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
echo 'V15 HOST PASS; physical acceptance is still required.'
