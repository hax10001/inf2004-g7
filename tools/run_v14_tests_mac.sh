#!/usr/bin/env bash
# Complete runnable V14 host tests. Does not claim physical Pico validation.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
python3 tools/test_v14_terrain_host.py
CC="${CC:-clang}" bash tools/run_v13_tests_mac.sh
printf '\nV14 PASS: collision rearm, STOP/START, smooth governor, navigation, dashboard.\n'
printf 'ARM SDK build and motor/ramp physical verification remain REQUIRED.\n'
