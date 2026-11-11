#!/usr/bin/env bash
# Native host checks only. Not an ARM build or physical robot acceptance.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
python3 tools/check_app_standard.py
python3 tools/test_network_owner_boundary.py
bash tools/test_precision_ranges.sh
CC=clang python3 tools/test_v11_minbits.py
python3 tools/test_obstacle_integration.py --cc clang
bash tools/test_autonav.sh
(cd app_program/dashboard && node --test --test-concurrency=1 tests/*.test.js)
printf '\nV11 host regressions passed. ARM build and hardware tests remain separate.\n'
