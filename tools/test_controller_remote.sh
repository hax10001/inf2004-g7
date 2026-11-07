#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp "${TMPDIR:-/tmp}/g7-controller-XXXXXX")"
trap 'rm -f "$OUT"' EXIT
cc -std=c99 -O2 -Wall -Wextra -Werror \
  -ffunction-sections -fdata-sections -Wl,--gc-sections \
  -fno-asynchronous-unwind-tables -fno-unwind-tables \
  -DTARGET_DIR=pico_rp2040 -DPROHIBIT_DEF_SIZE_T \
  -include stddef.h -I"$ROOT/tools/tests/stubs" \
  -I"$ROOT" -I"$ROOT/include" -I"$ROOT/config" \
  -I"$ROOT/kernel/knlinc" -I"$ROOT/app_program" \
  -I"$ROOT/device/include" \
  "$ROOT/tools/test_controller_remote.c" \
  "$ROOT/app_program/app/nav_core.c" \
  "$ROOT/app_program/modules/line_barcode/line_follow.c" -o "$OUT"
"$OUT"
