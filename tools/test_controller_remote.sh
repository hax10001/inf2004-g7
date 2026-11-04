#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp "${TMPDIR:-/tmp}/g7-controller-XXXXXX")"
trap 'rm -f "$OUT"' EXIT
cc -std=c99 -O2 -Wall -Wextra -Werror \
  -ffunction-sections -fdata-sections -Wl,--gc-sections \
  -DTARGET_DIR=pico_rp2040 -DPROHIBIT_DEF_SIZE_T \
  -include stddef.h -I"$ROOT" -I"$ROOT/include" -I"$ROOT/config" \
  -I"$ROOT/kernel/knlinc" -I"$ROOT/app_program" \
  -I"$ROOT/device/include" \
  "$ROOT/tools/test_controller_remote.c" \
  "$ROOT/app_program/app/nav_core.c" -o "$OUT"
"$OUT"
