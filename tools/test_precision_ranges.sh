#!/usr/bin/env bash
# Host checks for code-proven exact ranges; not proof of on-car behavior.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$(mktemp "${TMPDIR:-/tmp}/g7-precision-XXXXXXXX")"
trap 'rm -f "$BIN"' EXIT
"${CC:-cc}" -std=c11 -O2 -Wall -Wextra -Werror \
  -I"$ROOT/app_program" -I"$ROOT/tools/tests/stubs" -I"$ROOT/include" \
  "$ROOT/tools/tests/test_precision_ranges.c" -o "$BIN"
"$BIN"
