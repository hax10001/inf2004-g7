#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$(mktemp "${TMPDIR:-/tmp}/g7-nav-XXXXXX")"
trap 'rm -f "$BIN"' EXIT
cc -std=c99 -O2 -Wall -Wextra -Werror -I"$ROOT/app_program" \
 "$ROOT/app_program/app/nav_core.c" "$ROOT/tools/test_autonav.c" -o "$BIN"
"$BIN"
