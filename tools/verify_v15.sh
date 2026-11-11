#!/usr/bin/env bash
# Reproducible host tests and clean source/UF2 verification.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
: "${PICO_SDK_PATH:?Set PICO_SDK_PATH to the installed Pico SDK 2.2.0}"
mkdir -p docs/V15/evidence
bash tools/run_host_tests.sh > docs/V15/evidence/host-tests.txt 2>&1
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 bash build_uf2.sh \
  > docs/V15/evidence/target-build.txt 2>&1
arm-none-eabi-size -A build_make/mtk3pico_smp1_usb_cdc_wifi_dhcp.elf \
  > docs/V15/evidence/arm-sections-v15.txt
echo 'V15 host checks and target UF2 build passed; physical trials pending.'
