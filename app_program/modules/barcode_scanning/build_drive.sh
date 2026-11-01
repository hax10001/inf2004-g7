#!/usr/bin/env bash
# Moving barcode build: USB console only, no interrupt-blocking UART mirror.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
# Force all C objects to pick up the console override. No kernel source edits.
make -C build_make -B SMP=1 CONSOLE=usb_cdc \
  'GCC=arm-none-eabi-gcc -DTM_CONSOLE_UART=0' -j4
cp build_make/mtk3pico_smp1_usb_cdc.uf2 \
  build_make/mtk3pico_smp1_usb_cdc_barcode_drive_usb_only.uf2
cp build_make/mtk3pico_smp1_usb_cdc.uf2 \
  build_make/mtk3pico_smp1_usb_cdc_barcode_drive_trace.uf2
cp build_make/mtk3pico_smp1_usb_cdc.uf2 \
  build_make/mtk3pico_smp1_usb_cdc_barcode_drive_decode.uf2
# Make's ordinary profile key omits this compiler override. Mark the actual
# profile so a later ordinary make rebuilds instead of reusing these objects.
printf '%s\n' "$(cat build_make/.build_profile)-barcode-usb-only" \
  > build_make/.build_profile
