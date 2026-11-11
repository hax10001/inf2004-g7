#!/usr/bin/env bash
# Build-only helper: never flashes hardware; live control requires opt-in.
set -euo pipefail
umask 077
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sdk_version() {
  local file="$1/pico_sdk_version.cmake"
  [[ -f "$file" ]] || return 1
  python3 - "$file" <<'PYVER'
import pathlib, re, sys
s = pathlib.Path(sys.argv[1]).read_text()
parts = []
for key in ("MAJOR", "MINOR", "REVISION"):
    match = re.search(r"^\s*set\(PICO_SDK_VERSION_" + key +
                      r"\s+(\d+)\)", s, re.M)
    if match is None:
        sys.exit(1)
    parts.append(match.group(1))
print(".".join(parts))
PYVER
}

# Respect an explicitly selected SDK. Without an override, try known SDK
# locations and prefer a compatible 2.2.0 install over the newest checkout.
if [[ -n "${PICO_SDK_PATH:-}" ]]; then
  SDK="$PICO_SDK_PATH"
else
  SDK=""
  for candidate in \
    "$HOME/pico/pico-sdk-2.2.0" \
    "$HOME/pico-sdk-2.2.0" \
    "$HOME/.pico-sdk/sdk/2.2.0" \
    "$HOME/pico-sdk" \
    "$HOME/pico/pico-sdk"; do
    if [[ "$(sdk_version "$candidate" 2>/dev/null || true)" == "2.2.0" ]]; then
      SDK="$candidate"
      break
    fi
  done
  if [[ -z "$SDK" ]]; then
    echo "Pico SDK 2.2.0 was not found in the usual locations." >&2
    echo "Your existing ~/pico/pico-sdk may be a newer incompatible SDK." >&2
    echo "Install a separate compatible checkout (keeps the original untouched):" >&2
    echo '  git clone --branch 2.2.0 --depth 1 --recurse-submodules https://github.com/raspberrypi/pico-sdk.git "$HOME/pico/pico-sdk-2.2.0"' >&2
    echo '  PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" ./build_uf2.sh' >&2
    exit 4
  fi
fi

VERSION="$(sdk_version "$SDK" 2>/dev/null || true)"
if [[ "$VERSION" != "2.2.0" ]]; then
  echo "Incompatible Pico SDK: $SDK (version ${VERSION:-unknown})." >&2
  echo "This micro T-Kernel Wi-Fi/CYW43 port is written for Pico SDK 2.2.0." >&2
  echo "Do not edit the SDK's hardware/clocks.h or add random header paths." >&2
  echo 'Use a separate checkout: git clone --branch 2.2.0 --depth 1 --recurse-submodules https://github.com/raspberrypi/pico-sdk.git "$HOME/pico/pico-sdk-2.2.0"' >&2
  echo 'Then: PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" ./build_uf2.sh' >&2
  exit 4
fi

if [[ ! -f "$SDK/lib/cyw43-driver/src/cyw43_ctrl.c" || ! -f "$SDK/lib/lwip/src/core/init.c" ]]; then
  echo "Pico SDK 2.2.0 has missing CYW43 or lwIP submodules: $SDK" >&2
  echo "Run: git -C '$SDK' submodule update --init --recursive" >&2
  exit 4
fi

echo "Using Pico SDK $VERSION from $SDK"
if [[ ! -f "$ROOT/config/wifi_credentials.h" ]] || \
   [[ ! -f "$ROOT/config/wifi_secrets.h" && \
      ! -f "$ROOT/app_program/modules/telemetry/buddy1_broker_local.h" ]]; then
  echo "Local Wi-Fi/broker config missing. Run this first:" >&2
  echo "  cd '$ROOT/app_program/dashboard' && npm run setup:wifi" >&2
  echo "Then rerun ./build_uf2.sh from the project root." >&2
  exit 2
fi

# Earlier versions of the setup wizard generated a header rejected by BARR-C.
# The repair never prints credentials; it preserves a private local backup.
if [[ -f "$ROOT/app_program/modules/telemetry/buddy1_broker_local.h" ]]; then
  python3 "$ROOT/tools/repair_local_broker.py"
fi
python3 "$ROOT/tools/check_app_standard.py"
if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  echo "ARM cross-compiler not installed. Install arm-none-eabi-gcc first." >&2
  exit 3
fi

CONSOLE_MODE="${G7_CONSOLE:-uart}"
LIVE_CONTROL="${G7_LIVE_CONTROL:-0}"
BARCODE_TEST="${G7_BARCODE_TEST:-0}"
if [[ "$BARCODE_TEST" != "0" && "$BARCODE_TEST" != "1" ]]; then
  echo "G7_BARCODE_TEST must be 0 or 1" >&2; exit 2
fi
if [[ "$BARCODE_TEST" == "1" ]]; then
  echo 'BARCODE TEST PROFILE: motors and obstacle detection OFF.'
  echo 'Push the robot by hand; every pass prints a raw waveform dump.'
fi
if [[ "$LIVE_CONTROL" != "0" && "$LIVE_CONTROL" != "1" ]]; then
  echo "G7_LIVE_CONTROL must be 0 or 1" >&2; exit 2
fi
if [[ "$LIVE_CONTROL" == "1" ]]; then
  echo 'CONTROLLED BENCH PROFILE: boots STOPPED. Start only from MQTT.'
  echo 'Network loss triggers a software STOP. Keep physical power stop ready.'
fi
if [[ "$CONSOLE_MODE" != "uart" && "$CONSOLE_MODE" != "usb_cdc" ]]; then
  echo "G7_CONSOLE must be uart or usb_cdc." >&2; exit 2
fi
# Pre-create dependency output directories. Some macOS builds can fail to
# open -MF paths when USB and TinyUSB compilation start concurrently.
mkdir -p "$ROOT/build_make/mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb" \
         "$ROOT/build_make/mtkernel_3/lib/libnet/lwip" \
         "$ROOT/build_make/mtkernel_3/tinyusb" \
         "$ROOT/out"
# Legacy subdir.mk rules sometimes trust unpacked file timestamps instead of
# actual source contents. Refuse to reuse ARM objects built from old code,
# including USB diagnostics, CYW43 network logic, or changed private settings.
# V6.2: V6 removed build_make/mtkernel_3 and its subdir.mk recipes.
# Restore only missing make rules, and prepare the paths for .o/.d outputs.
python3 "$ROOT/tools/g7_restore_make_rules.py" "$ROOT"
python3 "$ROOT/tools/g7_build_cache_guard.py" prepare "$ROOT"
# Serial USB firmware has numerous nested dependency dirs. Build it
# sequentially on macOS to avoid an observed race in the legacy make rules.
JOBS=4
if [[ "$CONSOLE_MODE" == "usb_cdc" ]]; then JOBS=1; fi
cd "$ROOT/build_make"
make -f makefile \
  SMP=1 CONSOLE="$CONSOLE_MODE" \
  WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 \
  WIFI_MQTT=1 WIFI_TCP=0 B1_LIVE_COMMANDS="$LIVE_CONTROL" \
  B1_BUDDY5_MODULE=1 \
  B1_BARCODE_TEST="$BARCODE_TEST" \
  PICO_SDK_PATH="$SDK" -j"$JOBS"
UF2="$ROOT/build_make/mtk3pico_smp1_${CONSOLE_MODE}_wifi_dhcp.uf2"
if [[ ! -s "$UF2" ]]; then
  echo "Build finished without expected UF2 at: $UF2" >&2
  exit 5
fi
# Verify this exact source revision reached the final flashable image.
# This prevents a stale UF2 from being silently copied after a cached build.
# The marker is inside the USB owner's static self-test string.
if [[ "$CONSOLE_MODE" == "usb_cdc" ]]; then
  if ! LC_ALL=C grep -aFq 'G7DIAG-V7-9-RECONNECT-20261102' "${UF2%.uf2}.elf"; then
    echo 'ERROR: linked firmware ELF lacks G7DIAG-V7-9-RECONNECT-20261102.' >&2
    echo 'The make cache may have reused stale USB objects. Do NOT flash this UF2.' >&2
    echo 'Remove build_make/mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/usb_console.o and rebuild.' >&2
    exit 6
  fi
  echo 'PASS: verified G7DIAG-V7-9-RECONNECT-20261102 in linked ELF used to produce UF2.'
fi
# Persist the build-input fingerprint only after the ELF marker check passes.
# If compilation or verification fails, the next attempt cannot trust cache.
python3 "$ROOT/tools/g7_build_cache_guard.py" commit "$ROOT"
mkdir -p "$ROOT/out"
# G7 V6: A firmware with MQTT compiled but never polled cannot connect.
# Verify the poll routine is reachable in the LINKED image, not merely .o.
if [[ -n "${G7_SKIP_MQTT_LINK_CHECK:-}" ]]; then
  echo 'NOTICE: MQTT linkage assertion skipped for development.' >&2
elif command -v arm-none-eabi-nm >/dev/null 2>&1; then
  if ! arm-none-eabi-nm "${UF2%.uf2}.elf" | grep -E '[[:space:]][Tt][[:space:]]mqtt_utk_poll$' >/dev/null; then
    echo 'ERROR: MQTT owner poll was not linked. Do NOT flash this UF2.' >&2
    echo 'Inspect the CYW43 task guard, then rebuild.' >&2
    exit 7
  fi
  echo 'PASS: mqtt_utk_poll linked into active CYW43 firmware.'
else
  echo 'WARN: arm-none-eabi-nm not installed; cannot assert MQTT linkage.' >&2
fi
OUT_NAME="g7-improve-movement-mqtt.uf2"
if [[ "$BARCODE_TEST" == "1" ]]; then OUT_NAME="g7-barcode-test.uf2"; fi
cp "$UF2" "$ROOT/out/$OUT_NAME"
echo "Created: $ROOT/out/$OUT_NAME"
echo "Console: $CONSOLE_MODE; barcode result ACK owned by telemetry."
echo "WiFi join deadline: 10 seconds; motion/IR starts independently."
if [[ "$BARCODE_TEST" == "1" ]]; then
  echo "BARCODE TEST IMAGE: motors and obstacle detection compiled OUT."
  echo "Push the robot by hand; capture with tools/barcode_capture.py."
elif [[ "$LIVE_CONTROL" == "1" ]]; then
  echo "Obstacle detection and bypass enabled; ultrasonic GP4/GP5, servo GP15."
else
  echo "Obstacle detection and bypass enabled; ultrasonic GP4/GP5, servo GP15."
  echo "Live MQTT commands OFF: firmware may autonomously move."
fi
if [[ "$BARCODE_TEST" != "1" && "$LIVE_CONTROL" == "1" ]]; then
  echo "Live MQTT Start/Stop ON: boots held, requires mapping,"
  echo "fresh IR/IMU and broker. Link loss requests local STOP."
fi
echo "Do not share this UF2: Wi-Fi/MQTT secrets may be embedded in the image."
