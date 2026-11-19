#!/usr/bin/env bash
# One-time setup: checks Node, creates the Pico Wi-Fi/MQTT config if it is
# missing, and points the firmware's broker address at this Mac.
# Afterwards: G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 ./build_uf2.sh, flash
# out/g7-improve-movement-mqtt.uf2, then ./run_live_dashboard.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if ! command -v node >/dev/null 2>&1; then
  echo "Node.js 18 or newer is required: https://nodejs.org" >&2; exit 1
fi
major="$(node -p 'process.versions.node.split(".")[0]')"
if (( major < 18 )); then
  echo "Node.js $major found; version 18 or newer is required." >&2; exit 1
fi
if [[ ! -f "$ROOT/config/wifi_credentials.h" || ! -f "$ROOT/config/wifi_secrets.h" ]]; then
  echo "No Pico Wi-Fi config yet: starting the setup wizard."
  (cd "$ROOT/app_program/dashboard" && npm run setup:wifi)
else
  echo "Pico Wi-Fi config found (use ./reset_wifi_setup.sh to change it)."
  python3 "$ROOT/tools/sync_broker_ip.py" --apply
fi
echo "Next: G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 ./build_uf2.sh, flash out/g7-improve-movement-mqtt.uf2, ./run_live_dashboard.sh"
