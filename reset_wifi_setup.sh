#!/usr/bin/env bash
set -euo pipefail
umask 077
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
BACKUP="$ROOT/.g7-wifi-private-backups/$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p "$BACKUP"
for name in config/wifi_credentials.h config/wifi_secrets.h \
 app_program/modules/telemetry/buddy1_broker_local.h; do
  if [[ -f "$name" ]]; then
    mkdir -p "$BACKUP/$(dirname "$name")"
    mv "$name" "$BACKUP/$name"
    echo "Backed up: $name"
  fi
done
echo 'Your dashboard .lan-secrets.json is PRESERVED (MQTT credentials remain matched).'
echo "Private backup location: $BACKUP"
echo 'Run the wizard and enter SSID/password again. This changes files, NOT the Pico.'
cd "$ROOT/app_program/dashboard"
exec npm run setup:wifi
