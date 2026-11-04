#!/usr/bin/env bash
# Configure the Pookie network locally. Password stays in private, ignored
# headers created by the interactive wizard; this ZIP contains no password.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT/app_program/dashboard"
G7_DEFAULT_WIFI_SSID=Pookie node tools/setup-wifi.js
