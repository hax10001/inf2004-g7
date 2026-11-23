#!/usr/bin/env bash
# Local live-only dashboard: physical Start/Stop require explicit opt-in.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LIVE_COMMANDS_ENABLED=1
export DASHBOARD_SOURCE=robot
cd "$ROOT/app_program/dashboard"
exec npm run lan
