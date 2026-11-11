#!/usr/bin/env bash
# One-command local ROBOT SIMULATION. Does not flash firmware or touch a real car.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DASH="$ROOT/app_program/dashboard"
if ! command -v node >/dev/null 2>&1; then
  echo "Node.js 18+ is required. Install Node.js, then run ./demo.sh again." >&2
  exit 1
fi
if ! command -v curl >/dev/null 2>&1; then
  echo "curl is required to check that the dashboard started." >&2
  exit 1
fi
if [ ! -f "$DASH/server/index.js" ]; then
  echo "Dashboard not found at $DASH; run this from the extracted project." >&2
  exit 1
fi
# Never attach to an unknown broker/server; don't stop another process.
for PORT in 3000 1883; do
  if (echo > "/dev/tcp/127.0.0.1/$PORT") >/dev/null 2>&1; then
    echo "Port $PORT is already in use. Stop the other dashboard/broker first." >&2
    exit 1
  fi
done
cd "$DASH"
export WEB_BIND=127.0.0.1 BROKER_BIND=127.0.0.1 DASHBOARD_SOURCE=sim BROKER_EXTERNAL=0
export WEB_PORT=3000 BROKER_PORT=1883 LIVE_COMMANDS_ENABLED=0 LIVE_IMU_RECAL_ENABLED=0
# A demo must remain local even if the user has previously configured LAN mode.
WEB_PID='' SIM_PID=''
cleanup() {
  trap - EXIT INT TERM
  [ -z "$SIM_PID" ] || kill "$SIM_PID" 2>/dev/null || true
  [ -z "$WEB_PID" ] || kill "$WEB_PID" 2>/dev/null || true
  [ -z "$SIM_PID" ] || wait "$SIM_PID" 2>/dev/null || true
  [ -z "$WEB_PID" ] || wait "$WEB_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
node server/index.js & WEB_PID=$!
READY=0
for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
  if curl -fsS --max-time 1 http://127.0.0.1:3000/api/state >/dev/null 2>&1; then READY=1;break;fi
  if ! kill -0 "$WEB_PID" 2>/dev/null;then break;fi
  sleep .25
done
if [ "$READY" -ne 1 ]; then echo "Could not start local dashboard. Check messages above." >&2;exit 1;fi
node tools/demo.js --auto-start --fast & SIM_PID=$!
sleep .4
if ! kill -0 "$SIM_PID" 2>/dev/null; then echo "Simulator did not start." >&2;exit 1;fi
printf '\n=============================================\n'
printf ' G7 BUDDY 5 / ROBOT MOTION DEMO (SIMULATED)\n'
printf '=============================================\n'
printf ' Dashboard: http://127.0.0.1:3000\n'
printf ' The car will move automatically in the demo.\n'
printf ' Choose Map & robot to view the route and 3D wheels.\n'
printf ' Choose Monitor to see messages, START / STOP, and export.\n'
printf ' Synthetic data only. Not connected to real hardware.\n'
printf ' Press Ctrl+C to stop both processes.\n\n'
if [ "$(uname -s)" = Darwin ] && command -v open >/dev/null 2>&1; then
  open 'http://127.0.0.1:3000' >/dev/null 2>&1 || true
fi
wait "$SIM_PID"
