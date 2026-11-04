# G7 V7.4 – MQTT + improve-movement dashboard handoff

## Scope

This is a **dashboard-only** cleanup built on the V7.3 improve-movement MQTT integration. No Pico firmware, motor control, radio settings, broker login, or hardware sensor code changes. The obstacle-detection module has **not** been integrated into this version.

## Interface

- **Overview:** robot connectivity, mapping, start-readiness, sensor-feed freshness, odometry metrics, and concise drive/line/barcode display.
- **Map & robot:** robot-acknowledged letter assignments, barcode detection order, odometry-estimated route, and illustrative 3D chassis. No SLAM or fabricated position.
- **Telemetry Console:** continuous MQTT IR/IMU/encoder/barcode readings. Expand **Advanced sensor diagnostics** for packet counters, fault bits, stale mask, and individual measurements.
- **Command Monitor:** mapping/START/STOP requests, robot ACK/rejection/timeout, and significant system events. Repeated no-op mission event-0 entries are hidden from the browser console only; recordings remain intact.

**A live MQTT heartbeat is not evidence of fresh sensor readings.** The Overview marks a connected robot separately from a stale or missing sensor feed. Do not treat an accepted START ACK as confirmation of physical movement or a remote STOP as an emergency stop.

## Run

From your existing project folder:

```sh
bash run_live_dashboard.sh
```

Then open http://localhost:3000. Read-only monitoring is also available through `cd app_program/dashboard && npm run lan`, but mapping/Start/Stop are disabled in read-only mode.

No new UF2 build or flash is necessary for V7.4. Keep the previously working flashed firmware.

## Validation

`cd app_program/dashboard && npm test` – 38/38 dashboard tests passed in the preparation environment. Browser layout has been reviewed structurally but not verified on the user's actual browser or Pico.

## Git handoff

**Suggested branch:** `feature/improve-movement-mqtt-integration`

**Commit subject:** `feat(inf2004): integrate improve-movement MQTT telemetry and clean mission control UI`

**PR title:** `Integrate improve-movement with MQTT telemetry and dashboard`

**PR description:**

- Connected the Pico W's improve-movement telemetry to a LAN MQTT broker and dashboard, with live packet/connection monitoring.
- Added barcode route mapping/ACK-before-START flow and separate live telemetry vs command/ACK views.
- Added a mapped-barcode detection list and odometry-only route estimates.
- Simplified Overview; moved detailed sensor counters behind an expandable Telemetry diagnostics panel.
- Clearly distinguish connected robot status from stale sensor measurements.

**Validation:** dashboard/unit/integration suite 38/38 passing. Hardware observation has confirmed Wi-Fi and broker packets, but obstacle detection is out of scope and motor actuation/physical safety have not been fully verified. Next work: integrate working obstacle-detection module against the existing telemetry and controller interfaces after testing.

**Important:** do not commit `app_program/dashboard/.lan-secrets.json`, `config/wifi_secrets.h`, `.env` files, flashed UF2s or generated build directories. Review `git status` before staging.
