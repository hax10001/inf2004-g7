# G7 V7.2 — Mapping 409 and separated consoles (dashboard-only)

## What caused HTTP 409 on `POST /api/command`?
`npm run lan` intentionally starts a **read-only** server. Even though it receives live binary MQTT telemetry, it does not allow physical mapping or Start/Stop commands. When you need controlled real commands, stop the read-only process and start this explicit bench profile **from your project root**:

```bash
bash run_live_dashboard.sh
```

This sets `LIVE_COMMANDS_ENABLED=1` and `DASHBOARD_SOURCE=robot` for the real robot. Only use it with wheels raised and a physical power cutoff. Normal `npm run lan` remains read-only on purpose. Restart the dashboard server after updating; hard-refresh the browser if it keeps old JavaScript.

The new mapping dialog displays whether live controls are enabled. A read-only 409 now tells you exactly how to start the controlled dashboard; failures no longer silently close the mapping dialog.

## Two deliberately different consoles
- **Live Sensor Telemetry:** continuous MQTT-decoded IR L/R/barcode sensor, IMU roll/pitch/heading/calibration, encoders, movement/terrain events, decoded barcode letters, map data. It does **not** require the Pico's USB serial port.
- **Command Monitor:** shows outgoing SET_MAPPING / START / STOP / SNAPSHOT requests, the matching robot ACK or rejection reason, timeouts, and high-level mission/network state changes. A request ledger displays pending, accepted and rejected transactions. Per-packet telemetry, duplicate-sequence noise, and dropped-frame noise don't flood the command view.

**Code 39** is the barcode encoding standard, not the number of letters. The four route mappings choose four distinct **A–Z** letters (26 options). Do not assume A/B/C/D correspond to the physical track unless those are the printed labels.

## How to use
1. Keep your working Pico firmware and hotspot/MQTT credentials. **No new UF2 build or flash is required for this dashboard-only update**.
2. Stop the old dashboard server (Ctrl+C), from project root run `bash run_live_dashboard.sh`.
3. Open `http://localhost:3000`, reload, confirm live sensor data in Live Sensor Telemetry.
4. Go to Mapping, choose the actual four track letters, click **Send mapping**, then wait for an **ACK ACCEPTED** in Command Monitor and **Mapping: LOADED** on status.
5. To start safely, on raised wheels click Start only when readiness is satisfied; or use **Send mapping, then Start (after ACK)**. This latter workflow sends START only after the mapping ACK, and still uses the controller safety checks.
6. If you use the browser remotely rather than `localhost`, enter the admin token as required; authorization errors return HTTP 401, not 409.

Do not confuse `MQTT command sent` with physical motor movement. A physical cutoff remains required.
