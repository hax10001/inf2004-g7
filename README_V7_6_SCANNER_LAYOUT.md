# G7 Improve Movement V7.6 — Barcode Results and Dashboard Layout

Dashboard-only update over the V7.5 sensor-verification source. No Pico reflash is required for the visual/event-handling changes; however **barcode diagnostic packets require V7.5-or-newer firmware actually running on the Pico**.

## Changes

- A real Code 39 EVENT carrying a value 0–25 generates a prominent `SCAN SUCCESS — decoded letter X` alert; this does not infer letters from IR edges.
- A positive change in the firmware barcode-diagnostic rejected counter generates `SCAN FAILED ×N`, including the reported failure code and decoded reason. If several failures occurred between packets, the last reported reason is not necessarily the reason for every failure.
- A completed-count change without a decoded-letter EVENT is marked `SCAN COMPLETE — letter event not received`; it is **not** assigned a fabricated letter.
- The scanner latest result and short history appear in Telemetry; Scanner result alerts also appear in Command Monitor, independently of the continuous IR/IMU stream. Overview shows the last result without duplicating the log.
- New sessions reset scan history. Failed scans do not clear the confirmed route mapping or control motors.
- Freshness bar now has four spaced status cells with age since the latest MQTT packet and last sensor sample. A sensor log remains visible as history when the connection is stale.
- Consistent panel padding, responsive three/two/one-column sensor cards, compact headers, legible console rows, and fully visible four-tab mobile navigation across Overview, Map, Telemetry, Monitor and dialogs.
- Fixes server projection of `lastMotionSeq`, so the existing Map & Robot view can display a route when **actual incremental ODOM packets** arrive. This does not invent odometry.

## Install

From the directory where you extracted the update ZIP:

```sh
cd ~/Downloads/g7-improve-movement-live-v7
python3 ~/Downloads/g7-v7-6-ui-scanner-patch/apply.py "$PWD"
cd app_program/dashboard
npm test
cd ../..
bash run_live_dashboard.sh
```

Then hard-refresh `http://localhost:3000`. If your active project has a different name, replace the `cd` path with the correct project root. Stop any old dashboard server first.

## Physical tests

1. Confirm actual `BARCODE_DIAG` messages by watching the scanner card's fresh counters; no USB serial reader is necessary.
2. Run a Code 39 barcode under the sensor with the required wheel motion. A letter only appears if the robot firmware emits its successful barcode EVENT. `Completed` should rise.
3. Make a deliberately invalid/partial scan. If the firmware increments its `rejected` counter, Telemetry and Command Monitor show the failure. Merely moving the sensor past black/white transitions does not guarantee a completed or rejected attempt.
4. Stop incoming telemetry: the status should become `STREAM PAUSED` with time since last packet, while previously received logs remain clearly historical.
5. If ODOM frames are missing, Map & Robot continues to state that the path is unavailable; raw sensor packets do not imply position tracking.

The underlying native Code 39 decoder failures reported in V7.5 remain unresolved. This update improves observation and navigation, not decoder accuracy or physical hardware quality. Keep your existing motor safety cutoff.

## Validation

- 46/46 Node dashboard tests passed.
- Browser-rendered 12 layouts at 1440, 900 and 390 CSS pixels, across all four navigation views: no page script errors and no horizontal viewport overflow in simulated layouts.
- No changes to C firmware, motor control, credentials, Wi-Fi, MQTT broker or safety gating.
- Real Pico scans and production user/browser configurations still need hardware testing.
