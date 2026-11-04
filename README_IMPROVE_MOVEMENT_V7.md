# G7 — Improve Movement + MQTT / Telemetry V7

Source of movement/IR/IMU: the original `inf2004-g7-improve-movement(2).zip` (motion/IR/IMU files verified byte-identical). The remote Start/Stop controller overlay is intentionally retained. Experimental Buddy 5 ultrasonic, servo, obstacle/bypass and associated reference code are REMOVED. MQTT binary V1 frame legacy obstacle fields remain present but always uninitialized (sentinel); no ultrasonic reading is invented.

## Start rejected (code 2)
The physical firmware defines `B1_BAD_STATE=2`. Most commonly the route's four distinct barcode-letter mapping has **not been sent since boot**. The status frame now reports `mappingLoaded`, `ready`, and `running`. Open Mapping, enter the actual track's four distinct A–Z letters, click **Send mapping**, await ACK, and then START. Or click **Send mapping, then Start (after ACK)**; START is sent only after valid mapping ACK and still goes through the real controller's safety gating. **Do not assume the UI's default A,B,C,D letters match your physical track.** Code 5 means IR/IMU/controller readiness failed; it is not safe to bypass this.

## Separate live telemetry (NO VS Code USB Serial Monitor)
Run `bash run_live_dashboard.sh`, open http://localhost:3000, choose **Telemetry Console**. Incoming SENSOR packets on `g7/b1/robot/telemetry` are decoded and logged as IR left/right/barcode, IMU roll/pitch/heading/calibration/hump phase and encoder counts. Barcode events and mapping-ready status also appear. These are MQTT telemetry, **not** raw USB printf output, which is still optional for deep firmware boot debug. Use the original USB monitor only if MQTT itself stops working.

## Restore current local credentials and build
Recommended: apply the V7 update ZIP to your working V6.2 tree; it deliberately preserves `config/wifi_secrets.h`, `config/wifi_credentials.h`, `app_program/modules/telemetry/buddy1_broker_local.h` and `app_program/dashboard/.lan-secrets.json`. For a fresh folder, run `bash setup_pookie.sh` (or `npm run setup:wifi` inside app_program/dashboard), then `python3 tools/sync_broker_ip.py --apply` on the same hotspot as the car.

Build and flash (wheels raised; motor power isolated):

```
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh
```

Expected UF2: `out/g7-improve-movement-mqtt.uf2`. Build still includes V6.1 verified diagnostic marker and MQTT linkage checks. Never assume an ACK proves motor motion. STOP is a network/software request, not a physical emergency stop.

## Deployment truth and known limits
Software tests alone do not demonstrate physical motor safety or WiFi reliability. The Code 39 decoder in the original improve-movement snapshot has pre-existing host fixture failures; no thresholds are modified here. Calibrated wheel odometry may remain unavailable unless the team's geometry flags are enabled. No obstacle sonar measurements exist in this build. Commands are authorized only in a controlled-profile firmware with a working coordinator; test with wheels off ground first.
