# V12 — scanner card / collision bench configuration (preservation checkpoint)

**Scope:** This updates the V11 precision-reviewed source. It does **not** implement or validate the downhill governor; use `DOWNHILL_RAMP_IMPLEMENTATION_PROMPT.md` for that work after physical IMU pitch-sign verification. **This is NOT complete Week 13 bit-width certification or an on-car acceptance release.**

## Changes

1. Moved the latest barcode spotlight and scanner diagnostic/counters **into the Barcode scan results card**, retaining their existing DOM IDs and real MQTT update paths. Added an explicit navigation-outcome summary and an **Edit barcode action mapping** button that invokes the existing mapping dialog. Removed the standalone three-card strip below the four consoles. Wheel and compass diagnostics remain in the collapsible Advanced Diagnostics area rather than being deleted. Reduced unified-pane row height to fit better; independent follow-to-latest behavior is unchanged.
2. Replaced raw `Scan not queued · mission · value=2` presentation with a meaningful explanation: **the decoded letter is absent from the active four-letter robot mapping, or mapping was not loaded**. This is a **mission acceptance** refusal, not a scanner decode failure or necessarily a full queue. The controller's refusal and motor safety are intentionally **unchanged**; the dashboard must never invent an action for an unmapped letter. Value 1 means navigation's state or pending single-use action policy refused the queue attempt.
3. Switched on the existing IMU task in the **normal live MQTT build** via `APP_IMU_ENABLED 1` (`app_program/app/app_config.h`), which activates existing stationary startup calibration, horizontal-jerk collision latch and controller STOP path. `B1_BARCODE_TEST` retains `APP_IMU_ENABLED 0` to avoid changing scanner-only motor-free capture timing. The existing 5,000 mg/s horizontal jerk threshold is unchanged and **uncalibrated for this robot**. A latched collision generally requires reset; no automatic motion restart is added. Do not use on the floor before bench safety tests.
4. No new bit-field state, no wire-format changes, no motor control changes, no changed MQTT command IDs/ACK/CRC, no changes to barcode recognition, no new MQTT subscriptions, and no changes to route/3D logic.

## Verified in this environment

- V11 regression shell suite, including application style, network boundary, C/JS golden vector, packed-field/precision assertions, obstacle/controller integration and 144 navigation assertions: **PASS**.
- Dashboard Node: **73/73 PASS** (70 existing + 3 new).
- Headless Chromium page: four equal desktop quadrants, barcode spotlight/counters *inside* Barcode card, old separate three-card strip absent, wheel/compass still under advanced details, navigation reason visible, no page errors, responsive phone view without horizontal overflow: **PASS**.
- ARM firmware build, target RAM and Pico W I2C/collision behavior: **NOT VERIFIED**. No hardware sensor/ramp calibration has been performed.

## Mac commands

Extract the full ZIP under `~/Downloads/inf2004-g7-v12-scanner-collision`, then:

```bash
cd ~/Downloads/inf2004-g7-v12-scanner-collision
bash tools/test_precision_ranges.sh
bash tools/run_v11_tests_mac.sh
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
  PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh
```

Flash the generated `out/g7-improve-movement-mqtt.uf2` **only after a successful ARM build**, keeping motor power disconnected while copying to BOOTSEL `/Volumes/RPI-RP2/`. Raise wheels and maintain a physical power cutoff for initial collision/STOP checks. Verify stationary IMU calibration, live roll/pitch and that an actual controlled impact triggers a STOP without auto-restarting. Use soft controlled stimuli, not damage. Compare V11/ V12 ARM RAM before claiming target savings.

## Security

Your diagnostic paste included a live broker password and dashboard admin token. **Rotate both** before sharing any further logs or operating on an untrusted network. This ZIP excludes private credentials/recordings. Preserve your locally configured secrets only in your own machine and never upload them to the project repository.
