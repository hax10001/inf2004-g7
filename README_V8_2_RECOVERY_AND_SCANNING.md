# V8.2 — Recoverable navigation stop, repeat barcode scans, and START/STOP handoff

**Baseline:** V8.1 autonomous navigation source. This is a **bench-only firmware candidate**; hardware motor operation, encoder polarity, line sensor geometry, and print-size accuracy must be physically verified. It is not evidence that the full ARM build succeeds or that a physical car is safe to run unattended.

## Symptom reproduced from field telemetry

- START and SET_MAPPING were ACK-accepted; PWM briefly went nonzero.
- A scanner result arrived as the robot entered `Mission state 2 -> 7`, `Navigation safety stop` and `PWM 0,0`.
- Later STOP ACKs did **not** clear the state: the V8.1 navigation controller mapped `NAV_FAULT` to the same `ROBOT_ERROR` state used for genuine controller/hardware failures. V8.1's STOP correctly refused to clear that undifferentiated error, but the result was an unrecoverable software lockout.
- Some scan completion counters arrived before decoded-letter events and were temporarily advertised as separate successes; scanner error code 0 was mislabeled "No error".

## V8.2 changes

1. **Recoverable navigation stop:** dedicated `ROBOT_NAV_SAFETY_STOP`, reported as mission code **5**. Explicit STOP applies zero motor command, clears only pending navigation state and returns to remote hold. START remains separate and still checks IR freshness, IMU calibration, mapping, MQTT, and controller readiness. Genuine `ROBOT_ERROR` (mission 7) stays latched; STOP never bypasses it. STOP is accepted even while navigation is stopped and takes priority over queued START.
2. **Barcode/crossing separation:** a failed decoder result does not itself call STOP. While the scanner reports an active/recent waveform or awaits white-background clearance, a both-black marking is crossed only within **400 ms and 180 encoder counts** from the first both-black reading. If either limit is exceeded, or a genuinely ambiguous junction remains, a *recoverable* safe stop occurs instead of unbounded driving.
3. **Repeatable scanner:** after decode, wait for **250 consecutive valid near-white analog samples** before learning another barcode baseline. The previous baseline is retained only for the white-clearance check, and newly seen black ink resets the count. A scan in progress or uncleared background participates in the bounded navigation-crossing guard. Scanner capture/decoding is still kept out of motor PWM ownership and the MQTT callback.
4. **Restore proven capture target:** barcode capture target returns to V7.7's **15 RPM**. V8.1's unverified 10 RPM scan slowdown is reversed. This is not proof that small labels now work: physical print size, optical footprint and barcode minimum encoder-unit thresholds remain limiting factors.
5. **Accurate scan telemetry:** an actual decoded-letter event, not merely a completed-counter increment, creates a success alert. Buffered rejection errors are correctly reported in both diagnostic and result payloads; unknown code 0 is marked unknown rather than "No error". Readiness messages now explicitly distinguish `NAVIGATION SAFETY STOP` (press STOP once, wait for READY, then START) from `CONTROLLER FAULT` (do not bypass).

## Source configuration — requires track calibration

- `APP_NAV_SCAN_CROSS_MAX_MSEC=400` milliseconds, `APP_NAV_SCAN_CROSS_MAX_COUNTS=180` combined-count estimate (NOT millimeters).
- `APP_BARCODE_SCAN_RPM=15.0f` (nominal target, not measured wheel speed).
- `BC_WAVE_WHITE_SAMPLES=250` at the existing 2-ms ADC waveform sampling cadence (~500 ms with clean samples).
- `g_scan_guard_until_usec=freeze+600000` is a short recency guard, in addition to the `BC_WAVE_CLEAR` phase. Neither permits an unlimited crossing because navigation independently enforces distance and time limits.

**No unconditional forward bypass:** Without a pending decoded route action, a sustained both-black that is NOT a bounded scanner stripe remains an ambiguous intersection/safety stop. With just two line sensors, code cannot reliably distinguish every painted junction from a barcode or lifted sensor. Changing this requires measured sensor traces or additional hardware, not removing the safety limits.

## Install and test on Mac

```bash
cd ~/Downloads/g7-improve-movement-live-v7
unzip -o ~/Downloads/G7_V8_2_Recovery_Patch.zip -d ~/Downloads
python3 ~/Downloads/g7-v8-2-recovery/apply.py "$PWD"

bash tools/test_autonav.sh
bash tools/test_controller_remote.sh
bash app_program/modules/telemetry/run_native.sh
cd app_program/dashboard && node --test --test-concurrency=1 tests/*.test.js && cd ../..
python3 tools/check_app_standard.py

G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
  PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh
```

Only flash the freshly linked `out/g7-improve-movement-mqtt.uf2` if the ARM build reports success. Disconnect motor power while flashing. The installer guards *all* updated source hashes and makes timestamped backups under `.g7_v8_2_recovery_backup/`.

## Physical test checklist (wheels supported, reachable hardware power cutoff)

1. Send a confirmed mapping, then START once. Confirm running state and nonzero *commanded* PWM, then verify physically that the wheels turn.
2. Press STOP **during normal following**. Verify the robot reaches HOLD, PWM goes to 0/0 and wheels physically stop. Repeat STOP: it should be harmless.
3. Test an invalid printed symbol. Confirm a **single reported SCAN FAILED** and continued line following if the line sensors remain safe. If the robot instead reports `NAVIGATION SAFETY STOP`, send STOP once, wait for READY, then START once. Read the junction/fault detail rather than repeatedly pressing START.
4. Pass the same valid Code 39 barcode repeatedly, with white background between attempts. Confirm exactly one success event per actual scan and successful rearming; avoid holding the barcode stationary under the scanner.
5. Simulate an ambiguous prolonged both-black intersection: the robot must STOP, report mission 5, and remain stopped until explicit STOP and later START. Simulate a real driver/controller fault: it must report mission 7 and **not** become READY from STOP.
6. Test STOP during left/right/U-turn at supported wheels-raised speeds. Confirm it cancels the maneuver and physically sets motor output to zero. Do not run unattended or on the floor until steering directions, reverse polarity, and timeouts are calibrated.
7. Disconnect/reconnect Wi-Fi during a run; verify no unexpected automatic restart. After reconnect and fresh STATUS, manual STOP → READY → manual START should be possible only when safety conditions are met.

## Caveats and rollback

- Software-only test fixtures cannot verify printed small barcodes, actual PWM applied to motor terminals, wheel response, calibrated turns or real junction geometry. Raw ADC edge widths, measured speed, and encoder displacement must be recorded before tuning further.
- Preserves the contributed scanner's Code 39 validity thresholds rather than manufacturing decode success.
- If the guard rejects an altered local firmware file, **do not bypass it**. The patch does not modify the user's Mac until manually installed.
- Rollback: cut motor power; stop the dashboard; restore the files from the timestamped `.g7_v8_2_recovery_backup/` directory (and remove newly introduced files not present in V8.1), then rebuild and reflash the previous known-good UF2. Back up any desired diagnostics first.
