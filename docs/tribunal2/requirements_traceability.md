# G7 requirements traceability (Tribunal 2, 23 Nov)

Requirement IDs and wording: `g7_pretribunal_requirements_proposed.md`.

How to run the tests:

- Host tests: `bash tools/run_host_tests.sh`. 162 PASS lines, ending `HOST PASS`. It also runs the dashboard suite.
- Dashboard tests: `cd app_program/dashboard && npm test`. 107/107 pass.
- Firmware build: `G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" ./build_uf2.sh`. This also runs `tools/check_app_standard.py`.

Videos go in `docs/videos/` and are named `<ID>_<snake_case>.mp4`.

New in this pass (marked **new** in the table):

- `tools/tests/test_requirements_trace.c` covers FR1, FR2, FR13 and NFR13 on the real `buddy1.c` and `buddy1_protocol.c`.
- `tools/tests/test_line_search.c` covers the FR4 lost-line search on the real `motion.c`.
- `tools/test_no_heap.py` covers NFR7: no heap calls in application code.
- `app_program/modules/terrain/host/test_calibration.c` covers FR3. It existed but was not run. It now runs in the suite, using a new `tools/tests/encoder_read_stub.c`.
- `tools/test_requirement_constants.py` checks 95 numeric values in the requirement table against the `#define` that implements each one. It fails if the code and the wording drift apart.

Type is H (host test), D (dashboard test) or V (on-car video).

| ID | Short text | Test(s) | Type | Status |
|---|---|---|---|---|
| FR1 | START refused (NOT_READY) on any blocker | `test_controller_handoff.c` (START gating), `test_tel.c` start_requires_mapping, **new** `test_requirements_trace.c` (START NOT_READY), `start-readiness.test.js`, `map-then-start.test.js`, **new** constants (200 ms IR age); video FR01_mqtt_connect_hold_calibrate.mp4 | H, D, V | Covered (filmed); refused START covered by host/dashboard tests |
| FR2 | SET_MAPPING: 4 distinct A–Z letters, S/L/R/U order, refused while running | `test_tel.c` protocol, **new** `test_requirements_trace.c` fr2_mapping, `protocol.test.js` (mapping packs 20 bits), `route-map-live.test.js`; video FR06_barcode_junction_turn_map.mp4 | H, D, V | Covered (filmed) |
| FR3 | Boot calibration: IMU 50 samples at 800–1200 mg, servo FRONT 1540 µs, barcode white from 250 samples | **new in suite** `terrain/host/test_calibration.c` (real `task_imu.c`: 50 samples, read failure, movement, gravity magnitude, persistent failure), `test_scanner_ready.c` (white learning), `test_tel.c` IMU guard, **new** constants; video FR01_mqtt_connect_hold_calibrate.mp4 | H, V | Covered (filmed) |
| FR4 | 18 mm line at 30 RPM, PID observer, gentle steer after 50 mm, search after 100 mm | `tests/line_follow_sim.c` (30/35/55 RPM), **new** `tests/test_line_search.c`, `test_autonav.sh`, **new** constants; videos FR04_line_follow_pid.mp4, FR16_line_loss_restart.mp4 (line-lost search) | H, V | Covered (filmed); line-lost search (drive on, pivot to last side, back, stop, resume) covered by **new** `tests/test_line_search.c` |
| FR5 | Code 39 both directions on GP27; dark runs under 1000 units dropped; EVENT plus hand-off | `test_scanner_ready.c`, `test_scan_handoff.c`, `tests/test_barcode_port.c`, `tests/barcode_layout.c`, `barcode-scanner.test.js`, `scanner-alerts.test.js`, `barcode-serial.test.js`, **new** constants; video FR05_barcode_decode.mp4 | H, D, V | Covered (filmed) |
| FR6 | Junction: 7 mm both black, 120 counts after barcode, 134 mm advance, 8 s / 1.5 s decode wait | `test_autonav.sh` (227 assertions), `test_controller_handoff.c` (junction actions, decode wait), **new** constants; video FR06_barcode_junction_turn_map.mp4 | H, V | Covered (filmed); U-turn covered by `test_autonav.sh` and `route-tracker.test.js`; unmapped/failed barcode by `test_controller_handoff.c` and `test_autonav.sh` |
| FR7 | Per-wheel PI (Kp 0.1, Ki 0.5), 8 RPM ramp, 1056 counts/rev, 70 mm wheel, 200 ms brake | `tests/obstacle_motion.c` (V15 plant: production PI), `test_ramp_motion.c`, **new** constants; video FR04_line_follow_pid.mp4 | H, V | Covered (filmed) |
| FR8 | Hump: 3°/3 samples start, 1.5°/15 samples end, height 0–127 mm, 5 mm climb and 40 mm travel, lift discard | `tests/test_hump_climb.c`, `tests/test_telemetry_map.c` (hump EVENT, discarded humps), `sensor-detail.test.js` (hump peak), **new** constants; video FR08_hump_detect_height.mp4 | H, D, V | Covered (filmed) |
| FR9 | Obstacle confirmed after 3 median-of-3 readings under 250 mm | `tests/obstacle_watch.c`, `test_obstacle_integration.py` ("front monitor triggers below 250 mm"), **new** constants; video FR09_obstacle_stop_bypass_rejoin.mp4 | H, V | Covered (filmed) |
| FR10 | Scan 60° L to 70° R, 5°/10° steps, edge vote, 40 mm margin, 245 mm minimum, 15 mm tie, clear-path resume | `tests/scan_planner.c`, `tests/obstacle_watch.c`, `run_planner_side_tracking.py`, `sensor-detail.test.js` (scan width), **new** constants; video FR10_obstacle_scan_width.mp4 | H, D, V | Covered (filmed). **Gap:** resume-when-the-way-clears (`OBS_RESUME_*`, 3 echoes beyond 400 mm) is only traced by the constants test, not behaviour-tested. A host test belongs in the obstacle tests, which were left alone during the pending scan-offset work |
| FR11 | Bypass: 5 absent echoes (600 mm), return 60°, drive to black, verify 10 periods, 4 search legs, rescans | `tests/obstacle_return.c`, `tests/side_tracking.c`, `tests/obstacle_rejoin_sim.c`, `tests/obstacle_motion.c`, **new** constants; video FR09_obstacle_stop_bypass_rejoin.mp4 | H, V | Covered (filmed) |
| FR12 | Binary MQTT telemetry: STATUS 1 s, SNAPSHOT 250 ms, SENSOR, HEARTBEAT 2 s, ODOM, 500 mm CHECKPOINT, EVENTs | `test_tel.c` aggregation, `tests/test_telemetry_map.c`, `test_minbits_wire_format.py`, `protocol.test.js`, `sensor-detail.test.js`, `e2e.test.js`, `mqtt.integration.test.js`, **new** constants; videos FR01_mqtt_connect_hold_calibrate.mp4, FR14_live_route_map.mp4 | H, D, V | Covered (filmed) |
| FR13 | Only 6 commands; each ACKed with a result; STATUS echoes last transaction; no remote steering | `test_tel.c`, **new** `test_requirements_trace.c` fr13_commands, `commands.test.js`, `barcode-scanner.test.js` (RECALIBRATE_BARCODE = 6), `reconnect.test.js`, `control-mode.test.js`; video FR01_mqtt_connect_hold_calibrate.mp4 | H, D, V | Covered (filmed) |
| FR14 | Route dead-reckoned from ODOM (130 mm track), 500 mm checkpoints, turn heading correction, markers | `route-tracker.test.js` (11 tests), `route-map-live.test.js`, `tests/test_telemetry_map.c`, **new** constants; videos FR14_live_route_map.mp4, FR06_barcode_junction_turn_map.mp4 | H, D, V | Covered (filmed) |
| FR15 | MQTT loss stops in next 50 ms period, back to REMOTE_HOLD; 15 s / 5 s link detection; no auto-resume; 32/24 rings | `test_controller_handoff.c` (MQTT loss), `tests/test_telemetry_map.c` (offline ODOM ring, overflow counted), `reconnect.test.js` (never auto-START), `firmware-wiring.test.js`, **new** constants (15 s, 5 s, 32, 24); video FR15_mqtt_loss_stop.mp4 | H, D, V | Covered (filmed) |
| FR16 | Critical fault forces PWM to zero; STOP then START recovery; hardware faults latch | `test_controller_handoff.c` (faults, restart gating), `tests/obstacle_motion.c` (zero-output STOP), `recovery-safety-stop.test.js`, `recovery-fault-sources.test.js`, `remote-hold.test.js`; video FR16_line_loss_restart.mp4 | H, D, V | Covered (filmed) |
| FR17 | Stop when lifted over 40 mm; resume after 1 s back on the floor | `tests/test_lift_math.c`, `test_controller_handoff.c` (lift stop latched and resume), `lift-height.test.js`, **new** constants; video FR17_lift_stop_resume.mp4 | H, D, V | Covered (filmed) |
| FR18 | Hump descent: RPM target to 65 % after 3 mm climb; 40 RPM cut; runaway stop; pitch trigger off | `tests/test_downhill_governor.c` (3 signs), `test_ramp_motion.c`, `tests/obstacle_motion.c` (terrain descent), `tests/test_hump_climb.c` (slow-down only on real humps), **new** constants; video FR08_hump_detect_height.mp4 (optional FR18_downhill_speed_cap.mp4) | H, V | Covered (filmed with FR08); separate clip optional |
| NFR1 | µT-Kernel 3.0 SMP, Pico SDK 2.2.0, style check before every build | `build_uf2.sh` (SDK check), `tools/check_app_standard.py`, `firmware-wiring.test.js` | H | Covered (build log + tests) |
| NFR2 | IR task priority 5 at 1 ms; 2 ms barcode ISR into a 64-entry queue; encoder ISR | `tests/test_ir_latches.c`, `test_scan_handoff.c`, `test_minbits_records_queue.py`, **new** constants | H | Covered (internal) |
| NFR3 | Controller priority 6 every 50 ms; 2 ms mutex wait; stop after 4 misses | `test_controller_handoff.c` (mutex-busy fail-safe, "Mailbox contention"), **new** constants | H | Covered (internal); no on-target timing trace |
| NFR4 | IMU priority 7 every 20 ms, alpha 0.2; 200 ms stale, 1 s lost stops | `test_controller_handoff.c` (stale IMU), `test_tel.c` IMU guard, **new** constants | H | Covered (internal) |
| NFR5 | Ultrasonic 30 ms timeout, 50 ms stuck ECHO, 60 ms spacing, median filter | `tests/obstacle_watch.c` ("pings >= 60 ms apart"), `test_obstacle_integration.py`, **new** constants | H | Covered (internal) |
| NFR6 | ADC threshold 2200, hysteresis 50, 600-count trigger rise | `test_scanner_ready.c` (capture gate), `tests/test_barcode_port.c`, **new** constants | H | Covered (internal) |
| NFR7 | No application malloc/free; boot-time stacks; fixed lwIP heap | **new** `tools/test_no_heap.py` (no malloc/calloc/realloc/free in 95 application files), `test_network_owner_boundary.py`, `test_minbits_records_queue.py` | H | Covered (internal) |
| NFR8 | Telemetry and barcode decoder at priority 9 | **new** constants (`B1_TASK_PRIORITY`, `BC_CAL_DIAG_PRIORITY`); recorded frame statistics in `requirements_audit.md` | H | Covered (internal) |
| NFR9 | Every stop path commands zero output; brake up to 200 ms | `tests/obstacle_motion.c` (immediate STOP, zero output), `test_controller_handoff.c` (STOP priority), `reconnect.test.js` (STOP sent on a fresh link), **new** constants | H, D | Covered |
| NFR10 | Encoder and barcode ISRs at priority 2, count/copy only | **new** constants (`ENC_INT_PRIORITY`, `BC_FAST_PRIORITY`), `test_scan_handoff.c` | H | Covered (internal) |
| NFR11 | Host tests and dashboard tests pass before release | `tools/run_host_tests.sh` (162 PASS), `npm test` (107/107) | H, D | Covered |
| NFR12 | Minimum-bit data; 103,594 B to 69,598 B; binary frames 12 % of JSON | `test_minbits_all.py`, `test_minbits_wire_format.py`, `test_minbits_records_queue.py`, `tests/test_minbits_saturation.c`, `test_precision_ranges.sh` | H | Covered (internal) |
| NFR13 | 13-byte LE header plus CRC-8 (0x07); bad CRC, length or reserved bits rejected | `test_tel.c` protocol (CRC, truncation), **new** `test_requirements_trace.c` nfr13_frame (independent CRC-8 reference, LE layout), `protocol.test.js`, `lift-height.test.js` (reserved bits) | H, D | Covered (internal) |


## On-car video shot list

Videos cover the functional requirements only. Sub-cases and every NFR are covered by host or dashboard tests in the table above.

### Filmed (all in `docs/evidence/videos/`)

| File | Covers |
|---|---|
| FR01_mqtt_connect_hold_calibrate.mp4 | FR1, FR3, FR12, FR13 |
| FR04_line_follow_pid.mp4 | FR4, FR7 |
| FR05_barcode_decode.mp4 | FR5 |
| FR06_barcode_junction_turn_map.mp4 | FR2, FR5, FR6, FR14 |
| FR08_hump_detect_height.mp4 | FR8 (FR18 optional) |
| FR09_obstacle_stop_bypass_rejoin.mp4 | FR9, FR10, FR11 |
| FR10_obstacle_scan_width.mp4 | FR10 |
| FR14_live_route_map.mp4 | FR12, FR14 |
| FR15_mqtt_loss_stop.mp4 | FR15 |
| FR16_line_loss_restart.mp4 | FR16, FR4 (line-lost search) |
| FR17_lift_stop_resume.mp4 | FR17 |

FR18 has no clip of its own. It is covered by host tests, and the hump clip (FR08) shows the descent. Every requirement is covered by a clip above or by tests.
