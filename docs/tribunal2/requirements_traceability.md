# G7 requirements traceability (Tribunal 2)

Requirement IDs and wording: [`g7_pretribunal_requirements_proposed.md`](g7_pretribunal_requirements_proposed.md).
Demo videos are in [`docs/evidence/videos/`](../evidence/videos/).

## How to run

| Suite | Command | Result (25 Nov) |
|---|---|---|
| Host tests (also runs the style check and the dashboard suite) | `bash tools/run_host_tests.sh` | 369 PASS lines, ends `HOST PASS` |
| Dashboard tests | `cd app_program/dashboard && npm test` | 110 / 110 |
| Firmware build (runs the style check first) | `G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" ./build_uf2.sh` | builds, 0 style errors |

Host tests compile the real firmware sources (`app_program/...`, `lib/libnet/lwip/mqtt_utk.c`) against small stubs for the RTOS, the hardware and lwIP. Test files are in `tools/` and `tools/tests/`; each file's header names the requirements it covers.

## Requirement → tests → video

| ID | Requirement (short) | Host and dashboard tests | Video | Status |
|---|---|---|---|---|
| FR1 | START refused (NOT_READY) on any blocker | `test_controller_handoff.c` (START gating), `test_tel.c` start_requires_mapping, `test_requirements_trace.c` (START NOT_READY), `start-readiness.test.js`, `map-then-start.test.js`, `test_requirement_constants.py` (200 ms IR age), `tests/test_ct_controller_ready.c` (each START blocker bit alone, combinations, ready mask) | FR01_mqtt_connect_hold_calibrate.mp4 | Covered (host tests + video) |
| FR2 | SET_MAPPING: 4 distinct A–Z letters, S/L/R/U order, refused while running | `test_tel.c` protocol, `test_requirements_trace.c` fr2_mapping, `protocol.test.js` (mapping packs 20 bits), `route-map-live.test.js`, `tests/test_ct_protocol_codec.c` (mapping packing, reserved bits, duplicates) | FR06_barcode_junction_turn_map.mp4 | Covered (host tests + video) |
| FR3 | Boot calibration: IMU 50 samples at 800–1200 mg, servo FRONT 1540 µs, barcode white from 250 samples | `terrain/host/test_calibration.c` (real `task_imu.c`: 50 samples, read failure, movement, gravity magnitude, persistent failure), `test_scanner_ready.c` (white learning), `test_tel.c` IMU guard, `test_requirement_constants.py` | FR01_mqtt_connect_hold_calibrate.mp4 | Covered (host tests + video) |
| FR4 | 18 mm line at 30 RPM, PID observer, gentle steer after 50 mm, search after 100 mm | `tests/line_follow_sim.c` (30/35/55 RPM), `tests/test_line_search.c`, `test_autonav.sh`, `test_requirement_constants.py`, `tests/test_mt_steering.c` (PI/observer maths, clamps, hold and bend memory, line_follow_decide) | FR04_line_follow_pid.mp4, FR16_line_loss_restart.mp4 | Covered (host tests + video) |
| FR5 | Code 39 both directions on GP27; dark runs under 1000 units dropped; EVENT plus hand-off | `test_scanner_ready.c`, `test_scan_handoff.c`, `tests/test_barcode_port.c`, `tests/barcode_layout.c`, `barcode-scanner.test.js`, `scanner-alerts.test.js`, `barcode-serial.test.js`, `test_requirement_constants.py`, `tests/test_bc_code.c`, `tests/test_bc_scan.c`, `tests/test_bc_layout.c` (incl. the decision margin rule), `tests/test_bc_match.c` (A-Z both directions, bad and unreadable codes) | FR05_barcode_decode.mp4 | Covered (host tests + video) |
| FR6 | Junction: 7 mm both black, 120 counts after barcode, 134 mm advance, 8 s / 1.5 s decode wait | `test_autonav.sh` (227 assertions), `test_controller_handoff.c` (junction actions, decode wait), `test_requirement_constants.py` | FR06_barcode_junction_turn_map.mp4 | Covered (host tests + video) |
| FR7 | Per-wheel PI (Kp 0.1, Ki 0.5), 8 RPM ramp, 1056 counts/rev, 70 mm wheel, 200 ms brake | `tests/obstacle_motion.c` (V15 plant: production PI), `test_ramp_motion.c`, `test_requirement_constants.py`, `tests/test_mt_wheel_speed.c` (edge timing, timeout, wraparound) | FR04_line_follow_pid.mp4 | Covered (host tests + video) |
| FR8 | Hump: 3°/3 samples start, 1.5°/15 samples end, height 0–127 mm, 40 mm travel (no minimum climb since 24 Nov), lift discard | `tests/test_hump_climb.c`, `tests/test_telemetry_map.c` (hump EVENT, discarded humps), `sensor-detail.test.js` (hump peak), `test_requirement_constants.py`, `tests/test_mt_imu.c` (hump height bias, discard rules), `tests/test_mt_terrain.c` (tilt/heading maths, calibration) | FR08_hump_detect_height.mp4 | Covered (host tests + video) |
| FR9 | Obstacle confirmed after 3 median-of-3 readings under 250 mm | `tests/obstacle_watch.c`, `test_obstacle_integration.py` ("front monitor triggers below 250 mm"), `test_requirement_constants.py`, `tests/test_obs_state.c` (packed status word) | FR09_obstacle_stop_bypass_rejoin.mp4 | Covered (host tests + video) |
| FR10 | Scan 60° L to 70° R, 5°/10° steps, edge vote, 40 mm margin, 245 mm minimum, 15 mm tie, clear-path resume | `tests/scan_planner.c`, `tests/obstacle_watch.c`, `run_planner_side_tracking.py`, `sensor-detail.test.js` (scan width), `test_requirement_constants.py`, `tests/test_obs_resume.c` (resume when the path clears), `tests/test_obs_bypass.c` (reported width = measured + offset) | FR10_obstacle_scan_width.mp4 | Covered (host tests + video) |
| FR11 | Bypass: 5 absent echoes (600 mm), return 60°, drive to black, verify 10 periods, 4 search legs, rescans | `tests/obstacle_return.c`, `tests/side_tracking.c`, `tests/obstacle_rejoin_sim.c`, `tests/obstacle_motion.c`, `test_requirement_constants.py`, `tests/test_obs_bypass.c` (return turn at OBS_RETURN_DEG both sides, rejoin) | FR09_obstacle_stop_bypass_rejoin.mp4 | Covered (host tests + video) |
| FR12 | Binary MQTT telemetry: STATUS 1 s, SNAPSHOT 250 ms, SENSOR, HEARTBEAT 2 s, ODOM, 500 mm CHECKPOINT, EVENTs | `test_tel.c` aggregation, `tests/test_telemetry_map.c`, `test_minbits_wire_format.py`, `protocol.test.js`, `sensor-detail.test.js`, `e2e.test.js`, `mqtt.integration.test.js`, `test_requirement_constants.py`, `tests/test_ct_telemetry_poll.c` (ODOM, checkpoints, sentinels, SENSOR/BARCODE_DIAG frames), `robot-log-export.test.js` | FR01_mqtt_connect_hold_calibrate.mp4, FR14_live_route_map.mp4 | Covered (host tests + video) |
| FR13 | Only 6 commands; each ACKed with a result; STATUS echoes last transaction; no remote steering | `test_tel.c`, `test_requirements_trace.c` fr13_commands, `commands.test.js`, `barcode-scanner.test.js` (RECALIBRATE_BARCODE = 6), `reconnect.test.js`, `control-mode.test.js`, `tests/test_ct_protocol_codec.c` (command codec, sessions, duplicates), `tests/test_ct_controller_ready.c` (command refusals) | FR01_mqtt_connect_hold_calibrate.mp4 | Covered (host tests + video) |
| FR14 | Route dead-reckoned from ODOM (130 mm track), 500 mm checkpoints, turn heading correction, markers | `route-tracker.test.js` (11 tests), `route-map-live.test.js`, `tests/test_telemetry_map.c`, `test_requirement_constants.py` | FR14_live_route_map.mp4, FR06_barcode_junction_turn_map.mp4 | Covered (host tests + video) |
| FR15 | MQTT loss stops in next 50 ms period, back to REMOTE_HOLD; 15 s / 5 s link detection; no auto-resume; 32/24 rings | `test_controller_handoff.c` (MQTT loss), `tests/test_telemetry_map.c` (offline ODOM ring, overflow counted), `reconnect.test.js` (never auto-START), `firmware-wiring.test.js`, `test_requirement_constants.py` (15 s, 5 s, 32, 24), `tests/test_ct_mqtt_link.c` (real `mqtt_utk.c`: 5 s ping, drop at 15.001 s not 15 s, link-down, STOP priority, no START replay after reconnect) | FR15_mqtt_loss_stop.mp4 | Covered (host tests + video) |
| FR16 | Critical fault forces PWM to zero; STOP then START recovery; hardware faults latch | `test_controller_handoff.c` (faults, restart gating), `tests/obstacle_motion.c` (zero-output STOP), `recovery-safety-stop.test.js`, `recovery-fault-sources.test.js`, `remote-hold.test.js`, `tests/test_ct_controller_ready.c` (fault source per stop, reported once, cleared by STOP) | FR16_line_loss_restart.mp4 | Covered (host tests + video) |
| FR17 | Stop when lifted over 40 mm; resume after 1 s back on the floor | `tests/test_lift_math.c`, `test_controller_handoff.c` (lift stop latched and resume), `lift-height.test.js`, `test_requirement_constants.py`, `tests/test_mt_imu.c` (39 mm not lifted, 40 mm lifted, floor evidence, settle after landing) | FR17_lift_stop_resume.mp4 | Covered (host tests + video) |
| FR18 | Hump descent: RPM target to 65 % after 3 mm climb; 40 RPM cut; runaway stop; pitch trigger off | `tests/test_downhill_governor.c` (3 signs), `test_ramp_motion.c`, `tests/obstacle_motion.c` (terrain descent), `tests/test_hump_climb.c` (slow-down only on real humps), `test_requirement_constants.py`, `tests/test_mt_downhill.c` (40 RPM boundary, 10-period runaway latch, cap only on a real descent) | FR08_hump_detect_height.mp4 | Covered (host tests + video) |
| NFR1 | µT-Kernel 3.0 SMP, Pico SDK 2.2.0, style check before every build | `build_uf2.sh` (SDK check), `tools/check_app_standard.py`, `firmware-wiring.test.js` | — | Covered (build and style check) |
| NFR2 | IR task priority 5 at 1 ms; 2 ms barcode ISR into a 64-entry queue; encoder ISR | `tests/test_ir_latches.c`, `test_scan_handoff.c`, `test_minbits_records_queue.py`, `test_requirement_constants.py` | — | Covered (host tests) |
| NFR3 | Controller priority 6 every 50 ms; 2 ms mutex wait; stop after 4 misses | `test_controller_handoff.c` (mutex-busy fail-safe, "Mailbox contention"), `test_requirement_constants.py`, `tests/test_ct_controller_ready.c` (stale-IMU 1000 ms boundary) | — | Covered (host tests) |
| NFR4 | IMU priority 7 every 20 ms, alpha 0.2; 200 ms stale, 1 s lost stops | `test_controller_handoff.c` (stale IMU), `test_tel.c` IMU guard, `test_requirement_constants.py`, `tests/test_mt_imu.c`, `tests/test_ct_controller_ready.c` (IMU 200 ms freshness boundary) | — | Covered (host tests) |
| NFR5 | Ultrasonic 30 ms timeout, 50 ms stuck ECHO, 60 ms spacing, median filter | `tests/obstacle_watch.c` ("pings >= 60 ms apart"), `test_obstacle_integration.py`, `test_requirement_constants.py` | — | Covered (host tests) |
| NFR6 | ADC threshold 2200, hysteresis 50, 600-count trigger rise | `test_scanner_ready.c` (capture gate), `tests/test_barcode_port.c`, `test_requirement_constants.py` | — | Covered (host tests) |
| NFR7 | No application malloc/free; boot-time stacks; fixed lwIP heap | `tools/test_no_heap.py` (no malloc/calloc/realloc/free in 95 application files), `test_network_owner_boundary.py`, `test_minbits_records_queue.py` | — | Covered (host tests) |
| NFR8 | Telemetry and barcode decoder at priority 9 | `test_requirement_constants.py` (`B1_TASK_PRIORITY`, `BC_CAL_DIAG_PRIORITY`); recorded frame statistics in `requirements_audit.md` | — | Covered (host tests) |
| NFR9 | Every stop path commands zero output; brake up to 200 ms | `tests/obstacle_motion.c` (immediate STOP, zero output), `test_controller_handoff.c` (STOP priority), `reconnect.test.js` (STOP sent on a fresh link), `test_requirement_constants.py` | — | Covered (host/dashboard tests) |
| NFR10 | Encoder and barcode ISRs at priority 2, count/copy only | `test_requirement_constants.py` (`ENC_INT_PRIORITY`, `BC_FAST_PRIORITY`), `test_scan_handoff.c` | — | Covered (host tests) |
| NFR11 | Host tests and dashboard tests pass before release | `tools/run_host_tests.sh` (369 PASS), `npm test` (110/110) | — | Covered (host/dashboard tests) |
| NFR12 | Minimum-bit data; 103,594 B to 69,598 B; binary frames 12 % of JSON | `test_minbits_all.py`, `test_minbits_wire_format.py`, `test_minbits_records_queue.py`, `tests/test_minbits_saturation.c`, `test_precision_ranges.sh` | — | Covered (host tests) |
| NFR13 | 13-byte LE header plus CRC-8 (0x07); bad CRC, length or reserved bits rejected | `test_tel.c` protocol (CRC, truncation), `test_requirements_trace.c` nfr13_frame (independent CRC-8 reference, LE layout), `protocol.test.js`, `lift-height.test.js` (reserved bits), `tests/test_ct_protocol_codec.c` (bit-exact snapshot layout, every single-bit flip rejected) | — | Covered (host/dashboard tests) |

## Coverage

Line and branch coverage of the team's firmware sources over the whole host suite, measured with clang source-based coverage (`-fprofile-instr-generate -fcoverage-mapping`, `llvm-cov report`). The dashboard suite covers 91 % of the dashboard server's lines (`node --test --experimental-test-coverage`).

| Source | Lines before (%) | Lines now | Branches now |
|---|---:|---:|---:|
| `app/nav_core.c` | 95 | 95.03% | 89.57% |
| `app/task_controller.c` | 74 | 85.17% | 76.67% |
| `drivers/motor.c` | 60 | 60.00% | 45.24% |
| `modules/barcode_scanning/bc_code.c` | 0 (never run) | 100.00% | 96.77% |
| `modules/barcode_scanning/bc_fastq.c` | 68 | 67.61% | 50.00% |
| `modules/barcode_scanning/bc_layout.c` | not compiled | 98.02% | 90.53% |
| `modules/barcode_scanning/bc_match.c` | not compiled | 94.93% | 86.15% |
| `modules/barcode_scanning/bc_port.c` | 50 | 50.35% | 45.45% |
| `modules/barcode_scanning/bc_scan.c` | 34 | 97.77% | 97.83% |
| `modules/barcode_scanning/bc_wave.c` | 48 | 48.12% | 36.74% |
| `modules/line_barcode/ir_shared.c` | 67 | 67.02% | 60.29% |
| `modules/line_barcode/line_follow.c` | 30 | 100.00% | 100.00% |
| `modules/motion/downhill.c` | 96 | 100.00% | 90.48% |
| `modules/motion/motion.c` | 83 | 86.65% | 77.70% |
| `modules/motion/steering.c` | 40 | 98.48% | 95.24% |
| `modules/motion/wheel_speed.c` | 93 | 100.00% | 100.00% |
| `modules/obstacle/obs_state.c` | not run | 100.00% | 96.67% |
| `modules/obstacle/obstacle.c` | 66 | 74.58% | 70.54% |
| `modules/obstacle/scan_plan.c` | 90 | 91.00% | 78.95% |
| `modules/obstacle/task_obstacle.c` | 80 | 79.62% | 70.57% |
| `modules/telemetry/buddy1.c` | 88 | 96.72% | 86.26% |
| `modules/telemetry/buddy1_protocol.c` | 87 | 98.89% | 91.27% |
| `modules/telemetry/telemetry_peers.c` | 65 | 96.78% | 84.24% |
| `modules/terrain/lift_math.c` | 100 | 100.00% | 96.15% |
| `modules/terrain/task_imu.c` | 46 | 83.82% | 86.29% |
| `modules/terrain/terrain.c` | 46 | 100.00% | 97.22% |
| `modules/terrain/tilt_math.c` | 42 | 100.00% | 100.00% |
| `lib/libnet/lwip/mqtt_utk.c` | not compiled | 89.53% | 78.16% |
| **All measured modules** | 70 | **83.09%** | **75.63%** |

"Before" is the suite at the start of this test review. Files not listed are not compiled by any host test. They are either hardware drivers or RTOS glue (see below).

## Not unit-testable on the host

| What | Why | Evidence instead |
|---|---|---|
| Motor PWM, encoder ISR, IR GPIO/ADC sampling, LSM303 I2C, ultrasonic timing, servo PWM (`drivers/*`, `ultrasonic.c`, `servo.c`, `bc_fast.c` ISR) | They need the RP2040 peripherals and real timing. | Videos FR04, FR05, FR08, FR09/FR10, FR17. The logic above them is tested on the host. |
| Task scheduling, priorities and periods (`tasks.c`, `task_ir.c`, task entry points) | Real µT-Kernel SMP scheduling and timing. | NFR2–NFR4 constants are traced by `test_requirement_constants.py`. The car videos show the timing working. |
| Wi-Fi join and the CYW43 radio (`lib/libwifi`) | Needs the radio. | Video FR01 (connect), FR15 (link loss stop). The MQTT client is tested with stub lwIP in `test_ct_mqtt_link.c`. |
| End-to-end line following, barcode reading and obstacle passing on the real track | Sensor noise, friction and lighting. | Videos FR04, FR05/FR06, FR09/FR10, FR14. Host simulations cover the control logic (`line_follow_sim.c`, `obstacle_rejoin_sim.c`). |
| Barcode scanner state machine on live ADC data (`bc_wave.c` 48 %, `bc_port.c` 50 %) | The rest of their paths need the 2 ms ISR stream and console. | `test_scanner_ready.c` and `test_scan_handoff.c` run the real state machine. The decoders it calls are covered 95–100 %. |

## Videos

| File | Requirements |
|---|---|
| FR01_mqtt_connect_hold_calibrate.mp4 | FR1, FR3, FR12, FR13 |
| FR04_line_follow_pid.mp4 | FR4, FR7 |
| FR05_barcode_decode.mp4 | FR5 |
| FR06_barcode_junction_turn_map.mp4 | FR2, FR5, FR6, FR14 |
| FR08_hump_detect_height.mp4 | FR8, FR18 (hump descent) |
| FR09_obstacle_stop_bypass_rejoin.mp4 | FR9, FR10, FR11 |
| FR10_obstacle_scan_width.mp4 | FR10 |
| FR14_live_route_map.mp4 | FR12, FR14 |
| FR15_mqtt_loss_stop.mp4 | FR15: the car stops within 15 s of the link loss and stays stopped until START |
| FR16_line_loss_restart.mp4 | FR16, FR4 (line-lost search) |
| FR17_lift_stop_resume.mp4 | FR17 |

Every functional requirement FR1–FR18 has a clip, and every requirement has host or dashboard tests. The NFRs are internal and are covered by tests only.
