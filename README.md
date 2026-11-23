# INF2004 G7 robot car

Line-following robot car on a Raspberry Pi **Pico W** (µT-Kernel 3.0, SMP).
It follows a line, reads Code 39 barcode letters to pick turns at junctions,
bypasses obstacles with a servo-mounted ultrasonic sensor, measures humps
with an IMU, and streams telemetry over Wi-Fi/MQTT to a browser dashboard
that also sends the route mapping, START and STOP.

> [!WARNING]
> Dashboard STOP is a network request, **not an emergency stop**. Keep a
> physical motor power cutoff within reach.

## Scripts

| Script | What it does |
| --- | --- |
| `./setup_dashboard.sh` | One-time setup: checks Node.js 18+, creates the Pico Wi-Fi/MQTT config, points the firmware at this Mac. |
| `./reset_wifi_setup.sh` | Backs up the current Wi-Fi config and runs the wizard again (new network or password). |
| `./run_live_dashboard.sh` | Starts the dashboard and its MQTT broker at http://localhost:3000. |
| `G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 ./build_uf2.sh` | Builds the firmware into `out/g7-improve-movement-mqtt.uf2` with the USB console and dashboard Start/Stop (boots held until START). Without the two settings it builds a UART console and the robot may start moving on its own. |

Typical run: join the Mac to the same 2.4 GHz network as the robot,
`./setup_dashboard.sh`, the build command above, hold BOOTSEL and copy the UF2 to the
Pico, then `./run_live_dashboard.sh`. The broker IP is compiled into the
firmware: if the Mac's IP changes, run `./setup_dashboard.sh` and rebuild.

## Requirements

- `arm-none-eabi-gcc` and `arm-none-eabi-nm`.
- **Pico SDK 2.2.0** with submodules (newer SDKs do not build this port):
  `git clone --branch 2.2.0 --depth 1 --recurse-submodules https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk-2.2.0`
- Node.js 18+ (the dashboard has no npm dependencies) and Python 3.

## Layout

```
app_program/app/        Controller, navigation, app_config.h (main tunables)
app_program/modules/    barcode_scanning, line_barcode, motion, obstacle
                        (obs_config.h), telemetry, terrain
app_program/dashboard/  Dashboard server, UI and tests; recordings/ holds logs
kernel/ include/ lib/ device/ config/   µT-Kernel port, Wi-Fi/lwIP/MQTT
build_make/             make build;  tools/  build helpers and host tests
```

## Minimum-bit data representation (Change Requirement 1)

[`docs/min-bits/MINIMUM_BITS.md`](docs/min-bits/MINIMUM_BITS.md): the data
representation table, RAM and message-size comparisons and test/timing
evidence, with the generated tables beside it.

## Evidence (Tribunal 2)

- [`docs/evidence/`](docs/evidence/README.md): demo videos per requirement, the latest test and build output, and a requirement → video → test table.
- [`docs/tribunal2/requirements_traceability.md`](docs/tribunal2/requirements_traceability.md): every FR/NFR mapped to its tests.
- [`docs/tribunal2/barr_c_compliance_notes.md`](docs/tribunal2/barr_c_compliance_notes.md): the BARR-C:2018 pass, including rules skipped and why.

## Tests

- Firmware host tests: `bash tools/run_host_tests.sh` (runs on the Mac, no
  robot needed; ends with `HOST PASS`)
- Dashboard: `cd app_program/dashboard && npm test`

What the firmware tests cover (all run by the one script above):

| File in `tools/` | Checks |
| --- | --- |
| `test_controller_handoff.c` | Start/stop from the dashboard, junction actions, waiting at a junction for a barcode still decoding |
| `test_autonav.sh` | Whole navigation run in simulation |
| `tests/line_follow_sim.c` | Line following on curves and corners |
| `test_scan_handoff.c`, `test_scanner_ready.c`, `tests/test_barcode_port.c` | Barcode scanner hand-off between tasks |
| `test_obstacle_integration.py` | Runs the obstacle tests below |
| `tests/scan_planner.c` | Obstacle scan: side choice, width, drive-out distance |
| `tests/obstacle_watch.c` | Servo sweep, edge vote, scan timing |
| `tests/obstacle_return.c`, `tests/side_tracking.c`, `tests/obstacle_motion.c` | Bypass: turn-out peek, passing the box, return to the line |
| `test_ramp_motion.c`, `tests/test_downhill_governor.c`, `tests/test_lift_math.c` | Hump, downhill speed, lift detection |
| `test_minbits_all.py` (+ `test_minbits_*.py`, `tests/test_minbits_*.c`, `tests/test_ir_latches.c`) | Minimum-bit packing: field widths, saturation, wire format |
| `test_precision_ranges.sh` | Value ranges fit their packed fields |
| `check_app_standard.py` | Coding standard (also run by the build) |

Private files (`config/wifi_*.h`, `app_program/dashboard/.lan-secrets.json`)
are gitignored. Do not share UF2 files: they contain the Wi-Fi password.
