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

## Tests

- Firmware host tests: `bash tools/run_host_tests.sh`
- Dashboard: `cd app_program/dashboard && npm test`

Private files (`config/wifi_*.h`, `app_program/dashboard/.lan-secrets.json`)
are gitignored. Do not share UF2 files: they contain the Wi-Fi password.
