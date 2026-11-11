# INF2004 G7 robot car

Firmware and tools for Group 7's line-following robot car. It runs on a
Raspberry Pi **Pico W** under µT-Kernel 3.0 (dual-core SMP port). The car
follows a line, reads Code 39 barcode letters to choose turns at junctions,
measures tilt with an LSM303 IMU, can bypass obstacles with a servo-mounted
ultrasonic sensor, and reports live telemetry over Wi-Fi/MQTT to a browser
dashboard that can also send mapping, START and STOP.

> [!WARNING]
> Dashboard STOP is a network request, **not an emergency stop**. It cannot
> reach the car if Wi-Fi or MQTT is down. Bench-test with the wheels raised and
> a physical motor power cutoff within reach. Host tests passing does not show
> that the physical car is safe.

## Repository layout

```
app_program/          Robot application: tasks, drivers, modules, dashboard
  app/                Controller, navigation core, app_config.h (main tunables)
  board/              Pin assignments (board_config.h)
  modules/            barcode_scanning, line_barcode (IR), motion, obstacle,
                      telemetry (MQTT protocol), terrain (IMU)
  dashboard/          Node.js dashboard and built-in MQTT broker
kernel/ include/ lib/ device/ config/
                      µT-Kernel 3.0 SMP port, Wi-Fi/lwIP/MQTT owner, config
build_make/           GNU make build (pico_rp2040.mk)
tools/                Build helpers, host tests, diagnostics, barcode tools
docs/                 Guides, reviews and per-release evidence (docs/V9-V15)
```

## Prerequisites

- Pico W, USB **data** cable, and a phone hotspot or router that offers **2.4 GHz**
  (the Pico W cannot join 5 GHz-only networks).
- `arm-none-eabi-gcc`, plus `arm-none-eabi-nm` for the build's MQTT linkage check.
- **Pico SDK 2.2.0, exactly**, with submodules. `build_uf2.sh` rejects other
  versions, because newer SDKs fail on `pico/util/fixed_bitset.h`. Install it
  next to any other SDK:
  ```sh
  git clone --branch 2.2.0 --depth 1 --recurse-submodules \
    https://github.com/raspberrypi/pico-sdk.git "$HOME/pico/pico-sdk-2.2.0"
  ```
- Python 3 (build checks, serial monitor) and Node.js 18+ (dashboard; no npm
  packages need installing).

## Setup and run

All commands run from the project root unless shown otherwise.

### 1. Configure Wi-Fi and MQTT (once per folder)

```sh
cd app_program/dashboard && npm run setup:wifi
```

The wizard asks for the 2.4 GHz SSID, the Wi-Fi password (hidden) and the
broker IPv4, which defaults to your Mac's current address. It writes the
gitignored files `config/wifi_credentials.h` and `config/wifi_secrets.h`. It
**refuses to overwrite** existing files. To start again, run
`bash reset_wifi_setup.sh`, which moves the old headers into
`.g7-wifi-private-backups/` and reruns the wizard. It keeps
`app_program/dashboard/.lan-secrets.json`, so the MQTT login stays matched.
`bash setup_pookie.sh` runs the same wizard with the SSID pre-filled as
`Wireless@SG`.

An older broker header, `app_program/modules/telemetry/buddy1_broker_local.h`,
is still accepted.

### 2. Point the firmware at your computer's current IP

Phone hotspots often hand out a new address each time you reconnect. The broker
IP is compiled into the firmware, so whenever it changes:

```sh
python3 tools/sync_broker_ip.py          # report only
python3 tools/sync_broker_ip.py --apply  # back up and update the broker IP
```

Then rebuild and reflash (steps 3 and 4).

### 3. Build the firmware

```sh
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" \
  bash build_uf2.sh
```

| Variable | Values | Effect |
|---|---|---|
| `PICO_SDK_PATH` | path | SDK 2.2.0 checkout. If unset, the script searches `~/pico/pico-sdk-2.2.0`, `~/pico-sdk-2.2.0`, `~/.pico-sdk/sdk/2.2.0`, `~/pico-sdk` and `~/pico/pico-sdk` for a 2.2.0 install. An exported value is always used as-is, so an exported newer SDK fails |
| `G7_CONSOLE` | `uart` (default), `usb_cdc` | Console on UART0 (GP0/GP1) or on the Pico's USB port. With `usb_cdc`, UART0 is unused, leaving GP0/GP1 to the left encoder |
| `G7_LIVE_CONTROL` | `0` (default), `1` | `1`: boots **stopped** and moves only after an MQTT mapping and START; losing the link triggers a software STOP. `0`: no remote commands, and **the firmware may move on its own** |
| `G7_BARCODE_TEST` | `0` (default), `1` | `1`: motors and obstacle detection compiled out; push the robot by hand to capture barcode waveforms |
| `G7_SKIP_MQTT_LINK_CHECK` | set to skip | Development only: skip the check that `mqtt_utk_poll` is linked |

The script checks the SDK version and submodules, your local Wi-Fi config, the
application coding standard (`tools/check_app_standard.py`) and the toolchain.
It then builds `SMP=1 WIFI=cyw43 ... WIFI_MQTT=1` with make and verifies the
linked ELF before copying the result to **`out/g7-improve-movement-mqtt.uf2`**
(or `out/g7-barcode-test.uf2`). It never flashes. **Do not share the UF2**:
it contains your Wi-Fi and MQTT credentials.

Feature switches such as `APP_OBSTACLE_ENABLED`, `APP_COLLISION_ENABLED` and
`APP_DOWNHILL_NOSE_DOWN_SIGN` are in `app_program/app/app_config.h`. Changing
them requires a rebuild. See [docs/ROBOT_OPERATION.md](docs/ROBOT_OPERATION.md).

### 4. Flash

Disconnect motor power. Hold **BOOTSEL** while plugging in USB, copy the UF2
to the `RPI-RP2` drive, then let the Pico reboot normally (without BOOTSEL).

### 5. Start the dashboard

| Command | Use |
|---|---|
| `bash run_live_dashboard.sh` | **Real robot with mapping and START/STOP.** LAN dashboard and broker with live commands enabled (`LIVE_COMMANDS_ENABLED=1`) |
| `cd app_program/dashboard && npm run lan` | Real robot, **read-only** monitoring. Commands are refused with HTTP 409 |
| `bash demo.sh` | Simulated car on `127.0.0.1` only. No hardware involved |
| `cd app_program/dashboard && npm start` | Dashboard and broker bound to `127.0.0.1`. The Pico **cannot** reach this broker |

Open `http://localhost:3000`. The LAN modes print the broker address, MQTT
login and dashboard admin token, which are kept in
`app_program/dashboard/.lan-secrets.json`. Keep these private. If port 3000
is busy, run with `WEB_PORT=3001`. The startup banner still prints 3000. See
[docs/DASHBOARD.md](docs/DASHBOARD.md) for the views, the mapping-then-START
flow and all environment variables.

### 6. Watch the Pico's serial output (optional)

Requires a `G7_CONSOLE=usb_cdc` build.

```sh
bash serial_monitor.sh --list                     # find the port
bash serial_monitor.sh                            # print and mirror to the dashboard
bash serial_monitor.sh --no-dashboard /dev/cu.usbmodemXXXX
```

On first use it installs `pyserial` into `.venv-serial/` (or set
`SERIAL_PYTHON`). Only one program can read the port at a time. Telemetry
normally reaches the dashboard over MQTT without this.

### 7. If it doesn't connect

Run `python3 tools/diagnose_wifi_mqtt.py`. It compares the configured broker IP
with your Mac's address, checks that port 1883 is listening and that the MQTT
login matches, and reports whether a UF2 exists. It never prints passwords. Then
read the `[NET]` serial line using
[docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md).

## Tests

```sh
bash tools/run_host_tests.sh
```

This is the full native (non-ARM) regression suite. It covers downhill and motion
control, the coding standard, the network-owner boundary, precision ranges,
minimum-bit records, obstacle, controller and telemetry tests, autonomous
navigation, scan handoff, the ramp plant model and all dashboard tests
(`node --test`). It uses `CC` (default `clang`) and writes
`docs/V15/evidence/plant-sign-*.csv`.

Other entry points:

| Command | Notes |
|---|---|
| `cd app_program/dashboard && npm test` | Dashboard tests only |
| `bash tools/test_controller_remote.sh`, `bash app_program/modules/telemetry/run_native.sh`, `bash tools/test_autonav.sh` | Single suites. They are also built by the full suite, there with different compiler flags |
| `python3 tools/tests/run_buddy5_clearance.py --cc clang` | Obstacle planner and side-clearance tests (also run by the full suite) |
| `PICO_SDK_PATH=... bash tools/verify_v15.sh` | Full suite plus ARM build, writing release evidence into `docs/V15/evidence/` |
| `python3 tools/test_app.py --cc cc` | Older monolithic harness. **Currently fails to compile**: its kernel stub lacks the semaphore calls `bc_port.c` uses |
| `python3 app_program/modules/terrain/host/run_tests.py` | IMU calibration test. **Currently fails to link** (`tk_slp_tsk` missing from stubs) |

Code style: `python3 tools/format_app.py` (clang-format 14+),
`python3 tools/check_app_standard.py`, and
`python3 tools/analyze_app.py --clang-tidy clang-tidy`. See
[app_program/README.md](app_program/README.md).

## Documentation

| Doc | Covers |
|---|---|
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Wi-Fi, DHCP, MQTT, USB serial, SDK and build failures, START rejections |
| [docs/DASHBOARD.md](docs/DASHBOARD.md) | Dashboard modes, views, mapping flow, environment variables |
| [docs/ROBOT_OPERATION.md](docs/ROBOT_OPERATION.md) | Navigation, barcode, descent, collision and obstacle behaviour; calibration; bench checklist |
| [app_program/README.md](app_program/README.md) | Application architecture and ownership rules |
| [app_program/modules/obstacle/README.md](app_program/modules/obstacle/README.md) | Obstacle scan, bypass and return |
| [tools/barcode_data/README.md](tools/barcode_data/README.md) | Recording and replaying barcode passes |
| [docs/CHANGELOG.md](docs/CHANGELOG.md) | Version history V5 to V15.1; links to per-release evidence in `docs/V9`-`docs/V15` |
| [docs/KERNEL_PORT.md](docs/KERNEL_PORT.md), [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md), [docs/PORT_RP2040.md](docs/PORT_RP2040.md) | The µT-Kernel SMP port: build profiles, demo, validated features |
| [docs/ROBOT_CODE_REVIEW.md](docs/ROBOT_CODE_REVIEW.md), [docs/DESIGN_CONFORMANCE_REVIEW.md](docs/DESIGN_CONFORMANCE_REVIEW.md) | BARR-C review and design-conformance audit |
| [README_POKIE_BARCODE_TEST.md](README_POKIE_BARCODE_TEST.md) | Team's barcode bench notes (kept as-is) |

## Secrets

Never commit or share `config/wifi_credentials.h`, `config/wifi_secrets.h`,
`app_program/modules/telemetry/buddy1_broker_local.h`,
`app_program/dashboard/.lan-secrets.json`, recordings or any built UF2. All of these are
gitignored (`out/` UF2s via `*.uf2`). Check `git status` before staging.

## Licensing

This repository combines upstream µT-Kernel 3.0 source with port work written
for the RP2040. Nothing here is relicensed: upstream copyright headers are
left intact in every file, and where this section and a file header disagree,
**the file header governs**. Full licence texts are in [`LICENSES/`](LICENSES/).

| Part of the repository | Copyright | Licence |
|---|---|---|
| `kernel/`, `include/`, `lib/libtk/`, `lib/libtm/` (core), `device/`, `config/` - µT-Kernel 3.0 and this port's SMP additions to it: per-core kernel state, the global ready queue, the recursive big kernel lock, cross-core dispatch, and the RP2040 SMP backend | µT-Kernel 3.0, © 2006-2023 Ken Sakamura, released by TRON Forum | T-License 2.2 - [`T-License-2.2_TEF000-219-200401.pdf`](LICENSES/T-License-2.2_TEF000-219-200401.pdf) |
| `lib/libnet/lwip/`, `lib/libwifi/`, `lib/libtm/sysdepend/pico_rp2040/usb/`, `app_program/` - the lwIP raw-API driver, the CYW43439 polling service, the TinyUSB console glue, and the demo application. Built against third-party libraries, but not derived from them | © 2026 Muhamed Fauzi Bin Abbas | Apache-2.0 - [`Apache-2.0.txt`](LICENSES/Apache-2.0.txt) |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_bus_pio_spi.pio.h`, `tools/elf2uf2/` - generated from, or copied from, Pico SDK sources | © 2020 Raspberry Pi (Trading) Ltd. | BSD-3-Clause - [`BSD-3-Clause_Raspberry-Pi.txt`](LICENSES/BSD-3-Clause_Raspberry-Pi.txt) |

### A note on T-License 2.1

TRON Forum releases µT-Kernel 3.0 as a whole under **T-License 2.2**, and the
upstream repository names `TEF000-219-200401.pdf` as the governing document.

64 files still carry "T-License 2.1" in their headers, predating the relicense
- 36 of them in `kernel/tkernel/`, the rest spread across `kernel/knlinc/`,
`kernel/tstdlib/`, `kernel/usermain/`, `kernel/sysdepend/`, `include/sys/`,
`include/tk/`, `include/tm/` and `lib/libtk/`. T-License 2.1
([`T-License-2.1_TEF000-218-150401.pdf`](LICENSES/T-License-2.1_TEF000-218-150401.pdf))
is included in `LICENSES/` so those headers resolve to a text present in this
repository. This is a record of what the files say, not a claim that any part
of µT-Kernel 3.0 is offered under 2.1 rather than 2.2.

Every file added by this port carries a 2.2 header, an Apache-2.0 SPDX tag, or
the original Raspberry Pi BSD-3-Clause notice.

### Build dependencies (not redistributed here)

**The `arm-none-eabi` GCC toolchain** is required to build and is not included.

**The Raspberry Pi Pico SDK** is required only for the `CONSOLE=usb_cdc` and
`WIFI=cyw43` profiles, and is not included. Those profiles compile sources
from three of its submodules, each under its own licence and obtained with the
SDK rather than from here:

| Component | Upstream | Licence |
|---|---|---|
| TinyUSB | `hathach/tinyusb` | MIT, © 2018 hathach (tinyusb.org) |
| lwIP | `lwip-tcpip/lwip` | BSD-3-Clause, © 2001-2002 Swedish Institute of Computer Science |
| cyw43-driver | `georgerobotics/cyw43-driver` | © 2019-2022 George Robotics Pty Ltd. **Hardware-restricted** - see below |

> [!IMPORTANT]
> The **cyw43-driver licence is not a plain BSD licence.** Raspberry Pi Ltd
> grants use "solely with the Licensor's microcontroller chip (RP2040) or any
> other semiconductor device produced by the Licensor. No other use is
> permitted." Its redistribution clause carries the same restriction. That
> covers this port, which is RP2040-only by definition, but it does mean the
> `WIFI=cyw43` profile cannot be carried to non-Raspberry-Pi silicon. The
> kernel, the qualified UART and USB-CDC profiles, and everything under
> `kernel/`, `include/`, `lib/libtk/` and `lib/libtm/` are unaffected.

**CYW43439 radio firmware** is distributed as a binary blob inside the Pico
SDK under Infineon's own terms. It is not redistributed here.

### Sources of the licence texts

| Licence | Obtained from |
|---|---|
| T-License 2.1 and 2.2 | TRON Forum, distributed with the upstream µT-Kernel 3.0 release |
| Apache-2.0 | The canonical text from apache.org |
| BSD-3-Clause (Raspberry Pi) | `LICENSE.TXT` from the Pico SDK repository |
