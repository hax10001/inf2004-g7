# INF2004 G7 — Pico W line-following robot car

A line-following robot car built on a Raspberry Pi **Pico W** running
**µT-Kernel 3.0 (dual-core SMP)**. It follows a black line, reads Code 39
barcodes to choose its turn at each junction, detours around obstacles with
a servo-mounted ultrasonic sensor, measures humps and lifts with an IMU, and
streams binary telemetry over Wi-Fi/MQTT to a browser dashboard that also
loads the route mapping and sends START and STOP.

> [!WARNING]
> Dashboard STOP is a network request, **not an emergency stop**. Keep a
> physical motor power cut-off within reach.

**Quick links:**
[requirements](docs/tribunal2/g7_pretribunal_requirements_proposed.md) ·
[traceability](docs/tribunal2/requirements_traceability.md) ·
[evidence and videos](docs/evidence/README.md) ·
[BARR-C compliance](docs/tribunal2/barr_c_compliance_notes.md) ·
[minimum-bit data](docs/min-bits/MINIMUM_BITS.md) ·
[PID report](docs/pid/G7_PID_report.pdf)

---

## Features

| Area | What the car does |
|---|---|
| Line following | Two digital IR sensors and a line observer steer the car at 30 RPM on an 18 mm line. If the line is lost it steers gently towards the last side seen, then pivots left and right to search, then stops. |
| Speed control | A PI controller per wheel with feed-forward and anti-windup, driven by quadrature encoders (1056 counts per revolution, 70 mm wheels). The tuning is described in the [PID report](docs/pid/G7_PID_report.pdf). |
| Barcode navigation | An analogue IR sensor decodes Code 39 in both directions. The decoded letter picks the action (straight, left, right or U-turn) at the next junction, from a four-letter mapping sent by the dashboard. |
| Obstacle detour | Stops when an obstacle is confirmed under 250 mm, sweeps the servo to measure the obstacle's width, chooses the clearer side, passes it while watching it with the sonar, and rejoins the line. It resumes at once if the path clears during the scan. |
| Humps and lift | Detects humps from the IMU tilt and reports their height. Caps the wheel speed on the descent. Stops when the car is lifted and resumes once it is back on the floor. |
| Safety | Every fault stops the motors. START is refused while any readiness check fails. Losing MQTT stops the car, and it does not move again until a new START. |
| Telemetry | Compact binary MQTT frames (snapshot, odometry, events, sensor detail, status) with sequence numbers and CRC-8, buffered while offline. |
| Dashboard | Node.js server with its own MQTT broker. Shows a live route map and sensor views, and sends the route mapping, START, STOP and recalibration commands. |

The numeric requirements (FR1–FR18, NFR1–NFR13) are in
[`docs/tribunal2/g7_pretribunal_requirements_proposed.md`](docs/tribunal2/g7_pretribunal_requirements_proposed.md).

## Hardware and pin map

From [`app_program/board/board_config.h`](app_program/board/board_config.h):

| Signal | Pico W pin |
|---|---|
| Line IR left / right (digital) | GP6 / GP2 |
| Barcode IR (analogue, ADC1) | GP27 |
| Left motor A / B | GP8 / GP9 |
| Right motor A / B | GP10 / GP11 |
| Left encoder A / B | GP0 / GP1 |
| Right encoder A / B | GP7 / GP28 |
| Ultrasonic TRIG / ECHO (HC-SR04) | GP4 / GP5 (divide the 5 V ECHO down) |
| Servo | GP15 |
| IMU (LSM303DLHC) | I2C0, through the µT-Kernel I2C driver |

The encoders take GP0/GP1, which are UART0 at boot. Build with the USB
console (`G7_CONSOLE=usb_cdc`).

## Repository layout

```
app_program/
  app/          controller task, navigation, app_config.h (main tunables)
  drivers/      encoder, IR, motor, LSM303 IMU
  modules/      barcode_scanning, line_barcode, motion, obstacle
                (obs_config.h), telemetry, terrain
  dashboard/    Node.js dashboard: server/, public/, tests/
lib/libnet/lwip/mqtt_utk.c   MQTT client (the only lwIP user)
kernel/ include/ lib/ device/ config/   µT-Kernel 3.0 RP2040 SMP port
build_make/     make build used by build_uf2.sh
tools/          host tests, style checker, build helpers
docs/           requirements, traceability, evidence, min-bits, PID report
```

## Build and flash

Requirements:

- `arm-none-eabi-gcc` toolchain.
- **Pico SDK 2.2.0** with submodules (newer SDKs do not build this port):
  `git clone --branch 2.2.0 --depth 1 --recurse-submodules https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk-2.2.0`
- Node.js 18+ (the dashboard has no npm dependencies) and Python 3.

Wi-Fi and broker settings are compiled in from two private headers that git
ignores: `config/wifi_credentials.h` (SSID and password) and
`config/wifi_secrets.h` (broker IP and MQTT login). They are never committed.
`./setup_dashboard.sh` creates them with a wizard and points the broker
address at this Mac. `config/wifi_credentials.example.h` shows the format.

```bash
./setup_dashboard.sh
```

```bash
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" ./build_uf2.sh
```

The build writes `out/g7-improve-movement-mqtt.uf2`. Hold BOOTSEL, plug the
Pico in and copy the UF2 to it. With `G7_LIVE_CONTROL=1` the car boots held
until START from the dashboard. The broker IP is compiled in, so after the
Mac's IP changes run `./setup_dashboard.sh` again and rebuild. Do not share
UF2 files: they contain the Wi-Fi password.

Before compiling, the build runs the BARR-C style checker
(`tools/check_app_standard.py`) over the team's sources and stops on any
error.

| Script | What it does |
|---|---|
| `./setup_dashboard.sh` | One-time setup: checks Node.js, creates the private Wi-Fi/MQTT headers, sets the broker IP to this Mac. |
| `./reset_wifi_setup.sh` | Backs up the Wi-Fi headers and runs the wizard again (new network or password). |
| `./run_live_dashboard.sh` | Starts the dashboard and its MQTT broker at http://localhost:3000, with live START/STOP enabled. |
| `./build_uf2.sh` | Builds the firmware (see above). |

## Run

1. Join the Mac to the same **2.4 GHz** network as the car.
2. `./run_live_dashboard.sh`, then open http://localhost:3000.
3. Power the car. It connects and shows as held.
4. Send the route mapping, then START. STOP holds it again.

## Tests

| Suite | Command | Current result |
|---|---|---|
| Firmware host tests (no robot needed; also runs the dashboard suite) | `bash tools/run_host_tests.sh` | 162 PASS lines, ends with `HOST PASS` |
| Dashboard | `cd app_program/dashboard && npm test` | 107 / 107 |
| Style check (also run by every build) | `python3 tools/check_app_standard.py` | 112 files, 0 errors |

The host tests compile the real firmware modules against simulated hardware:
- line following on curves and corners, and the lost-line search;
- navigation runs, controller START/STOP and safety stops;
- the barcode scanner hand-off;
- obstacle scan planning, passing and rejoin;
- hump, downhill and lift detection;
- the telemetry codec and command handling;
- the minimum-bit packing;
- a check that every number in the requirement table matches its `#define`.

Saved outputs are in [`docs/evidence/test_results/`](docs/evidence/test_results/).

## Documentation and evidence

- **Requirements:** [`docs/tribunal2/g7_pretribunal_requirements_proposed.md`](docs/tribunal2/g7_pretribunal_requirements_proposed.md).
- **Traceability:** [`docs/tribunal2/requirements_traceability.md`](docs/tribunal2/requirements_traceability.md). Every FR and NFR is mapped to its tests and videos.
- **Evidence:** [`docs/evidence/`](docs/evidence/README.md). A demo video per functional requirement in [`videos/`](docs/evidence/videos/), and the latest test and build output.
- **Coding standard:** [`docs/tribunal2/barr_c_compliance_notes.md`](docs/tribunal2/barr_c_compliance_notes.md). What was done for BARR-C:2018, how the build enforces it, and the formal deviation list.
- **Minimum-bit data representation:** [`docs/min-bits/MINIMUM_BITS.md`](docs/min-bits/MINIMUM_BITS.md) (Change Requirement 1).
- **PID report:** [`docs/pid/G7_PID_report.pdf`](docs/pid/G7_PID_report.pdf). Wheel speed controller design and tuning.

## Contributors

From the git history of this repository: Linus-Tay, Adib, SilentFather and
hax10001. The µT-Kernel 3.0 RP2040 SMP port is by Muhamed Fauzi Bin Abbas
(see Licensing).

---

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

The robot application in `app_program/` and the MQTT client `lib/libnet/lwip/mqtt_utk.c`
were written by the G7 team on top of this port; see the git history.

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
