# Buddy 5 experimental MQTT: repair of 2 Nov 2026 make failure

## Result

The pasted build log stopped at `tools/check_app_standard.py` with 123 violations.
The supplied archived source reproduces 118 issues because the user's private,
gitignored broker header is not in the archive. This repair changes the app
source formatting, adds missing BARR-style documentation/guards, moves the
native-only telemetry test to `modules/telemetry/host/`, corrects the MQTT
header template, and replaces one `continue` with equivalent guarded control
flow. It does not change GPIO assignments, motor PWM algorithms, UI, or motion
module logic. No style checks have been disabled.

Checks conducted here:

- `python3 tools/check_app_standard.py`: 83 application files, 0 errors.
- `bash app_program/modules/telemetry/run_native.sh`: PASS.
- `cd app_program/dashboard && npm test`: 14/14 PASS.
- Existing **legacy** broker header reformatted with placeholder data only:
  84 application files, 0 errors; private values left unchanged.
- **Not tested:** ARM toolchain cross-compilation, Pico W UF2 link, physical
  motor/IMU behavior, complete Buddy 5 obstacle controller, MQTT radio hardware.

## On your Mac, from this project's root

1. Copy your existing private headers from the old local project *without
   sharing them*: `config/wifi_credentials.h` and
   `app_program/modules/telemetry/buddy1_broker_local.h`. Alternatively run
   `cd app_program/dashboard && npm run setup:wifi` once to create new ones.
   The tool refuses to overwrite existing credentials.
2. Install **Pico SDK 2.2.0** with submodules, and an `arm-none-eabi-gcc`
   cross-compiler. The build helper rejects incompatible SDK versions and
   searches common locations for SDK 2.2.0. The latest Mac log used an
   incompatible `~/pico/pico-sdk`, causing missing `fixed_bitset.h`.
   See `README_SDK_BUILD_FIX.md` for the two-command safe repair.
3. Run `./build_uf2.sh` at the project root. It repairs the old generated
   broker header **without printing secrets**, creates a private backup,
   runs the real application-standard checker, and attempts the firmware build.
   **Live MQTT START/STOP remains disabled** (`B1_LIVE_COMMANDS=0`).
4. On a *successful* firmware link, the image is copied to
   `out/g7-buddy5-mqtt-bench.uf2`. This output is generated on your Mac;
   **no prebuilt UF2 is included**.

If there is another compiler error, save the last ~50 error lines rather than
pasting the full thousands of build commands:

```bash
./build_uf2.sh 2>&1 | tee g7-build.log
tail -n 60 g7-build.log
```

Do **not** upload `g7-build.log` without reviewing it for secrets or local
paths. Local Wi-Fi and broker credentials may be embedded in a generated UF2;
never publish that image.

## Safety

This is an **experimental, unverified physical-car profile**. The Buddy 5
obstacle code is staged, not integrated into the original sole-PWM motion
controller. `B1_LIVE_COMMANDS=0` prevents **remote** motor commands, but does
not prove that normal firmware cannot autonomously start driving. For first
power-up disconnect motor power and use an independently controlled physical
stop; the UF2 should not be considered ready for autonomous track testing.
The one-command `./demo.sh` remains host simulation only.
