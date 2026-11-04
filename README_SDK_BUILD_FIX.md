# Buddy 5 UF2 build: Pico SDK mismatch repair (2 November 2026)

## Root cause in the latest Mac build log

The firmware's application-standard checker completed with **84 files, 0 errors**.
The compiler then stopped while compiling the Pico W Wi-Fi driver:

```
fatal error: pico/util/fixed_bitset.h: No such file or directory
  from .../pico-sdk/src/rp2_common/hardware_clocks/include/hardware/clocks.h
```

This is a **Pico SDK version mismatch**, not a missing robot file or a BARR-C
violation. The project port is written for **Pico SDK 2.2.0**. The logged SDK
at `~/pico/pico-sdk` includes a newer `hardware/clocks.h` that uses
`fixed_bitset.h`, unlike the official 2.2.0 header. Avoid modifying the newer
SDK or adding one include path at a time: later hardware APIs may also differ.

## Fix on the Mac (keep the existing SDK untouched)

From your extracted project root, run:

```bash
git clone --branch 2.2.0 --depth 1 --recurse-submodules \
  https://github.com/raspberrypi/pico-sdk.git \
  "$HOME/pico/pico-sdk-2.2.0"

PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" ./build_uf2.sh
```

If the separate `pico-sdk-2.2.0` directory already exists, skip the clone,
then (if needed) run:

```bash
git -C "$HOME/pico/pico-sdk-2.2.0" submodule update --init --recursive
```

If `arm-none-eabi-gcc` is missing, install it before building. The helper
runs its normal source checker, preserves your local Wi-Fi/MQTT headers and
builds **without** enabling remote motor commands.

If compilation and linking succeed, the output should be:

```
out/g7-buddy5-mqtt-bench.uf2
```

## What changed in the patch

- `build_uf2.sh` checks the SDK version **before** compiling and displays
  an actionable message if it is not exactly 2.2.0. It also searches common
  folders for compatible SDKs when `PICO_SDK_PATH` is unset.
- `build_make/pico_rp2040.mk` now identifies the **selected SDK path** in the
  build profile. Switching SDK paths invalidates older compiled `.o` and `.d`
  objects so the result is not linked against stale Wi-Fi driver code.
- No motion, GPIO, sensor, MQTT protocol, dashboard or controller algorithms
  were changed by this patch.

You can apply the **small patch ZIP** over the previous `g7-buddy5-mqtt-barr-c-
fixed-uf2-build` source to keep your private local config intact, or start from
this complete project ZIP and copy your private configuration headers over.

## Known limitations

The ARM cross-compiler and complete Pico SDK are not installed in the build
verification environment, so a new UF2 has **not** been compiled or
hardware-flashed here. The compiler or linker may expose a further problem
once the SDK mismatch is fixed. Do not flash onto a powered free-running car:
Buddy 5 avoidance remains experimental, normal autonomous startup may still
actuate motors, and dashboard Start/Stop/Recalibrate on physical hardware are
not yet validated. The host-side `./demo.sh` remains simulation only.

Do not share Wi-Fi secrets or compiled UF2 images containing credentials.
