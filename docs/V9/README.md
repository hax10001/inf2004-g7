# G7 V9 — Preservation-first implementation and evidence

**Release scope:** Working-code refactor checkpoint (NOT final Week 13 compliance or physical acceptance).

**macOS host-test portability update:** `tools/test_obstacle_integration.py`, `tools/tests/run_buddy5_clearance.py` and `tools/test_controller_remote.sh` now use appropriate Clang/Apple linker flags on macOS, rather than passing GCC-only `-fwhole-program` or ELF-only `--gc-sections` options. The C host suite passed with both `gcc` and `clang` on Linux; macOS execution has not been tested here. These changes only affect host tests, not the ARM firmware build or navigation logic.

## Applied changes

- One unified monitoring page with four independently scrolling panels: sensor telemetry, barcode results, firmware console log, and commands/system events. The existing MQTT log topic, binary telemetry stream, SSE connection, command IDs and ACK logic remain separate and unchanged.
- Existing control buttons (including remote STOP), recording endpoints, log-copy/export, filters, route/3D view and mapping dialogs have been retained. Old Command Monitor/Robot Log navigation buttons are shortcuts to the unified page.
- Large repeated barcode/sensor cards moved into a compact **Device Status & Diagnostics** section below the consoles; advanced details remain collapsible.
- Historical barcode successes use neutral state when the live feed is stale. Mapping/readiness show unknown rather than stale confidence.
- Lower-memory native representations in `buddy1.h`, `buddy1_protocol.h`, and `telemetry_peers.h`. Pack changes concern internal records only, not the fixed binary MQTT wire schema; shared `B1_Store` accesses retain existing task mutex ownership. The telemetry peer state is owned by the telemetry task, not an ISR.
- Added 3 dashboard static regressions plus a native C/JavaScript cross-decoding golden-vector test.

## What passed in this runtime

- Application style check: 112 source files, 0 errors.
- Network-owner boundary check: PASS.
- Existing obstacle/controller/telemetry/navigation host integration checks: PASS, including 144 navigation assertions.
- Dashboard Node tests: 67/67 PASS.
- Native codec regression + JS decoder using C-produced snapshot/frame: PASS.
- Headless Chromium DOM smoke test using local document injection: four panes found in the right order; controls present. Browsing local URLs was blocked by the runtime environment, so this is **not** a real live-broker browser trial.

## Measured native sizes (x86_64 GCC, NOT the ARM firmware RAM numbers)

| Structure | Before (B) | After (B) | Delta |
|---|---:|---:|---:|
| `B1_Motion` | 24 | 20 | -4 |
| `B1_Line` | 20 | 12 | -8 |
| `B1_Terrain` | 20 | 16 | -4 |
| `B1_Mission` | 28 | 20 | -8 |
| `B1_Obstacle` | 20 | 16 | -4 |
| `B1_SnapshotPayload` | 22 | 18 | -4 |
| `B1_Store` | 1312 | 1152 | -160 |
| `G7_TelemetryPeers` | 72 | 68 | -4 |

`B1_Store` improves by 160 B (12.2%) in this host ABI, mostly due to the 32-entry motion ring and smaller records. `G7_TelemetryPeers` improves by 4 B. These are **sizeof** measurements for known types, not a whole-firmware `.data`/`.bss`/stack measurement. Compile both original and V9 with the *same* ARM toolchain/flags to produce the actual target comparison.

## MQTT message size comparison

Wire frames are intentionally **unchanged** in V9 to preserve version 1 protocol interoperability: snapshot 13 B payload / 27 B whole frame; mapping command 6 B payload / 20 B frame; other commands 3 B payload / 17 B frame; status 14 B / 28 B frame. Message-size savings are therefore **0 B**. See `message_size_comparison.csv` for all message types. The current wire version was NOT changed; adopting a compact 5-byte mapping command requires a coordinated versioned migration with firmware and JS decoders.

## Minimum-bit requirement completion: NOT YET FULLY VERIFIED

`data_representation_verified.csv` documents reviewed fields and their semantics. `application_fields_candidate_inventory.csv` is an automated first-pass scan of structs in application headers and **not a complete semantic audit**. It flags fields whose precise valid range, reserved values and ISR/task ownership still need review. In particular:

- Internal sensor, barcode, motion and obstacle structures beyond the listed packed records still require semantic range confirmation and measured minimum-width storage.
- Some numeric fields are intentionally 16/32-bit in memory for correct calculations, wrap protection or safe clamping even though the wire uses fewer bits. Confirm with assessor whether packed storage is required at every internal site, and then change consumers safely.
- Current command/status/sensor/barcode diagnostic frames retain version 1 byte-aligned fields, so strict end-to-end minimum-bit wire compliance is not established.
- Independent high-frequency ISR/shared-task synchronization and real-time system performance must be reviewed on the target, even where the telemetry store already holds its mutex.
- Full ARM link/ELF/map (`.data`, `.bss`, heap/stack), full-route tests, physical STOP timing, queue/drop counts and latency comparisons were **not available** here. Do not claim physical acceptance.

## Mandatory Week 13 submission work remaining

1. Finish **every** application field's valid-range, minimum-bit and concurrency audit, including non-telemetry structs, RTOS queues and ISR-shared fields; include this in the data-representation table.
2. Produce **actual ARM RAM usage before/after** with the same build flags, `arm-none-eabi-size`, ELF/map and stack headroom evidence.
3. Add any assessor-required compact binary message revisions, with versioned firmware/JS cross-language vectors and measured before/after sizes (the currently preserved wire format saves 0 B).
4. Run live physical robot tests with motor power and safe cutoff, measuring Wi-Fi loss/STOP, barcode yield, obstacle bypass, hump heights, packet loss, load/jitter, queue drops and full-course repeatability. Provide video/logs and test conditions.

## Running and verification

This ZIP extracts into **`inf2004-g7-v9-unified-console/`**, a new folder distinct from the original input folder. **You must extract it before running `cd`**. If the directory is not present, Terminal remains in your old project, so your 64-test suite is not V9. The V9 dashboard suite has 67 tests. Extract into Downloads without overwriting the previous project, and run commands from the new project root.

On macOS:

```bash
cd "$HOME/Downloads"
unzip -oq INF2004_G7_V9_Unified_Console_Mac_Test_Fix.zip
cd "$HOME/Downloads/inf2004-g7-v9-unified-console"
pwd
python3 tools/test_v9_minbits.py
python3 tools/test_obstacle_integration.py --cc clang
cd app_program/dashboard
node --test --test-concurrency=1 tests/*.test.js
```

From the repository root:

```bash
python tools/check_app_standard.py
python tools/test_network_owner_boundary.py
python tools/test_obstacle_integration.py --cc gcc
bash app_program/modules/telemetry/run_native.sh
python tools/test_v9_minbits.py
cd app_program/dashboard
node --test --test-concurrency=1 tests/*.test.js
```

From the repo root on the configured Mac with Pico SDK and ARM GCC:

```bash
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh
# Expected: out/g7-improve-movement-mqtt.uf2
bash run_live_dashboard.sh
```

Do NOT flash the earlier artifact if the new target build fails. Disconnect motor power while flashing, raise the wheels for initial controls/STOP tests and keep an accessible physical cutoff. The dashboard is not a hardware emergency stop. Never share local `.lan-secrets.json`, Wi-Fi/passwords, or recordings.
