# INF2004 G7 V10 — Unified monitoring and minimum-bit expansion

**Release status: code checkpoint. Full Change Requirement 1 acceptance is NOT established yet.**

## What changed

### Unified operator dashboard
- True equal-height 2×2 desktop grid for sensors, barcode results, robot logs, and commands. Mobile screens stack cards without horizontal overflow.
- Normalized card headers, filters, backgrounds, paddings, scrolling bodies, and the barcode empty state with a small real-data summary.
- **Scroll lock per console and per barcode list:** if the operator scrolls up, incoming messages continue to be received and recorded but the viewport does not jump. Scrolling to bottom, checking Follow, or clicking the **↓ Latest** button resumes following.
- Separate HTTP/SSE/MQTT routes, commands, ACK validation, recording/export and physical STOP safety semantics are not redesigned.
- No fresh data is fabricated. Offline page screenshots use simulated browser-only entries.

### Native minimum-width fields
- `app/nav_core.h`: nine navigation states in **4 bits** (not 3!), 2-bit actions, 5-bit letters, 3-bit fault reason, 1-bit state flags; 2-byte `nav_result_t` on host.
- `barcode_scanning/bc_scan.h`, `bc_code.h`, `bc_layout.h`, `bc_match.h`: decoder validity, scanner state, error and letter fields packed according to verified sentinel ranges. The layout decoder still uses its **255** invalid-letter sentinel; the code/match decoder uses **31**.
- `barcode_scanning/bc_fastq.h` and `bc_fast.c`: ISR sample ADC packed to **12 bits** with three 1-bit flags; use addressable locals for ADC hardware reads, reject values above 4095. The SPSC ring remains whole-record, release-index published, acquire-index consumed; it is never updated in place by both cores.
- `line_barcode/ir_shared.h`, `terrain/terrain.h`: state flags packed; existing task-to-task mutex ownership maintained.
- `telemetry/buddy1.h`: store queue head/count ranges and five-bit source mask packed under existing store mutex. No change to V1 wire layout.

## V9 to V10 native-host size comparison (x86_64 GCC)

| Record | V9 bytes | V10 bytes | Change |
|---|---:|---:|---:|
| `nav_config_t` | 52 | 52 | +0 |
| `nav_sample_t` | 20 | 16 | -4 |
| `nav_controller_t` | 116 | 104 | -12 |
| `nav_result_t` | 12 | 2 | -10 |
| `bc_code_result_t` | 4 | 2 | -2 |
| `bc_scan_t` | 164 | 164 | +0 |
| `bc_fastq_sample_t` | 28 | 24 | -4 |
| `bc_fastq_ring_bytes` | 1792 | 1536 | -256 |
| `bc_match_result_t` | 20 | 16 | -4 |
| `bc_layout_result_t` | 14 | 12 | -2 |
| `ir_shared_snapshot_t` | 20 | 8 | -12 |
| `terrain_snapshot` | 40 | 24 | -16 |
| `B1_Store` | 1152 | 1152 | +0 |
| `G7_TelemetryPeers` | 68 | 68 | +0 |


These are host `sizeof` figures, not ARM RAM reports. The barcode queue's 64 records total **1,792 → 1,536 bytes** (256 fewer bytes in this ABI). The sampled type and ring figures refer to the same storage; do not add them twice. No aggregate system RAM saving is claimed.

## Field audit status

`application_field_audit.csv` covers the current automated header candidate inventory and the additional reviewed V9/V10 fields. **95** field rows are reviewed for these widths; **212** remain `REVIEW_REQUIRED` because true ranges, sentinels, or ownership still need to be proven. In particular: other motion/encoder control structures, all timed ranges/precision, other obstacle/IMU numeric fields, some queue message structures, exact ARM/ISR synchronization and memory usage.

**MQTT application payloads are already binary**, and the current V1 protocol is unchanged intentionally for compatibility. Byte-level size savings over V9 are **0** for these messages. The binary status/sensor/command frames still need an assessed range-by-range wire optimization; do not assert strict end-to-end bit minimality yet.

## Verification

- Dashboard: **70/70 Node tests passed**.
- Browser DOM: 4 equal panels at desktop width; responsive phone layout with no horizontal overflow; scroll-up preserved at pixel 140 despite new data; ↓ Latest + manual bottom both resumed following.
- Navigation host simulation: **144 assertions passed**.
- Obstacle/controller integration host suite: PASS.
- V10 native packed-state and barcode SPSC test: PASS (ADC 4095, overflow invalidation, queue wrap), plus V9 C→JavaScript binary golden vector PASS.
- Style check: 112 files, 0 errors.
- **NOT VERIFIED:** ARM compilation/link, `.data/.bss`, real ISR runtime timing, repeated physical robot runs or fail-safe STOP measurements. A browser test with synthetic data is not proof of a live broker/robot session.

## Run tests on macOS

```bash
cd ~/Downloads/inf2004-g7-v10
python3 tools/check_app_standard.py
python3 tools/test_network_owner_boundary.py
python3 tools/test_v10_minbits.py
python3 tools/test_obstacle_integration.py --cc clang
bash tools/test_autonav.sh
cd app_program/dashboard
node --test --test-concurrency=1 tests/*.test.js
```

## Build and flash on Pico W

From the project root, run `cd app_program/dashboard && npm run setup:wifi && cd ../..` once if local settings are missing. Do not print or upload generated Wi-Fi secrets.

```bash
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh
```

Check that `out/g7-improve-movement-mqtt.uf2` was freshly generated. With **motor power disconnected**, hold BOOTSEL while reconnecting USB. After `/Volumes/RPI-RP2` appears on Mac:

```bash
cp out/g7-improve-movement-mqtt.uf2 /Volumes/RPI-RP2/
```

Run `bash run_live_dashboard.sh` and open http://localhost:3000. Test the wheel commands with wheels elevated, verify physical STOP with an accessible power cutoff, then test reconnection and the real route. Remote MQTT STOP is not a hardware emergency stop.

## Mandatory Week 13 evidence still missing

1. Finish the unreviewed fields in the audit, checking types/ranges and real shared-memory ownership. Do not blindly truncate counters, timestamps or precision-critical measurements.
2. Build V9 and V10 with **the same** ARM compiler, SDK, flags and networking profiles. Run:

```bash
arm-none-eabi-size -A build_make/mtk3pico_smp1_usb_cdc_wifi_dhcp.elf
arm-none-eabi-size build_make/mtk3pico_smp1_usb_cdc_wifi_dhcp.elf
```

Compare both builds' `.data`, `.bss`, stack/queue allocation and map outputs, not just host `sizeof`.
3. Resolve any strict wire-size requirements with a deliberate, versioned C + JavaScript rollout and before/after message measurements, not uncoordinated byte changes.
4. Collect real Pico timing, ISR/queue-drop, barcode, obstacle/terrain and STOP/reconnect tests under load. Report hardware conditions, errors and deadlines.

## Additional barcode regression and prior limitation

The independent `bc_scan.c` + `bc_code.c` deterministic suite has **362 assertions, 1 failed** in both the unmodified V9 baseline and V10: `extreme short narrow run still rejected`. The source-of-truth result is therefore **not a new V10 regression**, but remains a pre-existing acceptance gap. The recorded on-disk C-layout replay generated **identical output in V9 and V10** across the checked date folders, with **179 recorded passes, 89 correctly decoded, 0 incorrect**; rejected trials remain rejected. Do not generalize these saved sets to live barcode accuracy.

Repeat that test and resolve the underlying case with the barcode owner before claiming complete project acceptance. This release does not intentionally modify barcode recognition algorithms.
