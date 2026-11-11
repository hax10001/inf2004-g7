# INF2004 G7 V11 — Extended Minimum-Bit Firmware Refactor

**Release classification: preservation-first CODE CHECKPOINT. The entire Change Requirement 1 cannot yet be certified.** This version does not assert on-car safety, ARM RAM savings, physical timing, or final Week 13 submission acceptance.

## V11 source changes (from working V10)

- **Bounded MQTT telemetry memory:** `B1_Store.motions` now holds only the fields needed to serialize the next ODOM packet (10 bytes per ring item instead of 20). Full latest motion/checkpoint data remain in their original owner fields. The 24-message event ring uses an explicit 9-byte record instead of a padded 12-byte structure, preserving the same 6-byte event MQTT payload. The records are protected by the existing `B1_Store` mutex. Byte-by-byte extraction avoids unaligned Cortex-M0+ access.
- **Saturated sensor assignment:** in-memory snapshot distances are 10-bit and speeds are signed 12-bit. The producer clamps to the existing wire representation **before** assigning to bit-fields; out-of-range inputs cannot wrap during storage. The wire format, its session/sequence/CRC, and JavaScript decoder are unchanged.
- **Navigation, motion, sensing:** compacted the task-owned steering flag, motion validity, obstacle drive flags, IR edge/junction latches (6-bit shared byte protected by the existing mutex), ultrasonic scan angles (7-bit signed) and range measurements (12-bit), Code 39 pattern (9 bits), scanner status and 12-bit ADC summary fields.
- **Barcode port:** diagnostic state/error/run flags are packed under the existing semaphore. A full public validated `bc_port_config_t` still arrives at initialization, but the port retains only its 3 private thresholds and 3 one-bit flags; the scanner already owns the full calibration config. `bc_port_clear` preserves calibration and decoding snapshots the minimum-run threshold while the lock is held, to avoid a concurrent reinitialization race.
- **No riskier protocol migration:** MQTT frame version remains 1; `/telemetry`, `/command`, `/ack`, `/log` and `/availability` retain their meaning. No dashboard data handlers, 3D renderer or START/STOP logic was changed in V11. The V10 live-console UI and operator-controlled follow-to-latest behavior remain intact.

### Measured record sizes — native x86_64 GCC only

| Record | V10 | V11 | Change |
|---|---:|---:|---:|
| `B1_Store` including its rings | 1,152 B | 760 B | **−392 B (−34.0%)** |
| One ODOM ring record | 20 B | 10 B | −10 B |
| One event ring record | 12 B | 9 B | −3 B |
| One scan sample | 8 B | 4 B | −4 B |
| `B1_SnapshotPayload` | 18 B | 16 B | −2 B |
| Barcode ADC stats | 24 B | 20 B | −4 B |
| Barcode port diagnostic stats | 196 B | 180 B | −16 B |
| Barcode port private config | 36 B | 8 B | −28 B |

`B1_Store` reduction **already includes** its event and ODOM ring reductions. These rows must not be summed as independent global RAM savings. See `host_sizes_v10_to_v11.csv` for complete host `sizeof` comparison. **These are NOT measured ARM `.data`, `.bss`, heap, or RTOS stack figures.**

### Protocol/message comparison

`mqtt_message_size_comparison.csv` records the exact unchanged version-1 payload sizes: SNAPSHOT=13 B, ODOM=6 B, EVENT=6 B, STATUS=14 B, SENSOR=24/32 B, BARCODE_DIAG=28/42 B, mapping COMMAND=6 B and other COMMAND=3 B. **V11 MQTT wire savings: 0 bytes.** Frames already transmit compact binary for the main telemetry payload, but the existing status/command envelopes are not proven bit-minimal. A coordinated versioned firmware + JavaScript change and compatibility tests would be necessary for stricter packing.

### Field audit

`application_field_audit.csv` documents **313 identified header/runtime fields or composite entries** with existing representation, known range where supportable, V11 representation, protection/owner, verification status and limitations. This is a traceable inventory, **not proof of exhaustive source coverage**; additional static/global fields may exist beyond the inventoried headers. In particular, unverified barcode matcher/layout score ranges and terrain float/height precision were not arbitrarily narrowed. All minimum-width assertions must be reviewed against actual course ranges and target compiler layout before final submission.

### Host test results

Evidence in `test_logs/`:

- Dashboard: **70/70** Node tests passed, including four-panel UI / manual-scroll-follow regressions.
- Autonomous navigation: **144 assertions** passed.
- Obstacle/controller/telemetry integration: passed with GCC and Clang.
- Application style: **112 source files, 0 errors**; network-owner boundary: PASS.
- V9/V10/V11 native and cross-language snapshot/frame vectors: PASS with GCC and Clang.
- Added V11 checks: extremes ±32767 input saturation; highest 12-bit ADC value; event/ODOM record sizes; IR latch take-and-clear; task-owned barcode config, semaphore and reset behaviour: PASS.

**Known unrelated baseline limitation:** `tools/test_app.py --cc gcc` fails in BOTH V10 and V11 because its older host stub lacks T-Kernel barcode semaphore APIs (`T_CSEM`, `tk_cre_sem`, etc.). The new V11 dedicated barcode-host test supplies a test-only semaphore shim and passes. The independent barcode-core suite had an existing one-assertion failure in V10 (extreme short narrow run), with the recorded barcode replay unchanged; V11 does not alter that decoding algorithm. None of these failures are claimed fixed by the minimum-bit refactor.

### Remaining requirements before final Week 13 sign-off

1. Resolve or explicitly justify every `NOT_FINALLY_VERIFIED`, `PRECISION_REVIEW_PENDING`, and `WIRE_COMPATIBILITY_REVIEW` entry; inspect non-header internal globals too. The source/precision limits must be grounded in code, real sensor/calibration data, and assessor expectations.
2. Verify that any additional compacting of ISR/task-shared data is atomic or protected; only already-task-owned or mutex-protected fields were changed in V11.
3. Decide whether compact **v2 binary MQTT COMMAND/STATUS/SENSOR** framing is required by the marker. If yes, migrate firmware and JS together; retain a tested old-frame compatibility strategy and never silently reinterpret v1 payloads.
4. Build **both V10 and V11** with the *same* Pico toolchain and flags, record ELF/UF2/map and actual `.data`/`.bss` plus RTOS stack high-water measurements. `python3 tools/compare_arm_ram.py V10.elf V11.elf` helps compare static sections after both builds.
5. Physically verify STOP priority, no automatic restart following link loss, sensor freshness, barcode yield, queue drops/latency, full-course navigation and resource performance. Host simulations are not proof of real-world behavior.

## Commands on macOS

Extract this ZIP into `~/Downloads/inf2004-g7-v11` without overwriting V10.

```bash
cd ~/Downloads/inf2004-g7-v11
bash tools/run_host_tests.sh  # replaces the version-named runner used at the time
# Configure Wi-Fi/private MQTT settings again in this new folder if needed.
# Use private setup files locally only; never put them in shared ZIPs.
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" \
bash build_uf2.sh
ls -lh out/g7-improve-movement-mqtt.uf2
bash run_live_dashboard.sh
```

Flash only the UF2 produced by a successful target build, with motor power disconnected and BOOTSEL held while connecting USB. The Pico BOOTSEL drive is `/Volumes/RPI-RP2` on macOS. Keep wheels elevated and a physical power cutoff available for first START/STOP tests. Remote MQTT STOP is not an emergency stop.

**The ARM compiler and Pico SDK were absent from the packaging environment. The ARM build, memory comparison, and physical commissioning are not verified here.**
