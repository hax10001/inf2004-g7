# BARR-C:2018 compliance pass (23 Nov)

**Scope.** This pass covered the team's own C sources under `app_program/`. These were not touched:

- Vendor and SDK code: `kernel/`, `lib/` (including `lib/libnet/lwip/mqtt_utk.c`), `include/`, `device/`, `config/`, `etc/` and the pico-sdk.
- The dashboard JavaScript.
- `app_program/modules/obstacle/*` and its tests (`tools/tests/obstacle_*.c`, `scan_planner.c`). The pending scan-offset work is in progress there. They are byte-identical to the checkpoint.

**Checkpoint before the pass:**

All in `~/g7-checkpoints/`:

- `g7-pre-barrc-checkpoint-20261123-133350.tar.gz`: the whole tree as it was before the pass, except `.git`, `out/` and `node_modules`.
- `g7-pre-barrc-20261123-133350.diff`: `git diff` at the start.
- `g7-post-barrc-20261123.diff`: `git diff` after the pass.
- `g7-improve-movement-mqtt-pre-barrc-20261123-133350.uf2`: the firmware image, the same bytes before and after.

The first copies were written next to the repo in `~/Downloads/`, but something removed them from there at about 13:47. The tarball above was rebuilt from the pre-pass copy of `app_program/` kept for comparison. The pass's own changes were then reverted in it: the new tests and docs removed, and `run_host_tests.sh` restored.

**Behaviour proof.** The build is reproducible, so each step was rebuilt and the UF2 compared. After every firmware change, `out/g7-improve-movement-mqtt.uf2` is byte-identical to the image built before the pass: SHA-1 `8629fce61706442fdd6f3e26ae41ba0c7f663b3f` both before and after. The firmware the car runs has not changed, and it does not need re-flashing. Host builds use `-Wall -Wextra -Werror`, and they cover the host-only paths that are not in the UF2.

## Results

| | Before | After |
|---|---|---|
| `bash tools/run_host_tests.sh` | 150 PASS lines, `HOST PASS` | 162 PASS lines, `HOST PASS` |
| `npm test` (dashboard) | 107/107 | 107/107 |
| `./build_uf2.sh` (usb_cdc, live 1) | OK, style check 0 errors | OK, style check 0 errors, same UF2 |

## Files touched (firmware sources)

`app/nav_core.c`, `app/task_controller.c`, `drivers/motor.c`, `modules/barcode_scanning/{bc_cal.h, bc_port.c, bc_wave.c, scan_handoff.h}`, `modules/motion/{downhill.c, motion.c}`, `modules/telemetry/{buddy1.c, buddy1_protocol.c, buddy1_rt.c, telemetry_peers.c}`, `modules/terrain/{task_imu.c, terrain.c, tilt_math.c}`.

Host-only file: `modules/telemetry/host/test_tel.c` (braces only).

New test files:

- `tools/tests/test_requirements_trace.c`
- `tools/tests/test_line_search.c`
- `tools/test_requirement_constants.py`

`tools/run_host_tests.sh` gained three entries for them. `build_make/.g7_firmware_sources.sha256` is the build's source fingerprint and is updated by the build itself.

## Categories of fixes

1. **Braces on every `if` / `else` / `for` / `while` body (rule 1.3.a, Allman style per 1.3.d).** There were about 170 sites. Most were done by a script that only wraps a single statement and skips macros, preprocessor branches and nested cases. The remaining 5 were edited by hand (`buddy1.c`, `buddy1_protocol.c`).
2. **`//` comments changed to `/* */` (rule 2.1.a).** 31 lines in `task_imu.c`, `terrain.c`, `tilt_math.c`, `task_controller.c` and `motion.c`. A commented-out `tm_printf` became a labelled "Disabled trace" block comment.
3. **Fixed-width types (rule 5.2.a).** Non-bit-field locals and static parameters moved from `unsigned int` / `unsigned` / `int` to `uint32_t` / `int32_t` in `motion.c`, `downhill.c`, `buddy1_protocol.c` (bit reader and writer, loop counters) and `buddy1_rt.c`. Width and signedness are the same on both targets, and none is printed.
4. **Magic numbers replaced by named constants (BARR-C "no magic numbers" guidance, plus the "not hard coded" rubric item).**
   - `task_controller.c`: mission state codes `CTRL_MISSION_*`, START readiness bits `CTRL_BLOCK_*`, fault sources `CTRL_FAULT_*`, navigation stop reasons, and the IMU freshness and loss times. Each block's comment names the dashboard table it mirrors (`missionNames`, `READINESS_REASONS`, `controllerFaults`).
   - `nav_core.c`: letter and action limits, and the duplicate-barcode window.
   - `buddy1.c`: STATUS, HEARTBEAT and SNAPSHOT periods, the stale-source age, the challenge window and the 10/12-bit sentinels.
   - `buddy1_rt.c`: task priority and period, frames per period, Wi-Fi deadline, network log interval and the "no mission" sentinel.
   - `telemetry_peers.c`: the 10-bit "unavailable" value 1023 and the 1022 maximum.
   - `motion.c`: ramp effort limits, integral clamp and IMU freshness.
5. **Macro body parenthesised (rule 6.1.c).** `BC_CAL_LEFT_FORWARD_SIGN` became `(-1)`.
6. **Already compliant (checked, no change needed):**
   - Every `switch` has `default:` (rule 1.7.c).
   - Header guards, module `@file` / `@brief` blocks, the EOF marker, line length ≤ 80, no tabs, upper-case macros, and no `goto`, `continue`, `short` or `long` (all enforced by `tools/check_app_standard.py`).
   - Every non-static function is used from another file.
   - No other non-static file-scope variables, apart from the two below.

## Rules intentionally skipped, and why

| Rule | Why skipped |
|---|---|
| 5.2 for bit-fields (`unsigned int x:3`, `unsigned char x:1`) | C99 bit-fields are only portable as `int`/`unsigned int`. Changing the `unsigned char` ones changes struct size and layout, which the minimum-bit evidence (`docs/min-bits/*`) and the RAM figures depend on. |
| 5.2 for public `int` return codes (`b1_*`, `g7_nav_*`, `nav_barcode`, `motion_line_hint`) | On arm-none-eabi `int32_t` is `long`, a different type from `int`. Changing the prototypes ripples through callers and tests, and could trigger `-Wformat` on `%d` prints. It is an API change, not a mechanical one. |
| 5.2 for `char` strings | BARR-C allows plain `char` for character data. |
| 8.x static for `g7_usermain_stage` / `g7_usermain_error` (`app_main.c`) | These are deliberately global `volatile` debug markers, read from the ELF by a debugger. |
| Event-kind literals in `telemetry_peers.c` (`event_kind=1u` and others) | `firmware-wiring.test.js` asserts the literal text `event_kind=1u`. Renaming would need a matching test change. Left for a coordinated change. |
| Remaining magic numbers in the barcode decoder (`bc_layout.c`, `bc_match.c`, `bc_wave.c`, about 170 lines) and wire payload offsets (`f.payload[20]` and others) | Most are tuned decoder model coefficients or byte offsets documented in `buddy1_protocol.h`. Naming 170 tuning values one by one is a large, low-value edit. A rename slip in the decoder is a real risk just before the tribunal. |
| Obstacle module (`app_program/modules/obstacle/*`) | Pending scan-offset work by another thread. Not touched at all. It has about 23 brace sites and some plain `int` uses still to fix. |
| Naming prefixes (`g_`, `p_`, `b_`) on existing identifiers | Most code already follows them. Renaming the rest touches many lines for no behaviour value, and some names are read by Python tools (`audit_fields.py`, min-bits CSVs). |
| Function header comment on every function | Many public prototypes already have Doxygen briefs. Adding one to every static helper is a large documentation-only edit, so it was deferred. |
| Converting function-like macros to `static inline` | Only `B1_SOURCE_INGEST` in `buddy1.c` qualifies. It generates several functions, so it cannot become a single inline. |
| Brace in `task_controller.c` around `else` + `#endif` + `if` | The `else` belongs to a `#ifndef B1_LIVE_COMMANDS` branch. Braces there would change which statements each build profile compiles. |
| `lib/libnet/lwip/mqtt_utk.c` | Under `lib/` (lwIP glue), so out of scope by rule. It is only read by the new constants test. |

## Known coverage gaps found while tracing

- FR10 "resume when the way clears" has no behaviour test. Only its constants are traced.
- `test_calibration.c` (FR3) and an NFR7 no-heap scan were added to the suite afterwards (`tools/tests/encoder_read_stub.c`, `tools/test_no_heap.py`).
