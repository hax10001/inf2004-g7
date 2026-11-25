# BARR-C:2018 compliance notes

This document covers the team's own C sources:
- everything under `app_program/`, including the host test harnesses;
- the MQTT client `lib/libnet/lwip/mqtt_utk.c` and its header. They are not in the µT-Kernel port; the team added them.

The rest of the µT-Kernel port, the Pico SDK, lwIP, the CYW43 driver and TinyUSB are vendor code and are out of scope.

## How the build enforces it

`build_uf2.sh` runs [`tools/check_app_standard.py`](../../tools/check_app_standard.py) before compiling and stops on any error. `tools/run_host_tests.sh` runs it too. It also checks the host unit tests and their stubs in `tools/`. The test code may include a module's `.c`, call `exit`, use plain `int` and mirror vendor macro names in its stubs. The current result is **166 files, 0 errors**.

It checks:

| Rule | Check |
|---|---|
| 1.2 | Lines of 80 columns or less; no trailing whitespace, tabs or CR. |
| 1.3.a | Every `if` / `else` / `for` / `while` body is braced. |
| 1.7 / 8.3.b | Every `switch` has `default`. |
| 2.1.a | No `//` comments. |
| 2.2 / 3.3 | Every file has an `@file` / `@brief` header and ends with `/*** end of file ***/` and a blank line. |
| Function headers | Every function definition, or its prototype, has a comment directly above it. |
| 4.1 / 4.2 / 4.3 | Module names are lower case and unique in the first 8 characters. Each `.c` has a matching `.h` and includes it first. Headers have include guards. |
| 5.2.a | No plain `int` / `unsigned` / `signed` / `short` / `long`. The only exceptions are `main()` and the fixed `tm_netlog_take()` API. |
| 6.1 | Macro names are upper case. No unparenthesised negative macro body. |
| 7.1 | One declaration per line. |
| 8.2.c | No assignment inside an `if` / `while` condition. |
| 8.5 | No `goto`, `continue`, `abort`, `exit`, `setjmp`, `longjmp`, `auto` or `register` in firmware code. |
| Platform boundary | No direct Pico SDK peripheral calls from application code. |

## What was changed

The work was done in three passes (23–25 Nov):
- **Layout:** braces on every body, with braces on their own line. One statement and one declaration per line. Lines of 80 columns or less, 4-space indent. The telemetry files were converted from K&R style.
- **Types (5.2):**
  - Every bit-field is now declared on a fixed-width type (`uint8_t`, `uint32_t`, `int32_t`).
  - The public return codes (`b1_*`, `g7_nav_*`, `nav_barcode`, `motion_line_hint`) are now `int32_t` throughout: declarations, definitions, callers and tests.
  - Plain `int` / `unsigned` locals and casts are now fixed-width types. Diagnostic `printf` calls use `PRIu32` with matching casts.
- **Named constants:** about 400 literals were replaced by named constants. This covers:
  - the barcode decoder (`BC_*`);
  - the telemetry wire offsets, lengths, field widths and EVENT kinds (`B1_*`, in `buddy1_protocol.h` and `buddy1.h`);
  - mission, readiness and fault codes (`CTRL_*`, each noting the dashboard table that mirrors it);
  - the obstacle, motion, terrain and driver modules;
  - the MQTT packet constants (`MQTT_*`).
  
  `event_kind=1u` became `B1_EVT_BARCODE`, and the dashboard test now checks for that name and its value.
- **Naming (7.1):**
  - Telemetry types renamed to lower case with a `_t` suffix (`b1_store_t`, `b1_frame_t`, …, `g7_telemetry_peers_t`, `terrain_snapshot_t`), with the tests and min-bits tables updated to match.
  - Globals use `g_` / `gp_` / `gb_` / `gh_` prefixes. Pointer parameters use `p_`.
  - The debug globals are now `g_usermain_stage` / `g_usermain_error`. They keep external linkage because the USB console prints them.
- **Macros:**
  - Function-like macros converted to `static inline` functions: `SAT_INC8`, `SCAN_MIN`, `OBS_COSF` / `OBS_SINF` / `OBS_ATAN2F`, `ENC_*` and `B1_BARRIER`.
  - Every remaining macro body and expression parameter is parenthesised.
- **Structure:** the `else` that joined two build variants in `task_controller.c` was restructured so both variants are braced.
- **`mqtt_utk.c`:** rewritten in the same style. Helper functions were split out, and the code that reset the connection, written out in two places, now lives in one function.
- **Comments:** history notes, dates, version tags, commit references, test-session stories and commented-out code were removed. Each remaining comment states the reason in the present tense. Every function has a short `@brief`. A code comparison with comments and whitespace removed confirms no code changed in this step.

## Dead code removed

Each item had no remaining references (grep across `app_program`, `lib`, `tools` and the build files). After each removal the build and every test still passed.

- **Encoder-calibration logger:** `bc_enc.c` and `bc_enc.h`, never called (`bc_enc_run` had no caller). Also removed: the `BC_CAL_ENCODER_ONLY` flag, the host harness `barcode_scanning/host/test_encoder.c`, and its three orphan host headers. The linker had already discarded this code.
- **Unused functions:**
  - `console_diag_level()`;
  - `ir_shared_lock()` / `ir_shared_unlock()`;
  - `b1_runtime_store()` (and `buddy1_integration.h`);
  - `b1_mqtt_queue_drops()`;
  - `g7_nav_note_decoded()`, a no-op, and its one call.
- **Write-only state:** in `mqtt_utk.c`, `g_tx_drops` and `g_rx_drops`, `g_last_tx_ms` and `g_awaiting_ping` (nothing ever read them), and the unused `B1_KEEPALIVE_MS`.
- **Unused macros:** `BC_CODE_LETTER_MASK`, `OBS_ALIGN_TOL_DDEG` and `CONSOLE_DIAG_READY`.

## Behaviour verification

Every batch of style edits was rebuilt, and the UF2 compared with the build before it.

**Identical UF2 (no generated code changed):**
- layout and braces;
- the bit-field types (the record sizes in `docs/min-bits/record_sizes.csv` are also unchanged);
- the `int32_t` interfaces;
- the type and identifier renames;
- the named constants;
- the comments.

**Changed UF2:** each change was checked by comparing the per-function code sizes in the ELF and disassembling the changed functions.

| Step | What changed in the generated code |
|---|---|
| `mqtt_utk.c` rewrite | Only `mqtt_utk.c` functions changed:<br>• `discard` and `tcp_error` now share `reset_session`;<br>• `process_packet` is inlined into `tcp_recv_cb`.<br>`.text` is 112 B smaller; `.data` and `.bss` are unchanged. |
| Macros to inline functions | `run_scan` now calls `servo_*_limit_deg()` (a pure getter) once instead of twice. In `obstacle_bypass_step` the operands of a float addition swapped order, which gives the same result because IEEE addition is commutative. |
| Dead code | Only the removed write-only counters and the no-op call went: `b1_mqtt_submit`, `mqtt_write`, `tcp_recv_cb` and `g7_telemetry_poll` got smaller. |

The final clean build (all 163 objects recompiled, CZ2128Delta configuration) is `3351b17cfcab3c2a295db74a5b389d71428499f2`.
- Host tests: 369 PASS lines with `HOST PASS`.
- Dashboard tests: 110/110.

## Remaining deviations

| # | Rule | Where | Reason |
|---|---|---|---|
| D1 | Signed bit-fields | `int32_t x:N` fields holding signed values (sides −1/0/+1, tilt, speed, pitch ×10) | The values are signed. Making the fields unsigned would need explicit sign extension at every read, which changes the arithmetic. |
| D2 | Macro instead of an inline function | `B1_SOURCE_INGEST` (`buddy1.c`) | It generates whole functions with token pasting, so no function can replace it. Its expression parameters are parenthesised. |
| D3 | Macro instead of an inline function | `SERVO_MIN_OF` (`servo.c`) | It is used in constant expressions (`SERVO_LEFT_DEG` / `SERVO_RIGHT_DEG`). Fully parenthesised. |
| D4 | 5.2 plain types | `int main()` in host tests; the `tm_netlog_take()` prototype and call in `mqtt_utk.c` | `main` must return `int`. `tm_netlog_take()` is the lib/libtm USB-console API, so its types are fixed there. |
| D5 | `const` pointer parameters | Remaining non-`const` pointer parameters | A script listed every pointer parameter that is only read. All of them were already `const`. The rest are output or in/out parameters, or must match RTOS, lwIP or callback signatures (`b1_lock_fn_t`, task entry functions). |
| D6 | No magic numbers | Lookup-table data (`g_sine_milli`, `g_phi`, `g_scales`, `g_anchor_percent`, `g_offsets`), and `0`, `1` and `2` used for halving or doubling | Table data stays as data; the table sizes are named. |
| D7 | Unused definitions | `B1_BAD_MAPPING`, `B1_PENDING`, `OBS_FAULT_LINE_LOST`, `OBS_FAULT_SERVO` | They document wire result codes and fault bits the dashboard decodes, even though the firmware never sends them today. |
| D8 | Build variants | `G7_LIVE_CONTROL=0` (legacy, non-live) | It already failed to compile before this work (`line_heading_publish` undeclared). It is not the configuration that is flashed. |
| D9 | Build variants | `make TEST=1` on-target self-tests (`test_*.c`) | Not built or run by the scripts. Kept as tests. |
| D10 | Scope | Vendor files in `lib/`, `kernel/` and `device/` | Out of scope. The team's two-line rename of the boot-stage globals in `lib/libtm/.../usb_console.c` is the only edit there. |
| D11 | Evidence | `docs/min-bits/MINIMUM_BITS.md` instance totals (103,594 → 69,598 B) | The per-record table and the CSVs were regenerated from the final ELF. The instance-weighted RAM totals were not recomputed. |
