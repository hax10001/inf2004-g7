# IR sampling and line following

This module owns the robot's three IR sensors and the line-following
decision. Despite the folder name, **barcode decoding is not here**. The
Code 39 engine is in [`../barcode_scanning/`](../barcode_scanning/). This
module only hands it the barcode sensor sample on each IR tick.

| File | Role |
| --- | --- |
| `line_barcode.c/.h` | Lifecycle: `line_barcode_init()` and `line_barcode_start()`. |
| `task_ir.c/.h` | IR sampling task (`APP_IR_PERIOD_MSEC`, 1 ms). Reads left/right (digital) and the barcode sensor on GP27, which is analog when `APP_BARCODE_ANALOG_ENABLED=1`. Arms the barcode worker; never decodes. |
| `ir_shared.c/.h` | Mutex-protected IR snapshot and junction latches. The IR task is the only writer; the controller reads copies. |
| `line_follow.c/.h` | Pure function mapping sensor levels to `LINE_FORWARD`, `LINE_CORRECT_LEFT`, `LINE_CORRECT_RIGHT` or `LINE_UNCERTAIN`. No task. |
| `test_lbar.c/.h` | Non-mutating on-target test, built only with `make TEST=1`. |

Pins come from `app_program/board/board_config.h`: left IR GP6, right IR GP2,
barcode GP27. Timing and junction settings are in
`app_program/app/app_config.h` (`APP_IR_*`, `APP_JUNCTION_*`, `APP_NAV_*`).

Host coverage: `line_follow.c` is compiled into the controller and navigation
tests run by `tools/run_host_tests.sh`, and `tools/test_v11_minbits.py` tests
the `ir_shared.c` latches.

The earlier digital scanner that once lived in this folder is described in
[docs/history/DIGITAL_BARCODE_SCANNER.md](../../../docs/history/DIGITAL_BARCODE_SCANNER.md).
