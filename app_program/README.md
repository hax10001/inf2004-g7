# Robot application

The car uses micro T-Kernel tasks and the existing RP2040 BSP/device
interfaces. The Vehicle Controller owns all motion commands. Each subsystem
has separate initialization and task-start operations; shared resources are
created before any task starts.

| Location | Responsibility |
| --- | --- |
| `app_main.c` | Two-phase application startup |
| `app/controller.c` | Controller lifecycle |
| `app/task_controller.c` | Mission state, line decisions and bypass integration |
| `app/tasks.c` | Common task creation |
| `board/board_config.h` | Motor, encoder, IR, sonar and servo wiring |
| `drivers/` | Existing motor, encoder, IR and IMU drivers |
| `modules/motion/` | Buddy 2 speed control and forward wheel-target interface |
| `modules/line_barcode/` | Buddy 3 IR sampling, protected data and line following |
| `modules/terrain/` | Buddy 4 IMU sampling and terrain estimation |
| `modules/obstacle/` | Buddy 5 scanning, profiling, bypass and line recovery |

Buddy 5 is disabled by default. Select `OBSTACLE=1` explicitly in the Make
build after checking wiring, supply voltage and geometry. The controller
calls Buddy 2 for movement and Buddy 3 for recovery decisions; the obstacle
task owns only sonar/servo hardware. Reverse motion is not implemented.

See [the Buddy 5 report](../docs/BUDDY5_IMPLEMENTATION_REPORT.md) for exact
coverage, build commands, representation tables and the unrun physical
verification checklist. Generated artifacts have `_obs` and/or `_test`
suffixes so enabled and test images are distinguishable.

From the repository root:

```text
python tools/test_buddy5.py --cc gcc
python tools/test_app.py --cc gcc --units-only
python tools/check_app_standard.py
python tools/measure_buddy5.py --cc gcc
```

The host tests use simulated hardware and scheduling. The standard checker
covers a defined subset of BARR-C; full conformance and project-wide
minimum-bit evidence require further review. The older captured regression
traces use previous driving/startup settings and are not current evidence.

The car build excludes the inherited root-level kernel demo, reference
text, host harnesses and unused core scaffolding. Do not add a second motor
owner or access live packed fields outside their owning module and mutex.
GP0/GP1 belong to the encoders during operation; use the USB console profile
with a compatible SDK checkout for physical test logging.
