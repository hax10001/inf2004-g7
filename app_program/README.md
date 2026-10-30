# Robot application architecture

Buddy 5 is now implemented behind the default-off `OBSTACLE` build option.
See [implementation and verification](../docs/BUDDY5_IMPLEMENTATION_REPORT.md)
for the current branch configuration, hardware checks and build commands.
The historical configuration notes below predate current tuning; use
`app/app_config.h` and `board/board_config.h` as the configuration source.

The robot uses micro T-Kernel tasks and the supplied BSP/device interfaces.
Application code must not include Pico SDK GPIO, PWM, I2C, timing or interrupt
APIs. Wi-Fi and UART are the only user-authorised SDK exceptions; no new SDK
dependency is introduced by this restructure.

## Ownership

| Location | Responsibility |
| --- | --- |
| `app_main.c/.h` | Kernel entry point and two-phase startup. |
| `app/controller.c/.h` | Controller lifecycle. |
| `app/task_controller.c/.h` | Controller state machine and periodic loop. |
| `app/tasks.c/.h` | Common task creation mechanism, not subsystem policy. |
| `app/app_config.h` | Tested scheduling configuration. |
| `board/board_config.h` | Wiring used by the working application. |
| `drivers/` | Motor, IR and LSM303 access via BSP or kernel device calls. |
| `modules/line_barcode/` | IR sampling, protected readings and line decisions. |
| `modules/motion/` | Movement policy and motor-driver ownership. |
| `modules/terrain/` | IMU task lifecycle and measurement calculations. |
| `modules/obstacle/` | Servo sonar scanning, bypass and line recovery. |
| `modules/telemetry/` | Reserved telemetry integration. |
| `platform/console.c/.h` | Existing monitor state and UART fallback. |
| `tests/` | Shared on-target reporting and excluded reference material. |

There is no additional HAL. Each implemented feature has a lifecycle source
and header, plus a separate source/header pair for each independent task.
Line decisions are normal functions; there is no line-following task.
Barcode decoding, encoders, ultrasonic processing, servo control, obstacle
detection and telemetry have not been invented or enabled during this move.

## Template correspondence

`line_barcode.c`, `motion.c`, `terrain.c` and `app/controller.c` correspond to
the template's `subsystemN.c`: they own their `_init()` and `_start()` functions.
The shared task helper is separate, but each subsystem chooses its tasks and
priority. All resource initialisation completes before tasks start.

Keep subsystem headers narrow. Internal task headers are used by their own
lifecycle implementation. `ir_shared.h` and `line_follow.h` are deliberately
shared contracts consumed by the controller; their state remains private.
The IR sampler is the only snapshot writer. Consumers receive mutex-protected
copies; volatile alone is not synchronisation.

Tests are named `test_lbar.c/.h` and `test_ter.c/.h`, rather than appending
`_test` to long module names: BARR-C 4.1.b requires module names to differ in
their first eight characters. These are non-mutating on-target tests compiled
only with `TEST=1`. Motion/controller behaviour is additionally covered by
the host regression harness and its captured working-baseline traces.

## Preserved configuration

- IR pins: left GP6, right GP2, barcode GP27.
- Motor pins: GP8/9 left, GP10/11 right; left forward uses GP9.
- PWM wrap: 12499; forward wheel speeds: 15 percent each.
- Priorities: IR 5, controller 6, IMU 7; stack size 4096 bytes.
- Controller interval: 50 ms; IR interval: 100 ms; maximum IR age: 200 ms.
- Initial delay: 5000 ms; original console wait and settling delays remain.
- Task creation order: IR, IMU, controller. IMU is created but stays dormant.
- No CPU affinity is added; current RTOS scheduling policy is preserved.

`APP_IMU_ENABLED` stays zero. Enabling it is a functional change requiring
robot validation. IR pin configuration stays inside the sampling task's
startup, preserving when GPIO accesses happen. The motion driver still
initialises before task creation.

## Build and checks

From `build_make/`, retain your existing SMP and console settings. A default
UART build has no Pico SDK dependency:

```sh
make
make SMP=1 CONSOLE=uart
make SMP=1 CONSOLE=uart TEST=1
```

The pre-existing optional `CONSOLE=usb_cdc` profile consumes TinyUSB and Pico
SDK headers in upstream code. It has not been changed here and is outside the
stated Wi-Fi/UART exception. Do not treat successful application scans as
approval for that dependency; use UART when strict SDK exclusion is required.

The build enumerates production directories, excludes reference text and
host tests, and uses separate application object directories for `TEST=0/1`.
Test ELF/UF2 names end in `_test`, keeping the normal artifacts separate.
Do not reuse obsolete root-level object files in a custom linker command.
Application sources use C99; kernel and BSP compiler settings are unchanged.
Python 3 is now required for the application standard check during builds;
override `PYTHON` if its executable is not named `python3`.

Legacy Eclipse Debug/Release metadata includes the application and device
header roots and excludes host/reference/opt-in test sources. Its other
inherited settings are unchanged. `TEST=1` selection and the automatic
standard gate belong to the Make flow; run the checks manually for Eclipse.
The Eclipse metadata was XML-validated, not built in the IDE.

Run from the repository root:

```sh
python3 tools/format_app.py
python3 tools/check_app_standard.py
python3 tools/analyze_app.py --clang-tidy clang-tidy
python3 tools/test_app.py --cc cc
```

On Windows, use your Python executable and:

```powershell
python tools/test_app.py --vcvars 'PATH/TO/VC/Auxiliary/Build/vcvars64.bat'
```

Formatting requires clang-format 14 or newer; supply `--clang-format PATH`
when it is outside PATH. The formatter retains BARR-C's final blank line and
case-level break alignment. The checker verifies a defined subset of rules
and rejects direct SDK peripheral includes/calls. It is not a full compliance
certification. ARM analysis uses the real kernel/BSP headers, with minimal
freestanding standard headers if no target sysroot is supplied; it does not
replace a GCC link or hardware qualification.

See [the code review](../docs/ROBOT_CODE_REVIEW.md) for remaining findings and
integration exceptions. Reference diagnostics are not firmware source.

## Design-document status

This structure matches the template's lifecycle organisation, but preserves
the working baseline's task arrangement. It does **not** yet strictly match
the G7 design's sections 3 and 4. The user identifies `task_controller` as the
Mission Coordinator; it currently also executes line decisions and motor
updates. The design instead separates those runtime responsibilities.
See [the design-conformance audit](../docs/DESIGN_CONFORMANCE_REVIEW.md) for
the existing-code differences, with unimplemented features excluded.
