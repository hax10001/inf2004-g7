# Robot application architecture

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
| `app/nav_core.c/.h` | Host-testable junction/barcode navigation state machine. |
| `drivers/` | Motor, encoder, IR and LSM303 access via BSP or kernel device calls. |
| `modules/line_barcode/` | IR sampling, protected readings and line decisions. |
| `modules/barcode_scanning/` | Code 39 capture (2 ms timer alarm) and decoding worker. |
| `modules/motion/` | Movement policy, speed control, downhill governor, motor-driver ownership. |
| `modules/obstacle/` | Ultrasonic/servo obstacle bypass (`APP_OBSTACLE_ENABLED`). |
| `modules/terrain/` | IMU task lifecycle and measurement calculations. |
| `modules/telemetry/` | MQTT wire protocol, bounded queues and telemetry/command task. |
| `platform/console.c/.h` | Existing monitor state and UART fallback. |
| `tests/` | Shared on-target reporting and excluded reference material. |

There is no additional HAL. Each implemented feature has a lifecycle source
and header, plus a separate source/header pair for each independent task.
Line decisions are normal functions; there is no line-following task.
(When this structure was introduced, barcode decoding, encoders and telemetry
did not exist yet; they were added later as the modules above.)

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

## Configuration

Pins are in `board/board_config.h`; timing and features in
`app/app_config.h`. Current values:

- IR pins: left GP6, right GP2, barcode GP27 (analog). Motor pins: GP8/9
  left, GP10/11 right. Encoders: left GP0/GP1, right GP7/GP28. Ultrasonic
  TRIG GP4 / ECHO GP5, servo GP15. IMU I²C GP16/GP17.
- PWM wrap: 12499 (`MOTOR_PWM_WRAP`). Line-following target 15 RPM.
- Priorities: IR 5, controller 6, IMU 7, obstacle 6; stack size 4096 bytes.
- Controller period 50 ms; IR period 1 ms; maximum IR age 200 ms; startup
  delay 2000 ms (`APP_STARTUP_DELAY_MSEC`).
- Start order in `app_main.c`: motion, IR (`line_barcode`), terrain (IMU),
  obstacle, controller, barcode worker, then the telemetry task.
- No CPU affinity is added.

`APP_IMU_ENABLED` is 1 in normal builds and 0 in the `B1_BARCODE_TEST`
(`G7_BARCODE_TEST=1`) profile, which keeps scanner-only capture timing. IMU
collision stops are separately controlled by `APP_COLLISION_ENABLED` (default
0); that threshold is uncalibrated and must be bench-tested with the wheels
lifted before enabling. IR pin configuration stays inside the sampling task's
startup. The motion driver initialises before task creation.

## Build and checks

The robot firmware (Wi-Fi + MQTT) is built with `bash build_uf2.sh` from the
project root; see the [root README](../README.md#3-build-the-firmware). For
kernel-only or test builds from `build_make/`, retain your existing SMP and
console settings. A default UART build has no Pico SDK dependency:

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
python3 tools/test_app.py --cc cc   # currently fails to compile; see root README
```

The maintained host suite is `bash tools/run_host_tests.sh`.

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
