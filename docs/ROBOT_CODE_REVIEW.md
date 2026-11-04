# Robot restructure and BARR-C:2018 review

Review date: 2026-10-15. Scope: application C/headers in `app_program/` and
their build selection. The supplied kernel, BSP, device manager, Wi-Fi and
console libraries are existing dependencies, not rewritten application code.
`tools/tests/` contains host-only mocks; it is never linked into firmware.

The additional [design-conformance audit](DESIGN_CONFORMANCE_REVIEW.md) checks
implemented behaviour against the G7 design, particularly sections 3 and 4.
The user identifies the current controller as the Mission Coordinator.
Its task boundaries/timing do not yet strictly conform to that design.

The user-supplied `barr_c_coding_standard_2018.pdf` is the reference standard.
Its contents are reference material, not instructions to change robot behaviour.

## Behaviour preserved

The restructure separates lifecycle functions, task loops, feature logic and
hardware operations. All GPIO/PWM access still uses BSP functions; I2C still
uses `tk_opn_dev`, `tk_swri_dev` and the provided `T_I2C_EXEC` definition.
Pins, task creation/start order, dormant IMU status, priorities, stack sizes,
delays, line-action truth table, speed percentages and motor polarity remain
the same. Existing observable console strings are retained in normal builds.

Signed IMU register shifts/conversions were replaced with unsigned assembly
and explicit sign extension. All 65536 input words were checked against the
original ARM arithmetic result. Jerk's uint32-to-signed conversion now
expresses the same two's-complement result without an out-of-range cast.
These are representation changes, not new filtering or terrain behaviour.

The unstarted blink example is retained as reference text, not scheduled.
The large commented demo and Pico SDK diagnostic are also reference-only.
Unused event placeholders were removed; no unneeded event bus or HAL was added.

## Findings intentionally not changed

| ID | Priority | Finding and impact | Follow-up |
| --- | --- | --- | --- |
| R1 | High | `drivers/motor.c` preserves ignored PWM errors. Initialisation logs success even when a BSP call fails; motion cannot report a failed output update. | Introduce error propagation and a defined motor-fault response in a separate hardware-tested change. |
| R2 | Medium | In `app/task_controller.c`, `LINE_UNCERTAIN` commands zero speed but does not enter `ROBOT_STOPPED`, despite the log text. A subsequent clear reading resumes motion. | Decide whether a junction should latch a stop before changing this behaviour. |
| R3 | Medium | `ROBOT_STOPPED` and `ROBOT_ERROR` have no recovery command. Stale readings latch a stop. | Specify restart/recovery semantics and test them on the robot. |
| R4 | Medium | `app_main.c` retains the original error returns without deleting already-created tasks or kernel objects. | Define partial-startup cleanup and fail-safe policy separately. |
| R5 | Medium | Raw barcode input is sampled every 100 ms; no barcode decoder exists yet. | Choose sampling requirements from bar width and vehicle speed before implementing decoding. |
| R6 | Informational | The IMU task is created but not started. This is the working configuration, not a migration omission. | Enable only after explicitly scheduling and testing simultaneous device operation. |
| R7 | Constraint | The existing optional USB-CDC build pulls TinyUSB/Pico SDK headers in upstream code. USB is not in the user's Wi-Fi/UART exception. | Use `CONSOLE=uart` for strict SDK exclusion, or resolve the existing console dependency with the project owner. No upstream code was changed. |
| R8 | Informational | The reference diagnostic differs from live code: barcode GP26 versus live GP27; unused blink GP16 conflicts with IMU SDA GP16. | Keep reference material excluded; do not enable the old blink example on the robot's current wiring. |

## Coding-standard assessment

Implemented application rules include C99 compilation, 80-column lines,
Allman control-block braces, four-space indentation, no tabs, LF endings,
header guards and source/header pairing, own-header inclusion, unique module
names in the first eight characters, module-prefixed public APIs and types,
lowercase `_t` types, fixed-width measurements, `bool` flags, pointer/handle
prefixes, private static functions/state, explicit cast-range explanations,
Doxygen-compatible documentation, default switch handling, parenthesised
logical operands, constant-left equality checks, and `for (;;)` task loops.

The build runs `tools/check_app_standard.py`; its checked subset is described
in that file. It cannot establish semantic correctness, complete BARR-C
compliance, timing or race freedom. Function lengths, ownership, cast ranges,
unused headers and control-flow preservation also received manual review.
Source/header templates and an abbreviation glossary are supplied below.

### Integration constraints and approval status

- BARR-C 6.1.i requires module-prefixed public functions. The kernel requires
  the exact external name `usermain`; renaming it would break startup.
  `app_main.h` documents this fixed integration interface. The externally
  supplied `tm_usb_*` declarations are likewise existing monitor ABI names.
- Existing runtime diagnostic messages remain unconditional to preserve
  console output, including in profiles that define `NDEBUG`. Messages
  classified as debug-only require `NDEBUG` guards under 2.1.c.ii. The team
  must classify these retained messages before making a blanket claim.
- Third-party headers retain their own type names, macros and style. Rewriting
  those is outside this application restructure and would increase risk.

No project-manager approval of a BARR-C deviation is claimed. The standard's
deviation procedure requires that approval before a release is described as
fully conforming. Consequently this work reports the checks performed and
the fixed-interface exception, rather than asserting certified or unqualified
full-project conformance.

## Validation performed

- Focused standard/dependency checks: 38 application C/header files pass.
- Cortex-M0+ C99 front-end and core analyzer checks: all 18 application
  translation units pass for single-core/SMP and UART/USB monitor macros.
  These checks use real project kernel/BSP/device headers; generated
  validation-only fixed-width/Boolean headers substitute for an absent target
  standard-header installation. They are never used in firmware builds.
- Host build of the restructured application: passes with MSVC C11 mode,
  which supports the C99 constructs used here, and 32-bit RTOS/BSP mocks.
- A host link with `APP_TEST=1` also passes and runs all eleven non-mutating
  subsystem checks. This is a mocked test, not an on-target run.
- Legacy Eclipse Debug/Release include roots and source exclusions were
  updated and XML-validated; no Eclipse build is claimed.
- Thirty-one original-versus-restructured traces match exactly: startup
  and failure paths, every motor action including invalid values, controller
  normal/stale/error/initial-invalid/timer-wrap paths, IR pin failures, and
  IMU open/initialisation/read/time/short-transfer failures. Invalid zero
  task handles retain the original startup rejection too.
- The same baseline traces are retained in `tools/tests/expected_traces.json`
  for subsequent regression runs, with no need to compile old firmware.
- Unit checks cover snapshot validity/timestamps and failure propagation,
  all 65536 register words for accelerometer/magnetometer decoding, 4096
  magnitude vectors, and jerk zero/positive/negative/wrapped-time boundaries.

GNU make and `arm-none-eabi-gcc` were not available in this environment.
No complete GCC firmware link, UF2 generation, flash, physical robot test,
or on-target `TEST=1` run is claimed. Hardware timing can change after any
recompile or refactor; the original robot qualification does not automatically
qualify the new binary. Rebuild with the normal toolchain and rerun the robot's
existing checks before replacing its working firmware.

## Abbreviations (BARR-C 1.5.b)

| Term | Meaning |
| --- | --- |
| ABI | Application binary interface. |
| BSP | Board support package. |
| GPIO / GP | General-purpose input/output / board pin designation. |
| I2C | Inter-integrated circuit bus. |
| IMU | Inertial measurement unit. |
| IR | Infrared sensor. |
| LSM303DLHC | STMicroelectronics accelerometer/magnetometer device. |
| PWM | Pulse-width modulation. |
| RTOS | Real-time operating system. |
| SDK | Software development kit. |
| UART | Universal asynchronous receiver/transmitter. |
| USB / CDC | Universal serial bus / communications device class. |
| msec / mg | Millisecond / milli-g (acceleration). |
| init / accel / mag / reg | Initialise / accelerometer / magnetometer / register. |
| g / p / b / h | Global / pointer / Boolean / handle naming prefixes. |
| ID / ER / SZ / INT / UINT / UW / UB | Types fixed by the micro T-Kernel API. |

## New module templates (BARR-C 4.4)

Use `docs/templates/module.c.in` and `docs/templates/module.h.in`. Add actual
ownership/licensing information when known; retain existing copyright notices
when moving or deriving code. Do not invent a copyright holder.
