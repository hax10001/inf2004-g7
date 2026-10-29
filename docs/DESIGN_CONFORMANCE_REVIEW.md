# Implemented-code review against the G7 design

Reviewed: 2026-10-15.

Reference: user-supplied `P3_G7_Design_Review_Template.pdf`, primarily section 3
(RTOS Task Table, PDF page 6) and section 4 (Task Interaction Diagram and
explanation, PDF page 7). The priority interpretation on page 5 is also used.
The diagram was visually inspected as well as its surrounding text.

**Verdict: the currently implemented application does not strictly conform
to sections 3 and 4.** The restructure preserves the working robot behaviour;
matching a folder template does not establish conformity with this task design.

The user confirmed that the current controller represents the design's
Mission Coordinator. Findings below use that mapping rather than treating
the controller as a temporary line-following test task.

## Scope and exclusions

Assessed implemented code: startup, coordinator state logic, IR sampling,
snapshot sharing, line-following decisions, open-loop motor output, and the
implemented but dormant IMU acquisition/measurement code. Disabled IMU code
is assessed as source/configuration only, not as an executing runtime task.

Not assessed as missing functionality: MQTT/administrative command handling,
barcode decoding, digital filtering, encoder feedback/PID/odometry, junction
and line-recovery algorithms, terrain classification/calibration/hump/height
processing, heading fusion, obstacle scanning/recovery, telemetry and route
tracing. Their absence is not counted as a failed implementation.

However, the trigger, priority, execution context and timing of functionality
that already exists are assessed. These are properties of the current code,
not requests to implement every future feature.

## Confirmed differences in sections 3 and 4

| ID | Design expectation | Current implemented behaviour | Source evidence |
| --- | --- | --- | --- |
| D1 | Section 3: the Mission Coordinator is event-driven. | It polls its state and inputs, then delays for 50 ms. No event wait drives the implemented coordinator loop. | `app_program/app/task_controller.c:45` and `:71`; `app_program/app/app_config.h:12`. |
| D2 | Section 3: the Mission Coordinator has the highest application precedence; IR is in the next rank. Page 5 explicitly says ranks are relative, not final kernel numbers. | IR priority is 5 and coordinator priority is 6. In micro T-Kernel, lower numeric priority means higher precedence, so IR outranks the coordinator. | `app_program/app/app_config.h:8-10`; task creation in `app/controller.c` and `modules/line_barcode/line_barcode.c`. |
| D3 | Section 3: IR sampling has a nominal 2-5 ms interval. | The existing sampler delays for 100 ms after each sample. Its nominal maximum rate is about 10 Hz before work/scheduling overhead, rather than 200-500 Hz. | `app_program/app/app_config.h:14`; `app_program/modules/line_barcode/task_ir.c:59`. |
| D4 | Sections 3-4: Line + Barcode performs perception, the coordinator arbitrates modes/targets, and the Motion Control task owns PWM execution. | The coordinator calls `line_follow_decide()` and then `motion_apply()` synchronously. The latter immediately reaches the motor driver. `motion_start()` creates no task. Existing perception and PWM work therefore execute in the coordinator context rather than the diagram's separate task contexts. | `app_program/app/task_controller.c:145-171`; `app_program/modules/motion/motion.c:21-69`; `app_program/drivers/motor.c:61-67`. |
| D5 | Sections 3-4: time-critical motor control is independent of slower work; Motion Control is scheduled at 5-10 ms. | Current motor updates inherit the coordinator's 50 ms delay plus synchronous work and console output. There is no independent cadence for the PWM path that is already implemented. This finding does not count missing PID/encoders as failures. | `app_program/app/task_controller.c:71`; `app_program/modules/motion/motion.c:32`, `:66-70`. |

D4 distinguishes module ownership from task ownership. Only the motion
module currently invokes the motor driver during normal control, which is a
useful source-code boundary. Calling that module from the coordinator does
not create a Motion Control task or transfer execution to one. There are no
direct PWM register accesses in the coordinator source; the difference is
the synchronous call chain and execution context, not the include structure.

## Implemented but inactive IMU code

**D6 — defined sampling cadence differs.** Section 3 specifies approximately
10 ms, and the section 4 diagram labels IMU/Terrain at approximately 100 Hz.
`app_program/modules/terrain/task_imu.c:13` defines a 20 ms delay, used at
`:97`: at most approximately 50 Hz before I2C, logging and scheduling costs.

`APP_IMU_ENABLED` is zero, and `terrain_start()` only creates the task in the
default configuration. Therefore no current runtime IMU sampling frequency
is claimed. The difference applies if the existing implementation is enabled;
the lack of implemented hump processing is deliberately excluded.

## Timing guarantees that are not established

**D7 — a bounded non-interfering control path is not demonstrated.** Section 4
expects bounded execution and task communication that does not delay control.
The current coordinator executes `tm_printf()` through the normal motion path
before PWM writes. UART transmission synchronously waits for FIFO space in
`lib/libtm/sysdepend/pico_rp2040/tm_com.c:66-79` (with a finite spin limit).
In USB-console builds, `lib/libtm/libtm_printf.c:370-400` takes an
interrupt-preserving output lock around the whole print call. The dependency
comments explicitly identify this as diagnostic rather than hard-real-time
output. These operations add latency to the same task doing motor updates.

Snapshot locks use `TMO_FEVR` in `modules/line_barcode/ir_shared.c:54` and
`:85`. The protected sections are short and use priority inheritance, so
this is not evidence of an observed deadlock. It does mean there is no
application-level timeout or measured worst-case blocking bound in this review.
No timing logs or hardware measurements were produced to establish the design's
bounded-response guarantees. This is an unverified guarantee, not a claim
that an overrun or deadlock was observed on the robot.

## Implemented aspects consistent with the design

| Aspect | Assessment |
| --- | --- |
| RTOS and hardware access | Application tasks and synchronisation use micro T-Kernel. IR/PWM use BSP functions; IMU uses the supplied kernel I2C device interface. |
| Existing pin assignments | IR GP6/GP2/GP27 and motor GP8-11 match section 3 and the section 4 diagram. The existing I2C0 device driver configures GP16/GP17 for the IMU. |
| IR sampling approach | Samples are taken in a task rather than an ISR per raw edge. Filtering itself is unimplemented and excluded. |
| Snapshot consistency and memory size | A single fixed-size latest snapshot is protected by a priority-inheritance mutex. This provides bounded storage and consistent copies. |
| IR synchronisation choice | Section 4's explanation leaves the final IR snapshot synchronisation mechanism to integration. A mutex-backed latest snapshot is not rejected merely because message buffers are proposed elsewhere. Its blocking-time bound remains unverified. |
| IMU device ownership | Only the IMU task calls the LSM303 driver; the driver accesses it through the kernel device API. The code reads an accelerometer and magnetometer, not a nonexistent gyroscope. |
| Existing stale/error handling | Once the following state detects stale IR or a snapshot/time error, it commands zero motor output and latches a stopped/error state. This is consistent with the stop intent for those implemented cases; end-to-end response time is not proven. |

No exact core-affinity layout or literal priority numbers 1-5 are imposed
by this review. The design defers numeric priority selection and measurement;
the D2 finding is about the current reversed precedence relationship.

## Additional contradiction in implemented startup

**D8 — automatic motion instead of START-gated motion.** This is outside the
main section 3/4 focus, but directly contradicts FR1 and the section 5 startup
description. `handle_idle()` transitions to line following when a fresh valid
IR sample exists (`app/task_controller.c:75-108`). That transition does not
require an accepted START. Subsequent clear-line readings issue forward PWM.

The START/MQTT handler itself is unimplemented and excluded; this finding is
about the positive auto-start behaviour that already exists. It is not a
request to implement networking during this audit. The motor driver starts
with zero duty, but enables PWM slices during initialisation; zero duty should
not be described as hardware PWM being disabled.

## Disposition

No runtime changes were made to resolve D1-D8 during this design check.
Changing priorities, task boundaries, sampling rates or startup gating would
change the tested robot behaviour and needs a separate implementation and
robot-validation step. Behavioural equivalence tests are evidence that the
restructure preserved the baseline, not that the baseline matches the design.

For the eventual design-alignment change, keep the existing helper logic but
place its execution in the designed tasks: IR Sampling -> Line/Barcode ->
Motion Control, with Mission Coordinator arbitration and prioritised stop/fault
handling. Keep PWM writes exclusively in Motion Control's task context and
use the design's bounded command/data mechanisms. Only then adjust periods
and precedence with measured timing and repeat the existing robot tests.
