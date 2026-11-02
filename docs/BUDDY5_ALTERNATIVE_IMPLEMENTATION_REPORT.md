# INF2004 Buddy 5 — adopting the alternative obstacle avoidance code

**Source:** `inf2004-g7(20261102-125316).zip`, uploaded 2 November 2026.

This reviewed version uses the obstacle avoidance subsystem **from the new
upload** as the preferred source. It is not a merge of the previous Buddy 5
progressive/guard implementations. All other production application sources
remain byte-for-byte identical to the new uploaded ZIP.

## What the newly uploaded obstacle algorithm actually does

- Servo scan: **9 bearings**, from -40° through +40° in 10° steps, 3
  ultrasonic pings each. This is 27 sensor pings, not 3 bearings.
- It estimates approximate obstacle edges and selects one of 30°, 45°, 60°
  or 90° according to available forward clearance and side offset.
- It integrates encoder counts into a relative pose in which the original
  line is assumed straight, drives along a diagonal departure using both
  wheels, turns parallel and uses side-looking ultrasonic returns to locate
  the object's trailing edge.
- It calculates rear clearance and then turns inward to reacquire the
  original line using Buddy 3's IR sensors and encoder position.
- The vehicle controller still owns the motor control. This alternative
  requires its **matching `app/task_controller.c` integration** from the
  uploaded project, so copying only the obstacle folder into a different
  revision is not a tested upgrade.

This algorithm potentially addresses the previous prolonged pivoting, but
its turn, pass, safe rear clearance, and line recovery remain **unverified
on the physical car**. It does not guarantee completion around arbitrary
obstacles or correct line recovery at crossings.

## Small, isolated fixes made to the uploaded snapshot

1. `app_program/modules/obstacle/obs_config.h`: initial stop threshold
   increased from 250 to **350 mm** provisionally, and sustained missing
   echoes now raise a fail-safe condition. This may require calibration
   for false no-echo reports in open space.
2. `app_program/modules/obstacle/scan_plan.c/.h`: new
   `scan_plan_build_confirmed` routine retains a previously confirmed
   centre obstacle when the later centre scan incorrectly reports far;
   invalid arguments yield `NO_ROUTE`, not accidental success.
3. `app_program/modules/obstacle/task_obstacle.c`: the scan task passes
   the previously recorded front distance to that routine. It does not
   overwrite the original distance or force forward driving on uncertainty.
4. `app_program/modules/obstacle/README.md`: replaces the obsolete
   placeholder with the actual algorithm's features and caveats.
5. `tools/tests/stubs/tk/tkernel.h`, `tools/tests/stubs/bsp/libbsp.h` and
   `tools/tests/robot_harness.c`: **host-only** RTOS semaphore/event flag
   and output-GPIO stubs added so the new application can link under GCC.
6. `tools/tests/buddy5_planner.c`: new standalone 5-scenario C regression
   tests for the supplied scanner including the confirmed-front bug.

No motor driver, Buddy 1-4 production module, RTOS kernel, task priority,
GPIO assignment, or vehicle controller source was modified during this
review. The only change in sensor trigger policy is the stop threshold;
real braking distances and false echo rates must be measured.

## Verification performed

From the updated source folder:

```sh
python tools/check_app_standard.py
# 61 application C/H files, 0 automated errors

gcc -std=c99 -Wall -Wextra -Werror -I app_program \
  app_program/modules/obstacle/scan_plan.c \
  tools/tests/buddy5_planner.c -o /tmp/buddy5_planner
/tmp/buddy5_planner
# PASS: all 5 standalone planner scenarios

python tools/test_app.py --cc gcc
# FAIL: pre-existing host motor assertion in tools/tests/robot_harness.c
# near its line 313: status.left_target_rpm == 40.0f
```

The full host test now **compiles** with the updated test stubs, but the
existing motor assertion still fails. That test failure is not evidence
that the obstacle bypass itself passed. The full simulation, ARM build,
UART/USB telemetry, real servo, real ultrasonic measurements, 90-degree
turn calibration, ramps, barcodes, and final reacquisition have **not**
been verified in this environment.

## Minimum-bit and coding-standard review

The alternative plan uses a two-word packed route, and phase/status data
use bit masks. Fixed-width integers are used for many shared fields and
locks protect the packed status word. The standard checker passed, but
this does **not** constitute complete BARR-C:2018 or minimum-bit evidence.
The planner still stores each scan bearing as two 32-bit integers, so a
full minimum-bit data-representation and RAM-size comparison is pending.

## How to use this version

**Recommended:** Put the full updated project ZIP in a *new working folder*
or separate Git branch, build with the project's matching controller,
then test before replacing the current Buddy 5 branch. Do **not** mix
`obstacle.c`, `obs_state.c` or `scan_plan.c` individually into your prior
progressive project; their interfaces and shared RTOS mechanisms differ.

To apply the isolated fixes **only if you are already using the new
uploaded project**, extract the changed-files ZIP at the project root,
replace matching files, and leave the other production sources unchanged.

On your Windows toolchain, from `build_make` and after setting SDK/ARM GCC
paths, use your verified `OBSTACLE=1 CONSOLE=usb_cdc TEST=0` command. Confirm
new UF2 timestamp and build exit status. First test with wheels raised;
verify 3.3 V-safe ECHO conditioning, emergency motor-power stop, actual
servo polarity, encoder directions and physical obstacle clearance.

Do not flash this as a hardware-verified solution: the files supplied are
**source code only**, and no new UF2 was produced in this environment.
