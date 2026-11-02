# Buddy 5 — side-corridor and rear-clearance improvements

Date: 2 November 2026
Base: `INF2004_G7_Alternative_Buddy5_Reviewed_20261102.zip`

## Problem observed during the real car test

The existing car **does detect an obstacle, drives around it and recovers
line following**. However, it can choose a departure side without checking
far enough outward, pass too close to the object, or start returning to
its original line while the object has not yet ended.

The following faults existed in the alternative algorithm:

1. The nine-angle ±40-degree scan did not check the farther outward
   directions before committing to a left/right bypass.
2. The passing offset could be insufficient at the object's side. It
   increased the desired offset by the same gap error on *every ping*,
   potentially accumulating an inconsistent target.
3. Missing side echoes were counted as clear readings, even though missing
   Echo is not proof of the object's end.
4. A fixed assumed depth could permit return before the side sensor had
   ever observed the rear of the object.
5. A previously clear side reading could be reused at the later return
   position, even if the object had reappeared in the meantime.

## Behavior implemented

**Before choosing a bypass direction**

The original nine-profile-bearing scan is retained, plus two outer side
checks at **−70 and +70 degrees**. Each outer bearing takes a filtered
three-ping measurement. An object closer than **240 mm** in a proposed
outward direction vetoes that direction, and the planner chooses the other
side if its geometry is feasible. These are **11 servo positions total**;
they add side visibility but do not provide omnidirectional coverage.

**Leaving the line**

The planner's target now reserves a **65 mm nominal safety margin plus
40 mm outward extra** beyond the profiled object edge. For an unseen
edge, the minimum offset increases to **430 mm**. The route remains
bounded by available scan geometry. During the outward diagonal leg, the
servo watches the *departure side* as well as occasionally glancing
forward. Two consecutive close outward echoes below **190 mm** stop that
leg for reassessment, rather than ignoring a nearby obstruction.

**Passing alongside the object**

The side-looking sonar must observe the object with **two consecutive
side hits**. Its lateral gap is used to increase the target distance only
when necessary; the same correction is not added repeatedly at each ping.
Two side hits indicating a gap below **100 mm** request a guarded route
reassessment. The nominal pass/reentry margin is increased from 40 to
**110 mm**.

**Confirming the object has ended**

The fixed-depth fallback is removed. The object must have been observed
from the side before the car may return. The controller requires:

- at least three genuine clear side returns after the last object echo;
- at least **110 mm** encoder-estimated travel since that last echo;
- continued passing along the side until the computed rear body/turning
  clearance is met; and
- a **new** side observation at the actual return position, obtained at
  **65 degrees** rather than the usual 70-degree side bearing.

At the planned inward turn point the car pauses and waits for the new
observation. A new close object echo returns the car to the PASS_SIDE
phase. A missing sensor echo (`-1`) is not counted as an end. If the
obstacle never yields confirmed side clearance within the bounded pass,
the robot stops rather than driving into it.

The original encoder-control, RTOS interface, Buddy 3 IR handoff, motor
code, barcode code and ramp code have not been modified.

## Exact production source changes

- `app_program/modules/obstacle/obs_config.h` — clearance and scan limits
- `app_program/modules/obstacle/scan_plan.h` — checked-side planner API
- `app_program/modules/obstacle/scan_plan.c` — left/right vetoes and margin
- `app_program/modules/obstacle/task_obstacle.c` — outer scans, dynamic
  side-watch aiming, distinct missing/out-of-range echo handling
- `app_program/modules/obstacle/obstacle.c` — outward guard, side pass
  confirmation, rear travel, fresh reentry check and safer return
- `app_program/modules/obstacle/README.md` — implementation details

The two new small state counters use 2-bit bit-fields each; the fresh
return probe state uses one bit. There is no new task, semaphore, queue,
GPIO assignment or externally transmitted status field. Existing packed
plan/status layouts are preserved. No comparable before/after RAM figure
is available without an ARM cross-build under the same settings.

## Tests executed in the host environment

```sh
python tools/check_app_standard.py
python tools/tests/run_buddy5_clearance.py --cc gcc
python tools/test_app.py --cc gcc
```

Actual results:

- `check_app_standard.py`: **61 application files, 0 errors**.
- Focused planner and live-side logic tests: **18/18 passed**, including
  blocked-left/right selection, object reappearance, missing side echo,
  rear travel requirement, fresh return-point echo, and close-side sensing.
- Strict GCC C99 syntax checks: **passed** for the three modified `.c`
  files, with `-Wall -Wextra -Werror` and host stubs.
- The TEST=1 startup firmware source linked under host GCC, and the host
  startup reported **16 PASS checks** (11 existing non-obstacle tests and
  five existing obstacle tests).
- `python tools/test_app.py --cc gcc`: **FAILED** on an existing
  `tools/tests/robot_harness.c:313` motor target assertion
  (`status.left_target_rpm == 40.0f && status.steer == 0.0f`). No Buddy 2
  motor source was changed to suppress or mask this discrepancy.
- ARM firmware build and physical course test: **not performed** in this
  execution environment. The ARM cross-compiler is not available here.

The automated coding checker is not a complete BARR-C:2018 proof. The
updated application code was also reviewed for C99 compatibility, bounded
side-hit counters, paired documentation headers and shared-state usage.

## Installation and Windows build

Use the **changed-files ZIP only with the alternative Buddy 5 code** from
`INF2004_G7_Alternative_Buddy5_Reviewed_20261102.zip`. Back up or commit
your existing work before extracting it into `inf2004-g7`, and allow only
matching files to be replaced. Do not delete the obstacle folder or
replace your working `.git` folder.

From project root, run the two focused checks shown above. To create the
real USB-CDC firmware, open PowerShell inside `build_make`:

```powershell
$env:Path = "C:\msys64\usr\bin;C:\msys64\ucrt64\bin;C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\gcc-arm-none-eabi\bin;$env:Path"
$sdk = "$env:USERPROFILE\pico\pico-sdk" -replace '\','/'
& 'C:\msys64\usr\bin\make.exe' -B -j4 all SMP=1 CONSOLE=usb_cdc OBSTACLE=1 TEST=0 "PICO_SDK_PATH=$sdk" PYTHON=python SHELL=C:/msys64/usr/bin/sh.exe
```

Verify Make's exit code and the modification time of
`mtk3pico_smp1_usb_cdc_obs.uf2`. The ZIP contains source changes; no
newly compiled hardware firmware is supplied.

## Required physical checks

- Confirm the HC-SR04 Echo voltage is safe for 3.3 V Pico GPIO, servo
  polarity, wheel dimensions and encoder direction.
- On a raised-wheel test, confirm the scan visits ±70 degrees and the
  servo changes to ±65 degrees for the return-point check.
- On a wide, flat test surface with an immediately accessible motor-power
  stop, try objects close to the departure left/right corridors, then
  centered small/wide objects. Verify the body side clearance by ruler.
- Follow the `[OBS] Outward sides`, `[OBS] Side object confirmed`,
  `[OBS] Object ends`, `[OBS] Rear confirmed clear`, and return-phase logs.
- Test different widths and use real run data to tune the 240 mm departure
  veto, 430 mm unknown-edge offset, 100 mm critical gap, 110 mm rear
  travel and the body stopping distance before the assessment.

The sensor is not omnidirectional and an HC-SR04 can miss a surface at
unfavorable incidence angles. A physical route existing on the course
cannot by itself guarantee that this particular sensor/encoder system can
verify it. Never force the vehicle into a detected object or onto an
unverified floor area merely to avoid a `NO_ROUTE` status.
