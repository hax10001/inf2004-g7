# Buddy 5 implementation and verification

Branch: `buddy5`, starting at `e470bfb`, 30 October 2026. Changes are local;
no branch switch, merge, commit, push or physical flashing was performed.
The checked-out application was the baseline. Selected historical obstacle
code was reviewed and adapted; other branches were not copied wholesale.

## Implementation status

All rows below are implemented and covered by host logic tests. None has
been qualified on the physical robot. A successful ARM build proves that
the firmware links, not that the car can safely complete the course.

| Requirement | Implemented behaviour |
| --- | --- |
| Forward detection | Centred sonar, median window, repeated confirmation, immediate close-return stop and freshness gating. |
| Coarse scan | Stationary readings at -60, -30, 0, +30, +60 degrees. |
| Fine scan | Complete -70..+70 grid at 10-degree intervals, beginning near the coarse obstacle. |
| Obstacle profile | Approximate width, nearest forward depth/direction and radial left/right return depths with explicit invalid data. |
| Clearance planning | Footprint and uncertainty bounds, entry-turn/diagonal envelope, side corridor checks, 30/45/60-degree candidates. |
| Moving monitoring | Repeated front/inner/outer panning, including pings while wheel targets are nonzero. |
| Safe stopping | Controller requests Buddy 2 stop on hazard, missing/stale data, encoder failure, excessive heading error or exhaustion of limits. |
| Whole-vehicle passing | Positive side observation, three valid far observations, continued inner clearance and conservative travel by both wheels. |
| Line recovery | Encoder heading, bounded search, Buddy 3 IR confirmation, alignment and repeated verification. Both-black cannot confirm recovery. |
| Return to mission | Clear bypass IR latches, restore barcode capture, stop, require fresh centred sonar, then resume normal following. |

Missing echoes never establish free space. There is no blind encoder-only
passing fallback. Buddy 2 supports forward motion only; failure permits one
stationary rescan, then a persistent stopped/error state requiring reset.
During a bypass, reassessment resumes the saved phase only after a CLEAR
stationary scan; a new occupied/unknown route is rejected instead of
attempting an unverified change of route beside the obstacle.

## Exact production-file changes

All paths in this table are relative to `app_program/`, except the build file.
Each listed `.c` and `.h` is a separate retained production file.

| Files | Purpose and compatibility |
| --- | --- |
| `modules/obstacle/obs_config.h` | New calibration, geometry, uncertainty, bounded retries, motion and sensor timing settings. |
| `modules/obstacle/ultrasonic.c`, `modules/obstacle/ultrasonic.h` | New HC-SR04 driver; hardware-counted pulse width and 30 ms hardware-timer deadline, with sleeping polling. |
| `modules/obstacle/servo.c`, `modules/obstacle/servo.h` | New servo PWM, clamping, settling estimate and error reporting. No position feedback is implied. |
| `modules/obstacle/obs_geom.c`, `modules/obstacle/obs_geom.h` | New corner envelope, projection clearance and rear-turn distance calculations. |
| `modules/obstacle/scan_plan.c`, `modules/obstacle/scan_plan.h` | New compact samples/profile, conservative geometry and left/right route selection. |
| `modules/obstacle/obs_state.c`, `modules/obstacle/obs_state.h` | New packed status, plan and live-coverage exchange using existing IR synchronization. |
| `modules/obstacle/task_obstacle.c`, `modules/obstacle/task_obstacle.h` | New single owner of servo/sonar; monitor, stationary scan and live panning. |
| `modules/obstacle/obstacle.c`, `modules/obstacle/obstacle.h` | New lifecycle and controller-stepped, encoder-guided bypass/recovery sequencer. |
| `app/app_config.h` | Add default-off obstacle feature and priority 7; preserve existing priorities, periods and tuning. |
| `app_main.c` | Initialize obstacle resources before tasks start; start obstacle task before controller. Original team TEST=1 checks remain. |
| `app/task_controller.c` | Add obstacle authority state and stop/fault gates; pack local state/history; preserve normal junction policy and existing quiet logs. |
| `board/board_config.h` | Add GP4/GP5/GP15; use user-confirmed 70 mm wheel diameter instead of the old 65 mm estimate. Existing pins remain assigned as before. |
| `modules/motion/motion.c`, `modules/motion/motion.h` | Add `motion_drive()` forward-wheel requests sharing the existing speed loop; clamp to existing 15 RPM maximum, ramp up, stop immediately, reject NaN. Preserve normal steering/control gains. |
| `modules/line_barcode/ir_shared.c`, `modules/line_barcode/ir_shared.h` | Pack binary IR snapshot/latches, reject nonbinary publication. Existing field names, APIs and barcode mutex access remain. |
| `modules/line_barcode/line_barcode.c`, `modules/line_barcode/line_barcode.h` | Add mutex-protected avoidance capture gate. IR sampling continues; partial capture resets, completed results and enabled/disabled mode survive. |
| `build_make/mtkernel_3/app_program/subdir.mk` | Add OBSTACLE=0/1 validation, separate object directories and `_obs` image suffix; omit obstacle objects entirely when disabled. Preserve team tests behind TEST=1. |

Supporting changes: `app_program/README.md` and the obstacle README describe
integration; `.gitignore` excludes the pre-existing `.tmp` scratch area.
`tools/test_app.py` excludes host/obstacle sources from its default-off
regression build, records logs and adds explicit `--units-only` mode.
`tools/tests/robot_harness.c` updates stale 40 RPM expectations to this
branch's existing 15 RPM and tests the motion extension and packed IR.
The coding checker was not changed or weakened.

No changes were made to the kernel, BSP, Wi-Fi, telemetry, motor driver,
encoder driver, IR GPIO driver, barcode decoder/scanner/configuration,
IMU/terrain implementation, task helper or controller lifecycle module.
The newer barcode and magnetometer work is preserved.

## Geometry and calibration assumptions

Coordinates: axle midpoint is the origin; forward is positive longitudinal,
left is positive lateral. Heading increases to the left. Encoder signs are
configurable: left -1, right +1, matching this branch's barcode settings.
Confirm these signs mechanically; wrong-direction progress stops the bypass.

| Parameter | Configuration |
| --- | --- |
| Measured overall width including left IR extension | 153 mm |
| Maximum height | 160 mm; sonar mounting height still needs measurement |
| Front/rear axle offsets | 135 / 65 mm |
| Conservative overall length | 200 mm; latest axle measurements supersede the earlier 190 mm estimate |
| Wheel outside diameter / width | 70 / 16 mm |
| Wheel centre spacing | 110 mm |
| Sonar forward / lateral axle offset | 90 / 0 mm, centred |
| Conservative side bound | +/-90 mm: width153 minus wheel outer half-span63; accommodates asymmetric IR extension |
| Corner radius about axle | 163 mm |
| Safety / position / braking / step reserves | 40 / 20 / 15 / 10 mm; combined 85 mm |
| Nominal sonar half-beam / servo uncertainty | 8 / 5 degrees; combined 13 degrees |
| Rear clearance bound after observed side edge | 803 mm conservative forward progress, using 95% of the smaller wheel travel |

The rear bound is 90 + 163 + ceil(600*cos_upper(70-10-13)) + 40 + 85.
The lookup uses the larger 45-degree cosine bound, giving 425 mm for the
forward blind wedge: total803. Both wheels must travel about846 mm after
edge confirmation, remain within10 degrees of the original heading and
retain valid side clearance. A reappearing edge restarts this window.

Sonar distances are floored to centimetres. Required width/lateral travel
and diagonal length are rounded upward; reported clearance/depth is rounded
downward. Angular gaps include half the scan step plus beam/servo uncertainty.
A near return on an entry turn or diagonal rejects that side even when it
falls inside the main obstacle's outer edge.

This is a conservative sampled geometry model, not a complete occupancy map
or proof of navigability. Sparse sonar cannot rule out thin/absorbent objects,
overhangs, moving rear obstacles or vertical hazards. Use only a supervised,
appropriate test environment after physical calibration.

## Integration, timing and synchronization

The existing controller remains the sole motor-command caller. The obstacle
task publishes sensing results only. `motion_drive()` uses Buddy 2's encoder
feedback, PI/feedforward, PWM driver and status mutex. It adds no independent
motor driver or reverse command. Buddy 3 remains the sole IR publisher.

Existing periods/priorities remain: IR requested delay1 ms, priority5;
controller50 ms, priority6; IMUpriority7. The kernel tick remains10 ms, so a
requested1 ms delay is not a guaranteed1 ms sampling interval. The new
obstacle task uses priority7 and the existing4096-byte task-stack setting.

The four existing mutexes remain motion1, IR1, terrain2. Obstacle state reuses
the IR priority-inheritance mutex for short copies/read-modify-writes and
adds one event flag. Never hold that mutex over sensor access, logging,
motion calls, delays or a nested shared-state call. Barcode sampling retains
its nonblocking mutex attempt. RTOS operations, not volatile, protect data.
Failed status/coverage reads fail closed; event-write errors latch a fault.

Forward detection is configured at800 mm, with an immediate raw threshold
of250 mm. Before a scan, zero movement must persist for two controller reads
and at least300 ms; continued coasting times out after2000 ms. Actual braking
performance must validate these choices before enabling the feature.

Trigger is12 us; echo timeout30 ms uses the microsecond hardware timer,
including wraparound. A60 ms task delay after each ping makes trigger cycles
longer than60 ms, including after failed pings. Servo commands allow
40 + 3*angular_travel ms settling; commanded angle is not measured position.

Every moving phase requires valid front/inner/outer observations in the same
coverage epoch before permission. Travel is limited to10 mm or3 degrees
before stopping for new coverage, with one controller-period stopping
allowance. Panning and repeated pings continue during authorized increments.
Epoch age is bounded to2500 ms; front data to250 ms; odometry to200 ms.
Uncommanded backward motion, count jumps and heading deviations fail closed.
Independent wheel stall timers allow1000 ms cumulative commanded time without
progress, retaining that time across coverage pauses.

Scan/reassessment timeout is12 s plus settling. Moving phase timeout is180 s,
with a900 s whole-bypass cap and one rescan maximum. Search and side passing
also have distance bounds. No failed recovery searches indefinitely.
Line identity is inferred from the planned return and stable two-sensor IR
readings; these sensors cannot distinguish an arbitrary alternative stripe.

## Minimum-bit representation and Week 13 evidence

| Stored data | Range / precision | Bits |
| --- | --- | --- |
| Status phase / route | 0..14 / none,left,right | 4 / 2 |
| Independent faults / completed bypass count | five flags / 0..15 saturating | 5 / 4 |
| Status distance / width | cm, invalid511 / cm, invalid255 | 9 / 8 |
| Scan point angle / range | 15 angle indices / cm0..400 plus invalid511 | 4 / 9 in2 bytes |
| Plan result / side / angle choice | clear,route,no-route / left,right / 30,45,60 | 2 / 1 / 2 |
| Plan width / near / lateral / diagonal | bounded centimetres | 8 / 9 / 9 / 9 |
| Plan left/right radial depth / nearest direction | cm0..62,unknown63 / right,front,left | 6 each / 2 |
| Live ranges / valid mask / reason | three cm ranges / three views / four reasons | 27 / 3 / 2 |
| Live epoch / request timestamp | 1..255 / ms modulo2^24 | 8 / 24 |
| Front freshness | valid + ms modulo2^24 | 25 in one32-bit word |
| Wheel requests / drive flags | RPM0..31 / stop,follow,done,fault | 5 each / 4 |
| IR snapshot / timestamp | four binary values / milliseconds | 4 / 32 in8 bytes |
| Commanded servo position | -80..80 degrees | signed8 |

Status layout: phase0..3, route4..5, faults6..10, count11..14,
distance15..23, width24..31. Plan wordA: result0..1, side2, angle3..4,
width5..12, near13..21, lateral22..30. WordB: diagonal0..8,
left9..14, right15..20, direction21..22. Reserved bits are zero.
Live ranges use three9-bit fields, validity27..29, reason30..31;
the clock word stores epoch0..7 and timestamp8..31.

These explicit word layouts are deterministic integer representations. Local
C bit-fields are not a wire format. No new radio message is transmitted;
the existing telemetry folder is a placeholder. No MQTT payload or network
latency reduction is claimed. A future wire transport must specify byte order.

Measured `sizeof` results agreed on host GCC14.2 and ARM GCC10.3.1:

| Payload | Unpacked equivalent / previous layout | Compact bytes |
| --- | ---: | ---: |
| One scan point | 8 | 2 |
| Fifteen scan points | 120 | 30 |
| Status fields | 24 | 4 |
| Plan fields | 40 | 8 |
| Drive handoff | 12 | 4 |
| IR snapshot, actual prior layout | 20 | 8 |
| Live coverage | 28 | 8 |

Except IR, these comparisons are equivalent unpacked schemas, not a claim
that those objects previously existed in this branch. Alignment padding is
included. Counters/flags use packed local fields with compile-time range
checks; calculation intermediates and RTOS identifiers retain necessary
arithmetic/API widths. Mutex-protected copies prevent torn multiword plans.
Tests checked saturation, invalid/overflow inputs, conservative quantisation,
stale epochs, sticky first hazard, timestamp wrap and encoder wrap.
Hardware CPU cost, worst-case execution time, actual sensor latency, stack
high-water marks and whole-system RAM savings have not been measured.

## Coding review

The unchanged repository checker passes after cleanup. Manual review covered
new module/header pairing, unique module prefixes, public-function briefs,
80-column lines, four-space indentation, LF C/header files, guarded headers,
final markers, fixed-width packed fields, checked driver initialization,
bounded loops/timeouts, stop authority, overflow-aware encoder differences,
short mutex ownership, conservative rounding and BSP register interfaces.
No Pico SDK peripheral call or new SDK dependency was added to the application.
The echo pad remains input-only before its PWM input mux is configured.

This is a scoped BARR-C review, not independent certification of complete
repository-wide compliance. Passing the mechanical subset cannot establish
all BARR-C rules or hardware safety.

## Test and build evidence

Before edits, `python tools/check_app_standard.py` reported54 files/0 errors.
`python tools/test_app.py --cc gcc` already failed because its unit expectations
required40 RPM while production used15 RPM. After correcting those expectations,
the unit checks and original11 TEST=1 checks pass. The default historical
trace comparison still fails at `motor_0`: its captured logging/speed settings
predate the checked-out baseline. Fixtures were not rewritten to hide this.

The barcode core suite already reported3409 checks/83 failures before edits.
The decoder, scanner and configuration remain unchanged. Independent service
comparisons against the original HEAD service/storage gave identical results:
enabled57888/0 failures, invalid8085/0, uncalibrated8085/0, diagnostics71787/28.
The last28 failures are pre-existing diagnostic expectations. The new bypass
gate test passed25198 checks, including incomplete-frame reset, preserved
pending result/enable mode and no additional mutex allocation.

Actual development commands, before removing temporary test sources:

```powershell
python tools/test_buddy5.py --cc gcc
python tmp/buddy5_extra_tests.py
python tmp/buddy5_verify.py barcode
python tools/measure_buddy5.py --arm-cc 'C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi/bin/arm-none-eabi-gcc.exe' --arm-nm 'C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi/bin/arm-none-eabi-nm.exe'
python tmp/buddy5_verify.py arm
```

The51 Buddy 5 scenarios passed with C99, `-Wall -Wextra -Werror` host builds.
Strict ARM compilation passed all28 production translation units with both
`APP_OBSTACLE_ENABLED=0` and1, `-mcpu=cortex-m0plus -mthumb -std=c99
-Wall -Wextra -Werror -ffreestanding -D_PICO_RP2040_ -DAPP_TEST=0`.
Includes were `include`, `config`, `kernel/knlinc`, `app_program`, `device/include`.

Development-only drivers/harnesses, helper scripts, sizeof programs and the
new `test_obs.c/.h` were removed after recording results. Existing team test
sources, fixtures and tools remain. TEST=0 excludes all test sources; final
TEST=1 includes only the original team checks. No physical test was performed.

Final post-cleanup results and build sizes are recorded below.

## Build, configuration and manual flashing

From the repository root in this Windows environment:

```powershell
$env:Path = 'C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\gcc-arm-none-eabi\bin;C:\msys64\ucrt64\bin;C:\msys64\usr\bin;' + $env:Path
Set-Location build_make
& 'C:\msys64\usr\bin\make.exe' -j4 SMP=1 CONSOLE=uart OBSTACLE=0 TEST=0 PYTHON=python SHELL=C:/msys64/usr/bin/sh.exe
# Only after the hardware checks, build the active obstacle image:
& 'C:\msys64\usr\bin\make.exe' -j4 SMP=1 CONSOLE=uart OBSTACLE=1 TEST=0 PYTHON=python SHELL=C:/msys64/usr/bin/sh.exe
# Optional original team checks; not the production image:
& 'C:\msys64\usr\bin\make.exe' -j4 SMP=1 CONSOLE=uart OBSTACLE=1 TEST=1 PYTHON=python SHELL=C:/msys64/usr/bin/sh.exe
```

Outputs: `build_make/mtk3pico_smp1_uart.uf2`,
`build_make/mtk3pico_smp1_uart_obs.uf2`,
`build_make/mtk3pico_smp1_uart_obs_test.uf2`, respectively. `OBSTACLE=1`
overrides the default-off header value. `OBS_GEOMETRY_VERIFIED=1` records the
supplied geometry; it does not certify physical calibration. Set it0 to
forbid bypass planning. Calibration constants are in `obs_config.h` and
`board_config.h`; validate the encoder counts-per-revolution1056 assumption.

The USB console build was attempted and failed before compilation because
`../../sdk/pico-sdk/lib/tinyusb` is absent. UART builds succeeded. GP0/GP1
encoders take over the UART pins, so UART is a compile/bring-up profile with
no usable console after encoder initialization. For USB telemetry, provide
the compatible Pico SDK/TinyUSB tree expected by the existing build and use
`CONSOLE=usb_cdc PICO_SDK_PATH=<actual-sdk-path>`, then qualify that profile.
No RTOS/BSP change or dependency download was made to conceal this blocker.

To flash manually, first disconnect motor power, hold BOOTSEL and connect
USB. From the repository root, copy the chosen UF2 to the `RPI-RP2` volume:

```powershell
$picoVolume = Get-Volume -FileSystemLabel 'RPI-RP2' | Select-Object -First 1
if ($null -eq $picoVolume) { throw 'Connect the Pico in BOOTSEL mode first.' }
Copy-Item -LiteralPath '.\build_make\mtk3pico_smp1_uart_obs.uf2' -Destination ('{0}:\' -f $picoVolume.DriveLetter)
```

These are instructions only; the assistant did not connect, flash or operate
hardware. Flashing an OBSTACLE=0 image preserves ordinary baseline autonomy;
it is not an obstacle-protected mode.

## Essential supervised hardware checks

1. Confirm GP4 TRIG, GP5 ECHO, GP15 servo; check for physical wiring conflicts.
   GP5 PWM slice2 is separate from motor slices4/5 and servo slice7.
   Preserve IR GP6/2/27, encoder GP0/1/7/28 and existing IMU I2C wiring.
2. HC-SR04 uses5 V. Level-shift/divide Echo to a measured safe3.3 V GPIO level;
   never connect its5 V output directly. Verify common ground and a suitable
   servo supply; do not power servo load from the Pico3.3 V rail.
3. With motor power disconnected, check1500 us forward, correct left sign,
   pulse limits600..2400 us, commanded +/-80-degree mechanical travel and
   settling under load. Current scans need at least +/-70 degrees.
4. Confirm153 mm outer width,135/65 mm offsets, centred sonar90 mm forward,
   sonar mounting height,70 mm wheels,110 mm track and asymmetric IR extension.
5. Raise the wheels, check raw encoder signs and count one wheel revolution.
   Verify12 RPM requests and prompt individual wheel stop at the actual load.
6. Measure stopping distance and response at baseline15 RPM and bypass speeds,
   including processing/panning delays. Increase margins or lower speed if
   the15 mm braking allowance or10 mm movement bound is insufficient.
7. Test straight stops first, then coarse/fine profiles, wide left/right paths,
   blocked/narrow paths, missing echoes, live close objects and wheel stalls.
   Confirm both rear corners clear before any return turn. Keep a physical
   power cut-off accessible and initially test at reduced power/speed.
8. Test original-line recovery, repeated encounters, line loss and both-black
   rejection; confirm barcode capture resumes and IMU behaviour remains intact.
   Record real clearances, stopping distances, task timing and failures before
   describing any stage as hardware-verified.

## Final post-cleanup verification

`python tools/check_app_standard.py`: 69 files, 0 errors.
`python tools/test_app.py --cc gcc --units-only`: UNIT_CHECKS_PASS and
11 original TEST=1 subsystem checks passed. Historical trace comparison
is explicitly skipped by this option; the default command's failure is
recorded above, not counted as a pass.

All three final `make -j4 SMP=1 CONSOLE=uart OBSTACLE=0/1 TEST=0/1
PYTHON=python SHELL=C:/msys64/usr/bin/sh.exe` profiles built ELF and UF2.
Commands selected (OBSTACLE,TEST)=(0,0),(1,0),(1,1), not all four pairs.

| Final ELF stem | Text bytes | Data bytes | BSS bytes |
| --- | ---: | ---: | ---: |
| `mtk3pico_smp1_uart` | 79512 | 276 | 14784 |
| `mtk3pico_smp1_uart_obs` | 91480 | 280 | 14888 |
| `mtk3pico_smp1_uart_obs_test` | 92124 | 280 | 14888 |

`arm-none-eabi-nm` confirmed production images contain no team test entry
points, all images exclude the removed `test_obs_run`, and the default-off
image contains no `obstacle_bypass_step`. The TEST=1 image retains the
original team checks. Size is linked section usage, not dynamic task-stack
or heap high-water measurement. No board has run these final images.


## Recorded Buddy 5 scenario output

The following is retained evidence from the temporary harness, not physical
test output. Tick/movement counters are simulated scheduler observations.

```text
PASS profiles (ticks=0, drive=0, scans=0)
PASS monitor (ticks=6, drive=0, scans=0)
PASS monitor_bad (ticks=6, drive=0, scans=0)
PASS monitor_recovered (ticks=9, drive=0, scans=0)
PASS guard (ticks=2, drive=0, scans=0)
PASS guard_bad (ticks=2, drive=0, scans=0)
PASS scan_route (ticks=63, drive=0, scans=0)
PASS scan_clear (ticks=26, drive=0, scans=0)
PASS scan_bad (ticks=23, drive=0, scans=0)
PASS scan_abort (ticks=8, drive=0, scans=0)
PASS left (ticks=1190, drive=942, scans=1)
PASS right (ticks=1190, drive=942, scans=1)
PASS wrap (ticks=1191, drive=942, scans=1)
PASS clear (ticks=11, drive=0, scans=1)
PASS no_route (ticks=24, drive=0, scans=2)
PASS stall (ticks=45, drive=20, scans=1)
PASS scan_timeout (ticks=269, drive=0, scans=0)
PASS stale (ticks=28, drive=3, scans=1)
PASS fault (ticks=24, drive=0, scans=1)
PASS odometry (ticks=24, drive=0, scans=1)
PASS lost (ticks=1809, drive=1452, scans=1)
PASS junction (ticks=1143, drive=902, scans=1)
PASS blocked_junction (ticks=14, drive=0, scans=0)
PASS encoder_wrap (ticks=1190, drive=942, scans=1)
PASS guard_all (ticks=11, drive=0, scans=0)
PASS live_close (ticks=33, drive=1, scans=2)
PASS live_bad (ticks=33, drive=1, scans=2)
PASS live_resume (ticks=1198, drive=942, scans=2)
PASS guard_timeout (ticks=82, drive=0, scans=2)
PASS no_seen (ticks=1337, drive=1071, scans=1)
PASS blind_outer (ticks=82, drive=0, scans=2)
PASS odom_stale (ticks=16, drive=0, scans=0)
PASS one_wheel_stall (ticks=113, drive=65, scans=1)
PASS return_close (ticks=1085, drive=854, scans=2)
PASS pass_close (ticks=554, drive=418, scans=2)
PASS front_wait (ticks=31, drive=0, scans=0)
PASS front_recover (ticks=12, drive=0, scans=0)
PASS rear_reappears (ticks=1293, drive=1025, scans=1)
PASS coasting (ticks=57, drive=0, scans=0)
PASS odom_jump (ticks=26, drive=1, scans=1)
PASS wrong_sign (ticks=26, drive=1, scans=1)
PASS heading_drift (ticks=91, drive=48, scans=1)
PASS barcode_lock (ticks=14, drive=0, scans=0)
PASS echo (RTOS polls=1, writes=7)
PASS echo_wrap (RTOS polls=1, writes=7)
PASS no_echo (RTOS polls=3, writes=7)
PASS stuck_echo (RTOS polls=0, writes=6)
PASS too_far (RTOS polls=3, writes=7)
PASS long_echo (RTOS polls=3, writes=7)
PASS servo (RTOS polls=0, writes=10)
PASS uncalibrated (ticks=24, drive=0, scans=2)
PASS: 51 Buddy 5 host scenarios. Hardware unverified.
```
