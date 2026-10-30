# Buddy 5 continuous-monitoring test evidence

Captured 30 October 2026 on `adibbest`. No robot was flashed or physically
tested. This document replaces the earlier stationary/partial-monitoring
results. See [the implementation report](BUDDY5_IMPLEMENTATION_REPORT.md)
for measured geometry, assumptions and unrun hardware tests.

## Toolchains and profiles

- ARM: `arm-none-eabi-gcc.exe (GNU Arm Embedded Toolchain 10.3-2021.10) 10.3.1 20210824 (release)`
- Host: `gcc (Rev2, Built by MSYS2 project) 14.2.0`
- Cortex-M0+ production compile checks: **26 files**, both OBSTACLE=0 and
  OBSTACLE=1, using `-std=c99 -Wall -Wextra -Werror`.
- Make profiles: SMP=1, UART console, enabled/disabled/TEST=1 as below.
- Application standard checker: **63 files, zero errors**. It checks a
  defined subset of BARR-C, not complete certification.
- `git diff --check`: pass. Final application blank lines are required by
  the project's BARR-C formatting rule.

## Host scenarios

Command: `python tools/test_buddy5.py --cc gcc`

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
PASS left (ticks=1024, drive=805, scans=1)
PASS right (ticks=1024, drive=805, scans=1)
PASS wrap (ticks=1025, drive=805, scans=1)
PASS clear (ticks=11, drive=0, scans=1)
PASS no_route (ticks=24, drive=0, scans=2)
PASS stall (ticks=45, drive=20, scans=1)
PASS scan_timeout (ticks=269, drive=0, scans=0)
PASS stale (ticks=28, drive=3, scans=1)
PASS fault (ticks=24, drive=0, scans=1)
PASS odometry (ticks=24, drive=0, scans=1)
PASS lost (ticks=1461, drive=1165, scans=1)
PASS junction (ticks=977, drive=765, scans=1)
PASS blocked_junction (ticks=14, drive=0, scans=0)
PASS encoder_wrap (ticks=1024, drive=805, scans=1)
PASS guard_all (ticks=11, drive=0, scans=0)
PASS live_close (ticks=33, drive=1, scans=2)
PASS live_bad (ticks=33, drive=1, scans=2)
PASS live_resume (ticks=1032, drive=805, scans=2)
PASS guard_timeout (ticks=82, drive=0, scans=2)
PASS no_seen (ticks=1216, drive=971, scans=1)
PASS blind_outer (ticks=82, drive=0, scans=2)
PASS odom_stale (ticks=16, drive=0, scans=0)
PASS one_wheel_stall (ticks=117, drive=68, scans=1)
PASS return_close (ticks=919, drive=717, scans=2)
PASS pass_close (ticks=433, drive=318, scans=2)
PASS front_wait (ticks=31, drive=0, scans=0)
PASS front_recover (ticks=12, drive=0, scans=0)
PASS rear_reappears (ticks=1127, drive=888, scans=1)
PASS echo (RTOS polls=1, writes=7)
PASS echo_wrap (RTOS polls=1, writes=7)
PASS no_echo (RTOS polls=3, writes=7)
PASS stuck_echo (RTOS polls=0, writes=6)
PASS too_far (RTOS polls=3, writes=7)
PASS long_echo (RTOS polls=3, writes=7)
PASS servo (RTOS polls=0, writes=10)
PASS uncalibrated (ticks=24, drive=0, scans=2)
PASS: 46 Buddy 5 host scenarios. Hardware unverified.
```

The real scanner runs in the sensor scenarios, including sampling with
nonzero simulated wheel commands. Controller scenarios run the real
controller, manoeuvre sequencer, geometry, profiling and shared-state code,
with deterministic sensor/odometry/motion/RTOS dependencies. The IO cases
run the real sonar and servo drivers against simulated registers and
10 ms RTOS ticks. The profile case also runs all four on-target obstacle
checks and checks epoch rejection, compact snapshots and front freshness.

Assertions prevent turn-back before the computed rear-clearance distance
has been travelled by both wheels, reject unobserved side edges and require
fresh complete coverage. Live hazard cases cover turns, passing and return.
The rear_reappears case requires the rear-clearance window to restart.
The uncalibrated build confirms that invalid geometry cannot authorise a
bypass. These tests are not physical measurements of collision clearance.

## Existing subsystem checks

Command: `python tools/test_app.py --cc gcc --units-only`

```text
UNIT_CHECKS_PASS: IR, mutex failures, all 65536 register words, 4096 magnitude vectors, jerk boundaries
PASS: TEST=1 host link and 11 non-mutating subsystem checks.
SKIP: historical captured traces use previous operating settings.
```

This also checks the actual Buddy 2 bounded wheel interface and immediate
stop, plus Buddy 3 IR snapshots/latches and unchanged decision truth table.

## ELF / UF2 builds

All three links and UF2 packaging steps passed. GNU size output in bytes:

| Profile | text | data | bss | UF2 SHA-256 |
| --- | ---: | ---: | ---: | --- |
| `mtk3pico_smp1_uart` | 77636 | 1088 | 13928 | `3bdb8329a6f482f0d556f31f1e7d3923066ef35e0a93ecf0e00747d669af1125` |
| `mtk3pico_smp1_uart_obs` | 82608 | 1088 | 13992 | `061645af4330f495d60fc228cc63a8c1bd3d78b3ddece925e8f3c85b6bb880da` |
| `mtk3pico_smp1_uart_obs_test` | 83860 | 1088 | 13992 | `9090b2f96b778d9d33fdff84ee6320f910ca9f9689c16e7d3d14078dea4aaed1` |

`data+bss` is static storage, not runtime stack/heap usage. Feature-profile
differences are not before/after packing savings. UART uses encoder pins on
this car: these are compile/link artifacts, not USB console qualification
or evidence of a physical drive. The configured USB SDK is unavailable.

## Representation sizes

`python tools/measure_buddy5.py --cc gcc` was also run with the ARM compiler
and `--arm-nm`; the host and ARM measurements agree.

| Object | Unpacked / original bytes | Compact bytes |
| --- | ---: | ---: |
| scan_sample | 8 | 2 |
| scan_array | 120 | 30 |
| status | 24 | 4 |
| plan | 44 | 8 |
| drive | 12 | 4 |
| ir | 20 | 8 |
| live_guard | 28 | 8 |

Drive and IR use their actual original field layouts. Other comparisons
use equivalent unpacked payloads, not historical firmware images. New
coverage has three range fields, validity, a hazard reason, epoch and time,
packed in eight bytes under a mutex. Complete project-wide representation,
whole-firmware before/after RAM, binary MQTT payload and target timing
measurements remain team-wide work.

## Unrun physical checks

All physical rows in the implementation report remain **Not run**. They
include braking, servo uncertainty/settling, encoder scale/slip, left/right
bypass, faults during motion, complete rear-swing clearance and reliable
recovery of the original line. No collision-free hardware guarantee or
full BARR-C compliance certification is inferred from these software tests.
