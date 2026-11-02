# Buddy 5 — servo ultrasonic scanning and obstacle bypass

The obstacle task owns the HC-SR04 and the SG90 pan servo. The controller
remains the only caller of the motion module and drives the bypass one
non-blocking step per period through `obstacle_bypass_step()`. Buddy 2
motion, Buddy 3 line, barcode and IR code, and Buddy 4 terrain code are
not modified.

## Files

| File | Role |
|---|---|
| `obs_config.h` | Every tunable value; `Measure` marks values to calibrate. |
| `obs_geometry.c/.h` | Sensor mounting transform, pivots and swept-body tests. |
| `scan_plan.c/.h` | Turns one scan into a packed bypass plan; no I/O. |
| `task_obstacle.c/.h` | Front monitor, scan, side watch, front glances. |
| `obstacle.c/.h` | Bypass sequencer and line reacquisition. |
| `obs_state.c/.h` | Status word, event flags, tagged readings, sensor aim. |
| `servo.c/.h`, `ultrasonic.c/.h` | Drivers, unchanged. |
| `test_obs.c/.h` | On-target self-tests and calibration routines. |

## Behaviour in brief

- **Scan.** One right-to-left sweep: far right, the ±40° profile in 10°
  steps, far left. The far bearing is `OBS_SCAN_FAR_DEG` (75°) cut back to
  the servo limits, so the arc is 140° at the present 70° limits and 150°
  once 75° is confirmed on the robot. A bearing stops pinging once two
  pings agree within 30 mm. The servo returns to FRONT and settles before
  the plan is released.
- **Readings.** Every published ping carries the bearing the servo really
  had, what it found (echo, nothing in range, or no echo) and its time.
  The sequencer uses a side ping only if it was taken at the bearing it
  asked for, after it asked. A missing echo is never free space.
- **Departure.** For each side the planner plays the whole manoeuvre
  through with the full body rectangle: pivot out, diagonal, pivot back,
  pass. A side whose outer bearings gave no echo at all is not verified.
- **Passing.** The offset keeps half the body, the 65 mm body margin and
  the 40 mm position allowance from the object's edge. Side echoes raise
  the offset to an absolute target, never cumulatively, and only within
  the room the scan checked.
- **Rear edge.** The object must be seen from the side, then seen clear
  several times over 110 mm of axle travel. The assumed depth stands until
  a clear side view reaches past it.
- **Return.** The car turns back once its rear is 45 mm past the object's
  estimated rear and a fresh look down the way back, plus a swept-body
  check of the pivot and the run to the line, finds nothing in the way.
  Otherwise it passes on 100 mm and looks again.
- **Line.** Black counts as the original line only within 120 mm of where
  the line should be, so crossings and stray marks are ignored.

## Calibrate before driving

Run `OBS_TEST_SERVO_CAL` and `OBS_TEST_FIND_LIMITS` from `test_obs.c`,
then set `OBS_SERVO_FRONT_USEC`, `OBS_SERVO_LEFT_SIGN`, the two servo
limits, `OBS_SENSOR_LATERAL_MM`, `OBS_SERVO_ARM_MM` and
`OBS_SENSOR_HEIGHT_MM`, and set `OBS_GEOMETRY_MEASURED` to 1. Until then the
task prints a warning at startup.

## Tests

```sh
python tools/tests/run_buddy5_clearance.py --cc gcc
```

This runs the geometry, servo and planner checks, the sequencer checks,
and a closed-loop host simulation of 20 scenarios in both bypass
directions in three builds. These are host simulations, not robot trials.

See `docs/BUDDY5_GEOMETRY_CLEARANCE_REPORT.md` for the full design, the
geometry assumptions, test results and what remains unverified.
