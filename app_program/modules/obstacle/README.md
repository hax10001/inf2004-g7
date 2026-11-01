# Buddy 5: scanning, avoidance and line recovery

The current `buddy5` branch integrates this subsystem through the existing
vehicle controller, Buddy 2 speed controller and Buddy 3 IR sampler.
Build with `OBSTACLE=1` for the separate obstacle-enabled UF2 image.

Front detection starts avoidance at 300 mm or closer, measured from the
ultrasonic sensor. The first near sample stops forward permission while a
three-reading median confirms the obstacle. At 250 mm or closer, a raw
return immediately triggers the obstacle stop. Invalid, missing or stale
front readings withhold motion; sustained sensor failure latches a fault.

The planner checks 30, 45, 60 and 90-degree detours. A 90-degree route allows
sideways travel before passing the obstacle, with a bound for the whole
vehicle's two turns and a two-degree sideways heading limit. It chooses a
verified left or right corridor. An unchecked best-effort route is never
executed. If neither side fits, it rescans once while stopped, then latches
NO_ROUTE and stays stopped until reset.

Failed scanning, sensing, odometry or line recovery cannot return movement
authority to normal following. Faults survive cancellation. An unavailable
enabled obstacle sensor/task fails startup. There is no detection cooldown.
Only successful clearance/recovery hands control back, after fresh centred
sonar. Normal IR and IMU startup checks remain in place.

- `ultrasonic.c/.h`: GP4 trigger, GP5 input-only PWM pulse measurement,
  hardware-timer deadline, explicit invalid results and median filtering.
- `servo.c/.h`: GP15, 20 ms PWM, bounded pulse width and settling time.
- `task_obstacle.c/.h`: sole sensor owner; forward monitoring, coarse and
  fine scanning, and continuing front/inner/outer sampling during bypass.
- `scan_plan.c/.h`: compact samples, approximate profile, conservative
  entry and side corridors, side selection and packed plan.
- `obs_geom.c/.h`: footprint, swept turns, rear bound and integer heading math.
- `obs_state.c/.h`: protected packed status, plan and coverage epochs;
  reuses the existing IR mutex, adding one event flag and no mutex.
- `obstacle.c/.h`: encoder-guided bypass, bounded movement permissions,
  rear clearance, estimated original-line recovery and bounded safe stops.
- `obs_config.h`: geometry, sensor calibration, margins, speeds and timeouts.

The controller is the sole motor-command owner. All movement goes through
Buddy 2. Reverse is unsupported. Unusable scan data keeps the car stopped.
Barcode capture pauses during bypass without losing completed results.

Current geometry uses a centred sonar 110 mm ahead of the axle. The supplied
front/rear offsets are 135/65 mm, so the conservative total is 200 mm, despite
the earlier 190 mm overall estimate. Width is 153 mm, track 110 mm and wheel
diameter 70 mm. The rear-clearance travel bound is now 823 mm after an
observed edge, using 95% of the smaller wheel travel.

Encoder heading and travel track lateral displacement from the starting
line. Early stripes outside the return region are ignored. Both black can
identify a transverse crossing while searching; final handoff requires a
stable normal IR edge. Lost confirmation stops after 4 seconds or 50 mm of
verification travel. The two IR sensors cannot uniquely identify arbitrary
competing stripes; physical alignment still requires supervised checks.

A missing echo is unknown, never free space. One scanning sonar cannot
verify the complete side/rear or vertical environment. Servo position has
no feedback, and line identity is inferred from the return path and IR
confirmation. Physical avoidance and recovery remain unverified.

Host regression tests run the real monitor, planner, sequencer and vehicle
controller with simulated hardware. These cover 200/250/251/299/300/301 mm
boundaries, safe left/right 90-degree bypasses, rear clearance, live hazards,
line recovery, drift and persistent stops. Detailed results are in
[the implementation report](../../../docs/BUDDY5_IMPLEMENTATION_REPORT.md).

```powershell
python tools/check_app_standard.py
python app_program/modules/obstacle/host/run_tests.py --cc gcc
python tools/test_app.py --cc gcc --units-only
```
