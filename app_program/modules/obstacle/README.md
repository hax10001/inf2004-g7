# Buddy 5: ultrasonic obstacle sensing and recovery

This module was developed from the archived `origin/adib-obs` branch while
preserving the uploaded `improve-movement` application as the baseline.

**The subsystem is disabled by default:** `APP_OBSTACLE_ENABLED` is zero in
`app/app_config.h`. This is deliberate because hardware configuration and
collision clearances have not been checked on the physical robotic car.
Complete the hardware checklist in `docs/BUDDY5_IMPLEMENTATION_REPORT.md`,
then select `OBSTACLE=1` in the Make build for supervised bring-up. The Make
option overrides the header default and creates a separate `_obs` image.

- `ultrasonic.c`: GP4 trigger, GP5 gated echo pulse width counting, timeout.
- `servo.c`: GP15 PWM servo timing and configurable angular calibration.
- `task_obstacle.c`: front monitor, coarse 30-degree scan, fine 10-degree
  scan, filtered samples, and ongoing front/inner/outer panning in every
  moving phase. The task never commands motors.
- `obs_geom.c`: measured footprint, sensor offset and rear-swing bounds.
- `scan_plan.c`: conservative approximate obstacle geometry and packed
  two-word bypass plan, including closest direction and radial side depths.
- `obs_state.c`: mutex-protected 32-bit status and 8-byte plan handoff,
  eight-byte live-coverage epochs, front freshness and fault bitmask.
- `obstacle.c`: bounded, non-blocking-by-controller-period manoeuvre
  sequence, encoder-based progress, one stationary rescan on no route,
  bounded movement permissions, measured rear travel, line recovery and
  fail-safe timeouts. There is no blind encoder-only passing fallback.
- `test_obs.c`: non-mutating scan and packed field tests in `TEST=1` builds.

The vehicle controller retains exclusive movement authority. It invokes
`motion_drive()` only through the existing Buddy 2 motion subsystem's new
small bounded forward-wheel-target interface. No reverse motor command is
implemented: the baseline's motor driver is forward-only. If the bypass
cannot be planned, a single stationary rescan is allowed, followed by STOP.

Measured obstacle width/clearances are **estimates**, not exact 3D shape or
proof that a lane is free. Actual system performance depends on sonar beam
spread, servo calibration, robot dimensions, wheel encoders, and environment.

Host logic tests: `python tools/test_buddy5.py`.
Formatting check: `python tools/check_app_standard.py`.
Existing units: `python tools/test_app.py --cc gcc --units-only`.
Payload measurements: `python tools/measure_buddy5.py --cc gcc`.
Historical full trace comparisons use earlier driving/startup settings.

Follow the report's hardware safety checklist before enabling or flashing.
