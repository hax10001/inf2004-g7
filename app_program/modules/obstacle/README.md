# Buddy 5: scanning, avoidance and line recovery

The current `buddy5` branch integrates this subsystem through the existing
vehicle controller, Buddy 2 speed controller and Buddy 3 IR sampler.
`APP_OBSTACLE_ENABLED` defaults to zero. Build `OBSTACLE=1` only after the
electrical, servo, encoder and stopping checks in the
[implementation report](../../../docs/BUDDY5_IMPLEMENTATION_REPORT.md).

- `ultrasonic.c/.h`: GP4 trigger, GP5 input-only PWM pulse measurement,
  hardware-timer deadline, explicit invalid results and median filtering.
- `servo.c/.h`: GP15, 20 ms PWM, bounded pulse width and settling time.
- `task_obstacle.c/.h`: sole sensor owner; forward monitoring, coarse and
  fine scanning, and continuing front/inner/outer sampling during bypass.
- `scan_plan.c/.h`: compact samples, approximate profile, conservative
  entry and side corridors, side selection and packed plan.
- `obs_geom.c/.h`: vehicle envelope, angular uncertainty and rear-turn bound.
- `obs_state.c/.h`: protected packed status, plan and coverage epochs;
  reuses the existing IR mutex, adding one event flag and no mutex.
- `obstacle.c/.h`: encoder-guided bypass, bounded movement permissions,
  stationary reassessment, rear clearance, line recovery and safe failures.
- `obs_config.h`: geometry, sensor calibration, margins, speeds and timeouts.

The controller is the sole motor-command owner. All movement goes through
Buddy 2, including immediate stops. Reverse is unsupported; one stationary
retry is followed by a persistent stop if no route can be verified.
Barcode capture pauses during bypass without losing completed results.

A missing echo is unknown, never free space. One scanning sonar cannot
verify the complete side/rear or vertical environment. Servo position has
no feedback, and line identity is inferred from the return path and IR
confirmation. Physical avoidance and recovery remain unverified.

Development-only Buddy 5 harnesses were removed after verification.
The report retains the recorded results and minimum-bit evidence. Team
checks remain available from the repository root:

```powershell
python tools/check_app_standard.py
python tools/test_app.py --cc gcc --units-only
```
