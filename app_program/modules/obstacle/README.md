# Buddy 5: scanning, avoidance and line recovery

The current `buddy5` branch integrates this subsystem through the existing
vehicle controller, Buddy 2 speed controller and Buddy 3 IR sampler.
Build with `OBSTACLE=1` for the separate obstacle-enabled UF2 image.

Front detection starts avoidance at 250 mm or closer, measured from the
ultrasonic sensor. The three-reading median must be near for three decisions;
one short spike does not start a sweep. The servo parks forward at startup
and only sweeps after a confirmed near obstacle. Invalid, missing and
out-of-range responses leave normal line following running.

If checked turning geometry finds no route in a valid profile, result `3`
(`BEST_EFFORT`) chooses the side with the larger minimum measured range
at 40-70 degrees. Ties choose left. It attempts a 60-degree bypass using
the estimated object edge to set its travel distance. This is an attempt,
not a verified collision-free path.

If scanning, sensing, odometry or the manoeuvre fails, the controller
clears the obstacle fault and resumes the current line-following action.
The sensor task waits one second before starting another avoidance attempt.
The barcode avoidance gate is released during this handoff. An unavailable
obstacle sensor/task does not prevent application startup or line following.
Fresh IR checks and the original IMU startup calibration remain in place.
The stationary sweep and turns can still pause the wheels briefly; failures
no longer latch the controller in its obstacle error state.

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
  rear clearance, line recovery and fallback to normal line following.
- `obs_config.h`: geometry, sensor calibration, margins, speeds and timeouts.

The controller is the sole motor-command owner. All movement goes through
Buddy 2. Reverse is unsupported. Unusable scan data returns control to the
line follower rather than causing repeated stationary scans.
Barcode capture pauses during bypass without losing completed results.

A missing echo is unknown, never free space. One scanning sonar cannot
verify the complete side/rear or vertical environment. Servo position has
no feedback, and line identity is inferred from the return path and IR
confirmation. Physical avoidance and recovery remain unverified.

Host regression tests run the real monitor, planner, sequencer and vehicle
controller with simulated hardware. These check 200/250/251 mm boundaries,
invalid readings, startup, fault recovery and both best-effort wheel commands:

```powershell
python tools/check_app_standard.py
python app_program/modules/obstacle/host/run_tests.py --cc gcc
```
