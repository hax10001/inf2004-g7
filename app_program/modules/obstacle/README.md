# Obstacle avoidance (Buddy 5)

Servo-scanned ultrasonic obstacle detection, a sideways bypass, and return to
the original line. Compiled in only when `APP_OBSTACLE_ENABLED=1` in
`app_program/app/app_config.h` (default 0). It is always off in the
`G7_BARCODE_TEST=1` build. All tunables are in `obs_config.h`, which explains
each one; values below are the committed defaults.

| File | Role |
| --- | --- |
| `ultrasonic.c/.h`, `servo.c/.h` | Ultrasonic ranging (TRIG GP4, ECHO GP5, echo timed by a PWM counter, no GPIO interrupt) and the scan servo (GP15) |
| `task_obstacle.c/.h` | Front monitor, panning scan, side watch and front watch. Sole owner of the sensor and servo; never drives motors |
| `scan_plan.c/.h` | Edge finding and bypass geometry from the scan's range profile |
| `obstacle.c/.h` | Lifecycle and the bypass sequencer run by the controller |
| `obs_state.c/.h` | Packed status word, event flags and plan hand-off |
| `test_obs.c/.h` | On-target self-test (`make TEST=1`) |

## Behaviour

1. **Detect:** a front reading below `OBS_STOP_MM` (250 mm), confirmed by a
   median of `OBS_MEDIAN_SAMPLES` and `OBS_CONFIRM_HITS` consecutive hits, stops
   the robot.
2. **Scan:** the servo sweeps both extremes, −70° to +70° in 10° steps
   (`OBS_SCAN_*`), with `OBS_SCAN_PINGS` pings per bearing. Flat faces stop
   echoing beyond about 30°, so wide obstacles look narrower than they are;
   `OBS_MIN_LATERAL_MM` (245) sets a floor on the passing offset.
3. **Choose a side:** `OBS_PREFERRED_SIDE=0` takes the clear corridor that needs
   the smaller sideways move. If both sides are blocked the planner reports
   `NO_ROUTE`. A mid-bypass rescan keeps the side already taken, or stops.
4. **Pass:** the servo watches the obstacle's side. Acquiring it needs
   `OBS_SIDE_SEEN_HITS` (2) echoes. An end candidate needs `OBS_SIDE_CLEAR_HITS`
   (5) consecutive absent samples. A returning echo cancels clearance. Side
   samples older than `OBS_SIDE_MAX_AGE_MSEC` (1000) cannot authorise a return,
   and a stalled worker faults. `OBS_PASS_FRONT_GLANCE=0` means no front checks
   while passing. If the object is never seen from the side,
   `OBS_UNSEEN_DEPTH_MM` (300) past its front face stands in for its end.
   `OBS_PASS_MAX_MM` (1200) bounds the pass.
5. **Return:** the turn back (`OBS_RETURN_DEG`, 60°) starts once the measured gap
   and the swept body clear the obstacle's rear by `OBS_SAFETY_MARGIN_MM` (40),
   using the actual sensor position (`OBS_SENSOR_TO_AXLE_MM` 110,
   `OBS_ROBOT_REAR_MM` 65). Odometry decides where the line should be. If no
   black is seen within `OBS_VERIFY_CYCLES`, the robot zig-zags at
   `OBS_SEEK_DEG` across ±`OBS_SEEK_BAND_MM` for up to `OBS_SEEK_LEGS` legs,
   then stops with `LINE_LOST`.

STOP, MQTT-loss STOP and the existing travel and time bounds stay active
throughout. "Where the line should be" is the straight extension of the line at
the point of detection: a curve after the obstacle is only found by the IR seek.

## Limits and tests

A missing echo can mean open space, a weak reflection or a sensor fault. The
geometry constants (`OBS_ROBOT_*`, `OBS_SENSOR_TO_AXLE_MM`, servo limits) must
be verified on the car. Servo reach, acoustic dropout and real stopping
distance are untested on hardware.

Host coverage runs in `tools/run_host_tests.sh`, via
`tools/test_obstacle_integration.py`. That covers the planner and side
clearance (`tools/tests/run_buddy5_clearance.py`), the worker, return geometry,
controller handoff and motion. Development history is in
[docs/CHANGELOG.md](../../../docs/CHANGELOG.md).
