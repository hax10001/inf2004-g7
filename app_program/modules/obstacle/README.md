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
2. **Creep, then scan:** at the first stop the robot creeps forward at
   `OBS_APPROACH_RPM` until the face is `OBS_SCAN_FROM_MM` (150 mm) away
   (bounded by `OBS_APPROACH_*`, stopping if the echo is lost): from closer
   the square-on zone covers less of the face, so the corners of a box only
   a little off the line can be heard (log: `Creeping closer…`,
   `Crept … mm`). Then the servo sweeps −70° to +70°, every 5° within ±50° (the
   square-on zone ±30° and the corner band beyond it) and 10° outside
   (`OBS_SCAN_*`, `OBS_CORNER_STEP_DEG`), with `OBS_SCAN_PINGS` (2) pings per bearing, 60 ms
   apart, starting at the left end (+70°) and sweeping right. It then sweeps
   back, right to left, over ±30° only (`OBS_SCAN_RETURN_DEG`), keeping
   the nearer reading of each bearing so servo lag cancels where the edges
   are found. About 5 s in all. The reported object width allows for the
   ultrasonic beam (`OBS_BEAM_HALF_DEG`, 10°): a bearing echoes while any part
   of its beam is on the object, so raw edges look about 10° too wide each
   side. Bearings 5° apart limit it to roughly ±3–4 cm. The bypass itself
   still plans from the wider, conservative edges. Flat faces stop
   echoing beyond about 30°, so wide obstacles look narrower than they are;
   `OBS_MIN_LATERAL_MM` (245) sets a floor on the passing offset.
   **Why a box can look centred when it isn't:** the sensor faces the box
   square on, so the flat front face acts like a mirror. Every bearing
   within about ±30° gets the same perpendicular echo wherever the box
   sits, and the echo stops at about ±30° whatever the box's width (logged:
   box on the left read −30..30° every time). Only the box's vertical
   corners echo beyond that, weakly. So at the corner bearings (±40°, ±50°:
   `OBS_CORNER_MAX_DEG`) each bearing takes up to `OBS_CORNER_PINGS` (4)
   pings until two agree on an echo. A corner's sideways position comes
   from its range: √(range² − face²). An echo counts as a corner only
   within `OBS_CORNER_DEPTH_MM` (150) of the face range and at least
   face·tan(bearing − `OBS_CORNER_BEAM_DEG`) out, so a face that isn't
   square on and keeps echoing at 40° is not mistaken for one. The robot
   seldom stops square to the box (logged: the face normal wanders ±5°,
   and sat 3–10° right through one session of box-on-left runs), and where
   the normal leans, the face keeps echoing further out on that side. So
   each scan measures the normal from its own ranges (`OBS_FACE_BOWL_MM`,
   the centre of the bearings within 10 mm of the nearest echo) and corner
   bearings count only from `OBS_CORNER_FROM_DEG` (35°) off it. A corner
   heard on a single ping is kept when the next corner bearing heard the
   same range (`OBS_CORNER_NEIGHBOUR_MM`). The log prints `Square to the
   face at … deg`. About 6 s per scan, 7 s when the corner band is silent.
3. **Choose a side (least resistance):** with `OBS_PREFERRED_SIDE=0`, the
   planner works out how far sideways each side needs the robot to move and
   takes the clear side needing less outside the comparison band. Within
   the band it checks the nearest and average room past each edge.
   `OBS_FRESH_SCAN_CENTRE=1` starts each scan with a new range at servo zero.
   `OBS_FORWARD_EDGE_SPLIT=1` projects corner evidence into the robot's
   straight-ahead frame (`OBS_SERVO_FRONT_USEC` marks 0°). A longer terminal
   echo consistent with the face and the beam can break a tie. The log
   reports `Front split` with an echo estimate when an edge is inferred.
   Both corners must be confirmed to shorten the planned clearance.
   The projected box includes its rear-edge shift and margin, and the
   narrower path must pass the corridor check. Wide initial passes on
   either side credit half the parallel pivot
   (`OBS_WIDE_PASS_PIVOT_GAIN_PCT`).
   If both sides are blocked
   the planner reports `NO_ROUTE`. A mid-bypass rescan keeps the side already
   taken, or stops. Each bearing takes 2 pings; if they differ by more than
   `OBS_SCAN_AGREE_MM` (50) a third decides (median), so a stray echo or an
   edge that echoes only some of the time cannot swing the choice.
   **Servo direction:** keep `OBS_SERVO_LEFT_SIGN` at 1. With −1 every
   bearing is mirrored: the scan puts the box on the wrong side and the side
   watch looks away from the box while passing.
4. **Pass:** the servo watches the obstacle's side. Acquiring it needs
   `OBS_SIDE_SEEN_HITS` (2) echoes. An end candidate needs `OBS_SIDE_CLEAR_HITS`
   (5) consecutive absent samples. A returning echo cancels clearance. Side
   samples older than `OBS_SIDE_MAX_AGE_MSEC` (1000) cannot authorise a return,
   and a stalled worker faults. `OBS_PASS_FRONT_GLANCE=0` means no front checks
   while passing. **Default length:** if the side watch cannot confirm the end
   (it never saw the object, saw it only once, or noise keeps breaking the
   absence run) and hasn't seen it in the last 5 pings, the sensor passing
   the larger of `OBS_PASS_DEFAULT_MM` (40) and the object's beam-corrected
   scanned width (depth is not measured; at most `OBS_OBJECT_MAX_MM`, 150)
   beyond the front face stands in for the end. **Steering:** while passing,
   the robot only holds its offset or turns out, never in towards the
   object (encoder odometry can overstate how far out it is). A side echo
   showing the gap short of half the body plus `OBS_SAFETY_MARGIN_MM` turns
   it out at once (`OBS_SIDE_PUSH_GAIN_DDEG_PER_MM`). An
   object still being seen keeps the leg going. `OBS_PASS_MAX_MM` (1200) is the
   hard limit: past it the robot stops with a `NO_ROUTE` fault.
5. **Return:** the turn back (`OBS_RETURN_DEG`, 60°) starts once the measured gap
   and the swept body clear the obstacle's rear by `OBS_SAFETY_MARGIN_MM` (40),
   using the actual sensor position (`OBS_SENSOR_TO_AXLE_MM` 110,
   `OBS_ROBOT_REAR_MM` 65). IR detection corrects the odometry estimate. The
   robot turns along the line before returning control to line following. If
   no black is seen within `OBS_VERIFY_CYCLES`, the robot zig-zags at
   `OBS_SEEK_DEG` across ±`OBS_SEEK_BAND_MM` for up to `OBS_SEEK_LEGS` legs,
   then stops with `LINE_LOST`.

With `OBS_DEBUG_LOG=1` the log shows `[OBS] Side need L=… R=… mm, clear L=… R=…,
<reason> -> <side>` after each scan, and `[OBS] Pass … by side watch|default
distance: sensor N mm past face …` when the straight leg ends and when the
robot turns back.

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
