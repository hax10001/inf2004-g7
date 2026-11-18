# Changelog

Condensed from the release notes and patch READMEs that used to sit in the
repository root. Most releases were distributed as ZIP snapshots or patch ZIPs,
which is why some entries mention installers and backup folders. Detailed
evidence for V9 onwards (test logs, size audits, checklists) is in
`docs/V9/` … `docs/V15/`. Test counts are what each release reported at the time.

## Unreleased

- Fixes after the 22:27 robot run:
  - Line following PID: the left encoder counts down when driving forward
    (logged encL=-53231, encR=49602). The observer used signed counts and
    saw the robot spinning on the spot, so it alternated the wheels. It
    now uses count sizes, as the obstacle odometry does.
  - `STEERING_OBSERVER 0` is now exactly the committed PI steering (KP 0.3,
    KI 2.0, decay 0.85, no lost-line search): the steering barcodes were
    read well with. The barcode code is unchanged.
  - Obstacle creep off (`OBS_SCAN_FROM_MM 0`): it left less room to clear
    the box than the version that cleared it well.
  - Obstacle side: edges compared from the face's normal (tilt measured up
    to 25 degrees over the whole object), so a robot stopped turned no
    longer makes the box look longer on one side; within 45 mm the box is
    about centred and the side with more room past it wins. Replayed on
    the 22:27 scans: 6 of 7 now go left (the open side), against 0.

- Line following: PID with encoder D (`STEERING_OBSERVER`). The steering
  had always been PI on a -1/0/+1 error; the zig-zag grew as the speed went
  15 -> 25 -> 35 RPM (18 Nov). The encoders now follow the line through
  the gap between the sensors, so P, I and D act on a real offset and
  heading. Closed-loop model, 35-65 RPM and plant variants: straight-line
  wander 4.1 mm RMS against 8.0 (3.2 mm at 35 RPM, the old steering's 15
  RPM level), runs completed 98 % against 95 %. Sharp corners rely on the
  lost-line pivot. `STEERING_OBSERVER 0` restores the PI.
- Obstacle: creep to `OBS_SCAN_FROM_MM` (150 mm) before the first scan, so
  a box only ~2 cm off the line has its far corner outside the sensor's
  square-on zone and is located by it (from ~215 mm, boxes up to ~5 cm off
  read as centred). Bounded by `OBS_APPROACH_*`; stops on lost echoes.

- Line following at speed and round corners (motion and controller; done
  on request). Checked in a new closed-loop model
  (`tools/tests/line_follow_sim.c`: the real motion, steering, IR latches
  and navigation on a simulated robot, 1 ms IR sampling, 18 mm tape),
  randomised over start pose, motor gains and lag, tape edge and controller
  phase, and repeated for sensor gaps of 20-34 mm, tracks of 110-130 mm and
  motor lags of 0.07-0.15 s. Courses completed, all six courses, 35-65 RPM:
  38 % before, 96 % after.
  - Steering: `STEERING_KP` 0.3 -> 0.08, `STEERING_KI` 2.0 -> 7.0,
    `STEERING_DECAY` 0.85 -> 0.96. The sensors are 125 mm ahead of the
    axle, so the old kick swung them across the 8 mm between them within a
    couple of periods; the weave grew with speed until the tape escaped.
  - Steering is counted in distance, not time (`STEERING_REF_RPM` 35): the
    integral, its decay and the hold/bend windows scale with speed.
    `MOTION_TARGET_RPM` can be overridden at build time.
  - Lost-line search (`MOTION_SEARCH_*`): after 100 mm with no line, pivot
    towards where it was last seen (120 degrees), then back, then stop.
    Sharp 90 degree corners: 0 % before, 70-95 % after.
  - The controller reports the line each period (`motion_line_seen()`), so
    the search knows where the line was while navigation drives straight
    over a both-black run, and a turn still held the other way is dropped
    there.

- Obstacle, box off to the left (obstacle module only). Box-on-right was
  going round correctly every time but box-on-left only 4 in 10. In 92
  logged scans the square-on echo zone sat 2.3 degrees right on average and
  its left end was cut short 28 times against 3 on the right; corner echoes
  were heard 49 times on the right and 21 on the left. The zone follows the
  angle the robot stops at, not the box, so a box 3-7 cm left (its far
  corner 26-35 degrees out) was invisible:
  - The corner band is sampled every 5 degrees (±35, 40, 45, 50), where
    that corner is; nothing was sampled between 30 and 40.
  - Each scan measures where it is square to the face and judges corner
    bearings from there (at least 35 degrees off it), so a tilted stop no
    longer fakes a corner on the side the face leans to.
  - A corner heard on a single ping is kept when the next corner bearing
    heard the same range (5 of 32 corner hearings were single pings).
  - A corner echo cut off from the face by a silent bearing is placed by
    its range in the corridor check, and the object's own corner is never
    its own side's blocker.
  - Replayed on all 95 logged scans: no decision changes (old logs have no
    35/45 degree samples). Scan 6 s, 7 s with a silent corner band.

- Obstacle (obstacle module only):
  - Side choice is least resistance: the smaller sideways move wins
    however small the difference. `OBS_SIDE_PREFERENCE_THRESHOLD_MM`,
    `OBS_SIDE_DEFAULT`, `OBS_SIDE_TIE_BREAK` and `OBS_TIE_OPEN_MARGIN_MM`
    are removed. Exactly equal moves go to the side with more room past the
    object, then the smaller drive out, then the more open half of the
    sweep; a mirror-symmetric scan goes by the parity of its readings.
  - Corner echoes locate an off-centre box. Facing a flat face square on,
    every bearing within about ±30 degrees reads the same perpendicular
    distance and the echo stops at about ±30 degrees wherever the box sits,
    so a box on the left read as centred. Corner bearings (±40, ±50) now
    take up to `OBS_CORNER_PINGS` (4) pings until two agree; a corner's
    sideways offset is √(range² − face²). When found, the sides are
    compared on the real extent (`OBS_SIDE_CORNER_ECHO`, `OBS_CORNER_*`),
    and the robot drives out far enough to clear a seen corner.
  - The two-way scan starts at the left end (left→right, then right→left
    over ±30 degrees); 5.5 s in the host model.
  - `OBS_SERVO_LEFT_SIGN` back to 1. With −1 the side watch looked away
    from the box while passing (19:2x log: no box echo on either pass, only
    the open side 0.6–0.9 m out) and scans were mirrored.
  - Replayed on the 11 logged box-on-left scans (18:47): 7 now go right
    (4 before); all 3 with a corner echo go right.

- Obstacle return length: with no confirmed end, the turn back assumes the
  object is as deep as its beam-corrected width, not the padded planning
  width (which hit the 300 mm cap and ran ~230 mm past a 15 cm box in the
  17:44 log), capped at `OBS_OBJECT_MAX_MM` (150 mm, the largest obstacle on
  the course). Never shorter than `OBS_PASS_DEFAULT_MM`; a deeper object is
  still caught by the front guard on the return.

- Obstacle side choice (obstacle module only): the sides are compared on
  beam-corrected edges instead of edges padded to the next empty bearing
  (whose 5/10 degree spacing tilted close calls), with the tie band at
  `OBS_SIDE_PREFERENCE_THRESHOLD_MM` 45 (was 90, tuned for the padded
  edges). Ties go to the side with more open space past the object
  (`OBS_SIDE_TIE_BREAK`), else `OBS_SIDE_DEFAULT`. Replaying 127 logged
  scans: wrong on clearly one-sided scans 5/53 (was 7/53), left/right
  73/20 (was 86/7). The log also prints the object centre and the
  `OBS_SERVO_FRONT_USEC` that would centre it, for servo calibration.
  Drive offsets are unchanged.

- Obstacle bypass, after the 18 Nov collision (video + log): the robot
  reached ~160 mm out by the video while encoder odometry said 231 mm, so
  the pass steered back in towards the box and rode onto its corner.
  - Passing never steers towards the object; it only holds or turns out.
  - A side echo showing the gap short of half the body plus margin turns
    out at once, `OBS_SIDE_PUSH_GAIN_DDEG_PER_MM` (3: 15 deg for 50 mm),
    and moves the offset target out from where the robot is, not
    cumulatively. The push lasts only while echoes keep showing it.
  - With no confirmed end, the turn back now assumes the object is as
    deep as its conservative scanned width (at most `OBS_PASS_MAX_MM` / 2),
    not just `OBS_PASS_DEFAULT_MM` (40 mm).
  - Scan pings: 2 per bearing and a third when they differ by more than
    `OBS_SCAN_AGREE_MM` (median). Keeping the larger of 2 dropped a real
    +20 degree edge (230 mm / no echo) on 18 Nov.

- Reported obstacle width allows for the ultrasonic beam: each edge is
  taken midway to the next empty bearing, moved in by `OBS_BEAM_HALF_DEG`
  (10 degrees, fitted to the logged scans of the 15 cm obstacle). Over the
  122 logged scans the median reported width falls from 280 to 140 mm
  (160 mm on 18 Nov). Reporting only: replaying those scans, every plan
  (route, side, offset, turn angle) is unchanged.

- Obstacle scan in about 5 s (was 16-18 s on the robot): 2 pings per
  bearing instead of 3 (`OBS_SCAN_PINGS`, the larger is used), and the
  return pass re-reads only +-30 degrees (`OBS_SCAN_RETURN_DEG`), where the
  edges are found. Replaying the 117 scans in the robot logs through the
  real planner, both changes gave the same plan as before in every scan;
  one ping per bearing changed the side in 4. Simulated worst case (every
  echo timing out): 34 bearings, 68 pings, 5.5 s.

- Faster obstacle scan with the same measurement: scan pings are spaced
  `OBS_PING_PERIOD_MSEC` (60 ms) trigger to trigger instead of 60 ms after
  each echo ends, the servo moves and settles while the last ping's echo
  dies away, and the two-way sweep no longer pings the turnaround bearing
  twice. Simulated: 41 bearings x 3 pings in ~9.1 s, from at least 13.3 s.
  New `ultrasonic_last_trigger()` reports the last trigger time.

- A barcode capture no longer slows or brakes the robot: barcodes are read
  at the driving speed (`MOTION_TARGET_RPM`). `APP_BARCODE_SCAN_RPM` and
  `APP_BARCODE_SCAN_BRAKE_MSEC` are removed.

- Junction actions back to single-use (`APP_NAV_REPEAT_ACTION 0`; it had
  been left at 1): a barcode's action is done at the next junction and
  then forgotten, so a later junction with no new barcode goes straight.

- Barcode capture gate, tuned on the logged runs (66 decoded, 338 failed
  captures): a capture starts only 600 ADC counts darker than white
  (`APP_BARCODE_TRIGGER_RISE`), and one whose dark part ends under 1000
  units (`APP_BARCODE_MIN_DARK_UNITS`) is dropped without decoding while
  the scanner stays armed. Together they would have removed 252 of the
  failed captures and none of the decoded ones. No new capture during
  junction centring, pivots and reacquisition
  (`APP_BARCODE_SUPPRESS_IN_TURNS`). New `bc_port_nav_abort()` ends a
  dropped capture's navigation transaction with no result.
- Scan-speed entry: when a capture starts above `APP_BARCODE_SCAN_RPM`
  the target is cut straight to scan speed (no 8 RPM/period ramp) and the
  motors short-brake for `APP_BARCODE_SCAN_BRAKE_MSEC` (50 ms) while still
  faster, so the first bars are not read during a coast-down.

- Barcode white reference back to re-learning after every barcode
  (`APP_BARCODE_KEEP_STARTUP_WHITE 0`). Logs show the track's white varies
  ~180..370 ADC within a run; reusing the startup white let lighter floor
  read as bars, and every pass of the 13:27/13:42 runs failed `many_bars`.
  With the option on, the startup white is now reused only within its
  sigma + 8 (it accepted 3 sigma + 24, beyond the 3 sigma + 8 trigger).

- Mid-run stops with no STOP and MQTT connected (logged 13:40–13:41): the
  controller treated one busy period on its STOP/START mailbox as a STOP
  and ended the run silently. It now waits up to 2 ms
  (`APP_REMOTE_LOCK_WAIT_MSEC`) and stops only after 4 busy periods in a
  row (`APP_REMOTE_LOCK_MISS_LIMIT`), with a log line.
- False lift stop on a junction (fired 10 ms after stopping there): the
  height estimate could build up while driving and show the moment both
  line sensors went black. It is now held at 0 while a sensor sees the
  floor and resets after five floor samples. The IMU uses the latest line
  sensor readings and publishes floor contact with the lift estimate.
  A stationary robot held in the air stays paused. Automatic resume requires
  fresh floor readings with at least one sensor seeing white for the full delay.
- Removed IMU collision detection, jerk diagnostics and collision event handling.

- No barcode problem stops a running robot by default: a barcode read too
  close to its junction now drops that action and crosses straight, and a
  failed barcode handoff keeps line following and retries each period.
  `APP_NAV_SCAN_FAIL_STOP 1` restores all the barcode safety stops
  (`nav_config_t.clearance_straight` carries it into `nav_core`).

- Barcode white reference: the scanner learns white once, at the first
  arming after boot, and reuses it after every barcode
  (`APP_BARCODE_KEEP_STARTUP_WHITE 1`; 0 restores re-learning each time).
  A floor that no longer reads as that white uses its own reading for that
  pass, so the scanner cannot lock up triggering on the floor. "Recalibrate
  barcode" replaces the reference.

- A barcode whose letter is not in the route mapping (or with no mapping
  loaded), or a scan result lost to a handoff overflow, no longer causes a
  navigation safety stop: it is ignored like a failed decode and line
  following continues (the overflow mark is reset so later barcodes are
  still read). `APP_NAV_SCAN_FAIL_STOP 1` restores the stop.

- The robot no longer stops when the barcode scanner is still rearming at
  the end of an obstacle bypass, or when it is put down after a lift: it
  carries on line following and the scanner finishes rearming on the move,
  as it already did after every barcode (`g7_nav_continue_run()`). Only a
  failed barcode handoff still stops it. START still waits for an armed
  scanner, so the first barcode of a run is always captured.

- Lift stop: picking the robot up while it may drive cuts the motors
  (fault 14) and pauses it (mission state 4, "LIFTED (PAUSED)"). Put back on
  the line, it resumes line following after `APP_LIFT_RESUME_MSEC` (1000) on
  the floor; STOP or an MQTT loss while lifted ends the run.
  `APP_LIFT_RESUME_ENABLED 0` latches the stop instead (STOP, then START).
  `APP_LIFT_STOP_HEIGHT_MM` (default 40) sets the height that counts as
  lifted and `APP_LIFT_STOP_ENABLED` turns the stop off. The height is an
  accelerometer estimate (`modules/terrain/lift_math.c`), confirmed by both
  line sensors losing the floor, and is shown on the dashboard (Overview,
  Map & robot and Console sensor panels). The SENSOR frame grows by one 16-bit word
  (10-bit height + lifted flag); 24/32-byte frames still decode.
- Minimum-width terrain storage: each hump height uses 7 bits, lift height
  uses 10, floor contact uses 1, and heading uses 16 at telemetry precision.
  Lift confirmation counters use 2, 5 and 3 bits.
- `modules/terrain/host/run_tests.py` links again (missing `tk_slp_tsk`
  stub).

- Obstacle bypass: the straight leg could overshoot to the 1200 mm
  `OBS_PASS_MAX_MM` fault whenever the side watch could not confirm the
  object's end. A single unconfirmed echo turned off the never-seen fallback,
  and a seen object had no fallback at all. The new `OBS_PASS_DEFAULT_MM`
  (which replaces `OBS_UNSEEN_DEPTH_MM`) ends the leg in all of those cases,
  while a confirmed end still finishes earlier.
- Bypass side choice: sides within `OBS_SIDE_PREFERENCE_THRESHOLD_MM` (80 mm:
  the ultrasonic beam cannot resolve offsets of a few cm) count as centred
  and use `OBS_SIDE_DEFAULT`. `OBS_DEBUG_LOG` prints both side
  clearances, the choice, and how and where the straight leg ended.
  The scan log now also shows each bearing's raw pings, tags every sample
  L/R/front and object/near/open, and gives each side's edge, the floor that
  set its need, the open space beyond it and any corridor blocker.
- Junction with no barcode action queued (none read since START, or a
  decode that gave no letter) is crossed straight on instead of a safety
  stop (`APP_NAV_STRAIGHT_DEFAULT=1`; 0 restores the stop).
- Junction actions are single-use by default (`APP_NAV_REPEAT_ACTION=0`):
  a barcode's action is done at the next junction, then discarded. An action
  not yet used now survives an obstacle bypass in single mode too.
- Obstacle scan steps 5 degrees within +-30 degrees of the front
  (`OBS_SCAN_FINE_DEG`, `OBS_SCAN_FINE_STEP_DEG`) and 10 degrees outside: 21
  points instead of 15, so a small object's edges are found to half the
  step. The edge padding uses the real gap to the next sample.
- Side choice compares the measured needs: the `OBS_MIN_LATERAL_MM` floor
  only sets how far the chosen side drives. One sample crossing
  `OBS_MIN_LATERAL_FROM_DEG` used to add ~35 mm to one side and decide it.
- Scan sweeps both ways by default (`OBS_SCAN_DIRECTION=0`) and keeps the
  nearer reading of each bearing, cancelling servo backlash that shifted
  every bearing one way; `1` / `-1` sweep one way (for comparison). The log
  shows both passes. `OBS_SCAN_TIMEOUT_MSEC` allows for the longer scan.
- Ultrasonic waits (up to `OBS_ECHO_CLEAR_MSEC`) for ECHO to fall before
  triggering, so a clone that holds ECHO high after a missed echo cannot
  have the tail of the old pulse timed as a range.
- Bypass return: when straightening up left the IR sensors beside the line
  (both swept over it, or neither reached it), line following drove on along
  the white. The robot now makes a seek leg back towards the line instead.
  `OBS_RETURN_DEG` accepts 30..90 and is checked at compile time.
- Bypass return: once the IR sensors confirm the line on the main return,
  line following takes over and steers onto it (`OBS_RETURN_LINE_FOLLOW=1`)
  instead of an odometry straightening turn. The rejoin check now applies
  only to the main return's straightening turn (at least
  `OBS_ALIGN_CHECK_DEG`), never to seek legs, which had used up all the legs
  on false rejoins and raised `FAULT 0x4`.

- Repository cleanup: the root patch notes were merged into the
  [README](../README.md), [TROUBLESHOOTING](TROUBLESHOOTING.md),
  [DASHBOARD](DASHBOARD.md), [ROBOT_OPERATION](ROBOT_OPERATION.md) and this
  changelog. The version-chained test runners became `tools/run_host_tests.sh`.
- Obstacle avoidance and IMU collision stops are disabled by default
  (`APP_OBSTACLE_ENABLED=0`, `APP_COLLISION_ENABLED=0`). Handling the car while
  stopped no longer counts as a collision, and scan-failure reasons are shown.

## V15.1: Console UI and control refinements

- One **Console** view replaces three aliased monitoring views. The barcode
  pane shows one live-state badge and timestamped letters and rejections
  (`docs/V15/UI_UPDATE.md`).
- START requires the scanner's armed WAIT phase and preserves its learned
  white reference. The IMU hump-descent signal engages the governor, ramping 5
  points per 50 ms to 65%. Obstacle trigger reduced from 350 to 250 mm, scan
  widened to ±70°, the smaller corridor wins, and the return uses the actual
  sensor position plus a 40 mm margin (`docs/V15/CONTROL_REFINEMENTS.md`).
- 78 dashboard tests. ARM target build passed.

## V15: navigation recovery and ramp

- The barcode worker publishes an owned transaction (sequence, capture, decode,
  result, busy) under one semaphore. A valid result completed before a
  junction deadline is accepted even if consumed just after it.
- STOP writes all four motor compares to zero before resetting state; START
  rechecks readiness when executed. Collision acknowledgement compares the
  event sequence. EVENT kind 21 reports a fault source without changing
  payload sizes.
- The PI controller integrates at reduced ramp targets and can request bounded
  short-brake/coast PWM.
- Details: `docs/V15/ROOT_CAUSE.md`, `VALIDATION.md`, `MINIMUM_BITS.md`,
  `BUILD_AND_ROLLBACK.md`, `PHYSICAL_CHECKLIST.md`.

## V14: controller recovery

- Fixed a persistent collision latch that blocked STOP → HOLD → START. The
  collision event is acknowledged under an RTOS mutex. Clearer STOPPED versus
  HOLD reporting. Downhill derating no longer pulses duty to zero or trips on
  short overshoot. 74 dashboard tests, 144 navigation assertions
  (`docs/V14/README.md`).

## V13: calibrated downhill governor

- Controller-owned filtered pitch governor at 65% wheel targets after
  steering; stale-sensor and encoder-overspeed latches; bounded brake. Default
  sign `APP_DOWNHILL_NOSE_DOWN_SIGN=0` disables it until measured. 73 dashboard
  tests (`docs/V13/README.md`).

## V12: scanner card, collision configuration

- Scanner-card cleanup and mapping-rejection diagnostics. The normal MQTT build
  enables the IMU (`APP_IMU_ENABLED=1`) and the collision latch; the barcode
  test build keeps it off. The downhill change was prepared but not activated
  (`docs/V12/`).

## V11: minimum-bit expansion and precision review

- Native `B1_Store` cut from 1152 to 760 bytes; compact obstacle, IR, terrain,
  motion, barcode and 12-bit ADC records; a duplicate barcode configuration
  removed. MQTT v1 wire format unchanged. 70 dashboard tests.
- Precision addendum: bit-fields for steering `last_side`/`bend_side`,
  `bc_match` support, `bc_layout` bars/module width and the terrain calibration
  flag, with `tools/test_precision_ranges.sh` (`docs/V11/PRECISION_REVIEW.md`).

## V10 and V9: unified monitoring, minimum-bit checkpoints

- V9: unified live-monitoring dashboard; compact internal data; macOS host
  test portability (`--cc clang`).
- V10: 2×2 monitoring layout, scroll-follow with Latest buttons, more native
  minimum-width fields (`docs/V9/`, `docs/V10/`).

## Obstacle module reintroduced (5 Nov 2026)

- The alternative Buddy 5 obstacle module returned under
  `app_program/modules/obstacle/`: servo scan, side-facing tracking (2 hits to
  acquire, 5 absent to end), no assumed-depth shortcut, and body-clearance
  travel before the return. Later the same day: preferred side, the servo aimed
  at the object while passing, an odometry-decided return with an IR zig-zag
  seek, an unseen-object depth fallback, and a pivot-gain setting.

## V8.2: recoverable navigation stop

- `ROBOT_NAV_SAFETY_STOP` (mission 5) is cleared by STOP; genuine
  `ROBOT_ERROR` (mission 7) stays latched. Bounded crossing of barcode stripes
  during a scan. The scanner rearms after 250 white samples. Scan speed
  restored to 15 RPM. A success alert needs a real decoded-letter event.

## V8.1: autonomous navigation

- `app/nav_core.c` executes mapped, validated letters at sustained both-black
  junctions; unmapped or ambiguous junctions and timeouts stop safely. The
  buffered analog scanner rearms automatically in live builds, and rejected
  scans are published. `LINE_UNCERTAIN` and STOP apply immediate zero PWM.
  Navigation events 12–19 are added. 53 dashboard tests.
- `tools/test_app.py` was already broken from V8.0 (its stub lacks
  `tk_*_sem`) and still is.

## V7.x: improve-movement + MQTT

- **V7:** rebased on the team's `improve-movement` motion, IR and IMU code with
  MQTT telemetry; experimental Buddy 5 removed at this point. START codes 2
  and 5 explained. `tools/sync_broker_ip.py` added.
- **V7.1:** fixed malformed string literals in `usb_console.c`.
- **V7.2:** `npm run lan` is read-only (HTTP 409). `run_live_dashboard.sh`
  enables commands. Separate live-telemetry and command consoles.
- **V7.4:** Overview, Map, Telemetry and Command Monitor cleanup; heartbeat
  shown separately from sensor freshness. 38 dashboard tests.
- **V7.5:** `BARCODE_DIAG` frames (type 8), raw encoder counts, provisional
  odometry (`G7_ODOMETRY_MODE=1`), magnetometer heading. At this point the
  native Code 39 decoder tests reported 83 of 3,409 checks failing.
- **V7.6:** `SCAN SUCCESS` / `SCAN FAILED ×N` alerts, freshness bar, responsive
  layout; fixed `lastMotionSeq` so ODOM routes draw. 46 dashboard tests.
- **V7.7:** the team's `barcode_scanning/` engine became authoritative:
  `bc_fast` samples GP27 on TIMER alarm 2 every 2 ms, decoding runs in a
  worker, the telemetry task ACKs after publishing (`BC_CAL_AUTO_ACK` off), and
  the bench flags `BC_CAL_TEST_IGNORE_INPUT_ERRORS`/`RELAXED_CAPTURE` were
  adopted. The scanner's own suite had one failing assertion (`extreme short
  narrow run still rejected`).
- **V7.9:** USB reconnect diagnostics; the build verifies the
  `G7DIAG-V7-9-RECONNECT-20261102` marker in the linked ELF.

## V5 and V6: Wi-Fi diagnostics and build guards

- **V5:** after the first failed join, a one-shot Wi-Fi scan prints
  `[WIFI SCAN]`/`[WIFI AP]` on USB; radio country set to Singapore;
  `reset_wifi_setup.sh` added.
- **V6–V6.2:** `build_uf2.sh` asserts that `mqtt_utk_poll` is linked;
  `g7_restore_make_rules.py` and `g7_build_cache_guard.py` refuse stale ARM
  objects.

## Before V5: Buddy 5 experimental MQTT bring-up (2 Nov 2026)

- MQTT moved behind the CYW43 network owner (`lib/libnet/lwip/mqtt_utk.c`)
  with bounded queues. The setup wizard writes `config/wifi_credentials.h` and
  `config/wifi_secrets.h`.
- `build_uf2.sh` requires Pico SDK 2.2.0 and invalidates objects when the SDK
  path changes. It pre-creates USB dependency directories and builds `usb_cdc`
  sequentially to avoid a macOS make race. Fixed coding-standard violations
  without disabling the checker.
- Replaced `__atomic_exchange_n` (unsupported on Cortex-M0+) with a kernel
  mutex in the controller.
- `[USB SELFTEST]` heartbeat from the USB service; UART0 is off in `usb_cdc`
  builds. The MQTT build joins the SSID directly, and the `[NET]` diagnostic
  line and `tools/diagnose_wifi_mqtt.py` were added.
- `serial_monitor.sh` installs pyserial into `.venv-serial`. It is a single
  serial reader that mirrors to the dashboard and doubles as the barcode
  fallback when MQTT is down.
- Dashboard palette and layout polish; `demo.sh` simulator on loopback.
