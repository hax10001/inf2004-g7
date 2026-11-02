# Buddy 5 — ultrasonic geometry, left/right clearance and line recovery

Date: 3 November 2026
Branch: `adib-obs-last`. Base: the working tree on top of commit `02c6e27`,
including the uncommitted Buddy 5 changes already present in it.

No hardware trial was possible for this change. Every result below comes
from host builds, host simulations or the ARM cross-build.

## 1. Outcome

| Problem reported | What was wrong | Result in the closed-loop simulation |
|---|---|---|
| Scan not far enough left and right | Main profile ±40°; ±70° probes compared one raw range with 240 mm | One 140° sweep, 150° once 75° limits are confirmed; full-body check of each departure |
| Servo mounting not accounted for | Every range assumed to start 110 mm ahead of the axle | Ranges start at the transducer, carried round the servo axis |
| Too close to the object | Departure check ignored the pivoting body corners | 75 mm swept-body margin to every scanned surface; minimum body clearance 67 mm |
| Object assumed ended too early | Bearing of side pings unknown; unknown echoes and frames mixed | Ping used only at the bearing asked for; rear needs clear views over axle travel |
| Turning inward into a blocked path | One side reading at 65° decided the return | Fresh look down the return path plus a swept-body check of the turn and run |
| Not turning promptly when clear | Always stopped at the turn point; travel measured from the echo point | No stop at the turn point; turn starts 115 to 170 mm past the true rear |

The same simulation was run against the module as it was before this
change. 22 of 40 runs passed before, and 40 of 40 pass now. Section 8 has
the details.

## 2. Root causes found in the previous code

1. **Narrow profile and a raw threshold.** The ±70° probes vetoed a side
   only for an echo under 240 mm. In the simulation, the old module saw a
   box 298 mm away in the left departure corridor, allowed that side, and
   drove into the box.
2. **No sensor origin model.** `OBS_SENSOR_TO_AXLE_MM` was added along
   the robot heading for every bearing. The servo axis, the arm carrying
   the transducer and any lateral offset were not represented.
3. **Front-only departure check.** The angle choice compared the forward
   advance with the face distance, using the unrotated front overhang. The
   outer front corner of a pivoting car reaches 53 mm further forward than
   that.
4. **Frame mix in the rear test.** Travel since the last side hit was
   measured from the echo point, about 195 mm ahead of the axle, to the
   axle. The car therefore drove roughly 300 mm past the last hit before it
   could turn, then stopped again for a fresh ping.
5. **Untagged readings.** A side ping carried no bearing. A ping finished
   at the old servo angle could certify the return check at the new one.
6. **No look down the way back.** The return used one reading at 65°
   towards the object. Nothing checked the inward path for another object,
   and nothing checked the whole body during the turn.
7. **Ungated line search.** In the search phase any black counted as the
   line. A mark 170 mm off the line left the old module following 257 mm
   off course.
8. **No lost-sensor check while moving.** A sensor that died during the
   departure or the pass was never detected. The old module drove on until
   the 1200 mm pass limit.

## 3. Changes by file

All production changes are inside `app_program/modules/obstacle/`.

| File | Change and reason |
|---|---|
| `obs_geometry.c/.h` (new) | Integer geometry: 1° sine table, sensor origin at any bearing, echo point, pivot about a stopped wheel, and an exact body-rectangle-to-box clearance test. Shared by the planner and the sequencer so both use one model. |
| `obs_config.h` | Mounting values, scan far bearing, pan-plane check, pass, rear and return parameters. Existing calibrated values are kept. `OBS_DEPART_PROBE_DEG`, `OBS_DEPART_BLOCK_MM`, `OBS_SIDE_VERIFY_DEG` and `OBS_FRONT_CLEAR_MM` are removed: the swept-body checks replace them. `OBS_PASS_MARGIN_MM` (110) becomes `OBS_REAR_PASS_MARGIN_MM` (45) plus the 65 mm rear overhang, which keeps the same 110 mm axle distance. |
| `obs_state.c/.h` | Side and front readings now carry echo class, true servo bearing, kernel time and a sequence count. They are copied whole under the existing binary semaphore. A sensor orientation state is added: forward, left, right, returning, settling or unavailable. |
| `task_obstacle.c` | One right-to-left sweep across 11 bearings, each stopping once two pings agree. The servo is returned to FRONT and settled before the plan is released. All pings are tagged with the clamped bearing. A newly requested bearing is served before a due front glance, unless the glance is overdue. Startup log reports the mounting and warns while it is unmeasured. |
| `scan_plan.c/.h` | Points placed in the vehicle frame from the real sensor origin. Unknown samples are never free space. The whole manoeuvre is checked for each side and angle. A side may widen to pass a second surface, up to 430 mm. Departure angle chosen so the side beam still sees the object. Adds a 6-bit slack field in spare bits of the packed plan. |
| `obstacle.c` | Tagged side intake. Hit, clear and unknown classification in the line frame. Evidence-based rear edge. Bounded, absolute offset corrections. Fresh forward check while passing. Return probe armed before the turn point. Swept return check. Gated line search. Lost-sensor faults in every watched phase. Bounded waits that end in a servo fault. |
| `test_obs.c` | The "objects on both sides give no route" self-test now places those objects at ±30° and ±40°, where passing round them would exceed 430 mm. At ±20° the new planner correctly finds a way round all three. |
| `README.md` | Rewritten for the current behaviour. |

Unchanged: `servo.c`, `ultrasonic.c`, `task_controller.c`, all Buddy 1 to 4
modules, the kernel, the BSP and all GPIO assignments.

Test files: `tools/tests/buddy5_planner.c` and `buddy5_side_clearance.c` are
rewritten. `tools/tests/buddy5_sim.c` is new.
`tools/tests/run_buddy5_clearance.py` is rewritten to run all three and no
longer needs a GNU-only linker option, so it now runs on macOS too.

## 4. Mounting and coordinate geometry

**Frames.** Vehicle frame: origin at the midpoint of the driven-wheel
axle, x forward, y left, z up, angles positive to the left. Line frame: the
original line is y = 0, x along it, origin where the obstacle was
confirmed, pose from encoder dead reckoning.

**Sensor origin.** With the servo at bearing b, the range starts at

```
axis   = (OBS_SENSOR_TO_AXLE_MM - OBS_SERVO_ARM_MM, OBS_SENSOR_LATERAL_MM)
origin = axis + OBS_SERVO_ARM_MM * (cos b, sin b)
echo   = origin + range * (cos b, sin b)
```

In the line frame both are rotated by the car's heading and added to its
position. The ray angle is heading plus b.

**Values in use.**

| Value | Setting | Status |
|---|---|---|
| Transducer face ahead of axle at FRONT | 110 mm | Existing value |
| Transducer left of centre | 0 mm | Measure |
| Transducer face ahead of servo axis (arm) | 0 mm | Measure; 0 keeps the old behaviour |
| Transducer height above floor | 0 mm | Measure; documentation only, planar maths ignores it |
| FRONT pulse, beam straight ahead | 1500 µs | Calibrate with `OBS_TEST_SERVO_CAL` |
| Pulse per degree | 10 µs | Existing value |
| Longer pulse turns left | `OBS_SERVO_LEFT_SIGN` 1 | Measure; reverses if mounted upside down |
| Physical limits left and right | 70° each | Measure with `OBS_TEST_FIND_LIMITS` |
| PWM room at FRONT 1500 µs | 90° left, 100° right | Verified from the pulse range |
| Pan plane | `OBS_SERVO_PANS_HORIZONTALLY` 1 | Build stops with an error if 0 |
| Body: front, rear, width, track | 135, 65, 153, 110 mm | Existing values |
| IR sensors ahead of axle | 125 mm | Existing value |

The mechanical forward angle is the FRONT pulse measured from the SG90
centre pulse. It is printed at startup and is 0° at present.
`OBS_GEOMETRY_MEASURED` stays 0, which prints a warning, until the four
mounting values are measured.

**Orientation state.** The obstacle task publishes "returning" or
"settling" when it commands the servo, and "forward", "left" or "right" once
the settle time has passed. No ping is taken while the servo moves. Each
ping carries the bearing it was taken at. The sequencer uses a side ping
only if that bearing equals the one it asked for and the ping ended after
the request. Front pings are taken only at FRONT. A bypass plan is used
only once the state is "forward".

## 5. Scan and departure logic

**Sweep.** The sweep runs from −70° through −40° to +40° in 10° steps,
then to +70°: 11 bearings. With both limits confirmed at 75°, the same
code sweeps −75° to +75°, the 150° arc. Each bearing takes up to three
pings. If two pings agree within 30 mm the third is skipped, since two
agreeing values already bound the median. A ping with no echo pulse is
dropped. Fewer than two informative pings make the bearing unknown. The
whole scan takes 3.1 s in simulation, against 4.5 s before.

**Object and edges.** The object is the run of near profile samples around
the front, as before. An unknown sample next to that run is taken as more
of the object, never as its edge. Edges are padded by half a step. An edge
reaching the end of the profile is unseen and uses the 430 mm offset.

**Passing offset.**

```
offset = edge + body width / 2 + body margin + position allowance
       = edge + 76 + 65 + 40 mm
```

Here the edge is measured from the axle centreline, after the sensor origin
correction.

**Departure check.** For each side and each angle (30°, 45°, 60°, 90°), the
planner plays the manoeuvre in 5° and 20 mm steps:

1. Pivot out on the wheel on the bypass side.
2. Run on the diagonal until the offset is reached.
3. Pivot back to parallel on the other wheel.
4. Pass until the car's rear clears the assumed depth.

At every step the full body rectangle must stay 75 mm from every object
sample, which is the 65 mm margin plus 10 mm range error. The same applies
to every other echo seen anywhere in the sweep.

**Verification.** A side counts as verified only if its far bearing
returned an echo or a beyond-range result. Any sample on that side with no
echo makes the side unverified. Beyond range is accepted as "nothing
reflected" only because the sensor is shown healthy by its other echoes.
The departure watch keeps pinging that side during the diagonal, and two
echoes under 190 mm stop the car.

**Angle choice.** The gentlest clear angle is used, provided the side beam
still meets the object's side within 40 mm of its front face. This keeps
very thin objects visible from the side. If no clear angle meets that,
the clear angle that looks back furthest is used.

**Side choice.** If only one side fits, it is used. If both fit, the side
needing at least 40 mm less offset wins. Otherwise the side with more
checked room beyond its path wins. If a second surface on a side blocks the
path, that side may widen its offset to pass both, up to 430 mm.

## 6. Passing clearance and rear-edge detection

**Classification.** Each accepted side ping becomes one of three things:

- A hit: an echo between 60 and 500 mm whose point lies between the car
  and the line.
- Clear: beyond range, or an echo from past the object's band.
- Unknown: no echo, an echo inside the car's own 60 mm, or an odd
  geometry.

**Body clearance.** The measured gap from the car's centreline must be at
least 76 + 65 + 10 = 151 mm. If it is less, the passing offset is raised to
an absolute target, so the same error is never added twice. The target may
grow by at most the smaller of 60 mm and the room the scan checked beyond
the path. Needing more than that, or two gaps under 100 mm, stops the car
for a new scan instead of steering into unchecked space.

**Rear-edge rule.** All of the following must hold:

1. The object was seen with two side hits.
2. At least three clear pings followed the last hit.
3. The axle has travelled at least 110 mm since the last hit, measured
   from the axle, not from the echo point.

Unknown pings neither add to nor erase this evidence. Five in a row are a
lost-sensor fault.

**Rear estimate.** The rear is the furthest beam-axis hit point. For a
reflecting side face the last echo comes from the beam's rearmost ray,
which meets the face behind the axis point, so the estimate is never too
early. The 150 mm assumed depth stands until a clear side view reaches
past it. A new hit at any time cancels the end and moves the rear on. An
object never seen from the side is never taken as passed: the car stops
with a no-route fault at 1200 mm.

**Forward check while passing.** The car drives on only while a forward
ping from the last 150 mm of travel found the way clear. Otherwise it stops
and waits, for at most 2 s, then faults.

## 7. Return-corridor check and line recovery

**Turn point.** The axle may turn back once it is 110 mm past the rear
estimate. That puts the car's rear 45 mm past it.

**Probe.** 60 mm before that point the servo is turned to look down the
return path, at 60° inwards less the present heading. Each probe ping is
handled as follows:

- An echo between the car and the line means the object goes on: the car
  keeps passing.
- An echo further on is kept as an obstacle point.
- Beyond range is a clear verdict.
- No echo is no verdict.

**Swept return check.** At the turn point the car turns back at once if
all of these hold:

- the probe was clear and taken within the last 120 mm;
- the rear rule still holds;
- the whole body, through the 60° pivot on the inner wheel and the
  straight run until the axle reaches the line, stays 65 mm clear of the
  object region and of any probe point.

The object region is everything behind the rear estimate and inwards of
the side face actually measured, right across the line. This check also
stops a turn aimed at the rear corner, and a turn by a car hugging the
object too closely.

If the probe was clear but the swept check fails, the car passes on 100 mm
and probes again. If there is no verdict yet, it stops and waits, for at
most 2 s.

**Line recovery.** The car runs to the line at 60°. Black is accepted as
the original line only when the IR sensors are within 120 mm of where the
encoder pose puts the line, in the turn, the search, the alignment and the
verification phases. Crossing lines and stray marks elsewhere are ignored.
The existing line-found pose fix, the alignment turn and the 10-period
verification then hand control back to Buddy 3. Barcode and ramp handling
resume unchanged.

## 8. Tests and actual results

Commands, run on macOS with Apple clang as `gcc` and Arm GNU toolchain
15.3.

```sh
python3 tools/check_app_standard.py
python3 tools/tests/run_buddy5_clearance.py --cc gcc
python3 tools/test_app.py --cc gcc
cd build_make && make SMP=1 -j8 && make SMP=0 -j8 && make SMP=1 TEST=1 -j8
```

| Check | Result |
|---|---|
| `check_app_standard.py` | Pass: 63 files, 0 errors |
| Geometry, servo, planner checks, robot configuration | 27 of 27 pass |
| Same, with 20 mm arm, 10 mm lateral offset and 75° limits | 27 of 27 pass |
| Sequencer checks, left and right | 43 of 43 pass |
| Closed-loop simulation, 20 scenarios, both directions, 3 builds | 120 of 120 pass |
| `test_app.py` | Fails at `robot_harness.c:313`, a Buddy 2 motor assertion. It fails identically on the code before this change, and no motion code was touched. |
| `test_app.py` later stages, run separately | TEST=1 self-tests: 16 pass, 0 fail, including all 5 obstacle tests. All 35 controller, motor, IR and IMU traces are byte-identical before and after this change. |
| Stored `expected_traces.json` | 20 of 35 traces differ from it, identically before and after this change. The script also expects 11 self-test passes where there are 16; the previous Buddy 5 report already recorded 16. |
| ARM `SMP=1` | Builds, 0 application warnings |
| ARM `SMP=0` | Builds, 0 application warnings |
| ARM `SMP=1 TEST=1` | Builds, 0 application warnings |
| ARM `CONSOLE=usb_cdc` | Not built: the Pico SDK is not installed here |
| Physical robot | Not run |

**Firmware size.** For the `SMP=1` image, text grows from 82,808 to 89,480
bytes. Data grows from 260 to 284 bytes and BSS from 14,092 to 14,132
bytes, so RAM rises by 64 bytes. New data uses the narrowest fixed-width
types that fit: `int16_t` ranges, `int8_t` bearings, `uint8_t` echo class
and sequence. The sine table is `int16_t`, and the plan's 6-bit slack
field sits in previously spare bits.

**Coverage of the requested scenarios.**

| # | Scenario | Where tested |
|---|---|---|
| 1 | Servo forward and left/right polarity | Planner checks: FRONT pulse, sign, clamping |
| 2 | Mounting offset | Geometry checks; offset-mount simulation build |
| 3 | Maximum scan angles | Sweep checks at 70° and 75°; 75° simulation build |
| 4 | Ahead, both sides open | `open` |
| 5, 6 | Left blocked, right blocked | `left_blocked` and its mirror; planner checks |
| 7 | Object in the outward corridor | `outward_corridor`, `departure_late`; planner far-bearing checks |
| 8 | Too close while passing | `widens`; sequencer offset checks |
| 9 | Very short obstacle | `short` (40 mm board) |
| 10 | Long obstacle | `long` (700 mm); sequencer long-object check |
| 11 | Side reading missing | `side_dropout` (4 lost pings); sequencer checks |
| 12 | Beyond range but not clear | `quiet_face`, `quiet_long`; sequencer checks |
| 13 | Obstacle reappears before the turn | `reappears`, `quiet_long`; probe check |
| 14 | Object in the return corridor | `return_blocked`; swept-path check |
| 15 | Rear of car not yet clear | Rear-rule and rear-corner sequencer checks; every simulation run checks the car's rear was past the true rear |
| 16 | Prompt return when clear | Every single-object run: turn within 220 mm of the true rear and at most 2 stopped periods |
| 17 | Original line recovered | Every completed run: within 25 mm of the line after return |
| 18 | Crossing or mark taken as the line | `crossing` (15 periods over off-line black); line-gate check |
| 19 | Repeated obstacles | `repeated` (two bypasses); fresh-bypass check |
| 20 | Timeout, settling error, invalid data | `sensor_dead`, `sensor_dead_pass`, `servo_slow`, `servo_stuck`, `spikes`; bounded-wait and FRONT-before-move checks |

**Before and after, default build.** "Past rear" is how far the axle was
beyond the true rear when the turn began. A dash means the scenario ends in
a deliberate stop or has no single rear.

| Scenario | Before | After | Body clearance mm | Past rear mm |
|---|---|---|---|---|
| open | Pass | Pass | 161 | 115 |
| left_blocked, both directions | Fail: stopped 4 periods at the turn | Pass | 179 | 151 |
| outward_corridor | Fail: collision | Pass | 194 | 161 |
| outward_corridor, mirrored | Fail: stopped 3 periods | Pass | 194 | 161 |
| departure_late | Pass: safe stop | Pass: safe stop | 93 | – |
| widens | Fail: stopped 9 periods | Pass | 67 | 127 |
| short | Pass | Pass | 161 | 165 |
| long | Pass | Pass | 161 | 169 |
| side_dropout | Fail: stopped 6 periods | Pass | 161 | 141 |
| quiet_face | Fail: injected fault never occurred | Pass | 161 | 141 |
| quiet_long | Fail: stopped 4 periods | Pass | 161 | 132 |
| reappears | Pass | Pass | 161 | 126 |
| return_blocked | Pass | Pass | 161 | 154 |
| crossing | Fail: 257 mm off the line after return | Pass | 161 | 115 |
| repeated | Pass | Pass | 161 | – |
| both_blocked | Pass: no-route stop | Pass: no-route stop | 197 | – |
| sensor_dead | Fail: drove on, no-route fault | Pass: no-echo fault | 225 | – |
| sensor_dead_pass | Fail: drove on, no-route fault | Pass: no-echo fault | 161 | – |
| servo_slow | Pass | Pass | 161 | 119 |
| servo_stuck | Pass: no-route stop | Pass: no-route stop | 301 | – |
| spikes | Pass | Pass, with 19 stopped periods at the turn | 161 | 115 |

Rows without "both directions" are shown for the first direction. The
mirrored results match, apart from the corridor row shown separately.

**What the simulation models.** A differential-drive car with encoders,
the left one counting backwards. A 25° beam whose rays echo from a face
within 40° of square, or near a corner. An SG90 turning at 0.6° per ms, or
slower in `servo_slow`. Lost, beyond-range and stray pings. Black stripes
and marks. Body-to-box clearance every 5 ms. The production tasks run on a
deterministic cooperative scheduler on a simulated clock.

**What the simulation does not model.** Wheel slip, coasting after a stop,
odometry drift, real echo strength, beam shape, timing on two cores, or the
barcode and ramp tasks.

## 9. Calibration still required and unverified behaviour

**Measure on the robot before driving.**

1. FRONT pulse, so the beam is square at bearing 0 (`OBS_TEST_SERVO_CAL`).
2. Left/right polarity.
3. Physical limits (`OBS_TEST_FIND_LIMITS`). Set both to 75° only if the
   sensor, bracket and wires clear the car there. That enables the 150°
   scan.
4. Transducer lateral offset, servo arm and height. Then set
   `OBS_GEOMETRY_MEASURED` to 1.
5. That the servo shaft is vertical, so the beam pans rather than tilts.

**Tune on the course.**

- Body margin (65 mm) and position allowance (40 mm), against real
  odometry drift through the pivots.
- `OBS_TURN_LEAD_DDEG`, against real coasting.
- `OBS_REAR_PASS_MARGIN_MM` (45 mm), if turns need to come earlier or
  later.
- `OBS_SIDE_NEAR_MM`, `OBS_SIDE_MIN_MM` and `OBS_OUTWARD_GUARD_MM`, against
  the real echoes of the course's obstacles. A side face that never echoes
  at 70° leaves the object unseen, and the car then stops rather than turn
  back.
- `OBS_MIN_DEPTH_MM` (40 mm), the thinnest object the planner keeps
  visible from the side. It is a design choice, not a measurement.

**Known limits that remain.**

- The passing-offset controller keeps its existing gains and corrects
  slowly: about 300 mm of travel for a 30 mm correction. In `widens` the
  car still kept 67 mm.
- After a stop part-way along the diagonal, the new scan is planned from
  the car's current heading, as before.
- A probe bearing that always sees the car's own body would never give a
  verdict. The car would stop with a servo fault after 2 s.
- None of this has been run on the robot. Hardware behaviour, RTOS timing
  on two cores and real sensor behaviour are unverified.
