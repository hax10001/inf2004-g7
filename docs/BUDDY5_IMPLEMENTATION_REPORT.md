# Buddy 5: continuous monitoring, avoidance and recovery

Updated 30 October 2026 on `adibbest`. Software tests pass; no firmware was
flashed and no physical robot result is claimed. Captured output and build
hashes are in [BUDDY5_TEST_EVIDENCE.md](BUDDY5_TEST_EVIDENCE.md).

## Geometry supplied by the user

| Measurement | Configured value |
| --- | ---: |
| Overall width, including wheels and left IR sensor | 153 mm |
| Maximum height | 160 mm |
| Axle to frontmost point | 135 mm |
| Axle to rearmost point | 65 mm |
| Length used for clearance | 200 mm |
| Wheel outside diameter | 70 mm |
| Wheel width | 16 mm |
| Wheel centre spacing | 110 mm |
| Sonar forward of axle midpoint | 90 mm |
| Sonar lateral offset | 0 mm, centred between the wheels |

The later axle measurements total **200 mm**, rather than the initial
190 mm estimate. The longer envelope is used. The 55 mm measurement is
the distance from the centred sensor to either wheel centre, not a lateral
sensor offset.

The IR extension makes the body asymmetric. Given the overall 153 mm width
and wheels reaching +/-63 mm, either side can extend at most 90 mm from
the axle midpoint. The model conservatively uses +/-90 mm instead of
assuming the extension is symmetrically divided. Its farthest axle-to-body
corner radius is rounded up to **163 mm**.

Dimensions are recorded in `obs_config.h`; the shared wheel diameter is
in `board/board_config.h`. `OBS_GEOMETRY_VERIFIED=0` deliberately prevents
bypass planning. The current supplied geometry is enabled, but the whole
obstacle subsystem remains disabled by default through
`APP_OBSTACLE_ENABLED=0`. These measurements do not calibrate encoders,
servo direction, servo pulse timing, stopping distance or wheel slip.

## Behaviour and ownership

The obstacle task exclusively operates the servo and HC-SR04. The Vehicle
Controller remains the only movement owner, calling Buddy 2's existing
`motion_drive()` / `motion_apply()` functions. Buddy 3 supplies protected
IR snapshots and the existing line decision function. There is no second
motor controller or direct motor access from the scanning task.

1. **Detection:** while line following, the sonar faces forward. The car
   cannot drive without a valid front reading no older than 250 ms. A
   median-of-three and three confirmed hits trigger a stop at the
   configurable 600 mm threshold. A raw return at or below 250 mm requests
   an immediate stop without waiting for the median. An invalid front
   reading also inhibits forward movement; sustained no-echo faults latch.
2. **Stationary route scan:** the stopped car scans relative
   -60/-30/0/+30/+60 degrees, then refines around the detected object and
   completes a 10-degree grid. These coarse angles correspond to nominal
   servo positions 30/60/90/120/150 degrees. Samples include range and
   angular uncertainty; unknown readings cannot establish free space.
3. **Moving avoidance:** all moving phases, including every turn, alignment
   and line recovery, require fresh front, obstacle-side and outer-side
   views. The task keeps sampling/panning during movement. The controller
   grants only a small movement increment before requiring fresh coverage.
4. **Rear clearance:** the obstacle must first be positively seen, then
   followed by three valid far readings. Missing echoes are never treated
   as a trailing edge. Both wheel encoders must then establish sufficient
   additional forward travel before turn-back can begin.
5. **Line recovery:** the car searches toward the original line, confirms
   an IR edge, aligns with the original heading and verifies repeated edge
   readings using Buddy 3. Both-black recovery input stops the robot. The
   controller clears bypass IR latches and waits for a fresh, centred
   forward sonar reading before resuming ordinary line following.

The original heading and segment progress survive a permitted stationary
reassessment. The car does not silently redefine its original line or
restart a partly completed distance segment after a transient hazard.
Per-period motor request, speed and CSV printing are disabled in the
obstacle-enabled build to reduce encoder interrupt delays; concise obstacle
transition logs remain available.

## Clearance and live movement permissions

`obs_geom.c` computes conservative body projections and the turn envelope.
The static planner includes the two forward-wheel arcs, axle spacing,
corner radius, sensor offset, diagonal travel and an expanded outer
corridor. It checks samples outside the front cluster as well as the
primary obstacle. Profile left/right depths remain radial observations,
not exact lane-width measurements.

Current configurable reserves are:

| Reserve or limit | Value |
| --- | ---: |
| Safety margin | 40 mm |
| Position / range uncertainty | 20 mm |
| Angular uncertainty | 5 degrees |
| Braking / controller-period overrun allowance | 15 mm |
| Movement between renewed coverage requests | 10 mm or 3 degrees |
| Coverage collection / freshness deadline | 2500 ms |
| Avoidance wheel target | 12 RPM, through Buddy 2 |
| Odometry maximum age | 200 ms |
| Encoder stall limit | 1000 ms of commanded movement per wheel |
| Static scan timeout | 12 seconds, plus stop settling where applicable |
| Moving phase timeout | 180 seconds |
| Entire bypass timeout | 900 seconds |
| Shared rescan budget | One per bypass, including live reassessment |

Panning has a settling delay, so the car advances in short increments and
pauses when coverage is incomplete or its movement budget is used. This
prevents continuing blindly while the servo is obtaining another view.
The sampler also repeats a view without panning and cycles through the
views while a movement permission is active. The host tests explicitly
observe real sampler calls while simulated wheel commands are nonzero.

Each coverage request has an epoch. A measurement started for a previous
pose cannot satisfy a new request. All three views must be valid and the
packet must be fresh. A close return or unreliable moving measurement
requests STOP and stationary reassessment at the next controller decision
(nominal period 50 ms). Sensor acquisition and scheduling also contribute
to response latency; no hard real-time stopping guarantee is claimed.

A successful stationary rescan may resume the interrupted manoeuvre only
if the current front region is clear; all live views must be reacquired
before motion. If an occupied route needs a different plan, the robot
stops rather than making an unverified mid-bypass route switch. A second
hazard, exhausted retry budget, scan failure or timeout leaves it stopped.
Servo failures and invalid odometry stop directly. A stalled wheel is
checked independently, accumulating commanded time across scan pauses.

## Confirming the entire vehicle has passed

The former encoder-only fallback that returned without ever seeing the
obstacle is removed. Not seeing an object is insufficient to begin recovery.
After a positively confirmed side edge, the rear-clearance distance includes:

- the sonar's 90 mm forward offset;
- the 163 mm corner radius, allowing the rear to sweep during turn-back;
- the forward-looking blind wedge of the side ray, enlarged for up to
  10 degrees of heading error and 5 degrees of angular uncertainty;
- pass margin, safety, position uncertainty and braking/movement reserves.

With the current values this totals **723 mm after edge confirmation**.
Progress is 95% of the smaller wheel's measured forward travel, so roughly
762 mm of travel by **both wheels** is needed. Wheel averaging or rotation
of only one wheel cannot satisfy this check. Heading must remain within
10 degrees, and live inner/front/outer measurements must still be valid.

If the obstacle reappears during rear clearance, the car returns to side
passing and restarts rear clearance after the newly confirmed edge. If it
is never seen or never clears, bounded distance/time limits stop the car.
These are conservative estimates, dependent on correct encoder scale and
acceptable wheel slip; they are not a physical collision-free certificate.

## Compact representation and synchronisation

| Data | Bits / layout |
| --- | --- |
| Phase, including REASSESS (0-14) | 4 |
| Route (none / left / right) | 2 |
| Independent faults | 5-bit mask |
| Saturating bypass count (0-15) | 4 |
| Front distance, cm / invalid sentinel | 9 |
| Profile width, cm / invalid sentinel | 8 |
| Shared status total | 32 bits |
| Scan point | 4-bit angle index + 9-bit cm range, in 2 bytes |
| Plan handoff | Two masked 32-bit words, 8 bytes |
| Live view ranges | Three 9-bit cm fields |
| Valid views / live hazard reason | 3 bits / 2 bits |
| Live request epoch / request time | 8 bits / modulo-2^24 ms |
| Complete live coverage snapshot | 8 bytes |
| Normal front validity / sample time | 1 bit / modulo-2^24 ms |
| Drive decision | Two 5-bit targets + four 1-bit actions, 4 bytes |
| Local saved phase / epoch / reassessment flag | 4 / 8 / 1 bits |
| Local per-wheel stall timers | 11 bits each, saturating at 1000 ms |
| IR left/right/barcode/valid | One bit each, snapshot total 8 bytes |

Status, plan, front freshness and live coverage use the existing
priority-inheritance mutex. Requests invalidate old coverage under the
same lock used for publication. The first hazard is latched within an
epoch. Scan requests are retired before DONE is published, preventing an
SMP rescan request from being erased. IR snapshots/latches retain their
own mutex. Local manoeuvre fields have only the controller as writer.

A stale front sampler latches a timeout rather than allowing an abandoned
compact timestamp to become fresh again on a later wrap. Live coverage
transactions are bounded to 2.5 seconds and manoeuvres to 15 minutes.

`python tools/measure_buddy5.py` measures the compact payloads. Host and ARM
agree: scan point 8->2 bytes, 15-point array 120->30, status 24->4,
plan 44->8, drive decision 12->4, IR snapshot 20->8 and live coverage 28->8.
Scan/status/plan/live comparisons use equivalent unpacked structures;
drive and IR use the original layouts. These are not claims of a net
whole-firmware RAM reduction. New monitoring state adds functionality and
storage. Whole-project RAM/payload/timing comparisons and a complete field
and BARR-C review still require team-wide evidence. C bit-fields are not
a portable telemetry wire format; MQTT encoding belongs to its owner.

## Verification and build

```text
python tools/test_buddy5.py --cc gcc
python tools/test_app.py --cc gcc --units-only
python tools/check_app_standard.py
python tools/measure_buddy5.py --cc gcc
```

The Buddy 5 suite has **46 passing host scenarios**. It exercises the real
controller, sequencer, scanner, profiler, shared state and geometry with
deterministic motion/sensor/RTOS dependencies. Separate register tests run
the real ultrasonic and servo drivers. Cases include incomplete coverage,
measurements while moving, live hazards on passing/return, safe resumption,
rear-edge reappearance, never-seen obstacles, stale odometry, per-wheel
stalls, initial front readiness, epoch rejection and timer/encoder wrap.
The four on-target obstacle profile checks also execute in the host suite.

Existing subsystem units and the TEST=1 host link with 11 checks pass.
Older captured trace comparisons use previous operating settings and are
not claimed as current passing evidence. The focused standard checker
passes **63 files**; it is a subset of BARR-C, not full certification.
ARM translation-unit checks, ELF links, UF2 sizes and hashes are recorded
in the evidence document. Host tests do not measure actual servo latency,
vehicle clearance, RTOS worst-case timing or line recovery on the car.

```text
cd build_make
make SMP=1 CONSOLE=uart OBSTACLE=1 TEST=1 PYTHON=python
```

The UART builds are compile/link evidence: GP0/GP1 belong to the encoders
on this car. For physical log capture use the repository's USB console
profile and its compatible initialized SDK checkout:

```text
make SMP=1 CONSOLE=usb_cdc PICO_SDK_PATH=/path/to/pico-sdk OBSTACLE=1 TEST=1
```

The configured USB SDK checkout is unavailable here; USB was not built or
qualified in this session. Encoder counts/revolution remain the original
1056 and require measurement. Servo forward/direction/settling, uncertainty
bounds, braking allowance, wheel slip and actual sensor coverage also need
bench calibration. APP_IMU_ENABLED remains 1, requiring a valid startup
level reference before the controller permits normal driving.

## Physical evidence still required

| Test | Evidence | Status |
| --- | --- | --- |
| Servo, known ranges and dimensions | Measured angle/range errors and settling | Not run |
| Left and right bypass | Full-car clearance and live scan logs/video | Not run |
| Changing / approaching obstacle | Stop during turns, passing and return | Not run |
| Trailing edge and rear swing | Both encoder traces and full rear clearance | Not run |
| Reappearing / never-seen obstacle | Restart clearance or bounded STOP | Not run |
| Sensor, encoder and IR failures | Safe stop, retry limit and latched faults | Not run |
| Original line recovery | Reliable capture, alignment and continued mission | Not run |
| Repeated course runs | Trials, failures, timing, slip and minimum clearance | Not run |

The write-up and briefing define the Buddy 5 stages; Change Requirement 1
requires compact data and measured evidence. Those documents are requirement
sources, not instructions authorising unrelated repository operations.
This revision uses the existing kernel/BSP and buddy interfaces. It does
not establish approval of any course architecture exception.

A single horizontal HC-SR04 sweep cannot prove clearance above/below its
beam or detect every surface/material or rear-approaching obstacle. The
160 mm height is recorded, but vertical overhead clearance is not measured.
Line identity is inferred from the bypass geometry and IR; arbitrary nearby
black markings need additional navigation context. Unknown measurements
cause conservative stops. Reverse remains unsupported by the existing
motion driver; no reverse command or alternate motor driver was introduced.
