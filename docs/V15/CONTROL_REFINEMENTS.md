# Scanner START, descent control and obstacle return refinements

This follow-up applies to the current workspace after the initial V15 export.
The previously exported release ZIP is a separate, older snapshot.

## Scanner readiness

START requires the worker's armed WAIT phase, with no pending cancellation,
relearning or rearm request. CLEAR and DONE are not READY. The dashboard's
blocker text now explains that the scanner must arm before START.

Executing START resets navigation mailbox ownership without cancelling or
reinitializing the waveform sampler. Its learned white reference and
pretrigger buffer survive. STOP still cancels old work and suppresses triggers;
the worker completes clearance/learning while held. Readiness is checked both
when START is requested and when the controller executes it.

## Continuous descent speed

The existing, valid IMU `onHump` / `TERRAIN_HUMP_DESCENDING` signal now engages
the governor immediately. Detection itself and collision thresholds are unchanged.
Pitch-based slope detection remains available for a verified mounted pitch sign;
its default sign is still zero. A pitch-only slope without the existing hump
descent signal therefore still requires verifying that sign on the car.

The target changes by at most five percentage points per 50 ms period to 65%
(9.75 RPM from the 15 RPM straight target). Ordinary drive/brake effort changes
by at most two percentage points per period during descent and the handoff back
to level. Forward PWM and bounded short-brake/coast PWM use encoder feedback;
ordinary overshoot does not generate full-brake STOP/restart pulses.
An intentionally stopped inner wheel remains zero even with a learned integral.
Explicit STOP, collisions, stale sensors and sustained gross runaway still stop
the robot immediately through their existing paths.

## Obstacle selection and return

The confirmed front trigger changes from below 350 mm to below 250 mm, with
the existing median and consecutive-hit confirmation. Scanning includes both
configured servo extremes, currently -70 to +70 degrees at 10-degree steps.
The smaller clear lateral corridor wins. If both are blocked, the planner
reports NO_ROUTE. A rescan keeps the side already taken or stops if blocked.

The side watch continues on the side facing the obstacle. A first real echo
must receive its second confirmation; the never-seen fallback can no longer
overwrite that measured gap. Five consecutive absent side samples are still
required. The return rear bound uses actual sensor position and heading, retains
every observed obstacle point, and can tighten with later fresh clearance rays.
The turn begins when the measured gap and swept body meet the existing 40 mm
safety margin. The additional fixed 40 mm straight run has been removed.
The front-mounted sensor's seeing the end still does not mean the axle/body
has cleared it. Stale samples cannot authorize a return; reappearing obstacles
cancel clearance. Never-seen-object fallback, return guard, travel limit and
line reacquisition remain present.

## Minimum-bit storage and evidence

Controller-owned obstacle flags use one bit; signed directions two bits; phases
four bits; two-hit acquisition two bits; five-hit absence three bits. On ARM the
sequencer falls from 180 to 132 bytes. The stored previous signed PWM needs
15 bits (-3124..12499) and the two wheel flags one bit each; the wheel pair stays
56 bytes by using the old boolean's alignment space. The governor stays eight
bytes, navigation 104 bytes and scan handoff 12 bytes. The scan angle needs eight
signed bits for +/-70; its 12-bit range record remains four bytes. No wire frame
layout changes. This is a scoped storage improvement, not whole-project minimum-
bit certification.

Run the existing VS Code task **Verify V15 host tests and firmware**, or:

```sh
PICO_SDK_PATH=/Users/linustay/pico/pico-sdk-2.2.0 bash tools/verify_v15.sh
```

The maintained suite includes 76 dashboard tests, 170 navigation assertions,
ten START/STOP runs, a real sampler first-stripe readiness test, both extreme
scan bearings and mirrored bypasses. Return geometry is checked for 23,485
gap/angle combinations. The production PI/encoder/PWM plant checks cover pitch
signs 0, +1 and -1, changing descent/level conditions, both steering directions,
bounded effort transitions, stopped inner wheels and immediate zero-output STOP.
Pure governor tests also cover stale data and runaway with the sign-zero terrain
path. These are host simulations, not physical qualification.

Results are in [host tests](evidence/host-tests.txt) and
[target build](evidence/target-build.txt). Firmware is generated locally in
`out/g7-improve-movement-mqtt.uf2` with USB CDC and live MQTT control.

On the car, confirm READY stays armed across START and catches the first barcode;
compare descent RPM/effort traces across entry and exit; test narrow/long obstacles
on both sides, both-side blockages and the closer trigger. Servo reach, acoustic
dropout and actual stopping distance require those physical checks.
