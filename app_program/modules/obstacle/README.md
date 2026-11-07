# Buddy 5 — alternative obstacle-avoidance implementation

Based on `inf2004-g7(20261102-125316).zip`, this version retains the
alternative servo scan, variable-angle bypass, side-facing ultrasonic
tracking, and encoder/IR original-line recovery. It does **not** mix code
from the preceding progressive Buddy 5 architecture.

Nine scan bearings (-40 to +40 degrees) remain in use, with three ultrasonic
pings per bearing. A previous reliable front detection is retained if the
subsequent centred scan appears unexpectedly far away. The provisional
stop threshold is 350 mm, pending on-car measurement of stopping distance.

The return now requires the side watch to acquire the obstacle first. The
old 150 mm assumed-depth shortcut has been removed. Five consecutive absent
side pings start an end candidate; scanning continues while encoder travel
clears both the front-mounted sensor and rear of the robot. Any returning
echo cancels the candidate. A fresh absent sample is needed at the turn.
Valid side echoes inside the obstacle corridor are tracked beyond 500 mm.

With the current geometry, the minimum run from the first absent sample
is 110 + 65 + 40 = 215 mm, and the furthest observed obstacle point can
require more. `OBS_PASS_MARGIN_MM` is the extra margin after body clearance;
`OBS_SIDE_CLEAR_HITS` controls absence debounce. Missing echoes can also
mean poor reflections, so this policy still needs physical testing.
See the root `README_OBSTACLE_INTEGRATION.md` for behavior and limits.

Hardware bypass, ramp, barcode, line recovery, and servo geometry remain
unverified. See `docs/BUDDY5_ALTERNATIVE_IMPLEMENTATION_REPORT.md`.
