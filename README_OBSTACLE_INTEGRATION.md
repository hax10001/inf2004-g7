# Obstacle side tracking and return clearance

This update starts from the restored reviewed obstacle module from
`INF2004_G7_Alternative_Buddy5_Reviewed_20261102/inf2004-g7` and changes the
side-tracking/end-of-obstacle decision. Runtime changes are limited to
`obs_config.h` and `obstacle.c`. The stationary width planner, ultrasonic
worker and hardware drivers retain the reviewed implementation.

Use `out/g7-obstacle-side-tracking.uf2`. The normal build helper produces
that same firmware as `out/g7-improve-movement-mqtt.uf2`. The previous
`g7-obstacle-restored.uf2` remains available as the baseline image. The
older clearance/fallback experiment images are superseded.

Wireless@SG, the current PC broker address, USB CDC diagnostics and
dashboard START/STOP are retained. The robot boots stopped. The initial
nine-bearing +/-40 degree scan and 350 mm unknown-edge sideways minimum
remain as restored; this update does not add lateral padding.

## Return decision

The previous code could turn back using a 150 mm assumed object depth if
it never saw the obstacle from the side. Once it had seen the object,
three non-hit samples could end the passing phase. A front-mounted sensor
can lose the obstacle while the rear of the chassis is still beside it.

The new behavior is:

1. Aim the ultrasonic toward the obstacle at the available side angle,
   normally 70 degrees, and acquire it with two qualifying side echoes.
   Valid echoes inside the corridor between the robot and line can remain
   tracked beyond the previous 500 mm cap, up to the sensor's range limit.
2. Require five consecutive absent readings before treating the end as a
   candidate. A valid echo beyond the line or no-return/out-of-range ping
   can indicate absence after acquisition. An invalid/self echo resets
   the absence run. Fresh sample sequences are consumed only once.
3. Keep the side watch active through the body-clearance run. It still
   takes the original periodic front guard glance and re-aims at the side.
   Any returning obstacle echo cancels clearance and resumes passing.
4. Travel at least sensor-to-axle + rear-body + pass-margin distance from
   the first absent sample: 110 + 65 + 40 = 215 mm with current geometry.
   Also place the axle at least rear-body + margin (105 mm) beyond the
   furthest observed obstacle point, with more added if the return arc
   requires it. The later of those requirements controls the turn.
5. Require a fresh absent side sample at the actual return decision.
   Cached clear data or an early black IR mark cannot bypass the body
   clearance check. A front-guard rescan starts fresh side confirmation.

There is no assumed-depth escape when the side watch never acquires the
obstacle. Existing travel/time bounds stop it if an end cannot be found.
If side sample publication stalls for over one second, it stops with the
existing timeout fault. STOP, MQTT-loss STOP and front guards remain active.

A missing echo can mean open space, weak reflection or a sensor problem.
Repeated absence and measured travel are a practical end estimate, not
proof of an empty route. The robot has not been physically tested by the
agent, and the sensor/servo geometry still needs verification on the car.

## Configuration

In `app_program/modules/obstacle/obs_config.h`:

- `OBS_SIDE_SEEN_HITS=2`: readings needed to acquire the obstacle.
- `OBS_SIDE_CLEAR_HITS=5`: consecutive absent readings needed for an end
  candidate; the body-distance requirement must also complete.
- `OBS_PASS_MARGIN_MM=40`: extra forward margin after the rear clears.
- `OBS_SIDE_MAX_AGE_MSEC=1000`: maximum side sample age; this accommodates
  the normal front glance and servo settling time.
- `OBS_PASS_MAX_MM=1200`: bounded travel beyond the object's front face.
  Long objects need room for their full length and the body-clearance run;
  exceeding this limit stops rather than authorizing a guessed return.
- `OBS_SIDE_NEAR_MM=OBS_FAR_MM`: maximum valid side echo range. Projected
  position relative to the original line determines obstacle membership.
- `OBS_SENSOR_TO_AXLE_MM=110` and `OBS_ROBOT_REAR_MM=65`: measured geometry
  used in clearance; verify physically before tuning around a mismatch.

Changing a parameter requires rebuilding and reflashing. The earlier
proposal to add lateral padding has not been implemented.

## Validation

Run `python tools/test_obstacle_integration.py --cc gcc`,
`python tools/check_app_standard.py`, and
`python tools/test_network_owner_boundary.py`.

The native tests cover the unchanged reviewed planner, side acquisition,
short dropouts, long obstacles, echoes beyond 500 mm, body/encoder travel,
left/right return, object reappearance, stopped sample publication, early
IR marks, rescan/reconfirmation, and the never-acquired distance bound.
A production-worker test verifies continuous side samples, front peeks,
re-aiming within the freshness budget and servo parking without fake data.
Controller STOP/START, RPM, telemetry and autonomous navigation regressions
are retained. The full return path includes IR alignment and handback.

The controlled USB-CDC Wi-Fi/MQTT build validates linked diagnostics and
MQTT owner linkage before copying the UF2. Its log is
`tmp/obstacle_tests/obstacle-side-tracking-build.log`.

## 2026-11-05 update: preferred side, servo aim, odometry return

- `OBS_PREFERRED_SIDE` (default 1 = left; -1 right; 0 scan decides). If the
  preferred side is crowded by another object the other side is used; if
  both are, the preferred side is widened to pass outside everything the
  scan saw. A seen object no longer produces `NO_ROUTE`. A mid-bypass
  rescan keeps the side already taken. Something ahead on the way out now
  stops and rescans instead of faulting.
- The servo points at the object from the turn out until it is passed.
  `OBS_PASS_FRONT_GLANCE=0` (default) means no front glances while passing,
  so nothing checks the path ahead until the return; set 1 for the old
  glance every `OBS_GLANCE_EVERY` side pings.
- Return: odometry decides. The robot turns back at `OBS_RETURN_DEG` and
  straightens up where the straight-on line should be, whether or not IR
  saw black. Black near that point only corrects the estimate. If no black
  is seen from the straightening turn through `OBS_VERIFY_CYCLES` of line
  following, it zig-zags at `OBS_SEEK_DEG` across +/-`OBS_SEEK_BAND_MM`
  for up to `OBS_SEEK_LEGS` legs, then stops with `LINE_LOST`.
- Limitation: "where the line should be" is the straight extension of the
  line at the point the obstacle was confirmed. A curve after the obstacle
  is not known to the robot; only the IR seek can find it.
- Unseen object: if the side watch never confirms the object (a smooth
  face at the 70 degree look can reflect the ping away), the sensor passing
  `OBS_UNSEEN_DEPTH_MM` beyond the front face stands in for the end, and the
  normal body-clearance run and fresh-absence check still apply. Later side
  echoes switch back to normal tracking; the return front guard catches a
  longer object. `OBS_SIDE_LOG_MM` prints a side-watch summary while passing.
- Return clearance (supersedes the 215 mm / rear-body rule above): the
  turn back pivots on the wheel nearest the object, so the rear swings
  away. The axle only needs the side-corner clearance for the return arc,
  at least `OBS_PASS_MARGIN_MM`, past the object's rear. For a seen object
  the rear is bounded by where the first empty ray crossed its side.
- Edge estimate: an edge now reaches as far as the next (empty) scan ray
  at the edge sample's depth, not half a step. Flat faces stop echoing
  beyond about 30 degrees, so wide obstacles look narrower than they are;
  `OBS_MIN_LATERAL_MM` sets a floor on the passing offset when the widest
  obstacle is known.
- Drive out: `OBS_PIVOT_GAIN_PCT=0` makes the robot reach the full passing
  offset before pivoting parallel. The pivot's outward carry (up to half a
  track) is no longer counted, because the stopped inner wheel slides; it
  becomes spare clearance for that wheel.
