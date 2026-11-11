# INF2004 G7 V13 — Calibrated downhill speed governor

**Release classification: host-tested implementation checkpoint.**
No Pico W firmware build, real pitch-sign calibration, braking evaluation, or
full-course physical tests were possible here. V12 is preserved as rollback.

## Production changes

- New pure, controller-owned `modules/motion/downhill.c/.h` computes a downhill
  decision from IMU relative pitch. The default sign is **0 (disabled)**;
  the hardware cannot infer nose-down sign from a stationary level baseline.
- Setting `APP_DOWNHILL_NOSE_DOWN_SIGN` to `1` or `-1` **after physically
  verifying the direction** activates the governor. Input pitch must be fresh
  (<= 200 ms), calibrated and valid. Bad/NaN/out-of-range input is rejected.
- Pitch filtering uses integer tenths of degrees, engage at +5 degrees for
  3 controller periods, release below +3 degrees for 4 periods, with a
  5-percentage-point step back to full speed. Initial derate is **65%** of
  the requested forward target, unvalidated on the course.
- A new downhill state is not acquired when absolute roll is >= 8 degrees;
  this avoids treating a one-wheel hump as verified ramp descent. Once
  engaged, a transient roll event cannot cancel the speed restriction.
- The reduction is applied equally **after** the original steering split.
  Left turns retain slower left / faster right targets; right turns retain
  slower right / faster left. Forward obstacle bypass commands are reduced
  too. Encoder PI still owns the individual wheel speed loops.
- When a downhill wheel is already above target by more than 1 RPM, its
  applied motor PWM is capped at zero rather than using stall-boost or PI to
  accelerate it. Three consecutive samples exceeding 130% target + 2 RPM
  latch a safety STOP. A stale IMU during driving for four controller periods
  also latches a STOP. Very small inner-wheel targets (<5 RPM) are excluded
  from this overspeed check to avoid a false trip when dragged through turns.
- The STOP uses the existing short-brake path for at most four 50-ms cycles,
  then a zero-output stop. The fault stays latched until the controller
  receives an explicit STOP; a separate authorized START is still needed.
  **This is not a guarantee of holding the car stationary on a steep ramp.**
- The motor controller remains the only motor PWM writer. No extra tasks,
  MQTT topics, frame version changes, or queue ownership changes were added.
  Existing barcode, obstacle, collision, mapping and telemetry code remains.
  The barcode-only motor-free profile excludes downhill control explicitly.
- Dashboard robot logs show `[RAMP]` status, including disabled at boot,
  active slowdown, restored speed and safety stop.

## Exact sign verification before activation

1. Keep motors disconnected from power. Place the fully mounted car level and
   motionless, reboot, and wait for the IMU calibration to complete.
2. On the direct-MQTT sensor telemetry console note the live roll/pitch values
   (not magnetometer heading). Level pitch should be near 0 degrees.
3. With the car **stationary**, tilt the FRONT downward about 8–10 degrees.
   Check the sign and repeat several times. Nose-up must give the opposite
   sign. If the readings are stale, noisy, invalid or ambiguous, leave sign 0.
4. Edit `app_program/app/app_config.h`: change
   `#define APP_DOWNHILL_NOSE_DOWN_SIGN 0` to `1` if nose-down pitch is
   positive, or `-1` if nose-down pitch is negative.
5. Rebuild the UF2 and confirm the boot log displays the selected sign.
   Do NOT use a guessed sign or a firmware from a different source folder.

## Host tests

From the V13 root on macOS:

```bash
bash tools/run_host_tests.sh  # replaces the version-named runner used at the time
```

Tests compile the pure governor for signs 0/+1/-1 and the actual motion
controller for +1/-1, verify steering ratios and obstacle drive targets,
reject unsafe tilt and stale samples, and verify fail-stop latching.
The existing V12 obstacle/remote/telemetry, nav, precision/min-bit and
73 dashboard tests run unchanged. Details in `host_test_results.txt`.

## Target build on the user's Mac

```bash
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
  PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh
ls -lh out/g7-improve-movement-mqtt.uf2
```

Disconnect motor power, raise wheels, hold BOOTSEL while plugging in USB,
and only flash a UF2 from a successful **new** build. Keep a physical power
cutoff. Check STOP before actual floor/ramp driving.

## Week 13 bit-width policy

The new runtime state is updated exclusively by the controller task and is
not accessible to ISR writers, so packed read-modify-write is single-owned.
All added flags, counters, angle storage and scale store have minimum
necessary logical widths. Calculations expand them to 32-bit ints or floats
before filtering, comparison or multiplication; no unsafe truncation of
IMU inputs or RPM. See `new_state_bit_inventory.csv`.

Existing V11 minimum-bit audit and prior incomplete protocol review remain
as documented. V13 is **not** a certification that all of Change Requirement 1
is completely satisfied; MQTT protocol envelopes were intentionally unchanged.

## ARM evidence limitations

`clang --target=armv6m-none-eabi` successfully emitted ARM Cortex-M0+ object
files for the production motion, downhill and controller C modules using a
host-stub include path. This is **not a full SDK firmware build or final
link**. The partial object comparison (`arm_object_subset.csv`) shows an
8-byte increase in motion-module `.bss` for the new owner state. It is
not whole-robot RAM, heap, RTOS stack or flash consumption. Confirm the
finished UF2, ELF/map and real timing with your local Pico SDK and toolchain.

## Physical test matrix (NOT VERIFIED)

- Flat travel: confirm original RPM, line following and barcode scanning.
- Downhill 5/8/10 degrees: verify correct sign, 3-period activation,
  lower left/right wheel targets and wheel-speed measurements under gravity.
- Left/right turns on downhill: verify inner wheel stays slower with safe
  line tracking; test a barcode just before a junction.
- Uphill and levelling: confirm no false downhill detection and gradual
  target recovery; cover noisy sensor data and one-wheel humps.
- Obstacle bypass/return while pitched downhill: confirm clearance, stop
  priority and original-line reacquisition.
- Speed runaway, unplugged IMU and MQTT loss: record encoder speed, motor
  commands, state latch and no automatic restart. Ensure a human can cut
  motor power; remote MQTT STOP is **not** a hardware emergency stop.
- Normal acceleration/braking, ramp edges and controlled collision stimuli:
  check collision-jerk false positives and genuine impact stopping.
- Repeated trials with independent real speed/angle measurements, CPU/stack
  headroom, timing and RAM/flash logs. Adjust thresholds only using evidence.

**Motor power alone may not provide braking torque sufficient to control
speed on a steep descent. Do not treat a reduced PWM target as proven safe
without ramp trials.**
