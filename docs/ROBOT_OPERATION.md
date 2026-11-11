# Robot operation, calibration and bench testing

How the firmware behaves on the car, the settings you are expected to calibrate,
and the order to test things in. **`app_program/app/app_config.h` is the source
of truth**; every setting there is commented. Values below are the committed
defaults at the time of writing. Change a value only after measuring on the car,
then rebuild and reflash.

## Control model

- **One motor writer:** `app/task_controller.c` → `modules/motion/motion.c` →
  `drivers/motor.c`. Barcode, IMU, obstacle and network code never drive the
  motors directly.
- **Network isolation:** the CYW43 Wi-Fi task owns lwIP and calls
  `mqtt_utk_poll()` (`lib/libnet/lwip/mqtt_utk.c`). Application tasks exchange
  bounded, non-blocking queues with it (`mqtt_utk.c`): 8-frame telemetry TX,
  8-frame priority ACK, 8-frame command RX and an urgent 2-frame STOP queue.
  Overflow increments drop counters. Wi-Fi joins time out
  after 10 s; networking never delays motor, IR or IMU startup.
- **Live-control build (`G7_LIVE_CONTROL=1`):** boots in remote hold with zero
  PWM. Motion needs an accepted mapping, then START, with fresh IR, IMU
  calibration, an armed scanner and an MQTT connection. Readiness is checked
  both when START is requested and when the controller executes it. Losing
  MQTT while running requests a local STOP, and **reconnecting never resumes**:
  only a fresh explicit START does.
- **STOP** writes zero to all motor outputs before resetting run state. It keeps
  sensor baselines and the route mapping. A navigation safety stop (mission 5)
  is cleared by STOP. A genuine controller fault (mission 7) stays latched.

## Line following and junctions

Two digital IR sensors (left GP6, right GP2) are sampled every 1 ms
(`APP_IR_PERIOD_MSEC`); the controller runs every 50 ms
(`APP_CONTROLLER_PERIOD_MSEC`). A junction is both sensors black over
`APP_NAV_JUNCTION_CONFIRM_MM` (5 mm) of travel, or
`APP_NAV_JUNCTION_CONFIRM_MAX_MSEC` (1500 ms) if the robot barely moves.

`app/nav_core.c` executes the turn mapped to the last **validated** barcode
letter. An unmapped or ambiguous junction, a missing reacquisition or a
maneuver timeout (`APP_NAV_MAX_TURN_MSEC` 6000, `APP_NAV_MAX_CROSS_MSEC` 1800)
gives a recoverable safety stop. Turns use measured encoder counts
(`APP_NAV_TURN90_COUNTS`, from `APP_NAV_TRACK_MM`). The robot waits up to
`APP_NAV_DECODE_WAIT_MSEC` (8000) for a decode before a junction.

Two line sensors cannot tell every junction from a barcode stripe or a lifted
robot, so a both-black reading without a pending mapped action stops safely.
Changing that needs measured traces or more sensors, not looser limits.

## Barcode scanning

The scanner is in `modules/barcode_scanning/`. The barcode sensor's analog
output goes to GP27/ADC1 (`APP_BARCODE_ANALOG_ENABLED=1`) and is sampled with
wheel counts by TIMER alarm 2 every 2 ms (`BC_FAST_PERIOD_USEC`). Decoding runs
in a worker task, never in the interrupt or the IR task. The wide-bar layout
decoder (`APP_BARCODE_LAYOUT_ENABLED`) decides.

| Setting | Default | Notes |
|---|---|---|
| `APP_BARCODE_ADC_THRESHOLD` / `_HYSTERESIS` | 2200 / 50 | ADC codes 0–4095. Tune from the `[BC ADC]` white/black readings |
| `APP_BARCODE_SCAN_RPM` | 15.0 | Proven scan speed. Change only with measured print widths and success rates |
| `BC_CAL_MIN_BARCODE_UNITS` / `MAX` (`bc_cal.h`) | 1300 / 4800 | A real minimum-width gate: prints that are too small cannot pass |
| `BC_CAL_TEST_IGNORE_INPUT_ERRORS`, `BC_CAL_TEST_RELAXED_CAPTURE` | 1, 1 | **Bench relaxations.** Review before production navigation |
| `BC_WAVE_WHITE_SAMPLES` (`bc_wave.c`) | 250 | Consecutive white samples (~0.5 s) before rearming for the next barcode |

The worker publishes one transaction (sequence, capture, decode, result) under
one semaphore. Navigation consumes it independently of the telemetry ACK. A
completed scan never starts the motors. A failed decode is reported
(`SCAN FAILED`) but does not itself stop the robot.

At 15 RPM a 70 mm wheel moves about 55 mm/s, so samples are about 0.11 mm
apart. That is a nominal wheel-surface figure, not measured sensor travel.
Small prints may need a lower sensor or a slower speed, not only faster
sampling.

To collect raw passes for offline tuning, build with `G7_BARCODE_TEST=1`
(motors off, push by hand) and follow
[tools/barcode_data/README.md](../tools/barcode_data/README.md).

## IMU: descent, collision, heading

The LSM303DLHC is on I²C GP16/GP17. `APP_IMU_ENABLED` is 1 in normal builds
and 0 in the barcode test build. The robot must be still and level at boot
for calibration.

- **Descent governor:** the IMU's hump-descent signal reduces wheel targets to
  `APP_DOWNHILL_REDUCED_PERCENT` (65%, so 9.75 RPM from 15), changing by at most
  5 points per 50 ms. Sustained overspeed above `APP_DOWNHILL_RUNAWAY_RPM`
  (40) latches a STOP. Pitch-based slope detection additionally needs
  `APP_DOWNHILL_NOSE_DOWN_SIGN`, which is **0 (disabled)** until measured. At
  boot the firmware prints `[RAMP] IMU descent control ON; pitch sign
  unverified`, or `Governor enabled, nose-down sign=N`.
- **Measuring the pitch sign:** with motor power off, boot level, then slowly
  tilt the front down while watching IMU pitch in the dashboard Console. If
  pitch rises positive, set the sign to `1`; if it goes negative, set `-1`.
  Confirm that nose-up gives the opposite sign. Then rebuild. Downhill
  transitions print `[RAMP] DOWNHILL -> 65% wheel targets`.
- **Collision stops:** `APP_COLLISION_ENABLED` (default **0**). When 1, an IMU
  impact latches a STOP. The threshold is uncalibrated, and false positives
  leave motion stopped.
- **Heading:** the magnetometer heading is level-only and not calibrated to
  true north.

## Obstacle avoidance

Compiled in only when `APP_OBSTACLE_ENABLED=1` (default **0**). It uses the
ultrasonic sensor (TRIG GP4, ECHO GP5) on a servo (GP15). See
[app_program/modules/obstacle/README.md](../app_program/modules/obstacle/README.md).

## Odometry

`config/g7_odometry_config.h` sets `G7_ODOMETRY_MODE=1` (nominal) from
`board_config.h`: 1056 counts per revolution and a 70 mm wheel. That is a
**provisional** estimate. Measure both wheels over a full rotation before
trusting the route drawn on the dashboard.

## Serial sensor line

`[SENS] IR L=… R=… B=… roll_cd=… pitch_cd=… heading_cd=… hump=… sonar=… phase=… encL=… encR=… valid=…`

Angles are in centidegrees. `sonar=65535` and `phase=255` mean unavailable, and
`heading_cd=65535` means no valid heading. The `valid` bits are IR=1, IMU=2,
ultrasonic=4, motion=8 and heading=16, so `valid=27` means everything except
ultrasonic.

## Bench commissioning order

Keep the wheels raised and a physical motor-power cutoff within reach
throughout. The full per-release checklist is in
[docs/V15/PHYSICAL_CHECKLIST.md](V15/PHYSICAL_CHECKLIST.md).

1. Motor power off. Flash, then confirm `[NET] wifi=1 … mqtt=1` and live data in
   the Console.
2. Cover each IR sensor in turn and check its bit changes. Tilt the chassis and
   check roll and pitch.
3. Spin each wheel by hand and check that the encoder counts change with the
   expected sign.
4. Send the mapping and wait for the ACK. Issue one START: check that commanded
   PWM is non-zero and that the wheels turn. Issue STOP: PWM goes to 0/0 and the
   wheels stop. Repeat STOP; it should do nothing.
5. Disconnect the hotspot while running: the car must stop locally and **not**
   restart when Wi-Fi returns.
6. Pass large `*A*`, `*F*`, `*Z*` labels at the scan speed. Expect exactly one
   success per pass, rearming on white, and a reported failure for invalid
   prints. Repeat across print sizes before claiming accuracy.
7. Test STOP during a scan, approaching a junction and mid-turn. Then try
   junction turns on a mock track, and only then a guarded floor test.

An ACK means the request was accepted. Zero commanded PWM does not prove the
wheels are stationary.

### Suggested acceptance gates

- 0 unexpected moves after STOP, fault or reconnect in 20 deliberate
  interrupts.
- 0 false letter-to-turn executions in 60 controlled barcode passes.
- At least 19/20 successful decodes of *large* prints in a stated
  speed/print/orientation setup before claiming 95%; report smaller prints
  separately.
- All four mapped actions at a simulated and then a physical junction,
  including reacquisition, with PWM and encoder evidence.
