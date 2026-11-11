# Changelog

Condensed from the release notes and patch READMEs that used to sit in the
repository root. Most releases were distributed as ZIP snapshots or patch ZIPs,
which is why some entries mention installers and backup folders. Detailed
evidence for V9 onwards (test logs, size audits, checklists) is in
`docs/V9/` … `docs/V15/`. Test counts are what each release reported at the time.

## Unreleased

- Obstacle bypass: the straight leg could overshoot to the 1200 mm
  `OBS_PASS_MAX_MM` fault whenever the side watch could not confirm the
  object's end. A single unconfirmed echo turned off the never-seen fallback,
  and a seen object had no fallback at all. The new `OBS_PASS_DEFAULT_MM`
  (which replaces `OBS_UNSEEN_DEPTH_MM`) ends the leg in all of those cases,
  while a confirmed end still finishes earlier.
- Bypass side choice: sides within `OBS_SIDE_PREFERENCE_THRESHOLD_MM` count as
  centred and use `OBS_SIDE_DEFAULT`. `OBS_DEBUG_LOG` prints both side
  clearances, the choice, and how and where the straight leg ended.

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
