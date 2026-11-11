# INF2004 G7 V14 — STOP/START recovery, collision event reset, smoother downhill

**Release type:** Preservation-first source patch against the provided V13.
**Test status:** Host regressions passed; target firmware link and physical-car trials NOT verified in this environment.

## Root-cause findings

- The IMU worker previously set `imu_history_t.b_collision_latched` to `true` and never reset it. Every subsequent IMU snapshot could therefore report the **same old impact**, even after an explicit STOP reset the controller to `ROBOT_REMOTE_HOLD`. A controller loop could immediately leave HOLD for STOPPED again. This explains a concrete persistent lockout path.
- `g7_controller_mission_state()` maps both STOPPED and REMOTE_HOLD to mission code 6. The firmware's separate `remote_hold` status bit provides the real distinction; the dashboard now says **STOPPED but not in remote hold** and tells the operator to issue STOP and wait for applied HOLD. STOP ACK alone is only receipt, not proof of physical actuation.
- V13's downhill logic zeroed an individual wheel's PWM when it was only 1 RPM above its reduced target and latched STOP after three samples above 130% + 2 RPM. Both could cause undesirable stop/go behavior during normal gravity-driven overshoot. This was changed to a smooth 5 percentage-point/cycle governor transition and normal PI correction for modest errors. A **sustained gross runaway** still latches STOP.
- The supplied junction log proves `Junction confirmed -> Left` and a transition to reported mission 6. It does not alone prove the cause was collision; IR staleness, a STOP command, or another controller stop could also explain it. New logs, physical testing and the improved status explanation are required before claiming the specific junction is solved.

## Source changes

1. `modules/terrain/task_imu.c` publishes an impact *event* rather than an un-resettable local latch. `modules/terrain/terrain.c/.h` holds the sticky event under its existing RTOS snapshot mutex and adds `terrain_acknowledge_collision()`.
2. `app/task_controller.c` acknowledges the impact on explicit STOP and controlled START rearm, without automatically authorizing movement; only a driving controller state responds to new collision events. If acknowledgement fails it stays safely stopped, logging the failure. Other hardware faults remain latched.
3. `modules/motion/downhill.c/.h` gradually changes the requested speed factor on ramp entry and exit, scaling the left and right RPM targets together *after* steering. No ordinary downhill state transition commands STOP. Mild overspeed is processed by the existing PI loop; the runaway guard applies above `APP_DOWNHILL_RUNAWAY_RPM` for ten control periods. Missing IMU data holds the last safe factor for up to 19 cycles before a fail-closed stop on the 20th.
4. `app/app_config.h` defines the initial unverified runaway guard at 40 RPM. This is a **bench-test candidate**, NOT a physical speed certification. The known 65% downhill target, calibrated-sign requirement and baseline motor/steering logic remain.
5. `dashboard/server/state.js` distinguishes mission-6 STOPPED from REMOTE_HOLD using the existing status bit. No MQTT topic, frame layout, command, mapping or telemetry wire version changed.

## Minimum-bit compliance of NEW state

- `downhill_governor_t.stale`: 5 bits for 0–20.
- `downhill_governor_t.overspeed`: 4 bits for 0–10.
- All other governor state fields retain their minimal widths and single-controller-task ownership.
- Collision event is still a 1-bit field inside `terrain_snapshot`, and all shared read/write/acknowledge operations remain protected by the existing terrain snapshot mutex. No unsafe cross-core packed read-modify-write was added.
- V11/V13's broader Week 13 unresolved range/protocol/ARM-RAM evidence still requires verification; this update does not certify the entire prior minimum-bit audit.

## Reproduce tests on macOS

```bash
cd ~/Downloads/inf2004-g7-v14-recovery
bash tools/run_v14_tests_mac.sh
```

The suite runs the production terrain collision event/ACK test, pure/production motion governor tests with nose-down signs -1, 0 and +1, the controller STOP/START handoff tests, network ownership, 144 navigation assertions, obstacle/telemetry integration, data-width tests, style check, and Node dashboard suite.

Verified here (GCC and Clang): 74/74 dashboard tests, 144 nav assertions, remote controller/STOP priority tests, terrain collision ACK test, obstacle and protocol host tests. Four affected C modules also emitted Cortex-M0+ ARM object files via Clang with host stubs; **this is not an SDK link/UF2**. No real robot was operated.

## Build and safe physical acceptance

1. Do not overwrite a known-good V13 or V12 project. Extract V14 as its own folder.
2. Configure local Wi-Fi/MQTT credentials privately; nothing sensitive is bundled.
3. Start with the verified `APP_DOWNHILL_NOSE_DOWN_SIGN=0` default (downhill governor off), then establish stable remote STOP -> HOLD -> START and barcode/junction actions with wheels supported. Check the `[CTRL]` and `[IMU]` lines in Robot Log around any unexpected STOP.
4. Compile the matching source on your Mac, with `G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh`.
5. Copy the freshly built `out/g7-improve-movement-mqtt.uf2` to the `RPI-RP2` boot drive. Never flash a stale artifact when compilation failed.
6. Confirm STOP applies physical zero PWM. Wait until fresh STATUS shows `remoteHold=true`, `ready=true`, and no blockers, then START manually. Confirm no auto-restart on disconnect, impact, or fault.
7. Run barcode G -> Left junction multiple times with live event/IMU/IR logs and record if the controller stops for a real collision, stale IR, navigation fault, or an intentional STOP. Recalibrate collision threshold if normal braking triggers false positives; never assume a safe value without physical trials.
8. Only after verifying *stationary* nose-down pitch sign, set `APP_DOWNHILL_NOSE_DOWN_SIGN` to +1/-1 and recompile. On a shallow supported ramp, verify smooth continuous target reduction, correct inner/outer wheel ratio, measured wheel speed, and an accessible independent power cutoff. Gravity may still cause overspeed; reduced PWM is not a mechanical brake.

## Explicit limits

- **Not verified:** real Pico SDK build, USB flashing, physical start/stop transitions, downhill speed behavior, collision threshold and full-track junction success. These require hardware and user-local network configuration.
- **Not claimed:** all of Change Requirement 1 satisfied, or a guarantee against physical runaway on an inclined surface.
- **Unchanged known issue:** older standalone barcode-core test has one pre-existing assertion failure; V14 does not alter the decoder.
