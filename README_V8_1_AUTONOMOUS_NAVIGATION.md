# INF2004 G7 — V8.1 autonomous navigation integration (bench candidate)

**Base:** V8.0 improve-movement + MQTT, with the user's newer buffered analog Code 39 scanner. **Status:** source/host-verified, **not** flashed, ARM-linked or physically tested in this environment.

## What changed

- `bc_wave.c` now permits automatic rearming after the result-processing phase in live-control builds. Frozen samples are never reused until decoding completes. The one-shot *diagnostic* motor hold is no longer a motor interlock in the live profile; STOP, MQTT connection, IR freshness, IMU and controller faults remain authoritative.
- The existing `bc_port` result mailbox now publishes **rejected buffered scans**, not just successful letters. The dashboard's existing scanner rejected/completed counters therefore have an actual source for V8.1 captures. A noisy stripe is **not** considered a valid letter.
- Decoder tries and per-width USB prints are suppressed in live mode to reduce low-priority task blocking. The scanner still captures on RP2040 TIMER alarm 2 every **2 ms**; it does not decode in an interrupt.
- A small, host-testable `nav_core` state machine consumes *mapped, validated* decoded letters and waits until a **sustained** both-black junction with sufficient measured encoder displacement. On unmapped/ambiguous junctions, missing reacquisition or maneuver timeout it commands a safe stop and latches the controller's existing fault state.
- Motion remains single-writer through `task_controller.c` → `motion.c` → `motor.c`. `LINE_UNCERTAIN` and STOP now apply immediate physical zero-PWM commands and publish zero commanded duty instead of feeding a zero target through the PI loop. Left/right turns use a bounded **forward outer wheel / stopped inner wheel** command; U-turn uses bounded forward/reverse wheel duty with original tested forward polarity preserved. Reverse output **must** be verified on a wheels-raised bench; direction was not previously available in this BSP wrapper.
- The controller sends navigation event kinds 12–19 through the same MQTT mission-event structure. The dashboard maps them to readable names. Mapping is looked up from the Pico-acknowledged mapping in the MQTT store, not from a guessed UI selection. No second barcode decoder was introduced.
- A completed scan never starts the motors. Reboot/reconnect/STOP still requires an explicit START and fresh readiness checks. Any STOP applies motor-zero **before** clearing the navigation mailbox.

## Actual scheduling in source

| Worker | Existing rate | V8.1 |
|---|---|---|
| Barcode ADC/encoder sampler | 2000 µs hardware alarm | unchanged, scan processing decoupled |
| IR sampler | 1 ms | unchanged |
| Controller and PWM commands | 50 ms | unchanged |
| MQTT application task | 50 ms | unchanged |
| MQTT priority STATUS | 1000 ms | unchanged |
| Barcode scan speed target | 10 RPM | enabled **during active buffered capture** only |
| Line-following target | 15 RPM | preserved |

## Bench-only navigation configuration

Located in `app_program/app/app_config.h`:

- `APP_NAV_JUNCTION_CONFIRM_MSEC=150`: sustained both-black before junction confirmation (not a single latched spike).
- `APP_NAV_MIN_TURN_MSEC=250`: minimum maneuver before a new edge may be considered.
- `APP_NAV_MAX_TURN_MSEC=2500`: bounded left/right/U-turn timeout.
- `APP_NAV_MAX_CROSS_MSEC=1800`: bounded straight-crossing timeout.
- `APP_NAV_BARCODE_TO_JUNCTION_MIN_COUNTS=120`: encoder displacement after barcode; not verified physical millimetres.

`MOTION_TURN_PWM_MAX=1700` and `MOTION_TURN_PWM_STEP=250` are preliminary duty limits in `motion.c`, **not measured wheel turn rates**. Do not tune these on the floor before the reverse-polarity and STOP tests.

### Barcode print size constraints

The contributed scanner's `bc_cal.h` uses `BC_CAL_MIN_BARCODE_UNITS=1300` and a maximum of 4800 encoder-sum units in the current relaxed profile. This is a **real minimum-width gate**: sufficiently small printed barcodes cannot pass even if ADC transitions are observed. The waveform envelope in `bc_wave.c` also uses `BC_CAL_MAX_BARCODE_UNITS / 12` as its width scale. V8.1 preserves the working thresholds rather than claiming that arbitrary small barcodes have been validated. During the physical test below, collect measured encoder-unit spans for each printed size, then separately adjust and retest these limits and waveform scale, plus the printer contrast and sensor standoff.

At the nominal 65-mm wheel diameter, 15 RPM is approximately 51 mm/s, meaning a 2-ms sample moves ~0.10 mm; 10 RPM yields ~34 mm/s or 0.068 mm/sample. These are **nominal wheel-surface speeds**, not measured effective barcode-sensor travel. Minimum print element width and optical footprint are unknown; smaller bars may need a closer/lower sensor and slower speed, not only faster sampling.

## Physical commissioning: do not skip

1. Disconnect drive power while flashing; support the robot so wheels cannot contact the floor. Keep a physical motor power cutoff reachable.
2. Confirm Pico and dashboard on same hotspot, MQTT connected, fresh IR and IMU, acknowledged four-letter mapping, stationary robot, zero PWM.
3. Issue **one START**, observe left/right command PWM and measured encoder counts. Issue **STOP once**, verify commanded PWM reaches zero and wheels actually stop. If not, abort and cut power.
4. Slowly move valid large `*A*`/`*F*`/`*Z*` barcodes under the GP27 sensor at controlled speeds. Confirm *one* valid letter/event, auto-rearm, and **continued** normal motion without another START. Verify invalid captures increment rejected counts without triggering navigation.
5. Repeat at least 20 trials per letter, orientation, and three print widths. Record: detected letter, success/failure, false positives, speed, edge gaps, ADC contrast, encoder travel. Do not claim reliable narrow-width scanning from host tests.
6. For junction maneuvers, use a controlled mock track with raised wheels first, then a guarded floor test in a clear enclosure. Verify left/right direction physically, 90° turning behavior, and U-turn reversal. **Do not assume the 250/2500-ms timings are correct; they are fail-safe limits, not angle feedback.**
7. Verify STOP during a scan, while approaching a junction, and during a turn; verify motor outputs zero before reconnect/timeout recovery. Verify no automatic restart when Wi-Fi returns.
8. If both IR sensors see black from lifting the robot or from a barcode marking, expect fail-safe stop. **Only two digital line sensors cannot guarantee unambiguous four-way junction identification or proof of line reacquisition.** Hardware validation and potentially a richer line sensor array are required before autonomous unattended driving.

### Suggested acceptance gates for a later PR

- 0 unexpected moves after STOP, fault or reconnect in 20 deliberate interrupts.
- 0 false letter-to-turn executions in 60 controlled barcode passes.
- At least 19/20 successful *large* barcode decodes in a specified speed/print/orientation test before claiming 95% in that test, and report smaller-print success rates separately.
- All four mapped actions at a simulated and physical junction, including reacquisition and PWM/encoder corroboration; **do not call this complete until it passes on hardware**.

## Build and run (Mac)

```bash
cd ~/Downloads/g7-improve-movement-live-v7
bash tools/test_autonav.sh
bash tools/test_controller_remote.sh
cd app_program/dashboard && node --test --test-concurrency=1 tests/*.test.js && cd ../..
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh
```

Only flash `out/g7-improve-movement-mqtt.uf2` if the ARM build and linked ELF marker checks succeed. The pre-existing V7.9 diagnostic marker remains unchanged; the addition is source-backed, not a claim that a V8.1 firmware has been flashed.

Run the dashboard with `bash run_live_dashboard.sh`. Normal MQTT telemetry does not need VS Code Serial Monitor.

## Validation performed outside the car

- 53/53 dashboard tests passed; 35 navigation assertions passed; controller handoff passed; native C telemetry protocol tests passed.
- 94 application `.c`/`.h` source files, zero style errors.
- 11 relevant application C files passed host `-Werror` syntax checks with RP2040/live-build macros; **not an ARM compile**.
- The monolithic historical `python tools/test_app.py --cc cc` test is **already broken in unmodified V8.0**: the old host kernel stub lacks `T_CSEM`, `tk_cre_sem`, `tk_wai_sem` and `tk_sig_sem`, while the contributed barcode port uses them. This V8.1 change does not pretend that baseline suite passes.
- The latest contributed barcode decoder previously had an existing native assertion failure; no unsupported correction to the actual pattern matching was made.
- **ARM toolchain, Pico SDK and robot unavailable** in the implementation environment: the complete firmware link, narrow barcode success rate, PWM pin reversal and physical turns are not verified.

## Rollback

The guarded installer writes a timestamped backup under `.g7_v8_1_nav_backup/`. Stop the dashboard and motors, restore listed files from that directory, remove the new `nav_core.c` and `.h` and tests if they did not previously exist, then rebuild/reflash the earlier known-good UF2. Do not restore an old UF2 while the robot is powered for motion. It is also possible to restore a Git commit after reviewing local uncommitted work and retaining Wi-Fi credentials.
