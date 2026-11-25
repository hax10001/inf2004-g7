# G7 Tribunal 2 pre-tribunal requirements: proposed corrected wording (23 Nov)

**This is a proposal, not the submission.** Every number below was checked against the code as it is now (branch `nov19-smooth-follow-junction` plus today's uncommitted changes). Evidence is in `docs/tribunal2/requirements_audit.md`.

**Source of the "submitted" wording.** I did not have the filled-in submission file. The only copy on disk is the blank template (`~/Downloads/Tribunal_2_Pre_Tribunal_Submission_Template.docx`). The submitted wording used for the diff in section 4 is the short form relayed to me today.

**What the template asks for.**
- Section A1 is "Final / Updated Requirements": maximum 2 pages, an `ID | Final / Updated Requirement` table.
- It says: "Use the latest requirements, not the original Week 6 requirements. No lengthy justification is required in this section."
- A2 is the software block diagram (maximum 2 pages), and the whole submission is at most 4 pages.
- So justifications are not required in the pre-tribunal submission.

Justifications are needed in the live Tribunal 2 text file (`Tribunal_2_Immediate_Tribunal_Template.txt`). That file asks for every change: Accept / Modify / Reject / No Change, plus "Why?" and "Engineering Evidence". Section 2 below is written so it can be copied into that file.

Section 1 is the text to paste into template A1. It has no justification column, because the template does not ask for one. The large changes carry a short reason inside the row, so the whole submission still fits the 4-page limit. Everything after section 1 is notes for Linus, not for submission.

## 1. Proposed requirement table (paste into template A1)

| ID | Final / Updated Requirement |
|---|---|
| FR1 | The robot shall refuse START (coded NOT_READY) while any of these holds: MQTT offline, controller not in remote hold, a START/STOP still pending, IMU not calibrated, IR reading missing or older than 200 ms, barcode scanner not armed, obstacle sensor fault, or either motor PWM non-zero. |
| FR2 | SET_MAPPING shall load four distinct letters A–Z, in the order Straight, Left, Right, U-turn. It is rejected while running or committed, and the mapping applies to the whole run. |
| FR3 | At boot the robot shall calibrate the IMU from 50 samples, accepting a gravity magnitude of 800–1200 mg. It shall centre the servo at FRONT = 1540 µs and learn the barcode sensor's white level from 250 samples. The line IR sensors are digital and need no learning. |
| FR4 | The robot shall follow an 18 mm line at 30 RPM. It steers with a PID line observer: offset, estimated heading across the line, and an integral over distance travelled. After 50 mm without the line it steers gently towards the line's last side; after 100 mm it stops and pivots to search. |
| FR5 | The robot shall decode Code 39 in both directions from the ADC barcode sensor on GP27. A dark run shorter than 1000 encoder units is discarded. A decoded letter is sent as an EVENT and handed to navigation. |
| FR6 | The robot shall confirm a junction after 7 mm of both sensors black, at least 120 encoder counts after a barcode. It then advances the axle 134 mm (125 mm sensor-to-axle + 9 mm) before pivoting. With no decoded letter it waits up to 8 s, or 1.5 s after the decoder goes idle, whichever comes first. |
| FR7 | Each wheel shall have its own PI speed controller (Kp 0.1, Ki 0.5, with feed-forward and anti-windup), an 8 RPM/period ramp, 1056 counts/rev (nominal) and a 70 mm wheel. A stop brakes for 200 ms. |
| FR8 | A hump shall start when tilt exceeds 3° for 3 samples (20 ms apart) and end when it stays under 1.5° for 15 samples. Its height is the larger of 110 mm × sin(peak tilt) and the integrated climb, limited to 0–127 mm. It is discarded if the robot travelled under 40 mm on it, or was lifted or within 3 s of a lift. |
| FR9 | An obstacle shall be confirmed after 3 consecutive median-of-3 ultrasonic readings under 250 mm. |
| FR10 | The scan shall sweep from 60° left to 70° right of FRONT in a single left→right pass: 5° steps within ±50°, 10° beyond. It re-votes the edges at 20–45° with 5 pings each, applies a 40 mm margin and a 245 mm minimum side move, breaks ties within 15 mm, and treats 3 echoes beyond 400 mm as clear. |
| FR11 | The bypass shall end the pass after 5 absent side echoes (600 mm limit) and turn back at 60°. It drives until a line sensor sees black, advances the axle onto the line, and pivots until the line is between the sensors. It verifies for 10 periods. Otherwise it uses up to 4 ±30° search legs, then hands back to line following. Up to 10 rescans are allowed. |
| FR12 | Telemetry shall be compact binary MQTT: STATUS 1 s, SNAPSHOT ~250 ms, SENSOR 1 s, HEARTBEAT 2 s, ODOM on each 50 ms period with wheel motion, a CHECKPOINT every 500 mm, and EVENTs. The EVENT kinds are barcode, obstacle/bypass/fault, hump crossed, mission state, and navigation (junction, turn start/end, line reacquired, scan rejected, decode wait, safety stop). |
| FR13 | The robot shall accept only SET_MAPPING, START, STOP, SNAPSHOT, RECAL_IMU and RECAL_BARCODE. Every command is ACKed with a result code, and STATUS echoes the last transaction. There is no remote steering. |
| FR14 | The dashboard shall draw the route dead-reckoned from ODOM wheel deltas (130 mm track), bridging lost updates with 500 mm checkpoints. Heading is corrected at each junction turn (±90°/180°). Markers are placed at the robot's pose for barcodes (with letter), junction turns (with direction), humps (with height) and obstacles. |
| FR15 | *Changed: safety must not depend on the network.* If MQTT is lost while the robot may move, it shall stop the motors in the next 50 ms control period, abort any bypass, clear the queued action and return to REMOTE_HOLD. The link is declared down on Wi-Fi loss, TCP error, or 15 s with no reply to 5 s pings. Reconnection never resumes motion; a new START is needed. Offline telemetry is held in bounded rings (32 ODOM, 24 EVENT) and drops are counted. |
| FR16 | A critical fault shall force PWM to zero. Navigation, IR-stale and safety stops recover only by STOP then START. Hardware/sensor errors (IR read, clock, obstacle sensor, bypass NO_ROUTE) stay latched until reset. A lift stop resumes after 1 s back on the floor. |
| FR17 | *New: the car drove off when picked up mid-run.* The robot shall stop when lifted more than 40 mm with no floor under the line sensors, and resume after 1 s back on the floor. |
| FR18 | *New: overspeed on descents.* Once a hump's measured climb reaches 3 mm and the detector reports the descending phase, the wheel RPM target shall be scaled to 65 %. A wheel above 40 RPM is cut, and a runaway lasting 10 periods (~0.5 s) is a safety stop. The pitch trigger stays off until its sign is calibrated (it is 0 now). |
| NFR1 | The firmware shall run on µT-Kernel 3.0 SMP (dual core) built with Pico SDK 2.2.0. `build_uf2.sh` rejects other SDKs and runs a BARR-C-subset style check before every build. |
| NFR2 | The IR task (priority 5) shall sample with a 1 ms delay loop (~16 ms between logged samples under load), with edge latches. A 2 ms timer ISR snapshots the barcode ADC and encoder counts into a 64-entry queue; encoders count in their own GPIO ISR. |
| NFR3 | *Changed from 5–10 ms: fast sampling moved to the 1 ms IR task and 2 ms ISR.* The controller task (priority 6) shall run every 50 ms. It waits at most 2 ms for the command mutex, and stops the motors after 4 failed attempts in a row. |
| NFR4 | *Changed from 100 Hz: a hump lasts over 1 s, so 50 Hz suffices and halves I2C load.* The IMU task (priority 7) shall run every 20 ms with accelerometer smoothing (alpha 0.2). Data older than 200 ms is not used; data older than 1 s while moving stops the robot. |
| NFR5 | Ultrasonic pings shall time out at 30 ms, flag a stuck ECHO after 50 ms, be spaced 60 ms apart, and be median-filtered. |
| NFR6 | The barcode decoder shall use an ADC threshold of 2200 with 50 hysteresis, start a capture on a 600-count rise above learned white, and decode from the smoothed waveform. |
| NFR7 | Application code shall not call malloc/free. Task stacks are allocated once at boot by the kernel, and lwIP uses its own fixed 12 KB heap and pools. |
| NFR8 | The telemetry task and the barcode decoder shall run at priority 9, below all control tasks. Measured over 129,775 recorded frames: SNAPSHOT p95 interval 301 ms, sequence continuity 99.78 %. |
| NFR9 | Every stop path shall command zero motor output. A brake drives both pins high for up to 200 ms. |
| NFR10 | The encoder and barcode ISRs shall run at priority 2 and only count or copy; they never block or print. |
| NFR11 | Host tests shall pass before release: `tools/run_host_tests.sh` (149 checks, including the coding-standard check and simulations) and 107 dashboard tests. |
| NFR12 | *Change Requirement 1.* Application data shall use the minimum bits for its range. Calculated application record storage fell from 103,594 B to 69,598 B, and binary MQTT frames are 12 % of the same data encoded as JSON. |
| NFR13 | Every MQTT frame shall have a 13-byte little-endian header (magic, version, type, flags, session, sequence, ms, length) plus a CRC-8 (0x07). Frames with a bad CRC or length are rejected; reserved bits in payload fields are rejected. |

---

# Notes for Linus (not for submission)

**Rows worded to match the code where the code, not the wording, may be what should change:** FR10, FR13, FR16, FR18, NFR7, NFR9, NFR13.
- FR10: bearings read about +7° left in a one-way sweep; a correction is pending a measurement.
- FR11: the return is validated in simulation and one car run.
- FR13: RECAL_IMU is always rejected; the IMU calibrates at boot only.
- FR16: the hardware error latch needs a reset.
- FR18: the pitch trigger is off.
- NFR7: kernel stacks are allocated at boot.
- NFR9: three paths print or report before stopping (MQTT lost, mutex misses, navigation fault).
- NFR13: header reserved bits are not checked.

## 2. Changes since Tribunal 1 (Week 8), with justification

The Week 8 PDFs (`Week8_Tribunal1_Template-2/-3/-4.pdf`, versions A, B and C) reference the Week 6 requirements but do not contain their full text. Where no wording is quoted, the Tribunal 1 cell says "not available in files".

| Req. | Tribunal 1 (Week 6/8) | Tribunal 2 | Justification and evidence |
|---|---|---|---|
| NFR3 | "5-10 ms nominal loop" (Version A/B, Buddy 2) | 50 ms controller period | IR and encoder sampling moved to a 1 ms task and a 2 ms ISR, so the controller only decides once per 50 ms (~1.5 mm of travel at 30 RPM). It holds the remote mutex at most 2 ms. Evidence: host controller tests (`test_controller_handoff.c`). **NO EVIDENCE YET:** no on-target loop timing or jitter trace, which T1 asked for. |
| NFR4 | "nominal 100 Hz bounded I2C task" (Version A, Buddy 4) | 20 ms (50 Hz), alpha 0.2 | A hump at 30 RPM takes over 1 s, so 50 Hz gives 50+ samples and halves the I2C load next to Wi-Fi. Evidence: hump heights on the car (`HUMP DONE` 6–16 mm in recordings). **NO EVIDENCE YET:** no measured comparison with 100 Hz. |
| NFR2 | IR "2-5 ms sampling" (Version A, Buddy 3) | 1 ms IR task plus 2 ms barcode ISR and queue | Faster, ISR-timed barcode sampling decoded both directions. Evidence: barcode tests (`test_scanner_ready.c`, `barcode_layout.c`) and decoded letters in today's runs. Caveat: the logged IR interval under load is ~16 ms. |
| FR13 | "SET MAPPING, START, STOP/ABORT, SNAPSHOT" (Version A) | Adds RECAL_IMU and RECAL_BARCODE, coded ACK results and a STALE challenge | Stale commands after a reconnect must be refused (T6). Evidence: `test_controller_handoff.c`, dashboard `commands.test.js`. RECAL_IMU is not implemented yet. |
| FR15 | Not available in files. T1 risk R5/T6: "fixed capacity, overflow/drop counter … fresh snapshot on reconnect" | Stop on link loss, 15 s/5 s detection, 32/24 rings held offline | Safety must not depend on MQTT (Version B). The old 45 s silence allowed up to 45 s of driving after a silent loss. Evidence: the longest uplink stall on the hotspot was 4.3 s over 11,800 frames; `test_controller_handoff.c`; `tools/tests/test_telemetry_map.c` (ring retention). |
| FR14 | Not available in files (route reporting via incremental motion: Version B change log) | 130 mm track, checkpoints, turn-corrected route, per-kind markers | Version B chose incremental motion data. Markers were hidden by a sequence bug, and turns were over-drawn by 8 %. Evidence: `route-tracker.test.js` (11 tests); on the car, 133/134 events placed and 37 checkpoints 496–505 mm apart. |
| FR16 / NFR9 | "Week 6 FR16/NFR9 defines the existing safety requirements" (Version C) | Fault latching and lift resume stated explicitly | Version C asked to measure a stop under load. Evidence: controller host tests. **NO EVIDENCE YET:** no PWM-to-zero timing trace (T2). |
| FR17 | None (new) | Lift stop at 40 mm, resume after 1 s | The car was picked up mid-run and drove when put down. Evidence: `test_lift_math.c`, controller lift tests, and 10 lift stops logged today. |
| FR18 | None (new) | Downhill 65 % on the hump descent; pitch trigger off | Overspeed on descents. Evidence: `test_downhill_governor.c`, `test_ramp_motion.c`. The pitch sign is not yet calibrated on the car. |
| FR11 | Not available in files | Return by line sensor, axle onto the line, pivot until centred | Angled stops missed the line. Evidence: `obstacle_rejoin_sim.c` (±15°/±25° and curves; was failing before), and one successful bypass on the car today. |
| NFR12 | Change Requirement 1 (Week 8 Appendix A1) | Minimum bits, RAM and message figures | The injected requirement. Evidence: `docs/min-bits/*`, `test_minbits_all.py`. The RAM figure is calculated, not measured from a build. |

## 3. Not yet verified on the car

- The hump marker and "Hump crossed" EVENT: no hump was completed today.
- Offline buffering and the flush after a reconnect: the link never dropped today.
- The angled-approach rejoin: simulation only, plus one head-on-ish car run.
- The scan side offset (~+7°): waiting for measured centred-box trials. The side choice is still biased to the right.
- FR18's pitch trigger: off (`APP_DOWNHILL_NOSE_DOWN_SIGN 0`).
- NFR3/NFR9 timing: no on-target loop or stop-latency trace.

## 4. Row-by-row diff against the submitted wording

- **Unchanged:** FR1 (adds the two extra blockers), FR2, FR3 (adds the barcode-only note), FR5, FR17, NFR5, NFR8, NFR10.
- **FR4:** "PI steering" becomes "PID line observer"; the 50 mm gentle-steer stage is added.
- **FR6:** adds "or 1.5 s after the decoder goes idle".
- **FR7:** names the PI gains and anti-windup; 1056 counts/rev marked nominal.
- **FR8:** "110 mm × sin(roll)" becomes "larger of 110 × sin and integrated climb"; no minimum climb (the 23 Nov 5 mm minimum was removed on 24 Nov: course humps can be under 5 mm) and the lift discard.
- **FR9:** "at or under 250 mm" becomes "under 250 mm, 3 consecutive median-of-3".
- **FR10:** "−60 to +70" becomes "60° left to 70° right, single left→right pass"; offset note added.
- **FR11:** odometry return becomes sensor-terminated return; hand-back after 4 search legs instead of a fault.
- **FR12:** "SNAPSHOT 300 ms, ODOM 60 ms" becomes "~250 ms, per-period ODOM, 500 mm CHECKPOINT"; EVENT list corrected (hump crossed added).
- **FR13:** RECAL_IMU flagged as not implemented.
- **FR14:** adds 130 mm track, checkpoints, turn heading correction and per-kind markers.
- **FR15:** adds the link-down definition (15 s / 5 s); "first 50 ms period" becomes "next 50 ms period".
- **FR16:** adds which faults latch until reset, and lift resume.
- **FR18:** "65 % cap" becomes "65 % RPM target on hump descent after 3 mm of climb"; pitch trigger stated as off.
- **NFR1:** check is a BARR-C subset, run by `build_uf2.sh`.
- **NFR2:** real IR interval; encoders counted in their own ISR.
- **NFR3:** "2 ms mailbox, 4 misses" becomes "2 ms mutex wait, 4 failed attempts".
- **NFR4:** "alpha 0.2" is accelerometer smoothing, not a complementary filter.
- **NFR6:** threshold 2200; 600 is the capture-start rise; one-sample edge filter removed.
- **NFR7:** "no malloc" becomes "no application malloc; boot-time kernel stacks; lwIP fixed heap".
- **NFR9:** "pins zero before status work" becomes "zero output; brake pins high"; the 3 exceptions flagged.
- **NFR11:** "140 checks, 96 tests" becomes "149 checks, 107 tests".
- **NFR12:** "static RAM" becomes "calculated application record storage"; 12 % is versus a JSON encoding of the same frames.
- **NFR13:** "14-byte header" becomes "13-byte header + CRC-8"; reserved bits are checked in payloads only.
