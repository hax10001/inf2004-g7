# Tribunal 2 pre-tribunal requirements: code audit (2026-11-23)

This is a read-only audit of branch `nov19-smooth-follow-junction`, including the uncommitted working tree. No source was changed and no robot commands were sent.

- Paths are relative to the repo root unless they start with `app/` or `modules/`, which are under `app_program/`.
- Live state comes from the running dashboard at 00:45 UTC, while the robot was driving.

Verdicts used: MATCHES, PARTLY (what differs), NOT IMPLEMENTED, and CANNOT VERIFY FROM CODE.

## Summary

| ID | Verdict | One-line reason |
|---|---|---|
| FR1 | MATCHES | All six START blockers exist, plus "not held" and "STOP/START pending". |
| FR2 | MATCHES | 4 distinct letters A–Z; the slot index is the action (0 Straight, 1 Left, 2 Right, 3 U-turn). |
| FR3 | MATCHES | 50 samples, 800–1200 mg, FRONT 1540 µs, barcode white-learn. Only the barcode sensor learns white; the line IR sensors are digital. |
| FR4 | PARTLY | The active steering is a PID line observer, not PI. 18 mm, 30 RPM and 100 mm are correct. |
| FR5 | MATCHES | Decodes both directions on GP27 and drops dark runs under 1000 units. |
| FR6 | PARTLY | 7 mm, 120 counts and 134 mm are correct. The 8 s wait is only a maximum; it also ends 1.5 s after the decoder goes idle. |
| FR7 | MATCHES | Per-wheel PI, 8 RPM ramp, 1056 counts/rev (marked unverified in its comment), 70 mm wheel, 200 ms brake. |
| FR8 | PARTLY | Height is the larger of 110·sin(roll) and an integrated climb. The thresholds are strict (> 3°, < 1.5°). |
| FR9 | PARTLY | Code uses `< 250`, not `≤ 250`. Each "reading" is a median of the last 3 pings. |
| FR10 | PARTLY | The numbers are right, but the servo sign is left-positive, so the code sweeps −70..+60. Step size is 5° within ±50 and 10° beyond. |
| FR11 | MATCHES | 5 / 600 mm / 60° / 10 / ±30° with 4 legs / 10 rescans. |
| FR12 | PARTLY | SNAPSHOT is every 250 ms (not 300). ODOM has no 60 ms period: it is sent each 50 ms poll when the wheels moved. Terrain sends no EVENTs at all. |
| FR13 | PARTLY | RECAL_IMU is always answered NOT_READY. Everything else matches. |
| **FR14** | **PARTLY: event markers are effectively never drawn in a real run** | Hump events are never emitted and checkpoints are never sent. A sticky "uncertain" flag hides every marker, all markers look alike, and the dashboard track width is 120 mm against the firmware's 130 mm. |
| **FR15** | **PARTLY** | Stop, bypass abort, queue clear, REMOTE_HOLD and no auto-resume all match. But "link down" can take up to 45 s, telemetry is not held in the 32/24 rings while offline, and a print runs before the stop. |
| FR16 | PARTLY | STOP then START only clears STOPPED and NAV_SAFETY_STOP. ERROR-class faults stay latched until reboot, and lift resumes on its own. |
| FR17 | MATCHES | 40 mm lift threshold; resumes after 1000 ms back on the floor. |
| FR18 | PARTLY | With sign = 0 the governor still runs, triggered by the hump "descending" phase. The 65 % scales the RPM target; it is not a PWM cap. |
| NFR1 | PARTLY | Kernel 3.0 and the SDK 2.2.0 gate are correct. SMP=1 and the style check only happen through build_uf2.sh, and the checker covers only some BARR-C rules. |
| NFR2 | PARTLY | Priority 5, latches, 2 ms ISR and 64-entry queue are correct. The "1 ms" is a relative delay (about 16 ms between logged samples), and the encoders count in their own GPIO ISR. |
| NFR3 | PARTLY | 50 ms and priority 6 are correct. The "2 ms mailbox wait" is a mutex timeout, and the "4 misses" are failed lock attempts, not deadline misses. Only in the live-control build. |
| NFR4 | PARTLY | 20 ms, priority 7, 200 ms and 1 s are correct. Alpha 0.2 is a plain smoothing filter on the accelerometer, not a complementary filter (there is no gyro). |
| NFR5 | MATCHES | 30 / 50 / 60 ms timing and the median of three are in the code. |
| NFR6 | PARTLY | Hysteresis 50 is right. The threshold is 2200; 600 is the capture-start rise. The one-sample ISR filter is compiled out by default. |
| NFR7 | PARTLY | No malloc in the firmware. The kernel allocates task stacks from its heap at boot (USE_IMALLOC=1), and lwIP has a 12 KB static heap. |
| NFR8 | MATCHES | Priority 9 for both tasks. 129,775 frames, p95 301 ms and 99.78 % are in docs/min-bits/telemetry_timing.txt. |
| NFR9 | PARTLY | Three stop paths print or send telemetry before stopping. Braking drives the pins high, not zero. |
| NFR10 | MATCHES | Encoder ISR priority 2, and the ISRs are minimal. |
| NFR11 | MATCHES (counts differ slightly) | Host suite passes with 142 PASS lines (not 140). Dashboard: 96 tests, 96 pass. |
| NFR12 | PARTLY | 69,598 B is a calculated subset of application structs, not a measured build. The 12 % compares against JSON re-encoding of the same frames. |
| NFR13 | PARTLY | 13-byte header + CRC-8 = 14 bytes, little-endian. Bad CRC and bad length are rejected. Reserved header-flag bits are not checked. |

## FR14: dead-reckoned route and markers (priority 1)

### What the firmware sends

ODOM payload: left Δmm (int16), right Δmm (int16) and source seq (uint16).
- Built in `modules/telemetry/telemetry_peers.c:51-83`. A frame is made each telemetry poll (every 50 ms) when either wheel moved at least 1 mm.
- Decoded in `app_program/dashboard/server/protocol.js:50`.

Distance checkpoints are never sent.
- `b1_motion_t.checkpoint` and `cumulative_mm` exist (`modules/telemetry/buddy1.h:21-25`), and the store would send a CHECKPOINT frame if the flag were set (`buddy1.c:76-77, 181-184`).
- But the only producer never sets either field (`telemetry_peers.c:61-80`).
- The dashboard's checkpoint check (`server/state.js:303-304`) therefore never runs on a real robot.

EVENT kinds actually raised:

| Kind | Meaning | Source |
|---|---|---|
| 1 | Barcode decoded | `telemetry_peers.c:116` |
| 4 | Obstacle confirmed | `telemetry_peers.c:169` |
| 5 | Bypass side chosen | `telemetry_peers.c:173` |
| 6 | Bypass count changed | `telemetry_peers.c:176` |
| 7 | Obstacle fault | `telemetry_peers.c:165` |
| 10 | Mission state changed | `buddy1_rt.c:221` |
| 12–21 | Navigation events | `app/nav_core.h:23-29`, raised via `task_controller.c:1127-1248` |

The navigation events are: 13 junction confirmed, 14 turn started, 15 turn completed, 16 line reacquired, 17 following, 18 fault, 19 scan rejected, 20 decode wait, 21 safe stop.

- **No hump event exists.** Terrain records always carry `event_kind = 0` (`telemetry_peers.c:141-143`: "do not invent one here").
- The dashboard defines names for kinds 2 (junction), 3 (hump detected), 8 (line lost) and 9 (hump finished) (`server/state.js:10`), but nothing emits them.
- Hump information only reaches the dashboard as:
  - the snapshot's max hump (`state.js:260`, a log line only), and
  - the serial `HUMP DONE` line (`state.js:66-68`).
- **Kind 6 is mislabelled.** The firmware means "bypass count changed", but the dashboard shows it as "Line reacquired" (`state.js:10`). The live log shows this.

### Dashboard pose integration (`server/state.js:263-271`)

- Uses the midpoint method: `mm=(L+R)/2`, `dθ=(R−L)/trackWidth`, and x/y are advanced along `θ+dθ/2`.
- **Track width mismatch.** The dashboard default is 120 mm (`server/index.js:43`, `state.js:28`). The firmware uses `APP_NAV_TRACK_MM 130` (`app/app_config.h:261`).
- With 120 mm, every turn is over-estimated by about 8 % (a 90° pivot draws as about 97.5°).
- Fix: set `TRACK_WIDTH_MM=130` in the environment, or change the default.
- `points` keeps only 2000 samples (`state.js:271`). At about 50 ms per ODOM that is roughly 100 s of drawn route; older route is dropped but its markers are kept.

### Markers on the map (`app_program/dashboard/public/app.js:366-368`)

- The route is drawn as a polyline. Uncertain segments are dashed amber.
- Every event with `positionValid` gets a 5 px circle at the pose of the last ODOM point at or before the event's time (`state.js:283-285`).
- Colours exist only for kinds 1–10 (`app.js:12`). Junction and turn events (13/14/15) fall back to `#314660`, the same colour as the route line, so they cannot be told apart. There are no per-kind symbols or legend.
- **`positionValid` is `lastMotionSeq!==null && !this.uncertain`, and `this.uncertain` is sticky.** Nothing clears it except a session reset (`state.js:39`). It is set by:
  1. Any wire-sequence or ODOM gap (`state.js:148, 264`).
  2. Any frame with flags bit 0 (`state.js:162`). In firmware that bit is `s->uncertain`, which is also never cleared (`buddy1.c:140`). It is set on any dropped frame (`buddy1_rt.c:191, 231`), any ring overflow (`buddy1.c:28, 65`) and any source seq jump.
  3. **A firmware bug sets it on the first mission event of every run.** `buddy1_rt.c:216/222` does `event_id=++mission_seq`, then `:225` does `seq=++mission_seq`. So any mission frame that carries an event moves `seq` forward by 2. `b1_ingest_mission` reads that as a gap and sets `uncertain=1` (`buddy1.c:108-109`). The first mission-state change after START does this.
- **Live evidence** (`/api/state`, robot driving, 1502 frames): `uncertain: true`, `dropped: 58`, `odomGaps: 1`, and every event, including "Obstacle detected" and "Bypass chosen", has `positionValid: false`.
- So **no markers are placed on the map in a real run.** Events appear only in the event list and log.

### Verdict per marker kind

| Marker | Placed at its pose on the map? |
|---|---|
| Barcode (kind 1) | Emitted and would be placed, but in practice hidden by the sticky uncertain flag. List/log only. |
| Junction / turn (13–15) | Emitted, but hidden for the same reason. Even when shown, it is the same colour as the route. List/log only. |
| Hump | **Not emitted at all.** It cannot appear as a marker. |
| Obstacle (4, 5, 6, 7) | Emitted, but hidden for the same reason. Kind 6 is mislabelled. List/log only. |

### Tests

- `dashboard/tests/route-map-live.test.js:31-38` only checks that barcode events without odometry are `positionValid:false`.
- No test checks that a marker is placed at the right pose, that markers stay valid after a flagged frame, that mission event seq stays contiguous, or that the track width matches the firmware.

### To make FR14 true

1. Firmware: use a separate counter for `mission.event_id` so `seq` stays contiguous (`buddy1_rt.c:216, 222`).
2. Firmware: set `checkpoint`/`cumulative_mm` every N mm, or remove "distance checkpoints" from the requirement.
3. Firmware: emit hump start and done events from terrain (kinds 3/9) with an event ID.
4. Dashboard: set the track width to 130 mm.
5. Dashboard: keep drawing markers with an "approximate" style instead of hiding them when uncertain, or clear `uncertain` after a CHECKPOINT agrees.
6. Dashboard: give each kind its own colour/shape and a legend, and fix the kind-6 label.
7. Add tests for all of the above.

### Uncommitted `server/state.js` change

It adds 11 lines: when the log shows `[OBS] Way ahead clear again`, the last real obstacle width is restored. This only affects the obstacle-width display and is unrelated to markers.

## FR15: MQTT loss (priority 2)

### Branch (`app/task_controller.c:643-675`)

- When `!b1_mqtt_connected()` in LINE_FOLLOWING, OBSTACLE, IDLE, AT_JUNCTION or LIFTED, it:
  - sets `stop_request` and calls `motion_stop_now()` (:655),
  - clears the decoded letter (`g7_nav_clear_decoded`),
  - resets the history,
  - calls `obstacle_abort()` (bypass abort) and `nav_forget()` (clears the queued action),
  - then sets state = REMOTE_HOLD (:669).
- Turn, centre and crossing are navigation sub-states inside LINE_FOLLOWING/AT_JUNCTION, so they are covered.
- STOPPED, NAV_SAFETY_STOP, ERROR and REMOTE_HOLD are not listed, but they are already motionless and stay where they are.
- Bypass driving happens in OBSTACLE, so it is covered.

### Timing

- The check runs at the top of each 50 ms controller cycle. Once the flag reads 0, the stop happens within one period (plus relative-delay jitter). That part MATCHES.
- **But NFR9 is broken here:** `tm_printf("[CTRL] MQTT lost …")` (:649-650) runs before `motion_stop_now()`.

### How fast the link is reported down

The flag `mqtt_link_ready` is cleared in `discard()`/`tcp_error` (`lib/libnet/lwip/mqtt_utk.c:155-169`). That happens on:

- Wi-Fi netif link down (`mqtt_utk.c:319-320`, polled every 2 ms by the CYW43 service, `cyw43_utk.c:42, 637`). How fast this fires depends on the CYW43 firmware noticing deauth or lost beacons, typically seconds. It cannot be verified from the code.
- A TCP error or RST from lwIP (`tcp_error`).
- No CONNACK within 10 s (:333-334).
- **No bytes received for 45 s** (:335), with a PINGREQ every 15 s (:341). Keepalive is 30 s (:120).

So for a silent loss (hotspot dies, out of range without a deauth, broker host asleep), the robot can keep driving for **up to 45 s** before the "first 50 ms period" even starts. The requirement should say "within 50 ms of the link being reported down; link-down is reported within ≤45 s (no-RX timeout) or on Wi-Fi link loss", or the 45 s should be shortened.

### Reconnect

- START needs REMOTE_HOLD, a fresh start request and `b1_mqtt_connected()` (:676-694). It also needs a STATUS challenge from this boot, no more than 8 s old (`buddy1_rt.c:170-188`).
- Reconnecting never changes state. MATCHES.

### Offline buffering

- The rings are 32 ODOM / 24 EVENT (`buddy1.h:9-10`). Overflow overwrites the oldest entry and counts it as dropped (`buddy1.c:25-29, 62-66`).
- **But they are not held while offline.** `buddy1_rt.c:228-233` takes up to 6 frames per 50 ms and `b1_mqtt_submit` queues them into the 8-slot tx queue without checking the link (`mqtt_utk.c:38-48, 8`).
- Once that queue is full, each submit fails and the frame is **dropped and counted** (`store.dropped++`, `tx_drops++`). The rings empty within a fraction of a second.
- So about 7 frames survive an outage, not 32 + 24. Drops are counted and sent in the HEARTBEAT (`buddy1.c:187`).
- Live, `dropped` was already 58 while connected, so the tx queue overflows in normal driving too.

### Tests

- `tools/test_controller_handoff.c:302-305` (link down while running → REMOTE_HOLD, zero output, no resume), `:644-646` and `:748-751` (START refused while offline).
- `dashboard/tests/reconnect.test.js` and `remote-hold.test.js` cover the dashboard side.
- No test covers the 45 s detection, offline ring retention, or print-before-stop.

## Other FR details

### FR4
- `modules/motion/steering.h:37-38` sets `STEERING_OBSERVER 1`, which gives PID with KP 0.06, KD 0.5, KI 0.0002 (`steering.c:72-74`).
- The PI path (KP 0.3, KI 2.0, `steering.c:14-15`) is inactive.
- Values that match: 18 mm (`steering.c:57`), 30 RPM (`motion.c:45`), pivot after 100 mm (`motion.c:130, 819`).
- Extra stage: after 50 mm without the line, it keeps driving and steers gently towards the line (`motion.c:142`).

### FR6
- Values that match: 7 mm (`app_config.h:185`), 120 counts (:176), 125 + 9 = 134 mm (:249-251), 8000 ms (:241).
- `APP_NAV_DECODE_GRACE_MSEC 1500` (:242) ends the wait 1.5 s after the decoder goes idle (`nav_core.c:213-218`).

### FR8
- Uses strict > 3° ×3 and < 1.5° ×15 (`modules/terrain/task_imu.c:42-51, 590-603, 677-686`).
- Height = max(110·sin(roll), Σds·sin(tilt)) (`task_imu.c:93-98, 712-716, 767-768`).
- 40 mm minimum travel and 127 mm clamp match. Humps within 3 s of a lift are ignored (:117).

### FR9
- `modules/obstacle/task_obstacle.c:257` tests `median < OBS_STOP_MM` (250).
- It needs 3 consecutive medians of a sliding 3-ping window, so at least 5 pings.

### FR10
- `servo.c:118` uses `LEFT_SIGN 1`, so positive angles turn left. The limits are left 60°, right 70° (`obs_config.h:73-74`).
- Step size is 5° within ±30 and in the corner band out to ±50, and 10° beyond (:103-105, 156, 163).
- The other values match:
  - Edge vote 20–45° with 5 pings (:213-215).
  - Margin 40 mm (:324).
  - Minimum 245 mm (:294).
  - Tie 15 mm (:412).
  - 3 echoes beyond 400 mm (:518-519).

### FR12
- STATUS every 1000 ms (`buddy1.c:144`).
- SNAPSHOT `>=250` (`buddy1.c:188`, sent from a 50 ms loop, so about 250–300 ms).
- HEARTBEAT every 2000 ms (:184).
- SENSOR every 1000 ms (`telemetry_peers.c:194`).
- ODOM goes out on each 50 ms poll when the wheels moved. The only 60 ms value is the obstacle ping period.

### FR13
- RECAL_IMU returns `B1_NOT_READY` ("No live IMU re-baselining API", `task_controller.c:478-479`).
- The other commands, the ACK result codes 0–7 and the STALE (7) challenge all match.

### FR16
- Fault codes 2–14 all zero the output.
- `remote_reset_after_stop` (`task_controller.c:494`) keeps ERROR. ERROR is entered for IR read or clock errors, obstacle sensor faults, bypass faults and NO_ROUTE, and stays until reboot.
- Lift resumes on its own (`APP_LIFT_RESUME_ENABLED 1`).

### FR18
- With sign 0, pitch never engages the governor (`downhill.c:85`).
- `motion_observe_terrain()` (`motion.c:986-995`) engages it whenever the hump phase is DESCENDING.
- While engaged it ramps the RPM targets down to 65 % (`downhill.c:128-141`).
- The 40 RPM runaway check zeroes the fast wheel. After 10 periods it latches fault 9 (`downhill.c:174-182`).
- A stale IMU for 20 periods latches fault 11.

## NFR details

### NFR1
- Kernel 3.0: `include/sys/profile.h:45-46`. The SDK 2.2.0 gate stops the build with exit 4 (`build_uf2.sh:7-22`).
- SMP=1 is only set by `build_uf2.sh:127`; the makefile default is `SMP ?= 0`.
- The checker (`tools/check_app_standard.py`, run at `build_uf2.sh:82`) says itself that it is "not a complete BARR-C compliance tool".
- Observed on today's build: the first try with `~/pico/pico-sdk` (2.3.0) was rejected, as designed.

### NFR2
- `tk_dly_tsk(1)` at `task_ir.c:87`. `app_config.h:192-193` notes "~3 samples per 50 ms".
- The 2 ms alarm ISR is at `bc_fast.c:117-163`. The encoder GPIO ISR is at `encoder.c:145-175`. `BC_FASTQ_CAPACITY 64` is at `bc_fastq.h:10`.

### NFR3
- 2 ms wait: `tk_loc_mtx(g_remote_mutex, 2)` (`task_controller.c:516`). 4 failed attempts (`app_config.h:30`) lead to a stop (`task_controller.c:518-531`).
- The loop uses a relative 50 ms delay.

### NFR4
- `IMU_FILTER_ALPHA 0.2F`, applied as `f += 0.2*(raw−f)` (`task_imu.c:32, 553`). There is no gyro (:39).
- 200 ms (`task_controller.c:722`) only gates use of the data. The 1 s stop (:745-752) applies in the following and obstacle states.

### NFR6
- `APP_BARCODE_ADC_THRESHOLD 2200`, `HYSTERESIS 50`, `TRIGGER_RISE 600` (`app_config.h:50-51, 84`).
- The ISR filter is off in buffered mode (`bc_fast.c:38-39`, `app_config.h:60`).

### NFR7
- `config/config.h:107` sets `USE_IMALLOC (1)`, and task stacks come from `knl_Imalloc` at boot.
- lwIP: `MEM_SIZE` 12 KB static plus static pools.

### NFR8
- Source: `docs/min-bits/telemetry_timing.txt:8-10`, produced by `tools/telemetry_timing.js`.
- It covers 672 recordings. Gaps of 5 s or more are excluded.
- It measures how regularly frames are sent, not end-to-end latency, and the recordings are not tied to a firmware version.

### NFR9
- These print or send before stopping:
  - MQTT lost (`task_controller.c:649-655`).
  - Mutex misses (:526-528 → :655).
  - Navigation fault: `nav_report` and prints (:1246-1262) run before `motion_brake` (:1295).
- `motion_brake`/`downhill_fault_stop` drive the pins high (`motor.c:83-97`), not to zero.

### NFR11
I ran both suites today:
- `bash tools/run_host_tests.sh` ends with `HOST PASS` (exit 0). The log has 142 PASS lines, and the coding-standard check reports 117 files with 0 errors.
- `npm test`: 96 tests, 96 pass, 0 fail.
- The test files shown as deleted in git status were renamed copies; the current files run.

### NFR12
- The numbers are in `docs/min-bits/MINIMUM_BITS.md:11, 123` and `mqtt_message_sizes.csv:15` (11.7 %).
- 69,598 is struct size × number of static instances, from debug info. About 65,536 B of it is `g_rows` (8192 × 8 B), and most of the 33,996 B saving is the shrink of those rows. No script in the repo produces 69,598 or 103,594.
- Today's fresh build:
  - text 444,837, data 2,560, bss 208,468. The bss matches the doc.
  - The text is 2,832 B larger than the documented 442,005, so the evidence predates the latest code.
- The doc says the kernel heap is inside .bss; it actually starts at `__bss_end`.

### NFR13
- `B1_HEADER 13` plus a 1-byte CRC-8 (polynomial 0x07). Little-endian.
- Length and CRC are rejected on both sides (`buddy1_protocol.c:45-48`, `protocol.js:20-21`).
- The header flags byte's reserved bits are not checked (`buddy1_protocol.c:50`, `protocol.js:22`).

## Tribunal 1 comparison

The Tribunal 1 text is **not in the repo or git history.** Commit 5ebe83e removed READMEs, changelogs and docs, but `git log --all -S "Tribunal"` and `-S "FR15"` find nothing.

The Week 8 Tribunal 1 PDFs are in `~/Downloads`: `Week8_Tribunal1_Template-2/-3/-4.pdf`, Versions A, B and C. They do not contain the full Week 6 FR/NFR list. They only refer to it:

- **NFR3 (Week 6 "5–10 ms nominal control loop"):** in Versions A and B, and risk R1 / test T1.
  - Now 50 ms, and supported by the code (`app_config.h:31`).
  - But the justification needs evidence. T1 asked for "worst-case loop time and jitter before/after packing under network load". The repo has no controller timing trace, only telemetry intervals (NFR8).
  - The 50 ms loop is consistent with ODOM and timing being every 50 ms. Show the reason (Wi-Fi load and the IR/encoder work moved to 1 ms / 2 ms ISRs) and measured deadline misses.
- **NFR4 (Week 6 "nominal 100 Hz" IMU):** Version A.
  - Now 20 ms (50 Hz), matching `task_imu.c:17`.
  - Justified by the filter design (alpha 0.2), but no measurement is in the repo.
- **IR "2–5 ms sampling" (Week 6 NFR2):** now a 1 ms task plus a 2 ms ISR, so faster than before. The code supports this (with the NFR2 caveat on the real interval).
- **FR13 commands (Week 6 "SET MAPPING, START, STOP/ABORT, SNAPSHOT"):** now also RECAL_IMU and RECAL_BARCODE. RECAL_IMU is not implemented (always NOT_READY).
- **FR15 (Week 6 MQTT handling, risk R5 / T6):** Tribunal 1 asked for "fixed capacity, overflow/drop counter, event priority, timestamps, session/sequence, fresh snapshot on reconnect".
  - Counters, priority, timestamps, session/seq and a stale-START guard exist.
  - The offline retention does not work: the rings drain into an 8-slot queue and are dropped (see FR15).
- **FR16/NFR9 (Week 6 safety):** Tribunal 1 asked for "STOP/fault must not depend on MQTT", and that holds. "PWM=0 forced stop" is only partly true (braking drives pins high, and some paths print first).
- **FR17 lift / FR18 downhill (new):** no Tribunal 1 counterpart. FR17 is supported by the code. FR18 must be described as "active on the hump descending phase; the pitch trigger is off until the sign is calibrated".
- **NFR12 (new, the minimum-bit change requirement from Tribunal 1 Appendix A1):**
  - Supported by docs/min-bits.
  - Present the RAM figure as "application record storage (calculated)", not "static RAM".
  - Present 12 % as "binary vs equivalent JSON encoding of the same frames".

## Changes made after this audit (23 Nov, not committed)

Rollback points:
- `git stash create` before any change: `354dc2ae97d50f9aeff0b6affec5cad0759bdf4e`. It does not include the untracked dashboard folder.
- Patches and tarballs are in `~/Downloads/g7_before_fr14/`:
  - `g7_before_fr14.patch` with `g7_before_fr14_untracked.tgz` (the dashboard).
  - `g7_before_sidefix.patch`.
  - `g7_before_rejoin.patch` with `dashboard_after_fr14.tgz`.

**FR14 (route map).**
- Firmware:
  - Mission EVENT IDs now have their own counter (`buddy1_rt.c`).
  - A crossed hump becomes terrain EVENT kind 9 with its peak height in mm (`telemetry_peers.c`).
  - An ODOM checkpoint goes out every 500 mm of travel, carrying the cumulative distance.
- Dashboard (`server/route.js`, used by `state.js`):
  - Track width is 130 mm.
  - Events are placed by robot time.
  - Only ODOM gaps make a segment approximate, and checkpoints bridge them.
  - Heading snaps to ±90°/180° at junction turns, and a U-turn retraces the outbound line.
  - Each marker kind has its own colour and shape, and kind 6 is now labelled "Obstacle bypassed".
- Tests: `tests/route-tracker.test.js` (11) and `tools/tests/test_telemetry_map.c`.

**FR15 (MQTT loss).**
- `mqtt_utk.c`: a ping every 5 s, and the link counts as down after 15 s of silence (it was 15 s / 45 s). The longest uplink stall on the hotspot this morning was 4.3 s.
- Telemetry stays in the 32/24 rings while offline or while the queue is full (`b1_mqtt_can_send`).
- The controller's print-before-stop order is unchanged, because the controller is out of scope.

**Rejoin (obstacle return).**
- `OBS_RETURN_TO_BLACK 1` in `obs_config.h`. Setting it to 0 restores the old return.
- Behaviour:
  - On the way back, any black under a line sensor counts as the line.
  - The robot keeps heading in until it sees the line, bounded by `OBS_RETURN_SEARCH_MM`.
  - Straightening up stops once the line is between the sensors, or turns up to `OBS_ALIGN_MORE_DEG` further to find it.
- Tested in simulation only: `tools/tests/obstacle_rejoin_sim.c`.
