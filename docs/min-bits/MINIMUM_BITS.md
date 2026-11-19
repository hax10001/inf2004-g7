# Change Requirement 1: Minimum-Bit Data Representation

Evidence for the Week 13 submission. Every number below is measured from
this repository's firmware (RP2040, `arm-none-eabi-gcc` 15.3.1, Pico SDK
2.2.0, SMP, USB console, live MQTT) or from real robot MQTT traffic recorded
by the dashboard. The commands to regenerate each table are at the end.

| Deliverable | Result | Detail |
| --- | --- | --- |
| 1. Data representation table | 616 record fields and 265 static instances inventoried from the compiled ARM sources | [Section 1](#1-data-representation-table), [`application_fields.csv`](application_fields.csv), [`application_globals.csv`](application_globals.csv) |
| 2. RAM usage comparison | **33,980 B less static RAM** (103,578 → 69,598 B across application instances); 613 B less per instance set of the 66 records | [Section 2](#2-ram-usage-comparison), [`record_sizes.csv`](record_sizes.csv) |
| 3. Message size comparison | MQTT binary frames are **12 %** of the same data as text (3.7 MB vs 31.9 MB over 128,592 real frames); queue records 9–10 B | [Section 3](#3-message-size-comparison), [`mqtt_message_sizes.csv`](mqtt_message_sizes.csv) |
| 4. Correctness and performance | Full host suite passes (140 checks incl. bit-boundary tests), 96 dashboard tests, telemetry cadence unchanged | [Section 4](#4-correctness-and-performance), [`telemetry_timing.txt`](telemetry_timing.txt) |

## 1. Data representation table

Rules applied throughout: a value gets the bits of its valid range (C
bit-fields, or explicit shifts and masks on the wire); fault conditions are
single bits in one mask; values a task shares are packed only behind the
lock that owns them, or kept as a single naturally aligned object when two
tasks touch them without a lock.

### Key fields by area

| Area | Field | Valid range | Bits | Representation |
| --- | --- | --- | --- | --- |
| **1. Motion / encoders** | ODOM queue record (time, left/right travel, sequence) | ms clock; ±32,767 mm; modulo 65,536 | 32+16+16+16 | `B1_OdomRecord`: 10-byte array, explicit little-endian packing (no padding, no unaligned access) |
| | Downhill governor percent / engaged / fault / pitch | 0 or 65..100 / 0..1 / 0..2 / −900..900 deci-deg | 7 / 1 / 2 / 11 | `motion_status_t` bit-fields under the motion mutex |
| | Lost-line search phase / last line side | 0..4 / −1..1 | 3 / 2 | `line_search_t` bit-fields, controller task only |
| | Travel since line last seen / pivot swept | 0..65,535 counts (saturating) | 16 / 16 | `uint16_t`, saturating add |
| | Wheel counts per 50 ms period | about ±60 | 16 | `int16_t` (`g_period_counts`, `g_nav_fwd_*`) |
| | Pivot PWM, brake and log countdowns | 750..3875 / 0..4 / 0..10 | 16 / 8 / 8 | Narrow integers with compile-time range checks |
| | Encoder lifetime count | full range | 32 | Kept: wraps arithmetically; ISR writes one aligned word |
| **2. Line / barcode** | IR left / right / barcode / valid | 0..1 each | 1 each | `ir_shared_snapshot_t` bit-fields under the IR mutex |
| | IR event latches | 6 events | 6 | Independent latch bits, mutex-protected take/clear |
| | Barcode waveform row (ADC, step, gap) | 0..4095 / 0..15 / < 65 ms | 12 / 4 / 16 | `bc_wave_row_t`: 8 bytes instead of 12, 8,192 rows |
| | Decoded letter | 0..25, 31 = invalid | 5 | `scan_handoff_t.letter:5` behind the port semaphore |
| | Scan ready / busy / enabled / owned / overflow | 0..1 each | 1 each | `scan_handoff_t` bit-fields, one semaphore for every read-modify-write |
| | Scan cancel / drop request | 0..1 | 1 logical bit in 1 byte | Separate `volatile uint8_t` per flag: a single byte store is atomic across the controller and scanner tasks, so no packed read-modify-write is shared |
| **3. Obstacle** | Scan point (bearing, range, lone echo) | −70..70°, 0..4095 mm, 0..4095 mm | 8 / 12 / 12 | `scan_plan_point_t`: 4 bytes instead of 12, 39 per scan |
| | Bypass plan (result, side, angle, distances in cm) | 0..2, 0..1, 0..3, 0..510 cm | 2 / 1 / 2 / 9 each | `scan_plan_bypass_t`: two 32-bit words, shifts and masks |
| | Obstacle fault conditions | no echo, no route, line lost, servo, timeout | 1 each | `OBS_FAULT_*` bits in one 5-bit mask |
| | Sequencer phase / side / hit and miss counts / seek legs | 0..13 / −1..1 / 0..5 / 0..4 | 4 / 2 / 2–3 / 3 | `obstacle_seq_t`: 27 flag and state fields in 49 bits (two 32-bit words) |
| | Turn angles / servo angles / offset | ±1800 deci-deg / ±90° / < 5.1 m | 16 / 8 / 16 | `int16_t` / `int8_t` / `int16_t` with explicit clamps |
| | Side-watch, guard, line, verify counters | compared to limits ≤ 10 | 8 | `uint8_t`, saturating increment |
| | Front monitor index / fill / hits / no-echo | 0..2 / 0..3 / 0..3 / 0..5 | 2 / 2 / 2 / 3 | `monitor_t` bit-fields with `#error` range guards |
| | Heading off the line (shared, controller → scanner) | −250..250 deci-deg, sentinel unknown | 10 (in 16) | One aligned `int16_t`: a single store and load, no lock needed |
| **4. Hump estimation** | Current / last / maximum hump height | 0..127 mm (saturating) | 7 each | `terrain_snapshot` bit-fields under the terrain mutex |
| | Lift height / lifted flag | 0..1022 mm (1023 = unknown) / 0..1 | 10 / 1 | Same 32-bit word as the hump heights |
| | Hump phase / level count / tilt count / filter ready / on hump / lifted during hump / travel during hump | 0..2 / 0..15 / 0..3 / 0..1 ×3 / 0..63 mm | 2 / 4 / 2 / 1 ×3 / 6 | `hump_state_t` bit-fields, IMU task only; gravity in 11 bits |
| | Lift estimator start / still / floor counts | 0..3 / 0..25 / 0..5 | 2 / 5 / 3 | `lift_math_state_t` bit-fields |
| **5. Vehicle / controller** | Navigation state / action / letter | 9 states / 0..3 / 0..25 | 4 / 2 / 5 | `nav_controller_t` bit-fields (17 fields) |
| | Junction flags (pending, hold, tape capture, capture roll, ...) | 0..1 each | 1 each | `nav_controller_t` bit-fields |
| | Staged event / value / sequence / fault / 3 flags | 0..21 / 0..14 / modulo 65,536 / 0..14 | 5 / 4 / 16 / 4 / 1 ×3 | `g_control`: exactly one 32-bit word |
| | Controller history flags | 0..1 | 1 each | `controller_history_t` `bool x:1` |
| | Navigation result (drive, event, value) | 0..5 / 0..21 / 0..5 | 3 / 5 / 3 | `nav_result_t`: 2 bytes |
| **6. ISR / task shared** | Encoder count and edge time | 32-bit counters | 32 | ISR writes whole aligned words, readers copy them; no packed read-modify-write |
| | Barcode fast-queue sample | ADC 12 bits + flags | packed | Single-producer single-consumer ring between the ADC ISR and the barcode task |
| | Terrain / motion / IR snapshots | as above | as above | Packed fields only written and read with the owning mutex held |
| **7. Inter-task queues** | Telemetry event record | time32, id16, value16, source 3, kind 5 | 72 | `B1_Event`: 9 bytes instead of 12, 24-entry ring |
| | Odometry record | see area 1 | 80 | `B1_OdomRecord`: 10 bytes instead of 20, 32-entry ring |
| | Barcode sample queue | as above | packed | `bc_fastq_sample_t`: 24 bytes, 64 entries |
| **8. MQTT** | Snapshot payload | mission 3, line 3, IR 3, letter 5, action 2+1, hump 10, max hump 10, obstacle 10, side 2, bypass 8, faults 8, wheel speeds 12+12, source and stale masks 5+5 | 99 bits | 13 bytes, explicit bit writer (no C structs on the wire) |
| | Route mapping command | four letters 0..25 | 5 each = 20 | 3 bytes |
| | Sensor frame lift word | height 10 + lifted 1 + 5 reserved | 16 | Appended field; the decoder rejects nonzero reserved bits |
| | Frame header | type, flags, session, sequence, ms, length, CRC-8 | 14 bytes | Fixed binary envelope |

### Full inventory

[`application_fields.csv`](application_fields.csv) lists every member of
every application record the ARM build compiles (616 fields): declared type,
stored bits, representable range, known valid domain, minimum bits, owner or
lock, and status. [`application_globals.csv`](application_globals.csv) lists
the 265 global and static instances. The inventory is generated by parsing
the real ARM compile commands with Clang, so it matches what is built.
[`prior_field_review.csv`](prior_field_review.csv) is the team's earlier
field-by-field range review, which the generator carries forward.

Kept at full width, on purpose:

* **Clocks and lifetime counters** (millisecond timestamps, encoder totals):
  they wrap and are compared by subtraction; narrowing them would break that
  arithmetic.
* **Floating-point control state** (PI integrators, pose, heading average):
  calculation precision for closed-loop control, not stored application
  values. Narrowing needs an error budget, and the control behaviour is
  validated by the closed-loop simulator in section 4.
* **Function-local working variables** in the scan planner: they live for
  one call and are packed into `scan_plan_bypass_t` before being stored or
  shared.

## 2. RAM usage comparison

### Record sizes, packed vs unpacked (linked ARM firmware)

[`record_sizes.csv`](record_sizes.csv) reads the real record sizes from the
firmware's debug information and compares each with the same members laid
out without bit-fields (each field at its declared type, natural alignment).

| Record | Unpacked (B) | Packed (B) | Saved (B) |
| --- | ---: | ---: | ---: |
| `obstacle_seq_t` (bypass sequencer) | 212 | 112 | 100 |
| `nav_controller_t` (navigation) | 168 | 104 | 64 |
| `scan_plan_sides_t` (scan report) | 88 | 40 | 48 |
| `hump_state_t` | 72 | 36 | 36 |
| `downhill_governor_t` | 36 | 8 | 28 |
| `B1_Store` (telemetry store) | 764 | 736 | 28 |
| `terrain_snapshot` | 40 | 20 | 20 |
| `line_search_t` | 36 | 16 | 20 |
| `scan_handoff_t` | 24 | 12 | 12 |
| `bc_wave_row_t` (× 8,192) | 12 | 8 | 4 per row |
| `scan_plan_point_t` (× 39 per scan) | 12 | 4 | 8 per point |
| **All 66 application records** | **2,994** | **2,381** | **613** |

### Static RAM by instance

Weighting each record by the static instances the firmware holds (arrays
counted in full): **103,578 B unpacked → 69,598 B packed, 33,980 B saved**.
The barcode waveform buffer alone saves 32,768 B (8,192 rows × 4 B).

### Linked image

| Build | `.text` | `.data` | `.bss` |
| --- | ---: | ---: | ---: |
| Current firmware, before the final packing pass | 441,269 | 2,560 | 208,508 |
| Current firmware, after | 442,005 | 2,560 | 208,468 |

The final pass (sequencer counters and angles, front monitor, side watch,
controller history, hump flags, shared heading) saves 40 B of static RAM
for 736 B of code (clamps and saturating counters). Earlier team
measurements of the main packing work, kept for the record: telemetry
store 1,152 → 760 B, odometry ring 640 → 320 B, event ring 288 → 216 B,
scan point array 72 → 36 B (V10 → V11, host compiler); V14 → V15 linked ARM
`.bss` 208,240 → 208,264 B (+24 B for a new barcode hand-off). Static
sections include the kernel's reserved heap; task stack high-water marks
are not part of an ELF measurement.

## 3. Message size comparison

### MQTT: binary vs the same values as text

[`mqtt_message_sizes.csv`](mqtt_message_sizes.csv) replays **128,592 real
frames** from the robot's recorded MQTT traffic. Each frame is decoded with
the dashboard's own decoder, and the decoded values (with the header's type,
session, sequence and time) are written as JSON text for comparison.

| Message | Frames | Binary payload (B) | Binary frame (B) | Same data as JSON text (B) | Binary / text |
| --- | ---: | ---: | ---: | ---: | ---: |
| SNAPSHOT | 46,818 | 13 | 27 | 272 | 10 % |
| ODOM | 32,655 | 6 | 20 | 112 | 18 % |
| STATUS | 13,607 | 14 | 28 | 270 | 10 % |
| BARCODE_DIAG | 13,548 | 42 | 56 | 456 | 12 % |
| SENSOR (with lift word) | 8,145 | 26 | 40 | 355 | 11 % |
| SENSOR | 5,420 | 24 | 38 | 356 | 11 % |
| HEARTBEAT | 6,819 | 4 | 18 | 92 | 20 % |
| EVENT | 954 | 6 | 20 | 120 | 17 % |
| ACK | 431 | 4 | 18 | 101 | 18 % |
| **All frames** | **128,592** | | **3,742,306 total** | **31,891,619 total** | **12 %** |

Commands from the dashboard: route mapping 6 B payload (four letters in 20
bits), START / STOP / other 3 B, each in the same 14-byte envelope.

### Inter-task records

| Record | Unpacked (B) | Packed (B) | Entries |
| --- | ---: | ---: | ---: |
| Odometry (`B1_OdomRecord`) | 20 | 10 | 32 |
| Event (`B1_Event`) | 12 | 9 | 24 |
| Scan point (`scan_plan_point_t`) | 12 | 4 | 39 per scan |
| Barcode hand-off (`scan_handoff_t`) | 24 | 12 | 1 |

## 4. Correctness and performance

**Functional tests** (`bash tools/run_host_tests.sh`, exit code 0, 140 checks):

* Bit-boundary and packing tests carried from V9 to V11: every packed field
  at its minimum and maximum, saturation, sentinels, invalid and reserved
  values rejected, and C and JavaScript protocol golden vectors agreeing.
* Shared-data safety: IR latch take/clear with no lost events, barcode port
  semaphore with no leaks, scan hand-off lifecycle and overflow.
* Behaviour on top of the packed state: navigation (223 assertions,
  including junction and barcode timing), obstacle planner and bypass,
  closed-loop line following at 35 and 55 RPM (all curve and corner
  courses), downhill governor and motion plant.

**Dashboard** (`npm test` in `app_program/dashboard`): 96 tests, including
the MQTT decoders for every packed frame and the four-letter mapping in
20 bits.

**Timing** ([`telemetry_timing.txt`](telemetry_timing.txt)), measured by the
robot's own millisecond clock over 129,775 recorded frames:

| Stream | Median interval | 95th percentile |
| --- | ---: | ---: |
| SNAPSHOT | 300 ms | 301 ms |
| ODOM | 60 ms | 69 ms |
| SENSOR / STATUS | 1,020 ms | 1,040 ms |

Frame sequence continuity is 99.78 % (283 gaps in 129,619 consecutive
pairs). The packed representation adds a
few shifts and masks per field. It costs no measurable slowdown: the 50 ms
control period and every telemetry rate hold their schedule. Hardware
demonstration on the course remains the final check.

## Regenerating the evidence

```sh
# 1. Build the firmware and keep the log (all compile commands).
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 ./build_uf2.sh > build.log 2>&1
# 2. Field and instance inventory (needs Clang and the ARM C headers).
python3 tools/audit_fields.py --build-log build.log \
  --arm-include "$(dirname "$(arm-none-eabi-gcc -print-libgcc-file-name)")/../../../../arm-none-eabi/include"
# 3. Packed vs unpacked record sizes from the linked ELF.
python3 tools/record_sizes.py build_make/mtk3pico_smp1_usb_cdc_wifi_dhcp.elf \
  docs/min-bits/application_fields.csv docs/min-bits/record_sizes.csv
# 4. MQTT sizes and telemetry timing from dashboard recordings.
node tools/message_sizes.js > docs/min-bits/mqtt_message_sizes.csv
node tools/telemetry_timing.js > docs/min-bits/telemetry_timing.txt
# 5. Tests.
bash tools/run_host_tests.sh && (cd app_program/dashboard && npm test)
```
