# INF2004 G7 — V11 precision and minimum-bit review

**10 November 2026. Source:** V11 minimum-bit checkpoint, reviewed against the application C implementation.  
**Scope:** 15 previously flagged `PRECISION_REVIEW_PENDING` entries, 19 `NOT_FINALLY_VERIFIED` entries, and the three wire-compatibility decisions.  
**Status:** **Code-level review performed; not final Week 13 acceptance.** No Pico W build, physical calibration, or hardware timing measurement was available in this environment.

## What was actually changed in this preservation-first precision patch

1. `steering_t.last_side` and `steering_t.bend_side` went from `float` to signed **2-bit** internal fields. All producers assign exactly −1, 0 or +1; these are lossless, exact values. Host tests compared **36,000 deterministic steering updates against unchanged V11**, including outputs and steering state, with no differences. `sizeof(steering_t)` went from **32 to 28 bytes** on native x86_64 GCC.
2. `bc_match_result_t.support` went from a byte to **3 bits**. The decoder has exactly six threshold levels (`BC_MATCH_LEVELS=6`), with at most one qualifying vote per level.
3. `bc_layout_result_t.bars` went from a byte to **7 bits**. The raw-bar counter is capped at 96, so 0–96 fits 7 bits.
4. `bc_layout_result_t.module_x10` went from 16 bits to **10 bits**. It is assigned on the accepted-fit path only, with `30 ≤ a ≤ 100` and stored as `(uint16_t)(a × 10)`, giving 300–1000; 0 is its default when a fit is unavailable.
5. `terrain_calibration.b_valid` is now a **1-bit** flag under the existing calibration mutex, rather than a whole byte-sized boolean.

All these edits affect **internal, task-owned or mutex-protected records only**. They do not alter MQTT frame version, command/ACK interpretation, motor update equations, barcode decision thresholds, ISR interfaces, or dashboard rendering. Structure layouts are implementation-defined: do not serialize native C bit-field structs directly.

**Native host size checks:** `bc_match_result_t` remained 16 B and `bc_layout_result_t` remained 12 B because of padding/alignment; representing a field with fewer bits does **not guarantee** a smaller struct. Native structure sizes are not ARM RAM evidence.

## 15 precision-review entries — field-level decisions

| Fields (source) | Proven behavior/range | Minimum-bit judgement | Decision and evidence still required |
|---|---|---|---|
| `motion_status_t.left_rpm`, `right_rpm` (`motion.c`, `wheel_speed.c`) | Non-negative float from encoder edge/time division; no enforced physical maximum RPM | **Not provable** without max RPM and required resolution | **Keep float32.** Measure real RPM extrema; verify effects of any fixed-point quantization on PI feedback and stall detection |
| `motion_status_t.left_target_rpm`, `right_target_rpm` (`motion.c`) | Lower bounded at zero by `motion_drive`, but the public call has no explicit upper clamp | **Not provable** from code alone | **Keep float32.** Define/validate an upper bound before considering compact storage |
| `motion_status_t.steer` (`motion.c`) | Controller steering constrained to approximately **−0.8..+0.8** by `steering_update` / `clamp_steer` | A *candidate* signed Q10 (1/1024 units) fits within 11 bits; max rounding error ≈0.000488 | **Retain float32 for now.** Need test against exact steering/timing thresholds, and demonstrate accepted quantitative error |
| `motion_status_t.speed_mm_per_sec` (`motion.c`) | `(left_rpm + right_rpm)/2 × mm_per_rev/60`; depends on encoder calibration | **Not provable** without bounded speed and calibration | **Keep float32.** Document valid speed range and intended output resolution first |
| `steering_t.integral` (`steering.c`) | Clamped to **−0.8..+0.8**, decays by 0.85 and influences 0.35/0.8 decision thresholds | Candidate signed Q10 / 11-bit storage; precision adequacy **not proven** | **Retain float32.** Recurrent rounding could alter the branch at 0.35; a bit-width alone is not sufficient |
| `steering_t.last_side`, `bend_side` (`steering.c`) | Only −1, 0 and +1 assigned | **2 signed bits each** | **Implemented and losslessly regression-tested** |
| `wheel_speed_t.rpm` (`wheel_speed.c`) | Encoder-derived nonnegative RPM. Edge interval and count increments are not physically bounded by the API | **Not provable** | **Keep float32** until encoder event limits and precision tolerance are established |
| `G7_TelemetryPeers.left_residual_mm`, `right_residual_mm` (`telemetry_peers.c`) | Fractional mm carried between odometry packets; normally less than 1 mm after successful un-clamped extraction, but extreme delta/clamping can leave a larger remainder | **Not provable** across all valid input paths | **Keep float32** until inputs are bounded; fixed point must preserve long-run cumulative distance and saturation behavior |
| `G7_TelemetryPeers.left_mm_per_count`, `right_mm_per_count` | Wheel calibration conversion floats; zero denotes unavailable | **Not provable** without calibrated scale and mission-length error budget | **Keep float32.** With fixed-point Q0.16 and scale <1 mm/count, maximum quantization error ≈0.00000763 mm/count; that is only a *candidate* subject to calibration and total tick count |
| `terrain_calibration.roll_zero_deg`, `pitch_zero_deg` (`terrain.c`) | Angles from `atan2f`, captured while level; calibration logs currently expose millidegrees | Width depends on required angular precision (e.g. signed centidegrees to cover ±180°: 16 bits) | **Keep float32.** Current millidegree diagnostic output does not establish measurement accuracy or an acceptable precision loss |
| `terrain_calibration.b_valid` (`terrain.c`) | Boolean; protected by the calibration mutex | **1 bit** | **Implemented and boundary-tested** |

**Quantization illustration, not an accepted design:** With a 0.01° roll step, maximum one-time angular rounding is 0.005°. For the current 110 mm one-wheel height model, the worst first-order height difference from one such rounding is approximately **0.0096 mm**. If both stored reference and live angle are rounded independently, allow approximately **0.0192 mm**. This estimate only concerns numerical error in the *existing* model; it does not validate that the model measures full-width humps correctly.

## 19 previously unresolved field ranges — source findings

| Field group | Code-derived range | Required bits / resolution | Current decision |
|---|---|---|---|
| `bc_layout_result_t.letter`, `.runner`, `.other` | A–Z = 0..25; **255** is the current `BC_LAYOUT_NONE` sentinel | **8 bits as currently encoded**, or 5 bits *if* all internal sentinels/consumers are migrated to 31 | **Keep 8 bits** rather than risk invalid-letter changes in the last stage |
| `bc_layout_result_t.bars` | 0..96 raw bars (`BC_LAYOUT_MAX_RAW`) | **7 bits** | **Packed and tested** |
| `bc_layout_result_t.err_centi` | 0..60000 (fit error is capped at 60000 in the no-fit path) | **16 bits** | Existing width correct |
| `bc_layout_result_t.module_x10` | 0 (no fit), otherwise 300..1000 after `layout_module_ok` | **10 bits** | **Packed and tested** |
| `bc_layout_result_t.gap_permille` | 0..60000, explicitly saturated | **16 bits** | Existing width correct |
| `bc_match_result_t.support` | 0..6 votes across six threshold levels | **3 bits** | **Packed and tested** |
| `bc_match_result_t.score_z100`, `.margin_z100`, `.runner_score_z100` | Signed scores; algorithm uses sentinel **−1,000,000** and evidence/geometry arithmetic | Candidate compact signed storage possible, but cross-branch arithmetic bounds not completely proven | **Keep int32** until score arithmetic is exhaustively bounded and the known barcode rejection-case regression is resolved independently |
| `bc_wave_status_t.white_rejects` | `g_white_rejects` saturates to **255** in the diagnostic copy | **8 bits** | Existing width correct |
| `bc_wave_status_t.last_process_ms` | Processing time saturates to **65535 ms** on assignment | **16 bits** | Existing width correct |
| `terrain_snapshot.roll_deg`, `.pitch_deg` | Relative angles wrapped by `angle_difference` into approximately **−180..+180°** | Signed 0.01° candidate fits **16 bits**, but no accepted sensor error budget | **Keep float32** until calibrated threshold tests and sample logs validate it |
| `terrain_snapshot.hump_height_mm`, `.last_hump_height_mm`, `.max_hump_height_mm` | Current code calculates **110 × sin(abs(roll)) mm**, rounded to integer, hence nominally **0..110 mm** for valid finite roll angles | **7 bits** for this specific one-wheel model, with validity stored separately | **Retain uint16_t pending physical hump-geometry confirmation**. A different future model may change the representable range; add sanitization before narrowing |
| `terrain_snapshot.heading_deg` | `atan2f` and normalization produce approximately **0..360°** when heading is valid | Candidate unsigned centidegrees 0..36000 fits **16 bits** | **Keep float32** until magnetometer/turn-rate accuracy and quantization tests are agreed |

**Additional caution:** The current hump algorithm explicitly estimates the elevation difference between two wheels (110 mm track spacing). A full-width hump can lift both wheels without producing that roll difference. That is a **physical modelling limitation**, not a number-of-bits issue, and should not be hidden by packing the height field.

## Three outstanding protocol decisions

1. Frame `payload_len` is 0..48, logically **6 bits**, yet version 1 stores it in a byte. It is an envelope field with a compatibility constraint; strict packing requires coordinated changes and a version bump.
2. The four route-letter mapping values use **20 bits** already. Do not repack them a second time merely to claim additional savings.
3. The command and status frames still contain version-1 byte-aligned controls. Decide with the assessor whether the requirement mandates packed **every field on wire**, or compact binary payloads at the current level suffice. A v2 migration must update both C serializer and JS parser with golden test vectors and explicit rejection of mis-versioned frames.

## Verification performed in this environment

- **36,000** deterministic steering updates: original V11 and precision-patched V11 produced byte-identical traces (return, integral, direction memory and hold/counters). GNU C host build at `-O2 -Wall -Wextra -Werror`.
- Native boundary test for signed ±1 steering memory, matcher 0..6, barcode bars 0..96, accepted module 300..1000, and terrain validity 0/1: PASS.
- Application style: 112 source files, 0 errors; network ownership: PASS.
- MQTT C/JavaScript golden-vector tests, internal minbits tests, barcode port and IR tests: PASS.
- Existing navigation tests: 144 assertions PASS; obstacle/controller integration GCC and Clang: PASS.
- Dashboard: 70/70 Node tests PASS.
- **Not verified:** ARM compile/map/RAM, real Pico W output, timing jitter, on-car STOP, wheel calibration, IMU noise, sustained distance accuracy, barcode scan yield under these changes.

**Barcode replay limitation:** The source ZIP contains only `tools/barcode_data/README.md`, not the raw labelled `.log` captures. Saved real-pass replay could not be repeated in this session. The isolated barcode field tests do not replace driven barcode recognition measurements.

## What is still needed from the physical car, and why

Only if you want a strict *final* precision sign-off, collect these (without personal Wi-Fi credentials):

1. **Wheel calibration and control logs:** loaded wheel circumference, counts/revolution, verified mm/count, straight-line distance error, measured upper RPM and wheel speeds. Defines numeric range/error for RPM, target speed and residual odometry fields.
2. **IMU/hump tests:** stationary ± noise, known 5°/10°/20° tilts or available reference angles, several known-height humps, orientation signs, magnetometer validity. Defines angular precision and confirms whether roll-based geometry is appropriate.
3. **Barcode trial logs:** representative small and large letters, accepted/rejected cases, process latency and score extremes; needed before narrowing score-related integers.
4. **Same-toolchain ARM ELF comparison:** build V10 and the reviewed V11 with identical flags and run `python3 tools/compare_arm_ram.py <V10.elf> <V11.elf>`; include RTOS stack/queue measurements from a real run.
5. **Assessment decision:** is a new minimum-bit MQTT v2 envelope mandatory? Do not make a breaking protocol change until this is settled.

## Recommended next decision

**Keep control-loop, calibrated odometry, IMU and scoring calculations as 32-bit float/integer where precision is not established.** Convert a continuous field to fixed point only after selecting a quantified tolerance, writing saturating conversion helpers, validating worst-case errors and replaying matching real inputs. Minimum-bit review is not synonymous with safely shrinking all arithmetic intermediates.

**Release classification:** exact-value compacting applied and host-verified; precision-dependent storage changes proposed but not yet approved. The entire Week 13 change is **not certified complete** without the remaining measurements and protocol decision.
