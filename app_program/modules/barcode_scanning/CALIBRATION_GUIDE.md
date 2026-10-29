# Buddy 3 calibration: safe hardware procedure

The barcode module is **disabled** in the supplied firmware. This is
intentional. The existing `app_main.c` and `line_barcode/task_ir.c` already
contain the startup/arm hooks. No changes outside `barcode_scanning/` are
needed merely to *enable the existing measurement path*, but team approval,
physical calibration and a new firmware build are necessary.

## Before connecting the robot

1. Have the team explicitly reserve **RP2040 TIMER ALARM2 and IRQ 2** for
   Buddy 3. This is not guaranteed by a successful build.
2. Confirm GP27 is the barcode IR sensor and the encoder pins are not used
   by the selected console. Prefer an approved USB CDC console; the existing
   UART configuration can conflict with GP0/GP1 encoders.
3. **Physically disconnect drive-motor power or prevent motor rotation.**
   The current shared controller may command motion without an approved
   START. Never calibrate with a free-moving powered robot.
4. Do not change the line task's 10 ms period or any RTOS/kernel file.
5. Keep the actual printed sample and sensor at the intended mounting
   height. Note the lighting and sensor-to-barcode distance.

## Measurement-only build (no barcode decoding or navigation)

In `bc_cal.h`, temporarily set:

```c
#define BC_CAL_READY 1
#define BC_CAL_MEASURE 1
#define BC_CAL_AUTO_ACK 0
```

Leave **every `BC_CAL_*_UNITS` threshold at zero** for now. Explicit
`BC_CAL_MEASURE 1` forces the low-priority worker to log measurements,
without decoding or publishing letters. `BC_CAL_READY 1` is required by
**the existing `task_ir.c` compilation guard** to arm the 4 ms sampler.
Do not confuse this measurement-only setup with calibrated normal scanning.

From MSYS2 UCRT64 / the repository root:

```sh
python3 tools/check_app_standard.py
bash app_program/modules/barcode_scanning/run_tests.sh
cd build_make
make SMP=1 CONSOLE=usb_cdc -j4
```

Flash only using your team's **approved safe motor-isolated procedure**.
If `CONSOLE=usb_cdc` is not supported on the actual setup, ask the team for
an encoder-safe console method; do not silently choose GP0/GP1 UART.

The output should announce `[BARCODE] Measurement mode; decoding off`.
If it reports `Disabled`, investigate the timer/worker issue instead of
changing other buddies' code. A successful build is not hardware proof.

## Read the measurement output

```text
[BC CAL] raw=0 L=123 R=124 gap_us=4010 step=2 skew=1 invalid=0
[BC CAL EDGE] level=0 units=45 us=700000 ok=1
```

`raw` is the most recent **unfiltered GPIO 27 level**, not a decoded
letter. The `EDGE` record describes the **preceding** black/white run:
`level` is that run's raw level, `units` is the sum of absolute left and
right encoder increments, `us` is the run duration, and `ok` indicates
whether all recorded samples in that run were valid. Capture the output
from the very start of a scan so its first white quiet zone is visible.
`gap_us` is the largest observed sample gap (cumulative since start),
`step` is the largest travel increment per sample, `skew` is the largest
left/right difference and `invalid` is the number of invalid samples.

**Do not use a run with `ok=0` as calibration evidence.** Printing a
large number of edges can block the low-priority worker and overflow its
32-sample queue; if `invalid` increases, slow manual movement / reduce
console traffic and repeat. Log output must not come from the IRQ.

## Session 1: black/white polarity

Stationary (motor drive disabled, encoder power on), place GP27 over
white, then black, then white again, staying at each position long enough
to see a `[BC CAL]` status row. Record the raw level for each surface.
If black is consistently 0, set `BC_CAL_BLACK_LEVEL 0U`; if black is
consistently 1, use `1U`. Inconsistent values require sensor mounting,
threshold or lighting corrections before you continue.

## Session 2: forward encoder sign and counts per millimetre

Mark a 100 mm distance on a flat track. Hand-roll the robot forward while
both wheels roll naturally; record L/R counts at the beginning and end.
Repeat at least three times without resetting the encoder between
readings. Define

```
left_counts_per_mm  = abs(L_end - L_start) / 100
right_counts_per_mm = abs(R_end - R_start) / 100
sum_units_per_mm    = left_counts_per_mm + right_counts_per_mm
```

Set `BC_CAL_LEFT_FORWARD_SIGN` and `BC_CAL_RIGHT_FORWARD_SIGN` to +1
for increasing counts or -1 for decreasing counts, as physically measured.
The scanner uses the **sum of normalised encoder increments**, *not* mm.
If one wheel slips or rotates backward the data is invalid; do not use an
in-air wheel-spin to calculate the robot's real travelled distance.

## Session 3: barcode widths and longitudinal quiet zones

With a ruler/caliper measure the actual narrowest black bar, narrowest
white gap, widest internal white gap, first-to-last bar length (nominal
~141 mm), and the leading/trailing WHITE distances in **the direction of
travel**. The 20 mm gap between barcode and line is **lateral** and must
not be used as a quiet zone. Hand-roll the barcode sensor along A, F and Z
(in both directions) while capturing **all** `[BC CAL EDGE]` rows.
Record actual travel distance and units for every valid run.

Convert physical lengths to scanner units using `sum_units_per_mm`.
Use real minima/maxima from repeated runs with margin for quantisation.
Do not copy example values or presume a precise 3:1 narrow/wide ratio.

Configuration guidelines:

| Field | Derive from real observations |
|---|---|
| `BC_CAL_MIN_RUN_UNITS` | Positive floor below valid shortest run but above noise |
| `BC_CAL_LEADING_UNITS` | Minimum observed leading WHITE run; must exceed MIN_RUN |
| `BC_CAL_TRAILING_UNITS` | Required trailing WHITE distance; greater than 4 × MIN_RUN |
| `BC_CAL_MIN_BARCODE_UNITS` | Low tolerance for the complete ~141 mm first-to-last bar span, and >= 29 × MIN_RUN |
| `BC_CAL_MAX_BARCODE_UNITS` | High tolerance for complete bar span, >= MIN_BARCODE |
| `BC_CAL_TIMEOUT_MSEC` | High enough for slowest legitimate scan, bounded for a stopped robot |
| `BC_CAL_MAX_STEP_DIFFERENCE` | Allowed left/right step disagreement per 4 ms sample |
| `BC_CAL_MAX_SAMPLE_UNITS` | Above verified sample travel but <= minimum narrow run / 4 as a conservative target |
| `BC_CAL_MAX_SAMPLE_MSEC` | Above measured worst valid sampling gap, not a substitute for the 4 ms deadline |

All `*_UNITS` values are **encoder-sum units**, not millimetres. An
infeasible `MAX_SAMPLE_UNITS` bound means the current encoder resolution,
physical element size and/or speed are insufficient; slow down or request
an approved sampling/measurement redesign. Do not weaken all validation
thresholds just to make scans pass.

## Session 4: 4 ms jitter, speed, tilt and rejection

Repeat the above at approved low speeds, then at expected running speed.
Observe `gap_us` (nominal 4000 us, with measured jitter), `step`, `skew`,
`invalid`, and whether each expected Code 39 letter is successfully decoded.
Repeat at realistic pitch/roll angles with the normal barcode sensor
mounting. Record the physical angle/height separately: **this logger does
not measure IMU tilt**. Tilt and wheel slip can distort readings; a bad or
ambiguous result must be rejected rather than guessed.

Initial engineering speed limit (not a measured guarantee):

```
max_speed_mm_s = min_element_mm / (4 * worst_gap_seconds)
```

For example only, with a 3 mm measured minimum and a 5 ms worst observed
gap, the four-sample limit is 150 mm/s. Confirm this with the robot.

## Session 5: enable validated scanning

After the team agrees to measured values and TIMER ALARM2 ownership:

```c
#define BC_CAL_READY 1
#define BC_CAL_MEASURE 0
```

Replace ALL zero thresholds with validated values. Use
`BC_CAL_AUTO_ACK 1` **only for a standalone multiple-barcode terminal
 demonstration** where no Mission Coordinator consumes the result. Leave
it 0 for mission integration, so valid letters remain pending until
acknowledged by the Mission Coordinator.

Rebuild and flash in an approved safe hardware setup. Confirm exactly
one `[BARCODE] Detected: X` per valid A/F/Z scan, both directions, and
reject truncated/noisy scans. Motor steering and barcode-to-turn mapping
are **not** part of this module. The Mission Coordinator still needs to
consume `bc_port_peek`, acknowledge via `bc_port_ack`, and call
`bc_port_clear` on STOP/FAULT; implementing those hooks requires the
controller owner's approval.

To return to the safe default after tests, restore:

```c
#define BC_CAL_READY 0
#define BC_CAL_MEASURE 0
#define BC_CAL_AUTO_ACK 0
```

## Important remaining proof

- Actual physical 4 ms interval and missed sample count, with both cores
  and other tasks active.
- No TIMER ALARM2/IRQ 2 ownership collision on the team's final firmware.
- Full ARM GCC link and UF2 build on your Windows Pico toolchain.
- Physical sensor polarity, encoder signs/counts, black/white widths and
  reliability under speed, tilt and lighting variation.
- System START/STOP/fault safety and navigation integration via owners.

Host tests cannot establish any of the above physical facts.
