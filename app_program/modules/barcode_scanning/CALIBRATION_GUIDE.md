# Buddy 3 calibration: safe hardware procedure

## Current build: drive and decode

### Fourth moving log: full reverse A rejected by narrow-width coherence

This log completed capture and the trailing quiet zone successfully:

```text
[BARCODE] Rejected: code 7
[BC FRAME] event=1 capture_error=0 runs=29/29 travel_units=3816
[BC WIDTHS] event=1 82,46,252,35,252,49,83,222,73,63,239,39,69,223,62,100,64,79,213,90,87,45,223,76,208,118,35,248,71
```

Sampling input checks remained clean, with maximum gap 4332 us. The sum
of the 29 run widths is 3446 units, approximately 279.25 mm. The reported
3816 units also include the 370-unit trailing quiet requirement.

The 20 narrow widths range from 35 to 118 units; their median is 70.
The nine wide widths range from 208 to 252. Their exact binary pattern
is reverse `*A*`: in traversal order the masks are 0x052, 0x121, 0x052;
reversing all 29 runs yields 0x094, 0x109, 0x094. No letter-table entry
or single-letter frame structure needs to change.

The old narrow-coherence check rejected the frame before pattern matching
because 118 exceeds twice 35. The revised check bounds each narrow run
between half and twice the median narrow width instead. For this frame
that is 35-140 units. It retains the 1.7 minimum wide/narrow separation,
minimum trustworthy run width, wide-group coherence, exactly nine wide
runs, narrow symbol separators, exact start/stop markers and A-Z matching.
It does not replace failed captures with the nearest letter.

The actual recorded widths now decode to **A, reverse traversal**, in a
host regression. Forward order and doubled scale also pass; additional
tests cover narrow variation across A-Z in both directions and rejection
of extreme short runs or ambiguous group separation. This changes the
accepted variation envelope, so continued physical testing is needed.

**Flash `build_make/mtk3pico_smp1_usb_cdc_barcode_drive_decode.uf2`.**
It keeps motion, IR, USB-only logging and all trace outputs enabled. A
repeat capture of this pattern should print `[BARCODE] Detected: A`.
The host replay establishes the decoder correction; a new moving test
is still needed to verify the rebuilt firmware on the robot.

### Third moving log: only 12 runs captured

The trace image returned:

```text
[BC FRAME] event=1 capture_error=5 runs=12/29 travel_units=4404
[BC WIDTHS] event=1 117,187,126,630,126,772,148,302,170,162,121,1521
```

All input-error counters stayed zero and the maximum observed sample gap
was **4088 us**. Thus this run supplies no evidence of capture scheduling,
encoder direction/skew/step or lock failures. It does show an incomplete
optical run sequence, ending after approximately `4404/12.34 = 356.89 mm`
of encoder-estimated travel from the candidate's first black bar.

The six completed white runs convert to approximately **15.15, 51.05,
62.56, 24.47, 13.13 and 123.26 mm**. Several greatly exceed the largest
nominal white feature of 18.96 mm for the reported prints. The observed
sequence is not a complete single-letter Code 39 frame. Raising the
distance bound cannot restore missing transitions or justify a letter.

The controller also repeatedly commanded strong left correction, including
zero left PWM while right PWM remained nonzero. That can take the barcode
sensor away from the stripe sequence. This is trajectory evidence, not
proof of which physical condition caused the missed bars.

Next test uses the **same trace UF2**, with motors and IR enabled:

1. With motor power off while adjusting, bring the TCRT5000 sensing face
   closer to the paper; start around 2.5-3 mm, based on the supplied
   TCRT5000 datasheet's 2.5 mm peak operating distance. This is a test
   position, not a guaranteed optimum for the assembled module.
2. Recheck comparator adjustment for stable white=0 and black=1 at that
   mounting height, including over the narrowest printed bar and gap.
3. Arrange the guide line so the normal line-following sensors remain on
   their intended track and GP27 crosses every barcode stripe. Avoid a
   test path that causes repeated turning across or away from the print.
4. Repeat the moving scan and send the trace rows. A completed candidate
   needs all 29 alternating runs, followed by sufficient white travel.

No motion policy, pin mapping, letter table or scan limits were changed in
response to this log. Physical height, comparator threshold and path remain
the next variables to check; a specific cause has not yet been isolated.

### Second moving log: input healthy, capture rejected by distance

The next COM8 log shows all adapter input-error counters at zero while
moving, with a worst processed sample gap of **4099 us**. GP27 reports
both raw levels. The repeated code-1 failure from the first run is absent.
One later `[BARCODE] Rejected: code 5` is a capture-distance failure, not
an unknown letter. The once-per-second raw snapshots cannot establish how
many black/white transitions were seen between reports.

The newest image adds bounded capture diagnostics. Flash
**`build_make/mtk3pico_smp1_usb_cdc_barcode_drive_trace.uf2`** for the next
moving scan; it keeps USB-only output, motion, IR and letter decoding
enabled, with the same capture validity and size limits.

- `[BC SCAN] edges=... state=... runs=.../29 travel_units=...` appears once
  per second. `edges` counts raw transitions cumulatively; `runs` is the
  number of completed alternating runs in the current candidate.
  States: 0 idle, 1 leading-white candidate, 2 capturing,
  3 trailing-white candidate, 4 complete, 5 invalid.
- `[BC FRAME] event=... capture_error=... runs=.../29 travel_units=...`
  preserves the latest terminal candidate before capture resets. Travel
  starts at its first black bar and includes trailing white if reached.
  Code 5 with fewer than 29 runs indicates excessive travel before the
  expected run sequence completed; code 5 with 29 runs indicates a complete
  run sequence whose first-to-last bar span failed the size limits.
- `[BC WIDTHS] event=... ...` lists the completed run widths for that
  terminal candidate, in encoder-sum units, beginning with black and then
  alternating white/black. Sum all 29 widths for first-to-last bar span;
  divide by the working 12.34 units/mm to estimate millimetres. This is
  a capture trace, not a guessed decoded letter. `capture_error=0` means
  capture completed; a later pattern decode can still reject it as code 7.

Send these lines together with `[BC INPUT]` and `[BARCODE]`. Ensure the
sensor path crosses each stripe rather than running along a stripe. A
model/height/threshold issue is one possible explanation for missed edges,
but the second log does not prove it. No physical cause is assumed and
no distance bounds were widened in response to this log. A host regression
now verifies that a single white-to-black transition with otherwise
healthy sampling produces a distance rejection and never a letter, while
small/large full frames preserve all 29 widths and still decode.

### Updated after the first moving COM8 log

The supplied moving test log contains repeated `[BARCODE] Rejected: code 1`
and no decoded letter. Code 1 is invalid sample input, which can come from
capture timing/queue validity, an excessive sample gap, reverse encoder
steps, skew, oversized steps or stale encoder timestamps. That log alone
does not identify which check failed.

The default USB CDC firmware also mirrors output to UART. Inspection of
`libtm_printf.c` shows interrupts stay masked throughout printing; the UART
sink waits for FIFO space. Heavy motor/controller output can consequently
delay the nominal 4 ms capture. The new test image disables only the UART
mirror at compile time while retaining USB logging, motors and normal IR.
No kernel, monitor or motion implementation source was edited for this fix.

**Flash `build_make/mtk3pico_smp1_usb_cdc_barcode_drive_usb_only.uf2`.**
It still enables movement. Open the USB serial terminal on COM8 (or its
new port number if Windows assigns one). Output is not available over a
GP0/GP1 UART adapter in this image.

Repeated code-1 messages are limited to one per second. A new `[BC INPUT]`
row reports cumulative counters once per second:

| Field | Meaning |
|---|---|
| `raw` | Latest GP27 level; reported black=1, white=0 |
| `n` | Samples processed by the adapter |
| `irq` | Records marked invalid by capture/queue; not exclusively an IRQ count |
| `gap` | Sample intervals above the configured 6 ms limit |
| `dir` | Normalized encoder steps going backward |
| `skew` | Left/right step mismatch or travel overflow |
| `step` | Sample travel above 10 encoder-sum units |
| `stale` | Changed encoder count with an overly old edge timestamp |
| `lock` | Samples dropped because the adapter lock was unavailable |
| `max_gap_us` | Largest processed sample interval in microseconds |

Counters can overlap for one sample and are not reset after each scan.
Send `[BC INPUT]` and `[BARCODE]` rows from a repeat moving test. Detection
still requires the full valid pattern and trailing white; no lost sample
is interpolated or accepted by relaxing the capture validity checks.

Rebuild this specific profile from the repository root with:

```sh
bash app_program/modules/barcode_scanning/build_drive.sh
```

The script forces recompilation with `TM_CONSOLE_UART=0`, copies the named
USB-only UF2 and marks the custom build profile. A later ordinary Make
invocation rebuilds its default console profile rather than silently
reusing the override. Plain `make CONSOLE=usb_cdc` does not itself disable
the UART mirror.

At the user's request, normal robot startup is restored. Current settings:
`BC_CAL_ENCODER_ONLY=0`, `BC_CAL_READY=1`, `BC_CAL_MEASURE=0`, and
`BC_CAL_AUTO_ACK=1`. Motion, IR, controller and the existing IMU startup run
normally. The dedicated 4 ms barcode sampler is armed after the IR driver
and barcode worker are ready. A complete valid scan prints
`[BARCODE] Detected: X`, where X is the decoded uppercase letter. Results
are acknowledged after console service so subsequent barcodes can be read;
letter-to-navigation mapping is not enabled.

Flash `build_make/mtk3pico_smp1_usb_cdc_barcode_drive.uf2` for this test.
**This image enables movement.** Open the USB serial terminal, let the
existing IMU startup calibration finish, then use the normal line-following
track with the barcode under GP27. Start on at least 30 mm of white before
the first bar and continue at least 30 mm beyond the last bar. The reported
50 mm quiet areas accommodate these requirements.

Initial drive-test settings use the measured 12.34 sum units/mm:

| Setting | Initial value | Basis |
|---|---:|---|
| Minimum run | 20 units | About half the smallest nominal narrow run |
| Leading/trailing white | 370 units each | About 30 mm each, inside available 50 mm |
| Complete barcode span | 1717-4032 units | Approximately 10% tolerance around 154.6-297 mm |
| Timeout | 15000 ms | Initial trial limit |
| Maximum sample travel | 10 units | At most one quarter of nominal minimum feature |
| Maximum wheel-step skew | 6 units | Initial trial limit, not measured at 4 ms |
| Maximum sample gap | 6 ms | Initial trial limit; ISR also rejects excessive lateness |

Signs remain left=-1, right=+1; black=1, white=0; sensor TCRT5000 at the
reported approximately 10 mm height. These limits are engineering starting
values for the requested moving test, not completed physical validation.
The new production-configuration host tests decode two consecutive frames
at small/large sizes with the measured encoder signs. ARM linking succeeds;
actual moving scans, lighting/height sensitivity and IRQ jitter remain to
be verified. If scans reject, send the `[BARCODE]` lines and surrounding
console output. Do not substitute a guessed character for a failed scan.

## Previous encoder-only test procedure

To rerun the wheel calibration, set `BC_CAL_ENCODER_ONLY` to **1**. The startup
gate in `app_main.c` runs only `bc_enc_run()`: motor initialization with
zero PWM, an explicit motor stop, and encoder initialization. It skips
the motion subsystem, Mission Coordinator/controller, IR, IMU and barcode
capture tasks. The USB console/kernel service tasks still run. No alarm-2
reservation or barcode sensor is required for this encoder-only session.

A fresh image is available at
`build_make/mtk3pico_smp1_usb_cdc_encoder_cal.uf2`. This image is for
encoder calibration, not line following or barcode decoding. Firmware
linking and host regression checks pass. The user has now supplied one
three-pass hardware session from COM8, recorded below. This does not yet
validate barcode scanning. Keep motor power disconnected during hand rolling.

1. Mark a straight 100 mm path on a flat surface.
2. Flash the encoder-calibration UF2 and open its USB serial port in a
   terminal. Enable terminal logging to a text file. If the terminal needs
   a baud setting, use 115200; this USB CDC link is not a GP0/GP1 UART cable.
3. Wait for `[ENC CAL] ENCODER-ONLY`. Timed passes wait for the USB serial
   terminal to open (DTR/connected state), not just USB enumeration.
4. Each pass has a **10-second PREP countdown**. Place the robot at the
   ruler start during PREP; returning it to the start is not measured.
5. At **GO**, push the whole robot forward exactly **100 mm**, then stop
   and keep it still until DONE. The capture window lasts 15 seconds;
   finish at least one second before it ends. Do not spin wheels in air.
6. Repeat for all three passes. The board then prints SESSION COMPLETE
   and sleeps with no normal robot tasks running. Reset to repeat.
7. Send the complete terminal text, especially these CSV rows:

```text
[ENC CAL CSV] pass,requested_mm,L_start,L_end,R_start,R_end,L_delta,R_delta,still_msec
[ENC CAL CSV] ...
```

`requested_mm=100` is the instructed distance, not an automatic distance
measurement. Report any overshoot, wheel slip, reverse movement, or bad
pass when sending the log. Zero counts on either wheel or movement during
the last second produce an INVALID warning; timing uses the actual encoder
microsecond clock rather than assuming RTOS delays run exactly on time.

If USB reconnects mid-session, reset and capture a complete new session.
Verify the right encoder's GP28 VBAT-sense jumper is open as described
below. This firmware needs `CONSOLE=usb_cdc`; an encoder-only UART build
is rejected because GP0/GP1 are used by the encoder.

To rebuild from the repository's MSYS2 shell:

```sh
make -C build_make SMP=1 CONSOLE=usb_cdc -j4
```

The Make output is `mtk3pico_smp1_usb_cdc.uf2`; the `_encoder_cal` file is
a copy of the build delivered here. Recopy it after any later rebuild if
you want to retain that descriptive name.

To restore normal startup, set **`BC_CAL_ENCODER_ONLY 0` and rebuild**.
For barcode sensor measurement mode later, additionally set READY=1 and
MEASURE=1 as described below. With encoder-only mode enabled, those barcode
flags do not start any IR/barcode task.

The sections below record measurements and the earlier measurement-mode
procedures. The active drive/decode profile is specified at the top of
this guide; do not confuse it with the archived encoder-only procedure.

## Confirmed sensor setup

Corrected user report on 2026-10-29: **TCRT5000**, mounted with its sensing face
approximately **1 cm (10 mm) above the paper**. Reported digital polarity
is **black=1, white=0**. Keep this mounting and comparator adjustment fixed
during the next edge-logging session. Recheck polarity at this height.

The earlier ST188 identification was withdrawn. The supplied TCRT5000
datasheet gives a 2.5 mm peak operating distance. The reported 10 mm height
is the current setup, not a validated optimum. Actual narrow-bar/white-gap
detection, noise and tilt
tests must establish whether it is reliable at the smallest print size.
The scanner has no model that compensates widths automatically for height.

## Latest encoder session: working conversion

The second COM8 session is saved as `encoder_calibration_02.csv`. Assuming
each forward pass actually covered 100 mm without slip:

| Pass | Left delta | Right delta | Sum units/mm | Stationary at end |
|---|---:|---:|---:|---:|
| 1 | -616 | +603 | 12.19 | 4229 ms |
| 2 | -623 | +624 | 12.47 | 1679 ms |
| 3 | -612 | +624 | 12.36 | 11489 ms |

All passes exceed the logger's 1000 ms stationary requirement. Each wheel
totals **1851 counts over 300 requested mm**, giving **6.17 counts/mm per
wheel**. The working encoder-sum conversion is **12.34 units/mm**, with
forward signs **left=-1, right=+1**. The observed per-pass range is
12.19-12.47 units/mm, a full spread of about 2.27% of the average. This is
repeatability evidence, not independent proof of the ruler distance or
the absence of lost pulses at higher speed.

Use this second session for the working conversion rather than averaging
it together with the less consistent first session. No additional
encoder-only repeat is required now unless a pass had a known physical
error. Continue with mounted barcode sensor edge/timing measurements.

| Physical distance | Expected encoder-sum units at 12.34 units/mm |
|---|---:|
| Available 50 mm white zone | 617.00 |
| Smallest 154.6 mm barcode span | 1907.76 |
| Largest 297 mm barcode span | 3664.98 |
| Nominal narrow feature on smallest print (154.6/47 mm) | 40.59 |

These are expected widths, not final acceptance thresholds. Actual edge
logs must establish noise rejection, tolerances, per-sample skew/step and
jitter. Sensor identity and mounting height are now recorded above.
At the time of this measurement the encoder-only profile was selected.
The user subsequently requested moving barcode decoding, now enabled as
specified in the active profile above. Sensor tolerances remain provisional.

## First encoder session (retained for comparison)

The supplied COM8 log is saved as `encoder_calibration.csv`. All three
passes recorded both wheels and ended stationary for more than 8 seconds.
The log's `requested_mm=100` is an instruction, so the conversions below
assume the user actually rolled straight forward 100 mm per pass without
slip. The logger cannot independently verify that physical distance.

| Pass | Left delta | Right delta | Left counts/mm | Right counts/mm | Sum units/mm |
|---|---:|---:|---:|---:|---:|
| 1 | -598 | +607 | 5.98 | 6.07 | 12.05 |
| 2 | -585 | +542 | 5.85 | 5.42 | 11.27 |
| 3 | -602 | +598 | 6.02 | 5.98 | 12.00 |

All three agree on forward signs: **left=-1, right=+1**. These signs are
now entered in `bc_cal.h`. The aggregate scale over 300 requested mm is
`(1785 + 1747) / 300 = 11.7733` encoder-sum units/mm. Per-wheel aggregate
values are 5.9500 left counts/mm and 5.8233 right counts/mm.

**Distance scale remains provisional:** passes 1 and 3 agree closely;
pass 2 is about 6.28% lower in total counts than their average. Its right
wheel records about 10.04% fewer counts than their average. This could
reflect distance variation, wheel slip, a curved push or encoder count
loss; the endpoint log cannot distinguish them. Retain all three rows
rather than silently discarding pass 2. A repeat was requested and has
now been received above; the second session supplies the working scale.

For orientation only, the provisional aggregate scale converts:

| Physical distance | Expected encoder-sum units |
|---|---:|
| Available 50 mm white zone | 588.67 |
| Smallest 154.6 mm barcode span | 1820.16 |
| Largest 297 mm barcode span | 3496.68 |
| Nominal narrow feature on smallest print (154.6/47 mm) | 38.73 |

These are expected distances, not acceptance thresholds. MIN_RUN,
leading/trailing requirements, barcode tolerance, per-sample step/skew
and jitter limits were unset during this first session. See the active
drive-test profile above for the subsequently selected initial limits.

## Dimensions from the supplied files

**Available quiet zones (user report, 2026-10-29):** 5 cm (**50 mm**)
of uninterrupted white travel before the first bar and 5 cm (**50 mm**)
after the last bar. These are available distances, not yet firmware
acceptance thresholds. With measured encoder-sum scale `S` units/mm,
each available zone corresponds to `50 * S` units. Select thresholds
within that available travel, allowing measured edge/encoder uncertainty,
and verify them with scan logs. Do not enter 50 directly into the
`*_UNITS` settings unless the measured scale is exactly one unit/mm.

**Updated size requirement (2026-10-29):** print size can vary. The user
reports a barcode size range of **15.46-29.7 cm (154.6-297 mm)** on A4
landscape paper. The user clarified that this describes the distance from
the first black bar to the last black bar on the printed page, across the
stripes. The user corrected the upper limit to 29.7 cm. An A4 sheet is
297 x 210 mm. At a full 297 mm bar span there is no white margin left
along that sheet edge; provide uninterrupted white surface before and
after the sheet for scanner quiet zones. Verify actual prints for clipping
and printer margins.

The earlier **141 mm** measurement is one sample, not a universal target.
Its nominal **3/9 mm** run widths apply only to that sample. Measure the
first-to-last black bar distance **across the stripes in the travel
direction**, after printing. The long dimension along an individual bar
does not set run widths. In the supplied photos, crossing the horizontal
bars means travelling approximately top-to-bottom or bottom-to-top.

For the clean 47-unit, 3:1 artwork, calculate nominal narrow width as
`actual_printed_span_mm / 47` and wide width as three times that value.
For the reported scan-span range, nominal narrow widths are about
**3.289-6.319 mm** and wide widths about **9.868-18.957 mm**. These are
derived from the supplied artwork ratio, not sensor-edge measurements.

Calibrate encoder units/mm once for the robot. For variable print sizes,
choose sampling/noise bounds using the smallest valid feature, and the
maximum scan distance using the largest valid printed span. The existing
decoder classifies relative narrow/wide widths; it does not require every
print to have exactly 3 mm narrow runs. Absolute capture limits still need
verified smallest/largest prints and hardware logs.

Source files: `BARCODE.pptx` and `Barcode Sample.pdf`, supplied on
2026-10-29. The deck has A/F/Z at several picture sizes. The PDF has A
on pages 1/2 and Z on pages 3/4, at two different sizes. The clean artwork
has 29 alternating runs: 20 narrow and 9 wide, with a 3:1 width ratio.
The complete bar span therefore contains 47 narrow-width units.

The following dimensions come from slide picture extents and PDF vector
coordinates. They apply at **100% document scale**, before printer scaling:

| Artwork | First-to-last black bar | Narrow bar/gap | Wide bar/gap |
|---|---:|---:|---:|
| PDF pages 2/4; PPT slides 4/8/12 | about 164.90-164.94 mm | 3.509 mm | 10.527 mm |
| PPT slides 3/7/11 | about 242.50 mm | 5.160 mm | 15.479 mm |
| PDF pages 1/3; PPT slides 2/6/10 | about 328.50 mm | 6.989 mm | 20.968 mm |

PDF derivation: narrow rectangle width 90.581, wide width 271.74;
form horizontal matrix 0.016405; page horizontal scale 13.333 (large),
6.6944 (small A) or 6.6929 (small Z). Multiply these three factors by
25.4/72 to obtain millimetres. The bar span runs from x=65.806 to
x=4323.101 in form coordinates. PPT picture widths are 338.667, 250,
or about 170 mm and use the same proportional clean A/F/Z artwork.

Actual physical measurements take precedence over document-size
calculations. "Fit to page" changes every length. Measure the printed
first-to-last bar span at the smallest and largest required print sizes.

The small PDF artwork has only about 2.55 mm of white margin on each side
inside the picture. This does not establish the available track quiet zone.
Include the uninterrupted white surface beyond the picture when measuring
leading/trailing travel. Keep the sensor away from the letter label and
the watermarks on the later W/X/Y slides.

Use the included worksheet calculator from the repository root:

```sh
python3 app_program/modules/barcode_scanning/calibrate.py --layout small
python3 app_program/modules/barcode_scanning/calibrate.py --span-mm 141
```

The second command illustrates the earlier 141 mm sample. Replace 141
with the actual printed scan span being checked. No fixed-size default
is assumed. After the
100 mm hand-roll, append `--left-delta L --right-delta R` using your signed
count differences. Append `--gap-us G` using the measured worst valid gap.
The calculator gives encoder-sum conversions and the conservative sampling
bound. It does not invent noise tolerances or write firmware thresholds.

For your print, `narrow_mm = measured_bar_span_mm / 47`,
`wide_mm = 3 * narrow_mm`, and
`expected_span_units = measured_bar_span_mm * sum_units_per_mm`.
Set barcode length limits around that expected span using repeated sensor
passes. Encoder scale, usable quiet-zone travel under the mounted sensor,
and jitter still need the robot validation described below; the available
white distance is now reported as 50 mm on each side.

For the earlier 141 mm sample, the nominal four-sample ceiling is 187.5 mm/s at
a 4000 us gap, or 150 mm/s at a measured 5000 us gap. These are calculated
ceilings. Validate the lower practical speed with sensor noise and tilt.

## What the additional hardware files establish

Reviewed the additional supplied files on 2026-10-29. These are component
specifications and wiring references, rather than recorded robot passes.

| File | Useful information | How it affects barcode calibration |
|---|---|---|
| `tcrt5000.pdf`, page 1 | TCRT5000 peak operating distance is 2.5 mm; stated 0.2-15 mm range is for relative collector current above 20% | If the installed sensor is TCRT5000, start height trials around 2.5 mm from its sensing face. This range does not guarantee reliable 3 mm feature detection. |
| `Infrared-Reflective-Sensor-UserManual.pdf`, pages 1-2 | Describes an ST188 sensor with LM393, supply 3.0-5.3 V, DOUT/AOUT and a 3.3 V wiring example | This describes a different sensor: the user corrected the installed model to TCRT5000. Do not use its ST188-specific specifications to calibrate this robot. |
| `PicoCar.pptx`, slides 1 and 4 | Motor connector labels for encoder ground, A, B, encoder supply and motor leads; sensor D0/A0 wiring | Helps identify the wires and digital output, but supplies no encoder PPR, gear ratio or wheel diameter. Match the actual connector before use. |
| `RP-008312-DS-2-pico-w-datasheet (1).pdf`, section 3.2 | Pico W GPIO uses a fixed 3.3 V rail; GP26-28 support digital input or ADC | GP27 digital input is supported. Verify sensor DOUT high voltage is compatible with this input; module supply ratings alone do not establish DOUT voltage. |
| `Robo_Pico.pdf` and `Robo Pico Board Datasheet.pdf`, page 4 | VBAT sense jumper connects divided battery voltage to GP28/ADC2 | The current robot uses GP28 for right encoder B. Verify that jumper is open before collecting right-wheel counts. No wiring/configuration outside barcode was changed. |
| `LM393-D.PDF` | Comparator specifications | Explains the threshold stage, but cannot determine the installed module's potentiometer setting, black/white transition positions or whole-sensor response time. |
| `OKY3231-2.pdf`, `lsm303dlhc.pdf`, `DS_lsm303d.pdf` | IMU module/chip references | Useful for separate tilt work; no wheel-count calibration or barcode quiet-zone measurements. Do not assume LSM303D and LSM303DLHC are interchangeable. |
| `HCSR04.pdf` | Ultrasonic ranging module reference | Supplies no barcode feature, encoder-scale or reflective-sensor mounting measurement. |
| `mtk3pico_smp1_uart.uf2` | Firmware image, not a recording | Its payload contains `[MOTOR SPEED] ... COUNT L=%d R=%d` but lacks the current `[BC CAL]` and `[BC CAL EDGE]` strings. It cannot substitute for the current measurement build. No firmware was flashed during this review. |

The slide's sensor photo has a comparator adjustment potentiometer. With
the actual sensor mounted and motors isolated, adjust it only as needed
for stable **white=0, black=1**, then repeat the raw-level checks at the
expected height and tilt. Record the height and leave the adjustment fixed
while measuring edges. No datasheet provides this robot's correct setting.

The UART UF2's `MM/S` value is not a measured distance reference unless its
wheel geometry/count scale has already been verified. If a working console
already shows `COUNT L=... R=...`, those signed counts can be recorded at
the beginning and end of the 100 mm hand-roll. Its UART name does not
establish console compatibility with the current GP0/GP1 encoder wiring.
Use the current encoder-safe measurement build for the barcode edge logs.

Still required on the robot:

1. Forward signs and a working 12.34 units/mm distance scale are recorded
   from the second session. Validate their use against actual barcode edge
   logs at intended speed; no further encoder-only repeat is needed now.
2. Actual sensor face height and stable raw levels at realistic tilt.
3. Validate the reported 50 mm white travel on each side under the mounted
   sensor; convert it to encoder units after measuring encoder scale.
4. Valid edge logs plus worst sample gap, step, skew and invalid count at
   intended scanning speed. These set acceptance tolerances.

`BOARD_ENC_COUNTS_PER_REV=1056` and `BOARD_WHEEL_DIAMETER_MM=65` are explicitly
marked unverified in the shared board header. None of the supplied motor
wiring references verifies them. They have not been used to fill barcode
encoder thresholds.

## Before connecting the robot

1. Have the team explicitly reserve **RP2040 TIMER ALARM2 and IRQ 2** for
   Buddy 3. This is not guaranteed by a successful build.
2. Confirm GP27 is the barcode IR sensor and the encoder pins are not used
   by the selected console. Read logs over the approved USB CDC console.
   The current USB profile also retains a UART mirror (`TM_CONSOLE_UART`);
   selecting USB alone does not disable it. Do not attach a UART adapter
   to encoder GP0/GP1; have the platform owner verify the actual pin mux
   and console setup before collecting wheel measurements.
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
The worker and IR task can each wait about 20 seconds for the console,
in addition to startup and settling delays; allow that time after reset.
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

Recorded from the user on 2026-10-29: **black = 1, white = 0** on GP27.
`bc_cal.h` now sets `BC_CAL_BLACK_LEVEL 1U`. Recheck this if the sensor,
mounting or comparator adjustment changes.

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

Keep a worksheet for each pass:

| Pass | Distance mm | L start/end | R start/end | Sum units/mm |
|---|---|---|---|---|
| 1 | 100 | | | |
| 2 | 100 | | | |
| 3 | 100 | | | |

Also save the console log, measured black level, sensor height, narrowest
bar/gap, longitudinal quiet zones, `gap_us`, `step`, `skew`, and `invalid`.
Use the observations to choose thresholds; the repository contains no
measured calibration values for this robot.

"Both directions" in barcode tests means forward wheel travel over a
normal or reversed printed pattern. The configured encoder signs describe
forward robot travel; driving the robot backward is rejected by the scan
adapter. Use the same forward wheel direction for these measurements.

## Session 3: barcode widths and longitudinal quiet zones

With a ruler/caliper measure the actual narrowest black bar, narrowest
white gap, widest internal white gap, first-to-last bar length (use the
selected print size above), and leading/trailing WHITE distances in **the direction of
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
| `BC_CAL_MIN_BARCODE_UNITS` | Low tolerance for the selected print's first-to-last bar span, and >= 29 × MIN_RUN |
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
