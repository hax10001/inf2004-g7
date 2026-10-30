# Digital Code 39 scanner

This subsystem reads exactly one uppercase letter in a Code 39 frame such
as `*A*` or `*Z*`. It measures both black bars and white spaces with the wheel
encoders. The returned letter excludes the stars; no checksum is required.
Widths are classified by their relative proportions, so there is no fixed
barcode size in millimetres.

The implementation uses the existing IR task, GP27 reading, encoder driver,
and IR snapshot mutex. No application startup, task priority, sampling delay,
motor command, GPIO assignment, shared driver, or build file is changed.
All changes and tests are inside `app_program/modules/line_barcode/`.

## Current commissioning state

Calibration lives in `bc_config.h`. Following the robot measurements, the
current settings use black level 1, left forward sign -1, right forward sign
1, `BC_CAL_READY=1`, and diagnostics enabled. Recheck these values after
changing hardware. Set `BC_CAL_READY=0` to collect commissioning statistics
without accepting barcode results. This does not disable the IR task.

Setting `BC_CAL_READY=1` with invalid settings also leaves the scanner in
measurement mode. It does not abort robot startup. There is no fifth mutex:
the four configured mutexes are already used by motion, IR, and terrain.

The Mission Coordinator is unchanged. It must explicitly consume results
and choose actions; scanning a letter does not command the motors.

## Data flow and public interface

```text
Existing task_ir
    timestamp -> ir_read(left, right, barcode) -> encoder_read(both)
                      |                              |
                      |                   confirm edges, measure widths
                      |                              |
                      |                   validate *letter* + white margins
                      |                              |
                      |                     persistent result mailbox
                      |                              |
               existing IR snapshot       coordinator get_result / ack
               and junction latches        (integration proposed below)
```

`line_barcode.h` exposes:

| Function | Behaviour |
| --- | --- |
| `line_barcode_get_result(&result)` | Copies the pending result. `E_OK` means available; `E_OBJ` means none. Does not consume it. |
| `line_barcode_ack(result.sequence)` | Consumes only that result. Stale or repeated acknowledgements return `E_OBJ`. |
| `line_barcode_set_enabled(bool)` | Discards partial capture on a change; preserves any pending result. Enabling uncalibrated settings returns `E_PAR`. |
| `line_barcode_get_diagnostic(&reading)` | Copies raw readings, maxima, last rejection, and the captured run widths under the same mutex. |

Results contain the letter, `BC_OK` status, sequence number, monotonic kernel
timestamp in milliseconds, narrow width, and full frame width. Widths use
summed wheel-count units. Optional `width_mm` is valid only when
`b_distance_valid` is true; it excludes the surrounding white margins.

The single-slot mailbox never overwrites an unacknowledged result. Sampling
for another frame resumes only after acknowledgement and fresh forward white
travel. Logging does not acknowledge results. Until the coordinator is wired
in, reboot between standalone first-result hardware trials, or acknowledge
through the public API from an approved consumer/debugger.

The capture states are `BC_WAIT_WHITE`, `BC_SCANNING`, `BC_TRAILING_WHITE`,
`BC_COMPLETE`, and `BC_ERROR`. Two consecutive equal samples confirm a changed
sensor level, using the position and time of the first candidate sample.
The fixed buffer holds 29 alternating runs: nine for each star/data symbol
and two white inter-character gaps. A result requires ten narrow widths of
white travel before and after the frame. Errors discard the frame and wait
for a new approach; they are reported through diagnostics, not as letters.

The sampler never waits for the barcode mutex and never prints. If a consumer
briefly holds the existing IR mutex, capture skips that sample; the next
sample's elapsed time and movement account for the gap. Decoder work is fixed
and bounded. The existing IR snapshot and latch publication still runs.

## Calibration on the robot

1. In `bc_config.h`, leave `BC_CAL_READY=0` and set `BC_DIAGNOSTICS=1`.
   Build and flash using the existing project procedure. Measurement mode
   does not stop the existing controller. For hand-rolling tests, disconnect
   motor power while keeping the Pico and sensors powered over USB.
2. Put the barcode sensor over plain white, then solid black, at its normal
   mounted height. Read `raw` in the once-per-second `[BC]` log. Set
   `BC_BLACK_LEVEL` to the level observed over black. If it flickers over a
   uniform patch, adjust sensor height/threshold before collecting widths.
3. Roll the robot straight forward and compare successive `L` and `R`
   counts. For each wheel independently, set its `BC_*_FORWARD_SIGN` to `1`
   if its raw count increases, or `-1` if it decreases. Zero is intentionally
   invalid. Do not infer these signs from IMU axis remapping.
4. Optionally roll a measured distance `D` millimetres, recording counts
   while stationary at each endpoint. Set:

   ```text
   BC_UNITS_PER_MM = (abs(L_end - L_start) + abs(R_end - R_start)) / D
   ```

   This is a sum, not an average. Use a floating-point constant such as
   `10.3f` only if that is what you measure. Leaving it zero still allows
   relative-width decoding; only the millimetre display remains unavailable.
   The unverified wheel diameter/counts-per-revolution are not used to guess
   this calibration.
5. Repeat straight driving at the intended scan speed with calibration still
   off. Inspect `gap`, `step`, and `skew`. They are maxima since startup:
   `gap` is microseconds between samples; `step` is the sum of absolute wheel
   count changes per sample; `skew` is the absolute difference between those
   two changes. Restart for a fresh trial. Set `BC_MAX_SKEW_UNITS` slightly
   above the observed straight-driving skew, allowing normal pulse timing
   differences. This is the count uncertainty allowance. Moderate steering
   also gets a proportional allowance through `BC_MAX_SKEW_PERCENT`, initially
   60. A distance window checks for sustained one-wheel pivots; do not disable
   that check just to make a strong turn produce a barcode result.
6. Measure the narrowest bar/space on the physical print, perpendicular to
   the bars. Verify that it provides sufficient resolution as described
   below. Allow plain white approach and departure regions of at least ten
   narrow widths each, with extra margin for positioning and measurement.
7. Set `BC_CAL_READY=1` after polarity, both signs, and the motion limits
   have been checked. Keep `BC_DIAGNOSTICS=1` for initial trials. A valid scan
   prints `[BC RESULT] A ...` or `Z ...`; `mode=2` means decoding is enabled.
   With no result consumer yet, the first valid letter stays pending.
8. Test both supplied sizes of A and Z, then other A-Z prints. Keep travel
   through the approach, frame, and departure, allowing normal line-following
   corrections. Qualify straight scans first. Turn diagnostic
   logging off for normal operation and recheck under the final workload.

Example log field meanings:

```text
[BC] mode=1 raw=0 L=123 R=-125 gap=20000 step=8 skew=2 run=1/40 err=0
```

`mode`: 0 disabled, 1 measuring, 2 enabled. `run=1/40` means the most recently
ended raw level-1 section measured 40 summed count units. It is a commissioning
measurement, not a validated bar; the first section after startup is partial.
Only the latest section is displayed, so these one-second logs are not a
complete edge history. The diagnostic API also exposes the decoder's fixed
run buffer. Printing is suppressed during an active decoding frame, and it
occurs outside the mutex, but console activity can still disturb timing.

| `err` | Meaning / useful check |
| --- | --- |
| 0 | No current rejection. |
| 1 | Invalid Code 39 shape, gap, start/stop, or width ratio. |
| 2 | Insufficient distance resolution; slow down or enlarge the print. |
| 3 | Leading white margin too small. |
| 4 | Whole-frame timeout. |
| 5 | Extra elements/insufficient trailing margin, or numeric/buffer overflow. |
| 6 | A normalized wheel count reversed; check forward signs and motion. |
| 7 | Wheel movement exceeded the steering/uncertainty allowance. |
| 8 | Excessive sample gap, delayed acquisition, or invalid digital level. |
| 9 | Robot stopped too long during an active scan. |

The initial limits are a 50 ms maximum sample gap, 1 ms maximum acquisition
window covering IR plus count reads, 500 ms stop timeout, and 15 s whole-frame
timeout. These are barcode-local settings, not scheduler changes. Qualify
them on the physical robot. A missed sample can invalidate a frame without
affecting the line-following snapshot.

## Steering while scanning

Normal line following changes the two wheel speeds. The barcode subsystem
does not require equal counts at every sample. For normalized forward wheel
changes `L` and `R`, the movement check now allows:

```text
abs(L - R) <= BC_MAX_SKEW_UNITS +
              (L + R) * BC_MAX_SKEW_PERCENT / 100
```

The comparison uses integer cross-multiplication without rounding. With
the current 6-count uncertainty and 60% proportional setting, a sample of
`L=20, R=12` is accepted; the previous absolute-only check rejected its
8-count difference. The percentage describes wheel-travel imbalance, not
motor duty or a PID gain. The uncertainty allowance softens that percentage
at small count distances, so it is not a strict wheel-speed ratio limit.

The same check runs over accumulated distance to distinguish asynchronous
encoder pulses from sustained one-wheel motion. The window is at least
`4 * BC_MAX_SKEW_UNITS + 4` units, and exceeds
`100 * BC_MAX_SKEW_UNITS / (100 - BC_MAX_SKEW_PERCENT)` units. This ensures a
persistent stationary-wheel pivot cannot repeatedly fit below the count
floor. The last partial window is also checked before publication. Each
window sums the two wheel distances independently, allowing opposite pulse
timing errors to cancel. Brief pivots can fit within the measurement
allowance; this is a conservative movement check, not a heading estimator.

Percentage settings must be below 100; zero supplies no proportional
allowance. Reversing is still rejected independently. The line-following
truth table, PID implementation, and motor commands are unchanged by this
barcode update. The scanner still measures summed wheel travel, which is a
proxy for chassis-centre distance. An offset barcode sensor or a changing
angle across the print can distort its actual path during turns. Code 39
width/pattern validation remains required, and steering tolerance needs
hardware qualification. This change does not increase the capture rate.

## Sample PDF and timing limits

The four pages of the supplied `Barcode Sample.pdf` were inspected before
implementation. Pages 1/2 encode `*A*`, pages 3/4 encode `*Z*`; their widths
have a 3:1 ratio. The smaller version halves horizontal widths, not bar
height. Independently extracted vector widths and source SHA-256 are saved
in `host/pdf_fixtures.json`; the tests do not regenerate those fixtures from
the decoder table. The A-Z table was checked against the maintained
[ZXing Code 39 reader](https://raw.githubusercontent.com/zxing-cpp/zxing-cpp/master/core/src/oned/ODCode39Reader.cpp).

The artwork's own side margins are only about 0.73 narrow widths. Surrounding
white paper/track must supply the rest of each required quiet zone. On the
large pages, the printed A/Z label intrudes into a space at centre height;
scan across a strip that avoids the text. At actual-size PDF printing, narrow
features are approximately 6.99 mm large and 3.51 mm small. Print-to-fit
changes those measurements, so measure the actual print.

The existing `APP_IR_PERIOD_MSEC=10` is a delay request. The kernel's 10 ms
tick and relative-delay insertion can make actual sample intervals roughly
10-20 ms plus execution time and jitter. Neither its delay nor its priority
is changed here. Use the measured worst gap, not the nominal value.

Every accepted narrow run must satisfy:

```text
smallest_narrow_units >= 4 * largest_sample_step_units + 2
```

The extra two units reserve encoder quantisation margin. Narrow and wide
clusters must also remain separated after allowing for measurement error.
The largest step includes the approach and departure, so sparse sampling in
white cannot silently hide extra bars. Widths are checked in two relative
clusters, allowing nominal 2:1 to 3:1 Code 39 proportions with tolerance.

A useful planning bound, before additional encoder/sensor margins, is:

```text
maximum_speed_mm_per_second < narrowest_printed_width_mm /
                              (4 * worst_sample_gap_seconds)
```

For a 20 ms gap, that is about 87 mm/s for a 6.99 mm element and 44 mm/s for
a 3.51 mm element. Real usable speeds are lower after quantisation and
variation are allowed. The current nominal 20 RPM / 65 mm wheel settings
imply about 68 mm/s, so the smaller full-size example may be undersampled
at the current speed. A speed change belongs to the motion/coordinator owner;
this module rejects unresolved scans and does not change motor settings.

Digital IR footprint, slipping wheels, unequal wheel scale, missing encoder
interrupts, turns, and other tasks' logging can still corrupt measurements.
The implementation uses count readings only, not paired count/edge timestamps
from the shared encoder driver. Host tests do not establish physical timing,
sensor performance, or encoder accuracy on SMP hardware.

## Coordinator integration proposal (not applied)

The existing coordinator is `app/task_controller.c`. A minimal approved
integration would add the barcode header, poll the result once per controller
cycle, look up the letter in a coordinator-owned 26-entry table, and
acknowledge after the event has been handled or deliberately ignored.
Enable scanning only in appropriate forward-travel states and disable it
when entering a stop/fault/sharp-turn state. Normal line-following corrections
can remain enabled. No scanner changes are required to
change a letter's assignment.

Use an explicit `assigned` flag in each mapping entry. Zero-initialising an
action enum alone is unsafe here: `LINE_FORWARD` is zero. An unassigned or
out-of-range letter must produce no action. A conceptual coordinator flow is:

```text
if get_result succeeds:
    if letter is A-Z and its mapping entry is assigned:
        submit that existing action to the coordinator's normal handling
        acknowledge only when the event has been accepted
    else:
        acknowledge and perform no action
```

The existing `line_follow_action_t` has forward/correct-left/correct-right/
uncertain decisions, not complete 90-degree turns or a barcode mission
command vocabulary. Do not assume that a one-cycle steering correction is a
complete mission action. The coordinator owner must select appropriate
existing actions or approve the smallest necessary event-handling extension.

There is no existing runtime mapping command channel in this checkout.
A coordinator-owned configuration table is the smallest initial approach;
changing it requires rebuilding/flashing. Updating it without reflashing
requires a separately approved command/configuration interface. No telemetry
or controller logic was added to invent that interface.

## Build, tests, and file inventory

The existing make rules automatically include production `.c` files directly
inside subsystem folders. `host/` tests are excluded from the firmware build.
No CMake or makefile addition is needed. The existing firmware command is:

```powershell
cd build_make
mingw32-make SMP=1 CONSOLE=usb_cdc PICO_SDK_PATH="C:\pico-sdk-2.2.0"
```

Run repeatable host tests from the repository root:

```powershell
python app_program/modules/line_barcode/host/run_tests.py
python tools/check_app_standard.py
```

The host runner uses GCC with strict warnings and temporary output files. It
tests the four PDF fixtures, every letter, proportional scaling, changing
speed, malformed/incomplete frames, quiet zones, debounce, timeout/stopping,
overflow, reversal/skew, timer/count wrap, sampling uncertainty, and rearming.
Service variants explicitly test enabled, uncalibrated, invalid calibration,
and diagnostic configurations. They exercise result retention/acknowledgement,
unchanged line decisions/latches, delayed acquisition, contention, timestamp
failure recovery, and the existing four-mutex startup budget. These are
simulated tests, not robot trials.

Steering regression cases decode all A-Z letters at two scales while
alternating unequal forward wheel speeds. Additional cases check opposite
encoder pulse timing, persistent slow pivots at several percentage settings,
and the final partial-window guard. Host test configuration is independent
of the local robot calibration macros.

Validation completed on 2026-10-30: 2,978 core checks and all four service
variants passed; the application standard checker reported zero errors.
A full firmware build also passed in an isolated copy with normal controller
startup, `SMP=1 CONSOLE=usb_cdc`, Pico SDK 2.2.0, `BC_CAL_READY=1`, and
diagnostics enabled. It produced an ELF and UF2 with zero compiler warnings
or errors. Original workspace startup edits were preserved. No firmware was
flashed and the steering update has not been tested on the physical robot.

| File | Change |
| --- | --- |
| `bc_config.h` | Added barcode-only commissioning and diagnostic settings. |
| `bc_decode.c`, `bc_decode.h` | Added pure A-Z Code 39 pattern and relative-width validation. |
| `bc_scan.c`, `bc_scan.h` | Added bounded edge/motion/quiet-zone acquisition state machine. |
| `line_barcode.c`, `line_barcode.h` | Extended existing lifecycle with sampling, persistent result API, diagnostics, and enable control. |
| `ir_shared.c`, `ir_shared.h` | Added internal lock/unlock access to the existing IR mutex; snapshot/latch logic unchanged. |
| `task_ir.c` | Added timestamp, sample hook, and optional logger around existing snapshot publication. |
| `task_ir.h` | Corrected stale interval documentation; task signature unchanged. |
| `host/test_core.c` | Added decoder/acquisition regression tests. |
| `host/test_service.c` | Added service, resource-budget, and IR preservation tests. |
| `host/pdf_fixtures.h`, `host/pdf_fixtures.json` | Added independent four-page sample fixtures and extraction provenance. |
| `host/run_tests.py` | Added temporary-directory host build/test runner. |
| `README.md` | Added architecture, calibration, validation, and separate coordinator proposal. |

Existing `line_follow.c/.h` and `test_lbar.c/.h` are unchanged. Hardware
commissioning, scan-speed qualification, and coordinator mapping integration
remain the next steps before demonstration use.
