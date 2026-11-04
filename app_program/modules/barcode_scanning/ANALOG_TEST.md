# Analog barcode trial

## Buffered pass diagnostics

`APP_BARCODE_TRACE_ENABLED = 1` enables one raw trace per boot. Current
threshold, hysteresis, movement and capture rules are not changed by it.
The trace uses 64 KiB of static storage for 4096 samples. The normal build
discovers the new `bc_trace.c` source automatically.

The worker records each dequeued sample BEFORE the two-sample filter.
It retains up to 128 lead-in samples, and starts the pass window on the
first valid black observation after encoder movement has been observed.
It freezes after 1.2 seconds without a raw colour change or at capacity
(roughly eight seconds at the nominal 2 ms sampling period). A capacity
freeze can truncate a longer pass. It does not require 29 widths, so a
scan stalled at 19/29 or 27/29 also produces a trace. No black trigger
after movement means the lead-in keeps rolling and no dump begins.

After freezing, look for:

```text
[BC TRACE SCAN] state=... runs=.../29 error=...
[BC TRACE WIDTH] i=0 units=...
[BC TRACE BEGIN] reason=... rows=... ...
[BC TRACE] dt_us,adc,left,right,level,flags
...
[BC TRACE END] rows=...; reboot to rearm
```

The SCAN/WIDTH snapshot is the live scanner at dump start, not necessarily
at the final buffered sample. It includes widths for an incomplete scan.
Normal FRAME/WIDTHS messages still report earlier terminal captures.
Reason 1 means quiet; reason 2 means full. Row fields contain elapsed
microseconds from the first retained record (including lead-in), raw ADC,
signed encoder counts, pre-filter hysteresis colour, and validity flags.
Flag bit 0 is capture/queue validity; bit 1 is ADC validity. A value of 3
means both were valid, not that all downstream encoder checks passed.
The trace records samples delivered by the queue; it cannot reconstruct
samples lost before delivery. Timestamp gaps and validity bits expose
where available input is incomplete.

The recorder itself does not print. Dumping is paced at one raw row per
worker poll AFTER the buffer freezes. Existing controller and ordinary
barcode logs still run, and the robot continues its normal motion.
Printing can affect later motion/sampling, so use the frozen trace for
the recorded pass, not a second pass during output. Keep serial connected
until TRACE END (allow roughly 30 seconds after the pass). Send the entire
log, including the BEGIN/END block, SCAN/WIDTH snapshot and FRAME messages.
Reboot to arm another pass. Set TRACE_ENABLED to 0 to remove the static
sample buffer and disable this diagnostic. No builds or tests were run.

Connect the sensor module's **AO** output to **GP27 / ADC1**, with a common
ground. AO must remain between **0 and 3.3 V**. Reading DO through the ADC
does not recover the reflected-light signal; change the wire to AO.

The analog settings are in `app_program/app/app_config.h`:

- `APP_BARCODE_ANALOG_ENABLED = 1` selects analog barcode capture.
- `APP_BARCODE_ADC_THRESHOLD = 2048` is an initial, uncalibrated midpoint.
- `APP_BARCODE_ADC_HYSTERESIS = 64` holds the previous colour within
  threshold +/- 64 ADC counts, reducing chatter.
- `APP_BARCODE_ADC_BLACK_HIGH = 1` assumes black produces higher ADC values.
  Set to 0 if measured black values are lower than white values.

GP27's digital input and pulls are disabled in analog mode. The ADC runs
continuously on channel 1; the 2 ms barcode interrupt reads its
latest 12-bit result without waiting. Left/right line inputs remain digital.
The barcode owns ADC conversions; do not open the generic ADC device while
this mode is active, because it can change the shared ADC channel/settings.

During normal scanning, the worker prints once per second:

```text
[BC ADC] raw=... min=... max=... n=... errors=... threshold=2048 hysteresis=64 black_high=1
```

`raw` is the latest AO reading (0..4095). `min` and `max` cover processed
samples since the previous report. `n` and `errors` also reset each report.
If `n=0`, the zero range fields do not represent a measured signal.
`[BC INPUT] raw` remains the classified binary black/white level. Existing
`[BC FRAME]` and `[BC WIDTHS]` messages still report completed/rejected scans.

Hold white under the sensor for several reports, then a narrow black bar,
at the intended mounting height. Record both stable ADC ranges. Choose a
threshold between the ranges, initially the midpoint of their typical
values. Both ranges must cross the appropriate hysteresis boundary:
with black-high polarity, white must fall below threshold - hysteresis,
and black must rise above threshold + hysteresis. Reduce hysteresis only
when the measured contrast requires it and noise remains smaller.

The initial 2048/64 settings are trial values, not automatic calibration.
If the white/black ranges overlap, adjust height/alignment before relying
on decoding. Send the white/black ADC reports and a complete moving scan
to tune these settings. ADC hardware failures are counted and skipped;
they are not replaced with an invented colour.

To restore digital input, set `APP_BARCODE_ANALOG_ENABLED` to 0 and reconnect
DO to GP27. This change has not been compiled or tested on hardware.

## Small-print calibration and edge filtering

Stationary measurements subsequently gave white 147..198, narrow black
222..276, and wide black 2449..2497. The current trial threshold is 210
with hysteresis 6, and black-high polarity. These stationary ranges do
not establish reliable separation during movement.

`APP_BARCODE_EDGE_FILTER_ENABLED = 1` adds one-sample lookahead in the
barcode worker, after raw ADC diagnostics and before capture. A changed
colour needs two matching observations, no more than 8 ms apart. An
accepted transition retains the first observation's encoder counts and
timestamp. Every output sample is delayed equally, preserving chronology.
Unconfirmed pulses retain the previous colour; no synthetic widths are
inserted. ADC hardware errors discard pending confirmation.

`[BC ADC] filtered` counts unconfirmed pulses suppressed since the previous
report, including pending changes not confirmed across a sampling gap.
ADC min/max remain unfiltered; `[BC INPUT]` edges describe filtered input.
Frame/width logging, thresholds, motor enable and exact decoding checks
are unchanged by the filter. The initial observation seeds the starting
colour; confirmation applies to subsequent changes.

Set `APP_BARCODE_EDGE_FILTER_ENABLED` to 0 for an unfiltered comparison.
Very short genuine runs may also be suppressed, so validate both print
sizes during movement. No build or test execution accompanied this edit.

## Retired distance pulse filter experiment

The distance filter described below has been removed from the source,
including its configuration switch and `short_pulses` diagnostic. This
section records the experiment only; it is not the current capture path.
Capture now feeds encoder widths directly after the two-sample filter.

The additional `APP_BARCODE_PULSE_MIN_UNITS` setting is initially 16
encoder-sum units (about 1.3 mm at the trial 12.34 units/mm scale).
This is below the 24-unit minimum in the successful forward F capture,
and above the 5..14-unit fragments in failed flipped captures. It is a
trial noise threshold, not a validated limit for every print or mounting.

After two-sample filtering, the adapter holds a candidate colour change
until it persists over that distance. Returning to the previous colour
before confirmation cancels the candidate and merges that pulse into
the surrounding run. Confirmed changes use the candidate's original
travel position and timestamp. While pending, timeout checks still run
at the candidate position; each sample still updates encoder diagnostics.

`[BC INPUT] short_pulses` is the cumulative count of cancelled distance
candidates; `edges` still counts input transitions before this distance
filter. `[BC ADC] filtered` remains the separate two-sample filter counter.
Threshold 210 +/- 6, movement and exact bidirectional pattern matching
are unchanged. Set `APP_BARCODE_PULSE_MIN_UNITS` to 0 to disable this stage.
Clearing the scanner also clears the filter's pending edge state.

Both orientations need physical trials: genuine pulses shorter than the
confirmation distance will also be suppressed. Existing digital host
fixtures do not exercise this analog-only stage. No build or tests were run.
