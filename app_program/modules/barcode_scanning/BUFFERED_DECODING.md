# Experimental buffered analog decoding

This is a source implementation for physical trials, not a validated
accuracy improvement. No build or test execution accompanied the change.
TCRT5000 mounting height and print sizes are not changed by this software.

## Current configuration

- `APP_BARCODE_BUFFERED_ENABLED = 1`: use waveform decoding instead of
  live fixed-threshold capture. Set to 0 for the previous live decoder.
- `BC_CAL_MEASURE = 0`: decoding is enabled instead of calibration-only.
- `APP_MOTOR_MOVEMENT_ENABLED = 0`: motors remain disabled as requested
  during calibration. Enable this explicitly for a driving trial, or
  roll the robot forward by hand so the encoders measure the pass.
- `APP_BARCODE_SCAN_RPM = 10.0f`: slow driving target when buffered mode
  and motor movement are enabled. Line following is retained; this does
  not override steering or assume the track is straight. The legacy
  `MOTION_TARGET_RPM` value is preserved for non-buffered mode.
- Sampling remains 500 Hz. The separate legacy trace remains disabled.
- The fixed ADC threshold 2200 +/- 50 is used only by the comparison
  decoder; waveform segmentation does not use it.

## Collection and processing

Start each trial with the sensor over white at the actual required
mounting height. Keep the same illumination and mounting during a pass.
The first 250 valid ADC samples (about 0.5 s) estimate white mean, noise
standard deviation and range. Wait for `BC WAVE READY` before moving.
An incorrect starting surface produces an incorrect baseline; restart
on white if the reported baseline is wrong.

The recorder maintains about one second of raw lead-in. Collection
triggers when encoder movement has been seen and analog contrast from
the learned white exceeds three noise standard deviations plus eight
ADC counts, in the configured black polarity. It does not require the
fixed black threshold or live run count to trigger. Very weak features
can still fail to trigger; lead-in can retain features preceding a
stronger trigger if they fit in that one-second history.

Collection normally freezes after return toward the learned white baseline
over 400 encoder-sum units (maximum span divided by 12), once at least
the minimum barcode span has been travelled after the trigger. This is
about 32 mm at the trial scale. It exceeds nominal internal white widths
for the configured 3:1 artwork range. A four-sample exponential average
reduces isolated spikes in this end check only; the raw data is unchanged.
This is an end heuristic, not proof of a complete code; decoding checks
still apply. Capacity (8192 rows), 16 seconds and maximum travel remain
fallback limits if the end check never fires. Put one
barcode in each pass and allow white beyond it. A truncated window may
still contain a complete candidate, but no candidate is accepted without
both recorded white zones and all 29 measured widths.

Processing happens in the lower-priority worker after the buffer freezes:

1. Symmetric three-sample mean smoothing, about 6 ms at nominal sampling.
2. Linear-time monotonic-queue lower/upper envelopes in encoder space.
   The radius is the maximum
   configured barcode span divided by 12, currently 400 encoder-sum units.
   Subtract the local lower envelope to reduce spatial baseline variation.
3. Nineteen absolute-offset and seven local-contrast-fraction segmentation
   trials, with noise-dependent hysteresis. These are candidate searches,
   not calibration values proven for the hardware.
4. Search candidate windows for 29 alternating widths, leading/trailing
   white clearance, allowed physical span, and exact star/letter/star
   patterns in either direction. Bars and spaces remain separately
   classified by the existing Code 39 decoder.
5. Rank class consistency using mean relative absolute width deviation.
   `fit_pct` is the average deviation within narrow/wide groups, separately
   by symbol and colour; it is not an accuracy percentage.

One segmentation trial gives at most one vote to a character. Trials
matching multiple different characters do not vote. Acceptance requires
at least three supporting trials, a clear support margin, no second
character supported by two trials, width fit no worse than 30 percent,
no ADC hardware failures and no sample interval above 20 ms. Duplicate
absolute thresholds clamped to the same noise floor do not add votes.
These thresholds are conservative trial heuristics, not validated
confidence probabilities. Searching many segmentations can overfit noise;
the exact delimiters, width checks, ambiguity rejection and comparison
logs reduce that risk but cannot guarantee correct characters.

ADC hardware errors, scheduling-invalid samples, backward encoder
increments and saturation are reported. Forward travel follows the
existing estimate, excluding negative increments; backward increments
therefore indicate degraded distance data even if a match passes.
ADC errors and excessive gaps prevent publishing. Missing samples are
not reconstructed. No speculative letters or invented widths are used.

Storage is about 132 KiB of static RAM, including raw samples and working
arrays. A reused 16 KiB index queue replaces repeated window scans.
Smoothing and each envelope pass process 256 rows per worker poll.
New samples arriving
while processing are counted as `busy_samples`, not silently reused as
another pass. The sampler and other robot tasks continue. Driving is held
at the next motion command after freeze, normally within a controller
period (50 ms), regardless of success. This is intended
for isolated trials; it is not continuous coverage of closely spaced
barcodes. Linker memory fit and hardware timing still need your build
and hardware checks. Do not enable the additional 64 KiB trace concurrently
without reviewing memory usage.

## Logs to send

Keep the terminal connected through `BC WAVE RESULT` and subsequent
`BC WAVE PUBLISH`/`BARCODE Detected` messages, or the unreadable message.
The trial remains DONE with driving held. Reboot on known white to rearm;
automatic white relearning has been removed. Moving/lifting after FROZEN
cannot alter the saved waveform or retrain the baseline.

- `BC WAVE READY`: measured white mean, standard deviation and range.
- `BC WAVE START`: trigger and pass number.
- `BC WAVE FROZEN`: reason 1=capacity, 2=travel, 3=time, 4=white clearance;
  sample count,
  span, worst interval, invalid captures, ADC errors, backward increments
  and near-rail ADC samples. Invalid capture counts include late/queue
  records, not just ADC faults. `capture_ms` measures trigger-to-freeze;
  `tail_quiet_units` is travel since the last above-baseline contrast.
- `BC WAVE LEGACY`: fixed-threshold decoder on the same raw buffer with
  the original two-observation confirmation. Includes pending run count
  and rejection counts for codes 2, 3, 5 and 7. This is an offline
  comparison with the input-validity bypass, not a second hardware pass.
- `BC WAVE TRY`: each segmentation trial. Mode 0 uses an ADC offset;
  mode 1 uses a percentage of local contrast. It reports run count,
  run-buffer overflow, exact candidate count and letter bit mask
  (bit 0=A through bit 25=Z). Excessive-noise trials with over 256 runs
  are rejected rather than truncated into an apparently valid frame.
- `BC WAVE RESULT`: accepted character index (0=A, 25=Z, 31=unreadable),
  best candidate, direction, support, competing support, fit and comparison.
  `process_ms` measures freeze-to-result before the final width log dump.
  Use it to verify speed on hardware; no latency guarantee is measured.
  The candidate is diagnostic only when `valid=0`; fit=65535 indicates
  that no candidate fit was retained.
- `BC WAVE PUBLISH`: status from the existing barcode result queue.
  Status 0 is successful; a busy result slot can prevent publication.
- `BC WAVE WIDTH`: the 29 widths used for an accepted buffered result.

Comparison labels describe agreement, not ground-truth accuracy:

| Label | Meaning on the same saved input |
|---|---|
| `both_same` | Both methods produce the same character |
| `buffered_only` | Buffered accepts; fixed-threshold method does not |
| `legacy_only` | Fixed-threshold accepts; buffered declines |
| `disagree` | Both accept different letters; buffered publication suppressed |
| `neither` | Neither method accepts a character |

Record the actual printed letter, size and orientation for every trial.
Only that ground truth can establish an improvement, regression or false
positive. Test plain white/line-only passes too, not just known barcodes.
The buffered decoder does not silently fall back to legacy-only results;
these are visible regressions to investigate. Accepted buffered results
use the existing result slot and `[BARCODE] Detected` path. The controller's
mission integration is unchanged.

## Post-result waveform dump

`APP_BARCODE_WAVE_DUMP_ENABLED = 1` enables a dump of the existing frozen
buffer, after the result and accepted-width messages. It does not allocate
another sample buffer, change decoding, or print raw samples during capture.
Driving remains held. Set the flag to 0 to disable only this output.

Keep serial connected through `[BC WAVE RAW END]`, not just RESULT or
PUBLISH. Two rows are emitted per worker poll: approximately 5 seconds for
2000 rows or 21 seconds for 8192 rows at the current 5 ms worker period,
plus console/scheduling overhead. Reboot on white to arm the next trial.

```text
[BC WAVE RAW BEGIN] pass=1 rows=... origin_units=... time_valid=1
[BC WAVE RAW COLUMNS] pass,i,dt_us,units,adc,smooth,floor,contrast
[BC WAVE RAW] 1,0,0,0,...
...
[BC WAVE RAW END] pass=1 rows=...
[BC WAVE DONE] Motors held; reboot on white to rearm
```

`i` is chronological sample index. `dt_us` and `units` are relative to the
first retained sample, including the pretrigger lead-in. `adc` is original
AO; `smooth` is polarity-normalized, three-sample averaged ADC; `floor`
is the local lower envelope; `contrast` is local maximum minus minimum.
Thus `smooth - floor` is the baseline-corrected signal used by the trials.
The timestamp reconstruction uses adjacent 16-bit microsecond differences;
`time_valid=0` warns that the recorded maximum gap can exceed that range.
Distances and indices remain useful even then. ADC-error samples were not
stored; the FROZEN summary records missing/invalid input diagnostics.

Send the complete log, including READY, FROZEN, RESULT and RAW BEGIN/END,
and identify the actual printed letter, size and orientation. Ordinary
live ADC lines may appear between rows; only RAW rows belong to the
frozen waveform. Moving or lifting after FROZEN cannot change those rows.
