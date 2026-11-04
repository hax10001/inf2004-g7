# Experimental analog template matching

No firmware build or test execution accompanied this implementation.
It is an experiment, not a demonstrated accuracy improvement. There is
no special handling for F or for the known small-F example.

## Configuration and integration

`APP_BARCODE_TEMPLATE_ENABLED = 1` adds matching after the existing
buffered segmentation trials. Set to 0 to disable this stage only.
Capture, white learning, driving hold, fixed-threshold comparison and
post-result waveform dumping keep their existing behavior. The matcher
reads the frozen arrays through a callback and does not allocate another
waveform buffer. New static storage is under 2 KiB.

All A-Z patterns come from the existing Code 39 table, surrounded by star
delimiters, and are evaluated in both orientations. It does not assume a
fixed printed size: scale and position are fitted to encoder distances,
subject to the existing minimum/maximum span and quiet-zone constraints.

## Search

1. Extract prominent peaks at six contrast levels: 15, 20, 25, 40, 55 and 70
   percent of the maximum baseline-corrected signal, with a noise floor.
   Peak centroids are weighted by signal and forward travel. More than
   twelve prominent peaks makes that anchor level unusable.
2. Fit the six wide-black-bar positions of each exact star/letter/star
   template to groups of six peaks. Try wide/narrow ratios 2.5, 3 and 3.5,
   plus modest global alignment shifts of +/- 15 percent of narrow width.
   Reject missing quiet zones, wrong spans, contradictory extra strong
   peaks inside the frame, and excessive anchor position residuals.
3. Score the nine predicted narrow black bars against the surrounding
   white-space centers. Use short spatial averages of the smoothed,
   baseline-corrected waveform and a linearly interpolated white baseline.
   This targets weak features that distinguish geometrically similar
   letters, including F and P. Wider white gaps affect the expected narrow
   bar positions and therefore the score.
4. Rank every letter under the same rules. A close runner-up remains
   ambiguous. Even candidates that fail weak-feature eligibility stay
   in the runner-up ranking so a cutoff does not inflate the margin.

The strongest model must have at least six positive narrow-bar contrasts,
including evidence in both star delimiters. Initially, positive means
at least half a measured white-noise standard deviation. Individual
evidence is clipped so one large response cannot dominate all other bars.

Initial acceptance controls in `app_config.h`:

- `APP_BARCODE_TEMPLATE_MIN_SCORE_Z100 = 450`: summed weak evidence floor.
- `APP_BARCODE_TEMPLATE_MIN_MARGIN_Z100 = 300`: winner/runner score margin.
- `APP_BARCODE_TEMPLATE_MIN_SUPPORT = 2`: wins at two anchor levels.

`z100` is 100 times an ADC contrast divided by the initially measured white
noise, with a small geometry penalty. The sum and margin are engineering
scores, not calibrated confidence probabilities or statistical p-values.
Temporal/spatial correlations and many searched models can bias them.
These defaults must be validated with other letters, both orientations,
the full size range and blank/line-only trials, not tuned only to F.

The anchor strategy assumes wide bars produce prominent separated peaks.
If all bars are equally strong or too few peaks survive, this stage can
decline; the existing segmentation decoder remains available on the same
input. It cannot recover truly indistinguishable black and white signals.

## Selection and logs

Search advances by at most two letter/orientation jobs per worker poll. The total
`process_ms` includes matching, so additional latency is visible. No speed
or accuracy measurement has been made on hardware.

- `BC MATCH ANCHORS`: relative percentage, effective threshold, peak count
  and overflow at each level. `duplicate=1` means the noise floor made
  this threshold identical to a previous level; it is not searched or
  counted toward support again.
- `BC MATCH RESULT`: best and runner letters, orientation, scores, margin,
  support, positive-feature count, eligibility and rejection counters.
  A `?` means no scoreable model. An unreadable result may still show a
  best candidate: that letter is diagnostic, not a detection.
- `BC MATCH MODEL`: top two models' fitted offset/scale in Q8, ratio times
  16, and mean anchor-position residual per thousand narrow widths.
  Physical narrow width is `scale_q8 * 16 / 256` encoder-sum units.
- `BC MATCH FEATURE`: expected narrow-bar position, local black/white
  corrected-signal averages, contrast and normalized evidence for the
  best and runner. These are the values to inspect for F/P ambiguity.
- `local_delta` / `local_shift_pct` in each FEATURE line explore nearby
  positions within +/-35 percent of narrow width. These are diagnostics
  only: local maxima do not change scores, alignment or acceptance.
- `BC MATCH COST`: whether integral caching was enabled, mean queries,
  cache hits and input callback reads. Compare `process_ms` across new
  hardware trials to measure the actual speed change.
- `BC WAVE RESULT source=segmentation`: accepted existing width decoding.
- `source=template`: accepted analog template after segmentation declined.
- `source=conflict`: segmentation and templates accepted different letters;
  nothing is published. A disagreement with a valid fixed-threshold
  comparison result also suppresses publication.

Data-quality gates for ADC errors and excessive gaps still apply to both
selection routes. Template results use the existing result slot and
`[BARCODE] Detected` output, with marginal quality. They do not claim 29
individually measured widths: `BC WAVE TEMPLATE` labels that distinction.
`votes` and `fit_pct` in the WAVE line still describe segmentation only;
use MATCH scores/support for template decisions. The fixed-threshold
comparison labels (`both_same`, `buffered_only`, etc.) remain unchanged,
with buffered acceptance now including the template route.

For a trial, start on white, record the actual letter/size/orientation,
and keep serial connected through `BC WAVE RAW END`. Reboot on white to
rearm. A successful fit to the known F alone does not validate the method.

## Potential and runner-up output

`APP_BARCODE_ESTIMATES_ENABLED = 1` reports a serial-only estimate when
the buffered pass has no confirmed detection. Existing confirmed scores,
eligibility, support and acquisition gates remain unchanged. Setting this
flag to zero removes the extra estimate output.

Example format (illustrative scores, not a new hardware measurement):

```text
[BARCODE ESTIMATE] pass=1 potential=F runnerup=P score_z100=292 runnerup_z100=214 margin_z100=78 reverse=0 runnerup_reverse=0 support=0 positive=2 eligible=0 reason=weak_evidence published=0
```

The two letters are the template matcher's highest-scoring distinct
letters, including models that fail confirmed-detection eligibility.
They do not represent segmentation vote rankings. Each letter retains
its own best orientation, which is printed separately. A missing runner
prints `?`; its score field is zero, not measured evidence for a runner.
Scores and margins are heuristic values, not confidence percentages.

An estimate needs a scored geometry fit, a positive total score and at
least one positive narrow-bar feature. It can still be wrong. No estimate
is offered for ADC errors, excessive sample gaps or conflicting accepted
decoders. These cases print `[BARCODE UNREADABLE]` with a reason:
`capture_quality`, `decoder_conflict`, `insufficient_evidence` or
`no_template_fit`. The existing detailed MATCH logs remain available.

Estimates are never passed to `bc_port_publish_buffered`, its result
queue, or the confirmed `[BARCODE] Detected` output. `published=0` makes
that distinction explicit. The ordinary confirmed-detection flow still
runs only after the original checks pass. This adds reporting, not a
small-barcode accuracy improvement. No firmware build or execution was
performed for this change.

## Averaging optimization and saved-data alignment analysis

Spatial averages now use an exact cumulative signal-by-travel integral.
Only every second prefix entry is stored. An odd endpoint needs at most
one interval correction, replacing repeated scans of the averaging window.
This uses the envelope-index workspace after envelope processing has
finished: its union changes active member, and no inactive member is read.
No raw/smoothed/floor/contrast arrays are overwritten. Extra workspace is
four bytes; the matcher retains a direct-sum fallback if the integral
would overflow or the provided workspace is insufficient.

The scoring arithmetic, search ranges and acceptance settings are unchanged.
This is a performance optimization, not evidence of better classification.
Firmware execution, memory fit and actual speed remain unverified.

`tools/barcode_alignment.py` analyzes a saved complete RAW dump and the
logged top-two model parameters using Python's standard library. It exports
waveform CSV, feature CSV and an alignment report. It compares the prefix
calculation with direct sums for each explored window. This is saved-data
analysis, not execution of the C firmware or a project test suite.

```text
python tools/barcode_alignment.py saved-log.txt tmp/barcode_alignment
powershell -File tools/plot_barcode_alignment.ps1 -DataDir tmp/barcode_alignment
```

The plotting helper uses the Windows chart library and writes alignment.png.
Blue/orange bands show the actual top-model sampling windows, including
a close-up of the feature separating F and P when those are the logged
models. Local search maxima can be noise and must not be treated as
verified narrow bars. Use repeated offsets across known star features
before considering an alignment change; do not tune only to a known F.
