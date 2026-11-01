# Saved small-barcode investigation

No firmware was built or executed. Decoder settings remain unchanged.

The offline replay in `tools/barcode_template_replay.py` reproduces the
logged three-ratio template scores on the small-F and big-F captures:

| Capture | F score | P score | Margin | Accepted |
|---|---:|---:|---:|---|
| Small F, f3c4935f attachment | 292 | 214 | 78 | No |
| Big F, 87698df5 attachment | 4575 | 3465 | 1110 | Yes |

Scores are engineering heuristics, not probabilities. The script uses
the current six anchor levels; these two logs originally used four.
The extra levels did not change these best scores.

## Experiments on the saved data

The finer geometry experiment searches wide/narrow ratios from 2 through
4 in increments of 1/16, instead of only 2.5, 3 and 3.5. It searches every
A-Z template in both orientations, retaining existing eligibility,
margin, support, span and geometry gates. It does not assume the label
provided by the user when ranking candidates.

| Capture | Signal used | Best / runner | Margin | Accepted |
|---|---|---|---:|---|
| Small F | Saved three-sample smoothing | F / P | 120 | No |
| Small F | Original ADC minus saved floor | F / P | 104 | No |
| Small A, 86af475d attachment | Saved smoothing | U / A | 58 | No |
| Small A | Original ADC minus saved floor | U / K | 29 | No |
| Big F | Saved smoothing | F / P | 1159 | Yes |
| Big F | Original ADC minus saved floor | F / P | 1139 | Yes |

The raw variant deliberately preserves the recorded baseline floor to
isolate the effect of removing the three-sample smoothing; it is not a
complete replacement preprocessing algorithm.

For the finer best small-F model, narrow-feature contrasts were
26, 3, 4, -1, 4, 3, 1, 0 and 2 ADC counts. The big-F logged contrasts were
52, 37, 53, 46, 48, 58, 59, 55 and 51 counts. Most small features provide
little separation from neighboring spaces. Broad-bar geometry alone
cannot distinguish all letters: F and P share wide-black-bar positions.

These experiments do not demonstrate a software recovery of small F or
A. They also do not prove that every possible algorithm must fail or
identify a specific physical cause. They rule out the tested ratio-grid
change and removal of this light smoothing as sufficient fixes on these
captures. Lowering acceptance settings until F passes is unjustified:
the A capture ranks another letter higher under both experiments.

No finer-search or raw-scoring change was enabled in firmware because
the saved-data experiment did not establish the intended improvement.
Hardware validation across labels, sizes, orientations and blank input
would still be required for any future decoder change.

Example saved-data analysis:

```text
python tools/barcode_template_replay.py --dense "saved-log.txt"
python tools/barcode_template_replay.py --dense --raw "saved-log.txt"
```

The helper requires a complete single-pass RAW dump. Its printed
`valid` describes template gates only, not the waveform decoder's
additional acquisition-quality gates or publication decisions.
