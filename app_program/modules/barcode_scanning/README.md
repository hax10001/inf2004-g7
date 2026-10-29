# Buddy 3 isolated barcode scanner (Code 39, one uppercase letter)

Source: Group G7 `improve-movement` at `db69547`. These files were added
without editing any existing team source, build rule or Git history.

## What exists

- `bc_code.[ch]`: 26 `const` nine-bit symbol masks, strict `*A*` to `*Z*`
  decoder, reversible traversal, widths sorted into 20 narrow and 9 wide.
- `bc_scan.[ch]`: leading white, 29 alternating black/white widths, complete
  start/letter/stop arrangement, confirmed *trailing* white distance; bounded
  time/distance/element count and explicit rejection. No wall-clock white end.
- `bc_port.[ch]`: read-only existing encoder API, signed wheel direction
  configuration, per-sample displacement/jitter validation, protected pending
  letter result (8 bytes) and independent four-slot deferred diagnostic queue.
- `bc_uart.[ch]`: `tm_printf` once per event from the deferred low-priority
  consumer. Does not print in the sampling task or interrupt context.
- `bc_fast.[ch]`, `bc_fastq.[ch]`: dedicated RP2040 TIMER ALARM2 4 ms
  sample capture of GP27 and both encoder counts; interrupt only fills a
  fixed 32-entry single-producer/consumer buffer. The lower-priority task
  converts buffered records to scan widths and prints results. A missed
  interrupt, timing excursion or queue overflow invalidates a scan.
- `test_fast.[ch]`: 4 ms deadline, wraparound, overflow and concurrent
  queue tests. Captured encoder readout is read-only.
- `bc_cal.h`, `bc_setup.[ch]`: opt-in calibrated lifecycle. **Disabled by
  default** (`BC_CAL_READY 0`). No motor, GPIO, RTOS, or shared configuration
  is modified. Activation requires teammate approval and real calibration.
- `test_barc.[ch]`, `test_port.[ch]`, `run_tests.sh`: independent tests;
  embedded A/F/Z drawing widths are fixtures, not real sensor recordings.

`barcode_result_t` equivalent: `bc_port_result_t` contains timestamp,
sequence and packed information, **not a navigation action**. No A=Left or
other mapping is hardcoded. The Mission Coordinator owns that mapping.

## Results and error flags

`bc_port_result_t` is 8 bytes (`timestamp_msec:32`, `sequence:16`,
`info:16`). The `info` bits are:

| Bits | Meaning |
|---|---|
| 0–4 | A=0 through Z=25, 26–30 reserved, 31 invalid |
| 5 | VALID |
| 6 | COMPLETE (29 runs plus trailing quiet zone) |
| 7 | REVERSE (observed reverse barcode pattern) |
| 8–9 | quality 0 invalid, 1 reserved, 2 good, 3 strong separation |
| 10–13 | error 0 none, 1 bad input, 2 width, 3 buffer overflow, 4 timeout, 5 distance, 6 interrupted, 7 pattern, 8 pending/busy (reserved) |
| 14–15 | reserved, always zero |

`bc_port_peek` copies the result under a µT-Kernel priority-inheritance mutex.
The accepted event persists until `bc_port_ack(sequence)` or
`bc_port_clear()` at STOP/FAULT. A second completed scan cannot overwrite an
unacknowledged event. `bc_port_diag_pop` consumes diagnostic records separately
so acknowledging a letter does not suppress a queued print. Diagnostic
storage is bounded to four entries; extremely delayed logging could drop
additional diagnostics, and application integration must ensure adequate
service frequency.

The capture state codes IDLE, START_CANDIDATE, CAPTURING, END_CANDIDATE,
COMPLETE and INVALID require three meaningful bits. Sensor input is a single
bit. Individual distance measurements remain 32-bit: reducing them would
risk barcode resolution/overflow.

## Current integration state (from `inf2004-g7(9).zip`)

The uploaded team code **already includes** barcode startup hooks in
`app_main.c` and a GP27 timer-arm hook in
`modules/line_barcode/task_ir.c`. They are preserved unchanged by this
module-only patch. You **do not** need to apply an older integration patch
for these two hooks again.

With `BC_CAL_READY 0`, the hook is compiled out and no barcode alarm is
armed. See `CALIBRATION_GUIDE.md` for the measurement-only mode and the
requirements before enabling actual barcode decoding.

The existing hooks:

1. Run `bc_setup_init()` after encoder-owning `motion_init()`.
2. Call `bc_setup_sample()` after `ir_read()`. Its first valid call arms
   TIMER ALARM2; later 10 ms calls do not feed or print samples. The
   dedicated barcode timer samples GP27 every 4000 microseconds nominally.
3. Start `bc_setup_start()` as a lower-priority decoder/diagnostic worker.

**Still missing:** no application Mission Coordinator consumer calls
`bc_port_peek()` and `bc_port_ack()`, and STOP/FAULT clearing is not
connected. These changes must be reviewed by the shared controller owner.
See `INTEGRATION_HANDOFF.md` for the safe API contract. No motor command
or letter-to-navigation mapping is implemented by this module.

### Calibration required

- Measure which GP27 level is black, sensor mount height/tilt sensitivity,
  and actual left/right encoder sign for forward travel.
- Measure the sum of **signed/normalised left+right encoder increments**
  across a ruler-marked travel distance. One scanner travel unit is one
  increment in that sum, *not* millimetres.
- Measure the narrowest black AND white barcode feature, leading and
  trailing quiet zones. 141 mm is the approximate barcode span from first
  to last bar; the separate 20 mm line gap is lateral, not the quiet zone.
- Set MIN_RUN to an experimentally safe lower bound, LEADING/TRAILING to
  measured quiet-zone limits, and MIN/MAX_BARCODE to an evidence-supported
  tolerance around the observed 141 mm span converted to travel units.
- Set MAX_SAMPLE_UNITS no greater than one-quarter of the measured minimum
  feature distance (conservative initial sampling rule). Set MAX_SAMPLE_MSEC
  from actual worst-case scheduler jitter; exceeding it rejects the scan.
- Set MAX_STEP_DIFFERENCE from ordinary wheel-count skew on straight/tilted
  runs; excess mismatch rejects rather than infers sensor travel.
- Keep the motor speed bounded by the Mission Coordinator. The scanner
  **does not** call `motion_apply` or change motor PWM.

### Original 10 ms polling analysis — provisional only

From the uploaded A/F/Z drawings, a Code 39 `*letter*` barcode has 20 narrow
and 9 wide elements with an approximately 3:1 wide-to-narrow ratio. If the
141 mm barcode span is distributed uniformly across 47 narrow units, the
nominal narrow element is ~3.0 mm. Therefore at 10 ms polling:

- For four samples per narrow element, ideal speed limit =
  3 mm / (4 × 0.010 s) = **75 mm/s**.
- If worst sampling gap is 12 ms, limit reduces to **62.5 mm/s**.
- The team source currently targets 20 RPM with an unverified 65 mm wheel
  diameter, equivalent to ~68 mm/s if achieved. That is about 4.4 ideal
  samples per 3 mm element (3.7 samples with 12 ms gaps): **marginal**.

These are conditional calculations, *not measured scanning speed or timing*.
Yaw, pitch, roll, uneven reflectance, vibration, lift and wheel slip can
shorten effective feature visibility or make encoder travel inaccurate.
Digital IR and wheel encoders alone cannot reconstruct pitch/roll accurately.
The scanner rejects inconsistent widths, wheel disagreement and oversized
per-sample movements, but **cannot guarantee rejection of every misleading
physical signal**. If hardware evidence is insufficient, slow down through
Buddy 2's approved motion API or seek approved finer IR sampling. The Week 6
nominal **2–5 ms** sampling requirement remains unresolved by these files.

## Build/test

From the repository root under Linux, MSYS2 UCRT64 or WSL:

```bash
bash app_program/modules/barcode_scanning/run_tests.sh
```

The tests compile the *actual* new C sources with GCC host stubs for RTOS
mutex, encoder and `tm_printf`. Test code is excluded from normal firmware
unless `TEST=1` is used. No helper source outside this module is modified.

For target firmware (with installed ARM GCC + Make), from `build_make`:

```bash
make SMP=1 CONSOLE=usb_cdc -j4
```

The Make file automatically discovers first-level module folders, including
`modules/barcode_scanning`. `CONSOLE=uart` shares GP0/GP1 with the team's
encoder; confirm a suitable approved USB CDC console setup before scanning.

This is a firmware compilation attempt, not physical validation. Keep drive
motor power off for initial pin/console checks; the existing controller's
START gating still requires team safety work.

## Physical demonstration checklist

1. Verify `BC_CAL_READY` remains zero before all calibration is signed off.
2. With motor power disconnected, validate GP27 black/white polarity and
   real sensor readings at level and tilted positions.
3. Hand-roll the wheels, confirm signs and travel-to-distance conversion.
4. Record actual maximum polling interval/jitter, tilt and edge stability.
5. Only after approval, enter measured values in `bc_cal.h` and
   build/flash the approved system; the startup/IR hooks already exist.
6. Hand-move the robot past A, F, Z in both directions; confirm *one* print
   and pending letter, not any motor command.
7. Test all available A–Z prints, incomplete/dirty barcodes, stalls, wheel
   mismatch, duplicate suppression, STOP/FAULT clearing and low speed.
8. Capture actual RAM, performance and message-size evidence with the final
   integrated build. Do not report host assertions as hardware pass results.

## Dedicated 4 ms barcode sampling — new hardware path

The unchanged shared IR task still runs at **10 ms** for line following.
Once real calibration has been established (`BC_CAL_READY=1`), the existing
`bc_setup_sample` call is a one-time *arming* notification only. The code
reserves **RP2040 TIMER ALARM2 / TIMER_IRQ_2 (external IRQ 2)** via the
µT-Kernel `tk_def_int` API, schedules an alarm every 4000 microseconds from
the existing encoder microsecond timer, and captures GP27 plus coherent
left/right encoder counts and edge timestamps. It does **not** configure
GPIO pins, change motor duty cycles, change global RTOS ticks, or print from
the interrupt. It does not read a second GP27 sample on the shared 10 ms
path.

**Team approval required:** Confirm TIMER ALARM2 and IRQ 2 are reserved for
Buddy 3 and are unused by any other present or future module. The adapter
refuses to arm an already enabled/armed timer alarm, but cannot detect a
handler reserved in software without enabling the alarm. Do not enable
`BC_CAL_READY` until ownership and physical calibration are agreed.

`bc_fastq_sample_t` stores the GPIO level, a 32-bit microsecond timestamp,
both signed encoder counts and both encoder-edge timestamps, plus a validity
flag. `sizeof` depends on the target ABI (expected 24 bytes per record);
32 records bound the buffer. The producer is the timer ISR, the single
consumer is the 10 ms diagnostics task. An aligned 32-bit index and ARM
`DMB` order the record between RP2040 cores, without blocking or RTOS mutex
calls inside an interrupt. The queue is **SPSC**, not suitable for multiple
simultaneous producers or consumers. A dropped sample marks a subsequent
sample invalid; the scan must be rejected rather than interpolated.

The timer maintains a 4 ms deadline phase; if it is over 1000 us late or
misses an interval, it marks the captured record invalid and reschedules
without catch-up bursts. The existing `BC_CAL_MAX_SAMPLE_MSEC` must allow
measured jitter while detecting lost samples; conservative validation and
physical measurements are mandatory. The low-priority task is set to a 10 ms
service interval (subject to kernel scheduling), and any prolonged UART
blocking can cause safe queue-overflow rejection.

The ideal four-samples-per-3-mm-element speed ceiling is
`3 / (4 * 0.004) = 187.5 mm/s`; it is **not** a validated safe speed.
With an observed 5 ms worst-case gap, the bound becomes `150 mm/s`.
Changes in tilt, sensor standoff, yaw, wheel slip, ISR latency and narrowest
physical width may lower these limits. Hardware measurements must set the
actual permitted speed, capture tolerance, and safe rejection criteria.

### Test coverage and limitations

`bash app_program/modules/barcode_scanning/run_tests.sh` runs the existing
A–Z and adapter suites as well as the new queue/scheduler host suite. Host
timing tests check scheduling arithmetic and cross-thread record order; they
do **not** measure real 4 ms IRQ response time, motor jitter or the actual
signal quality while the robot tilts. Tests on the RP2040 should measure
successive `time_usec` differences and record minimum, maximum, jitter,
overflows and invalid scans with motors physically isolated initially.

The 10 ms task and its unchanged code are intentionally not evidence of
barcode 4 ms timing. Actual motion/START safety and Mission Coordinator
consumption of decoded letters remain the team's responsibility.

## Latest module-local integration audit and calibration

See `INTEGRATION_HANDOFF.md` for the precise existing hook status and
mission-controller dependency; see `CALIBRATION_GUIDE.md` for safe on-target
GP27/encoder/4 ms measurements. `integration_audit.py` verifies the hooks
without changing other modules. Rejected events cannot block a later valid
letter, while an unacknowledged valid letter remains protected.
