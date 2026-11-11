# Verification and limits

## Authoritative baseline

The original adjacent V14 source was copied without modifying it. Its
`tools/run_v14_tests_mac.sh` passed on this machine: **74 dashboard tests**,
**144 navigation assertions**, collision latch/ACK, both slope signs plus
disabled-sign behavior, production motion, obstacle/controller integration,
packed fields/precision, C-to-JavaScript vectors and network ownership.

The initially attached older checkout was investigated separately; its older
runner/stub failures and 64-test dashboard are not used as V14 evidence.

## V15 results

The retained V14 acceptance suites and the V15 extensions pass:

- **76/76 dashboard tests**, including reason-code EVENT compatibility and
  existing STATUS lengths. Existing mapping/ACK/reconnect behavior remains.
- **170 navigation assertions**, including all original 144, deadline edges,
  completed-before-deadline/consumed-after-deadline, sequence and encoder/time
  wrap, and preservation of junction origin during result arrival.
- Ten START/STOP cycles with twenty G-to-LEFT junctions through the production
  controller cycle and real navigation core. Assertions check motor requests,
  HOLD/readiness, fault latching, restart gating and real manoeuvre selection.
- Real bc_port semaphore/result lifecycle tests cover independent telemetry ACK,
  idle-before-consume, contention retries, cancellation ownership, repeated
  independent scans, rejection, overflow and sequence zero.
- The production terrain mutex test verifies that a stale collision ACK cannot
  erase a new event, in addition to all V14 latch/reset assertions.
- Existing pitch/governor tests run for signs 0, +1 and -1. Existing production
  motion tests and the new encoder/PI/PWM plant simulation run for both signs.
- Source standard checker, packed field/precision tests, independent IR latches,
  source-ownership checks, obstacle planning/bypass and C/JS golden vectors pass.
- Full Pico SDK 2.2.0 ARM GCC link and UF2 generation pass. The actual ELF contains
  the sole MQTT owner. No SDK internals were edited. [ARM memory evidence](ARM_RAM.md)
  shows **24 additional static SRAM bytes** and **3392 additional .text bytes**
  against the matching V14 reference build.
- Parsed inventory: **535 record members and 240 global/static instances**, from
  **43 production ARM translation units**. Unproven semantic domains remain OPEN.

Full output is in [host-tests.txt](evidence/host-tests.txt). Target section
measurements and build evidence are alongside it. Reproduction commands are in
[BUILD_AND_ROLLBACK.md](BUILD_AND_ROLLBACK.md).

## Mandatory navigation / controller scenarios

| Case | Deterministic evidence |
|---|---|
| First G -> LEFT | Production controller manoeuvre and nav output assertions |
| Second consecutive G -> LEFT | Twenty junctions over ten explicitly authorized runs |
| Independent repeated G cards | Unique result sequences, displacement and repeated scan-port events |
| Different letter replaces G | Retained repeat-mode replacement/nav U-turn tests |
| Result before junction confirmation | Retained and new measured-origin tests |
| Result during wait | Production controller automatically issues LEFT without another START |
| Result near deadline | Exactly at, just before, just after; publication timestamp governs |
| Worker idle before result consumption | Semaphore snapshot and controller-cycle tests |
| Decode failure | Retained bounded single-mode stop and repeat-action fallback |
| Same symbol twice | Same sequence and same-letter/time/displacement suppression |
| Brief IR line change while scanning | Retained gap/line samples and controller scan processing |
| STOP during wait | STOP preempts nav state and clears action while holding zero output |
| STOP during turn | STOP preempts queued/pivot state before next manoeuvre output |
| Fault during scan | Collision/governor preempt controller; late scan cannot restart WAIT/FAULT |
| Restart after STOP | Ten cycles require an explicit later START and fresh readiness |
| STOP during bypass/downhill | Retained bypass preemption, governor reset and driver zero-output checks |
| STOP/START close together | STOP supersedes queued START; pending STOP blocks new START |
| MQTT loss/reconnection | Forced hold and no automatic restart |
| Old/new collision | Sequence compare ACK; genuine new event still stops |
| Calibration unavailable | START rejected; no blind re-zero during STOP |
| Controller mutex contention | Fail-closed stop; no fault acknowledgement on a failed mailbox read |
| Readiness changed after ACK | Execution rechecks sensors/network before mission authorization |

These tests do not prove a physical printed barcode was recognized. They verify
the scanner/controller contract and navigation/motor decisions for validated
decoder events; driven optical recognition remains a car trial.

## Production-loop plant simulation

The simple plant has assumed forward deadband/torque, linear drag, constant
gravity effort and proportional brake authority. Encoder edges are synthesized
from plant wheel speed and fed through the real wheel-speed estimator, PI and
PWM driver. Four cases are repeated for both mounting signs; targets/RPM/PWM and
pitch are checked, including both curve directions and immediate STOP.

| Assumed gravity equilibrium RPM | Pitch magnitude | Settled target RPM | Mean absolute error RPM | Tail peak-to-peak ripple RPM |
|---:|---:|---:|---:|---:|
| 0 | 0 degrees | 15.00 | about 0.001 | 0.004 |
| 3 | 6 degrees | 9.75 | about 0.001 | 0.005 |
| 8 | 10 degrees | 9.75 | about 0.001 | 0.004 |
| 12 | 14 degrees | 9.75 | about 0.001 | 0.004 |

Statistics cover the final ten seconds of each 80-second straight simulation.
Time series include targets, measured RPM, PWM, signed effort, pitch, actual
governor percent/engagement and fault reason in [positive-sign CSV](evidence/plant-sign-1.csv)
and [negative-sign CSV](evidence/plant-sign--1.csv).
The steepest model case needs fractional brake effort; no periodic full-stop
request is used for normal speed correction. Existing runaway tests still
fault after sustained gross overspeed. These inclines/plant parameters are
**not calibrated to the car**. The numbers are software model results only.

## Separate pre-existing failures

The standalone `test_barc.c` suite was executed unchanged against both V14 and
V15. Both report **362 assertions, 1 failure**, with byte-identical output:
`extreme short narrow run still rejected`. The legacy classifier accepts that
synthetic run. No expectation or decoder threshold was changed merely to make
the suite green. See [baseline comparison](evidence/barcode-core-baseline.txt).

The older generic `tools/test_app.py` fixture also remains documented upstream
as incompatible with the barcode port semaphore APIs; it is not substituted
for the maintained V14/V15 dedicated suites. These inherited issues prevent a
claim that *every* test in the tree is green. They are separate from the passing
maintained acceptance suites and new V15 tests.

## Outstanding acceptance

No physical STOP latency, coast-down, ramp stability, live optical scan yield,
collision classification, brake heating, Windows execution, stack high-water
mark, or target SMP timing was measured. The actual nose-down sign remains
unverified and the release retains sign 0. Final Week 13 precision/domain and
protocol-packing decisions remain open. The software release is reviewable and
buildable; **full real-car or minimum-bit certification is not declared complete**.
