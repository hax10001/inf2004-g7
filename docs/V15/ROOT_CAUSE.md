# V15 source findings and corrections

## Barcode/junction handoff

The V14 trace is IR sampling -> fast sample queue -> bc_wave capture/decoder ->
bc_port result -> telemetry_peers poll -> B1 store -> g7_nav_note_decoded ->
controller mailbox -> acknowledged map -> nav_barcode -> junction confirmation
-> centre/pivot -> line reacquisition. Barcode decoding never owns motor PWM.

Three source-proven failure paths exist in V14:

1. The worker's phase can leave DECODING before telemetry forwards its result.
   Navigation treats decoder idle plus a grace interval as evidence that no
   usable result exists. Busy status and result delivery have different owners
   and synchronization. At a newly confirmed junction an idle scanner with a
   result not yet forwarded can cause an immediate no-action fault.
2. g7_nav_note_decoded uses a POL mutex and returns void. If it cannot acquire
   the store mutex, it drops the notification. telemetry_peers still records
   the sequence and acknowledges bc_port, so that valid result is not retried.
   A dashboard success can therefore coexist with no navigation action.
3. g_capture_end_travel was updated using navigation_guard_active, which also
   includes CLEAR and a guard interval. A completed barcode's origin could
   advance towards a later junction, causing a false minimum-clearance fault.

V15 gives the controller its own **12-byte** scan transaction in bc_port, under
the existing semaphore. Enabled/owned distinguish the authorized run from an
old worker. Busy and ready change atomically with completion; telemetry ACK
cannot consume navigation's copy. Failed POL reads leave the result pending
and report pending work conservatively. The controller acquires the existing
map mutex before consuming the scan, so a contention miss cannot lose it.
Capture and decode completion use the controller's kernel millisecond clock.
Decoder publication time decides the deadline, rather than later consumption.
Overflow and unmapped letters fail closed. No fixed delay synchronizes success.

Capture origin advances only while capture_active. Existing junction evidence
survives a newly accepted letter. The event sequence has a separate valid bit,
so sequence zero after wrap is a real event. Encoder deltas and travel use
unsigned modular arithmetic instead of signed-overflow expressions.

Repeat mode retains the validated action after a manoeuvre and across obstacle
bypass. A new mapped letter replaces it; duplicate sequence or near-identical
same-letter/time/displacement readings are suppressed. Decode failure has the
existing bounded repeat fallback. Without a remembered action it stops. Explicit
STOP forgets the action. Late scans in WAIT/FAULT cannot authorize movement.

These are confirmed software defects with deterministic regression coverage.
The supplied physical log lacks synchronized IMU, IR age and fault-source data;
it cannot prove which defect caused that particular physical second-junction
stop. No real-track success is claimed from host tests.

## STOP, readiness and collision identity

V14 already fixed the permanent local collision latch. V15 preserves that fix.
V14 still acknowledged the *current* terrain event on START, so a new impact
between STOP and START could be cleared without being handled. A sequence is
now assigned under the snapshot mutex and STOP acknowledges only the observed
sequence. Intervening events survive. START performs no collision acknowledgement.
Unresolved hardware errors remain latched, including when a collision occurs.

The production controller cycle is directly testable. STOP calls motion_stop_now
before mailbox cancellation, event acknowledgement or other reset work. That
function writes zero PWM first and cancels prior timed brake pulses. Remote
hold/readiness also check the motor owner's reported compares. These are
**command/output observations**, not an encoder-confirmed physical standstill.

Mailbox contention and MQTT loss request a stop without acknowledging a
navigation/hardware fault. Readiness is checked again at START execution. HOLD
reapplies zero, scanner captures are suppressed outside driving, and cancellation
is performed by the barcode worker at its next incremental poll. Ownership is
invalidated immediately, so a cancelled worker cannot contribute to a new run.
STOP resets the governor/PI run state and temporary navigation state, preserving
route mapping, IMU level and barcode white calibration. Explicit live IMU
recalibration truthfully returns NOT_READY; it is not implemented by re-zeroing
the car on an unknown slope. Barcode recalibration requires stopped HOLD and a
rearmable scanner; the existing white-reference checks and operator placement
confirmation remain necessary.

Brief missing IMU samples retain control and the last governor restriction;
prolonged invalid/stale data stops. The sign-disabled configuration also has a
one-second last-valid-sample deadline. Recovery remains explicit STOP then START.

## Ramp control

V14 removed ordinary error-based zero-duty pulsing, but its PI integration gate
was still 15 RPM. The derated straight target is 9.75 RPM, below that gate.
PI therefore lost its integral correction exactly where sustained gravity load
needed it. Feedforward plus proportional correction cannot generally eliminate
that steady error. Forward/coast output also cannot hold a target below the
gravity-driven coasting equilibrium.

V15 retains V14's governor and target scaling, adds conditional anti-windup
integration at ramp targets, and bounds opposing effort to 25% short braking.
Positive effort uses the existing forward polarity. Negative effort applies
equal PWM to both inputs of that motor (brake while high, coast while low),
never a reverse direction command. The two inputs share an aligned PWM slice.
Even a low-target inner wheel can receive braking effort, preserving steering
authority when gravity would otherwise drag it faster than the outer wheel.
The existing 40 RPM/ten-period runaway latch remains; ordinary errors use PI.
Actual brake authority, heating, grip, inertia and safe RPM require measurement.

## Build defect

The V14 scaffold defaults B1_BUDDY5_MODULE to 0 and excludes obstacle C files,
although normal application/controller/telemetry code calls them. Its default
SDK link fails with undefined obstacle symbols. V15's build script explicitly
sets that source-selection option to 1. Reference RAM measurements use the
same option on the separate V14 copy; no original V14 file was changed.

## Fault diagnostics

Existing EVENT kind 21 carries these small reason codes; the dashboard displays
them as SAFE STOP causes and keeps the cause until READY/HOLD or a new boot.
Serial navigation events include state, sequence, capture/decode/controller
times. Logging follows events/state changes; no per-period NAV print was added.

| Code | Cause |
|---|---|
| 1 | Collision |
| 2 | Stale/invalid IR |
| 3 | IR read failure |
| 4 | Clock failure |
| 5 | Obstacle fault |
| 6 | Decode timeout |
| 7 | No usable action / junction clearance |
| 8 | Turn timeout / angle limit |
| 9 | Downhill runaway |
| 10 | IMU unavailable for the allowed interval |
| 11 | Governor stale-IMU fault |
| 12 | Scan handoff overflow |
| 13 | Unmapped barcode |

The existing bounded latest-event report is retained: multiple events in one
controller period may coalesce, and transport loss may lose diagnostics. Safety
decisions do not depend on diagnostic delivery. Controller-owned staged reports
are copied under the remote mutex so temporary contention cannot clear them.
