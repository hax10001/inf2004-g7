# Barcode scanner: integration status and team-owner handoff

Checked against the current workspace. Barcode implementation stays inside
`app_program/modules/barcode_scanning/`. At the user's later request to
disable motion and IR for hand-roll calibration, one startup gate was added
to `app_main.c`. `BC_CAL_ENCODER_ONLY=1` bypasses all normal robot startup
and runs the motor-off encoder logger. Set it to 0 to restore the startup
hooks documented below. No motion/IR/controller implementation was edited.
The user has now requested normal motion/IR operation again: the current
flag is 0 and barcode READY/console AUTO_ACK are 1. This is an initial
drive-and-decode test profile; navigation mapping is still not connected.

## Module-local integration fixes

- Barcode protection uses one priority-ordered binary semaphore, leaving
  the four configured mutex slots for motion (one), IR (one), and terrain
  (two). Previously, enabling calibrated barcode scanning consumed a fifth
  mutex and caused later terrain initialization to fail. The semaphore
  has no priority inheritance; locked work is bounded and contains no I/O,
  waits or barcode decoding.
- The worker processes each captured sample before feeding the next one.
  A rejection or completion no longer discards the remainder of a buffered
  batch and hides the next leading quiet zone.
- Alarm ownership is checked again when the IR hook first arms capture.
  Failure cleanup leaves an alarm untouched when barcode never armed it.
  Static TIMER ALARM2 reservation is still required.
- The read-only audit accepts explicit measurement/scanning mode flags;
  calibration no longer causes the test runner to fail merely because
  `BC_CAL_READY` was intentionally set to 1.

Register-model tests include an exhausted mutex pool, a rejection followed
by a valid frame in one queued batch, and an alarm claimed between setup
and arming. These checks do not establish physical timing or reliability.

## Already connected (do not reapply previous patches)

- `app_main.c`: `motion_init()` runs before `bc_setup_init()`, and the
  worker is started through `bc_setup_start()` after the IR task.
- `line_barcode/task_ir.c`: reads GP27, calls `bc_setup_sample()` under
  the `#if BC_CAL_READY` guard. This arms the dedicated 4 ms TIMER ALARM2
  scanner **once**, after the IR driver and worker are ready. The line
  task continues its separate 10 ms schedule.
- The existing build Makefile discovers new `modules/*` directories,
  including `barcode_scanning/`, and excludes `test_*.c` from firmware.
- Encoder data comes from Buddy 2's **existing read-only** API.
- Barcode output is **letter only**, never a hardcoded Left/Right/U-turn.

## Still blocked by another owner's code

`app/task_controller.c` does not consume the barcode result. The scanner
may decode and print a letter, but does not execute navigation. The team
integration owner must arrange the following at the correct mission state:

```c
bc_port_result_t result = {0U, 0U, 0U};
if (E_OK == bc_port_peek(&result))
{
    uint8_t code = (uint8_t)(result.info & BC_PORT_LETTER_MASK);
    if ((0U != (result.info & BC_PORT_VALID_FLAG)) &&
        (0U != (result.info & BC_PORT_COMPLETE_FLAG)) &&
        (code < 26U))
    {
        /* Map code via Buddy 1 configuration, when available.
         * Commit the mapped action as pending at Mission Coordinator.
         * Acknowledge only AFTER successful handoff:
         * (void)bc_port_ack(result.sequence);
         */
    }
}
```

This is an **illustrative call contract**, not a patch to the controller.
Do not ACK before a successful handoff, and do not execute a turn directly
from the scanner. On STOP/FAULT, the mission owner must call
`bc_port_clear()` and coordinate stopping/arming the hardware timer via the
approved lifecycle. The current scanner cannot infer mission START/STOP
on its own. START-before-motion safety is still a shared-team issue.

To demonstrate multiple letters **without** a Mission Coordinator consumer,
you may temporarily set `BC_CAL_AUTO_ACK 1` after physical calibration,
which ACKs results after terminal reporting. It sacrifices pending-letter
retention and **must be 0 again before navigation integration**.

## Hardware dependencies

- Reserve RP2040 TIMER ALARM2 / IRQ 2 for Buddy 3.
- Validate 4 ms polling interval and jitter with a real RP2040.
- Calibrate GP27 polarity, both encoder signs/counts, narrow/wide widths,
  ~141 mm printed barcode span, quiet zones and scanning speed.
- Validate realistic tilt, lighting changes, wheel slip and false positives.
- Use a console configuration that does not steal encoder GPIOs.

See `CALIBRATION_GUIDE.md` for the measurement-mode procedure.
Run `python3 app_program/modules/barcode_scanning/integration_audit.py`
for a **read-only** check of the current application's source hookups.
