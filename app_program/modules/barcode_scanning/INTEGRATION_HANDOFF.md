# Barcode scanner: integration status and team-owner handoff

Verified against `inf2004-g7(9).zip` (branch `barcode`). **No source outside
`app_program/modules/barcode_scanning/` was changed in this delivery.**

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
