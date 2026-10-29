#!/usr/bin/env python3
"""Read-only integration audit for the Buddy 3 barcode scanner.

This does not edit team-owned source or mistake hook presence for hardware
verification. Run from any working directory.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[3]


def check(name: str, ok: bool, detail: str) -> bool:
    print(f"{'PASS' if ok else 'FAIL'} {name}: {detail}")
    return ok


def main() -> int:
    app_main = (ROOT / 'app_program/app_main.c').read_text()
    task_ir = (ROOT / 'app_program/modules/line_barcode/task_ir.c').read_text()
    mk = (ROOT / 'build_make/mtkernel_3/app_program/subdir.mk').read_text()
    fast = (ROOT / 'app_program/modules/barcode_scanning/bc_fast.c').read_text()
    cal = (ROOT / 'app_program/modules/barcode_scanning/bc_cal.h').read_text()
    passed = []

    markers = ['motion_init()', 'line_barcode_init()', 'bc_setup_init()',
               'line_barcode_start()', 'bc_setup_start()']
    positions = [app_main.find(m) for m in markers]
    passed.append(check('startup hooks',
                        all(p >= 0 for p in positions) and
                        positions == sorted(positions),
                        'motion/encoder before barcode setup; IR before worker'))

    markers = ['ir_read(', 'ir_shared_publish(', 'bc_setup_sample(']
    positions = [task_ir.find(m) for m in markers]
    passed.append(check('IR handoff',
                        all(p >= 0 for p in positions) and
                        positions == sorted(positions) and
                        '#if BC_CAL_READY' in task_ir,
                        'only a one-time arm hook; GP27 remains IR-owned'))
    passed.append(check('automatic build discovery',
                        '$(wildcard ../app_program/modules/*)' in mk and
                        '$(filter-out $(APP_TEST_SRCS),$(APP_SRCS))' in mk,
                        'barcode sources included, tests excluded'))
    passed.append(check('4 ms alarm path',
                        'BC_FAST_ALARM_MASK' in fast and
                        'bc_fastq_push(&sample)' in fast,
                        'hardware timer and fixed handoff queue present'))
    passed.append(check('safe calibration gate',
                        re.search(r'^#define BC_CAL_READY 0$', cal, re.M)
                        is not None,
                        'scanner disabled until measured settings are entered'))

    controller = (ROOT / 'app_program/app/task_controller.c').read_text()
    if 'bc_port_peek(' in controller and 'bc_port_ack(' in controller:
        print('INFO mission integration: consumer present; runtime unverified')
    else:
        print('BLOCKED mission integration: no controller result consumer. '
              'Owner must add peek/ACK/STOP clearing; no shared edits made.')

    others = []
    for base in ('app_program', 'kernel', 'lib'):
        for p in (ROOT / base).rglob('*.c'):
            if 'barcode_scanning' in p.parts:
                continue
            if re.search(r'TIMER_ALARM2|TIMER_IRQ_2',
                         p.read_text(errors='replace')):
                others.append(str(p.relative_to(ROOT)))
    if others:
        print('WARNING TIMER ALARM2 references outside scanner: ' +
              ', '.join(others))
    else:
        print('INFO no other TIMER ALARM2 references found in scanned C files; '
              'this does NOT prove the alarm/IRQ is reserved')
    print('INFO real timing, GP27 polarity, wheel calibration, UART console '
          'and START/STOP safety require hardware/team verification.')
    return 0 if all(passed) else 1


if __name__ == '__main__':
    sys.exit(main())
