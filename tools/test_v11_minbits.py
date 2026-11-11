#!/usr/bin/env python3
"""Run all previous minimum-bit regressions plus V11 semantic boundaries."""
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT=Path(__file__).resolve().parent.parent
subprocess.run([sys.executable,'tools/test_v10_minbits.py'],cwd=ROOT,check=True)
with tempfile.TemporaryDirectory(prefix='g7-v11-packed-') as folder:
    binary=Path(folder)/'v11_boundaries'
    command=[os.getenv('CC','gcc'),'-std=c11','-O2','-Wall','-Wextra',
             '-Werror','-I',str(ROOT/'tools/tests/stubs'),
             '-I',str(ROOT/'app_program'),'-I',str(ROOT/'include'),
             '-I',str(ROOT),
             'tools/tests/test_v11_minbits.c',
             'app_program/modules/telemetry/buddy1.c',
             'app_program/modules/telemetry/buddy1_protocol.c',
             '-o',str(binary)]
    subprocess.run(command,cwd=ROOT,check=True)
    subprocess.run([str(binary)],cwd=ROOT,check=True)
    ir_binary=Path(folder)/'v11_ir_latches'
    ir_command=[os.getenv('CC','gcc'),'-std=c11','-O2','-Wall','-Wextra',
        '-Werror','-I',str(ROOT/'tools/tests/stubs'),'-I',str(ROOT/'app_program'),
        '-I',str(ROOT/'include'),'-I',str(ROOT),
        'tools/tests/test_v11_ir_latches.c',
        'app_program/modules/line_barcode/ir_shared.c', '-o',str(ir_binary)]
    subprocess.run(ir_command,cwd=ROOT,check=True)
    subprocess.run([str(ir_binary)],cwd=ROOT,check=True)
    port_binary=Path(folder)/'v11_barcode_port'
    port_command=[os.getenv('CC','gcc'),'-std=c11','-O2','-Wall','-Wextra',
        '-Werror','-include',str(ROOT/'tools/tests/v11_sem_stub.h'),
        '-I',str(ROOT/'tools/tests/stubs'),'-I',str(ROOT/'app_program'),
        '-I',str(ROOT/'include'),'-I',str(ROOT),
        'tools/tests/test_v11_barcode_port.c',
        'app_program/modules/barcode_scanning/bc_port.c',
        'app_program/modules/barcode_scanning/bc_scan.c',
        'app_program/modules/barcode_scanning/bc_code.c',
        '-o',str(port_binary)]
    subprocess.run(port_command,cwd=ROOT,check=True)
    subprocess.run([str(port_binary)],cwd=ROOT,check=True)
