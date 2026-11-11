#!/usr/bin/env python3
"""V10 host test: packed firmware records, SPSC queue and V9 codec vector."""
import os
import subprocess
import sys
import tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent
CC=os.getenv('CC','gcc')
with tempfile.TemporaryDirectory(prefix='g7-v10-') as temp:
    exe=Path(temp)/'v10_packing'
    cmd=[CC,'-std=c11','-O2','-Wall','-Wextra','-Werror',
         '-I',str(ROOT/'app_program'),'-I',str(ROOT),
         'tools/tests/test_v10_minbits.c',
         'app_program/modules/barcode_scanning/bc_scan.c',
         'app_program/modules/barcode_scanning/bc_fastq.c',
         '-o',str(exe)]
    subprocess.run(cmd,cwd=ROOT,check=True)
    subprocess.run([str(exe)],cwd=ROOT,check=True)
subprocess.run([sys.executable,'tools/test_v9_minbits.py'],cwd=ROOT,check=True)
