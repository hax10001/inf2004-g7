#!/usr/bin/env python3
"""Host-compiler C bit packing plus cross-language C-to-JS golden vector."""
import os
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = 'app_program/modules/telemetry/'
with tempfile.TemporaryDirectory(prefix='g7-v9-minbits-') as directory:
    binary = Path(directory) / 'v9_minbits'
    command = [os.environ.get('CC', 'gcc'), '-std=c11', '-O2', '-Wall',
               '-Wextra', '-Werror', '-I', str(ROOT / SRC),
               'tools/tests/test_v9_minbits.c', SRC + 'buddy1_protocol.c',
               SRC + 'buddy1.c', '-o', str(binary)]
    subprocess.run(command, cwd=ROOT, check=True)
    output = subprocess.check_output([str(binary)], cwd=ROOT, text=True)
    print(output, end='')
    values = dict(line.split('=', 1) for line in output.splitlines() if '=' in line)
    js = """
    const p=require('./app_program/dashboard/server/protocol');
    const b=Buffer.from(process.argv[1],'hex');
    const d=p.snapshot(b);
    if(d.mission!==7||d.barcode!=='Z'||d.speedLeft!==-2047||
       d.speedRight!==2047||d.faults!==0xa5||d.side!==2)
      throw Error('C/JS snapshot mismatch: '+JSON.stringify(d));
    const f=p.decodeFrame(Buffer.from(process.argv[2],'hex'));
    if(f.seq!==77||f.session!==313||p.interpret(f).barcode!=='Z')
      throw Error('C/JS frame mismatch');
    console.log('PASS C-to-JS snapshot/frame golden vector');
    """
    subprocess.run(['node', '-e', js, values['snapshot_hex'],
                    values['frame_hex']], cwd=ROOT, check=True)
