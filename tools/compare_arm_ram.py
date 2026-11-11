#!/usr/bin/env python3
"""Print comparable static RAM sections for two already-built Pico ELF files.

Does not infer RTOS stack usage or physical memory peaks from ELF sections.
Both ELFs must use the same ARM GCC and build flags for meaningful comparison.
"""
import argparse
from pathlib import Path
import re
import subprocess
import sys


def sections(elf: Path, tool: str):
    raw=subprocess.check_output([tool,'-A',str(elf)],text=True)
    found={}
    for line in raw.splitlines():
        match=re.match(r'^\s*(\.[A-Za-z0-9_.-]+)\s+(\d+)\s+\S+',line)
        if match:found[match.group(1)]=int(match.group(2))
    if not found:raise ValueError('No sections parsed from '+str(elf))
    return found


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('v10_elf',type=Path)
    parser.add_argument('v11_elf',type=Path)
    parser.add_argument('--size-tool',default='arm-none-eabi-size')
    args=parser.parse_args()
    if not args.v10_elf.is_file() or not args.v11_elf.is_file():
        parser.error('Both ELF files must exist. Build each version first.')
    before=sections(args.v10_elf,args.size_tool)
    after=sections(args.v11_elf,args.size_tool)
    print('Section                     V10(bytes)  V11(bytes)  Delta(bytes)')
    for key in sorted(set(before)|set(after)):
        if key.startswith(('.data','.bss','.noinit','.heap')):
            a=before.get(key,0); b=after.get(key,0)
            print(f'{key:<27} {a:>10} {b:>11} {b-a:>13}')
    names=lambda d:sum(n for k,n in d.items() if k.startswith(('.data','.bss','.noinit','.heap')))
    a=names(before);b=names(after)
    print(f'{"Static allocated RAM (approx)":<27} {a:>10} {b:>11} {b-a:>13}')
    print('Not included: runtime task stacks, ISR stack peaks, heap high-water mark,')
    print('resource/latency measurements, or memory represented only at runtime.')
    print('Compare exactly matching toolchain/build flags, then run on-car timing tests.')

if __name__=='__main__':
    try:main()
    except (OSError,ValueError,subprocess.CalledProcessError) as e:
        print('ERROR:',e,file=sys.stderr);sys.exit(1)
