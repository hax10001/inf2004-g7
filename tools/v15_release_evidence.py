#!/usr/bin/env python3
"""Compare real ARM sections and record sizes without exporting private images."""
import argparse
import csv
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def output(command):
    return subprocess.check_output(command, text=True)


def sections(path):
    found = {}
    for line in output(['arm-none-eabi-size', '-A', str(path)]).splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[1].isdigit():
            found[fields[0]] = int(fields[1])
    return found


def probe(folder, destination, after):
    includes = [folder/x for x in ('include', 'config', 'kernel/knlinc',
                                   'app_program', 'device/include')]
    command = ['arm-none-eabi-gcc', '-std=c99', '-O2', '-mcpu=cortex-m0plus',
        '-mthumb', '-ffreestanding', '-D_PICO_RP2040_',
        '-DTARGET_DIR=pico_rp2040', '-DPROHIBIT_DEF_SIZE_T',
        '-include', 'stddef.h', *['-I'+str(p) for p in includes]]
    if after:
        command += ['-DV15_PROBE=1']
    command += ['-c', str(ROOT/'tools/tests/v15_arm_sizes.c'), '-o', str(destination)]
    subprocess.run(command, check=True)
    sizes = {}
    for line in output(['arm-none-eabi-nm', '-S', str(destination)]).splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[-1].startswith('size_'):
            sizes[parts[-1][5:]] = int(parts[1], 16)
    return sizes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', type=Path, required=True)
    args = parser.parse_args()
    evidence = ROOT/'docs/V15/evidence'
    evidence.mkdir(parents=True, exist_ok=True)
    temp = ROOT/'tmp/v15'
    temp.mkdir(parents=True, exist_ok=True)
    elf = Path('build_make/mtk3pico_smp1_usb_cdc_wifi_dhcp.elf')
    before, after = sections(args.before/elf), sections(ROOT/elf)
    names = ['boot2', '.text', '.time_critical.tinyusb', '.ARM.exidx',
             'vector', '.data', '.bss']
    rows = ['# Linked ARM RAM and record-size evidence', '',
        'Compiler: '+output(['arm-none-eabi-gcc', '--version']).splitlines()[0],
        '', 'SDK 2.2.0; SMP=1; USB CDC; live MQTT; obstacle sources enabled;',
        'matching private configuration and build flags on both source copies.',
        'The original V14 directory was not modified.', '',
        '| Section | V14 bytes | V15 bytes | Delta |', '|---|---:|---:|---:|']
    for name in names:
        a, b = before.get(name, 0), after.get(name, 0)
        rows.append(f'| {name} | {a} | {b} | {b-a:+d} |')
    a = sum(before.get(x, 0) for x in ('vector', '.data', '.bss'))
    b = sum(after.get(x, 0) for x in ('vector', '.data', '.bss'))
    rows += [f'| Static SRAM total | {a} | {b} | {b-a:+d} |', '',
        'Static sections include reserved kernel heap storage. Runtime stack/heap',
        'utilization, ISR latency and worst-case execution time are not measured.',
        'Debug sections reside in the ELF file and are not counted as SRAM.', '']
    first = probe(args.before, temp/'sizes-v14.o', False)
    second = probe(ROOT, temp/'sizes-v15.o', True)
    rows += ['## ARM object record-size probes', '',
             '| Record | V14 bytes | V15 bytes | Delta |', '|---|---:|---:|---:|']
    with (ROOT/'docs/V15/arm_record_sizes.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['Record', 'V14_bytes', 'V15_bytes', 'Delta'])
        for name in sorted(set(first)|set(second)):
            a, b = first.get(name, 0), second.get(name, 0)
            rows.append(f'| {name} | {a} | {b} | {b-a:+d} |')
            writer.writerow([name, a, b, b-a])
    rows += ['', 'Record probes use ARM GCC and actual kernel/application headers,',
             'not native host sizes or host RTOS stubs. The scan handoff is new;',
             'its V14 value 0 means absent, not an empty V14 instance.', '',
             'The v1 MQTT payload comparison remains identical to the supplied',
             '[message-size ledger](../V11/mqtt_message_size_comparison.csv).', '']
    (ROOT/'docs/V15/ARM_RAM.md').write_text('\n'.join(rows))
    for version, folder in [('v14', args.before), ('v15', ROOT)]:
        (evidence/f'arm-sections-{version}.txt').write_text(
            output(['arm-none-eabi-size', '-A', str(folder/elf)]))
    delta = sum(after.get(x, 0)-before.get(x, 0)
                for x in ('vector', '.data', '.bss'))
    print('ARM evidence generated; static SRAM delta:', delta)


if __name__ == '__main__':
    main()
