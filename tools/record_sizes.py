#!/usr/bin/env python3
"""Packed vs unpacked size of every application record in the linked ARM ELF.

Reads the DWARF debug information of the real firmware (arm-none-eabi-objdump),
so sizes are what the RP2040 build actually uses. "Unpacked" re-lays the same
members out with each bit-field widened to its declared type (e.g. a 3-bit
`unsigned int` field becomes a 4-byte unsigned int, `bool x:1` a 1-byte bool)
with natural alignment: the representation without minimum-bit packing.

Usage: tools/record_sizes.py ELF docs/min-bits/application_fields.csv OUT.csv
"""
import csv
import re
import subprocess
import sys

die_re = re.compile(r'^\s*<(\d+)><([0-9a-f]+)>: Abbrev Number: \d+ \((DW_TAG_\w+)\)')
attr_re = re.compile(r'^\s*<[0-9a-f]+>\s+(DW_AT_\w+)\s*:\s*(.*)$')


def parse(elf):
    text = subprocess.check_output(['arm-none-eabi-objdump', '--dwarf=info', elf],
                                   text=True, errors='replace')
    dies, stack = {}, []
    current = None
    for line in text.splitlines():
        m = die_re.match(line)
        if m:
            depth, off, tag = int(m[1]), int(m[2], 16), m[3]
            current = dict(tag=tag, attrs={}, children=[], off=off)
            dies[off] = current
            del stack[depth:]
            if depth and len(stack) >= depth:
                stack[depth-1]['children'].append(current)
            stack.append(current)
            continue
        m = attr_re.match(line)
        if m and current is not None:
            current['attrs'][m[1]] = m[2].strip()
    return dies


def ref(value):
    m = re.search(r'<0x([0-9a-f]+)>', value)
    return int(m[1], 16) if m else None


def name(d):
    v = d['attrs'].get('DW_AT_name', '')
    return v.split(': ')[-1].strip() if v else ''


def resolve(dies, off):
    d = dies.get(off)
    while d and d['tag'] in ('DW_TAG_typedef', 'DW_TAG_const_type',
                             'DW_TAG_volatile_type'):
        d = dies.get(ref(d['attrs'].get('DW_AT_type', '')))
    return d


def size_align(dies, off):
    d = resolve(dies, off)
    if d is None:
        return 4, 4
    if d['tag'] == 'DW_TAG_array_type':
        es, ea = size_align(dies, ref(d['attrs']['DW_AT_type']))
        n = 1
        for c in d['children']:
            if 'DW_AT_upper_bound' in c['attrs']:
                n *= int(c['attrs']['DW_AT_upper_bound'], 0) + 1
            elif 'DW_AT_count' in c['attrs']:
                n *= int(c['attrs']['DW_AT_count'], 0)
        return es*n, ea
    size = int(d['attrs'].get('DW_AT_byte_size', '4'), 0)
    if d['tag'] in ('DW_TAG_structure_type', 'DW_TAG_union_type'):
        align = 1
        for c in d['children']:
            if c['tag'] == 'DW_TAG_member':
                align = max(align, size_align(dies, ref(c['attrs']['DW_AT_type']))[1])
        return size, align
    return size, min(size, 8) or 1


def unpacked(dies, struct):
    off, align = 0, 1
    for c in struct['children']:
        if c['tag'] != 'DW_TAG_member':
            continue
        s, a = size_align(dies, ref(c['attrs']['DW_AT_type']))
        off = (off + a - 1)//a*a + s
        align = max(align, a)
    return (off + align - 1)//align*align


def main():
    elf, fields_csv, out = sys.argv[1:4]
    wanted = {r['Record'] for r in csv.DictReader(open(fields_csv))}
    dies = parse(elf)
    found = {}
    for d in dies.values():
        target = None
        if d['tag'] == 'DW_TAG_typedef' and name(d) in wanted:
            target = resolve(dies, d['off'])
        elif d['tag'] == 'DW_TAG_structure_type' and name(d) in wanted:
            target = d
        if target is None or target['tag'] != 'DW_TAG_structure_type':
            continue
        if 'DW_AT_byte_size' not in target['attrs']:
            continue
        packed = int(target['attrs']['DW_AT_byte_size'], 0)
        bits = sum(1 for c in target['children'] if 'DW_AT_bit_size' in c['attrs'])
        found[name(d)] = (packed, unpacked(dies, target), bits)
    rows = sorted(found.items())
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['Record', 'Bit_fields', 'Unpacked_bytes', 'Packed_bytes',
                    'Saved_bytes'])
        for k, (p, u, b) in rows:
            w.writerow([k, b, u, p, u - p])
    tp = sum(v[0] for v in found.values()); tu = sum(v[1] for v in found.values())
    print(f'{len(rows)} records: unpacked {tu} B, packed {tp} B, saved {tu-tp} B')


if __name__ == '__main__':
    main()
