#!/usr/bin/env python3
"""Inventory parsed application records/globals using actual ARM build flags.

Type ranges are distinct from proven physical domains. Prior review evidence
is preserved explicitly; OPEN entries are not a compliance certification.
No initializers (including private configuration) are exported.
"""
import argparse
import csv
import json
from pathlib import Path
import re
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def walk(node):
    yield node
    for child in node.get('inner', []):
        yield from walk(child)


def constant(node):
    for child in walk(node):
        if child.get('kind') in ('ConstantExpr', 'IntegerLiteral'):
            if 'value' in child:
                return int(child['value'])
    return None


def owner(path):
    if path.endswith('/scan_handoff.h'):
        return 'bc_port semaphore; worker writes, controller consumes'
    if '/telemetry/' in path:
        return 'B1 store mutex / telemetry owner; inspect individual sharing'
    if path.endswith('/bc_port.c') or path.endswith('/bc_port.h'):
        return 'barcode port semaphore; separate navigation and telemetry ACK'
    if '/terrain/' in path:
        return 'IMU task; published snapshots under terrain mutex'
    if '/line_barcode/' in path:
        return 'IR task / existing IR mutexes'
    if '/obstacle/' in path:
        return 'obstacle owner / packed state semaphore; inspect member'
    if '/encoder' in path or '/bc_fast' in path:
        return 'ISR/task boundary; preserve existing atomic ownership'
    return 'controller-owned or stack-local; inspect shared instance'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-log', type=Path, required=True)
    parser.add_argument('--clang', default='clang')
    parser.add_argument('--arm-include', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=ROOT/'docs/V15')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    prior = {}
    with (ROOT/'docs/V11/application_field_audit.csv').open(newline='') as f:
        for row in csv.DictReader(f):
            prior[(row['Source'], row['Struct'], row['Field'])] = row
    commands = {}
    build_text = re.sub(r'\\\s*\n\s*', ' ', args.build_log.read_text())
    for line in build_text.splitlines():
        if not line.startswith('arm-none-eabi-gcc '):
            continue
        parts = shlex.split(line)[1:]
        if '-c' not in parts or '-o' not in parts:
            continue
        source = parts[-1]
        if '/app_program/' not in source.replace('\\', '/'):
            continue
        flags, skip = [], False
        for item in parts:
            if skip:
                skip = False
            elif item in ('-o', '-MF', '-MT'):
                skip = True
            elif item not in ('-c', '-MMD', '-MP', source):
                flags.append(item)
        commands[source] = flags
    if not commands:
        raise SystemExit('No application compiler recipes found')
    records, globals_ = {}, {}
    for source, flags in sorted(commands.items()):
        command = [args.clang, '--target=arm-none-eabi', '-isystem',
                   str(args.arm_include), *flags, '-Xclang', '-ast-dump=json',
                   '-fsyntax-only', source]
        parsed = subprocess.run(command, cwd=ROOT/'build_make', check=True,
                                capture_output=True, text=True)
        tree = json.loads(parsed.stdout)
        aliases = {}
        for node in walk(tree):
            if node.get('kind') == 'TypedefDecl':
                for child in walk(node):
                    target = child.get('ownedTagDecl', child.get('decl', {}))
                    if target.get('kind') in ('RecordDecl', 'EnumDecl'):
                        aliases[target['id']] = node['name']
        current = str((ROOT/'build_make'/source).resolve())

        def visit(node, inherited=None, in_function=False):
            nonlocal current
            loc = node.get('loc', {})
            loc = loc.get('spellingLoc', loc)
            if 'file' in loc:
                current = str((ROOT/'build_make'/loc['file']).resolve())
            file = inherited or current
            if 'file' in loc:
                file = current
            try:
                rel = str(Path(file).relative_to(ROOT))
            except ValueError:
                rel = ''
            application = rel.startswith('app_program/')
            line = loc.get('line')
            if application and not line and 'offset' in loc:
                content = Path(file).read_text()
                line = content.count('\n', 0, loc['offset'])+1
            kind = node.get('kind')
            if application and 'offset' in loc and kind == 'RecordDecl' and node.get('completeDefinition'):
                name = aliases.get(node['id'], node.get('name') or f'anonymous@{line}')
                for field in node.get('inner', []):
                    if field.get('kind') != 'FieldDecl':
                        continue
                    fname = field.get('name', '(anonymous member)')
                    types = field.get('type', {})
                    spelling = types.get('qualType', '?')
                    canonical = types.get('desugaredQualType', spelling)
                    bits = constant(field) if field.get('isBitfield') else None
                    old = prior.get((rel, name, fname), {})
                    known = old.get('Known_or_required_range', '')
                    rationale = old.get('Rationale_or_remaining_work', '')
                    status = old.get('Verification', 'OPEN_DOMAIN_REVIEW')
                    minimum = old.get('Minimum_bits', '')
                    signed = 'unsigned' not in canonical and (
                        'signed' in canonical or
                        canonical in ('int', 'short', 'long', 'long long'))
                    if bits is not None:
                        capacity = (f'{-(1 << (bits-1))}..{(1 << (bits-1))-1}'
                                    if signed else f'0..{(1 << bits)-1}')
                        if not known:
                            known = capacity+' (representation; domain to review)'
                            minimum = str(bits)
                            status = 'PACKED_DOMAIN_REVIEW'
                    elif '_Bool' in canonical or spelling == 'bool':
                        capacity, minimum, known = '0..1', '1', '0..1'
                        bits, status = 8, 'OPEN_BOOLEAN_STORAGE'
                    elif canonical in ('float', 'double'):
                        bits = 32 if canonical == 'float' else 64
                        capacity = 'IEEE floating point; domain-specific precision'
                        minimum = 'precision proof required'
                        rationale = rationale or 'Retain calculation precision; physical error budget remains open.'
                        status = 'PRECISION_RETAINED'
                    else:
                        match = re.fullmatch(r'(u?int)(8|16|32|64)_t', spelling)
                        if match:
                            bits = int(match[2])
                            signed = match[1] == 'int'
                        elif canonical in ('int', 'unsigned int', 'long', 'unsigned long'):
                            bits, signed = 32, 'unsigned' not in canonical
                        elif canonical in ('char', 'signed char', 'unsigned char'):
                            bits, signed = 8, canonical == 'signed char'
                        if bits:
                            capacity = (f'{-(1 << (bits-1))}..{(1 << (bits-1))-1}'
                                        if signed else f'0..{(1 << bits)-1}')
                            if not known or ('signed -2^31' in known and not signed):
                                known = capacity+'; no tighter proven domain'
                            minimum = minimum or str(bits)+' absent tighter domain'
                        else:
                            capacity = 'array/composite/enum/pointer; see declaration'
                            minimum = minimum or 'member/domain/ABI dependent'
                    if rel.endswith('scan_handoff.h'):
                        status = 'V15_TESTED'
                        if fname == 'letter':
                            known = '0..25 mapped alphabet; 31 invalid; 26..30 reserved'
                        elif fname in ('captured_msec', 'completed_msec'):
                            known, minimum = '0..4294967295 modulo kernel clock', '32'
                        elif fname == 'sequence':
                            known, minimum = '0..65535 modulo; zero valid', '16'
                        rationale = 'Port semaphore protects every read/modify/write; actual port tests cover lifecycle, cancellation and wrap.'
                    if rel.endswith('/task_controller.c') and fname in (
                        'event', 'value', 'sequence', 'fault', 'request_ok',
                        'capture_known', 'imu_seen'):
                        domains = {'event':'0..21', 'value':'0..13',
                            'sequence':'0..65535 modulo', 'fault':'0..13',
                            'request_ok':'0..1', 'capture_known':'0..1',
                            'imu_seen':'0..1'}
                        known, minimum = domains[fname], str(bits)
                        status = 'V15_TESTED'
                        rationale = 'Controller-only staged state; remote mutex protects published copy; no concurrent packed RMW.'
                    if fname == 'collision_sequence':
                        status, minimum = 'V15_TESTED', '32 (modulo event identity)'
                        rationale = 'Snapshot mutex and compare-before-ACK; new events cannot be erased by stale ACK.'
                    if name == 'motion_status_t' and fname.startswith('downhill_'):
                        domains = {'downhill_percent':'0 before init; 65..100',
                            'downhill_engaged':'0..1',
                            'downhill_fault':'0 none; 1 stale; 2 runaway',
                            'downhill_pitch_x10':'-900..900 deci-degrees'}
                        known, minimum = domains[fname], str(bits)
                        status = 'V15_TESTED'
                        rationale = 'Motion status mutex; copies existing bounded governor fields; production plant CSV records actual state.'
                    row = dict(Source=rel, Line=line, Record=name, Field=fname,
                        Declaration=spelling, Canonical_type=canonical,
                        Stored_bits=bits or '', Representation_range=capacity,
                        Known_domain_or_sentinels=known, Minimum_bits=minimum,
                        Ownership=old.get('Owner_or_synchronization', owner(rel)),
                        Status=status, Evidence_or_exception=rationale)
                    records[(rel, line, fname)] = row
            if application and kind == 'VarDecl' and (not in_function or node.get('storageClass') == 'static'):
                globals_[(rel, line, node.get('name'))] = dict(
                    Source=rel, Line=line, Name=node.get('name'),
                    Type=node.get('type', {}).get('qualType', '?'),
                    Storage=node.get('storageClass', 'external'),
                    Ownership=owner(rel),
                    Status='INSTANCE_REVIEW_REQUIRED; see record field table')
            nested_function = in_function or kind == 'FunctionDecl'
            for child in node.get('inner', []):
                visit(child, file if kind != 'TranslationUnitDecl' else None,
                      nested_function)

        visit(tree)
    for name, rows in [('application_fields.csv', records), ('application_globals.csv', globals_)]:
        values = [rows[key] for key in sorted(rows)]
        if not values:
            raise SystemExit('Empty inventory: '+name)
        with (args.output/name).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(values[0]))
            writer.writeheader()
            writer.writerows(values)
    print(f'Parsed {len(commands)} ARM translation units; {len(records)} record fields; {len(globals_)} global/static instances.')
    print('Inventory is source coverage evidence; OPEN items are not certified.')


if __name__ == '__main__':
    main()
