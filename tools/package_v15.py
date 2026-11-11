#!/usr/bin/env python3
"""Export source/evidence, excluding local secrets, firmware and intermediates.

Firmware UF2/ELF files are intentionally not distributed: local builds embed
private Wi-Fi/MQTT configuration. Configure and build on the receiving machine.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]
NAME = 'inf2004-g7-v15-navigation-recovery-ramp'
SKIP_DIRS = {'tmp', 'out', 'node_modules', 'recordings', '__pycache__',
             '.git', '.vscode', 'releases'}
SKIP_SUFFIXES = {'.o', '.d', '.elf', '.uf2', '.map', '.a', '.exe', '.pyc',
                 '.log', '.jsonl', '.orig', '.rej'}


def eligible(path):
    rel = path.relative_to(ROOT)
    if any(part in SKIP_DIRS for part in rel.parts):
        return False
    if rel.parts[:2] == ('build_make', 'mtkernel_3'):
        return False
    name = path.name.lower()
    if name in ('.ds_store', '.build_profile', '.g7_firmware_sources.sha256'):
        return False
    if path.suffix.lower() in SKIP_SUFFIXES:
        return False
    if name.endswith('.zip') and name != 'g7_make_scaffold.zip':
        return False
    if '.example' not in name:
        if 'secret' in name or 'credential' in name or 'broker_local' in name:
            return False
        if name == '.env' or name == 'network_config.h':
            return False
    if path.is_symlink():
        return False
    return True


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', type=Path, required=True)
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--refresh', action='store_true',
                        help='Replace only an unchanged generated release')
    args = parser.parse_args()
    destination = args.destination.resolve()
    if destination.exists():
        if not args.refresh:
            raise SystemExit('Refusing to replace an existing release directory')
        marker = destination/'release_manifest.json'
        previous = json.loads(marker.read_text())
        expected = previous.get('files', {})
        actual = {str(p.relative_to(destination)):p
                  for p in destination.rglob('*') if p.is_file() and p != marker}
        if previous.get('name') != NAME or set(actual) != set(expected):
            raise SystemExit('Release has untracked files; refusing refresh')
        if any(digest(actual[name]) != value for name, value in expected.items()):
            raise SystemExit('Release was modified; refusing refresh')
        archive = destination.with_suffix('.zip')
        checksum = archive.with_suffix('.zip.sha256')
        if digest(archive) != checksum.read_text().split()[0]:
            raise SystemExit('Archive was modified; refusing refresh')
        shutil.rmtree(destination)
        archive.unlink()
        checksum.unlink()
    if destination == ROOT or ROOT in destination.parents:
        raise SystemExit('Export must be outside the source tree')
    selected = [path for path in sorted(ROOT.rglob('*'))
                if path.is_file() and eligible(path)]
    private_values = set()
    for name in ('wifi_credentials.h', 'wifi_secrets.h'):
        local = ROOT/'config'/name
        if local.is_file():
            for macro, value in re.findall(
                r'^\s*#define\s+(\w+)\s+"([^"\n]+)"',
                local.read_text(), re.M):
                if any(key in macro for key in ('PASSWORD', 'SSID', 'MQTT_USER')):
                    if len(value) >= 8 and value not in ('password', 'replace-with-password', 'replace-with-ssid'):
                        private_values.add(value.encode())
    for path in selected:
        data = path.read_bytes()
        if any(value in data for value in private_values):
            raise SystemExit('Private configured literal found in '+
                             str(path.relative_to(ROOT)))
    changed = []
    for path in selected:
        rel = path.relative_to(ROOT)
        previous = args.before/rel
        if not previous.is_file() or digest(path) != digest(previous):
            changed.append((str(rel), 'changed' if previous.is_file() else 'new'))
    report = ROOT/'docs/V15/CHANGED_FILES.md'
    report.write_text('# Changes relative to V14\n\n'
        'Private/generated inputs are excluded from this comparison.\n\n'
        '| File | Change |\n|---|---|\n'+''.join(
        f'| [{name}](../../{name}) | {kind} |\n' for name, kind in changed))
    if report not in selected:
        selected.append(report)
    destination.mkdir(parents=True)
    for path in selected:
        target = destination/path.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
    manifest = dict(name=NAME, baseline='inf2004-g7-v14-recovery',
        status='implemented; maintained host suites and target build pass; physical acceptance pending',
        dashboard_tests=76, navigation_assertions=170,
        known_preexisting_barcode_core_failures=1,
        firmware_binary_included=False,
        firmware_binary_reason='Local firmware embeds private credentials; build from the included source.',
        minimum_bit_certification='partial; explicit OPEN entries and physical/assessor decisions remain',
        files={str(p.relative_to(destination)):digest(p)
               for p in sorted(destination.rglob('*')) if p.is_file()})
    (destination/'release_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    archive = destination.with_suffix('.zip')
    if archive.exists():
        raise SystemExit('Refusing to replace an existing ZIP')
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for path in sorted(destination.rglob('*')):
            if path.is_file():
                z.write(path, NAME+'/'+str(path.relative_to(destination)))
    with zipfile.ZipFile(archive) as z:
        if z.testzip() is not None:
            raise SystemExit('ZIP integrity check failed')
    checksum = digest(archive)
    archive.with_suffix('.zip.sha256').write_text(checksum+'  '+archive.name+'\n')
    print(f'Exported {len(manifest["files"])} files and a manifest.')
    print(f'ZIP: {archive}')
    print(f'SHA256: {checksum}')


if __name__ == '__main__':
    main()
