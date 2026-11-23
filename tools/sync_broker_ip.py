#!/usr/bin/env python3
"""Synchronize ONLY the Pico MQTT broker IPv4 to the current Mac Wi-Fi IPv4.

This does not edit hotspot SSID/password, MQTT login or server credentials.
Run after joining the same Wi-Fi hotspot as the Pico and before rebuilding.
"""
import argparse
import datetime
import ipaddress
import pathlib
import re
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
CANONICAL = ROOT / 'config' / 'wifi_secrets.h'
LEGACY = ROOT / 'app_program' / 'modules' / 'telemetry' / 'buddy1_broker_local.h'
PAT = re.compile(r'(?m)^(\s*#define\s+B1_BROKER_IPV4\s+)"(\d{1,3}(?:\.\d{1,3}){3})"(\s*(?:/\*[^\n]*\*/)?\s*)$')

def local_ip():
    if sys.platform != 'darwin':
        raise RuntimeError('This helper checks macOS en0 only.')
    output = subprocess.check_output(['ipconfig', 'getifaddr', 'en0'], text=True, stderr=subprocess.DEVNULL).strip()
    ip = ipaddress.IPv4Address(output)
    if not ip.is_private or ip.is_loopback or ip.is_link_local:
        raise RuntimeError(f'Mac en0 does not have an expected LAN address: {ip}')
    return str(ip)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='Back up and update broker IP only')
    args = parser.parse_args()
    target = CANONICAL if CANONICAL.is_file() else LEGACY
    if not target.is_file():
        raise SystemExit('Missing broker header. Use npm run setup:wifi first.')
    src = target.read_text()
    matches = list(PAT.finditer(src))
    if len(matches) != 1:
        raise SystemExit('Could not safely identify exactly one B1_BROKER_IPV4 macro. No edits made.')
    old = matches[0].group(2)
    try:
        new = local_ip()
    except (OSError, ValueError, subprocess.CalledProcessError, RuntimeError) as exc:
        raise SystemExit(f'Cannot determine Mac hotspot IPv4: {exc}')
    print(f'Mac en0: {new}')
    print(f'Compiled Pico broker target in {target.relative_to(ROOT)}: {old}')
    if old == new:
        print('MATCH: broker target already uses current Mac IP. No change.')
        return
    print('MISMATCH: Pico will not find the broker at the previous hotspot address.')
    if not args.apply:
        print('Connect Mac to the SAME hotspot as the Pico and rerun with --apply.')
        return
    backup_dir = ROOT / '.g7-wifi-private-backups'
    backup_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    timestamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    backup = backup_dir / (target.name + '.' + timestamp + '.bak')
    shutil.copy2(target, backup)
    backup.chmod(0o600)
    updated = PAT.sub(lambda m: m.group(1) + '"' + new + '"' + m.group(3), src, count=1)
    target.write_text(updated)
    target.chmod(0o600)
    print(f'UPDATED: {target.relative_to(ROOT)}. Private backup: {backup.relative_to(ROOT)}')
    print('SSID, Wi-Fi password, MQTT username/password and dashboard token UNCHANGED.')
    print('NEXT: rebuild UF2 and reflash the Pico; IP is compiled into firmware.')

if __name__ == '__main__':
    main()
