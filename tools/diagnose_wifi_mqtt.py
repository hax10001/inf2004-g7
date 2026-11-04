#!/usr/bin/env python3
"""Read-only macOS/Pico W Wi-Fi/MQTT diagnostic; never prints credentials."""
import ast
import json
import pathlib
import re
import shutil
import socket
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
WIFI = ROOT / 'config/wifi_credentials.h'
BROKER = ROOT / 'config/wifi_secrets.h'
SECRETS = ROOT / 'app_program/dashboard/.lan-secrets.json'
UF2 = ROOT / 'out/g7-buddy5-mqtt-bench.uf2'


def defs(path):
    if not path.is_file():
        return {}
    content = re.sub(r'\\\n\s*', ' ', path.read_text())
    result = {}
    for key, val in re.findall(r'^\s*#define\s+(\w+)\s+([^\n]*)', content, re.M):
        strings = re.findall(r'"(?:\\.|[^"\\])*"', val)
        if strings:
            try:
                result[key] = ''.join(ast.literal_eval(s) for s in strings)
            except (SyntaxError, ValueError):
                pass
    return result


def run(*args):
    if not shutil.which(args[0]):
        return ''
    try:
        p = subprocess.run(args, text=True, stdout=subprocess.PIPE,
                           stderr=subprocess.DEVNULL, timeout=4, check=False)
        return p.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ''


def report():
    net = defs(WIFI)
    cfg = defs(BROKER)
    if not net:
        print('FAIL: Missing config/wifi_credentials.h; run npm run setup:wifi')
    if not cfg:
        print('FAIL: Missing config/wifi_secrets.h; run npm run setup:wifi')
    ssid = net.get('UTK_WIFI_SSID')
    broker_ip = cfg.get('B1_BROKER_IPV4')
    print('Configured Pico SSID:', ssid or 'MISSING')
    print('Configured MQTT broker:', (broker_ip + ':1883') if broker_ip else 'MISSING')

    if sys.platform == 'darwin':
        iface = 'en0'
        ip = run('ipconfig', 'getifaddr', iface)
        wifi_status = run('networksetup', '-getairportnetwork', iface)
        if 'Current Wi-Fi Network:' in wifi_status:
            current_ssid = wifi_status.split('Current Wi-Fi Network:', 1)[-1].strip()
            print('Mac Wi-Fi SSID:', current_ssid)
            if ssid and current_ssid != ssid:
                print('FAIL: Mac and Pico have different configured SSIDs')
        else:
            print('Mac Wi-Fi SSID: not available from networksetup; check manually')
    else:
        ip = ''
        print('Mac adapter check: macOS only')
    print('Mac en0 IPv4:', ip or 'not detected')
    if broker_ip and ip and broker_ip != ip:
        print('FAIL: Broker IP differs from Mac en0 IP; check whether your Mac moved networks')
    if broker_ip and ip and broker_ip == ip:
        print('PASS: Broker target matches Mac en0 IPv4 (does not prove Pico association)')

    if SECRETS.is_file():
        try:
            host_creds = json.loads(SECRETS.read_text())
            user_match = cfg.get('B1_MQTT_USER') == host_creds.get('mqttUser')
            pwd_match = cfg.get('B1_MQTT_PASSWORD') == host_creds.get('mqttPassword')
            print('MQTT credential match:', 'PASS' if user_match and pwd_match else 'FAIL')
            if not (user_match and pwd_match):
                print('Fix: The Pico configuration and the dashboard use different MQTT credentials.')
        except (OSError, ValueError):
            print('FAIL: Cannot parse dashboard .lan-secrets.json')
    else:
        print('WARN: No dashboard .lan-secrets.json; start npm run lan in this project')

    if broker_ip and (not ip or broker_ip == ip):
        try:
            with socket.create_connection((broker_ip, 1883), timeout=2):
                print('PASS: Mac can open TCP port 1883 (NOT proof of MQTT login or Pico reachability)')
        except (OSError, TimeoutError):
            print('FAIL: Broker port 1883 not reachable from Mac; run npm run lan')
    elif broker_ip:
        print('SKIP: Port test because configured broker is not Mac en0 address')

    if UF2.is_file():
        print('UF2:', str(UF2.relative_to(ROOT)), '(%d bytes)' % UF2.stat().st_size)
        newer = [str(p.relative_to(ROOT)) for p in (WIFI, BROKER)
                 if p.is_file() and p.stat().st_mtime > UF2.stat().st_mtime]
        if newer:
            print('FAIL: WiFi/MQTT settings are NEWER than this UF2:', ', '.join(newer))
            print('Rebuild and reflash; old firmware cannot use updated SSID/password.')
        else:
            print('PASS: Local WiFi/MQTT config is not newer than UF2 (flash still unverified)')
    else:
        print('FAIL: No compiled UF2 in out/; build and verify first')
    print('NOTE: This cannot confirm the firmware flashed onto the Pico or its live Wi-Fi link.')
    print('Next: run bash serial_monitor.sh --list, then bash serial_monitor.sh.')
    print('For logging: set -o pipefail; bash serial_monitor.sh 2>&1 | tee g7-network.log')


if __name__ == '__main__':
    report()
