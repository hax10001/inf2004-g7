#!/usr/bin/env python3
"""Offline smoke checks for the sole lwIP/MQTT owner and build plumbing."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'app_program/modules/telemetry'
OWNER = ROOT / 'lib/libnet/lwip/mqtt_utk.c'
WIFI = ROOT / 'lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c'
MAKE = ROOT / 'build_make/pico_rp2040.mk'

assert OWNER.is_file(), 'Network owner MQTT source missing'
assert not (APP / 'buddy1_mqtt.c').exists(), 'Old app-side MQTT transport still compiled'
assert 'mqtt_utk_poll(' in OWNER.read_text()
assert 'mqtt_utk_poll(' in WIFI.read_text()
assert 'lib/libnet/lwip/mqtt_utk.o' in MAKE.read_text()
for source in APP.glob('*.c'):
    content = source.read_text()
    assert not re.search(r'#include\s+["<]lwip/', content), source.name
    assert not re.search(r'\b(tcp_connect|tcp_write|mqtt_utk_poll)\s*\(', content), source.name
assert 'offer(rxq,' in OWNER.read_text(), 'RX callback must copy command bytes'
assert 'B1_QUEUE_SIZE' in OWNER.read_text(), 'Bounded MQTT buffer absent'
assert 'config/wifi_secrets.h' in (ROOT / '.gitignore').read_text()
print('Network ownership & buffer boundary: PASS')
