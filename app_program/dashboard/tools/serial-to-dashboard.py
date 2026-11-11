#!/usr/bin/env python3
"""One-reader USB CDC/UART monitor: stdout + optional local dashboard mirror.

Reconnects on Pico resets/reflashes, and reports a silent open port instead
of indefinitely appearing to work. This is host-only; it cannot repair a
firmware USB console, Wi-Fi association, or the MQTT connection.
"""
import argparse
import sys
import time
from urllib.request import Request, urlopen


def candidates():
    from serial.tools import list_ports
    ports = list(list_ports.comports())

    def priority(port):
        device = port.device.lower()
        description = (port.description or '').lower()
        pico = (port.vid == 0x2e8a and port.pid == 0x000a) or 'mtk3' in description
        return (0 if pico else 1,
                0 if '/dev/cu.usbmodem' in device else 1,
                0 if 'usbmodem' in device or 'usbserial' in device else 1,
                device)
    return sorted(ports, key=priority)


def choose_port(explicit):
    if explicit != 'auto':
        return explicit
    ports = candidates()
    valid = [p for p in ports if (
        (p.vid == 0x2e8a and p.pid == 0x000a)
        or 'mtk3' in (p.description or '').lower()
        or 'usbmodem' in p.device.lower()
        or 'usbserial' in p.device.lower()
        or p.device.lower().startswith('com'))]
    return valid[0].device if valid else None


def post_line(line, url):
    req = Request(url, data=line.encode('utf-8'), method='POST',
                  headers={'Content-Type': 'text/plain'})
    with urlopen(req, timeout=1) as response:
        return response.status


def print_line(pending, url, dashboard_state, disable_dashboard):
    if not pending:
        return
    line = pending.decode('utf-8', 'replace').strip()[:280]
    pending.clear()
    if not line:
        return
    print(line, flush=True)
    if disable_dashboard:
        return
    try:
        post_line(line, url)
        dashboard_state['unavailable'] = False
    except (OSError, ValueError) as exc:
        if not dashboard_state['unavailable']:
            print('[SERIAL] Dashboard HTTP forwarding unavailable: ' + str(exc),
                  file=sys.stderr, flush=True)
            dashboard_state['unavailable'] = True


def read_serial(reader, args):
    """Return only when disconnected or interrupted; no motor commands sent."""
    opened = time.monotonic()
    last_bytes = opened
    next_warning = opened + args.silence_after
    pending = bytearray()
    dashboard_state = {'unavailable': False}
    while True:
        chunk = reader.read(256)
        now = time.monotonic()
        if chunk:
            last_bytes = now
            next_warning = now + args.silence_after
            for b in chunk:
                if b in (10, 13):
                    print_line(pending, args.url, dashboard_state,
                               args.no_dashboard)
                elif len(pending) < 512:
                    pending.append(b)
                else:
                    pending.clear()  # reject overflowing malformed frame
        elif now >= next_warning:
            quiet = int(now - last_bytes)
            print('[SERIAL] Port open but NO BYTES for {}s. This is NOT a '
                  'Wi-Fi password error; check Pico firmware console and '
                  'reset while monitor is open.'.format(quiet),
                  file=sys.stderr, flush=True)
            next_warning = now + 15


def main():
    parser = argparse.ArgumentParser(description='Pico serial monitor, terminal + dashboard')
    parser.add_argument('port', nargs='?', default='auto')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--wait', type=float, default=12,
                        help='retry interval before printing missing-port warning')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--url', default='http://127.0.0.1:3000/api/serial')
    parser.add_argument('--no-dashboard', action='store_true',
                        help='test raw USB output without HTTP forwarding')
    parser.add_argument('--silence-after', type=float, default=7)
    parser.add_argument('--reconnect-delay', type=float, default=1.5)
    args = parser.parse_args()
    if args.silence_after <= 0 or args.reconnect_delay <= 0 or args.wait <= 0:
        parser.error('timeouts must be positive')
    try:
        import serial
    except ImportError:
        print('pyserial missing: .venv-serial/bin/python -m pip install pyserial',
              file=sys.stderr)
        return 2

    if args.list:
        ports = candidates()
        if not ports:
            print('No serial ports. Use a data cable, normal Pico boot (not BOOTSEL).')
            return 3
        for p in ports:
            print(p.device, p.description, f'VID:PID={p.vid}:{p.pid}')
        return 0

    print('[SERIAL] No password changes needed. USB serial is independent of Wi-Fi.',
          flush=True)
    print('[SERIAL] Keep this running, then reset/reconnect the Pico normally; '
          'Ctrl+C to end.', flush=True)
    print('[SERIAL] One reader shares lines to Terminal and dashboard.', flush=True)
    deadline = time.monotonic() + args.wait
    serial_errors = (serial.SerialException, PermissionError, OSError)
    while True:
        try:
            device = choose_port(args.port)
            if not device:
                if time.monotonic() >= deadline:
                    print('[SERIAL] Waiting for Pico USB CDC port. '
                          'BOOTSEL RPI-RP2 is not a serial port.',
                          file=sys.stderr, flush=True)
                    deadline = time.monotonic() + args.wait
                time.sleep(args.reconnect_delay)
                continue
            deadline = time.monotonic() + args.wait
            print('[SERIAL] Opening {} at {} baud'.format(device, args.baud),
                  flush=True)
            with serial.Serial(device, args.baud, timeout=0.3,
                               write_timeout=1) as reader:
                # Some USB CDC implementations only transmit after DTR.
                # This sets the control line; it does not reset or steer robot.
                reader.dtr = True
                print('[SERIAL] Port OPEN, DTR asserted; waiting for actual '
                      'firmware bytes...', flush=True)
                read_serial(reader, args)
        except KeyboardInterrupt:
            print('\n[SERIAL] Stopped by user.', flush=True)
            return 0
        except serial_errors as exc:
            print('[SERIAL] USB disconnected/read failed: {}. '
                  'Reconnecting automatically...'.format(exc),
                  file=sys.stderr, flush=True)
            time.sleep(args.reconnect_delay)


if __name__ == '__main__':
    sys.exit(main())
