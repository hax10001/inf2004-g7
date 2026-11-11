# Buddy 5 Pico W: USB-silent / hotspot diagnostics

## What this revision changes

- In `CONSOLE=usb_cdc` builds, `TM_CONSOLE_UART=0` prevents UART0 activity on GP0/GP1, the left encoder pins.
- The kernel USB CDC service sends `[USB SELFTEST]` approximately every five seconds after enumeration, *without* calling `tm_printf` or waiting for the application.
- No Wi-Fi credentials, MQTT broker code, sensor thresholds or motor-control logic changed.
- The build profile version was changed, forcing a clean rebuild of objects when switching to this version.

## Build with hotspot config and controlled bench mode

```bash
cd ~/Downloads/g7-buddy5-live-sensors-control
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" \
bash build_uf2.sh
```

Flash `out/g7-buddy5-mqtt-bench.uf2` with motor power disconnected. Connect Pico with no BOOTSEL, start dashboard (`bash run_live_dashboard.sh`), then run `bash serial_monitor.sh --no-dashboard 2>&1 | tee g7-usb.log` while testing pure USB; to mirror to the dashboard omit `--no-dashboard`.

## Interpret observations

- **`[USB SELFTEST]` appears, but no `[MAIN]` or `[NET]` after 10–15 seconds:** USB is alive; application/telemetry task may not be executing or its normal logging may be blocked. Investigate startup sequencing; hotspot password is not the first issue.
- **`[USB SELFTEST]` and `[NET]` appear:** inspect `wifi`, `init`, `join_req`, `join`, `dhcp`, and `mqtt` to distinguish hotspot association from broker.
- **Neither `[USB SELFTEST]` nor `[MAIN]` appears after resetting with monitor open:** USB CDC service/output not functioning; check correct UF2 flashed, correct USB port, firmware runtime; try a known-good Pico SDK USB CDC program to isolate hardware.
- **`[NET] wifi=1 mqtt=0`:** hotspot link exists; investigate DHCP, MQTT credentials, firewall and host reachability.

Never disable hotspot security as a shortcut. Keep wheels off the ground and motor power disconnected for tests; a remote STOP is not a hardware safety stop. The USB selftest validates the USB service only, **not** a Wi-Fi connection. No physical test has been performed here.
