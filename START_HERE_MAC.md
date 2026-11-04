# G7 Buddy 5 — restore and run on Mac

This ZIP contains the full restored G7 source tree: the newest Wi-Fi retry/MQTT-owner code, dashboard sensors and guarded Start/Stop, build helper, and updated macOS USB serial monitor (automatically reconnects and warns on silent ports).

**Not included:** private Wi-Fi credentials, MQTT login, a compiled UF2, a Pico SDK/toolchain, or actual hardware test results. This is source; you must configure, build, and flash a UF2. Keep the car wheels raised and motor power disconnected for bench testing. Remote STOP is not a physical emergency stop.

## 0. Unzip and open Terminal

Extract the ZIP to `~/Downloads`. Open Terminal:

```bash
cd ~/Downloads/g7-buddy5-live-sensors-control
pwd -P
```

The path should start with `/Users/.../Downloads/`, **not** `.Trash`. Do not delete or move the folder while Terminal is in it.

## 1. Connect Mac to the 2.4 GHz hotspot

Connect the Mac to `Pookie`, and enable compatibility/2.4GHz/WPA2 on the phone. Leave its password enabled. The Pico's Wi-Fi password and MQTT credentials are separate. The Pico W cannot use a 5 GHz-only hotspot. Verify your Mac's current address:

```bash
ipconfig getifaddr en0
```

The previous broker IP was `10.74.40.243`, but use your **current** Mac hotspot IP if it differs. The Mac and Pico must be on the same reachable network. Some phone hotspots isolate clients, preventing connections even after Wi-Fi association.

## 2. One-time Wi-Fi + MQTT configuration

```bash
cd ~/Downloads/g7-buddy5-live-sensors-control
bash setup_pookie.sh
```

Enter the actual hotspot password at the hidden prompt. When prompted for broker IPv4, use your Mac hotspot IPv4; if it's still `10.74.40.243` accept that. The wizard creates private ignored files in `config/` and saves dashboard broker credentials locally. It will intentionally refuse to overwrite existing private headers, so don't rerun unnecessarily.

## 3. Build (does not flash)

```bash
cd ~/Downloads/g7-buddy5-live-sensors-control
set -o pipefail
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh 2>&1 | tee g7-build.log
ls -lh out/g7-buddy5-mqtt-bench.uf2
```

Requires Pico SDK **2.2.0** with submodules and `arm-none-eabi-gcc`. If the build fails, do not flash an old output file; read the last lines of `g7-build.log`. A successful source build does **not** prove the actual Pico can join the hotspot.

## 4. Flash the Pico

Disconnect motor power. Hold Pico **BOOTSEL** while connecting USB, copy `out/g7-buddy5-mqtt-bench.uf2` to `RPI-RP2`. After the drive disappears, reconnect normally **without BOOTSEL**. Never upload/share the generated UF2: it can contain the hotspot/MQTT secrets.

## 5. Start the real dashboard and broker (Terminal A)

```bash
cd ~/Downloads/g7-buddy5-live-sensors-control
bash run_live_dashboard.sh
```

Open `http://localhost:3000`. This runs the live LAN server and local MQTT broker (port 1883), not the synthetic moving car demo. It displays a local dashboard admin token on first run; keep the token private. The firmware can only receive Start/Stop through MQTT if the Pico reaches the Mac, subscribes, receives a valid mapping, and sensor readiness conditions pass. **Do not use `./demo.sh` to test physical connectivity:** it creates simulated data.

## 6. Read Pico USB output while mirroring it to dashboard (Terminal B)

```bash
cd ~/Downloads/g7-buddy5-live-sensors-control
bash serial_monitor.sh --list
bash serial_monitor.sh 2>&1 | tee g7-network.log
```

The script installs `pyserial` into a private `.venv-serial` if needed. It prints the **actual** bytes to Terminal and forwards the lines to the website Monitor using one USB port reader. Keep it running before resetting the Pico normally. When USB is silent, it reports a no-bytes warning and retries on unplug/replug. For a USB-only test with no dashboard, use `bash serial_monitor.sh --no-dashboard 2>&1 | tee g7-usb.log` instead. Stop with Ctrl+C. Avoid another serial app reading the same port simultaneously.

## 7. Diagnostics and what to expect

```bash
cd ~/Downloads/g7-buddy5-live-sensors-control
python3 tools/diagnose_wifi_mqtt.py
lsof -nP -iTCP:1883 -sTCP:LISTEN
```

Observe `[NET]` messages if firmware prints them. `wifi=0` means Wi-Fi joining/initialization is incomplete; `wifi=1 dhcp=0` means no IP yet; `wifi=1 dhcp=1 mqtt=0` means investigate broker reachability/credentials; `wifi=1 dhcp=1 mqtt=1` means MQTT connected but you still need live binary telemetry at `g7/b1/robot/telemetry` to confirm dashboard data. The dashboard's "Build/flash Pico" message is generic until it receives actual live robot telemetry.

If you see `Port OPEN, DTR asserted` but `NO BYTES`, the Pico's USB device enumerates but firmware logging/RTOS task execution has not been confirmed. Do not disable the hotspot password or infer Wi-Fi status from USB silence. Verify stable power, that the Pico is not in BOOTSEL mode, and whether startup logs ever appear after a normal reset.

## Safety and limits

- Live-control build boots held STOPPED; physical Start/Stop is not hardware-validated. Keep wheels raised for first tests and a physical motor cutoff ready.
- MQTT loss should request a local software stop, but packet loss, RTOS hangs or radio loss can delay/defeat software-based commands. Don't depend on the dashboard as an emergency stop.
- 3 IR sensors and IMU readings are wired to firmware telemetry, but real sensor accuracy still requires hardware confirmation.
- Buddy 5 obstacle/ultrasonic control **is not integrated** into the protected motor loop; distance reports unavailable until hardware integration is completed. Avoid free driving.
- Keep phone hotspot password and MQTT broker credentials private. The source ZIP intentionally excludes both.
