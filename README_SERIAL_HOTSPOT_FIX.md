# Fix empty serial logs and isolate hotspot failures (2 Nov 2026)

## Why g7-network.log was empty

The original `serial_monitor.sh` immediately exited when `pyserial` was absent.
Piping to `tee` did not make a serial connection; it only created an empty file.
The new script **automatically** creates `.venv-serial` and installs `pyserial`
there if needed. No system Python modifications. Installing from PyPI needs
an internet connection for the first run.

Use the **same project folder as the UF2 you most recently built**.
Do not combine the old `g7-buddy5-live-sensors-control` project with a different
folder's dashboard or private configuration without checking their builds.

From project root:

```bash
bash serial_monitor.sh --list
python3 tools/diagnose_wifi_mqtt.py
set -o pipefail
bash serial_monitor.sh 2>&1 | tee g7-network.log
```

`g7-network.log` is a **file in the project root**, not a folder. Run
`ls -lh g7-network.log` to confirm output. Leave Terminal open to stream lines.
On macOS Pico USB CDC should appear as `/dev/cu.usbmodem*`; if none shows,
verify a USB data cable, firmware compiled with `G7_CONSOLE=usb_cdc`, normal
boot (BOOTSEL released), and no competing serial reader. Only one tool should
read the serial port at a time; this bridge prints locally and mirrors to
the dashboard.

## How to distinguish Wi-Fi failures

1. Connect the Mac to the phone's **Pookie** hotspot (2.4 GHz WPA2).
2. Run `python3 tools/diagnose_wifi_mqtt.py` to inspect local SSID, Mac IP,
   listening port 1883, credential consistency and configuration/UF2 age.
   The command does not print the Wi-Fi or MQTT password.
3. Start broker/dashboard with `cd app_program/dashboard && npm run lan`.
4. With Pico USB connected, get `[NET]` serial lines **from the actual flashed
   UF2**. `join` negative means Wi-Fi association issue; `wifi=1 dhcp=0`
   means IP assignment issue; `wifi=1 dhcp=1 mqtt=0` means broker or MQTT
   authentication/reachability issue; `mqtt=1` means subscriber active.
5. If the diagnostics identify local config newer than the UF2, rebuild with
   `G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh`
   then flash the NEW `out/g7-buddy5-mqtt-bench.uf2`.

A correct SSID/password alone does not prove that the Pico joined the hotspot.
Phone hotspots can restrict peer-to-peer traffic; check port 1883 from another
client if Pico associates but MQTT fails. The browser dashboard cannot infer
Wi-Fi join status until it receives telemetry or serial status from the Pico.

**Safety:** Controlled firmware starts stopped. Test on raised wheels with a
physical power cutoff. Dashboard STOP is not a reliable emergency stop when
Wi-Fi is down. Do not expose secrets in logs or share configured UF2 files.
