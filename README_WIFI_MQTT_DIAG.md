# G7 Pico W hotspot / MQTT diagnostics

This project has a CYW43-owner MQTT client at `lib/libnet/lwip/mqtt_utk.c`.
Application telemetry uses bounded queues; the broker telemetry topic is
`g7/b1/robot/telemetry` with **binary B1 frames**. The static dashboard
instruction about building/flashing the firmware is **not an error message**.

## What changed

- In the MQTT-enabled profile, the CYW43 owner **joins the configured SSID
  directly**. Previously, it attempted to join only after a startup Wi-Fi scan
  finished; failed/stalled scanning could prevent joining entirely.
- The existing bounded 10-second join attempt and 3-second retry remain.
- `[NET]` serial diagnostics now include radio initialization, scan state,
  join-request result, join result, link state, DHCP state and assigned IP,
  as well as broker subscription state (MQTT=1).
- `tools/diagnose_wifi_mqtt.py` inspects Mac network state, saved credentials
  consistency, the LAN TCP listener, and whether a UF2 exists. **No passwords
  are displayed or sent outside your local setup.**
- No movement logic or physical STOP semantics were changed.

## Run on Mac

From **your current project root** (not an older copy):

```bash
python3 tools/diagnose_wifi_mqtt.py
```

Connect the Mac to hotspot `Pookie`, confirm the Mac's Wi-Fi SSID is `Pookie`
(and not a university Wi-Fi network), then start the LAN dashboard **in this
same folder**:

```bash
cd app_program/dashboard
npm run lan
```

Use a second terminal for the build from the project root:

```bash
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" \
bash build_uf2.sh
```

Only after build success, flash the new `out/g7-buddy5-mqtt-bench.uf2` to the
Pico W with motors disconnected and wheels raised. Do not assume the car has
this firmware until reflashed. Do not share the UF2 because it may include
private Wi-Fi and MQTT credentials.

In a third terminal, run the provided serial bridge with the Pico connected:

```bash
bash serial_monitor.sh | tee g7-network.log
```

A live status line resembles:

```
[NET] wifi=0 mqtt=0 init=0 scan=1 join_req=0 join=-3 link=-3 dhcp=0 ip=0.0.0.0 retry=1
```

These are **illustrative values**, not a measurement of your car. Diagnosis:

| Observed status | Next action |
| --- | --- |
| `init` negative | CYW43 bus/radio init failed. Check board/power. |
| `scan=1`, `join_req` not yet set | New MQTT-enabled direct-join build may not be flashed. |
| `join` negative or `link` negative | Wi-Fi association or authentication failed. Check hotspot compatibility and SSID; `link=-3` is commonly bad authentication (check against your actual SDK constants). |
| `wifi=1`, `dhcp=0` | Wi-Fi associated, DHCP/IP step still failing or incomplete. |
| `wifi=1`, `dhcp=1`, `mqtt=0` | Focus on broker IP, listener, firewall, MQTT username/password, or broker/client protocol. |
| `wifi=1`, `dhcp=1`, `mqtt=1` | MQTT subscription is active. Check that dashboard receives binary telemetry and last packet updates. |

The Mac IP `10.74.40.243` was previously observed, but **verify that it is still
assigned to the Mac after connecting to Pookie**. This script can identify
an IP mismatch. Phone hotspots may isolate clients or change IP addresses.

## Safety and limits

Do not test physically moving wheels until the motor-stop behavior is verified
on a raised chassis with a physical cutoff ready. `G7_LIVE_CONTROL=1` enables
remote administrative commands but is not an emergency-stop system. A network
STOP cannot be delivered when disconnected. This patch is **not** a verified
hardware fix; only on-device `[NET]` logs and received telemetry can prove
Wi-Fi/MQTT success on the Pico.
