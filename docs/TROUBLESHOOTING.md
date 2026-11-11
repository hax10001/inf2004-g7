# Troubleshooting

Work through the layers in order: build → USB serial → Wi-Fi → IP (DHCP) →
MQTT → dashboard. Each layer depends on the one before it. The dashboard's
"build/flash the Pico" message is a generic instruction shown until telemetry
arrives. It is not an error report.

Start with:

```sh
python3 tools/diagnose_wifi_mqtt.py
```

It reports the configured SSID and broker, the Mac's `en0` address and whether
the two match, whether the MQTT login matches the dashboard's
`.lan-secrets.json`, whether port 1883 is open locally, and whether a UF2
exists in `out/`. It never prints passwords. None of these checks prove the Pico
itself is connected; only its serial output and received telemetry do.

## Reading the Pico's status lines

With a `G7_CONSOLE=usb_cdc` build, `bash serial_monitor.sh` shows:

- `[USB SELFTEST G7DIAG-...] CDC alive` about every 5 s. This comes from the
  USB service itself, so it shows USB works even if the application is stuck.
- `[BOOTNET] stage=... init=... join_req=... join_rc=... wifi=... dhcp=... dhcp_rc=... mqtt=... ip=...`:
  radio owner state, printed by the USB service.
- `[NET] wifi=... mqtt=... init=... scan=... join_req=... join=... link=... dhcp=... ip=... retry=...`:
  printed by the telemetry task every 5 s and on every MQTT change.

| What you see | Meaning | What to do |
|---|---|---|
| `init` negative | CYW43 radio failed to initialise | Check board and power |
| `join`/`link` negative | Association or authentication failed (`-2` is no network; `-3` is commonly bad authentication) | Check the SSID and password, 2.4 GHz/WPA2 compatibility ("Maximise compatibility" on iPhone/OPPO hotspots). A join attempt times out after 10 s and retries every 3 s |
| `wifi=1 dhcp=0 ip=0.0.0.0`, `dhcp_rc=-5` | Joined, but still waiting for an IP address (`-5` = in progress) | Wait. DHCP gives up after 30 s |
| `dhcp_rc=-3` | DHCP timed out | The hotspot isn't giving out an address: check its device limit, restart it, and turn off client isolation |
| `wifi=1 dhcp=1 mqtt=0` | Has an IP, can't reach or log in to the broker | Check that the broker IP compiled into the firmware matches the Mac (run `tools/sync_broker_ip.py`, then rebuild and reflash), that the dashboard is running in a LAN mode, and that the macOS firewall allows incoming connections for `node` on 1883 |
| `mqtt=1` but no dashboard data | Subscribed | Check that the dashboard receives packets on `g7/b1/robot/telemetry`; restart it if it was started before the Pico |

After the first failed join, the radio runs one Wi-Fi scan per boot. The USB
service prints `[WIFI SCAN] rc=… total=… shown=… Pookie_found=…`, followed by up
to 8 `[WIFI AP] ssid=… ch=… rssi=… auth=…` lines, and repeats them about every
30 s. `Pookie_found` means **the configured SSID** was seen, whatever its name.
If it is 0 while other 2.4 GHz networks appear, check the SSID bytes and
channel. If it is 1 but the join still fails, look at security and
compatibility. The radio country is Singapore by default (`G7_WIFI_COUNTRY`
in `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`; there is no build-script
option for it).

**Never disable the hotspot password as a shortcut.** A correct SSID and
password alone do not prove the Pico joined.

## The broker IP changed

Phone hotspots often move to a new subnet when they reconnect. The broker IP
is compiled into the firmware (`B1_BROKER_IPV4` in `config/wifi_secrets.h`),
so the Pico keeps trying the old address. Run
`python3 tools/sync_broker_ip.py --apply` while on the same hotspot as the car,
then rebuild and reflash. Run `ipconfig getifaddr en0` first: if the address
hasn't changed since your last build, no reflash is needed.

## USB serial problems

- **No port found:** use a data cable. The firmware must be a `usb_cdc` build,
  and the Pico must not be in BOOTSEL mode (`RPI-RP2` is a storage device, not
  a console). On macOS look for `/dev/cu.usbmodem*` or `/dev/tty.usbmodem*`.
- **Port opens but stays silent:** reset the Pico with the monitor already
  open. If even `[USB SELFTEST]` never appears, the USB service isn't running;
  check that the correct UF2 is flashed. If `[USB SELFTEST]` appears but
  `[MAIN]`/`[NET]` never do, the application isn't starting. Wi-Fi is not the
  first suspect.
- **"Port busy":** close any other reader (Arduino IDE, `screen`, VS Code Serial
  Monitor). Only one program can read the port.
- **pyserial errors:** `serial_monitor.sh` installs pyserial into `.venv-serial/`
  (it needs internet the first time). Set `SERIAL_PYTHON` to use another
  interpreter that already has it.
- A `tee` log stays empty if the monitor exits. Use `set -o pipefail` to see
  the failure.

## Build failures

| Symptom | Cause and fix |
|---|---|
| `Incompatible Pico SDK ... (version 2.x.y)` or `pico/util/fixed_bitset.h: No such file` | The SDK isn't 2.2.0. Install 2.2.0 separately (see the README) and pass `PICO_SDK_PATH`. An exported `PICO_SDK_PATH` overrides the automatic search. Don't patch the newer SDK |
| `missing CYW43 or lwIP submodules` | `git -C "$PICO_SDK_PATH" submodule update --init --recursive` |
| `Local Wi-Fi/broker config missing` | Run `npm run setup:wifi` (README step 1) |
| `tools/check_app_standard.py` violations | Application code broke the coding standard. Fix the code; never disable the checker |
| `.d: No such file or directory` under `build_make/.../usb/` | Old parallel-make race. The script now pre-creates these directories and builds `usb_cdc` sequentially. Rerun it rather than calling `make` directly |
| `linked firmware ELF lacks G7DIAG-...` (exit 6) | Stale USB objects were reused. Delete `build_make/mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/usb_console.o` and rebuild. **Don't flash** |
| `MQTT owner poll was not linked` (exit 7) | `mqtt_utk_poll` isn't in the image, so the car could never connect. **Don't flash** |
| `undefined reference to __atomic_*` | Cortex-M0+ has no atomic read-modify-write instructions. Use the kernel mutex pattern in `task_controller.c`, not `-latomic` |

Switching the SDK path or the build profile invalidates old objects
automatically (`build_make/.build_profile`). When reporting a failure, send the
first `error:` line with about 10 lines of context, and remove secrets and
local paths first.

## Dashboard problems

| Symptom | Fix |
|---|---|
| `EADDRINUSE ... :3000` (or `:1883`) | Another process holds the port. Find it with `lsof -nP -iTCP:3000 -sTCP:LISTEN`, then stop it or run with `WEB_PORT=3001` |
| HTTP **409** on mapping/START | Started read-only (`npm run lan` / `npm start`). Restart with `bash run_live_dashboard.sh` |
| HTTP **401** | Remote browser without the admin token. Enter the token printed at startup (stored in `.lan-secrets.json`) |
| Old UI after an update | Restart the server and hard-refresh the browser |

## START rejected

| Code or state | Meaning |
|---|---|
| ACK code **2** (`B1_BAD_STATE`) | No route mapping accepted since this boot, or already running. Send the four distinct A–Z letters of the real track, wait for the ACK, then START (or use "Send mapping, then Start") |
| ACK code **5** (`B1_NOT_READY`) | IR, IMU, scanner or controller readiness failed. **Do not bypass it**; fix the sensor or calibration |
| Mission **5**, "NAVIGATION SAFETY STOP" | Recoverable. Press STOP once, wait for READY, then START |
| Mission **7**, "CONTROLLER FAULT" | Latched genuine fault. STOP does not clear it; investigate and reboot |

An accepted ACK means the request was accepted, not that the wheels moved.
