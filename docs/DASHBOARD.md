# Dashboard

`app_program/dashboard/` is a dependency-free Node.js (18+) server with a
built-in development MQTT broker. It decodes the robot's binary MQTT frames,
shows them live, records sessions and, when enabled, sends mapping, START and
STOP.

## Ways to run it

| Command (from the project root) | Binds | Commands | Use |
|---|---|---|---|
| `bash run_live_dashboard.sh` | LAN (`0.0.0.0`) | **Enabled** (`LIVE_COMMANDS_ENABLED=1`, `DASHBOARD_SOURCE=robot`) | Bench tests of the real robot |
| `cd app_program/dashboard && npm run lan` | LAN | Read-only (HTTP 409) | Monitoring the real robot |
| `bash demo.sh` | `127.0.0.1` | Simulator only | Demo without hardware. Exits if 3000 or 1883 is already in use |
| `cd app_program/dashboard && npm start` | `127.0.0.1` | Read-only unless enabled | Local use. The Pico cannot reach a loopback broker |
| `cd app_program/dashboard && npm run demo` (or `demo:drive`) | n/a | n/a | Simulator publisher only; run it alongside a server |

The LAN modes (`tools/start-lan.js`) create `app_program/dashboard/.lan-secrets.json`
on first run. It holds the MQTT username and password and a dashboard admin
token, and is reused afterwards. The startup banner prints them along with the
Mac's LAN address. These are the credentials the setup wizard compiles into the
firmware, so keep them private and don't delete the file unless you also rerun
the setup and rebuild.

Open `http://localhost:3000`. A browser on another device needs the admin token
for commands; without it, requests get HTTP 401.

## Views

- **Overview**: robot connection, mapping state, START readiness, sensor-feed
  freshness and the last scan. A live MQTT heartbeat is shown separately from
  sensor freshness, because a connected robot can still have stale sensors.
- **Map & robot**: robot-acknowledged letter assignments, barcode detection
  order, an odometry-estimated route (only when `ODOM` packets arrive) and an
  illustrative 3D chassis. It is not a SLAM map.
- **Console**: four panes for sensors, barcode results (decoded letters and
  rejected attempts with reasons), robot log, and commands/ACKs. Each pane has
  independent scroll-follow with a Latest button, plus copy, clear, pause and
  export. Command history is collapsed by default, and maintenance actions are
  under More controls.

Mapping is in the header. START and STOP are always visible. The palette is
navy `#182B42`, off-white `#FFFEFB`, mustard `#C49B3B` and beige `#F0EADF`;
green and red are reserved for START/STOP and meaningful state.

## Mapping, then START

The robot rejects START with code 2 until it has accepted a route mapping since
boot.

1. Open **Mapping** and enter the four **distinct A–Z letters printed on your
   track**. The UI's default letters are only placeholders.
2. Click **Send mapping** and wait for an accepted ACK. The Console should show
   the ACK, and the status should show the mapping loaded. Or use **Send
   mapping, then Start (after ACK)**, which sends START only after the mapping
   ACK (`/api/map-then-start`).
3. START still goes through the robot's own readiness checks (IR, IMU, scanner,
   MQTT). A rejection with code 5 means not ready.

Scan alerts come only from the firmware: a decoded-letter EVENT gives
`SCAN SUCCESS — decoded letter X`. An increase in the rejected counter gives
`SCAN FAILED ×N` with the last reason. Letters are never inferred from IR
edges.

## Environment variables

Export these before starting (there is no `.env` loader; `.env.example`
lists the defaults).

| Variable | Default | Meaning |
|---|---|---|
| `WEB_BIND` / `WEB_PORT` | `127.0.0.1` / `3000` | HTTP server. The LAN modes force `0.0.0.0`. The banner always prints 3000 |
| `BROKER_BIND` / `BROKER_PORT` | `127.0.0.1` / `1883` | Built-in broker. The LAN modes force `0.0.0.0` |
| `BROKER_EXTERNAL`, `BROKER_HOST` | off, `127.0.0.1` | `BROKER_EXTERNAL=1` uses an external broker such as Mosquitto (see `mosquitto.example.conf`) |
| `ADMIN_TOKEN`, `MQTT_USER`, `MQTT_PASSWORD` | from `.lan-secrets.json` | Required when binding beyond loopback |
| `LIVE_COMMANDS_ENABLED` | `0` | `1` allows mapping, START and STOP to reach the robot |
| `LIVE_IMU_RECAL_ENABLED` | `0` | `1` allows the IMU recalibration command |
| `DASHBOARD_SOURCE` | `auto` | `auto`, `sim` or `robot`. `auto` prefers hardware when both publish |
| `MQTT_NAMESPACE`, `ROBOT_ID` | `g7/b1`, `robot` | Topic prefix. Telemetry arrives on `g7/b1/robot/telemetry` |
| `RECORDING_DIR` | `./recordings` | Session recordings (gitignored) |
| `TRACK_WIDTH_MM` | `120` | Track width for the drawn route; set the measured value |

The serial bridge (`serial_monitor.sh`) POSTs lines to
`http://127.0.0.1:3000/api/serial`. If you change `WEB_PORT`, pass
`--url http://127.0.0.1:<port>/api/serial`.

## Tests

```sh
cd app_program/dashboard && npm test
```

These cover the binary protocol, commands and ACKs, MQTT integration, the
control modes, recording export and UI contracts. They are host-only.
