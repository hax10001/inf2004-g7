# Current source snapshot

This ZIP contains the complete project source: Pico firmware, kernel/libraries,
dashboard, build scripts, tests and documentation. It includes the V15 scanner
START fix, smooth descent control, adaptive obstacle bypass and the simplified
single-Console dashboard.

The latest interface changes are described in [UI update](docs/V15/UI_UPDATE.md).
Firmware behavior and hardware checks are described in
[control refinements](docs/V15/CONTROL_REFINEMENTS.md).

## Run the dashboard

Node.js 18 or newer is required. No third-party npm packages are required.

```sh
cd app_program/dashboard
npm start
```

Open `http://localhost:3000`. The navigation has Overview, Map & robot and one
Console destination. The console has sensors, barcode results, robot logs and
command events. Command history, export actions and maintenance controls remain
available on demand. The simulation is available through `bash demo.sh` from
the project root. For the actual robot's mapping and Start/Stop, use the existing
`bash run_live_dashboard.sh` workflow from the project root.

## Configure and build the firmware

Local Wi-Fi/MQTT credentials, dashboard tokens, recordings and compiled firmware
are excluded. Example configuration files are included; the original workspace's
local credentials are unchanged. A newly extracted folder needs its own local
configuration before connecting to a real robot.

Follow [Mac setup](START_HERE_MAC.md) and
[firmware build instructions](README_BUILD_UF2.md). The target requires Pico SDK
**2.2.0**, its submodules and the ARM toolchain. From a configured project root:

```sh
PICO_SDK_PATH=/path/to/pico-sdk-2.2.0 G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 bash build_uf2.sh
```

The resulting file is `out/g7-improve-movement-mqtt.uf2`.

The current source passed the maintained host regression suite, 78 dashboard
tests and the ARM target build. Browser checks cover desktop and phone layouts,
filtering, action menus, pause/resume, scroll following and stale scanner state.
Those software checks do not replace physical ramp and obstacle trials.

`release_manifest.json` lists the SHA-256 of every packaged file. Build output,
the SDK/toolchains, dependency caches and earlier release archives are excluded.
