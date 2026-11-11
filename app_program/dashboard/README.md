# G7 Improve Movement + Live Telemetry Console

Run from project root: `bash run_live_dashboard.sh`; open `http://localhost:3000` and click **Telemetry Console**. This view displays live IR/IMU/encoder binary MQTT frames and barcode events; **no USB cable reader or VS Code Serial Monitor is required** once MQTT is connected. The separate **Monitor** view retains commands, warnings, optional serial diagnostics, and exports. Run `npm test` in this folder for host validation.

START code 2 is BAD_STATE: configure the real four-letter barcode mapping in **Mapping**, then send mapping and wait for robot ACK before Start. Alternatively choose **Send mapping, then Start (after ACK)**. Code 5 is NOT_READY and reflects physical readiness constraints; never disable those checks to force movement. The same hotspot must allow Pico→Mac reachability and firmware broker IP must match the Mac.

No Buddy 5 sonar/servo/bypass components are present. The 3D model remains illustrative. Network STOP is not a physical emergency stop.
