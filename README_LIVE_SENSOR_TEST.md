# G7 Buddy 5 — guarded live MQTT + serial sensor bench test

## Current implementation boundary

- **Pico firmware:** µT-Kernel CYW43 service owns Wi-Fi/lwIP and MQTT; TX/RX
  buffers isolate the app. Wi-Fi attempts last at most 10 s each, with a
  short retry between attempts. MQTT reconnects independently.
- **Controlled build only:** `G7_LIVE_CONTROL=1` compiles in dashboard
  administrative Start/Stop. It boots in REMOTE_HOLD with motors commanded
  neutral. The dashboard must send a valid mapping, then START, after
  Wi-Fi/MQTT subscription and fresh IR + IMU calibration.
- **When MQTT is detected lost while following:** the controller locally
  requests a motor stop and remains held; reconnect NEVER auto-resumes.
  Detection can be delayed by Wi-Fi/TCP/MQTT timeouts; this is not an
  instantaneous disconnect interlock. These are
  *software* guards, **not** a verified hardware emergency stop. Keep a
  physical power cutoff, test with wheels in the air, and do not run
  unattended. A STOP sent during a disconnection cannot be delivered.
- **Sensors:** the extended v1 binary SENSOR packet (type 6) carries IR
  left/right/barcode, roll, pitch, magnetic heading (if valid), IMU hump
  phase, calibrated flag, encoder counts, ultrasonic range and obstacle
  phase *only when initialized*, at approximately 1 Hz. Missing values
  are explicitly unavailable. Original compact snapshots are unchanged.
- **Actual Buddy 5 integration:** obstacle source is staged; this build
  does not start the Buddy 5 servo/sonar obstacle task. Ultrasonic will
  show UNAVAILABLE, not invented readings, until Buddy 5's physical
  initialization and runtime task ownership are integrated and tested.
  Barcode A–Z result is observed but not hooked to junction mapping.
- **Simulation:** `./demo.sh` is synthetic. Never use it as evidence of
  real IMU, sonar, barcode accuracy or autonomous bypass.

## Build controlled firmware on your Mac

Keep your private `config/wifi_credentials.h` and `config/wifi_secrets.h`.
Confirm the broker target is your Mac's current Wi-Fi IP (previously
`10.74.40.243`, may change after reconnect). Start the hotspot first.

```bash
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh
ls -lh out/g7-buddy5-mqtt-bench.uf2
```

**Do not flash unless the build actually succeeded**; this patch has host
unit/static tests only, not a verified ARM/Pico physical test. No private
credentials are supplied in the ZIP.

## Launch dashboard without simulator

```bash
./run_live_dashboard.sh
```

This enables admin live controls on the host *only* for this launch. Enter
admin token from your own local `.lan-secrets.json` into the dashboard's
Mapping dialog; click **Send mapping**, then **Start**. Watch mission state,
confirm Stop applies and 3D wheel animation ceases. Command ACK means the
controller accepted it, not that motor power was independently measured.

## One serial reader for Terminal **and** dashboard

Use a Pico USB data cable and a `usb_cdc` build. From a second terminal:

```bash
python3 -m venv .venv-serial
.venv-serial/bin/python -m pip install pyserial
./serial_monitor.sh --list
./serial_monitor.sh | tee g7-pico-serial.log
```

The same **single process** opens `/dev/cu.usbmodem*`, prints each line
unmodified to Terminal and POSTs it to `http://127.0.0.1:3000/api/serial`.
A second serial viewer (e.g. screen/Arduino monitor) typically cannot
open that port simultaneously. Use `tee` for saved text and the website
Monitor for a second live view. Serial works even when MQTT is offline.

Expected examples:

```text
[NET] wifi=1 mqtt=1 join=0 dhcp=1 retry=ongoing
[SENS] IR L=1 R=0 B=1 roll_cd=-125 pitch_cd=250 heading_cd=12345 hump=8 sonar=65535 phase=255 encL=-456 encR=789 valid=27
[BARCODE] Decoded: M seq=2 (Code 39)
[CTRL] Remote STOP applied
```

`sonar=65535` and `phase=255` mean unavailable, not measured values.
IR 0/1 is the sampled digital state; black/white depends on your sensor
polarity/calibration. Roll/pitch/heading are centidegrees (value / 100).
`valid` bits: IR=1, IMU=2, ultrasonic=4, motion=8, heading=16.

## Hardware bench checklist

1. Lift wheels, disconnect motor power for Wi-Fi/serial test, verify
   correct Pookie hotspot settings (2.4 GHz, WPA2 compatible).
2. Confirm `[NET] wifi=1`, then `mqtt=1`; if Wi-Fi stays 0 inspect
   join errors and credentials. Broker IP alone does not prove connectivity.
3. Confirm IR bits change when covering each of three sensors.
4. Tilt chassis and check measured roll/pitch; heading only if valid.
5. Confirm ultrasonic displays unavailable unless Buddy 5 was initialized;
   do not treat it as a working sonar sensor.
6. With independent motor power stop ready and wheels raised, send mapping,
   then START; verify controller enters RUNNING only after readiness checks.
7. Send STOP; verify wheel PWM/physical halt on bench. Then disconnect
   hotspot while running; verify local stop and **no auto-resume**.
8. Scan a real Code 39 A-Z and check `[BARCODE] Decoded: ...` (capture
   accuracy is a separate physical test).

## Known limits

Physical compilation and hardware integration are unverified in this
container; the Pico SDK and cross compiler are not available here. Never
claim physical STOP latency or 100% scan accuracy from host tests alone.
