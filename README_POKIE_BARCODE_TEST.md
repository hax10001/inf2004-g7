# Pookie Wi-Fi + Barcode letter monitor (Buddy 5 experimental branch)

**The UF2 previously flashed on the car cannot be changed remotely**. This
package does not contain your Wi-Fi password. You must configure and rebuild
firmware locally, then flash the newly produced UF2. The Wi-Fi network must
support **2.4 GHz**. The Mac and Pico W must be able to reach each other.

## Configure the Pico W for Pookie

1. From the project root run `./setup_pookie.sh`. Press Enter to select
   `Pookie`, type your Wi-Fi password at the hidden prompt, then press Enter
   to use the displayed **Mac IPv4** as MQTT broker IP unless you run an
   external broker. This creates gitignored private config headers.
2. `cd app_program/dashboard && npm run lan` in a terminal on the same Mac.
   It prints the broker/admin credentials. Leave it running.
3. In another terminal run `PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" ./build_uf2.sh`
   to compile the UART-console bench UF2, or
   `G7_CONSOLE=usb_cdc PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" ./build_uf2.sh`
   for a USB serial-console UF2.
4. Flash `out/g7-buddy5-mqtt-bench.uf2` (BOOTSEL). **Disconnect motor
   power first** and test with wheels raised; live MQTT commands are disabled.

If the two local header files already exist, the setup wizard **does not
overwrite them**, since they may be correct. Back them up first if you must
reconfigure network settings. Rebuilding/reflashing is required after changes.

## What the ten seconds mean

Wi-Fi joins have a **10-second timeout after the join begins**. The robot
controller, motion, IR and IMU tasks start *without waiting for Wi-Fi or MQTT*.
A low-priority telemetry task logs its 10-second network status. Failed MQTT
connections retry in the background; networking never becomes a prerequisite
to line following. If Wi-Fi or MQTT is unavailable, the dashboard cannot
receive live MQTT data. This is **not** a guaranteed safety stop and no remote
STOP can be delivered when disconnected.

## Barcode output

The Buddy 3 scanner recognizes one **Code 39 uppercase letter**, printed as
`*A*`, `*B*`, etc., after its start/data/stop markers and adequate white
margins. With a valid result, the firmware produces:

    [BARCODE] Decoded: M seq=1 (Code 39)

And when MQTT works, Monitor logs `Barcode decoded: M` and the Overview
shows `Decoded barcode: M`. Scans which do not validate are not promoted into
letters; inspect existing `[BC]` diagnostic reports for failures. The current
scanner is known to be less reliable with small printed codes and requires
sensor/encoder calibration on the real car.

**Bench-only rearm:** The UF2 helper enables `B1_BARCODE_BENCH_ACK=1` so the
telemetry observer acknowledges successful results after publishing events,
allowing repeated barcode tests. This is intentionally *not* production
mission-action behavior: this path does **not** execute turns from barcode
letters. When a teammate adds a Mission Coordinator that owns barcode
commands, remove the bench ACK before deploying that integration.

## Monitor barcode letters even if Wi-Fi/MQTT fails

This needs a **physical serial cable**. A Wi-Fi outage cannot magically
transmit data to the web dashboard.

1. Build using `G7_CONSOLE=usb_cdc` as above and reflash; connect the Pico
   to your Mac over USB (or, for the default UART build, use a 3.3 V USB-UART
   adapter on the right UART pins, with common GND). The main car motor
   battery should remain disconnected during initial tests.
2. Install the local reader once: `python3 -m pip install pyserial`.
3. Find the device: `ls /dev/tty.usbmodem* /dev/tty.usbserial* 2>/dev/null`.
4. With `npm run lan` running in one terminal, use another:

       ./serial_monitor.sh /dev/tty.usbmodemXXXX

   Replace the device name with yours. The Python bridge prints serial lines
   in Terminal and POSTs them **only to localhost**, never to the network.
5. On the dashboard's Monitor tab, filter **Barcode letters** to see the
   decoded alphabet. Overview may show `M (USB serial)` when MQTT is absent.
   This does not imply MQTT connectivity or verified odometry.

**Not yet done:** The Buddy 5 ultrasonic bypass controller is *staged*
experimentally; this bench build does not automatically wire obstacle
avoidance into the original motor controller. Likewise the live hardware
Start/Stop/Recalibrate controls are not enabled. Barcode reads are telemetry,
not a complete autonomous junction-action implementation.

Do not share firmware UF2 files or private Wi-Fi/MQTT headers: they embed
credentials. No private credentials are included in this source ZIP.

## Verification status (source package)

- Dashboard tests: 18/18 pass, including binary MQTT barcode event and serial fallback.
- HTTP serial bridge smoke: `204` and decoded K visible with 0 MQTT frames.
- Application standards: 83 files scanned; 0 errors.
- Barcode decoder host tests: **pre-existing failures** in both upstream
  improve-movement and Buddy 5 archives; this package deliberately does not
  change their decoder thresholds or claim barcode accuracy is proven.
- Pico SDK / real car: not available for compilation or hardware testing in
  this workspace. You have successfully compiled an earlier revision on Mac,
  but this changed source requires a fresh UF2 build and flash.

LOCAL CUSTOM BARCODE NOTES - DO NOT OVERWRITE
