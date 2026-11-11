# G7 Buddy 5 — MQTT owner refactor + macOS USB-CDC build/test

This is **experimental bench firmware**, not a physically qualified obstacle
bypass or remote-motion release. It preserves the autonomous controller's
startup order. Wi-Fi joins time out after 10 s; network work never gates motor,
IR or IMU task startup. An offline car cannot forward logs to the dashboard
unless a USB/UART cable bridge is attached.

## RTOS Wi-Fi owner architecture

The **uploaded ZIP did not contain** `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`
or an upstream MQTT heartbeat client. That file is referred to in the class
forum guidance, but is not part of the supplied branch. Therefore this package
cannot claim to reuse or validate that external version. The existing
app-side TCP/MQTT transport was moved into
`lib/libnet/lwip/mqtt_utk.c` to respect the **current repository's**
`lib/libnet/lwip/` layout; it still uses a minimal raw-TCP MQTT protocol
implementation, **not** the external RTOS-provided MQTT client mentioned in
the forum. Do not cherry-pick the two implementations together without
reviewing their APIs and network-owner model.

- App buddy tasks -> `b1_mqtt_submit(frame)` -> bounded 8-frame telemetry TX
  ring (2 frames for ACKs). Only the Wi-Fi owner takes and publishes it.
- MQTT callback copies command bytes into bounded 8-frame RX ring, with an
  urgent 2-frame STOP queue. The app-side telemetry task takes decoded copies.
- `cyw43_utk.c` is the only site calling `mqtt_utk_poll()`, from the Wi-Fi
  service loop after lwIP timeout processing; all TCP/lwIP calls are in
  `lib/libnet/lwip/mqtt_utk.c`. No sensor or application task calls lwIP.
- Queue operations are bounded and non-blocking, including in callbacks.
  Overflow increments drop counts. Subscriptions are restored after CONNACK;
  no movement command replay across reconnects. Commands remain disabled for
  real motor control in the provided UF2 build (`B1_LIVE_COMMANDS=0`).
- This preserves the existing **single-producer/single-consumer** queues.
  They are fixed-size software message buffers, not `tk_cre_mbf` kernel objects.
  If the team adopts the instructor's separate upstream `mqtt_utk.c` or native
  µT-Kernel MBFs, review the queue ownership, initialization order and APIs
  before replacing this module. Do not call blocking `tk_rcv_mbf` from the
  Wi-Fi task.
- For new installations, the setup wizard saves Wi-Fi credentials to
  `config/wifi_credentials.h` and broker settings to
  `config/wifi_secrets.h`. Existing configurations using
  `app_program/modules/telemetry/buddy1_broker_local.h` still work. Neither
  local secret is included in the downloadable archive.

## macOS build (`.d`: No such file or directory)

The previous log showed the compiler failing to **open a dependency output**
under `build_make/mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/`, not a
missing C source header. The build helper now pre-creates that directory and
runs the USB-CDC profile sequentially to avoid the legacy make parallel-build
issue. It does **not** remove the coding-standard check.

From the project root after copying your existing private local headers:

```sh
chmod +x build_uf2.sh serial_monitor.sh
G7_CONSOLE=usb_cdc PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" \
    bash build_uf2.sh 2>&1 | tee build-usb.log
ls -lh out/g7-buddy5-mqtt-bench.uf2
```

If this fails, send the **first `error:` line** with 10 lines of context;
mask secrets first. Do not flash unless the UF2 was rebuilt successfully.
Do not share generated UF2 builds publicly since credentials may be embedded.

## macOS serial monitor

macOS generally uses `/dev/cu.usbmodem*` for outgoing connections, not the
Windows `COM` name. `/dev/tty.usbmodem*` may also exist. The firmware must be
built with `G7_CONSOLE=usb_cdc` and flashed; **BOOTSEL / RPI-RP2** is a USB
mass-storage device, not an active serial console. Use a USB **data** cable.

Start the dashboard in one terminal:

```sh
cd app_program/dashboard && npm run lan
```

From project root, in another terminal:

```sh
python3 -m venv .venv-serial
.venv-serial/bin/python -m pip install pyserial
./serial_monitor.sh --list
./serial_monitor.sh
```

The last command auto-selects a Pico USB CDC serial port and posts only to
`127.0.0.1:3000/api/serial`. To choose explicitly:
`./serial_monitor.sh /dev/cu.usbmodemXXXX`.

If no port is found, run `ls /dev/cu.usbmodem* /dev/tty.usbmodem* 2>/dev/null`,
reconnect the Pico with BOOTSEL **released**, check the USB data cable, and
inspect **System Information -> USB**. If a port exists but opening fails,
close the Arduino IDE, screen, minicom or other serial programs using it.
If a terminal opens but no output appears, restart the Pico after opening the
monitor and look for startup messages; inspect TinyUSB service/interrupt
initialisation on the physical Pico. A Mac's USB port alone cannot prove that
firmware enumeration has worked.

## Test order

1. Run `python3 tools/test_network_owner_boundary.py` and
   `python3 tools/check_app_standard.py` (both expected PASS).
2. Run the dashboard tests: `cd app_program/dashboard && npm test`.
3. Build and flash a **new** UF2 with motors disconnected and wheels raised.
4. Reboot without BOOTSEL. Check USB serial enumerates and startup logs arrive.
5. Start the dashboard LAN broker; confirm the Pico and Mac are both on
   **Pookie** and that broker address `10.74.40.243:1883` is still the Mac's
   *current* reachable IP. If not, update the local header and rebuild.
6. Scan large Code 39 `*A*`, `*B*`, `*C*`. Website Monitor should show
   validated `Decoded: A`, `Decoded: B`, `Decoded: C` via MQTT or serial.
   The known barcode decoder accuracy issues remain unresolved.
7. Unplug router / disable broker: motion continues its existing behavior and
   serial output (when connected) remains available. **Remote STOP is not an
   emergency stop and cannot arrive offline**. Supervise all wheel tests.

No physical Pico or ARM compiler is available in this environment; a full
UF2 compile, USB enumeration and the actual obstacle bypass remain unverified.
