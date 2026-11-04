# V7.1 — USB Wi-Fi Scan Build Fix

This V7.1 source archive corrects the two malformed C string literals in
`lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c` that prevented the
V7 firmware from compiling on the Pico W.

It does not alter the improve-movement motor/IMU/IR code, MQTT credentials,
Wi-Fi configuration or command-state safeguards.

For an existing V7 folder containing private Wi-Fi/MQTT settings, use the
separate `g7-improve-movement-V7-1-USB-COMPILE-HOTFIX.zip` rather than copying
this fresh source over your current project.

Build:

```
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh
```

The expected output is `out/g7-improve-movement-mqtt.uf2`. Flash only once
the build's ELF marker/MQTT linkage checks pass; motors disconnected for
initial bench tests. Actual ARM toolchain linking and hardware testing must
be performed on the target developer's Mac and Pico W.
