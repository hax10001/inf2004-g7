# G7 Wi-Fi Scan V5 / Reset setup:wifi

The Pico W continues a direct boot join, preserving RTOS/CYW43 owner-only network access. **After the first failed join**, the radio owner issues one bounded-result Wi-Fi discovery scan and records up to 8 visible SSIDs, RSSIs, channels and authentication modes. The USB owner later prints a compact `[WIFI SCAN]` summary and `[WIFI AP]` lines on the **existing** USB CDC serial port, without calling CYW43/lwIP itself. No password is logged. Scans are one-shot per boot, not continuous. The USB summary is repeated about every 30 seconds so a newly opened monitor can see it. The radio country defaults to Singapore rather than the more restrictive worldwide setting, with `G7_WIFI_COUNTRY` as an optional build-time override for other locations.

The USB output includes a unique `G7DIAG-V5-SCAN-20261102` marker, verified in the linked ELF. It has not been tested on Pico hardware here. A scan may complete without any visible access points, or fail to start; in that case look at `rc` and `total`, not just the `Pookie_found` field. If it hangs, the `[BOOTNET]` lines still show the join status. A failed join is `-2` for CYW43_LINK_NONET; seeing the SSID in a scan doesn't guarantee authentication will succeed.

## Reset Wi-Fi configuration safely

From the project root: `bash reset_wifi_setup.sh`. This moves local Wi-Fi headers to an ignored, private backup subfolder and reruns the interactive `npm run setup:wifi` wizard. It does **not** delete `.lan-secrets.json`, so the MQTT broker credentials remain consistent. To edit credentials after the wizard, rerun this script; `npm run setup:wifi` by itself intentionally refuses to overwrite existing files. The password will not be displayed. **Rebuild and reflash** whenever settings change.

## Build and flash

```bash
cd ~/Downloads/g7-buddy5-live-sensors-control
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh
```

Check for `PASS: verified G7DIAG-V5-SCAN-20261102 in linked ELF used to produce UF2.` and flash `out/g7-buddy5-mqtt-bench.uf2` with **motor power disconnected**. Open your original `/dev/tty.usbmodemMTK3PICO1` Serial Monitor at 115200. USB output includes the V5 marker, `[BOOTNET]`, and when scan completes `[WIFI SCAN] rc=0 total=... shown=... Pookie_found=...` followed by up to eight AP lines with signal and channel. The radio/scan remains separate from the motor/IR controller; unchanged obstacle integration limitations apply.

If `Pookie_found=0` but other 2.4 GHz networks appear, inspect channel/country settings and SSID bytes. If `Pookie_found=1` but join stays `-2`, inspect the Wi-Fi join path/security/channel compatibility. If `total=0` for every scan, investigate radio scan behavior rather than changing the hotspot password.
