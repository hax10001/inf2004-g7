# V13: safe first build and downhill sign calibration

This release extracts to `inf2004-g7-v13-downhill/`, preserving V12.

1. `cd ~/Downloads/inf2004-g7-v13-downhill`
2. `bash tools/run_v13_tests_mac.sh`
3. Keep `APP_DOWNHILL_NOSE_DOWN_SIGN 0` while measuring the actual pitch sign.
   With motor power off, boot level and slowly tilt the *front down*.
   Check live IMU pitch in Telemetry Console. A rising positive pitch means
   choose `1`; a negative pitch means `-1`. Confirm nose-up is opposite.
4. Set the sign in `app_program/app/app_config.h` and rebuild:

```bash
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
  PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" \
  bash build_uf2.sh
ls -lh out/g7-improve-movement-mqtt.uf2
```

5. Flash with motors disconnected (BOOTSEL + USB -> RPI-RP2 -> copy UF2),
   then bench-test with wheels raised and accessible physical power cutoff.
6. Use `bash run_live_dashboard.sh` for robot diagnostics. The firmware prints
   `[RAMP] Governor OFF` until the sign is configured and re-flashed, and
   `[RAMP] DOWNHILL -> 65% wheel targets` after confirmed downhill detection.

**Do not use on an actual ramp until the real sign, thresholds, braking
capacity and collision false positives have been physically validated.**

See `docs/V13/README.md` for tests, bit audit and evidence limitations.
