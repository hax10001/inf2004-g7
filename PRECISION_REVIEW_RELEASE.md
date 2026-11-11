# G7 V11 — source-grounded precision review addendum

This release is a **preservation-first addendum to V11**, not an assertion that Change Requirement 1 or the Pico W physical acceptance is complete.

Read `docs/V11/PRECISION_REVIEW.md` first. It describes exactly which finite-range fields were packed, what remains in float32 and why, the 19 other reviewed range questions, protocol compatibility decisions and unmeasured hardware checks.

## What changed from the original V11 ZIP

- `app_program/modules/motion/steering.h`: signed two-bit `last_side` and `bend_side` (exact -1/0/+1); 36,000 identical steering traces against the original V11.
- `app_program/modules/barcode_scanning/bc_match.h`: three-bit `support` (0..6 threshold votes).
- `app_program/modules/barcode_scanning/bc_layout.h`: seven-bit `bars` (0..96) and ten-bit `module_x10` (0 or 300..1000).
- `app_program/modules/terrain/terrain.h`: one-bit calibration-valid flag, retaining its mutex protection.
- `tools/tests/test_precision_ranges.c`, `tools/test_precision_ranges.sh`: native range boundary test.
- `docs/V11/PRECISION_REVIEW.md`, `docs/V11/PRECISION_REVIEW_AUDIT.xlsx`: technical review and field audit.

**Not changed:** MQTT protocol version and payloads; dashboard; motor control equations; wheel calibration; barcode decoder thresholds; IMU/hump algorithm; ISR queue boundaries; existing PIN or network settings.

## Run verification on macOS

```bash
cd ~/Downloads/inf2004-g7-v11-precision-reviewed
bash tools/test_precision_ranges.sh
bash tools/run_v11_tests_mac.sh
```

For ARM build use the same `G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1` and Pico SDK configuration as V11. The actual target build and robot tests are **not** claimed by the host evidence.

This source ZIP contains no local Wi-Fi credentials, private broker secrets or runtime recordings. Configure your network settings locally before building/flashing as needed.
