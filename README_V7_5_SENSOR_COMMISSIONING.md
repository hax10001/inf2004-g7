# V7.5 sensor commissioning over MQTT

The physical car already has three software pathways: (1) digital GP27 IR plus encoder-distance Code 39 decoder, (2) motion driver counting wheel A edges, (3) LSM303 magnetometer reading heading. This update **adds verification**, not untested hardware drivers.

- Barcode success: `B1_TYPE_EVENT` with source line, event kind=1, value 0..25 (A-Z). No letter/event means no confirmed successful barcode yet.
- Barcode scanner status: `B1_TYPE_BARCODE_DIAG` (type 8, 28 bytes), approximately 1 Hz. Show mode, sensor raw 0/1, cumulative raw edge transitions, active run count, rejected/complete counters, error code, left/right counts. GP27 is DIGITAL in this source. Use a correct Code 39 `*A*` label, with quiet zones, and pass it at moving wheel speed. Black/white contrast and wheel movement are both required; sensor toggles alone do not prove decoding.
- Raw wheel encoder counts: read directly from the encoder driver's atomic shadow counters (even when remote Start is held); published in type 6 sensor frames every second. Spin one wheel by hand with motors disconnected: counts must change. Encoder signs are normalized using the existing verified-in-this-build barcode forward-sign configuration when generating type 2 odometry frames; if wheel sign is wrong, do not claim a correct route.
- Odometry type 2: Enabled at NOMINAL scale with `config/g7_odometry_config.h` (`G7_ODOMETRY_MODE=1`). The old board defaults are UNVERIFIED: 1056 channel-A edges/rev and 65mm diameter. This is a **provisional estimate**, not reliable coordinates. Re-measure both wheels and count a complete full rotation, then update settings and set mode to 2 only when measured. Track width estimate in dashboard is still unverified; turning geometry is only illustrative.
- Magnetometer: Already read in `task_imu.c` and copied to MQTT sensor packet type 6, valid bit 16. The dashboard now shows heading separately from encoder-derived relative heading. Heading calculation is LEVEL-ONLY and not calibrated to true north. Keep it level, rotate slowly near no metal, and check value updates. When heading unavailable, I²C/magnetometer validity is exposed honestly.
- Safety: Telemetry module only. The motor controller, control-state gates, network secrets, barcode decoder and I²C driver are unchanged. Wheels off ground for moving tests, physical motor cutoff available. The code cannot guarantee a scan or accurate odometry without measurements.

Install patch, build with `G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh`, flash the new UF2, run `bash run_live_dashboard.sh`, open Telemetry Console. Export logs to verify events.

## Important pre-push test result

The unchanged native barcode decoder tests currently report **83 failed checks out of 3,409** on both V7.4 and V7.5. The new MQTT scan diagnostic only proves that the physical input is changing and that the decoder is attempting scans; a confirmed successful A–Z barcode event is required before claiming the scanner works. Decoder threshold changes require a separate review against real printed Code 39 labels.
