# V13 — Calibrated downhill governor

Source baseline: V12 Scanner & Collision Update. New folder preserves V12.

Implemented: controller-owned filtered signed IMU pitch governor, 65% forward
wheel-target scaling after steering (including bypass), stale-sensor and
encoder-overspeed fail-stop latches, bounded brake, boot/status logs,
2-bit/3-bit counters, 11-bit filtered angle and no MQTT wire modifications.
Default hardware sign **0** disables ramp handling pending physical check.

Verification: 73/73 dashboard tests and all previous host obstacle/navigation,
precision, network-owner tests pass. New pure and actual motion-control host
tests for both +1 and -1 pitch sign pass. Cortex-M0+ Clang objects compiled
for three affected modules (not a full Pico firmware build).

Mandatory outstanding: physically determine nose-down sign and enable,
rebuild with Pico SDK, test on robot and measure RAM, control loop timing,
wheel speed, barcode/obstacle/collision interactions. The wider minimum-bit
submission remains subject to pre-existing full audit/wire requirements.

See docs/V13/README.md for instructions and evidence.
