# V14 — Controller recovery and smoother downhill governor

This is a separate V14 release based on V13, preserving the live MQTT protocol,
remote STOP priority, binary telemetry, mapping, barcode scanner, navigation,
obstacle system, dashboard layout, and bit-packed application data.

**Fixed:** persistent collision latch preventing STOP -> HOLD -> START;
controller collision event acknowledging under an RTOS mutex; clearer STOPPED
versus HOLD monitoring; downhill derating no longer pulses wheel duty to zero
for minor target error and no longer trips a fault for a short benign overshoot.

**Tested:** 74/74 dashboard, 144 navigation assertions, host collision rearm,
remote controller, terrain mutex, binary/protocol, obstacle, GCC/Clang checks;
ARM object compilation for changed C modules. Full Pico SDK link and physical
ramp/junction tests **not performed**. Read `docs/V14/README.md` before flashing.
