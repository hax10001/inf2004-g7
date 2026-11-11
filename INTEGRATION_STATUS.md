# INF2004 G7 — Improve Movement + MQTT, V7

Motion/IR/barcode/terrain source matches the original improve-movement ZIP byte-for-byte. The experiment-only Buddy 5 obstacle/ultrasonic/servo module and reference files are removed. No ultrasonic task or bypass controller is compiled. Wire V1 snapshot retains unused obstacle fields for compatibility, always marked unavailable.

Dashboard MQTT bridge and RTOS-owned MQTT adapter retain the architecture required by the course: Wi-Fi service owns lwIP; subsystem publishes bounded messages and the owner drains them. Original motion controller retains sole PWM authority. Start/Stop is controlled by remote command queue with lock-protected safety checks; STOP over MQTT is NOT a hardware emergency stop.

**Start code 2:** B1_BAD_STATE: no mapping accepted since this Pico boot or already running. Status packet type 7 adds mapping/ready/running bits for diagnosis. The dashboard now has an explicit mapping-first flow and optional mapping-ACK-then-start sequence. Code 5 = B1_NOT_READY; do not bypass IR/IMU calibration.

**Live Telemetry Console:** direct binary MQTT type SENSOR/STATUS plus barcode event frames; no IDE serial monitor required after MQTT link established. Diagnostic USB printf is optional and is not duplicated as raw MQTT serial.

**Known limitations:** physical actuator behavior and stop latency are not independently verified. Original barcode decode accuracy has preexisting fixture failures; no threshold changes. Wheel geometry remains unverified, so route can be approximate/missing. No obstacle hardware data in this build.
