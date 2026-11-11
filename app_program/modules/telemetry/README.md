# Buddy 1 telemetry and MQTT

Active when the firmware is built with `WIFI_MQTT=1`, as `build_uf2.sh`
always does (it adds `-DB1_ROBOT_APP=1 -DB1_ENABLE_MQTT=1`).

- `buddy1_protocol.*`: compact, versioned, CRC-checked wire frames (MQTT v1).
  Never send raw C structs as MQTT messages.
- `buddy1.*`: bounded queues, protected snapshots, command validation and state.
- `telemetry_peers.*`: read-only adapters to the IR, IMU, motion, barcode and
  obstacle subsystems. Missing data is marked unavailable (see the `valid` bits
  in [docs/ROBOT_OPERATION.md](../../../docs/ROBOT_OPERATION.md#serial-sensor-line)).
- `buddy1_rt.*`: low-priority RTOS telemetry/command task. It also prints the
  `[NET]` status line.
- `buddy1_mqtt.h`: application API (`b1_mqtt_submit`, `b1_mqtt_receive_command`)
  for the fixed-size queues. The transport itself is
  `lib/libnet/lwip/mqtt_utk.c`, polled only by the CYW43 lwIP owner task;
  application code never calls lwIP.

The queues are single-producer/single-consumer software buffers, not
`tk_cre_mbf` kernel objects. If you adopt a different upstream `mqtt_utk.c` or
kernel message buffers, review queue ownership, initialisation order and APIs
first. Never call a blocking `tk_rcv_mbf` from the Wi-Fi task.

Remote commands are compiled in only with `G7_LIVE_CONTROL=1`
(`B1_LIVE_COMMANDS=1`, which requires `WIFI_MQTT=1`). Odometry frames follow
`G7_ODOMETRY_MODE` in `config/g7_odometry_config.h` (1 = nominal, provisional).

Host test: `bash run_native.sh` in this folder. The same sources are also
built by `tools/run_host_tests.sh`.

**An MQTT STOP cannot be delivered during a Wi-Fi outage.** Keep an independent
physical stop and bench-test first.
