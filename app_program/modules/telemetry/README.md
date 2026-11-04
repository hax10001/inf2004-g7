# Buddy 1 telemetry and MQTT (isolated, opt-in)

**Current integration status: host tests pass; hardware and cross-compiled firmware unverified.**

- `buddy1_protocol.*`: compact, versioned, CRC-checked wire frames. Never send raw C structs as MQTT messages.
- `buddy1.*`: bounded queues and protected snapshots, command validation and state.
- `telemetry_peers.*`: read-only adapters to the current IR, IMU and motion subsystems; missing data is marked unavailable.
- `buddy1_rt.*`: opt-in low-priority RTOS telemetry/command task.
- `buddy1_mqtt.*`: bounded non-blocking lwIP raw-TCP/MQTT client; its poll function runs only on the CYW43 lwIP owner task when opted into WIFI_MQTT=1.

Default build is untouched and never compiles this directory. For a reviewed test build use `WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_MQTT=1` with correctly configured network and broker headers; `B1_LIVE_COMMANDS=1` **only after stationary STOP/START tests**. Never flash without a safety review, since the physical MQTT control loop is not yet validated. The existing IMU interface does not support live recalibration; the dashboard cannot claim that command has completed on hardware. Provisional odometry is disabled by default.

**Important:** an MQTT STOP cannot be delivered across a WiFi outage. An independent physical stop and bench tests are required.
