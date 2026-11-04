# Buddy 5 / Buddy 1 — Cortex-M0+ controller linker fix

## Why the UF2 link failed

`task_controller.c` used GCC `__atomic_exchange_n` on 32-bit remote command
flags. On Cortex-M0+, this emits an external `__atomic_exchange_4` call,
which was not supplied by the firmware link. The previous error was a
linker failure, not a Pico SDK / Wi-Fi / barcode problem.

This update replaces those atomic read-modify-write operations with a short
µT-Kernel priority-inheritance mutex. The mutex is created in
`controller_init()` **before tasks start**. The network task merely queues
remote requests; the original controller task remains the sole motor writer.
The controller polls the mutex without waiting, and if it cannot get the
mutex in that tick it **commands a local stop** without discarding a pending
STOP. STOP always clears an unconsumed START; reconnect never resumes motion.

No `-latomic` workaround is required. Live control remains opt-in. This fix
does not enable Buddy 5 ultrasonic avoidance or validate motor-stop latency.

## Build on your Mac

From your existing project root, retaining your local Wi-Fi configuration:

```bash
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" \
bash build_uf2.sh 2>&1 | tee g7-build.log
```

**Important:** Because the command uses `tee`, check the last output says
`Created: .../out/g7-buddy5-mqtt-bench.uf2` before flashing. For an explicit
failure-catching version, run `set -o pipefail` in your shell first.

```bash
ls -lh out/g7-buddy5-mqtt-bench.uf2
```

For a fresh test, if you need to ensure this is not an old UF2, remove only
`out/g7-buddy5-mqtt-bench.uf2` before running the build. Do not remove your
configuration headers.

## Host tests

```bash
bash tools/test_controller_remote.sh
python3 tools/check_app_standard.py
bash app_program/modules/telemetry/run_native.sh
npm --prefix app_program/dashboard test
```

Host tests and an ARM object compile were performed, but a **full Pico UF2
link / physical test has not been performed here**. If the ARM linker reports
another unresolved symbol, provide only the final `undefined reference`
lines and the `make: ***` summary, after removing any secrets.

For bench testing keep the wheels off the floor, a physical power cutoff
available, and do not assume dashboard STOP can operate without MQTT.
