# Linked ARM RAM and record-size evidence

Compiler: arm-none-eabi-gcc (Arm GNU Toolchain 15.3.Rel1 (Build arm-15.149)) 15.3.1 20260627

SDK 2.2.0; SMP=1; USB CDC; live MQTT; obstacle sources enabled;
matching private configuration and build flags on both source copies.
The original V14 directory was not modified.

| Section | V14 bytes | V15 bytes | Delta |
|---|---:|---:|---:|
| boot2 | 256 | 256 | +0 |
| .text | 419789 | 423181 | +3392 |
| .time_critical.tinyusb | 456 | 456 | +0 |
| .ARM.exidx | 8 | 8 | +0 |
| vector | 192 | 192 | +0 |
| .data | 2556 | 2556 | +0 |
| .bss | 208240 | 208264 | +24 |
| Static SRAM total | 210988 | 211012 | +24 |

Static sections include reserved kernel heap storage. Runtime stack/heap
utilization, ISR latency and worst-case execution time are not measured.
Debug sections reside in the ELF file and are not counted as SRAM.

## ARM object record-size probes

| Record | V14 bytes | V15 bytes | Delta |
|---|---:|---:|---:|
| B1_Event | 9 | 9 | +0 |
| B1_Mission | 20 | 20 | +0 |
| B1_Motion | 20 | 20 | +0 |
| B1_OdomRecord | 10 | 10 | +0 |
| B1_Store | 736 | 736 | +0 |
| B1_Terrain | 16 | 16 | +0 |
| G7_TelemetryPeers | 68 | 68 | +0 |
| bc_fastq_sample_t | 24 | 24 | +0 |
| downhill_governor_t | 8 | 8 | +0 |
| motion_status_t | 48 | 48 | +0 |
| nav_controller_t | 104 | 104 | +0 |
| nav_result_t | 2 | 2 | +0 |
| nav_sample_t | 16 | 16 | +0 |
| scan_handoff_t | 0 | 12 | +12 |
| terrain_snapshot | 24 | 28 | +4 |

Record probes use ARM GCC and actual kernel/application headers,
not native host sizes or host RTOS stubs. The scan handoff is new;
its V14 value 0 means absent, not an empty V14 instance.

The v1 MQTT payload comparison remains identical to the supplied
[message-size ledger](../V11/mqtt_message_size_comparison.csv).
