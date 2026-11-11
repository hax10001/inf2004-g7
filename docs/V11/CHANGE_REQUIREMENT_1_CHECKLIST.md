# Change Requirement 1 — evidence and open acceptance checklist

Requirement | V11 result | Evidence / gap
--- | --- | ---
Minimum bits for state values and fault flags | **Partial, tested** | V10/V11 packed nav, mission, obstacle, barcode, IR and quality fields; application_field_audit.csv
Motion/encoder application data | **Partial, tested** | 10-byte ODOM queue, 16-bit sequence, signed deltas; physical scale and high-range arithmetic retain full widths
Barcode/line sensing | **Partial, tested** | 12-bit ADC, pattern9, status packed, IR 6-bit latch, barcode task/port tests; matcher score ranges still open
Obstacle detection | **Partial, tested** | Scan point angle7 + range12, obstacle flags; physical bypass clearance not verified
Hump estimation | **Partial** | Telemetry 10-bit levels and terrain states; terrain float/height geometry/precision review open
Vehicle/controller states | **Partial, tested** | Existing compact mission/navigation flags plus V11 steering, motion validity and obstacle flags
ISR/task shared data | **Partial, tested** | Barcode SPSC from V10, V11 independent IR latch mutex; remaining global shared data requires full inspection
Inter-task queue data | **Partial, tested** | ODOM/event rings and barcode queue packed; not all queues inventoried/verified
MQTT binary compact payloads | **Partial, wire-compatible** | v1 binary frames unchanged, C/JS golden vector passed, status/command fields not guaranteed mathematically minimal
Safe shared packed read-modify-write | **Partial, tested** | B1 mutex, IR mutex, barcode port semaphore, ISR owned queue; target SMP race audit pending
Week13 data representation table | **Provided; not final** | 313 known entries, several unresolved and full-source globals still to inspect
Week13 RAM before/after | **Host records only** | Host size comparison measured; no target ARM ELF/map or stack peaks
Week13 message size before/after | **Provided, zero wire change** | MQTT comparison CSV, static ring record reductions in size CSV
Week13 correctness/performance | **Host regressions passed; hardware missing** | 70 dashboard tests, 144 navigation assertions, GCC/Clang host tests; no Pico real-world timings

**This checklist is an evidence-based implementation checkpoint, not a claim of final compliance.**
