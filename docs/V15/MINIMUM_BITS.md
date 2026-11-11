# Data representation and ARM memory evidence

The supplied [V11 precision review](../V11/PRECISION_REVIEW.md),
[Change Requirement 1 checklist](../V11/CHANGE_REQUIREMENT_1_CHECKLIST.md) and
its field ledger remain authoritative review inputs. The original course
specification document itself was not present; its supplied checklist is not
silently substituted for an assessor's final decision.

## Full compiled-source inventory

[application_fields.csv](application_fields.csv) inventories application record
members parsed by Clang using the **actual ARM build's defines and include
paths**, including C-local records, anonymous structures and unions.
[application_globals.csv](application_globals.csv) separately inventories global
and function-static instances. No private initializer values are exported.
The earlier V11 ledger had 313 known entries; this extends coverage using parsed
declarations across the production target's translation units. Test-only and
unreferenced build-profile-specific declarations are outside that compiled
profile. This is a coverage improvement, not proof of all conditional profiles.

Each member lists its canonical signedness/type, stored bit width when scalar,
representational range, known semantic range/sentinel when supported by prior
review, minimum-width judgement, ownership and evidence/open status. The old
generator incorrectly classified some uint32 counters as signed through a
substring test; this inventory uses the parsed unsigned type. An inferred type
range is explicitly distinguished from a physically proven operating domain.

Reproduce with a full target build log (including compiler recipes), then:

```bash
python3 tools/audit_v15_fields.py --build-log path/to/full-build.txt \
  --arm-include path/to/arm-none-eabi/include --output docs/V15
```

## V15 state, exact ranges and synchronization

| Field | Valid values / sentinel | Minimum | Protection / reason |
|---|---|---|---|
| nav.have_barcode | 0/1 | 1 bit | Controller owner; preserves valid sequence zero |
| scan.letter | 0..25; 31 invalid; 26..30 reserved | 5 bits | Port semaphore, validate before packing |
| scan.ready/busy/enabled/owned/overflow | 0/1 each | 1 bit each | All reads and RMW use the same semaphore |
| scan.sequence | 0..65535, modulo, zero valid | 16 bits | Source event identity; no reserved-zero shortcut |
| scan.captured_msec/completed_msec | Full modulo kernel clock | 32 bits each | Capture provenance and deadline decisions across scheduling delays |
| controller staged event | 0..21 | 5 bits | Single controller writer |
| staged value/fault | 0..13 each | 4 bits each | Single writer, remote mutex around copied publication |
| staged sequence | Modulo 0..65535 | 16 bits | Existing diagnostic event identity |
| request_ok/capture_known/imu_seen | 0/1 each | 1 bit each | Controller owner; complete staged word is 32 bits |
| terrain.collision_sequence / observed sequence | Full modulo event count | 32 bits each | Snapshot mutex; compare-before-ACK prevents stale reset |
| last valid IMU timestamp | Full modulo kernel clock | 32 bits | Controller owner; bounds sign-disabled stale-data operation |
| motion downhill percent/engaged/fault/pitch | 0 or 65..100 / 0..1 / 0..2 / -900..900 deci-deg | 7 / 1 / 2 / 11 bits | Published under motion mutex, within existing alignment space |
| cancel request | 0/1 | 1 logical bit in an 8-bit object | Independent atomic byte access across worker/controller tasks; no packed shared RMW |
| EVENT kind 21 reason | 1..13; 0 none | 4 semantic bits | Existing v1 event-value carrier remains 16 bits for wire compatibility |

The scan transaction is 12 bytes: 64 timestamp bits + 16 sequence bits + 10
letter/flag bits, rounded to addressable bytes/alignment. Timestamp and event
identity storage is not shortened for cosmetic savings. Existing V14 governor,
IR, ADC, barcode layouts, odometry/event rings and snapshot packing are retained.
No native compiler-dependent bit-field structure is sent on MQTT.

PI integral/calculation values remain float32, with conditional anti-windup and
bounded output. Repacking calculation intermediates without an error budget
could change stability. Encoder lifetime counts and modulo clocks retain their
widths; V15 removes signed-overflow arithmetic in navigation travel deltas.

## ARM RAM / message comparisons

[ARM_RAM.md](ARM_RAM.md) contains same-toolchain linked ELF section evidence and
record-size probes. Both builds use SMP=1, USB CDC, live MQTT control, SDK 2.2.0,
the same private local network headers, and obstacle sources enabled. V14's
default build omission is documented in [ROOT_CAUSE.md](ROOT_CAUSE.md).

The telemetry store remains 736 bytes on ARM (760 on the native host), event
ring elements 9 bytes, odometry
ring elements 10 bytes, line records 12 bytes, mission records 20 bytes and
terrain telemetry records 16 bytes. The new scan channel is separate from those
wire/queue records. Native record-size tests are supported by ARM object probes;
host sizes alone are not substituted for the ELF RAM comparison.

All MQTT v1 payload sizes remain unchanged: snapshot 13, ODOM/event 6, status 14,
sensor 24/32 and barcode diagnostics 28/42 bytes, plus the 14-byte envelope/CRC.
Four letters remain 20 bits in three mapping bytes. Existing C/JS golden vectors
and legacy STATUS length compatibility are retained. A new protocol is not
needed for a diagnostic event value; no v2 migration was made.

## Remaining acceptance work

OPEN/PRECISION_RETAINED/INSTANCE_REVIEW_REQUIRED entries are explicit remaining
work. No claim is made that every float, decoder score, diagnostic counter or
byte-aligned v1 protocol field is mathematically minimal over a proven physical
domain. An assessor decision is still needed on v1 byte-aligned command/status
fields; changing them would require a coordinated, versioned firmware/dashboard
migration with measured benefit. Actual control-loop timing, ISR latency, stack
and heap high-water marks, queue pressure and cross-core stress still require
the car. Existing locks/ownership plus deterministic host boundary tests are
evidence of code behavior, not target SMP performance certification.
