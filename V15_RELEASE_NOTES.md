# inf2004-g7-v15-navigation-recovery-ramp

V15 is based on the adjacent **inf2004-g7-v14-recovery** source. The original
V14 folder is untouched. This is an implementation and verification release;
physical car acceptance and final minimum-bit certification remain outstanding.

For the newest complete source ZIP, start with [current snapshot](CURRENT_SNAPSHOT.md).
The [Console UI update](docs/V15/UI_UPDATE.md) replaces the three monitoring
navigation aliases with one destination and removes repeated scanner summaries.

The current workspace also includes [scanner, descent and obstacle refinements](docs/V15/CONTROL_REFINEMENTS.md)
made after the initial release ZIP: START preserves an already armed scanner,
confirmed IMU descent smoothly reduces RPM, and obstacle bypass scans both servo
extremes and returns according to live clearance with smaller controller state.

The barcode worker now publishes an owned transaction with sequence, capture
completion, decode completion, result and busy state under one semaphore.
Navigation consumes it independently of telemetry acknowledgement. A valid
result completed before a junction deadline is accepted even when consumed
just after that deadline, provided navigation has not already faulted.

STOP writes all four motor compares to zero before resetting run state. START
rechecks readiness when executed. Collision acknowledgement compares the event
sequence; START no longer clears an intervening collision. Only a fresh explicit
START authorizes motion. STOP preserves sensor baselines and route mapping.

The V14 packed governor, pitch sign, hysteresis, 65% factor, steering ratio and
runaway supervision are retained. Its existing PI now integrates at reduced ramp
targets and can request bounded proportional short-brake/coast PWM. The verified
default pitch sign remains **0 (pitch detection disabled)** until measured on the
car; the existing confirmed IMU hump-descent phase also activates the governor.
No collision
threshold was relaxed without physical evidence.

Firmware/dashboard use the existing MQTT v1 frames. EVENT kind 21 adds a fault
source without changing any payload length, topic, command or ACK. An ACK still
means request acceptance; zero commanded PWM is not proof of stationary wheels.

Read [root causes](docs/V15/ROOT_CAUSE.md), [validation](docs/V15/VALIDATION.md),
[data audit](docs/V15/MINIMUM_BITS.md), [build/rollback instructions](docs/V15/BUILD_AND_ROLLBACK.md)
and the [physical checklist](docs/V15/PHYSICAL_CHECKLIST.md).
