# V11 minimum-bit expansion — preservation-first release

The complete source is in this folder. Main implementation and test evidence are in `docs/V11/README.md`.

- Retains the V10 polished four-pane live dashboard and follow-to-latest scroll logic.
- Drops native-host `B1_Store` size from 1152 to 760 bytes by storing only necessary queued ODOM/event data.
- Adds compact obstacle, IR, terrain, motion, barcode state and 12-bit ADC representations.
- Removes a duplicate barcode configuration copy, preserving the public calibration contract and semaphore ownership.
- Preserves v1 binary MQTT packet format and control/ACK semantics.
- Tests pass in GCC/Clang host environment, including 70/70 dashboard cases.
- Final Week 13 compliance and target hardware results remain unverified; see checklist and gaps.
