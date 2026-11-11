# Physical acceptance — all items pending

No real car was driven or flashed during this implementation. Record console
and MQTT events, wheel targets/measured RPM/PWM, pitch, governor engagement and
the actual stop cause. Keep a physical cutoff accessible throughout.

- [ ] Support wheels: prove STOP writes zero output, measure request-to-output
  latency and wheel coast-down separately, and verify sustained HOLD.
- [ ] Repeat START -> STOP -> READY -> START at least ten times on hardware.
  Include closely spaced commands and stop during turns, waits, capture,
  decode, obstacle bypass and downhill travel.
- [ ] Disconnect/reconnect MQTT during motion: output stops, no automatic restart.
- [ ] Confirm old collision ACK permits recovery, while a new collision between
  STOP and START blocks readiness or stops the new mission.
- [ ] Test IR/IMU faults and task/mutex contention under normal Wi-Fi traffic.
  Measure worst-case controller/ISR execution time, scheduling jitter, queue
  depth, task stacks and heap high-water marks.
- [ ] Confirm a stationary, level startup reference. STOP on a tilted ramp must
  preserve it. Live IMU recalibration must return NOT_READY. Barcode calibration
  only over appropriate white, with dark-reference rejection checked.
- [ ] Map G to LEFT; execute first, second and at least five consecutive physical
  junctions. Repeat on independent G cards, then replace the action with another
  mapped letter. Test same-card noise, failed scans and unmapped letters.
- [ ] Place barcodes so results arrive before, during and near the junction wait
  deadline; verify automatic manoeuvre and bounded failure. Save capture/end,
  decode/end, controller time, sequence and nav state for each run.
- [ ] Measure wheel circumference/counts per revolution and effective track;
  validate junction confirmation, barcode clearance, centre travel and pivot
  line reacquisition on the actual tape geometry.
- [ ] Verify positive/negative nose-down pitch **while stationary**. Set
  APP_DOWNHILL_NOSE_DOWN_SIGN only after this measurement, then rebuild.
- [ ] Validate proportional short-brake PWM electrically and thermally at low
  power before supported ramp trials. Verify polarity and no reverse drive.
- [ ] Run flat, uphill, and multiple shallow downhill inclines, straight and both
  curves; record steady error, peak/rms error, ripple, brake duty and transition
  time. Determine the maximum incline/load with adequate brake authority.
- [ ] Test pitch noise, one-wheel humps, brief/prolonged IMU loss, normal encoder
  errors and genuine sustained runaway. Collision protection remains enabled;
  classify ramp-entry/braking events from measured jerk rather than suppressing
  impacts to make a trial pass.
- [ ] Follow downhill with a barcode, junction and obstacle. Confirm priorities,
  steering direction and reliable line reacquisition.
- [ ] Resolve OPEN precision/domain entries in the field ledger and assessor
  decisions about byte-aligned v1 wire fields before claiming full Week 13
  minimum-bit certification. Host and ELF evidence cannot supply these data.
