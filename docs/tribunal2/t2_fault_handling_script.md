# Tribunal 2: Fault handling and invalid data — discussion script (G7)

**New requirement (Tribunal 2):** If any part of the system fails or provides invalid data, the system must respond in a safe and predictable way. We must identify how this affects our system and technically justify the response. Implementation is not required.

**Buddy mapping used in this script** (from the module split in the code and the Tribunal 1 buddy assignments):

| Speaker | Area | Main files |
|---|---|---|
| Buddy 1 | Telemetry, MQTT, binary protocol, dashboard | `modules/telemetry/buddy1*.c`, `lib/libnet/lwip/mqtt_utk.c`, `dashboard/server/*` |
| Buddy 2 | Motion, motors, encoders, controller task and RTOS timing | `drivers/motor.c`, `drivers/encoder.c`, `modules/motion/*`, `app/task_controller.c` |
| Buddy 3 | Line IR sensors and barcode | `drivers/ir.c`, `modules/line_barcode/*`, `modules/barcode_scanning/*` |
| Buddy 4 | IMU, hump, lift and downhill | `modules/terrain/*`, `modules/motion/downhill.c` |
| Buddy 5 | Ultrasonic, servo, obstacle bypass | `modules/obstacle/*` |

**How to read the cues.** Lines in square brackets are screen-share cues: open that file at those lines before the next line is spoken. All paths are from the repo root (`inf2004-g7-v15.1-console-update-20261111/`). Every line range was checked against the code in that folder on 23 Nov 2026. If anyone edits these files before the tribunal, re-check the line numbers.

**What is implemented and what is proposed.** The buddies say "this is in the code" for implemented behaviour and "our proposed response" for gaps. Proposals are design responses only (the requirement says implementation is not required).

**Length.** The core script is about 6,600 spoken words, roughly 40 to 44 minutes at a normal pace. Blocks marked OPTIONAL add about 6 minutes; skip them to save time, or keep them ready as answers if the examiner asks.

---

## Part 1. Opening: what "safe" means for our car

**Buddy 1:** Okay, so the new requirement is basically: if any part of the system fails, or gives us data we can't trust, the car has to do something safe and predictable. Not just carry on as if the number was correct. And we have to justify the response for our system specifically. So I think we should start by agreeing what "safe" actually means on this robot, because everything after depends on it.

**Buddy 2:** Yeah. For us the safe state is simple: both motors at zero output, and the controller in a state that will not drive again by itself. So stopping is always a safe fallback. The question for each failure is whether stopping is the *best* response, or whether we can keep going in a limited way.

**Buddy 1:** And the requirement gives us four kinds of response: continue with limited operation, stop the affected function, attempt recovery, or require intervention. I think we should tag every failure we talk about with one of those four.

**Buddy 2:** Agreed. Let me show the controller states first, because that's where the four responses actually live.

[Share screen: app_program/app/task_controller.c, lines 27 to 38]

**Buddy 2:** These are the nine controller states. The ones that matter here are STOPPED, NAV_SAFETY_STOP, ERROR, REMOTE_HOLD and LIFTED. STOPPED and NAV_SAFETY_STOP are recoverable stops: the operator has to press STOP and then START again. ERROR is for hardware or OS-level faults, and the comment says it stays latched until reset.

**Buddy 3:** So that's already a mapping: STOPPED and NAV_SAFETY_STOP are "stop the function, operator recovers", ERROR is "requires intervention", and LIFTED is "attempt recovery".

**Buddy 2:** Exactly. And the recovery rule is enforced in one place.

[Share screen: app_program/app/task_controller.c, lines 482 to 495]

**Buddy 2:** `remote_reset_after_stop`. An explicit STOP aborts the bypass, resets the run, forgets the queued navigation action and clears the fault marker. But look at line 494: if the prior state was ROBOT_ERROR, it returns ROBOT_ERROR. So STOP can clear a navigation safety stop, but it can never clear a hardware error. And the comment at line 484 says STOP never authorises movement. Only a later START can.

**Buddy 4:** Which matters, because otherwise a sensor that's failing on and off could restart the car on its own.

**Buddy 2:** Right. Now, "motors at zero" sounds obvious, but we actually have two different stop actions at the driver level.

[Share screen: app_program/drivers/motor.c, lines 60 to 97]

**Buddy 2:** `motor_stop` writes compare value zero to all four pins: GP8, GP9, GP10, GP11. Both H-bridge inputs low means the motor coasts. `motor_brake`, from line 83, writes `MOTOR_PWM_WRAP + 1` to all four pins. A compare level above the wrap holds the output high for the whole period, so both inputs are high, which shorts the motor windings through the bridge: a short brake. We hold that for `APP_JUNCTION_BRAKE_MSEC`, 200 ms, then release to coast.

**Buddy 5:** Wait, so when we say "PWM zero", the brake isn't actually zero.

**Buddy 2:** Correct, and we should be honest about that. The commanded duty we publish is zero, but on a brake the pins are high for up to 200 ms. But for an emergency path, like operator STOP, we skip the brake. `motion_stop_now` clears `g_brake_cycles` and calls `motor_stop` directly. So operator STOP is a straight coast to zero.

**Buddy 1:** And at boot?

**Buddy 2:** `motor_init`, lines 47 to 50 in the same file, sets all four compare values to zero before enabling the slices. And the live-control build boots in REMOTE_HOLD. So a reset, for any reason, lands us in the safe state.

---

## Part 2. RTOS view: priorities, freshness and the controller loop

**Buddy 2:** Before we go sensor by sensor, the RTOS side, because most of our detection is actually "is this data fresh", not "is this value wrong".

[Share screen: app_program/app/app_config.h, lines 17 to 32]

**Buddy 2:** In µT-Kernel a lower number is a higher priority. IR task is priority 5, controller and obstacle are 6, IMU is 7. The telemetry task and the barcode decoder worker are priority 9, the lowest. The controller period is 50 ms, and line 32 is the key one: `APP_IR_MAX_AGE_MSEC` is 200.

**Buddy 3:** So every shared reading has a timestamp. My IR task publishes a snapshot with `timestamp_msec` and a `b_valid` flag every loop, and the controller compares the timestamp against the kernel clock.

[Share screen: app_program/app/task_controller.c, lines 978 to 989]

**Buddy 3:** Here. If the reading is not valid, or it's older than 200 ms, the controller calls `motion_apply(LINE_UNCERTAIN)` first, which is the stop, then prints, reports fault 2, and returns ROBOT_STOPPED.

**Buddy 4:** Why 200 and not, like, 50?

**Buddy 3:** Because the IR task asks for a 1 ms delay but under Wi-Fi load we logged about 16 ms between samples. 200 ms is more than ten of those gaps, so normal jitter never trips it, but a dead task does. At 30 RPM the wheel is 70 mm diameter, so about 220 mm per revolution, half a rev per second, so roughly 110 mm per second. 200 ms of blind driving is about 22 mm. The line is 18 mm wide, so that's about one line width. That's the budget.

**Buddy 2:** And the reason the IR data goes stale is usually not the sensor, it's starvation. That's why there's a diagnostic in the IR task.

[Share screen: app_program/modules/line_barcode/task_ir.c, lines 57 to 69]

**Buddy 3:** If the loop gap reaches `APP_IR_STALL_REPORT_MSEC`, 100 ms, it prints "Task stalled". So when we see "IR data stale" we can tell whether the task was starved or the data was actually invalid.

**Buddy 1:** Can I bring up the mutex? Because my telemetry task reads the controller's state through the same remote mutex.

**Buddy 2:** Yeah, that's a good one.

[Share screen: app_program/app/task_controller.c, lines 509 to 542]

**Buddy 2:** Every period the controller tries to read the STOP and START mailbox with `tk_loc_mtx(g_remote_mutex, APP_REMOTE_LOCK_WAIT_MSEC)`, which is a 2 ms wait. One miss is just contention with your task, so the request stays queued and is read next period. But if it misses `APP_REMOTE_LOCK_MISS_LIMIT` times in a row, which is 4, it forces `*p_stop = 1`. So a stuck mailbox becomes a stop after about 4 periods, roughly 200 ms.

**Buddy 1:** And the reason that's the right response: if the controller can't read the mailbox, it can't see an operator STOP. So it's not safe to keep driving with a deaf mailbox.

**Buddy 5:** What about priority inversion? Your task is priority 9 and it holds a lock that a priority 6 task wants.

**Buddy 1:** All our mutexes are created with `TA_INHERIT`.

[Share screen: app_program/modules/telemetry/buddy1_rt.c, lines 248 to 255]

**Buddy 1:** Line 249. `TA_TFIFO | TA_INHERIT`. While the controller waits on a mutex my priority 9 task holds, the kernel raises my task to the waiter's priority, so the IMU task at 7 can't pre-empt me in the middle and stretch the hold time. The motion, IR, terrain and remote mutexes are all created with `TA_INHERIT` too.

**Buddy 2:** So, honest gaps on the RTOS side. First: there is no hardware watchdog.

**Buddy 3:** Wait, really? I thought we had one.

**Buddy 2:** We touch the watchdog block, but only its tick generator.

[Share screen: app_program/drivers/encoder.c, lines 194 to 195]

**Buddy 2:** That register, defined at line 20 as `0x4005802C`, is the watchdog TICK register. We use it to clock the microsecond timer for the encoder edge times. The watchdog counter itself is never enabled. So if the controller task hangs, say a deadlock or a hard fault in one task, the PWM hardware keeps outputting the last compare value. The car would keep driving at the last duty.

**Buddy 4:** That's actually the worst failure we have, then.

**Buddy 2:** It is. So our proposed response: enable the RP2040 hardware watchdog with a timeout of 250 ms, five controller periods, and feed it only from the controller task, only at the end of a cycle that finished normally. If it expires, the chip resets. And we already showed that a reset lands in a safe state: `motor_init` zeroes all four pins and we boot into REMOTE_HOLD. Also, during the reset the RP2040's GPIO pads come up as inputs with pull-downs, so the H-bridge inputs read low, which is coast.

**Buddy 1:** And we can write the reason into a watchdog scratch register before... well, the watchdog won't let us write before, but on boot we can check the reset reason and send it in STATUS, so the dashboard shows "rebooted by watchdog" and the operator knows to look. That's "requires intervention", because the car comes back in REMOTE_HOLD and needs a fresh START.

> **[OPTIONAL — cut if short on time, about 53 s]**

**Buddy 2:** One thing we'd have to change for that: the debugger. The watchdog should pause on debug, otherwise breakpoints reset the board. The second gap: the controller loop uses `tk_dly_tsk(APP_CONTROLLER_PERIOD_MSEC)`, which is a relative delay.

[Share screen: app_program/app/task_controller.c, lines 623 to 627]

**Buddy 2:** So the real period is 50 ms plus the cycle's execution time. It drifts, it doesn't miss. We have no on-target measurement of the worst-case loop time, and that's on our list. The third gap: task stacks are 4096 bytes each, `APP_TASK_STACK_BYTES` at line 7 of app_config.h, and we have never measured stack high-water marks. Our min-bits document says so too. Proposal: fill stacks with a known pattern at boot and report the deepest use per task in the SENSOR frame, so a near-overflow shows up as data before it becomes a hard fault.

> **[END OPTIONAL]**

---

## Part 3. The fault code contract and the bit budget

**Buddy 1:** Before the sensors, I want to show how a fault actually gets reported, because "predictable" also means the operator sees the same code for the same failure every time.

[Share screen: app_program/dashboard/server/state.js, lines 5 to 10]

**Buddy 1:** This is the dashboard's table of controller fault codes. 2 stale IR, 3 IR read failure, 4 controller clock failure, 5 obstacle sensor fault, 6 decode timeout, 7 no usable action or junction clearance, 8 turn timeout or angle limit, 9 downhill overspeed, 10 IMU unavailable, 11 stale downhill IMU, 12 scan handoff overflow, 13 unmapped barcode, 14 robot lifted. Each one is sent as EVENT kind 21, safe stop, with the code as the value.

**Buddy 2:** And here's the bit-level catch.

[Share screen: app_program/app/task_controller.c, lines 61 to 71]

**Buddy 2:** `g_control` packs the staged event, value, sequence and fault into exactly one 32-bit word. `fault` is 4 bits and `value` is 4 bits. So fault codes can go 0 to 15. We use 2 to 14. That means there is exactly one free code left, 15.

**Buddy 4:** So if we propose more than one new fault, we have to widen the field.

**Buddy 2:** Yes. Today the word is exactly full: event 5, value 4, sequence 16, fault 4, and 3 one-bit flags, 32 bits. The fault code also travels in `value`, because `report_fault` calls `nav_report(NAV_EVENT_SAFE_STOP, reason)`. So both `fault` and `value` would go to 5 bits, 34 bits, and the struct becomes two words. That's 4 extra bytes of RAM, it's nothing, but it's a contract change: the dashboard table and the EVENT value must change together. I'd rather say that up front than pretend it's free.

**Buddy 1:** Same story for readiness.

[Share screen: app_program/app/task_controller.c, lines 341 to 406]

**Buddy 1:** `g7_controller_remote_blockers` builds an 8-bit mask: 1 MQTT offline, 2 not in remote hold, 4 STOP or START pending, 8 IMU uncalibrated or stale, 16 IR invalid, 32 IR stale, 64 controller unavailable, obstacle sensor fault or non-zero PWM, and 128 barcode scanner not armed. All 8 bits are used, and it goes on the wire as one byte in STATUS. START is refused with NOT_READY while any bit is set. So any sensor that is broken at boot simply blocks START. That's our "requires intervention" for boot-time faults.

> **[OPTIONAL — cut if short on time, about 26 s]**

**Buddy 3:** And bit 64 is overloaded, three different causes share it.

**Buddy 1:** Yeah, that's a fair criticism. The dashboard can't tell "obstacle sensor fault" from "PWM not zero" from that bit alone. It can from the SENSOR frame, but not from STATUS. If we add a blocker we'd widen it to 16 bits, which is one more byte in a 14-byte STATUS payload.

> **[END OPTIONAL]**

---

## Part 4. Line IR sensors

**Buddy 3:** Okay, my sensors. The two line sensors are digital, GP6 left and GP2 right. A digital input can't tell you it's broken. A disconnected wire, a dead sensor or a sensor stuck on black all just read as 0 or 1. So we can't detect a stuck IR sensor directly. What we can do is make sure that the *consequence* of a wrong reading is bounded.

**Buddy 5:** So what happens if one sensor gets stuck on white, both reading white?

**Buddy 3:** Both white is "line lost", and Buddy 2's motion module handles that with a staged search.

[Share screen: app_program/modules/motion/motion.c, lines 119 to 143]

**Buddy 2:** After `MOTION_SEARCH_ROLL_MM`, 50 mm, with no sensor on the line, the car keeps driving but steers gently towards the side it last saw the line, steer 0.2. That's limited operation. If it still hasn't seen the line at `MOTION_SEARCH_AFTER_MM`, 100 mm, it stops and pivots towards that side up to `MOTION_SEARCH_SWEEP_DEG`, 120 degrees, then back the other way.

[Share screen: app_program/modules/motion/motion.c, lines 848 to 866]

**Buddy 2:** And if both sweeps fail, it sets SEARCH_GAVE_UP and stays stopped, with `drive_wheels(LINE_UNCERTAIN)`.

**Buddy 3:** So for a sensor stuck white: limited operation for 50 mm, recovery attempt up to 100 mm plus the pivots, then stop. For a stuck black sensor, one side black all the time looks like the line is permanently under that sensor, so it steers into a circle. Both stuck black looks like a junction, and navigation then turns or crosses, and the turn faults if it sweeps past 135 degrees, which is fault 8.

**Buddy 1:** Hang on, when it gives up the search, does the dashboard know?

**Buddy 3:** ...No. That's a gap. Look at lines 858 to 866. The motion module stops the wheels, but the controller is still in LINE_FOLLOWING. There's no `report_fault`. So the dashboard sees zero speed and no event. The operator just sees a car that stopped for no reason.

**Buddy 2:** That's a good catch. Our proposed response: when the search gives up, motion sets a flag, the controller reads it and moves to ROBOT_STOPPED with a new fault code "line not found". That's code 15, our last free code. Recovery is STOP, place the car on the line, START.

**Buddy 4:** And there's an even worse case. The search counts distance with encoders.

**Buddy 2:** Yes. Hold that thought, I'll come back to it in the encoder section, because it's the most important gap we found.

**Buddy 3:** Then the stale data path, which we showed: invalid or older than 200 ms is a stop with fault 2, and if `ir_shared_get_latest` returns an error, line 959 onwards, it's ROBOT_ERROR with fault 3, latched. The difference is deliberate. Stale data can be a temporary CPU starvation, so it's a recoverable stop. A failed read of the shared snapshot means the mutex or kernel object is broken, which won't fix itself, so it latches.

> **[OPTIONAL — cut if short on time, about 19 s]**

**Buddy 5:** And during my bypass?

[Share screen: app_program/app/task_controller.c, lines 1310 to 1326]

**Buddy 3:** `handle_obstacle` checks the same thing first. If IR is unavailable during the bypass, it stops, calls `obstacle_abort`, and goes to STOPPED. Because the bypass ends by finding the line with these sensors, so without IR the bypass can't end safely.

> **[END OPTIONAL]**

---

## Part 5. Barcode sensor and navigation

**Buddy 3:** The barcode sensor is analog on GP27, ADC channel 1. Here invalid data is actually common: bad prints, lighting, the robot not square to the code. The design decision is: a barcode that can't be decoded never stops the robot.

[Share screen: app_program/app/app_config.h, lines 220 to 242]

**Buddy 3:** `APP_NAV_STRAIGHT_DEFAULT` is 1: a junction with no usable action is crossed straight. `APP_NAV_SCAN_FAIL_STOP` is 0: an unmapped letter or an overwritten result is ignored and the car keeps following. Line 237 literally says a barcode that could not be decoded never stops the robot.

**Buddy 4:** Is that actually safe though? Going straight when you were supposed to turn?

**Buddy 3:** It's safe, it's not correct. Going straight at a junction keeps the car on a line, at normal speed, with every other check still active. Stopping on every bad decode would make the car stop in the middle of the course, which is worse for the run and no safer. So this is "continue with limited operation", and the limit is: we lose one routing decision.

**Buddy 1:** But there is one case where it does stop.

**Buddy 3:** Yeah, decode timeout. If the car reaches a junction while the decoder is still working, it stops and waits up to `APP_NAV_DECODE_WAIT_MSEC`, 8 seconds, or 1.5 s after the decoder goes idle. If no letter comes, navigation reports reason 5.

[Share screen: app_program/app/task_controller.c, lines 1252 to 1264]

**Buddy 3:** Reason 5 maps to fault 6, decode timeout, and the controller returns NAV_SAFETY_STOP. That's a recoverable stop.

**Buddy 2:** Now the low-level side, because the barcode pipeline has its own self-recovery, which I think we should claim properly.

[Share screen: app_program/modules/barcode_scanning/bc_fast.c, lines 254 to 281]

**Buddy 3:** The barcode ADC is sampled by a 2 ms alarm ISR into a 64-entry queue. If no sample arrives for 25 sample periods while armed, `bc_fast_watchdog` stops and re-arms the alarm and logs it. That's "attempt recovery", done automatically, because losing the sampler silently would mean every later barcode is missed.

[Share screen: app_program/modules/barcode_scanning/bc_fastq.c, lines 46 to 79]

**Buddy 3:** And the queue never blocks the ISR. If it's full, line 46, the push increments `g_drop_count` and returns. Then on the next pop, lines 74 to 79, if a drop happened, that sample is marked `valid = 0`. So a scan with a missing sample is invalidated, not decoded from incomplete data. That's exactly the requirement: we don't continue as if the value were correct.

> **[OPTIONAL — cut if short on time, about 35 s]**

**Buddy 5:** What about the sensor sitting on black at boot? Then the white reference is wrong.

[Share screen: app_program/modules/barcode_scanning/bc_wave.c, lines 88 to 94]

**Buddy 3:** `BC_WAVE_WHITE_MAX_DARKNESS` is 800. White reads about 150 to 250 on this sensor. If the learned white is darker than 800, learning repeats instead of arming. Until it arms, readiness bit 128 is set and START is refused. And line 94: `BC_WAVE_CAPTURE_WATCHDOG_USEC`, 20 seconds. A capture must end by its 16 s or 8192-sample limit. If one is still open after 20 s it's forced to DONE and re-armed.

> **[END OPTIONAL]**

---

## Part 6. Encoders and motors

**Buddy 2:** Okay, encoders. This is where I think our biggest gap is.

[Share screen: app_program/drivers/encoder.c, lines 145 to 177]

**Buddy 2:** The encoder ISR runs at interrupt priority 2. On each A edge it reads B and adds plus or minus one.

**Buddy 4:** So what does a broken wire look like?

**Buddy 2:** Two cases. If A is disconnected, the pull-up holds it high, no edges, count frozen, measured speed goes to zero. If B is disconnected, B reads 1 always, so a rising A edge gives plus one and a falling A edge gives minus one. They cancel. The count barely moves even though the wheel turns. Both look like a stopped wheel.

**Buddy 4:** And the speed controller sees a stopped wheel and pushes harder.

**Buddy 2:** Yes, but bounded. That's implemented.

[Share screen: app_program/modules/motion/motion.c, lines 74 to 82]

**Buddy 2:** The stall rescue. A wheel commanded to turn but measured below 1 RPM gets 1 % more duty every period, capped at `MOTION_STALL_MAX_PERCENT`, 15 %.

[Share screen: app_program/modules/motion/motion.c, lines 66 to 68]

**Buddy 2:** And the PI integrator trim is clamped to `MOTION_TRIM_MAX_PERCENT`, 12 %. So a dead encoder can't make the motor run away to 100 %. The extra duty is bounded at 15 plus 12 above the feed-forward.

**Buddy 3:** So that's fine then?

**Buddy 2:** For the motor, yes. But here's the thing I said I'd come back to. The lost-line search measures distance with the encoders.

[Share screen: app_program/modules/motion/motion.c, lines 809 to 826]

**Buddy 2:** `white_counts` is incremented by `moved`, which comes from `search_travel()`, which reads the encoder counts. If the encoders are dead, `moved` is zero, `white_counts` never reaches the 50 mm or 100 mm thresholds, and the search never starts. So with dead encoders and the line lost, the car drives straight off the course. The one safety net for "line lost" depends on a sensor that can fail silently.

**Buddy 5:** Oh. That's bad.

**Buddy 1:** And the ODOM telemetry would also show zero distance, so the route on the dashboard just stops while the car is moving.

**Buddy 2:** Right. So our proposed response has two parts. One: an encoder plausibility check. If a wheel's commanded duty is above the deadband, and its stall boost has reached the 15 % cap, that's 15 periods, 750 ms, and its count still hasn't changed, while the *other* wheel is turning, then that encoder is faulty. Today we only print "Wheel stalled" at that exact moment, lines 1061 to 1071. We'd turn that print into a latched fault and go to ROBOT_ERROR. ERROR, not STOPPED, because a broken encoder wire won't heal, and every distance-based function depends on it: the junction confirmation, the 134 mm centring, the pivot angle, the lost-line search.

**Buddy 4:** Why require the other wheel to be turning?

**Buddy 2:** Because on a tight curve the inner wheel can legitimately be stopped, and a jammed wheel with both encoders fine also looks stalled. If both counts are frozen, it's more likely the car is physically stuck, and the stall boost is already the bounded response for that. Two: a time-based backup for the lost-line search. If both IR sensors read white for longer than, say, 1.5 s while driving, which is about 165 mm at 110 mm/s and well past the 100 mm search threshold, stop regardless of what the encoders say.

**Buddy 3:** That's a nice argument actually, two independent sources so one failure can't disable both.

> **[OPTIONAL — cut if short on time, about 36 s]**

**Buddy 2:** Yeah. And to show we already do this kind of thing for overspeed:

[Share screen: app_program/modules/motion/downhill.c, lines 197 to 216]

**Buddy 4:** That's mine, the downhill governor. While it's active, if either wheel measures above `DOWNHILL_RUNAWAY_RPM`, 40, for `DOWNHILL_OVERSPEED_COUNT`, 10 periods, so half a second, it latches a fault. The controller reports that as fault 9 and goes to STOPPED.

[Share screen: app_program/modules/motion/motion.c, lines 544 to 565]

**Buddy 2:** And in `drive_wheels`, the overspeed check runs *before* the PI and stall rescue. The comment on line 549 says "fail closed before PI or stall rescue can increase duty". So once we've decided it's a runaway, nothing later in the same period can push duty back up.

> **[END OPTIONAL]**

---

## Part 7. IMU: calibration, stale data, false humps and lift

**Buddy 4:** Okay, the IMU. It's the LSM303DLHC on I2C0. I'd split it into boot failure, runtime failure, and data that's valid but misleading.

[Share screen: app_program/modules/terrain/task_imu.c, lines 184 to 234]

**Buddy 4:** Boot. If the I2C device won't open, or `lsm303_init` fails, the task prints and sleeps forever, `tk_slp_tsk(TMO_FEVR)`. It never calibrates. Then it loops calibration: if calibration fails, it retries every `IMU_CALIBRATION_RETRY_MSEC`, 500 ms, and tells you to keep the robot still and level.

**Buddy 1:** And until it calibrates, my readiness bit 8 is set, so START is refused.

**Buddy 4:** Right. So a dead IMU at boot is "requires intervention", and it's safe because the car can't start.

[Share screen: app_program/modules/terrain/task_imu.c, lines 358 to 381]

**Buddy 4:** What counts as a valid calibration: 50 samples, each sample's gravity magnitude between 800 and 1200 mg, and no axis may move more than 50 mg from the first sample. We don't accept a reference that was taken while moving.

**Buddy 5:** What about mid-run?

[Share screen: app_program/modules/terrain/task_imu.c, lines 445 to 480]

**Buddy 4:** Each 20 ms sample starts as `b_valid = false`. It's only set true if the accelerometer read succeeded and the tilt computation returned E_OK. If the read fails, the snapshot is published as invalid. And line 478: an invalid sample resets `level_count`, so invalid data can't be counted towards finishing a hump.

[Share screen: app_program/app/task_controller.c, lines 717 to 753]

**Buddy 2:** And on the controller side, two thresholds. Lines 721 to 722: data is only used if it's at most 200 ms old. And lines 745 to 752: if the robot is following or in a bypass and there's been no fresh IMU data for 1000 ms, it's a stop, "SAFE STOP: IMU unavailable", fault 10.

**Buddy 3:** Why let it drive for a whole second without the IMU?

**Buddy 4:** Because the IMU isn't needed to follow the line. It's for humps, lift and downhill. So for up to a second we run in limited operation: we just don't use stale IMU data. After a second, we can no longer detect being lifted, and that's a real hazard, so we stop. There's also a hardware reason for picking a full second.

[Share screen: device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h, lines 32 to 34]

**Buddy 4:** The I2C driver's timeout is 1000 ms per transaction. A single stuck transaction can block the IMU task for a full second, and each sample does two reads, accelerometer and magnetometer. So the controller's 1 s limit is roughly where one stuck transfer becomes a stop.

**Buddy 2:** Honestly, I'd say that's a coincidence we should tighten. A 1000 ms I2C timeout for a 20 ms task is too long.

**Buddy 4:** Yeah, agreed. Proposed: lower the I2C timeout to around 10 ms, two sample periods. Then a stuck bus shows up as invalid samples within 10 ms instead of blocking for a second. We'd also add bus recovery: after N failed reads, clock SCL nine times to release a slave holding SDA low, then re-init the LSM303. That's "attempt recovery". If it still fails, the controller's existing 1 s rule stops the car.

**Buddy 4:** Now the "valid but misleading" data, which I think is the most interesting. We have no gyroscope. So a jolt, a sideways shove, looks like tilt.

[Share screen: app_program/modules/terrain/task_imu.c, lines 545 to 553]

**Buddy 4:** First filter: if the total acceleration is more than 50 mg away from the calibrated gravity, `IMU_GRAVITY_TOLERANCE_MG`, the sample is being shoved, not tilted, and it's counted as rejected rather than used as tilt.

[Share screen: app_program/modules/terrain/task_imu.c, lines 54 to 61]

**Buddy 4:** Second: the climb check. A real hump lifts the axle. We integrate ds times sin(tilt) from the encoders, and if the climb is under `HUMP_MIN_CLIMB_MM`, 5 mm, the hump is discarded. Real humps climbed 8.4 to 16.4 mm, false ones from pivots, START and restarts climbed 0.4 to 2.1 mm. So 5 mm sits in the gap.

[Share screen: app_program/modules/terrain/task_imu.c, lines 729 to 740]

**Buddy 4:** And here it is in code: "Tipped but never lifted: a jolt, not a hump", logged as HUMP DISCARDED. So the response to invalid hump data is "discard and report nothing", rather than reporting a wrong height.

> **[OPTIONAL — cut if short on time, about 28 s]**

**Buddy 2:** Which, by the way, also depends on my encoders.

**Buddy 4:** Yeah, true. With dead encoders the climb is always zero, so every hump would be discarded. That fails in the safe direction though: we'd under-report humps, we wouldn't invent them. And the downhill governor only engages after 3 mm of measured climb, so it also just wouldn't engage, and the 40 RPM runaway check still applies.

> **[END OPTIONAL]**

**Buddy 5:** And lifting the car?

[Share screen: app_program/app/task_controller.c, lines 754 to 790]

**Buddy 4:** If fresh IMU data says lifted, more than `APP_LIFT_STOP_HEIGHT_MM`, 40 mm, with no floor under the line sensors, in IDLE, following, junction or obstacle, it calls `motion_stop_now` first, aborts the bypass, clears the barcode capture, and goes to ROBOT_LIFTED with fault 14.

[Share screen: app_program/app/task_controller.c, lines 848 to 898]

**Buddy 4:** Then `lift_try_resume`. It needs fresh IMU, floor contact, not lifted and height under 40 mm, continuously, for `APP_LIFT_RESUME_MSEC`, 1000 ms. Any break restarts the timer, lines 851 to 855. When it resumes, lines 882 to 891, it throws away line touches latched while the car was being carried, so a finger under the sensor doesn't become a steering command. That's our only automatic recovery, and the justification is the 1 s of stable floor contact: someone putting the car down is a deliberate act, and a second of stability is a strong signal.

**Buddy 1:** And RECAL_IMU, since the examiner will ask.

[Share screen: app_program/app/task_controller.c, lines 469 to 480]

**Buddy 4:** It's not implemented. Line 478: "No live IMU re-baselining API is available in this version", and it returns `B1_NOT_READY`. So the defined response is "refuse with a coded ACK", which is safe, the command does nothing. But the honest consequence is: if the IMU reference drifts, the only recovery today is a reboot with the car level. Proposed: accept RECAL_IMU only in REMOTE_HOLD with PWM zero, re-run the same 50-sample calibration with the same 800 to 1200 mg and 50 mg checks, and keep the old reference if the new one fails.

---

## Part 8. Ultrasonic sensor, servo and the bypass

**Buddy 5:** Okay, mine. The HC-SR04 and the SG90 servo. The ultrasonic has three outcomes per ping, not two.

[Share screen: app_program/modules/obstacle/ultrasonic.c, lines 70 to 129]

**Buddy 5:** Before triggering, `wait_echo_low` waits up to `OBS_ECHO_CLEAR_MSEC`, 50 ms, for ECHO to be low. If it's still high, the result is NO_ECHO. Then the echo is timed by a PWM slice in level mode, so the hardware counter only runs while ECHO is high, no GPIO interrupt. We wait up to `OBS_ECHO_TIMEOUT_MSEC`, 30 ms. If we counted ticks and the pulse ended under 4000 mm, it's OK. If ticks started but it didn't end, or it's beyond 4000 mm, it's OUT_OF_RANGE. If there were no ticks at all, line 124 says it: "wiring, power or sensor fault", NO_ECHO.

**Buddy 3:** So out of range and no echo are treated differently.

**Buddy 5:** That's the key design decision.

[Share screen: app_program/modules/obstacle/obs_config.h, lines 30 to 34]

**Buddy 5:** Line 33: "Persistent missing echoes are unverified rather than free space." `OBS_FAILSAFE_NO_ECHO` is 1.

[Share screen: app_program/modules/obstacle/task_obstacle.c, lines 205 to 248]

**Buddy 5:** In the front monitor, NO_ECHO resets the hit count and increments `no_echo`. At `OBS_NO_ECHO_LIMIT`, 5 in a row, it sets `OBS_FAULT_NO_ECHO`. OUT_OF_RANGE is different: it's stored as `OBS_FAR_MM`, 4000, so it is treated as clear. If we treated "no echo" as "clear", an unplugged sensor would look like an empty course forever, and the car would drive into the first box.

**Buddy 2:** How long does it take to detect?

**Buddy 5:** Worst case per ping is 50 ms echo-clear wait, plus 30 ms echo wait, plus the 60 ms `OBS_PING_PERIOD_MSEC` delay. 140 ms, times 5, is 700 ms. At 110 mm/s that's about 77 mm. Our stop distance is 250 mm, so even if the sensor dies exactly when a box is 250 mm away, we flag the fault with roughly 170 mm to spare.

[Share screen: app_program/modules/obstacle/obstacle.c, lines 447 to 459]

**Buddy 5:** `obstacle_has_fault` is true when the NO_ECHO bit is set. And the controller, in `handle_following`, checks it before anything else.

[Share screen: app_program/app/task_controller.c, lines 991 to 999]

**Buddy 5:** Stop, abort, "Obstacle sensor fault -> ERROR", fault 5, ROBOT_ERROR.

**Buddy 1:** So that's latched until reboot, even if the echoes come back.

**Buddy 5:** Yes, and I've been thinking about whether that's right. The fault bit itself clears when an echo returns, line 227, but the controller state stays ERROR. I think latching within a run is correct, because an intermittent connector will drop out again. an intermittent connector that drops out once will drop out again, maybe when we're 200 mm from a box. But needing a power cycle is harsh. Proposed: allow STOP then START to clear it, but only if the monitor has seen, say, 10 consecutive valid pings in REMOTE_HOLD. So it's "requires intervention" plus a verified recovery, not just a reset.

**Buddy 4:** That needs a counter.

**Buddy 5:** A 4-bit counter next to the existing monitor bit-fields. The monitor struct in task_obstacle.c is already `next:2`, `filled:2`, `hits:2`, `no_echo:3`, with an `#error` range guard at lines 36 to 38.

**Buddy 3:** What about the servo? It's open loop.

**Buddy 5:** Right, an SG90 has no position feedback. We send a pulse width and assume it got there. And honestly:

[Share screen: app_program/modules/obstacle/obs_state.h, lines 36 to 41]

**Buddy 5:** There's an `OBS_FAULT_SERVO` bit, 0x08, and an `OBS_FAULT_LINE_LOST` bit, 0x04. Neither is ever raised anywhere in the code. They're defined but unused. So a jammed or unplugged servo is not detected.

**Buddy 1:** So what does the planner see then?

**Buddy 5:** If it's stuck facing front, every bearing reads the box, so both sides look blocked. That's the "both sides blocked" path: wait and rescan, up to 10 rescans, then NO_ROUTE fault, ERROR. So it fails safe, but slowly, and with the wrong reason. Proposed response: a plausibility check on the scan. When the servo is at ±60 degrees or more, the readings should differ from the front reading in almost any real layout. If every bearing in a sweep agrees within `OBS_SCAN_AGREE_MM`, 50 mm, the servo probably didn't move, so raise `OBS_FAULT_SERVO`, which already has its bit reserved. That costs zero new RAM.

**Buddy 2:** And the bearing error from the servo, the skew. That's a different kind of invalid data, right? It's not failed, it's biased.

[Share screen: app_program/modules/obstacle/obs_config.h, lines 77 to 91]

**Buddy 5:** Yeah. The SG90 settles slightly short or past each target, the same way every time from the same direction. So a sweep in one direction shifts every bearing the same way. Our response, which is implemented: every scan move arrives moving right. A move left first swings to the left limit, waits `OBS_SERVO_SWING_SETTLE_MSEC`, 250 ms, for the spring-back, then comes right to the bearing. That turns a random error into a constant offset, and a constant offset can be calibrated. Also `OBS_SERVO_FRONT_USEC` is 1540, line 53, moved from 1500 because the zero read a few degrees right.

> **[OPTIONAL — cut if short on time, about 41 s]**

[Share screen: app_program/modules/obstacle/obs_config.h, lines 116 to 128]

**Buddy 5:** And the alternative is there too: `OBS_SCAN_DIRECTION` 0 does both directions and keeps the nearer reading per bearing, so the two errors cancel, at the cost of a longer scan, 16 to 20 s instead of 8 to 10. We chose one-way. The remaining offset is about 7 degrees, and that's on our "not yet verified" list. The safety side of it is covered by the 40 mm margin in the side choice: a bearing error makes us pick the slightly worse side, it doesn't make us hit the box, because the side watch keeps pinging the box while passing.

> **[END OPTIONAL]**

**Buddy 4:** And the bypass itself has timeouts?

**Buddy 5:** Yeah, the bypass phases have time and distance bounds. If one runs out it raises `OBS_FAULT_TIMEOUT` or `OBS_FAULT_NO_ROUTE` and the controller goes to ERROR via `drive.b_fault`, task_controller lines 1336 to 1341. A bypass that can't find the line again is not a state we want to keep driving in.

---

## Part 9. Network, protocol and the dashboard

**Buddy 1:** Okay, the network, which is probably where the examiner expects the most. The requirement says "if any part fails". The network will fail.

**Buddy 2:** So first question, does the car need the network to be safe?

**Buddy 1:** No, and that's a design principle, not luck.

[Share screen: app_program/app_main.c, lines 145 to 161]

**Buddy 1:** The telemetry task is started last, at the lowest priority, and if `b1_rt_start` fails, it prints "MQTT offline; robot tasks remain running". No task that writes PWM depends on MQTT.

**Buddy 3:** But we still stop when MQTT drops.

**Buddy 1:** Yes, and that was an actual team debate. We even drafted a change to keep driving on MQTT loss. Here's why we kept the stop.

[Share screen: app_program/app/task_controller.c, lines 642 to 675]

**Buddy 1:** If the state is following, obstacle, idle, junction or lifted, and `b1_mqtt_connected()` is false, it sets `stop_request`. Then `motion_stop_now`, clear the decoded letter, reset history, abort the bypass, forget the queued action, and go to REMOTE_HOLD. The justification: the dashboard STOP is the operator's only remote way to stop the car. While MQTT is down, that STOP can't arrive. Continuing autonomously would mean the car is running with no operator override. So we stop the function that depends on the operator, which is motion.

**Buddy 4:** And it never restarts by itself.

**Buddy 1:** Line 642, "Network failure must never resume a stopped robot." Reconnecting does nothing. START needs REMOTE_HOLD, a fresh START request, MQTT connected and all readiness blockers clear, lines 676 to 694.

**Buddy 2:** But I want to flag the honest problem with this block. Look at lines 649 to 656.

**Buddy 1:** Yeah. The `tm_printf("MQTT lost -> motors STOPPED")` runs *before* `motion_stop_now()`. With the USB console a print can block. NFR9 says zero the motors before status work. So that's a gap. The fix is a two-line reorder: stop first, print after. The IR-stale path already does it the right way round, line 982 before 984.

**Buddy 5:** How fast does the car know the link is gone, though? The flag doesn't update itself.

[Share screen: lib/libnet/lwip/mqtt_utk.c, lines 126 to 132]

**Buddy 1:** `B1_PING_PERIOD_MS` is 5000 and `B1_LINK_SILENCE_MS` is 15000. We send a PINGREQ every 5 s, the broker answers immediately, and if we hear nothing at all for 15 s, three ping periods, the link is declared down.

[Share screen: lib/libnet/lwip/mqtt_utk.c, lines 345 to 357]

**Buddy 1:** Line 347 is the silence check, and the comment at 348 explains the old bug: we used to ping only after 15 s of no *transmit*, and a busy robot always transmits, so it never pinged, and the link only dropped after 45 s.

**Buddy 2:** 15 s at 110 mm/s is about 1.6 m of driving. Is that acceptable?

**Buddy 1:** It's the worst case for a *silent* loss, like the hotspot freezing without dropping Wi-Fi. During those 15 s every on-board check still works, so the only thing missing is the operator's STOP. The longest uplink stall we saw on the phone hotspot was 4.3 s over 11,800 frames, the comment says that at line 130. Lower than about 10 s and we'd get false stops on a healthy hotspot, which is "unpredictable" in the other direction. And this is exactly why the README says the dashboard STOP is not an emergency stop.

[Share screen: README.md, lines 9 to 11]

**Buddy 1:** "Dashboard STOP is a network request, not an emergency stop. Keep a physical motor power cutoff within reach." That's our "requires intervention" for the case software can't cover.

[Share screen: lib/libnet/lwip/mqtt_utk.c, lines 166 to 177]

**Buddy 1:** And `discard()`, when the link drops: it closes the PCB, publishes link down, and line 173 empties the receive queue and the STOP queue. Line 174, "Never replay queued START on reconnect". So a START typed while the link was flaky can't execute later when it reconnects.

> **[OPTIONAL — cut if short on time, about 20 s]**

**Buddy 3:** You said STOP queue. STOP is separate?

[Share screen: lib/libnet/lwip/mqtt_utk.c, lines 283 to 295]

**Buddy 1:** Yeah. When a command arrives, if it decodes as STOP it goes into `stopq`, a separate 2-slot queue, otherwise into the 8-slot `rxq`. Line 290: "A saturated ordinary command ring cannot suppress STOP." And `b1_mqtt_receive_command` always takes from `stopq` first.

> **[END OPTIONAL]**

**Buddy 4:** What stops an old START, like a retransmitted one, from being accepted?

[Share screen: app_program/modules/telemetry/buddy1_rt.c, lines 172 to 199]

**Buddy 1:** The STATUS challenge. Every STATUS we send records its timestamp in an 8-entry ring. Every command except STOP must echo one of those timestamps, no older than 8 s. If it doesn't, line 177, it's rejected with `B1_STALE_COMMAND`, code 7, and counted as invalid.

> **[OPTIONAL — cut if short on time, about 11 s]**

[Share screen: app_program/modules/telemetry/buddy1.c, lines 233 to 244]

**Buddy 1:** And the 8 s check is here, line 240. The session ID is also boot-local, so a command from before a reboot fails the session check.

> **[END OPTIONAL]**

**Buddy 1:** Now, invalid data on the wire. Every frame has a 13-byte header and a CRC-8.

[Share screen: app_program/modules/telemetry/buddy1_protocol.c, lines 16 to 29]

**Buddy 1:** CRC-8, polynomial 0x07, initial value 0. That polynomial is x^8 + x^2 + x + 1. It catches every single-bit error and every burst of up to 8 bits, and because it has (x + 1) as a factor it catches every error with an odd number of flipped bits. A random corruption gets through with probability about 1 in 256.

> **[OPTIONAL — cut if short on time, about 19 s]**

**Buddy 3:** TCP already has a checksum though. Why bother?

**Buddy 1:** TCP's checksum protects the hop. Ours is end to end: it also catches our own bugs, a truncated publish, a broker or dashboard that mangles bytes, or the wrong topic. It's one byte per frame.

> **[END OPTIONAL]**

[Share screen: app_program/modules/telemetry/buddy1_protocol.c, lines 44 to 55]

**Buddy 1:** `b1_frame_decode` rejects the frame if it's shorter than the header, longer than `B1_MAX_FRAME`, 62 bytes, wrong magic 0xB1, wrong version, payload length over 48, total length not equal to header plus payload plus one, or the CRC doesn't match. It returns -1 and nothing is used. A rejected command just never runs, and the dashboard times out and shows no ACK.

[Share screen: app_program/modules/telemetry/buddy1_protocol.c, lines 130 to 133]

**Buddy 1:** And at bit level: the SNAPSHOT payload is 99 bits in 13 bytes, which is 104 bits. The top 5 bits of the last byte are unused, and the decoder requires them to be zero, mask 0xF8. Same for the route mapping: four 5-bit letters is 20 bits in 3 bytes, and `b1_unpack_mapping` rejects the frame if the top 4 bits, mask 0xF0, are not zero, and it rejects letters above 25 or duplicate letters. So a corrupted mapping can't load a wrong route.

[Share screen: app_program/dashboard/server/protocol.js, lines 18 to 35]

**Buddy 1:** The dashboard does the same checks in JavaScript, lines 20 and 21, and line 27 for the snapshot's unused bits. And the sentinels: 1023 in a 10-bit field means "unavailable", and 31 in the 5-bit letter field means "no letter". The dashboard turns those into null, lines 31 to 33, so it never draws a hump of 1023 mm.

**Buddy 2:** Is there anything you don't check?

**Buddy 1:** The flags byte in the header. Bit 0 is the "uncertain" flag, but the other 7 bits are reserved and neither side checks they're zero. That's a gap, and it's a one-line fix each side.

**Buddy 5:** And dropped frames?

[Share screen: app_program/modules/telemetry/buddy1.c, lines 21 to 29]

**Buddy 1:** The ODOM ring is 32 records, 10 bytes each, the EVENT ring is 24 records, 9 bytes each. When a ring is full, the oldest is overwritten, `dropped` is incremented, and `uncertain` is set. The comment: "never pretend event history is complete". `dropped` goes out in every HEARTBEAT.

[Share screen: app_program/modules/telemetry/buddy1_rt.c, lines 235 to 243]

**Buddy 1:** And since today, frames are only taken from the rings while `b1_mqtt_can_send()` is true, so while offline the data stays in the rings and is sent in order after reconnect, instead of being pushed into an 8-slot TX queue and lost. That's "continue with limited operation" for telemetry: bounded memory, counted loss.

> **[OPTIONAL — cut if short on time, about 39 s]**

**Buddy 4:** And the dashboard handles gaps how?

**Buddy 1:** It marks the route segment as approximate, dashed, when an ODOM sequence number is skipped, and bridges it at the next 500 mm checkpoint, which carries cumulative distance. So a lost update degrades the map, it doesn't corrupt it. And for staleness:

[Share screen: app_program/modules/telemetry/buddy1.c, lines 121 to 133]

**Buddy 1:** If any source, motion, line, terrain, mission or obstacle, hasn't updated in 1500 ms, its bit goes into the 5-bit stale mask in every SNAPSHOT. So the dashboard can grey out a value instead of showing a frozen number as if it were live.

> **[END OPTIONAL]**

---

## Part 10. Wiring and power

**Buddy 3:** Can we do wiring? Because some failures are just physical.

[Share screen: app_program/board/board_config.h, lines 16 to 35]

**Buddy 3:** Three warnings are in the board file itself. GP0 and GP1 are UART0 at boot and the left encoder takes them over, so a UART console build loses its console. GP28 is ADC2 at boot and must be kept free of the VBAT-sense jumper, otherwise the right encoder's B channel reads a voltage instead of a logic level. And line 33: HC-SR04 ECHO is 5 V, fit a voltage divider before GP5.

**Buddy 5:** And if the divider isn't there it doesn't fail safe, it damages the pin over time. The RP2040 isn't 5 V tolerant. That's a case where the response is a build-time check, not a runtime one. Our proposed check is a commissioning step: measure GP5 with a meter before the first run.

**Buddy 2:** A loose motor wire is actually covered by the encoder logic. If a motor stops turning, its encoder stops, stall boost hits the cap. With the plausibility check we proposed that becomes a fault. Today it's just a log line, the bounded duty, and eventually the navigation timeouts: a turn that can't reach its angle faults at the time limit, fault 8.

> **[OPTIONAL — cut if short on time, about 52 s]**

**Buddy 4:** Battery sag?

**Buddy 2:** If the battery sags far enough under motor load, the RP2040 resets. And again: a reset boots into REMOTE_HOLD with all four motor pins at zero. So a brownout ends in the safe state, it just needs a new START. What we don't have is a battery-voltage measurement, because GP28, the usual VBAT sense, is used by the encoder. Proposed: if we free an ADC pin, warn at a threshold and refuse START below it, as a new readiness condition. But that needs a ninth readiness bit, so STATUS would go to a 16-bit mask.

**Buddy 1:** And I2C wiring for the IMU: SDA or SCL loose gives read failures, which is the invalid-sample path we already covered, then the 1 s stop.

> **[END OPTIONAL]**

---

## Part 11. Closing: summary of responses

**Buddy 1:** Okay, I think we've covered it. Let me try to summarise, and correct me. Our principle is: motion only happens on fresh, valid data and a current operator authorisation. Any loss of either collapses to zero output, through one function, `motion_apply(LINE_UNCERTAIN)`, or `motion_stop_now` when it must be immediate.

**Buddy 2:** And the four response types map like this. Limited operation: bad barcode decode, go straight; stale IMU under 1 s, don't use it; dead encoder, bounded duty; telemetry offline, keep it in bounded rings. Stop the affected function: stale IR, IMU unavailable, MQTT loss, mutex misses, decode timeout, turn timeout, downhill overspeed. Attempt recovery: barcode sampler restart, capture watchdog, white relearn, lost-line search, lift resume, MQTT reconnect with backoff. Requires intervention: ERROR states, IMU or barcode failing at boot via readiness blockers, and the physical cutoff because dashboard STOP is not an e-stop.

**Buddy 3:** And the gaps we're putting forward, in priority order?

**Buddy 2:** One, the hardware watchdog, because a hung controller is the only failure that leaves the motors driving. Two, encoder plausibility plus a time-based lost-line limit, because dead encoders silently disable the lost-line search. Three, the print-before-stop in the MQTT path. Four, report the lost-line give-up as a fault, using code 15, our last free 4-bit code. Five, servo plausibility using the `OBS_FAULT_SERVO` bit that's already reserved. Six, a shorter I2C timeout with bus recovery. Seven, check the reserved header flag bits. And RECAL_IMU, if there's time.

**Buddy 4:** And the bit cost of all of that is small. The watchdog is a hardware register. The encoder check is two small counters. The lost-line timer is one 32-bit timestamp. The servo fault bit already exists. The only contract change is the fault field running out of codes, which we flagged.

**Buddy 5:** I think the point I'd make to close is that for most of our failures we already had a response, we just hadn't written it down as a requirement. The thing this exercise found is the cases where one sensor's failure quietly disables another sensor's safety check, like encoders and the line search. That's what we'd fix first.

**Buddy 1:** Yeah. Okay, that's us.

---

## Appendix (not spoken): failure → detection → response table

| Failure / invalid data | Detection (in code) | Response | Class | Status |
|---|---|---|---|---|
| Controller task hangs | None (no HW watchdog) | Propose 250 ms HW watchdog, fed only by controller; reset boots REMOTE_HOLD, motors zero | Intervention | Gap, proposed |
| IR data stale or invalid | Age > 200 ms or `!b_valid` (`task_controller.c:979-988`) | Stop, fault 2, STOPPED | Stop function | Implemented |
| IR shared-object read error | `ir_shared_get_latest` < E_OK | Stop, fault 3, ERROR latched | Intervention | Implemented |
| Line sensor stuck white / line lost | Encoder distance 50 / 100 mm (`motion.c:119-143`) | Roll-steer, pivot ±120°, stop | Recovery, then stop | Implemented; give-up not reported (gap) |
| Encoder wire broken | Measured RPM < 1 while commanded | Stall boost ≤ 15 %, trim ≤ 12 % | Limited | Implemented; no fault (gap) |
| Encoder dead + line lost | Not detected (search uses encoder travel) | Propose encoder plausibility fault + 1.5 s both-white timer | Stop | Gap, proposed |
| Downhill runaway | > 40 RPM for 10 periods (`downhill.c:197-216`) | Latched stop, fault 9 | Stop function | Implemented |
| Barcode undecodable / unmapped | Decoder result < 0 | Cross straight, EVENT 19 | Limited | Implemented |
| Barcode still decoding at junction | Wait 8 s / 1.5 s grace | Stop, fault 6, NAV_SAFETY_STOP | Stop function | Implemented |
| Barcode sampler stalls | 25 periods with no sample | Re-arm alarm | Recovery | Implemented |
| Barcode queue overflow | `g_drop_count` | Scan invalidated (`valid = 0`) | Limited | Implemented |
| IMU fails at boot | Init/calibration failure | START blocked (readiness bit 8) | Intervention | Implemented |
| IMU stale mid-run | > 200 ms not used; > 1 s stop | Fault 10, STOPPED | Limited, then stop | Implemented |
| I2C bus stuck | 1000 ms driver timeout | Propose 10 ms timeout + 9-clock bus recovery | Recovery | Gap, proposed |
| Jolt read as tilt / false hump | 50 mg gravity check; 5 mm climb check | Sample rejected / hump discarded | Limited | Implemented |
| Robot lifted | IMU lift > 40 mm, no floor | Stop, LIFTED, resume after 1 s on floor | Recovery | Implemented |
| RECAL_IMU requested | Command decode | NOT_READY ACK | Refuse | Not implemented; proposal given |
| Ultrasonic no echo | 5 consecutive NO_ECHO | Fault 5, ERROR latched | Intervention | Implemented; propose verified STOP/START recovery |
| Ultrasonic ECHO stuck high | 50 ms clear wait | Counts as NO_ECHO | Intervention | Implemented |
| Servo jammed | Not detected (`OBS_FAULT_SERVO` unused) | Propose flat-sweep plausibility check | Stop | Gap, proposed |
| Servo bearing bias | Known, systematic | One-direction approach, FRONT 1540 µs, 40 mm margin | Limited | Implemented; ~7° residual |
| Bypass cannot finish | Phase timeouts / NO_ROUTE | ERROR | Intervention | Implemented |
| MQTT link lost | Wi-Fi down, TCP error, or 15 s silence | Stop, abort, REMOTE_HOLD; no auto-resume | Stop function | Implemented; print-before-stop (gap) |
| Stale or replayed command | 8 s STATUS challenge, session | STALE_COMMAND ACK | Refuse | Implemented |
| Corrupted frame | Magic, version, length, CRC-8 | Frame dropped | Refuse | Implemented; header reserved bits unchecked (gap) |
| Telemetry backlog / offline | Ring full | Oldest overwritten, counted, `uncertain` | Limited | Implemented |
| Stuck command mailbox | 4 failed 2 ms locks | Stop | Stop function | Implemented |
| Brownout | RP2040 reset | Boots REMOTE_HOLD, motors zero | Intervention | Implemented by design |
| 5 V ECHO without divider | Not detectable at runtime | Commissioning check | Intervention | Procedure |
