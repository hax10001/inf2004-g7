/** @file test_ct_controller_ready.c
 * @brief Host test: START readiness blockers, fault-source codes and
 *        command guards on the production task_controller.c (FR13, FR16).
 *
 * The controller source is included unchanged so its static state can be
 * set directly; every driver it calls is a stub whose result each test
 * case chooses. Each case checks one START blocker bit, one safety-stop
 * fault source (EVENT kind 21 value) or one remote-command guard that the
 * dashboard relies on (READINESS_REASONS and controllerFaults in
 * dashboard/server/state.js).
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define B1_ROBOT_APP 1
#define B1_LIVE_COMMANDS 1
#define APP_TEST 0
#define _PICO_RP2040_ 1
#include "app_program/app/task_controller.c"

#define CT_MUTEX_ID 42
#define CT_T0_MSEC 10000U
#define CT_PERIOD_MSEC 50U
#define CT_RUN_PWM 1000U

/* Stub state chosen by each test case. */
static uint32_t g_now = CT_T0_MSEC;
static ER g_clock_error = E_OK;
static uint8_t g_mqtt = 1U;
static bool g_lock_held = false;
static bool g_lock_fail = false;
static ER g_ir_error = E_OK;
static uint8_t g_ir_valid = 1U;
static uint32_t g_ir_time = CT_T0_MSEC;
static ER g_terrain_error = E_OK;
static uint8_t g_terrain_valid = 1U;
static uint32_t g_terrain_time = CT_T0_MSEC;
static bool g_calibrated = true;
static uint8_t g_scanner_ready = 1U;
static uint8_t g_scanner_can_rearm = 1U;
static bool g_obstacle_fault = false;
static uint32_t g_obstacle_aborts = 0U;
static uint16_t g_pwm = 0U;
static uint8_t g_governor = 0U;
static uint32_t g_relearns = 0U;
static line_follow_action_t g_action = LINE_UNCERTAIN;
static bool g_junction_black = false;
static ER g_latch_error = E_OK;

/** @brief Stub: last motor command; nonzero means the wheels may turn. */
void motor_last_command (uint16_t * p_left, uint16_t * p_right)
{
    *p_left = g_pwm;
    *p_right = g_pwm;
}

/** @brief Stub: the IMU snapshot chosen by the test. */
ER terrain_get_snapshot (terrain_snapshot_t * p_out)
{
    terrain_snapshot_t snapshot = {0};

    snapshot.b_valid = g_terrain_valid;
    snapshot.timestamp_msec = g_terrain_time;
    snapshot.b_floor_contact = 1U;
    *p_out = snapshot;
    return g_terrain_error;
}

/** @brief Stub: IMU level reference learned. */
bool terrain_is_calibrated (void)
{
    return g_calibrated;
}

/** @brief Stub: run reset in the motion module. */
void motion_reset_run (void)
{
}

/** @brief Stub: downhill governor terrain input (unused here). */
void motion_observe_terrain (terrain_snapshot_t const * p_snapshot,
                             uint32_t now)
{
    (void)p_snapshot;
    (void)now;
}

/** @brief Stub: downhill governor fault (0, 1 stale IMU, 2 overspeed). */
uint8_t motion_downhill_fault (void)
{
    return g_governor;
}

/** @brief Stub: ultrasonic sensor fault. */
bool obstacle_has_fault (void)
{
    return g_obstacle_fault;
}

/** @brief Stub: no obstacle ahead. */
bool obstacle_is_blocked (void)
{
    return false;
}

/** @brief Stub: count obstacle aborts. */
void obstacle_abort (void)
{
    ++g_obstacle_aborts;
}

/** @brief Stub: bypass never runs in these cases. */
obstacle_drive_t obstacle_bypass_step (uint32_t left, uint32_t right,
                                       uint32_t now)
{
    obstacle_drive_t drive = {0};

    (void)left;
    (void)right;
    (void)now;
    return drive;
}

/** @brief Stub: record the action; any action but UNCERTAIN drives. */
void motion_apply (line_follow_action_t action)
{
    g_action = action;
    g_pwm = (LINE_UNCERTAIN == action) ? 0U : CT_RUN_PWM;
}

/** @brief Stub: brake is a zero output. */
void motion_brake (void)
{
    motion_apply(LINE_UNCERTAIN);
}

/** @brief Stub: lost-line hint (unused). */
void motion_line_hint (int side)
{
    (void)side;
}

/** @brief Stub: obstacle scan centre (unused). */
void obstacle_set_line_offset_ddeg (int16_t offset_ddeg)
{
    (void)offset_ddeg;
}

/** @brief Stub: immediate zero output. */
void motion_stop_now (void)
{
    motion_apply(LINE_UNCERTAIN);
}

/** @brief Stub: wheel RPM drive (unused). */
void motion_drive (float left, float right)
{
    (void)left;
    (void)right;
}

/** @brief Stub: junction manoeuvre (unused). */
void motion_apply_maneuver (uint8_t action)
{
    (void)action;
}

/** @brief Stub: line position for the lost-line search (unused). */
void motion_line_seen (bool b_left, bool b_right)
{
    (void)b_left;
    (void)b_right;
}

/** @brief Stub: clear decoded barcode results. */
void g7_nav_clear_decoded (void)
{
}

/** @brief Stub: barcode handoff enabled at START. */
int32_t g7_nav_begin_run (void)
{
    return 1;
}

/** @brief Stub: barcode handoff re-enabled mid-run. */
int32_t g7_nav_continue_run (void)
{
    return 1;
}

/** @brief Stub: no scan result. */
int32_t g7_nav_take_scan (uint8_t * p_letter, uint8_t * p_action,
                          uint16_t * p_seq, uint32_t * p_ms,
                          uint32_t * p_done, uint8_t * p_pending)
{
    *p_letter = 0U;
    *p_action = 0U;
    *p_seq = 0U;
    *p_ms = 0U;
    *p_done = 0U;
    *p_pending = 0U;
    return 0;
}

/** @brief Stub: no decoded letter. */
int32_t g7_nav_take_decoded (uint8_t * p_letter, uint8_t * p_action,
                             uint16_t * p_seq)
{
    (void)p_letter;
    (void)p_action;
    (void)p_seq;
    return 0;
}

/** @brief Stub: wheels do not move. */
void encoder_read (encoder_wheel_t wheel, encoder_sample_t * p_out)
{
    encoder_sample_t sample = {0};

    (void)wheel;
    *p_out = sample;
}

/** @brief Stub: drop the running capture. */
void bc_wave_request_drop (void)
{
}

/** @brief Stub: no barcode guard. */
uint8_t bc_wave_navigation_guard_active (void)
{
    return 0U;
}

/** @brief Stub: no capture running. */
uint8_t bc_wave_capture_active (void)
{
    return 0U;
}

/** @brief Stub: capture suppression (unused). */
void bc_wave_set_suppressed (uint8_t b_suppressed)
{
    (void)b_suppressed;
}

/** @brief Stub: count white relearn requests. */
void bc_wave_request_relearn (void)
{
    ++g_relearns;
}

/** @brief Stub: no decode pending. */
uint8_t bc_wave_decode_pending (void)
{
    return 0U;
}

/** @brief Stub: scanner may be reset. */
uint8_t bc_wave_can_rearm (void)
{
    return g_scanner_can_rearm;
}

/** @brief Stub: scanner learned and armed. */
uint8_t bc_wave_is_ready (void)
{
    return g_scanner_ready;
}

/** @brief Stub: no side touches latched. */
ER ir_shared_take_sides (bool * p_left, bool * p_right)
{
    *p_left = false;
    *p_right = false;
    return E_OK;
}

/** @brief Stub: no junction latched. */
ER ir_shared_take_junction (bool * p_both)
{
    *p_both = g_junction_black;
    return g_latch_error;
}

/** @brief Stub: no both-black gap. */
ER ir_shared_take_both_gap (bool * p_gap)
{
    *p_gap = false;
    return E_OK;
}

/** @brief Stub: empty junction diagnostics. */
ER ir_shared_take_diag (ir_shared_diag_t * p_diag)
{
    ir_shared_diag_t const empty = {0U, 0U, 0U, 0U};

    *p_diag = empty;
    return E_OK;
}

/** @brief Stub: the IR snapshot chosen by the test (line centred). */
ER ir_shared_get_latest (ir_shared_snapshot_t * p_out)
{
    ir_shared_snapshot_t reading = {0};

    assert(!g_lock_held);
    reading.b_valid = g_ir_valid;
    reading.timestamp_msec = g_ir_time;
    *p_out = reading;
    return g_ir_error;
}

/** @brief Stub: task delay (unused). */
ER tk_dly_tsk (UINT ms)
{
    (void)ms;
    return E_OK;
}

/** @brief Stub: console output discarded. */
INT tm_printf (UB const * p_format, ...)
{
    (void)p_format;
    return 0;
}

/** @brief Stub: system clock, or the chosen clock error. */
ER tk_get_otm (SYSTIM * p_out)
{
    p_out->hi = 0;
    p_out->lo = g_now;
    return g_clock_error;
}

/** @brief Stub: the remote mutex is always created as ID 42. */
ID tk_cre_mtx (T_CMTX const * p_config)
{
    assert(TA_INHERIT == p_config->mtxatr);
    return CT_MUTEX_ID;
}

/** @brief Stub: lock, or fail every bounded wait when g_lock_fail. */
ER tk_loc_mtx (ID id, TMO timeout)
{
    assert(CT_MUTEX_ID == id);
    if (g_lock_fail && (TMO_FEVR != timeout))
    {
        return E_TMOUT;
    }
    assert(!g_lock_held);
    g_lock_held = true;
    return E_OK;
}

/** @brief Stub: unlock. */
ER tk_unl_mtx (ID id)
{
    assert(CT_MUTEX_ID == id);
    assert(g_lock_held);
    g_lock_held = false;
    return E_OK;
}

/** @brief Stub: MQTT link state. */
uint8_t b1_mqtt_connected (void)
{
    return g_mqtt;
}

/** @brief Put every input in its START-ready state, held, at time now. */
static void ready_world (void)
{
    g_now += CT_PERIOD_MSEC;
    g_clock_error = E_OK;
    g_mqtt = 1U;
    g_lock_fail = false;
    g_ir_error = E_OK;
    g_ir_valid = 1U;
    g_ir_time = g_now;
    g_terrain_error = E_OK;
    g_terrain_valid = 1U;
    g_terrain_time = g_now;
    g_calibrated = true;
    g_scanner_ready = 1U;
    g_scanner_can_rearm = 1U;
    g_obstacle_fault = false;
    g_pwm = 0U;
    g_governor = 0U;
    g_stop_pending = 0U;
    g_start_pending = 0U;
    g_reported_state = ROBOT_REMOTE_HOLD;
    g_junction_black = false;
    g_latch_error = E_OK;
}

/** @brief Assert one blocker mask and that START is refused for it. */
static void expect_blocked (uint8_t expected)
{
    assert(expected == g7_controller_remote_blockers());
    assert(0U == g7_controller_remote_ready());
    assert(B1_NOT_READY == g7_controller_remote_command(B1_CMD_START, NULL));
    assert(0U == g_start_pending);
}

/** @brief Test: before the mutex exists everything fails closed (FR16). */
static void test_uninitialised_fails_closed (void)
{
    uint8_t kind = 0xAAU;
    uint8_t value = 0xAAU;
    uint16_t seq = 0xAAAAU;

    ready_world();
    assert(g_remote_mutex <= E_OK);
    assert(CTRL_BLOCK_CONTROLLER == g7_controller_remote_blockers());
    g_mqtt = 0U;
    assert((CTRL_BLOCK_CONTROLLER | CTRL_BLOCK_MQTT_OFFLINE) ==
           g7_controller_remote_blockers());
    assert(CTRL_MISSION_STOPPED == g7_controller_mission_state());
    assert(0U == g7_controller_remote_hold_applied());
    assert(B1_NOT_READY == g7_controller_remote_command(B1_CMD_STOP, NULL));
    assert(B1_NOT_READY == g7_controller_remote_command(B1_CMD_START, NULL));
    g7_controller_navigation_report(&kind, &value, &seq);
    assert((0xAAU == kind) && (0xAAU == value) && (0xAAAAU == seq));
    puts("PASS FR16 no remote mutex: START blocked as CONTROLLER, mission "
         "STOPPED, commands refused");
}

/** @brief Test: each START blocker bit is set by its own condition only
 *  (FR13, FR16) and the mask is 0 when every input is ready. */
static void test_each_blocker_bit (void)
{
    ready_world();
    assert(0U == g7_controller_remote_blockers());
    assert(1U == g7_controller_remote_ready());

    ready_world();
    g_mqtt = 0U;
    expect_blocked(CTRL_BLOCK_MQTT_OFFLINE);

    ready_world();
    g_reported_state = ROBOT_LINE_FOLLOWING;
    expect_blocked(CTRL_BLOCK_NOT_HELD);

    ready_world();
    assert(B1_OK == g7_controller_remote_command(B1_CMD_STOP, NULL));
    assert(CTRL_BLOCK_CMD_PENDING == g7_controller_remote_blockers());
    assert(B1_NOT_READY == g7_controller_remote_command(B1_CMD_START, NULL));
    ready_world();
    g_start_pending = 1U;
    assert(CTRL_BLOCK_CMD_PENDING == g7_controller_remote_blockers());

    ready_world();
    g_calibrated = false;
    expect_blocked(CTRL_BLOCK_IMU);
    ready_world();
    g_terrain_error = E_TMOUT;
    expect_blocked(CTRL_BLOCK_IMU);
    ready_world();
    g_terrain_valid = 0U;
    expect_blocked(CTRL_BLOCK_IMU);
    ready_world();
    g_terrain_time = g_now - APP_IR_MAX_AGE_MSEC - 1U;
    expect_blocked(CTRL_BLOCK_IMU);
    ready_world();
    g_terrain_time = g_now - APP_IR_MAX_AGE_MSEC;
    assert(0U == g7_controller_remote_blockers());

    ready_world();
    g_ir_error = E_TMOUT;
    expect_blocked(CTRL_BLOCK_IR_INVALID);
    ready_world();
    g_ir_valid = 0U;
    expect_blocked(CTRL_BLOCK_IR_INVALID);
    ready_world();
    g_ir_time = g_now - APP_IR_MAX_AGE_MSEC - 1U;
    expect_blocked(CTRL_BLOCK_IR_STALE);
    ready_world();
    g_ir_time = g_now - APP_IR_MAX_AGE_MSEC;
    assert(0U == g7_controller_remote_blockers());

    ready_world();
    g_obstacle_fault = true;
    expect_blocked(CTRL_BLOCK_CONTROLLER);
    ready_world();
    g_pwm = 1U;
    expect_blocked(CTRL_BLOCK_CONTROLLER);
    ready_world();
    g_lock_fail = true;
    expect_blocked(CTRL_BLOCK_CONTROLLER);

    ready_world();
    g_scanner_ready = 0U;
    expect_blocked(CTRL_BLOCK_SCANNER);
    puts("PASS FR13/FR16 START blockers: MQTT, hold, pending, IMU (cal, "
         "error, invalid, >200 ms), IR invalid/stale, controller, scanner "
         "each set alone; 0 when ready");
}

/** @brief Test: a clock failure blocks START and several faults add up
 *  (FR16); IR invalid and IR stale are never reported together. */
static void test_blocker_combinations (void)
{
    ready_world();
    g_clock_error = E_TMOUT;
    expect_blocked((uint8_t)(CTRL_BLOCK_IMU | CTRL_BLOCK_CONTROLLER));

    ready_world();
    g_mqtt = 0U;
    g_reported_state = ROBOT_STOPPED;
    g_start_pending = 1U;
    g_calibrated = false;
    g_ir_valid = 0U;
    g_ir_time = g_now - APP_IR_MAX_AGE_MSEC - 1U;
    g_obstacle_fault = true;
    g_scanner_ready = 0U;
    assert((uint8_t)(CTRL_BLOCK_MQTT_OFFLINE | CTRL_BLOCK_NOT_HELD |
                     CTRL_BLOCK_CMD_PENDING | CTRL_BLOCK_IMU |
                     CTRL_BLOCK_IR_INVALID | CTRL_BLOCK_CONTROLLER |
                     CTRL_BLOCK_SCANNER) == g7_controller_remote_blockers());
    g_start_pending = 0U;
    puts("PASS FR16 blockers combine: clock error -> IMU|CONTROLLER, all "
         "faults at once -> 0xDF, IR invalid hides IR stale");
}

/** @brief Run one controller period from state at time now. */
static robot_state_t run_cycle (robot_state_t state)
{
    controller_history_t history = {0};

    return controller_cycle(state, &history);
}

/** @brief Start line following with fresh inputs and clear old faults. */
static robot_state_t start_following (void)
{
    nav_config_t const config = {
        150U, 250U, 2500U, 1800U, 120U, 96U, 1500U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
    };
    robot_state_t state;

    ready_world();
    g_control.fault = 0U;
    nav_init(&g_navigation, &config);
    nav_start(&g_navigation, g_now);
    state = run_cycle(ROBOT_LINE_FOLLOWING);
    assert(ROBOT_LINE_FOLLOWING == state);
    assert(0U != g_pwm);
    return state;
}

/** @brief Assert the published EVENT is SAFE_STOP with this fault source
 *  and that the sequence moved on from seq_before. */
static void expect_fault (uint8_t source, uint16_t seq_before)
{
    uint8_t kind = 0U;
    uint8_t value = 0U;
    uint16_t seq = 0U;

    g7_controller_navigation_report(&kind, &value, &seq);
    assert(NAV_EVENT_SAFE_STOP == kind);
    assert(source == value);
    assert(seq_before != seq);
    assert(0U == g_pwm);
}

/** @brief Read the published navigation event sequence. */
static uint16_t report_seq (void)
{
    uint8_t kind = 0U;
    uint8_t value = 0U;
    uint16_t seq = 0U;

    g7_controller_navigation_report(&kind, &value, &seq);
    return seq;
}

/** @brief Test: IR-side safety stops publish their fault source (FR16):
 *  stale 2, invalid 2, read error 3; 200 ms old IR still drives. */
static void test_ir_fault_sources (void)
{
    robot_state_t state = start_following();
    uint16_t seq = report_seq();

    g_now += CT_PERIOD_MSEC;
    g_terrain_time = g_now;
    g_ir_time = g_now - APP_IR_MAX_AGE_MSEC;
    state = run_cycle(state);
    assert(ROBOT_LINE_FOLLOWING == state);
    assert(0U != g_pwm);
    assert(seq == report_seq());
    g_now += 1U;
    g_terrain_time = g_now;
    state = run_cycle(state);
    assert(ROBOT_STOPPED == state);
    expect_fault(CTRL_FAULT_IR_STALE, seq);

    state = start_following();
    seq = report_seq();
    g_ir_valid = 0U;
    assert(ROBOT_STOPPED == run_cycle(state));
    expect_fault(CTRL_FAULT_IR_STALE, seq);

    state = start_following();
    seq = report_seq();
    g_ir_error = E_TMOUT;
    assert(ROBOT_ERROR == run_cycle(state));
    expect_fault(CTRL_FAULT_IR_READ, seq);
    puts("PASS FR16 IR safety stops: 201 ms stale/invalid -> STOPPED "
         "source 2, read error -> ERROR source 3, 200 ms keeps driving");
}

/** @brief Test: clock and obstacle faults publish their sources (FR16). */
static void test_clock_obstacle_sources (void)
{
    controller_history_t history = {0};
    robot_state_t state = start_following();
    uint16_t seq = report_seq();
    uint32_t const aborts = g_obstacle_aborts;

    g_obstacle_fault = true;
    assert(ROBOT_ERROR == run_cycle(state));
    expect_fault(CTRL_FAULT_OBSTACLE, seq);
    assert(aborts + 1U == g_obstacle_aborts);

    /* handle_following() itself reports a clock failure as source 4. */
    (void)start_following();
    g_clock_error = E_TMOUT;
    g_control.fault = 0U;
    assert(ROBOT_ERROR == handle_following(&history));
    assert(CTRL_FAULT_CLOCK == g_control.fault);
    assert(NAV_EVENT_SAFE_STOP == g_control.event);
    assert(CTRL_FAULT_CLOCK == g_control.value);

    /* In a full period the IMU check sees the clock first: the robot
     * still stops, reported as IMU lost (source 10). */
    state = start_following();
    seq = report_seq();
    g_clock_error = E_TMOUT;
    assert(ROBOT_STOPPED == run_cycle(state));
    expect_fault(CTRL_FAULT_IMU_LOST, seq);
    puts("PASS FR16 obstacle fault -> ERROR source 5 and abort; clock "
         "error -> source 4 in following, IMU lost (10) in a full period");
}

/** @brief Test: IMU and downhill-governor stops publish their sources and
 *  the IMU may be silent for exactly 1000 ms but not longer (FR16). */
static void test_imu_fault_sources (void)
{
    robot_state_t state = start_following();
    uint32_t const last_good = g_now;
    uint16_t seq = report_seq();

    g_terrain_valid = 0U;
    g_now = last_good + CTRL_IMU_LOST_MSEC;
    g_ir_time = g_now;
    state = run_cycle(state);
    assert(ROBOT_LINE_FOLLOWING == state);
    assert(seq == report_seq());
    g_now += 1U;
    g_ir_time = g_now;
    state = run_cycle(state);
    assert(ROBOT_STOPPED == state);
    expect_fault(CTRL_FAULT_IMU_LOST, seq);

    state = start_following();
    seq = report_seq();
    g_governor = 1U;
    assert(ROBOT_STOPPED == run_cycle(state));
    expect_fault(CTRL_FAULT_IMU_STALE, seq);

    state = start_following();
    seq = report_seq();
    g_governor = CTRL_DOWNHILL_OVERSPEED;
    assert(ROBOT_STOPPED == run_cycle(state));
    expect_fault(CTRL_FAULT_OVERSPEED, seq);
    puts("PASS FR16 IMU silent 1000 ms drives, 1001 ms -> source 10; "
         "governor stale IMU -> 11, overspeed -> 9");
}

/** @brief Test: a fault source is reported once, not every period, and is
 *  reported again after an explicit STOP clears it (FR15/FR16). */
static void test_fault_reported_once (void)
{
    robot_state_t state = start_following();
    uint16_t seq = report_seq();
    uint16_t first;

    g_obstacle_fault = true;
    state = run_cycle(state);
    expect_fault(CTRL_FAULT_OBSTACLE, seq);
    first = report_seq();
    g_reported_state = ROBOT_LINE_FOLLOWING;
    assert(ROBOT_ERROR == run_cycle(ROBOT_LINE_FOLLOWING));
    assert(first == report_seq());
    g_obstacle_fault = false;
    assert(B1_OK == g7_controller_remote_command(B1_CMD_STOP, NULL));
    state = run_cycle(ROBOT_STOPPED);
    assert(ROBOT_REMOTE_HOLD == state);
    assert(0U == g_control.fault);
    state = start_following();
    g_obstacle_fault = true;
    (void)run_cycle(state);
    expect_fault(CTRL_FAULT_OBSTACLE, first);
    g_obstacle_fault = false;
    puts("PASS FR15/FR16 same fault reported once per stop, again after "
         "STOP clears it");
}

/** @brief Test: mission state codes not covered elsewhere (FR15). */
static void test_mission_codes (void)
{
    ready_world();
    g_reported_state = ROBOT_ERROR;
    assert(CTRL_MISSION_FAULT == g7_controller_mission_state());
    g_reported_state = ROBOT_STOPPED;
    assert(CTRL_MISSION_STOPPED == g7_controller_mission_state());
    g_reported_state = ROBOT_AT_JUNCTION;
    assert(CTRL_MISSION_STOPPED == g7_controller_mission_state());
    g_reported_state = ROBOT_IDLE;
    assert(CTRL_MISSION_SELF_CHECK == g7_controller_mission_state());
    g_lock_fail = true;
    g_reported_state = ROBOT_LINE_FOLLOWING;
    assert(CTRL_MISSION_STOPPED == g7_controller_mission_state());
    assert(0U == g7_controller_remote_hold_applied());
    g_lock_fail = false;
    g_reported_state = ROBOT_REMOTE_HOLD;
    puts("PASS FR15 mission codes: ERROR 7, STOPPED/junction 6, IDLE 1, "
         "busy mutex reports STOPPED and not held");
}

/** @brief Test: barcode relearn is refused unless held and resettable,
 *  and unsupported commands are refused (FR13). */
static void test_command_guards (void)
{
    uint32_t const relearns = g_relearns;

    ready_world();
    g_reported_state = ROBOT_LINE_FOLLOWING;
    assert(B1_NOT_READY ==
           g7_controller_remote_command(B1_CMD_RECAL_BARCODE, NULL));
    ready_world();
    g_pwm = 1U;
    assert(B1_NOT_READY ==
           g7_controller_remote_command(B1_CMD_RECAL_BARCODE, NULL));
    ready_world();
    g_scanner_can_rearm = 0U;
    assert(B1_NOT_READY ==
           g7_controller_remote_command(B1_CMD_RECAL_BARCODE, NULL));
    ready_world();
    g_start_pending = 1U;
    assert(B1_NOT_READY ==
           g7_controller_remote_command(B1_CMD_RECAL_BARCODE, NULL));
    assert(relearns == g_relearns);
    ready_world();
    assert(B1_NOT_READY ==
           g7_controller_remote_command(B1_CMD_RECAL_IMU, NULL));
    assert(B1_NOT_READY ==
           g7_controller_remote_command(B1_CMD_SNAPSHOT, NULL));
    assert(B1_NOT_READY ==
           g7_controller_remote_command(B1_CMD_MAPPING, NULL));
    assert((0U == g_stop_pending) && (0U == g_start_pending));
    puts("PASS FR13 relearn refused while moving/pending/not resettable; "
         "IMU recal, snapshot, mapping refused by the controller");
}

/** @brief Test: IDLE only starts following on fresh, valid IR with a
 *  calibrated IMU; faults latch ERROR; ERROR and an unknown state keep the
 *  motors off (FR16). */
static void test_idle_and_error (void)
{
    robot_state_t const unknown = (robot_state_t)99;
    uint32_t aborts;

    ready_world();
    g_calibrated = false;
    assert(ROBOT_IDLE == run_cycle(ROBOT_IDLE));
    ready_world();
    g_ir_valid = 0U;
    assert(ROBOT_IDLE == run_cycle(ROBOT_IDLE));
    ready_world();
    g_ir_time = g_now - APP_IR_MAX_AGE_MSEC - 1U;
    assert(ROBOT_IDLE == run_cycle(ROBOT_IDLE));
    assert(0U == g_pwm);
    ready_world();
    g_ir_error = E_TMOUT;
    assert(ROBOT_ERROR == run_cycle(ROBOT_IDLE));
    ready_world();
    g_clock_error = E_TMOUT;
    assert(ROBOT_ERROR == run_cycle(ROBOT_IDLE));
    ready_world();
    aborts = g_obstacle_aborts;
    g_obstacle_fault = true;
    assert(ROBOT_ERROR == run_cycle(ROBOT_IDLE));
    assert((aborts + 1U) == g_obstacle_aborts);
    ready_world();
    assert(ROBOT_LINE_FOLLOWING == run_cycle(ROBOT_IDLE));

    /* ERROR stays latched with good inputs and a live link. */
    ready_world();
    g_pwm = CT_RUN_PWM;
    for (uint32_t i = 0U; i < 5U; i++)
    {
        g_now += CT_PERIOD_MSEC;
        g_ir_time = g_now;
        g_terrain_time = g_now;
        assert(ROBOT_ERROR == run_cycle(ROBOT_ERROR));
        assert(0U == g_pwm);
    }
    assert(CTRL_MISSION_FAULT == g7_controller_mission_state());
    ready_world();
    g_pwm = CT_RUN_PWM;
    assert(ROBOT_ERROR == run_cycle(unknown));
    assert(0U == g_pwm);

    /* Obstacle sensor fault during a bypass: ERROR, bypass aborted. */
    ready_world();
    g_control.imu_seen = 1U;
    aborts = g_obstacle_aborts;
    g_obstacle_fault = true;
    g_pwm = CT_RUN_PWM;
    assert(ROBOT_ERROR == run_cycle(ROBOT_OBSTACLE));
    assert((aborts + 1U) == g_obstacle_aborts);
    assert(0U == g_pwm);
    ready_world();
    g_latch_error = E_TMOUT;
    g_control.imu_seen = 1U;
    assert(ROBOT_STOPPED == run_cycle(ROBOT_OBSTACLE));
    (void)start_following();
    g_latch_error = E_TMOUT;
    assert(ROBOT_ERROR == run_cycle(ROBOT_LINE_FOLLOWING));
    assert(0U == g_pwm);
    puts("PASS FR16 IDLE waits for calibrated IMU and fresh valid IR, "
         "IR/clock/obstacle faults -> ERROR; ERROR and unknown states keep "
         "motors off; bypass faults stop");
}

/** @brief Test: the junction hold ignores readings while coasting, waits
 *  for both-black (picked up) then a steady line before following, and a
 *  sensor error is ERROR (FR16 lift/placement safety). */
static void test_junction_wait (void)
{
    controller_history_t history = {0};
    robot_state_t state = ROBOT_AT_JUNCTION;

    ready_world();
    g_control.imu_seen = 1U;
    for (uint32_t i = 0U; i < JUNCTION_HOLD_CYCLES; i++)
    {
        state = controller_cycle(state, &history);
        assert(ROBOT_AT_JUNCTION == state);
    }
    /* A steady line before both-black was seen does not resume. */
    for (uint32_t i = 0U; i < (2U * JUNCTION_SETTLE_CYCLES); i++)
    {
        g_now += CT_PERIOD_MSEC;
        g_ir_time = g_now;
        g_terrain_time = g_now;
        state = controller_cycle(state, &history);
        assert(ROBOT_AT_JUNCTION == state);
    }
    g_junction_black = true;
    state = controller_cycle(state, &history);
    assert(ROBOT_AT_JUNCTION == state);
    g_junction_black = false;
    /* Stale IR restarts the settle count. */
    g_ir_time = g_now - APP_IR_MAX_AGE_MSEC - 1U;
    state = controller_cycle(state, &history);
    assert((ROBOT_AT_JUNCTION == state) && (0U == history.settle_cycles));
    for (uint32_t i = 1U; i < JUNCTION_SETTLE_CYCLES; i++)
    {
        g_now += CT_PERIOD_MSEC;
        g_ir_time = g_now;
        g_terrain_time = g_now;
        state = controller_cycle(state, &history);
        assert(ROBOT_AT_JUNCTION == state);
    }
    g_now += CT_PERIOD_MSEC;
    g_ir_time = g_now;
    g_terrain_time = g_now;
    state = controller_cycle(state, &history);
    assert(ROBOT_LINE_FOLLOWING == state);
    history.junction_cycles = 0U;
    g_latch_error = E_TMOUT;
    assert(ROBOT_ERROR == controller_cycle(ROBOT_AT_JUNCTION, &history));
    g_latch_error = E_OK;
    puts("PASS FR16 junction hold: 5 coast periods ignored, resumes only "
         "after both-black then 3 fresh line periods; latch error -> ERROR");
}

/** @brief Run every test case. */
int main (void)
{
    test_uninitialised_fails_closed();
    assert(E_OK == g7_controller_remote_init());
    test_each_blocker_bit();
    test_blocker_combinations();
    test_ir_fault_sources();
    test_clock_obstacle_sources();
    test_imu_fault_sources();
    test_fault_reported_once();
    test_mission_codes();
    test_command_guards();
    test_idle_and_error();
    test_junction_wait();
    puts("PASS controller readiness and fault-source checks");
    return 0;
}

/*** end of file ***/

