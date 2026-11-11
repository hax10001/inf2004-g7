/* Host test for the controller's Cortex-M0+-safe remote handoff. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define B1_ROBOT_APP 1
#define B1_LIVE_COMMANDS 1
#define APP_TEST 0
#define APP_COLLISION_ENABLED 1 /* Exercise optional impact-stop behavior. */
#define _PICO_RP2040_ 1
#include "app_program/app/task_controller.c"

static bool g_lock_held = false;
static bool g_force_busy = false;
static uint8_t g_connected = 1U;
static uint8_t g_barcode_can_rearm = 1U;
static bool g_obstacle_fault = false;
static unsigned int g_obstacle_aborts = 0U;
static bool g_obstacle_blocked = false;
static uint32_t g_ir_timestamp = 99U;
static unsigned int g_decoded_clears = 0U;
static unsigned int g_bypass_steps = 0U;
static line_follow_action_t g_last_action = LINE_FORWARD;
static float g_left_rpm, g_right_rpm;
static obstacle_drive_t g_bypass_drive;
bool obstacle_has_fault(void) {return g_obstacle_fault;}
bool obstacle_is_blocked(void) {return g_obstacle_blocked;}
void obstacle_abort(void) {++g_obstacle_aborts;}
obstacle_drive_t obstacle_bypass_step(uint32_t l,uint32_t r,uint32_t ms)
{
    (void)l;(void)r;(void)ms;
    ++g_bypass_steps;
    return g_bypass_drive;
}
void motion_apply(line_follow_action_t action) {g_last_action=action;}
void motion_brake(void) {g_last_action=LINE_UNCERTAIN;}
void motion_stop_now(void) {g_last_action=LINE_UNCERTAIN;}
void motion_drive(float l,float r) {g_left_rpm=l;g_right_rpm=r;}
void motion_apply_maneuver(uint8_t action) {(void)action;}
void motion_downhill_sample(float p,float r,bool f,bool m)
{(void)p;(void)r;(void)f;(void)m;}
bool motion_downhill_faulted(void) {return false;}
void motion_downhill_reset(void) {}
void g7_nav_clear_decoded(void) {++g_decoded_clears;}
int g7_nav_take_decoded(uint8_t *l,uint8_t *a,uint16_t *s)
{(void)l;(void)a;(void)s;return 0;}
void encoder_read(encoder_wheel_t wheel,encoder_sample_t *out)
{(void)wheel;*out=(encoder_sample_t){0};}
uint8_t bc_wave_navigation_guard_active(void) {return 0U;}
void bc_wave_set_suppressed(uint8_t b) {(void)b;}
static int g_relearn_calls = 0;
void bc_wave_request_relearn(void) {++g_relearn_calls;}
uint8_t bc_wave_decode_pending(void) {return 0U;}
ER ir_shared_take_sides(bool *l,bool *r) {*l=false;*r=false;return E_OK;}
ER ir_shared_take_junction(bool *both) {*both=false;return E_OK;}
ER ir_shared_take_both_gap(bool *gap) {*gap=false;return E_OK;}
ER ir_shared_take_diag(ir_shared_diag_t *d) {*d=(ir_shared_diag_t){0};return E_OK;}
ER tk_dly_tsk(UINT ms) {(void)ms;return E_OK;}
INT tm_printf(UB const *format,...) {(void)format;return 0;}
uint8_t bc_wave_can_rearm(void) {return g_barcode_can_rearm;}
uint8_t bc_wave_is_ready(void) {return g_barcode_can_rearm;}
static unsigned int g_unlock_count = 0U;

ID tk_cre_mtx(const T_CMTX *config)
{
    assert(config != NULL);
    assert(config->mtxatr == TA_INHERIT);
    return 42;
}

ER tk_loc_mtx(ID id, TMO timeout)
{
    assert(id == 42);
    assert(timeout == TMO_POL || timeout == TMO_FEVR);
    if (g_force_busy && timeout == TMO_POL) return E_TMOUT;
    assert(!g_lock_held);
    g_lock_held = true;
    return E_OK;
}

ER tk_unl_mtx(ID id)
{
    assert(id == 42);
    assert(g_lock_held);
    g_lock_held = false;
    g_unlock_count++;
    return E_OK;
}

uint8_t b1_mqtt_connected(void) {return g_connected;}
bool terrain_is_calibrated(void) {return true;}
static uint8_t g_collision_latched = 0U;
static bool g_collision_enabled;
ER terrain_set_collision_enabled(bool enabled)
{g_collision_enabled=enabled;return E_OK;}
static unsigned int g_collision_acks = 0U;
ER terrain_acknowledge_collision(void)
{
    g_collision_latched = 0U;
    ++g_collision_acks;
    return E_OK;
}
ER terrain_ack_collision(uint32_t seq)
{(void)seq;return terrain_acknowledge_collision();}
ER terrain_get_snapshot(terrain_snapshot *s)
{
    *s=(terrain_snapshot){.b_valid=1U,.timestamp_msec=99U,
                         .b_collision=g_collision_latched};
    return E_OK;
}
void motor_last_command(uint16_t *l,uint16_t *r) {*l=0U;*r=0U;}
void motion_reset_run(void) {motion_downhill_reset();}
void motion_observe_terrain(terrain_snapshot const *s,uint32_t now)
{(void)s;(void)now;}
uint8_t motion_downhill_fault(void)
{return (uint8_t)motion_downhill_faulted();}
uint8_t bc_wave_capture_active(void) {return 0U;}
int g7_nav_begin_run(void) {return 1;}
int g7_nav_take_scan(uint8_t *l,uint8_t *a,uint16_t *s,
                     uint32_t *ms,uint32_t *done,uint8_t *pending)
{*ms=0U;*done=100U;*pending=0U;return g7_nav_take_decoded(l,a,s);}
ER ir_shared_get_latest(ir_shared_snapshot_t * out)
{
    assert(!g_lock_held);
    out->b_valid = true;
    out->timestamp_msec = g_ir_timestamp;
    return E_OK;
}

static void obstacle_handoff(void)
{
    controller_history_t history={0};
    nav_init(&g_navigation,
        &(nav_config_t){150U,250U,2500U,1800U,120U,96U,1500U,0U,0U,0U,0U,0U,0U});
    nav_start(&g_navigation,1U);
    assert(nav_barcode(&g_navigation,0U,NAV_LEFT,1U,1U,0U)==1);
    g_obstacle_blocked=true;
    assert(handle_following(&history)==ROBOT_OBSTACLE);
    assert(g_last_action==LINE_UNCERTAIN);
    assert(g_navigation.pending==0U && g_navigation.state==NAV_WAIT);
    assert(g_decoded_clears>0U);

    g_bypass_drive=(obstacle_drive_t){0};
    g_bypass_drive.right_rpm=25;
    assert(handle_obstacle(&history)==ROBOT_OBSTACLE);
    assert(g_left_rpm==0.0f && g_right_rpm==25.0f);
    g_bypass_drive.b_stop=true;
    assert(handle_obstacle(&history)==ROBOT_OBSTACLE);
    assert(g_last_action==LINE_UNCERTAIN);

    g_bypass_drive=(obstacle_drive_t){0,0,false,true,false,false};
    assert(handle_obstacle(&history)==ROBOT_OBSTACLE);
    assert(g_last_action==LINE_FORWARD);
    g_bypass_drive=(obstacle_drive_t){0,0,false,false,true,false};
    assert(handle_obstacle(&history)==ROBOT_LINE_FOLLOWING);
    assert(g_navigation.state==NAV_FOLLOW && !g_navigation.pending);
    assert(g_last_action==LINE_UNCERTAIN);

    g_bypass_drive=(obstacle_drive_t){0,0,false,false,false,true};
    assert(handle_obstacle(&history)==ROBOT_ERROR);
    assert(g_last_action==LINE_UNCERTAIN);
    unsigned int steps=g_bypass_steps;
    g_ir_timestamp=UINT32_MAX-500U;
    assert(handle_obstacle(&history)==ROBOT_STOPPED);
    assert(g_bypass_steps==steps && g_last_action==LINE_UNCERTAIN);
    g_ir_timestamp=99U;
    g_obstacle_fault=true;
    assert(handle_following(&history)==ROBOT_ERROR);
    assert(g_last_action==LINE_UNCERTAIN);
    g_obstacle_fault=false;
    g_obstacle_blocked=false;
    puts("Obstacle handoff: preemption, RPM, stop, reacquisition, "
         "fault and stale IR PASS");
}
ER tk_get_otm(SYSTIM *out)
{
    out->hi = 0U;
    out->lo = 100U;
    return E_OK;
}

int main(void)
{
    uint8_t stop=0U, start=0U;
    assert(g7_controller_remote_init() == E_OK);
    assert(g7_controller_remote_ready() == 1U);
    /* A previously latched collision must never kick a remote-held or
     * stopped robot back into STOPPED before a later explicit START. */
    terrain_snapshot collision = {0};
    collision.b_collision = 1U;
    assert(!collision_requires_stop(ROBOT_REMOTE_HOLD,E_OK,&collision));
    assert(!collision_requires_stop(ROBOT_IDLE,E_OK,&collision));
    assert(!collision_requires_stop(ROBOT_STOPPED,E_OK,&collision));
    assert(!collision_requires_stop(ROBOT_NAV_SAFETY_STOP,E_OK,&collision));
    assert(collision_requires_stop(ROBOT_LINE_FOLLOWING,E_OK,&collision));
    assert(collision_requires_stop(ROBOT_OBSTACLE,E_OK,&collision));
    assert(!collision_requires_stop(ROBOT_LINE_FOLLOWING,E_OBJ,&collision));
    /* Barcode recalibration is forwarded to the barcode task only. */
    assert(g7_controller_remote_command(B1_CMD_RECAL_BARCODE,NULL) == B1_OK);
    assert(g_relearn_calls == 1);
    remote_take_requests(&stop,&start,ROBOT_REMOTE_HOLD);
    assert(stop == 0U && start == 0U); /* no motion request queued */
    g_obstacle_fault = true;
    assert(g7_controller_remote_command(B1_CMD_START,NULL) == B1_NOT_READY);
    assert(g7_controller_remote_blockers() & 64U);
    g_obstacle_fault = false;
    g_barcode_can_rearm = 0U;
    assert(g7_controller_remote_ready() == 0U);
    assert(g7_controller_remote_blockers() & 128U);
    g_barcode_can_rearm = 1U;
    assert(g7_controller_remote_command(B1_CMD_START,NULL) == B1_OK);
    assert(g7_controller_remote_command(B1_CMD_STOP,NULL) == B1_OK);
    remote_take_requests(&stop,&start,ROBOT_REMOTE_HOLD);
    assert(stop == 1U && start == 0U); /* STOP supersedes START. */
    assert(g7_controller_remote_command(B1_CMD_START,NULL) == B1_OK);
    remote_take_requests(&stop,&start,ROBOT_REMOTE_HOLD);
    assert(stop == 0U && start == 1U); /* Explicit later START. */
    assert(g7_controller_remote_command(B1_CMD_STOP,NULL) == B1_OK);
    g_force_busy = true;
    remote_take_requests(&stop,&start,ROBOT_LINE_FOLLOWING);
    assert(stop == 1U && start == 0U); /* Fail-safe on busy mutex. */
    g_force_busy = false;
    remote_take_requests(&stop,&start,ROBOT_REMOTE_HOLD);
    assert(stop == 1U && start == 0U); /* Pending STOP preserved. */
    g_connected = 0U;
    assert(g7_controller_remote_ready() == 0U);
    assert(g7_controller_remote_command(B1_CMD_START,NULL) == B1_NOT_READY);
    g_connected = 1U;
    remote_publish_state(ROBOT_LINE_FOLLOWING);
    assert(g7_controller_remote_ready() == 0U);
    assert(g7_controller_mission_state() == 2U);
    /* A confirmed navigation failure is a safe, recoverable STOP state.
     * No START can pass until operator STOP clears the navigation fault. */
    nav_init(&g_navigation, &(nav_config_t){150U,250U,2500U,1800U,120U,96U,1500U,0U,0U,0U,0U,0U,0U});
    g_navigation.state = NAV_FAULT;
    remote_publish_state(ROBOT_NAV_SAFETY_STOP);
    assert(g7_controller_remote_ready() == 0U);
    assert(g7_controller_mission_state() == 5U);
    assert(g7_controller_remote_command(B1_CMD_START,NULL) == B1_NOT_READY);
    assert(g7_controller_remote_command(B1_CMD_STOP,NULL) == B1_OK);
    remote_take_requests(&stop,&start,ROBOT_NAV_SAFETY_STOP);
    assert(stop == 1U && start == 0U);
    g_collision_latched = 1U;
    assert(remote_reset_after_stop(ROBOT_NAV_SAFETY_STOP) == ROBOT_REMOTE_HOLD);
    assert(g_collision_latched == 0U && g_collision_acks > 0U);
    assert(g_navigation.state == NAV_WAIT && g_navigation.pending == 0U);
    assert(g_obstacle_aborts != 0U);
    remote_publish_state(ROBOT_OBSTACLE);
    assert(g7_controller_mission_state() == 3U);
    assert(g7_controller_remote_command(B1_CMD_STOP,NULL) == B1_OK);
    remote_take_requests(&stop,&start,ROBOT_OBSTACLE);
    assert(stop == 1U && start == 0U);
    assert(remote_reset_after_stop(ROBOT_OBSTACLE) == ROBOT_REMOTE_HOLD);
    remote_publish_state(ROBOT_REMOTE_HOLD);
    assert(g7_controller_remote_ready() == 1U);
    /* Regression for the real-world STOP -> REMOTE HOLD -> START sequence
     * after an impact. STOP ACK alone does not imply physical completion. */
    g_collision_latched = 1U;
    remote_publish_state(ROBOT_STOPPED);
    assert(g7_controller_remote_command(B1_CMD_START,NULL) == B1_NOT_READY);
    assert(g7_controller_remote_command(B1_CMD_STOP,NULL) == B1_OK);
    remote_take_requests(&stop,&start,ROBOT_STOPPED);
    assert(stop == 1U && start == 0U);
    robot_state_t recovered = remote_reset_after_stop(ROBOT_STOPPED);
    assert(recovered == ROBOT_REMOTE_HOLD);
    remote_publish_state(recovered);
    assert(g_collision_latched == 0U);
    assert(g7_controller_remote_hold_applied() == 1U);
    assert(g7_controller_remote_ready() == 1U);
    assert(g7_controller_remote_command(B1_CMD_START,NULL) == B1_OK);
    remote_take_requests(&stop,&start,ROBOT_REMOTE_HOLD);
    assert(stop == 0U && start == 1U);
    /* A genuine controller error must never be cleared over MQTT. */
    assert(remote_reset_after_stop(ROBOT_ERROR) == ROBOT_ERROR);
    assert(g7_controller_remote_ready() == 1U); /* Ready only because published HOLD. */
    remote_publish_state(ROBOT_ERROR);
    assert(g7_controller_remote_ready() == 0U);
    assert(g7_controller_remote_command(B1_CMD_START,NULL) == B1_NOT_READY);
    assert(g_unlock_count > 0U);
    obstacle_handoff();
    puts("Controller handoff: STOP priority, no missed STOP, "
         "START gating and mutex-busy fail-safe PASS");
    return 0;
}
