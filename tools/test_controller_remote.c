/* Host test for the controller's Cortex-M0+-safe remote handoff. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define B1_ROBOT_APP 1
#define B1_LIVE_COMMANDS 1
#define APP_TEST 0
#define _PICO_RP2040_ 1
#include "app_program/app/task_controller.c"

static bool g_lock_held = false;
static bool g_force_busy = false;
static uint8_t g_connected = 1U;
static uint8_t g_barcode_can_rearm = 1U;
uint8_t bc_wave_can_rearm(void) {return g_barcode_can_rearm;}
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
ER ir_shared_get_latest(ir_shared_snapshot_t * out)
{
    assert(!g_lock_held);
    out->b_valid = true;
    out->timestamp_msec = 99U;
    return E_OK;
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
    nav_init(&g_navigation, &(nav_config_t){150U,250U,2500U,1800U,120U,400U,180U});
    g_navigation.state = NAV_FAULT;
    remote_publish_state(ROBOT_NAV_SAFETY_STOP);
    assert(g7_controller_remote_ready() == 0U);
    assert(g7_controller_mission_state() == 5U);
    assert(g7_controller_remote_command(B1_CMD_START,NULL) == B1_NOT_READY);
    assert(g7_controller_remote_command(B1_CMD_STOP,NULL) == B1_OK);
    remote_take_requests(&stop,&start,ROBOT_NAV_SAFETY_STOP);
    assert(stop == 1U && start == 0U);
    assert(remote_reset_after_stop(ROBOT_NAV_SAFETY_STOP) == ROBOT_REMOTE_HOLD);
    assert(g_navigation.state == NAV_WAIT && g_navigation.pending == 0U);
    remote_publish_state(ROBOT_REMOTE_HOLD);
    assert(g7_controller_remote_ready() == 1U);
    /* A genuine controller error must never be cleared over MQTT. */
    assert(remote_reset_after_stop(ROBOT_ERROR) == ROBOT_ERROR);
    assert(g7_controller_remote_ready() == 1U); /* Ready only because published HOLD. */
    remote_publish_state(ROBOT_ERROR);
    assert(g7_controller_remote_ready() == 0U);
    assert(g7_controller_remote_command(B1_CMD_START,NULL) == B1_NOT_READY);
    assert(g_unlock_count > 0U);
    puts("Controller handoff: STOP priority, no missed STOP, "
         "START gating and mutex-busy fail-safe PASS");
    return 0;
}
