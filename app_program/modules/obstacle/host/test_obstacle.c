/* Host integration test: real monitor, planner, sequencer and controller. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "app/task_controller.h"
#include "app/tasks.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/line_barcode/line_barcode.h"
#include "modules/motion/motion.h"
#include "modules/obstacle/obstacle.h"
#include "modules/obstacle/obs_config.h"
#include "modules/obstacle/task_obstacle.h"
#include "modules/obstacle/servo.h"
#include "modules/obstacle/ultrasonic.h"

static jmp_buf escape;
static const char *scenario;
static unsigned now_ms = 1000, cycles, pings, angles, drives, following;
static UINT flags;
static int sampler, avoidance;
static int is(const char *name) { return strcmp(scenario, name) == 0; }
INT tm_printf(const UB *format, ...) { (void)format; return 0; }
ER ir_shared_lock(TMO timeout) { (void)timeout; return E_OK; }
ER ir_shared_unlock(void) { return E_OK; }
ER ir_shared_get_latest(ir_shared_snapshot_t *out)
{
    *out = (ir_shared_snapshot_t){0};
    out->b_valid = true; out->timestamp_msec = now_ms;
    return E_OK;
}
ER ir_shared_take_junction(bool *seen) { *seen = false; return E_OK; }
ER ir_shared_take_sides(bool *left, bool *right)
{ *left = false; *right = false; return E_OK; }
ER line_barcode_set_avoidance(bool active) { avoidance = active; return E_OK; }
bool terrain_is_calibrated(void) { return true; }
ID tk_cre_flg(const T_CFLG *p) { flags = p->iflgptn; return 1; }
ER tk_ref_flg(ID id, T_RFLG *out) { (void)id; out->flgptn = flags; return E_OK; }
ER tk_set_flg(ID id, UINT bits) { (void)id; flags |= bits; return E_OK; }
ER tk_clr_flg(ID id, UINT keep) { (void)id; flags &= keep; return E_OK; }
ER tk_get_otm(SYSTIM *out) { out->hi = 0; out->lo = now_ms; return E_OK; }
ID tasks_create(tasks_entry_t entry, PRI priority)
{ (void)entry; (void)priority; return 1; }
ER tk_sta_tsk(ID id, INT code) { (void)id; (void)code; return E_OK; }
uint32_t encoder_now_usec(void) { return now_ms * 1000U; }
ER ultrasonic_init(void) { return E_OK; }
ER servo_init(void) { return is("init_failure") ? E_OBJ : E_OK; }
uint32_t servo_set_angle(int32_t angle) { (void)angle; ++angles; return 40; }
ultrasonic_result_t ultrasonic_ping(int32_t *range)
{
    ++pings; *range = 385;
    if (is("bad")) { *range = 0; return ULTRASONIC_OUT_OF_RANGE; }
    if (is("no_echo")) { *range = 0; return ULTRASONIC_NO_ECHO; }
    if (is("zero")) { *range = 0; }
    if (is("near20")) { *range = 200; }
    if (is("near25")) { *range = 250; }
    if (is("beyond")) { *range = 251; }
    if (is("spike")) { *range = pings == 1 ? 50 : 385; }
    return ULTRASONIC_OK;
}
int32_t ultrasonic_median_of_three(int32_t a, int32_t b, int32_t c)
{
    if (a > b) { int32_t t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return a > b ? a : b;
}
ER motion_get_status(motion_status_t *out)
{
    if (is("odometry_failure")) { return E_OBJ; }
    *out = (motion_status_t){0}; out->b_valid = true;
    out->timestamp_usec = now_ms * 1000U;
    return E_OK;
}
void motion_apply(line_follow_action_t action)
{ if (action != LINE_UNCERTAIN) { ++following; } }
void motion_drive(float left, float right)
{ assert(left > 0 || right > 0); ++drives; }
ER tk_dly_tsk(UINT delay)
{
    now_ms += delay;
    if (++cycles > 1000) { longjmp(escape, 1); }
    if (sampler)
    {
        if (pings >= 8 || (flags & OBS_EVT_BLOCKED)) { longjmp(escape, 1); }
    }
    else
    {
        if (flags & OBS_EVT_SCAN_REQ)
        {
            scan_plan_bypass_t plan;
            scan_plan_build(NULL, 0, &plan);
            if (is("best_left") || is("best_right"))
            {
                scan_plan_point_t points[OBS_SCAN_POINTS];
                for (int i = 0; i < OBS_SCAN_POINTS; ++i)
                {
                    int angle = -70 + i * 10;
                    int near = angle >= -10 && angle <= 10;
                    int better = is("best_right") ? angle < 0 : angle > 0;
                    scan_plan_point_set(&points[i], angle,
                                        near ? 225 : better ? 4000 : 1200);
                }
                scan_plan_build(points, OBS_SCAN_POINTS, &plan);
                assert(scan_plan_result(&plan) == SCAN_PLAN_BEST_EFFORT);
                assert(scan_plan_side_is_left(&plan) == is("best_left"));
            }
            assert(obs_state_plan_store(&plan));
            obs_state_flags_clear(OBS_EVT_SCAN_REQ);
            obs_state_flags_set(OBS_EVT_SCAN_DONE);
        }
        if (is("best_left") || is("best_right"))
        {
            obs_guard_snapshot_t guard;
            assert(obs_state_guard_load(&guard));
            if (guard.clock & 255)
            {
                for (unsigned view = 0; view < 3; ++view)
                { assert(obs_state_guard_sample(guard.clock & 255, view, 4000, 0)); }
            }
        }
        if (following || drives) { longjmp(escape, 1); }
    }
    return E_OK;
}

int main(int argc, char **argv)
{
    assert(argc == 2); scenario = argv[1];
    assert(obstacle_init() == E_OK);
    if (is("bad") || is("no_echo") || is("zero") || is("near20") ||
        is("near25") || is("beyond") || is("spike"))
    {
        sampler = 1;
        if (!setjmp(escape)) { task_obstacle_task(0, NULL); }
        assert(obstacle_is_blocked() == (is("near20") || is("near25")));
        assert(!obstacle_has_fault());
        assert(angles == 1); /* Only parks forward; never sweeps at startup. */
        if (is("near20") || is("near25")) { assert(pings >= 5); }
        else
        {
            sampler = 0;
            if (!setjmp(escape)) { task_controller_task(0, NULL); }
            assert(following > 0 && !avoidance);
        }
    }
    else
    {
        if (is("fault")) { obs_state_faults_update(OBS_FAULT_NO_ROUTE, 0); }
        if (is("no_route") || is("odometry_failure") ||
            is("best_left") || is("best_right"))
        { obs_state_flags_set(OBS_EVT_BLOCKED); }
        if (!setjmp(escape)) { task_controller_task(0, NULL); }
        if (is("best_left") || is("best_right")) { assert(drives > 0); }
        else
        {
            assert(following > 0 && !avoidance);
            assert(!obstacle_has_fault() && !(flags & OBS_EVT_BLOCKED));
            if (is("fault"))
            {
                assert(!obstacle_monitor_ready());
                now_ms += OBS_RETRY_DELAY_MSEC;
                assert(obstacle_monitor_ready());
            }
        }
    }
    printf("PASS %s (pings=%u line_commands=%u)\n", scenario, pings, following);
    return 0;
}
