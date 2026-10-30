/* Host-only integration tests. Real controller, scanner, packed state and
 * sequencer; deterministic sensor, encoder, motion and RTOS test doubles.
 */
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app/task_controller.h"
#include "app/app_config.h"
#include "app/tasks.h"
#include "modules/obstacle/obstacle.h"
#include "modules/obstacle/obs_config.h"
#include "modules/obstacle/obs_geom.h"
#include "modules/obstacle/test_obs.h"
#include "modules/obstacle/task_obstacle.h"
#include "modules/obstacle/servo.h"
#include "modules/obstacle/ultrasonic.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/motion/motion.h"

static jmp_buf escape;
static const char *scenario;
static UINT flags;
static uint32_t now_ms = 1000, visited;
static unsigned cycles, phase_cycles, scan_count, drive_count, normal_count;
static unsigned lock_depth, locks, unlocks, angle_count, ping_count;
static int last_phase = -1, angle, angles[64], sensor_mode;
static float left_rpm, right_rpm;
static motion_status_t motion = {.b_valid = true};
static int bad_lock, bad_flags, odometry_bad, stopped;
static int abort_injected;
static int hazard_injected;
static int rear_left, rear_right, return_started;
static unsigned moving_pings;
static int rear_reappeared, rear_resets;

static int is_case(const char *name) { return strcmp(scenario, name) == 0; }
INT tm_printf(const UB *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    int count = vsnprintf(buffer, sizeof(buffer), (const char *)format, args);
    va_end(args);
    assert(strstr(buffer, ": FAIL") == NULL);
    return count;
}
ID tk_cre_mtx(const T_CMTX *config) { (void)config; return 1; }
ER tk_loc_mtx(ID id, TMO timeout)
{
    (void)id; (void)timeout;
    if (bad_lock) { return E_OBJ; }
    assert(lock_depth++ == 0);
    ++locks;
    return E_OK;
}
ER tk_unl_mtx(ID id)
{
    (void)id;
    assert(lock_depth-- == 1);
    ++unlocks;
    return E_OK;
}
ID tk_cre_flg(const T_CFLG *config) { flags = config->iflgptn; return 2; }
ER tk_ref_flg(ID id, T_RFLG *info)
{
    (void)id;
    if (bad_flags) { return E_OBJ; }
    info->flgptn = flags;
    return E_OK;
}
ER tk_set_flg(ID id, UINT bits)
{
    (void)id;
    if (bits & OBS_EVT_SCAN_DONE) { assert(!(flags & OBS_EVT_SCAN_REQ)); }
    flags |= bits;
    return E_OK;
}
ER tk_clr_flg(ID id, UINT keep) { (void)id; flags &= keep; return E_OK; }
ER tk_get_otm(SYSTIM *time) { time->hi = 0; time->lo = now_ms; return E_OK; }
ID tasks_create(tasks_entry_t entry, PRI priority)
{ (void)entry; (void)priority; return 3; }
ER tk_sta_tsk(ID id, INT code) { (void)id; (void)code; return E_OK; }
bool terrain_is_calibrated(void) { return true; }
ER ultrasonic_init(void) { return E_OK; }
ER servo_init(void) { return E_OK; }
uint32_t servo_set_angle(int32_t value)
{
    angle = value;
    if (angle_count < 64) { angles[angle_count] = value; }
    angle_count++;
    return 40;
}
ultrasonic_result_t ultrasonic_ping(int32_t *range)
{
    ++ping_count;
    if (left_rpm > 0 || right_rpm > 0) { ++moving_pings; }
    *range = abs(angle) <= 10 ? 550 : 2000;
    if (is_case("guard")) { *range = 40; }
    if (is_case("guard_all")) { *range = 2000; }
    if (is_case("scan_clear")) { *range = 2000; }
    if (is_case("scan_bad") || is_case("monitor_bad") ||
        is_case("guard_bad") ||
        (is_case("monitor_recovered") && ping_count <= 5))
    { return ULTRASONIC_NO_ECHO; }
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
    if (odometry_bad) { return E_OBJ; }
    *out = motion;
    out->timestamp_usec = now_ms * 1000U;
    if (is_case("odom_stale")) { out->timestamp_usec -= 300000U; }
    return E_OK;
}
uint32_t encoder_now_usec(void) { return now_ms * 1000U; }
void motion_drive(float left, float right)
{
    assert(left >= 0 && right >= 0 && left <= 31 && right <= 31);
    assert(!stopped);
    ++drive_count;
    left_rpm = left; right_rpm = right;
}
void motion_apply(line_follow_action_t action)
{
    if (action == LINE_UNCERTAIN) { left_rpm = right_rpm = 0; }
    else
    {
        assert(!stopped);
        ++normal_count;
        left_rpm = right_rpm = 20;
    }
}

static void make_profile(scan_plan_point_t *points, int route)
{
    for (int i = 0; i < OBS_SCAN_POINTS; ++i)
    {
        int deg = -70 + i * 10;
        int range = abs(deg) <= 10 ? 550 : 2000;
        if (!route) { range = 2000; }
        if (route == 2 && deg == 50) { range = 260; }
        if (route == 3 && deg == -50) { range = 260; }
        if (route == 4) { range = 180; }
        scan_plan_point_set(&points[i], deg, range);
    }
}
static void supply_scan(void)
{
    scan_plan_point_t points[OBS_SCAN_POINTS];
    scan_plan_bypass_t plan = {0};
    int route = is_case("right") ? 2 : 3;
    if (is_case("no_route")) { route = 4; }
    if (is_case("clear")) { route = 0; }
    if (is_case("live_resume") && scan_count > 0) { route = 0; }
    make_profile(points, route);
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    assert(obs_state_plan_store(&plan));
    obs_state_flags_clear(OBS_EVT_SCAN_REQ);
    obs_state_flags_set(OBS_EVT_SCAN_DONE);
    ++scan_count;
}
ER tk_dly_tsk(UINT delay)
{
    now_ms += delay;
    if (++cycles > 20000) { longjmp(escape, 1); }
    if (sensor_mode)
    {
        if (is_case("scan_abort") && angle_count >= 2 && !abort_injected)
        { obs_state_flags_set(OBS_EVT_ABORT); abort_injected = 1; }
        else if (is_case("scan_abort") && abort_injected &&
                 !(flags & OBS_EVT_ABORT) && angle == 0)
        { longjmp(escape, 1); }
        if (flags & OBS_EVT_SCAN_DONE) { longjmp(escape, 1); }
        if (is_case("monitor") && (flags & OBS_EVT_BLOCKED))
        { longjmp(escape, 1); }
        if (is_case("monitor_recovered") && ping_count >= 8)
        { longjmp(escape, 1); }
        if (is_case("guard") || is_case("guard_bad") || is_case("guard_all"))
        {
            obs_guard_snapshot_t guard;
            assert(obs_state_guard_load(&guard));
            if ((guard.ranges >> 30) ||
                (is_case("guard_all") && ping_count >= 6))
            { longjmp(escape, 1); }
            if (is_case("guard_all") &&
                ((guard.ranges >> 27) & 7) == 7)
            { left_rpm = right_rpm = 12; }
        }
        if (is_case("monitor_bad") && obstacle_has_fault())
        { longjmp(escape, 1); }
        return E_OK;
    }
    int phase = obs_state_status_phase(obs_state_status_get());
    visited |= 1U << phase;
    phase_cycles = phase == last_phase ? phase_cycles + 1 : 0;
    last_phase = phase;
    if (phase == OBS_PHASE_PASS_END && phase_cycles == 0)
    {
        rear_left = motion.left_count; rear_right = motion.right_count;
        ++rear_resets;
    }
    if (phase == OBS_PHASE_TURN_BACK && !return_started)
    {
        float mm_per_count = 70.0f * 3.14159265f / 1056.0f;
        float left = (motion.left_count - rear_left) * mm_per_count;
        float right = (motion.right_count - rear_right) * mm_per_count;
        assert((left < right ? left : right) * 0.95f >=
               obs_geom_rear_clear_mm() - 1);
        return_started = 1;
    }
    /* Simulated encoders, with ideal movement proportional to wheel RPM. */
        if (!is_case("stall"))
    {
        if (!is_case("one_wheel_stall"))
        {
            motion.left_count = (int32_t)((uint32_t)motion.left_count +
                (uint32_t)(left_rpm * 0.88f + 0.5f));
        }
        motion.right_count = (int32_t)((uint32_t)motion.right_count +
            (uint32_t)(right_rpm * 0.88f + 0.5f));
    }
    if ((flags & OBS_EVT_SCAN_REQ) && !is_case("scan_timeout"))
    { supply_scan(); }
    if (phase >= OBS_PHASE_TURN_OUT && phase <= OBS_PHASE_VERIFY &&
        !is_case("guard_timeout"))
    {
        obs_guard_snapshot_t guard;
        assert(obs_state_guard_load(&guard));
        uint32_t epoch = guard.clock & 255;
        if (epoch)
        {
            for (unsigned view = 0; view < 3; ++view)
            {
                if (is_case("blind_outer") && view == OBS_GUARD_OUTER)
                { continue; }
                unsigned reason = 0;
                if (!hazard_injected && drive_count &&
                    (is_case("live_close") || is_case("live_resume")))
                { reason = OBS_GUARD_REASON_CLOSE; hazard_injected = 1; }
                if (!hazard_injected && drive_count && is_case("live_bad"))
                { reason = OBS_GUARD_REASON_SENSOR; hazard_injected = 1; }
                if (!hazard_injected && phase_cycles > 2 &&
                    ((is_case("return_close") && phase == OBS_PHASE_TURN_BACK) ||
                     (is_case("pass_close") && phase == OBS_PHASE_PASS_SIDE)))
                { reason = OBS_GUARD_REASON_CLOSE; hazard_injected = 1; }
                int range = (view == OBS_GUARD_INNER &&
                    phase == OBS_PHASE_PASS_SIDE && phase_cycles < 80)
                        ? 350 : 2000;
                assert(obs_state_guard_sample(epoch, view, range, reason));
            }
        }
    }
    if (phase == OBS_PHASE_PASS_SIDE)
    {
        assert(obs_state_watch_side_get() == (is_case("right") ? 1 : -1));
        obs_state_flags_set(OBS_EVT_SIDE_SEEN);
        if (is_case("no_seen"))
        { obs_state_flags_clear(OBS_EVT_SIDE_SEEN | OBS_EVT_SIDE_CLEAR); }
        else if (phase_cycles >= 80)
        { obs_state_flags_set(OBS_EVT_SIDE_CLEAR); }
    }
    if (is_case("rear_reappears") && phase == OBS_PHASE_PASS_END &&
        phase_cycles > 20 && !rear_reappeared)
    {
        rear_reappeared = 1;
        obs_state_flags_clear(OBS_EVT_SIDE_CLEAR);
        obs_guard_snapshot_t guard;
        assert(obs_state_guard_load(&guard));
        assert(obs_state_guard_sample(guard.clock & 255, 1, 350, 0));
    }
    if (phase == OBS_PHASE_TURN_OUT && is_case("fault"))
    { obs_state_faults_update(OBS_FAULT_NO_ECHO, 0); }
    if (phase == OBS_PHASE_TURN_OUT && is_case("odometry"))
    { odometry_bad = 1; }
    if (!(is_case("stale") && phase >= OBS_PHASE_TURN_OUT))
    {
        uint32_t left = 0, right = 0;
        if (phase >= OBS_PHASE_SEARCH_LINE && phase <= OBS_PHASE_VERIFY &&
            !is_case("lost"))
        {
            left = 1;
            if (is_case("junction")) { right = 1; }
        }
        if (is_case("blocked_junction")) { left = right = 1; }
        assert(ir_shared_publish(left, right, 0) == E_OK);
    }
    if (is_case("clear") && scan_count && !(flags & OBS_EVT_BLOCKED) &&
        left_rpm > 0) { longjmp(escape, 1); }
    if (obs_state_status_count(obs_state_status_get()) == 1 && left_rpm > 0)
    { longjmp(escape, 1); }
    if (!(flags & OBS_EVT_BLOCKED) && !is_case("front_wait") &&
        (!is_case("front_recover") || cycles > 10))
    { obs_state_front_mark(true, now_ms); }
    if (is_case("front_wait") && cycles > 30) { longjmp(escape, 1); }
    if (is_case("front_recover") && normal_count) { longjmp(escape, 1); }
    if ((phase == OBS_PHASE_FAULT || (flags & OBS_EVT_ABORT)) &&
        left_rpm == 0 && right_rpm == 0)
    { stopped = 1; }
    if (is_case("stale") && phase >= OBS_PHASE_TURN_OUT && phase_cycles > 20)
    { stopped = 1; longjmp(escape, 1); }
    if (stopped && phase_cycles > 12) { longjmp(escape, 1); }
    return E_OK;
}

static void test_profiles(void)
{
    test_obs_run();
    scan_plan_point_t points[OBS_SCAN_POINTS];
    scan_plan_bypass_t plan = {0};
    for (int route = 0; route <= 4; ++route)
    {
        make_profile(points, route);
        scan_plan_build(points, OBS_SCAN_POINTS, &plan);
        assert(scan_plan_result(&plan) == (route == 0 ? SCAN_PLAN_CLEAR :
               route == 4 ? SCAN_PLAN_NO_ROUTE : SCAN_PLAN_ROUTE));
        if (route == 2) { assert(!scan_plan_side_is_left(&plan)); }
        if (route == 3) { assert(scan_plan_side_is_left(&plan)); }
    }
    make_profile(points, 1);
    scan_plan_point_set(&points[4], -30, -1);
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_NO_ROUTE);
    scan_plan_point_set(&points[4], -33, 180);
    assert(scan_plan_point_range_mm(&points[4]) == -1);
    make_profile(points, 1);
    points[0] = points[1];
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_NO_ROUTE);
    scan_plan_build(NULL, 0, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_NO_ROUTE);
    scan_plan_build(NULL, 0, NULL);
    for (uint32_t phase = 0; phase <= OBS_PHASE_REASSESS; ++phase)
    {
        uint32_t word = obs_state_status_pack(phase, OBS_ROUTE_RIGHT,
                                             31, 15, 511, 255);
        assert(obs_state_status_phase(word) == phase);
        assert(obs_state_status_faults(word) == 31);
        assert(obs_state_status_count(word) == 15);
        assert(obs_state_status_distance_cm(word) == 511);
        assert(obs_state_status_width_cm(word) == 255);
    }
    obs_state_set_phase(OBS_PHASE_ALIGN);
    obs_state_set_route(OBS_ROUTE_LEFT);
    obs_state_faults_update(OBS_FAULT_SERVO, 0);
    obs_state_set_distance_mm(123);
    obs_state_set_width_cm(19);
    for (int i = 0; i < 20; ++i) { obs_state_count_bypass(); }
    uint32_t word = obs_state_status_get();
    assert(obs_state_status_phase(word) == OBS_PHASE_ALIGN);
    assert(obs_state_status_route(word) == OBS_ROUTE_LEFT);
    assert(obs_state_status_faults(word) == OBS_FAULT_SERVO);
    assert(obs_state_status_distance_cm(word) == 12);
    assert(obs_state_status_width_cm(word) == 19);
    assert(obs_state_status_count(word) == 15);
    bad_lock = 1;
    assert(!obs_state_plan_load(&plan));
    assert(obs_state_status_faults(obs_state_status_get()) & OBS_FAULT_TIMEOUT);
    bad_lock = 0; bad_flags = 1;
    assert(obs_state_flags_read() & OBS_EVT_ABORT);
    bad_flags = 0;
    uint32_t epoch = obs_state_guard_request(0xFFFFF0U);
    assert(epoch);
    assert(obs_state_guard_sample(epoch, 0, 999, 0));
    uint32_t next = obs_state_guard_request(0x1000010U);
    assert(next && next != epoch);
    assert(!obs_state_guard_sample(epoch, 1, 2000, 0));
    obs_guard_snapshot_t guard;
    assert(obs_state_guard_load(&guard) && guard.ranges == 0);
    assert(sizeof(guard) == 8);
    assert(obs_state_guard_sample(next, 0, 2000, OBS_GUARD_REASON_SENSOR));
    assert(obs_state_guard_sample(next, 1, 2000, OBS_GUARD_REASON_CLOSE));
    assert(obs_state_guard_load(&guard));
    assert((guard.ranges >> 30) == OBS_GUARD_REASON_SENSOR);
    obs_state_front_mark(true, 0xFFFFF0);
    assert(obs_state_front_ready(0x1000010, 250));
    assert(!obs_state_front_ready(0x1001000, 250));
    obs_state_front_mark(false, 0);
    assert(!obs_state_front_ready(0, 250));
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    scenario = argv[1];
    assert(obstacle_init() == E_OK);
    if (is_case("profiles")) { test_profiles(); }
    else if (strncmp(scenario, "scan_", 5) == 0 && !is_case("scan_timeout"))
    {
        sensor_mode = 1;
        obs_state_flags_set(OBS_EVT_BLOCKED | OBS_EVT_SCAN_REQ);
        if (!setjmp(escape)) { task_obstacle_task(0, NULL); }
        if (is_case("scan_abort")) { assert(!(flags & OBS_EVT_SCAN_DONE)); }
        else
        {
            scan_plan_bypass_t plan;
            assert(obs_state_plan_load(&plan));
            assert(flags & OBS_EVT_SCAN_DONE);
            assert(!(flags & OBS_EVT_SCAN_REQ));
            assert(angle == 0);
            assert(angle_count >= 7);
            int expected[] = {-60, -30, 0, 30, 60};
            for (int i = 0; i < 5; ++i) { assert(angles[i+1] == expected[i]); }
            assert(scan_plan_result(&plan) == (is_case("scan_clear") ?
                SCAN_PLAN_CLEAR : is_case("scan_bad") ? SCAN_PLAN_NO_ROUTE :
                SCAN_PLAN_ROUTE));
            if (is_case("scan_route")) { assert(ping_count == 45); }
        }
    }
    else if (strncmp(scenario, "monitor", 7) == 0 ||
             (strncmp(scenario, "guard", 5) == 0 &&
              !is_case("guard_timeout")))
    {
        sensor_mode = 1;
        if (strncmp(scenario, "guard", 5) == 0)
        {
            obs_state_flags_set(OBS_EVT_BLOCKED);
            obs_state_set_phase(OBS_PHASE_DIAG_OUT);
            assert(obs_state_guard_request(now_ms));
        }
        if (!setjmp(escape)) { task_obstacle_task(0, NULL); }
        if (is_case("monitor")) { assert(obstacle_is_blocked()); }
        else if (strncmp(scenario, "guard", 5) == 0)
        {
            obs_guard_snapshot_t guard;
            assert(obs_state_guard_load(&guard));
            if (is_case("guard_all"))
            {
                assert(((guard.ranges >> 27) & 7) == 7 && angle_count >= 3);
                assert(moving_pings > 0);
            }
            else { assert(guard.ranges >> 30); }
        }
        else { assert(obstacle_has_fault()); }
    }
    else
    {
        assert(ir_shared_init() == E_OK);
        assert(ir_shared_publish(0, 0, 0) == E_OK);
        if (!is_case("front_wait") && !is_case("front_recover"))
        { obs_state_flags_set(OBS_EVT_BLOCKED); }
        if (is_case("wrap")) { now_ms = UINT32_MAX - 200; }
        if (is_case("encoder_wrap"))
        { motion.left_count = motion.right_count = INT32_MAX - 100; }
        if (!setjmp(escape)) { task_controller_task(0, NULL); }
        uint32_t word = obs_state_status_get();
        if (is_case("left") || is_case("right") || is_case("wrap") ||
            is_case("encoder_wrap") || is_case("live_resume") ||
            is_case("rear_reappears"))
        {
            assert(obs_state_status_count(word) == 1);
            assert(!(flags & OBS_EVT_BLOCKED));
            assert(drive_count > 0 && normal_count > 0);
            for (unsigned i = OBS_PHASE_STOPPING; i <= OBS_PHASE_VERIFY; ++i)
            { assert(visited & (1U << i)); }
            if (is_case("rear_reappears"))
            { assert(rear_reappeared && rear_resets >= 2); }
        }
        else if (is_case("clear") || is_case("front_recover"))
        { assert(drive_count == 0 && normal_count > 0); }
        else
        {
            assert(left_rpm == 0 && right_rpm == 0);
            assert(obs_state_status_count(word) == 0);
            if (is_case("no_route")) { assert(scan_count == 2); }
            if (is_case("live_close") || is_case("live_bad") ||
                is_case("guard_timeout") || is_case("blind_outer") ||
                is_case("return_close") || is_case("pass_close"))
            {
                assert(scan_count == 2);
                assert(visited & (1U << OBS_PHASE_REASSESS));
            }
            if (is_case("guard_timeout") || is_case("blind_outer") ||
                is_case("uncalibrated")) { assert(drive_count == 0); }
            if (is_case("no_seen")) { assert(!return_started); }
            if (is_case("blocked_junction")) { assert(drive_count == 0); }
            if (!is_case("stale") && !is_case("blocked_junction") &&
                !is_case("front_wait"))
            { assert(obstacle_has_fault()); }
        }
    }
    assert(locks == unlocks && lock_depth == 0);
    printf("PASS %s (ticks=%u, drive=%u, scans=%u)\n",
           scenario, cycles, drive_count, scan_count);
    return 0;
}
