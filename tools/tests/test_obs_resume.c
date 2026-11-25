/** @file test_obs_resume.c
 * @brief Host test of FR10 "resume when the path clears" on the real
 *        obstacle.c sequencer, scan_plan.c planner and obs_state.c word:
 *        after the first scan, and after a both-sides-blocked rescan, the
 *        robot pings ahead; OBS_RESUME_HITS valid echoes in a row beyond
 *        OBS_RESUME_CLEAR_MM return it to line following without a
 *        bypass, anything else keeps the plan (FR10 scan/resume, FR11).
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <tk/tkernel.h>

/* Kernel semaphore and event flag doubles that obs_state.c needs. */
#ifndef TA_TFIFO
#define TA_TFIFO 0x00U
#endif
#ifndef TA_FIRST
#define TA_FIRST 0x00U
#endif
#ifndef TA_WMUL
#define TA_WMUL 0x08U
#endif

/** @brief Semaphore creation parameters (T-Kernel layout). */
typedef struct
{
    UW sematr;
    W  isemcnt;
    W  maxsem;
} T_CSEM;

/** @brief Event flag creation parameters (T-Kernel layout). */
typedef struct
{
    UW   flgatr;
    UINT iflgptn;
} T_CFLG;

/** @brief Event flag state as returned by tk_ref_flg(). */
typedef struct
{
    UINT flgptn;
} T_RFLG;

/** @brief Create a semaphore (double). */
ID tk_cre_sem(T_CSEM const * p_config);
/** @brief Take a semaphore (double). */
ER tk_wai_sem(ID sem, W count, TMO timeout);
/** @brief Release a semaphore (double). */
ER tk_sig_sem(ID sem, W count);
/** @brief Create an event flag (double). */
ID tk_cre_flg(T_CFLG const * p_config);
/** @brief Read an event flag (double). */
ER tk_ref_flg(ID flag, T_RFLG * p_info);
/** @brief Set event flag bits (double). */
ER tk_set_flg(ID flag, UINT bits);
/** @brief Clear event flag bits: keep only the given pattern (double). */
ER tk_clr_flg(ID flag, UINT keep);

#include "modules/obstacle/scan_plan.c"
#include "modules/obstacle/obs_state.c"
#include "modules/obstacle/obstacle.c"

/* One controller period, ms. */
#define PERIOD_MSEC 50U
/* Uniform 10 degree test scan from the right limit to the left limit. */
#define SCAN_N \
    (((OBS_SCAN_LEFT_DEG + OBS_SCAN_RIGHT_DEG) / OBS_SCAN_STEP_DEG) + 1)
#define SCAN_ZERO (OBS_SCAN_RIGHT_DEG / OBS_SCAN_STEP_DEG)
/* A far, plainly clear front echo. */
#define FAR_CLEAR_MM 1000
/* A near echo: the object is still there. */
#define NEAR_MM 300

static UINT     g_pattern = 0U;
static uint32_t g_now     = 1000U;

/** @brief Monitor output, ignored. */
INT tm_printf(UB const * p_format, ...)
{
    (void)p_format;
    return 0;
}

/** @brief Semaphore double: always created. */
ID tk_cre_sem(T_CSEM const * p_config)
{
    (void)p_config;
    return 1;
}

/** @brief Semaphore double: always taken. */
ER tk_wai_sem(ID sem, W count, TMO timeout)
{
    (void)sem;
    (void)count;
    (void)timeout;
    return E_OK;
}

/** @brief Semaphore double: release. */
ER tk_sig_sem(ID sem, W count)
{
    (void)sem;
    (void)count;
    return E_OK;
}

/** @brief Event flag double: always created. */
ID tk_cre_flg(T_CFLG const * p_config)
{
    (void)p_config;
    return 2;
}

/** @brief Event flag double: read the pattern. */
ER tk_ref_flg(ID flag, T_RFLG * p_info)
{
    (void)flag;
    p_info->flgptn = g_pattern;
    return E_OK;
}

/** @brief Event flag double: set bits. */
ER tk_set_flg(ID flag, UINT bits)
{
    (void)flag;
    g_pattern |= bits;
    return E_OK;
}

/** @brief Event flag double: T-Kernel ANDs the pattern with the mask. */
ER tk_clr_flg(ID flag, UINT keep)
{
    (void)flag;
    g_pattern &= keep;
    return E_OK;
}

/** @brief Encoders: the robot stands still while it scans. */
ER motion_get_status(motion_status_t * p_output)
{
    motion_status_t const zero = {0};

    *p_output         = zero;
    p_output->b_valid = 1U;
    return E_OK;
}

/** @brief Servo reach to the left. */
int32_t servo_left_limit_deg(void)
{
    return OBS_SERVO_LEFT_LIMIT_DEG;
}

/** @brief Servo reach to the right. */
int32_t servo_right_limit_deg(void)
{
    return OBS_SERVO_RIGHT_LIMIT_DEG;
}

/** @brief Unused driver start-up (link only). */
ER ultrasonic_init(void)
{
    return E_OK;
}

/** @brief Unused driver start-up (link only). */
ER servo_init(void)
{
    return E_OK;
}

/** @brief Unused task creation (link only). */
ID tasks_create(tasks_entry_t p_entry, PRI priority)
{
    (void)p_entry;
    (void)priority;
    return 1;
}

/** @brief Unused task start (link only). */
ER tk_sta_tsk(ID task, INT code)
{
    (void)task;
    (void)code;
    return E_OK;
}

/** @brief Unused worker entry (link only). */
void task_obstacle_task(INT start_code, void * p_context)
{
    (void)start_code;
    (void)p_context;
}

/** @brief Fill a uniform scan with one range everywhere. */
static void scan_fill(scan_plan_point_t * p_points, int32_t range_mm)
{
    int32_t index = 0;

    for (index = 0; index < SCAN_N; index++)
    {
        p_points[index].angle_deg =
            (index * OBS_SCAN_STEP_DEG) - OBS_SCAN_RIGHT_DEG;
        p_points[index].range_mm  = (uint32_t)range_mm;
        p_points[index].lone_mm   = 0U;
    }
}

/** @brief Real planner on a small box straight ahead: a route. */
static scan_plan_bypass_t plan_route(void)
{
    scan_plan_point_t  points[SCAN_N];
    scan_plan_bypass_t plan = {0U, 0U};

    scan_fill(points, 1500);
    points[SCAN_ZERO - 1].range_mm = 354U;
    points[SCAN_ZERO].range_mm     = 350U;
    points[SCAN_ZERO + 1].range_mm = 354U;
    scan_plan_build(points, SCAN_N, &plan);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    return plan;
}

/** @brief Real planner with both corridors blocked: no route. */
static scan_plan_bypass_t plan_blocked(void)
{
    scan_plan_point_t  points[SCAN_N];
    scan_plan_bypass_t plan = {0U, 0U};

    scan_fill(points, 1500);
    points[SCAN_ZERO - 2].range_mm = 400U;
    points[SCAN_ZERO - 1].range_mm = 254U;
    points[SCAN_ZERO].range_mm     = 250U;
    points[SCAN_ZERO + 1].range_mm = 254U;
    points[SCAN_ZERO + 2].range_mm = 400U;
    scan_plan_build(points, SCAN_N, &plan);
    assert(SCAN_PLAN_NO_ROUTE == scan_plan_result(&plan));
    return plan;
}

/** @brief One controller period through the public entry point. */
static obstacle_drive_t step(void)
{
    g_now += PERIOD_MSEC;
    return obstacle_bypass_step(0U, 0U, g_now);
}

/** @brief Phase published in the status word. */
static uint32_t phase_now(void)
{
    return (uint32_t)obs_state_status_phase(obs_state_status_get());
}

/** @brief Fresh obstacle: stop, settle and request a scan. */
static void begin(void)
{
    obstacle_drive_t out = {0};

    g_seq.b_active = false;
    g_pattern      = 0U;
    assert(E_OK == obs_state_init());
    out = step();
    assert(out.b_stop && (OBS_PHASE_STOPPING == phase_now()));
    g_now += OBS_STOP_SETTLE_MSEC;
    out = step();
    assert(out.b_stop && (OBS_PHASE_SCANNING == phase_now()));
    assert(0U != (g_pattern & OBS_EVT_SCAN_REQ));
}

/** @brief The scanner task hands over a plan, as task_obstacle does. */
static obstacle_drive_t scan_done(scan_plan_bypass_t const * p_plan)
{
    obs_state_plan_store(p_plan);
    obs_state_flags_clear(OBS_EVT_SCAN_REQ);
    obs_state_flags_set(OBS_EVT_SCAN_DONE);
    return step();
}

/** @brief The front watch publishes one ping (-1: no echo). */
static obstacle_drive_t ping(int32_t range_mm)
{
    obs_state_front_range_set(range_mm);
    return step();
}

/** @brief Still stopped and pinging ahead, plan not yet used. */
static void expect_rechecking(obstacle_drive_t const * p_out)
{
    assert(p_out->b_stop && !p_out->b_done && !p_out->b_fault);
    assert((0 == p_out->left_rpm) && (0 == p_out->right_rpm));
    assert(OBS_PHASE_SCANNING == phase_now());
    assert(0U != (g_pattern & OBS_EVT_FRONT_WATCH));
    assert(OBS_ROUTE_NONE == obs_state_status_route(obs_state_status_get()));
}

/** @brief Resumed line following: no turn, no route, no bypass counted. */
static void expect_resumed(obstacle_drive_t const * p_out)
{
    uint32_t const word = obs_state_status_get();

    assert(p_out->b_done && p_out->b_stop && !p_out->b_fault);
    assert((0 == p_out->left_rpm) && (0 == p_out->right_rpm));
    assert(OBS_PHASE_IDLE == obs_state_status_phase(word));
    assert(OBS_ROUTE_NONE == obs_state_status_route(word));
    assert(0U == obs_state_status_count(word));
    assert(0U == obs_state_status_faults(word));
    assert(0U == (g_pattern & (OBS_EVT_FRONT_WATCH | OBS_EVT_SCAN_REQ |
                               OBS_EVT_SCAN_DONE | OBS_EVT_BLOCKED)));
    assert(!g_seq.b_active);
}

/** @brief The plan was kept: turning out on the planned side. */
static void expect_plan_kept(scan_plan_bypass_t const * p_plan)
{
    bool const       b_left = scan_plan_side_is_left(p_plan);
    obstacle_drive_t out    = {0};

    assert(OBS_PHASE_TURN_OUT == phase_now());
    assert((b_left ? OBS_ROUTE_LEFT : OBS_ROUTE_RIGHT) ==
           obs_state_status_route(obs_state_status_get()));
    out = step();
    assert(!out.b_done && !out.b_stop && !out.b_fault);
    /* Pivot on the inner wheel towards the planned side. */
    assert((b_left ? 0 : OBS_DRIVE_RPM) == out.left_rpm);
    assert((b_left ? OBS_DRIVE_RPM : 0) == out.right_rpm);
}

/** @brief Object removed during the scan: HITS clear echoes resume. */
static void test_removed_resumes(void)
{
    scan_plan_bypass_t const plan = plan_route();
    obstacle_drive_t         out  = {0};
    uint32_t                 hit  = 0U;

    begin();
    out = scan_done(&plan);
    expect_rechecking(&out);

    for (hit = 1U; hit < OBS_RESUME_HITS; hit++)
    {
        out = ping(FAR_CLEAR_MM);
        expect_rechecking(&out);
    }

    /* Just beyond the limit is clear. */
    out = ping(OBS_RESUME_CLEAR_MM + 1);
    expect_resumed(&out);
    puts("PASS FR10 object removed during scan: resumes after "
         "OBS_RESUME_HITS clear pings, no turn-out, no bypass counted");
}

/** @brief The scanner itself found nothing ahead: resume at once. */
static void test_clear_scan_resumes(void)
{
    scan_plan_bypass_t const clear = {SCAN_PLAN_CLEAR, 0U};
    obstacle_drive_t         out   = {0};

    begin();
    out = scan_done(&clear);
    expect_resumed(&out);
    puts("PASS FR10 clear scan resumes line following without pinging");
}

/** @brief Object still there: one echo at the limit keeps the plan. */
static void test_still_there_keeps_plan(void)
{
    scan_plan_bypass_t const plan = plan_route();
    obstacle_drive_t         out  = {0};
    uint32_t                 hit  = 0U;

    begin();
    (void)scan_done(&plan);

    for (hit = 1U; hit < OBS_RESUME_HITS; hit++)
    {
        out = ping(FAR_CLEAR_MM);
        expect_rechecking(&out);
    }

    /* At the limit is not beyond it: the object is still there. */
    out = ping(OBS_RESUME_CLEAR_MM);
    assert(!out.b_done && out.b_stop);
    expect_plan_kept(&plan);
    puts("PASS FR10 echo at OBS_RESUME_CLEAR_MM after a clear run keeps "
         "the bypass plan");

    begin();
    (void)scan_done(&plan);
    out = ping(NEAR_MM);
    assert(!out.b_done);
    expect_plan_kept(&plan);
    puts("PASS FR10 first echo inside the limit commits at once (no "
         "extra pings)");
}

/** @brief Missing echoes are never clear and restart the clear run. */
static void test_missing_never_clear(void)
{
    scan_plan_bypass_t const plan = plan_route();
    obstacle_drive_t         out  = {0};
    uint32_t                 n    = 0U;

    begin();
    (void)scan_done(&plan);

    for (n = 1U; n < OBS_RESUME_PINGS; n++)
    {
        out = ping(-1);
        expect_rechecking(&out);
    }

    out = ping(-1);
    assert(!out.b_done);
    expect_plan_kept(&plan);
    puts("PASS FR10 only missing echoes: never resumes, plan kept after "
         "OBS_RESUME_PINGS");

    /* clear, clear, miss, clear: a miss must reset the run, or the
     * fourth ping would complete it. */
    begin();
    (void)scan_done(&plan);
    out = ping(FAR_CLEAR_MM);
    expect_rechecking(&out);
    out = ping(FAR_CLEAR_MM);
    expect_rechecking(&out);
    out = ping(-1);
    expect_rechecking(&out);
    out = ping(FAR_CLEAR_MM);
    expect_rechecking(&out);
    puts("PASS FR10 a missing echo restarts the clear run");
}

/** @brief OBS_RESUME_PINGS without a full clear run keeps the plan. */
static void test_ping_limit_keeps_plan(void)
{
    scan_plan_bypass_t const plan = plan_route();
    obstacle_drive_t         out  = {0};
    uint32_t                 n    = 0U;

    begin();
    (void)scan_done(&plan);

    /* Two clear then a miss, repeated: never OBS_RESUME_HITS in a row. */
    for (n = 1U; n < OBS_RESUME_PINGS; n++)
    {
        out = ping(((n % OBS_RESUME_HITS) == 0U) ? -1 : FAR_CLEAR_MM);
        expect_rechecking(&out);
    }

    out = ping(-1);
    assert(!out.b_done);
    expect_plan_kept(&plan);
    puts("PASS FR10 OBS_RESUME_PINGS without a clear run keeps the plan");
}

/** @brief A ping is counted once: periods without a new front ping do
 *  not advance the check, and a dead sonar ends in a timeout fault, not
 *  an endless wait or a resume.
 */
static void test_stale_and_timeout(void)
{
    scan_plan_bypass_t const plan  = plan_route();
    obstacle_drive_t         out   = {0};
    uint32_t                 start = 0U;

    begin();
    out   = scan_done(&plan);
    start = g_seq.enter_msec;
    out   = ping(FAR_CLEAR_MM);

    while ((g_now - start) < OBS_SCAN_TIMEOUT_MSEC)
    {
        expect_rechecking(&out);
        out = step();
    }

    /* The same reading was never counted again. */
    assert(1U == g_seq.recheck_clear);
    out = step();
    assert(out.b_fault && out.b_stop && !out.b_done);
    assert(OBS_PHASE_FAULT == phase_now());
    assert(0U != (obs_state_status_faults(obs_state_status_get()) &
                  OBS_FAULT_TIMEOUT));
    puts("PASS FR10 stale ping not re-counted; silent sonar times out "
         "to a fault");
}

/** @brief Both sides blocked: wait and rescan; the rescan pings ahead
 *  again and resumes if the way has cleared by then.
 */
static void test_rescan_then_resume(void)
{
    scan_plan_bypass_t const blocked = plan_blocked();
    obstacle_drive_t         out     = {0};
    uint32_t                 hit     = 0U;

    begin();
    out = scan_done(&blocked);
    expect_rechecking(&out);
    out = ping(NEAR_MM);
    assert(out.b_stop && !out.b_fault && !out.b_done);
    assert(OBS_PHASE_STOPPING == phase_now());
    g_now += OBS_STOP_SETTLE_MSEC;
    out = step();
    assert(OBS_PHASE_SCANNING == phase_now());
    assert(0U != (g_pattern & OBS_EVT_SCAN_REQ));
    out = scan_done(&blocked);
    expect_rechecking(&out);

    for (hit = 1U; hit < OBS_RESUME_HITS; hit++)
    {
        out = ping(FAR_CLEAR_MM);
        expect_rechecking(&out);
    }

    out = ping(FAR_CLEAR_MM);
    expect_resumed(&out);
    puts("PASS FR10 both sides blocked: rescan pings ahead again and "
         "resumes once clear");
}

/** @brief Blocked on every rescan: the wait is bounded and ends in a
 *  no-route fault after OBS_NO_ROUTE_RESCANS scans.
 */
static void test_rescan_bounded(void)
{
    scan_plan_bypass_t const blocked = plan_blocked();
    obstacle_drive_t         out     = {0};
    uint32_t                 scans   = 0U;

    begin();

    for (scans = 1U; scans <= OBS_NO_ROUTE_RESCANS; scans++)
    {
        out = scan_done(&blocked);
        expect_rechecking(&out);
        out = ping(NEAR_MM);

        if (scans < OBS_NO_ROUTE_RESCANS)
        {
            assert(!out.b_fault && (OBS_PHASE_STOPPING == phase_now()));
            g_now += OBS_STOP_SETTLE_MSEC;
            (void)step();
            assert(OBS_PHASE_SCANNING == phase_now());
        }
    }

    assert(out.b_fault && (OBS_PHASE_FAULT == phase_now()));
    assert(0U != (obs_state_status_faults(obs_state_status_get()) &
                  OBS_FAULT_NO_ROUTE));
    puts("PASS FR10 blocked on every rescan: no-route fault after "
         "OBS_NO_ROUTE_RESCANS scans");
}

/** @brief Run the FR10 resume tests. */
int main(void)
{
    test_removed_resumes();
    test_clear_scan_resumes();
    test_still_there_keeps_plan();
    test_missing_never_clear();
    test_ping_limit_keeps_plan();
    test_stale_and_timeout();
    test_rescan_then_resume();
    test_rescan_bounded();
    return 0;
}

/*** end of file ***/

