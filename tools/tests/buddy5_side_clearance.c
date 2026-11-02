/* Host regression of the Buddy 5 sequencer: side readings, rear-edge
 * evidence, passing clearance and the return check.
 *
 * The production obstacle.c is included so its private helpers are driven
 * directly; readings go through the real obs_state.c, and the planner and
 * geometry are the production files. Only the RTOS, the servo driver, the
 * motion module and the console are doubles.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "modules/obstacle/obstacle.c"

#define LOOK 70
#define SERVO_REACH 70

static uint32_t g_time = 1000U;
static UINT g_flags;
static int g_checks;
static motion_status_t g_motion;

/* Kernel, driver and console doubles. */
INT tm_printf(const UB *format, ...) { (void)format; return 0; }
ID tk_cre_sem(const T_CSEM *config) { (void)config; return 1; }
ER tk_wai_sem(ID sem, INT count, TMO timeout)
{
    (void)sem; (void)count; (void)timeout;
    return E_OK;
}
ER tk_sig_sem(ID sem, INT count) { (void)sem; (void)count; return E_OK; }
ID tk_cre_flg(const T_CFLG *config) { g_flags = config->iflgptn; return 2; }
ER tk_ref_flg(ID flag, T_RFLG *info)
{
    (void)flag;
    info->flgptn = g_flags;
    return E_OK;
}
ER tk_set_flg(ID flag, UINT bits) { (void)flag; g_flags |= bits; return E_OK; }
ER tk_clr_flg(ID flag, UINT keep) { (void)flag; g_flags &= keep; return E_OK; }
ER tk_sta_tsk(ID task, INT code) { (void)task; (void)code; return E_OK; }
ID tasks_create(tasks_entry_t entry, PRI priority)
{
    (void)entry; (void)priority;
    return 3;
}
void task_obstacle_task(INT start_code, void *p_context)
{
    (void)start_code; (void)p_context;
}
ER ultrasonic_init(void) { return E_OK; }
ER servo_init(void) { return E_OK; }
int32_t servo_left_limit_deg(void) { return SERVO_REACH; }
int32_t servo_right_limit_deg(void) { return SERVO_REACH; }
ER motion_get_status(motion_status_t *p_output)
{
    *p_output = g_motion;
    return E_OK;
}

static void pass(const char *name)
{
    g_checks++;
    printf("PASS %s\n", name);
}

/* A car passing a 150 mm box on the given side, as after the plan: the
 * box's near side 75 mm from the line, the car 320 mm out.
 */
static void setup(obstacle_seq_t *p, int32_t side)
{
    obs_state_reading_t latest;

    memset(p, 0, sizeof(*p));
    p->b_active       = true;
    p->phase          = OBS_PHASE_PASS_SIDE;
    p->side           = side;
    p->pose.x_mm      = 300.0F;
    p->pose.y_mm      = (float)(side * 320);
    p->lateral_mm     = 320;
    p->lateral_max_mm = 380;
    p->face_x_mm      = 420.0F;
    p->face_y_mm      = (float)(side * 75);
    p->rear_guess_mm  = 570.0F;
    p->search_side    = side;
    p->now_msec       = g_time;
    obs_state_side_reading_get(&latest);
    p->side_seq = latest.seq;
    obs_state_front_reading_get(&latest);
    p->front_seq = latest.seq;
    aim_side_watch(p);
}

/* Publish a ping at the bearing asked for and let the sequencer take it. */
static bool side_ping(obstacle_seq_t *p, obs_state_echo_t echo, int32_t mm)
{
    obs_state_reading_t reading;
    bool taken;

    g_time += 60U;
    p->now_msec = g_time;
    obs_state_side_reading_set(echo, mm, p->look_deg, g_time);
    taken = side_reading_take(p, &reading);
    if (taken)
    {
        (void)side_update(p, &reading);
    }
    return taken;
}

/* Range at 70 degrees to a side face this far from the car's centre. */
static int32_t range_for_gap(int32_t gap_mm)
{
    return (gap_mm * 1000) / 940;
}

static void test_reading_intake(int32_t side)
{
    obstacle_seq_t s;
    obs_state_reading_t reading;

    setup(&s, side);
    assert(s.look_deg == -side * LOOK);

    /* A ping taken at another bearing describes something else. */
    g_time += 60U;
    s.now_msec = g_time;
    obs_state_side_reading_set(OBS_ECHO_VALID, 260, -side * 60, g_time);
    assert(!side_reading_take(&s, &reading));

    /* A ping finished before the bearing was asked for is stale. */
    set_look(&s, -side * LOOK);
    obs_state_side_reading_set(OBS_ECHO_VALID, 260, -side * LOOK,
                               g_time - 10U);
    assert(!side_reading_take(&s, &reading));

    /* The same ping is never taken twice. */
    obs_state_side_reading_set(OBS_ECHO_VALID, 260, -side * LOOK, g_time);
    assert(side_reading_take(&s, &reading));
    assert(!side_reading_take(&s, &reading));

    set_look(&s, side * 120);
    assert(s.look_deg == side * SERVO_REACH);
    pass(side > 0 ? "L: only fresh pings at the asked-for bearing are used"
                  : "R: only fresh pings at the asked-for bearing are used");
}

static void test_side_evidence(int32_t side)
{
    obstacle_seq_t s;
    int32_t const hit = range_for_gap(245);

    setup(&s, side);
    assert(side_ping(&s, OBS_ECHO_VALID, hit));
    assert(!s.b_seen);
    assert(side_ping(&s, OBS_ECHO_VALID, hit));
    assert(s.b_seen);
    pass(side > 0 ? "L: two side hits are needed to confirm the object"
                  : "R: two side hits are needed to confirm the object");

    /* A missing echo never certifies the end, and never erases evidence. */
    for (int i = 0; i < 3; i++)
    {
        assert(side_ping(&s, OBS_ECHO_NONE, 0));
    }
    assert(0U == s.clear_hits);
    assert(!rear_clear(&s));
    assert(!sensor_lost(&s));
    assert(side_ping(&s, OBS_ECHO_NONE, 0));
    assert(side_ping(&s, OBS_ECHO_NONE, 0));
    assert(sensor_lost(&s));
    assert(side_ping(&s, OBS_ECHO_BEYOND, 0));
    assert(!sensor_lost(&s));
    pass(side > 0 ? "L: missing echoes are unknown; five in a row are a fault"
                  : "R: missing echoes are unknown; five in a row are a fault");

    /* Beyond range with no travel is not an end. */
    setup(&s, side);
    (void)side_ping(&s, OBS_ECHO_VALID, hit);
    (void)side_ping(&s, OBS_ECHO_VALID, hit);
    for (int i = 0; i < 3; i++)
    {
        (void)side_ping(&s, OBS_ECHO_BEYOND, 0);
    }
    assert(OBS_SIDE_CLEAR_HITS == s.clear_hits);
    assert(!rear_clear(&s));
    /* Travel counts from the axle at the last hit, not the echo point,
     * which sits about 200 mm further ahead.
     */
    s.pose.x_mm = s.hit_axle_mm + (float)OBS_SIDE_CLEAR_TRAVEL_MM - 1.0F;
    assert(!rear_clear(&s));
    s.pose.x_mm = s.hit_axle_mm + (float)OBS_SIDE_CLEAR_TRAVEL_MM + 1.0F;
    assert(rear_clear(&s));
    pass(side > 0 ? "L: the end needs clear pings and axle travel past the hit"
                  : "R: the end needs clear pings and axle travel past the hit");

    /* One clear ping too few is still not an end. */
    setup(&s, side);
    (void)side_ping(&s, OBS_ECHO_VALID, hit);
    (void)side_ping(&s, OBS_ECHO_VALID, hit);
    (void)side_ping(&s, OBS_ECHO_BEYOND, 0);
    (void)side_ping(&s, OBS_ECHO_BEYOND, 0);
    s.pose.x_mm += 500.0F;
    assert(!rear_clear(&s));
    pass(side > 0 ? "L: beyond range without enough clear pings is not clear"
                  : "R: beyond range without enough clear pings is not clear");

    /* The object reappears: the evidence starts again and the rear moves. */
    (void)side_ping(&s, OBS_ECHO_BEYOND, 0);
    assert(rear_clear(&s));
    float const rear_before = s.rear_x_mm;
    (void)side_ping(&s, OBS_ECHO_VALID, hit);
    assert(!rear_clear(&s));
    assert(s.rear_x_mm > rear_before);
    pass(side > 0 ? "L: a new side hit cancels the end and moves the rear on"
                  : "R: a new side hit cancels the end and moves the rear on");

    /* A ping inside the car's own body range is unknown, not a hit. */
    setup(&s, side);
    (void)side_ping(&s, OBS_ECHO_VALID, OBS_SIDE_MIN_MM - 10);
    assert(0U == s.side_hits);
    /* An echo from beyond the line means the band was clear. */
    (void)side_ping(&s, OBS_ECHO_VALID, hit);
    (void)side_ping(&s, OBS_ECHO_VALID, hit);
    assert(SIDE_CLEAR == side_update(&s, &(obs_state_reading_t){
        g_time, (int16_t)range_for_gap(400), (int8_t)(-side * LOOK),
        OBS_ECHO_VALID, 0U}));
    pass(side > 0 ? "L: own-body echoes are unknown; echoes past the line clear"
                  : "R: own-body echoes are unknown; echoes past the line clear");
}

static void test_rear_estimate(int32_t side)
{
    obstacle_seq_t s;
    int32_t const hit = range_for_gap(245);

    setup(&s, side);
    (void)side_ping(&s, OBS_ECHO_VALID, hit);
    (void)side_ping(&s, OBS_ECHO_VALID, hit);
    /* Evidence ends before the assumed depth: the guess still holds. */
    assert(s.rear_x_mm < s.rear_guess_mm);
    assert(turn_point(&s) ==
           s.rear_guess_mm + (float)(OBS_ROBOT_REAR_MM +
                                     OBS_REAR_PASS_MARGIN_MM));
    /* Clear views along the face beyond the guess disprove it. */
    for (int i = 0; i < 6; i++)
    {
        s.pose.x_mm += 25.0F;
        (void)side_ping(&s, OBS_ECHO_BEYOND, 0);
    }
    assert(s.b_clear_axis && (s.clear_axis_mm >= s.rear_guess_mm));
    assert(turn_point(&s) ==
           s.rear_x_mm + (float)(OBS_ROBOT_REAR_MM + OBS_REAR_PASS_MARGIN_MM));
    pass(side > 0 ? "L: the assumed depth yields to a clear view of the side"
                  : "R: the assumed depth yields to a clear view of the side");

    /* A long object: hits keep moving the rear; no assumed length cuts
     * the pass short.
     */
    setup(&s, side);
    for (int i = 0; i < 20; i++)
    {
        s.pose.x_mm += 40.0F;
        (void)side_ping(&s, OBS_ECHO_VALID, hit);
        assert(!rear_clear(&s));
    }
    assert(s.rear_x_mm > (s.rear_guess_mm + 500.0F));
    pass(side > 0 ? "L: a long object is passed for as long as it is seen"
                  : "R: a long object is passed for as long as it is seen");
}

static void test_clearance(int32_t side)
{
    obstacle_seq_t s;

    /* The side is 130 mm from the car's centre: 21 mm short of the body
     * margin, so the target offset grows by exactly that, once.
     */
    setup(&s, side);
    (void)side_ping(&s, OBS_ECHO_VALID, range_for_gap(130));
    int32_t const once = s.lateral_mm;
    assert((once >= 338) && (once <= 344));
    (void)side_ping(&s, OBS_ECHO_VALID, range_for_gap(130));
    (void)side_ping(&s, OBS_ECHO_VALID, range_for_gap(130));
    assert(s.lateral_mm == once);
    assert(!s.b_too_close);
    assert(offset_heading(&s) * side > 0);
    pass(side > 0 ? "L: too close: the offset grows once, steering outwards"
                  : "R: too close: the offset grows once, steering outwards");

    /* Needing more than the checked corridor allows: decide again. */
    setup(&s, side);
    (void)side_ping(&s, OBS_ECHO_VALID, range_for_gap(70));
    (void)side_ping(&s, OBS_ECHO_VALID, range_for_gap(70));
    assert(s.b_too_close);
    assert(s.lateral_mm <= s.lateral_max_mm);
    assert(CLOSE_HITS_LIMIT == s.close_hits);
    pass(side > 0 ? "L: a gap beyond the checked room stops for a new decision"
                  : "R: a gap beyond the checked room stops for a new decision");
}

/* The car beside the box with the object's end established. */
static void passed_rear(obstacle_seq_t *p, int32_t side)
{
    int32_t const hit = range_for_gap(245);

    setup(p, side);
    (void)side_ping(p, OBS_ECHO_VALID, hit);
    (void)side_ping(p, OBS_ECHO_VALID, hit);
    p->rear_guess_mm = 0.0F;
    for (int i = 0; i < 3; i++)
    {
        (void)side_ping(p, OBS_ECHO_BEYOND, 0);
    }
    p->pose.x_mm = turn_point(p);
    p->phase = OBS_PHASE_PASS_END;
    assert(rear_clear(p));
}

static bool probe_ping(obstacle_seq_t *p, obs_state_echo_t echo, int32_t mm)
{
    obs_state_reading_t reading;
    bool goes_on = false;

    g_time += 60U;
    p->now_msec = g_time;
    obs_state_side_reading_set(echo, mm, p->look_deg, g_time);
    assert(side_reading_take(p, &reading));
    goes_on = probe_update(p, &reading);
    return goes_on;
}

static void test_return(int32_t side)
{
    obstacle_seq_t s;

    passed_rear(&s, side);
    assert(!return_turn_safe(&s));
    aim_return_probe(&s);
    assert(s.look_deg == -side * OBS_RETURN_DEG);
    assert(!return_turn_safe(&s));
    assert(!probe_ping(&s, OBS_ECHO_BEYOND, 0));
    assert(return_turn_safe(&s));
    pass(side > 0 ? "L: clear way back: turn at once on a fresh probe"
                  : "R: clear way back: turn at once on a fresh probe");

    /* A probe from too far back is stale for this spot. */
    s.pose.x_mm += (float)(2 * OBS_RETURN_PROBE_LEAD_MM) + 10.0F;
    assert(!return_turn_safe(&s));
    assert(!probe_ping(&s, OBS_ECHO_NONE, 0));
    assert(!return_turn_safe(&s));
    pass(side > 0 ? "L: a stale or missing probe never allows the turn"
                  : "R: a stale or missing probe never allows the turn");

    /* Something in the band on the way back: the object goes on. */
    passed_rear(&s, side);
    aim_return_probe(&s);
    float const rear_before = s.rear_x_mm;
    assert(probe_ping(&s, OBS_ECHO_VALID, 200));
    assert(s.rear_x_mm > rear_before);
    assert(!rear_clear(&s));
    pass(side > 0 ? "L: the object reappearing before the turn keeps it passing"
                  : "R: the object reappearing before the turn keeps it passing");

    /* Another object just past the line, on the run back. */
    passed_rear(&s, side);
    aim_return_probe(&s);
    assert(!probe_ping(&s, OBS_ECHO_VALID, 427));
    assert(s.b_probe_clear && s.b_probe_point);
    assert(!return_path_clear(&s));
    assert(!return_turn_safe(&s));
    pass(side > 0 ? "L: an object in the swept return path blocks the turn"
                  : "R: an object in the swept return path blocks the turn");

    /* Rear of the car not yet past the object's rear. */
    passed_rear(&s, side);
    aim_return_probe(&s);
    (void)probe_ping(&s, OBS_ECHO_BEYOND, 0);
    s.pose.x_mm = turn_point(&s) - 30.0F;
    assert(!return_turn_safe(&s));
    pass(side > 0 ? "L: no turn until the car's rear is past the object's rear"
                  : "R: no turn until the car's rear is past the object's rear");

    /* Hugging the side: the inner rear corner needs more travel to keep
     * its margin from the object's rear corner during the pivot.
     */
    passed_rear(&s, side);
    s.face_y_mm = (float)side * (320.0F - 100.0F);
    s.pose.x_mm = s.rear_x_mm + (float)(OBS_ROBOT_REAR_MM +
                                        OBS_REAR_PASS_MARGIN_MM);
    assert(!return_path_clear(&s));
    s.pose.x_mm += 60.0F;
    assert(return_path_clear(&s));
    pass(side > 0 ? "L: the swept body clears the rear corner before turning"
                  : "R: the swept body clears the rear corner before turning");
}

static void test_watches(int32_t side)
{
    obstacle_seq_t s;
    obs_state_reading_t reading;

    setup(&s, side);
    set_look(&s, side * LOOK);
    for (int i = 0; i < 2; i++)
    {
        g_time += 60U;
        s.now_msec = g_time;
        obs_state_side_reading_set(OBS_ECHO_VALID, 150, side * LOOK, g_time);
        (void)outward_blocked(&s);
    }
    assert(outward_blocked(&s));
    g_time += 60U;
    s.now_msec = g_time;
    obs_state_side_reading_set(OBS_ECHO_BEYOND, 0, side * LOOK, g_time);
    assert(!outward_blocked(&s));
    pass(side > 0 ? "L: two close pings on the outward side stop the departure"
                  : "R: two close pings on the outward side stop the departure");

    /* The way ahead counts as verified only after a real reading, and
     * only for a while.
     */
    setup(&s, side);
    assert(!front_fresh(&s));
    obs_state_front_reading_set(OBS_ECHO_VALID, 900, g_time);
    assert(!guard_tripped(&s, false));
    assert(front_fresh(&s));
    s.pose.x_mm += (float)OBS_FRONT_STALE_MM + 1.0F;
    assert(!front_fresh(&s));
    for (int i = 0; i < (int)OBS_SIDE_NO_ECHO_LIMIT; i++)
    {
        obs_state_front_reading_set(OBS_ECHO_NONE, 0, g_time);
        (void)guard_tripped(&s, false);
        assert(!front_fresh(&s));
    }
    assert(sensor_lost(&s));
    obs_state_front_reading_set(OBS_ECHO_VALID, 150, g_time);
    (void)guard_tripped(&s, false);
    obs_state_front_reading_set(OBS_ECHO_VALID, 150, g_time);
    assert(guard_tripped(&s, false));
    pass(side > 0 ? "L: forward clearance must be fresh and real to drive on"
                  : "R: forward clearance must be fresh and real to drive on");

    /* Old pings from a previous bypass are not reused. */
    obs_state_side_reading_set(OBS_ECHO_VALID, 260, -side * LOOK, g_time);
    g_motion.b_valid = true;
    seq_begin(&s, 0, 0, g_time);
    s.look_deg = -side * LOOK;
    assert(!side_reading_take(&s, &reading));
    assert(!s.b_seen && (0.0F == s.rear_x_mm));
    pass(side > 0 ? "L: a new bypass starts with no old evidence"
                  : "R: a new bypass starts with no old evidence");
}

static void test_line(void)
{
    obstacle_seq_t s;

    setup(&s, 1);
    s.pose.heading_rad = -60.0F * OBS_RAD_PER_DEG;
    s.pose.y_mm = 300.0F;
    assert(!line_near(&s, true));
    s.pose.y_mm = 150.0F;
    assert(line_near(&s, true));
    assert(!line_near(&s, false));
    pass("black counts as the line only where the line should be");
}

static void test_faults(void)
{
    obstacle_seq_t s;
    obstacle_drive_t out = {0, 0, false, false, false, false};
    scan_plan_bypass_t plan = {0U, 0U};

    setup(&s, 1);
    wait_for_reading(&s, &out);
    assert(out.b_stop && !out.b_fault);
    s.now_msec += OBS_READING_WAIT_MSEC;
    wait_for_reading(&s, &out);
    assert(!out.b_fault);
    s.now_msec += 2U;
    wait_for_reading(&s, &out);
    assert(out.b_fault && (OBS_PHASE_FAULT == s.phase));
    pass("waiting for a reading is bounded; then a servo fault stops it");

    /* A plan is used only once the sensor is settled back at FRONT. */
    plan.word_a = SCAN_PLAN_ROUTE;
    obs_state_plan_store(&plan);
    setup(&s, 1);
    s.phase = OBS_PHASE_SCANNING;
    memset(&out, 0, sizeof(out));
    obs_state_aim_set(OBS_AIM_SETTLING, 0, g_time);
    handle_plan(&s, &out, g_time);
    assert(out.b_fault && (OBS_PHASE_FAULT == s.phase));
    obs_state_faults_update(0U, 0xFFU);
    setup(&s, 1);
    s.phase = OBS_PHASE_SCANNING;
    memset(&out, 0, sizeof(out));
    obs_state_aim_set(OBS_AIM_FORWARD, 0, g_time);
    handle_plan(&s, &out, g_time);
    assert(!out.b_fault && (OBS_PHASE_TURN_OUT == s.phase));
    assert(s.lateral_max_mm >= s.lateral_mm);
    assert(s.lateral_max_mm <= (s.lateral_mm + OBS_PASS_CORRECT_MAX_MM));
    pass("movement starts only with the sensor settled at FRONT");
}

int main(void)
{
    assert(E_OK == obs_state_init());

    for (int32_t side = 1; side >= -1; side -= 2)
    {
        test_reading_intake(side);
        test_side_evidence(side);
        test_rear_estimate(side);
        test_clearance(side);
        test_return(side);
        test_watches(side);
    }

    test_line();
    test_faults();
    printf("All %d Buddy 5 sequencer checks passed (left and right).\n",
           g_checks);
    return 0;
}
