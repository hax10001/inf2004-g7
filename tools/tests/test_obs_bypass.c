/** @file test_obs_bypass.c
 * @brief Host test of the real obstacle.c bypass on a simulated robot,
 *        with the real scan_plan.c planner and obs_state.c word: the
 *        reported object width with its OBS_WIDTH_REPORT_ADD_MM offset
 *        (FR10 scan report), the OBS_RETURN_DEG rejoin turn on both
 *        bypass sides (FR11 rejoin) and mid-bypass rescans (FR10/FR11).
 */
#include <assert.h>
#include <math.h>
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
#define PERIOD_S 0.05
/* Scan bearings in the firmware's sequence fit easily. */
#define SCAN_MAX 64
/* Robot axle at the stop is the origin; the sonar sits ahead of it. */
#define STOP_FACE_MM 250.0
/* Line sensor geometry for the simulated tape (same as the rejoin sim). */
#define TAPE_HALF_MM 9.0
#define SENSOR_HALF_GAP_MM 13.0
#define DEG_PER_RAD 57.29578
/* A near front echo that trips the guard on the way out. */
#define GUARD_NEAR_MM 120

/** @brief Simulated robot: true pose and encoder revolutions. */
typedef struct
{
    double x_mm;
    double y_mm;
    double heading_rad;
    double left_rev;
    double right_rev;
} sim_robot_t;

/** @brief A box on the line, in the obstacle frame. */
typedef struct
{
    double face_x_mm;
    double depth_mm;
    double centre_y_mm;
    double width_mm;
} sim_box_t;

static sim_robot_t g_robot;
static sim_box_t   g_box;
static UINT        g_pattern = 0U;
static uint32_t    g_now     = 1000U;

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

/** @brief Encoders from the simulated wheels. */
ER motion_get_status(motion_status_t * p_output)
{
    motion_status_t const zero = {0};

    *p_output             = zero;
    p_output->left_count  = (int32_t)(g_robot.left_rev *
                                      (double)BOARD_ENC_COUNTS_PER_REV);
    p_output->right_count = (int32_t)(g_robot.right_rev *
                                      (double)BOARD_ENC_COUNTS_PER_REV);
    p_output->b_valid     = 1U;
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

/** @brief Distance along a ray to the box, or -1 if it misses it. */
static double ray_box(double x0, double y0, double angle_rad)
{
    double const dx    = cos(angle_rad);
    double const dy    = sin(angle_rad);
    double       t_min = -1.0e9;
    double       t_max = 1.0e9;
    double       lo    = 0.0;
    double       hi    = 0.0;
    double       range = -1.0;

    lo = (g_box.face_x_mm - x0);
    hi = (g_box.face_x_mm + g_box.depth_mm - x0);

    if (fabs(dx) < 1.0e-9)
    {
        if ((lo > 0.0) || (hi < 0.0))
        {
            t_max = -1.0;
        }
    }
    else
    {
        double const a = lo / dx;
        double const b = hi / dx;

        t_min = fmax(t_min, fmin(a, b));
        t_max = fmin(t_max, fmax(a, b));
    }

    lo = (g_box.centre_y_mm - (g_box.width_mm / 2.0)) - y0;
    hi = (g_box.centre_y_mm + (g_box.width_mm / 2.0)) - y0;

    if (fabs(dy) < 1.0e-9)
    {
        if ((lo > 0.0) || (hi < 0.0))
        {
            t_max = -1.0;
        }
    }
    else
    {
        double const a = lo / dy;
        double const b = hi / dy;

        t_min = fmax(t_min, fmin(a, b));
        t_max = fmin(t_max, fmax(a, b));
    }

    if ((t_max >= t_min) && (t_min > 0.0))
    {
        range = t_min;
    }

    return range;
}

/** @brief One sonar ping from the current pose at a servo angle: the
 *  nearest echo inside a beam of +-OBS_BEAM_HALF_DEG, -1 if none.
 */
static int32_t sonar(int32_t servo_deg, double max_mm)
{
    double const h       = g_robot.heading_rad;
    double const sx      = g_robot.x_mm + (OBS_SENSOR_TO_AXLE_MM * cos(h));
    double const sy      = g_robot.y_mm + (OBS_SENSOR_TO_AXLE_MM * sin(h));
    double       nearest = -1.0;
    int32_t      offset  = 0;

    for (offset = -OBS_BEAM_HALF_DEG; offset <= OBS_BEAM_HALF_DEG; offset++)
    {
        double const range =
            ray_box(sx, sy, h + ((double)(servo_deg + offset) / DEG_PER_RAD));

        if ((range >= 0.0) && ((nearest < 0.0) || (range < nearest)))
        {
            nearest = range;
        }
    }

    return ((nearest < 0.0) || (nearest > max_mm)) ? -1 : (int32_t)nearest;
}

/** @brief Real planner on a ray-cast scan of the box from the stop. */
static void plan_scan(scan_plan_bypass_t * p_plan, scan_plan_sides_t * p_sides)
{
    scan_plan_point_t points[SCAN_MAX];
    int32_t           count = 0;
    int32_t           angle = 0;

    for (angle = -OBS_SCAN_RIGHT_DEG; angle <= OBS_SCAN_LEFT_DEG;
         angle = scan_plan_next_angle(angle))
    {
        int32_t const range = sonar(angle, (double)OBS_SCAN_RANGE_MM);

        assert(count < SCAN_MAX);
        points[count].angle_deg = angle;
        points[count].range_mm  = (uint32_t)((range < 0) ? OBS_FAR_MM : range);
        points[count].lone_mm   = 0U;
        count++;
    }

    scan_plan_build_side_report(points, count, (int32_t)STOP_FACE_MM, 0,
                                p_plan, p_sides);
}

/** @brief Line sensors over the tape along y = 0. */
static void line_sensors(uint32_t * p_left, uint32_t * p_right)
{
    double const h  = g_robot.heading_rad;
    double const fy = g_robot.y_mm + (OBS_IR_TO_AXLE_MM * sin(h));
    double const ly = fy + (SENSOR_HALF_GAP_MM * cos(h));
    double const ry = fy - (SENSOR_HALF_GAP_MM * cos(h));

    *p_left  = (fabs(ly) <= TAPE_HALF_MM) ? 1U : 0U;
    *p_right = (fabs(ry) <= TAPE_HALF_MM) ? 1U : 0U;
}

/** @brief Move the simulated robot by one period of wheel commands. */
static void drive(int32_t left_rpm, int32_t right_rpm)
{
    double const circ  = BOARD_WHEEL_DIAMETER_MM * 3.14159265;
    double const l_rev = ((double)left_rpm / 60.0) * PERIOD_S;
    double const r_rev = ((double)right_rpm / 60.0) * PERIOD_S;
    double const l_mm  = l_rev * circ;
    double const r_mm  = r_rev * circ;
    double const turn  = (r_mm - l_mm) / OBS_TRACK_WIDTH_MM;
    double const mid   = g_robot.heading_rad + (turn / 2.0);

    g_robot.x_mm        += ((l_mm + r_mm) / 2.0) * cos(mid);
    g_robot.y_mm        += ((l_mm + r_mm) / 2.0) * sin(mid);
    g_robot.heading_rad += turn;
    g_robot.left_rev    += l_rev;
    g_robot.right_rev   += r_rev;
}

/** @brief One period: side watch ping, sequencer step, then motion. */
static obstacle_drive_t period(void)
{
    obstacle_drive_t out   = {0};
    uint32_t         left  = 0U;
    uint32_t         right = 0U;

    if (0U != (g_pattern & OBS_EVT_SIDE_WATCH))
    {
        obs_state_side_range_set(
            sonar(obs_state_watch_angle_get(), (double)OBS_FAR_MM));
    }

    line_sensors(&left, &right);
    g_now += PERIOD_MSEC;
    out = obstacle_bypass_step(left, right, g_now);

    if (out.b_line_follow)
    {
        /* Simple follower: steer off the sensor that sees the tape. */
        drive((0U != left) ? 12 : 25, (0U != right) ? 12 : 25);
    }
    else if (!out.b_stop)
    {
        drive(out.left_rpm, out.right_rpm);
    }
    else
    {
        /* Stopped. */
    }

    return out;
}

/** @brief Phase published in the status word. */
static uint32_t phase_now(void)
{
    return (uint32_t)obs_state_status_phase(obs_state_status_get());
}

/** @brief Hand the scanner's plan over, as task_obstacle does. */
static obstacle_drive_t scan_done(scan_plan_bypass_t const * p_plan)
{
    obs_state_plan_store(p_plan);
    obs_state_flags_clear(OBS_EVT_SCAN_REQ);
    obs_state_flags_set(OBS_EVT_SCAN_DONE);
    return period();
}

/** @brief New obstacle at the stop: settle, scan, front recheck finds it
 *  still there (FR10), so the plan is used.
 */
static scan_plan_bypass_t start_bypass(sim_box_t const * p_box,
                                       scan_plan_sides_t * p_sides)
{
    scan_plan_bypass_t plan = {0U, 0U};
    sim_robot_t const  home = {0.0, 0.0, 0.0, 0.0, 0.0};

    g_robot        = home;
    g_box          = *p_box;
    g_seq.b_active = false;
    g_pattern      = 0U;
    assert(E_OK == obs_state_init());
    (void)period();
    g_now += OBS_STOP_SETTLE_MSEC;
    (void)period();
    assert(OBS_PHASE_SCANNING == phase_now());
    plan_scan(&plan, p_sides);
    assert(SCAN_PLAN_ROUTE == scan_plan_result(&plan));
    (void)scan_done(&plan);
    obs_state_front_range_set(sonar(0, (double)OBS_FAR_MM));
    (void)period();
    assert(OBS_PHASE_TURN_OUT == phase_now());
    return plan;
}

/** @brief Run periods until the published phase is reached. */
static obstacle_drive_t run_until(uint32_t phase, uint32_t max_periods)
{
    obstacle_drive_t out = {0};
    uint32_t         n   = 0U;

    for (n = 0U; (n < max_periods) && (phase_now() != phase); n++)
    {
        out = period();
        assert(!out.b_fault);
    }

    assert(phase_now() == phase);
    return out;
}

/** @brief Reported width (FR10 scan report): telemetry carries the
 *  measured width plus OBS_WIDTH_REPORT_ADD_MM, rounded to whole cm,
 *  for real scans of boxes of many widths and offsets.
 */
static void test_width_report(void)
{
    uint32_t cases      = 0U;
    double   width      = 0.0;
    double   centre     = 0.0;

    for (width = 60.0; width <= 260.0; width += 7.0)
    {
        for (centre = -40.0; centre <= 40.0; centre += 20.0)
        {
            sim_box_t const   box   = {STOP_FACE_MM + OBS_SENSOR_TO_AXLE_MM,
                                       150.0, centre, width};
            scan_plan_sides_t sides = {0};
            scan_plan_bypass_t plan = {0U, 0U};
            int32_t           plan_cm = 0;
            uint32_t          sent_cm = 0U;
            uint32_t          want_cm = 0U;

            if (fabs(centre) >= (width / 2.0))
            {
                /* Off the line: it would not have stopped the robot. */
                continue;
            }

            plan    = start_bypass(&box, &sides);
            plan_cm = scan_plan_width_cm(&plan);
            sent_cm = obs_state_status_width_cm(obs_state_status_get());
            assert(0U != sides.b_object);
            /* The log line's width already carries the offset (mm). */
            want_cm = (sides.width_mm + 5U) / 10U;
            assert(plan_cm > 0);
            /* The offset is added, never dropped or subtracted. */
            assert(sent_cm > (uint32_t)plan_cm);
            assert(sent_cm <= OBS_WIDTH_INVALID_CM);

            /* Telemetry adds the offset rounded to whole cm to the plan's
             * cm width; the log line rounds measured mm + offset once, so
             * the two agree to within 1 cm. */
            assert(sent_cm == ((uint32_t)plan_cm +
                               ((OBS_WIDTH_REPORT_ADD_MM + 5U) / 10U)));
            assert((sent_cm == want_cm) || ((sent_cm + 1U) == want_cm));

            cases++;
        }
    }

    printf("PASS width report = measured + %d mm offset (within 1 cm) on "
           "%u real scans\n", (int)OBS_WIDTH_REPORT_ADD_MM, (unsigned)cases);
}

/** @brief Plan widths at the edges: 0 (no width) stays 0 and the
 *  invalid code stays invalid; the offset never turns "no width" into a
 *  plausible small width.
 */
static void test_width_edges(void)
{
    static uint32_t const plan_cm[] = {0U, 1U, 15U, 250U, 255U};
    uint32_t              index     = 0U;
    scan_plan_sides_t     sides     = {0};
    sim_box_t const       box       = {STOP_FACE_MM + OBS_SENSOR_TO_AXLE_MM,
                                       150.0, 30.0, 150.0};

    for (index = 0U; index < (sizeof(plan_cm) / sizeof(plan_cm[0]));
         index++)
    {
        scan_plan_bypass_t plan = {0U, 0U};
        uint32_t           sent = 0U;
        uint32_t           want = 0U;

        g_robot.x_mm = 0.0;
        (void)start_bypass(&box, &sides);
        /* Same route, width field replaced (bits 5..12 of word A). */
        g_seq.b_active = false;
        g_pattern      = 0U;
        (void)period();
        g_now += OBS_STOP_SETTLE_MSEC;
        (void)period();
        plan_scan(&plan, &sides);
        plan.word_a = (plan.word_a & ~(WIDTH_MASK << WIDTH_SHIFT)) |
                      (plan_cm[index] << WIDTH_SHIFT);
        assert((uint32_t)scan_plan_width_cm(&plan) == plan_cm[index]);
        (void)scan_done(&plan);
        obs_state_front_range_set(GUARD_NEAR_MM);
        (void)period();
        assert(OBS_PHASE_TURN_OUT == phase_now());
        sent = obs_state_status_width_cm(obs_state_status_get());
        want = (0U == plan_cm[index]) ? 0U
               : (plan_cm[index] >= 251U)
                   ? OBS_WIDTH_INVALID_CM
                   : (((plan_cm[index] * 10U) + OBS_WIDTH_REPORT_ADD_MM +
                       5U) / 10U);

        assert(sent == want);
    }

    puts("PASS width 0 stays 0, invalid stays invalid, small widths get "
         "the offset");
}

/** @brief FR11 rejoin: after passing the box the return leg turns back
 *  towards the line by OBS_RETURN_DEG; checked on the simulated robot for
 *  a left and a right bypass (mirror images).
 */
static void test_return_angle(void)
{
    double   turned[2] = {0.0, 0.0};
    int32_t  sides[2]  = {0, 0};
    uint32_t run       = 0U;

    for (run = 0U; run < 2U; run++)
    {
        /* Box offset right -> left bypass, offset left -> right bypass. */
        double const      offset = (0U == run) ? -30.0 : 30.0;
        sim_box_t const   box    = {STOP_FACE_MM + OBS_SENSOR_TO_AXLE_MM,
                                    150.0, offset, 150.0};
        scan_plan_sides_t info   = {0};
        obstacle_drive_t  out    = {0};
        int32_t           side   = 0;
        double            begin  = 0.0;
        double            end    = 0.0;
        int32_t           target = 0;

        (void)start_bypass(&box, &info);
        side = g_seq.side;
        out  = run_until(OBS_PHASE_TURN_BACK, 2000U);
        target = -side * OBS_RETURN_DEG * DDEG_PER_DEG;
        assert(g_seq.turn_target_ddeg == target);
        assert(g_seq.turn_dir == -side);
        begin = g_robot.heading_rad * DEG_PER_RAD;
        /* Parallel to the line when the return starts. */
        assert(fabs(begin) < 10.0);
        /* Pivot about the wheel nearer the line, towards it. */
        assert((side > 0) ? ((OBS_DRIVE_RPM == out.left_rpm) &&
                             (0 == out.right_rpm))
                          : ((0 == out.left_rpm) &&
                             (OBS_DRIVE_RPM == out.right_rpm)));
        assert(((double)side * g_robot.y_mm) > OBS_HALF_PATH_MM);
        out = run_until(OBS_PHASE_SEARCH_LINE, 400U);
        end = g_robot.heading_rad * DEG_PER_RAD;
        assert(g_seq.search_ddeg == target);
        turned[run] = (end - begin) * (double)(-side);
        sides[run]  = side;
        /* Stopped OBS_TURN_LEAD_DDEG early, within one period's turn. */
        assert(turned[run] >= (OBS_RETURN_DEG -
                               (OBS_TURN_LEAD_DDEG / DDEG_PER_DEG) - 0.5));
        assert(turned[run] <= (double)OBS_RETURN_DEG);
        /* The run to the line holds the return heading: it keeps
         * steering towards OBS_RETURN_DEG, never back to parallel. */
        out = period();
        assert(OBS_PHASE_SEARCH_LINE == phase_now());
        assert((side > 0) ? (out.left_rpm >= out.right_rpm)
                          : (out.right_rpm >= out.left_rpm));
        assert(fabs((g_robot.heading_rad * DEG_PER_RAD) - end) < 1.0);
        printf("PASS FR11 %s bypass returns at %d deg (turned %.1f deg, "
               "lead %d ddeg)\n", (side > 0) ? "left" : "right",
               (int)OBS_RETURN_DEG, turned[run], (int)OBS_TURN_LEAD_DDEG);

        /* Carry on to the hand-back: the rejoin completes on the line. */
        for (target = 0; (target < 1500) && !out.b_done; target++)
        {
            out = period();
            assert(!out.b_fault);
        }

        assert(out.b_done);
        assert(1U == obs_state_status_count(obs_state_status_get()));
        assert(fabs(g_robot.y_mm) < 30.0);
        assert(fabs(g_robot.heading_rad * DEG_PER_RAD) < 30.0);
    }

    assert((sides[0] == 1) && (sides[1] == -1));
    assert(fabs(turned[0] - turned[1]) < 3.0);
    puts("PASS FR11 left and right returns mirror the same angle and both "
         "rejoin the line");
}

/** @brief Mid-bypass stop for something ahead: the rescan keeps the
 *  side and never runs the FR10 resume check, so a clear rescan carries
 *  on with the bypass instead of line following off the line.
 */
static void test_mid_bypass_rescan(void)
{
    sim_box_t const    box   = {STOP_FACE_MM + OBS_SENSOR_TO_AXLE_MM,
                                150.0, -30.0, 150.0};
    scan_plan_sides_t  info  = {0};
    scan_plan_bypass_t clear = {SCAN_PLAN_CLEAR, 0U};
    scan_plan_bypass_t other = {0U, 0U};
    obstacle_drive_t   out   = {0};
    uint32_t           route = 0U;

    other = start_bypass(&box, &info);
    route = obs_state_status_route(obs_state_status_get());
    assert(OBS_ROUTE_LEFT == route);
    (void)run_until(OBS_PHASE_DIAG_OUT, 400U);
    obs_state_front_range_set(GUARD_NEAR_MM);
    (void)period();
    obs_state_front_range_set(GUARD_NEAR_MM);
    out = period();
    assert(out.b_stop && (OBS_PHASE_STOPPING == phase_now()));
    g_now += OBS_STOP_SETTLE_MSEC;
    (void)period();
    assert(OBS_PHASE_SCANNING == phase_now());
    out = scan_done(&clear);
    assert(!out.b_done && (0U == g_seq.b_rechecking));
    assert(OBS_PHASE_DIAG_OUT == phase_now());
    puts("PASS FR10/FR11 clear rescan mid-bypass carries on, no resume "
         "off the line");

    obs_state_front_range_set(GUARD_NEAR_MM);
    (void)period();
    obs_state_front_range_set(GUARD_NEAR_MM);
    (void)period();
    assert(OBS_PHASE_STOPPING == phase_now());
    g_now += OBS_STOP_SETTLE_MSEC;
    (void)period();
    /* A plan for the other side: the bypass must keep its side. */
    other.word_a &= ~(SIDE_MASK << SIDE_SHIFT);
    assert(!scan_plan_side_is_left(&other));
    out = scan_done(&other);
    assert(!out.b_done && (0U == g_seq.b_rechecking));
    assert(OBS_PHASE_TURN_OUT == phase_now());
    assert(route == obs_state_status_route(obs_state_status_get()));
    assert(1 == g_seq.side);
    puts("PASS FR11 mid-bypass rescan keeps the bypass side, no recheck");
}

/** @brief Run the bypass width and return-angle tests. */
int main(void)
{
    obstacle_set_line_offset_ddeg(OBSTACLE_LINE_OFFSET_UNKNOWN);
    test_width_report();
    test_width_edges();
    test_return_angle();
    test_mid_bypass_rescan();
    return 0;
}

/*** end of file ***/

