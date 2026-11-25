/** @file line_follow_sim.c
 * @brief Closed-loop line-following regression (host only). * Real firmware:
 *        motion.c, steering.c, wheel_speed.c, downhill.c, ir_shared.c (1 ms
 *        latches), nav_core.c and line_follow.c, driven by a copy of the
 *        controller's live line-following step (task_controller.c).
 *        Simulated: motors (first-order lag fitted to motion.c's calibration:
 *        21-22 % duty for 40 RPM, 8 % deadband), encoders (1056 per rev), a
 *        70 mm-wheel differential drive with a 125 mm track, two IR spots 125
 *        mm ahead of the axle and 26 mm apart, over 18 mm tape. * Every trial
 *        runs in its own process (motion.c keeps static state) with its own
 *        start offset, heading, motor gains, motor lag, tape edge and
 *        controller phase. Build with -DMOTION_TARGET_RPM=<rpm>f
 *        -DSIM_RPM=<rpm>. * Measured with this harness before the steering
 *        change (KP 0.3, KI 2.0, decay 0.85, no lost-line search), 20 trials
 *        per course: 35 RPM: curves 60-100 %, 90 degree corner (R 30) 10 %,
 *        sharp 0 % 55 RPM: curves 10-70 %, corners 0 %. It also checks the
 *        zig-zag: the sensors' RMS distance off the line on the opening
 *        straight, once settled.
 */
#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include <tk/tkernel.h>
#include "modules/motion/motion.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/line_barcode/line_follow.h"
#include "app/nav_core.h"
#include "app/app_config.h"
#include "board/board_config.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"

#ifndef SIM_RPM
#define SIM_RPM 35
#endif

#define SIM_PI 3.14159265358979
#define TRIALS 8
#define LIMIT_SEC 45.0

/* ---------------- plant ---------------- */
static double p_track_mm = 125.0;
static double p_ahead_mm = 125.0;
static double p_gap_mm = 26.0;
static double p_tape_half = 9.0;
static double p_tau = 0.10;
static double const p_coast_tau = 0.15;
static double const p_brake_tau = 0.04;
static double p_gain[2] = {40.0 / (21.0 - 8.0), 40.0 / (22.0 - 8.0)};
static double const p_dead = 8.0;
static double const p_wheel_mm = 70.0;

/* ---------------- kernel and driver stubs ---------------- */
static uint32_t g_now_ms = 1000U;
static uint32_t g_now_us = 1000000U;
/** @brief Host stub of tm_printf(). */
INT tm_printf(UB const *format, ...) { (void)format; return 0; }
/** @brief Host stub of tk_cre_mtx(). */
ID tk_cre_mtx(const T_CMTX *cfg) { (void)cfg; return 42; }
/** @brief Host stub of tk_loc_mtx(). */
ER tk_loc_mtx(ID id, TMO t) { (void)id; (void)t; return E_OK; }
/** @brief Host stub of tk_unl_mtx(). */
ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
/** @brief Host stub of tk_get_otm(). */
ER tk_get_otm(SYSTIM *t) { t->hi = 0; t->lo = g_now_ms; return E_OK; }
/** @brief Host stub of bc_wave_capture_active(). */
uint8_t bc_wave_capture_active(void) { return 0U; }

static int32_t g_pwm[2];
static int g_mode[2]; /* 0 coast, 1 regulated, 2 brake, 3 directional */
static double g_rpm[2];
static double g_count_f[2];
static int32_t g_count[2];
static uint32_t g_edge_us[2];
static int g_pivot_periods;

/** @brief Host stub of motor_init(). */
void motor_init(void) { motor_stop(); }
/** @brief Host stub of motor_stop(). */
void motor_stop(void) { g_pwm[0] = g_pwm[1] = 0; g_mode[0] = g_mode[1] = 0; }
/** @brief Host stub of motor_brake(). */
void motor_brake(void) { g_pwm[0] = g_pwm[1] = 0; g_mode[0] = g_mode[1] = 2; }
/** @brief Host stub of motor_set_forward(). */
void motor_set_forward(uint32_t l, uint32_t r)
{ g_pwm[0] = (int32_t)l; g_pwm[1] = (int32_t)r; g_mode[0] = g_mode[1] = 1; }
/** @brief Host stub of motor_set_regulated(). */
void motor_set_regulated(int32_t l, int32_t r)
{ g_pwm[0] = l; g_pwm[1] = r; g_mode[0] = g_mode[1] = 1; }
/** @brief Host stub of motor_set_directional(). */
void motor_set_directional(int32_t l, int32_t r)
{ g_pwm[0] = l; g_pwm[1] = r; g_mode[0] = g_mode[1] = 3; g_pivot_periods++; }
/** @brief Host stub of motor_last_command(). */
void motor_last_command(uint16_t *l, uint16_t *r)
{ *l = (uint16_t)abs(g_pwm[0]); *r = (uint16_t)abs(g_pwm[1]); }
/** @brief Host stub of encoder_init(). */
ER encoder_init(void) { return E_OK; }
/** @brief Host stub of encoder_read(). */
void encoder_read(encoder_wheel_t w, encoder_sample_t *s)
{ s->count = g_count[w]; s->edge_usec = g_edge_us[w]; }
/** @brief Host stub of encoder_now_usec(). */
uint32_t encoder_now_usec(void) { return g_now_us; }

/* ---------------- track: 1 mm polyline ---------------- */
#define MAX_PTS 20000
static double tx[MAX_PTS], ty[MAX_PTS];
static int tn;
static double g_th;
static int g_hint;

/** @brief Test helper: track add. */
static void track_add(double heading)
{
    assert(tn < MAX_PTS);
    tx[tn] = tx[tn - 1] + cos(heading);
    ty[tn] = ty[tn - 1] + sin(heading);
    tn++;
}
/** @brief Test helper: track straight. */
static void track_straight(double len)
{
    for (double d = 1.0; d <= len; d += 1.0)
    {
        track_add(g_th);
    }
}
/* radius 0: a sharp vertex; + angle turns left */
static void track_arc(double radius, double angle_deg)
{
    double const ang = angle_deg * SIM_PI / 180.0;
    if (radius <= 0.5) { g_th += ang; return; }
    int const steps = (int)ceil(fabs(ang) * radius);
    for (int i = 0; i < steps; i++)
    {
        g_th += ang / steps;
        track_add(g_th - ang / steps / 2.0);
    }
}
/** @brief Test helper: track dist. */
static double track_dist(double x, double y, int *p_idx)
{
    int lo = g_hint - 600;
    int hi = g_hint + 600;
    if (lo < 0)
    {
        lo = 0;
    }
    if (hi > tn - 1)
    {
        hi = tn - 1;
    }
    double best = 1e18;
    int bi = lo;
    for (int i = lo; i <= hi; i++)
    {
        double const dx = tx[i] - x, dy = ty[i] - y;
        double const d = dx * dx + dy * dy;
        if (d < best) { best = d; bi = i; }
    }
    if (p_idx)
    {
        *p_idx = bi;
    }
    return sqrt(best);
}

/** @brief Host stub of course(). */
static void course(int which)
{
    tn = 1; tx[0] = 0; ty[0] = 0; g_th = 0.0;
    track_straight(400);
    switch (which)
    {
        case 0: track_arc(400, 60); track_arc(400, -120); track_arc(400, 60);
            break;
        case 1: track_arc(250, 90); track_straight(150); track_arc(250, -90);
            track_straight(150); track_arc(250, -90); track_arc(250, 90);
            break;
        case 2: track_arc(150, 90); track_straight(200); track_arc(150, -90);
            track_straight(200); track_arc(150, 180);
            break;
        case 3: track_arc(100, 90); track_straight(200); track_arc(100, -90);
            track_straight(200); track_arc(100, 90);
            break;
        case 4: track_arc(30, 90); track_straight(300); track_arc(30, -90);
            track_straight(300); track_arc(30, -90);
            break;
        default: track_arc(0, 90); track_straight(300); track_arc(0, -90);
            track_straight(300); track_arc(0, -90);
            break;
    }
    track_straight(400);
}

/* ---------------- robot ---------------- */
static double rx;
static double ry;
static double rth;
static nav_controller_t g_nav;
static uint32_t g_path_counts;
static int32_t g_last[2];
static int g_phase;

/** @brief Test helper: sensors. */
static void sensors(int *p_l, int *p_r)
{
    double const cx = rx + p_ahead_mm * cos(rth);
    double const cy = ry + p_ahead_mm * sin(rth);
    double const nx = -sin(rth), ny = cos(rth);
    *p_l = track_dist(cx + nx * p_gap_mm / 2, cy + ny * p_gap_mm / 2, NULL) <=
           p_tape_half;
    *p_r = track_dist(cx - nx * p_gap_mm / 2, cy - ny * p_gap_mm / 2, NULL) <=
           p_tape_half;
}

/** @brief Test helper: plant ms. */
static void plant_ms(void)
{
    double const dt = 0.001;
    for (int w = 0; w < 2; w++)
    {
        double ss = 0.0;
        double tau = p_coast_tau;
        if ((1 == g_mode[w]) || (3 == g_mode[w]))
        {
            double const duty = fabs((double)g_pwm[w]) * 100.0 / 12499.0;
            ss = (duty > p_dead) ? (duty - p_dead) * p_gain[w] : 0.0;
            /* Regulated negative is a brake; directional negative reverses. */
            if (g_pwm[w] < 0)
            {
                ss = (3 == g_mode[w]) ? -ss : 0.0;
            }
            tau = (fabs(ss) < fabs(g_rpm[w]) && duty <= p_dead) ? p_coast_tau
                                                                : p_tau;
        }
        else if (2 == g_mode[w])
        {
            tau = p_brake_tau;
        }
        g_rpm[w] += (ss - g_rpm[w]) * dt / tau;
        double const before = g_count_f[w];
        g_count_f[w] += g_rpm[w] / 60.0 * BOARD_ENC_COUNTS_PER_REV * dt;
        if (floor(g_count_f[w]) != floor(before))
        {
            g_count[w] = (int32_t)floor(g_count_f[w]);
            g_edge_us[w] = g_now_us;
        }
    }
    double const mm_per_rev = SIM_PI * p_wheel_mm;
    double const vl = g_rpm[0] / 60.0 * mm_per_rev;
    double const vr = g_rpm[1] / 60.0 * mm_per_rev;
    rth += (vr - vl) / p_track_mm * dt;
    rx += 0.5 * (vl + vr) * cos(rth) * dt;
    ry += 0.5 * (vl + vr) * sin(rth) * dt;
}

/** @brief Test helper: path counts. */
static uint32_t path_counts(void)
{
    for (int w = 0; w < 2; w++)
    {
        uint32_t const d = (uint32_t)g_count[w] - (uint32_t)g_last[w];
        g_last[w] = g_count[w];
        g_path_counts += (d > INT32_MAX) ? 0U - d : d;
    }
    return g_path_counts;
}

/* The live (B1_LIVE_COMMANDS) line-following step of task_controller.c. */
static void controller_period(void)
{
    ir_shared_snapshot_t reading = {0};
    bool b_junction = false;
    bool b_left = false;
    bool b_right = false;
    bool b_gap = false;
    (void)ir_shared_get_latest(&reading);
    (void)ir_shared_take_junction(&b_junction);
    (void)ir_shared_take_sides(&b_left, &b_right);
    if ((b_left != b_right) && (0U == reading.left) && (0U == reading.right))
    {
        reading.left = b_left ? 1U : 0U;
        reading.right = b_right ? 1U : 0U;
    }
    motion_line_seen(0U != reading.left, 0U != reading.right);
    if (ir_shared_take_both_gap(&b_gap) < E_OK)
    {
        b_gap = true;
    }
    nav_sample_t sample = {0};
    sample.both_interrupted = (uint8_t)b_gap;
    sample.now_msec = g_now_ms;
    sample.turn_counts = path_counts();
    sample.travel_counts = sample.turn_counts / 2U;
    sample.left_black = (uint8_t)(reading.left != 0U);
    sample.right_black = (uint8_t)(reading.right != 0U);
    nav_result_t const decision = nav_step(&g_nav, &sample);
    switch (decision.drive)
    {
        case NAV_OUTPUT_FOLLOW:
            if (reading.left && reading.right)
            {
                motion_apply(LINE_FORWARD);
            }
            else
            {
                motion_apply(line_follow_decide(reading.left, reading.right));
            }
            break;
        case NAV_OUTPUT_FORWARD: motion_apply(LINE_FORWARD); break;
        case NAV_OUTPUT_LEFT: motion_apply_maneuver(NAV_LEFT); break;
        case NAV_OUTPUT_RIGHT: motion_apply_maneuver(NAV_RIGHT); break;
        case NAV_OUTPUT_UTURN: motion_apply_maneuver(NAV_UTURN); break;
        default: motion_brake(); break;
    }
}

typedef struct
{
    int ok;
    double secs;
    double pivot_secs;
    double straight_mm; /* RMS of the sensors off the line, opening straight */
} result_t;
static double g_sq;
static long g_sn;

/** @brief Test helper: urand. */
static double urand(unsigned *seed)
{
    *seed = *seed * 1103515245u + 12345u;
    return ((*seed >> 8) & 0xFFFFu) / 65535.0;
}

/** @brief Host stub of run(). */
static result_t run(unsigned seed)
{
    unsigned sd = seed * 2654435761u + 1u;
    result_t res = {0, 0.0, 0.0, 0.0};
    ry = (urand(&sd) - 0.5) * 10.0;
    rth = (urand(&sd) - 0.5) * 6.0 * SIM_PI / 180.0;
    p_gain[0] *= 0.9 + 0.2 * urand(&sd);
    p_gain[1] *= 0.9 + 0.2 * urand(&sd);
    p_tau *= 0.8 + 0.4 * urand(&sd);
    p_tape_half += (urand(&sd) - 0.5) * 2.0;
    g_phase = (int)(urand(&sd) * APP_CONTROLLER_PERIOD_MSEC);
    assert(E_OK == motion_init());
    assert(E_OK == ir_shared_init());
    nav_config_t const cfg = {
        APP_NAV_JUNCTION_CONFIRM_MSEC, APP_NAV_MIN_TURN_MSEC,
        APP_NAV_MAX_TURN_MSEC, APP_NAV_MAX_CROSS_MSEC,
        APP_NAV_BARCODE_TO_JUNCTION_MIN_COUNTS, APP_NAV_JUNCTION_CONFIRM_COUNTS,
        APP_NAV_JUNCTION_CONFIRM_MAX_MSEC, APP_NAV_DECODE_WAIT_MSEC,
        APP_NAV_DECODE_GRACE_MSEC, APP_NAV_CENTRE_COUNTS,
        APP_NAV_CENTRE_MAX_MSEC, APP_NAV_TURN90_COUNTS,
        APP_NAV_REPEAT_ACTION, APP_NAV_STRAIGHT_DEFAULT,
        (APP_NAV_SCAN_FAIL_STOP != 0) ? 0U : 1U};
    nav_init(&g_nav, &cfg);
    nav_start(&g_nav, g_now_ms);
    motion_stop_now();
    double off_sec = 0.0;
    for (int ms = 0; ms < (int)(LIMIT_SEC * 1000.0); ms++)
    {
        int l;
        int r;
        sensors(&l, &r);
        (void)ir_shared_publish((uint32_t)l, (uint32_t)r, 0U);
        if (0 == ((ms + g_phase) % (int)APP_CONTROLLER_PERIOD_MSEC))
        {
            controller_period();
        }
        plant_ms();
        g_now_ms++;
        g_now_us += 1000U;
        int idx;
        double const dev = track_dist(rx + p_ahead_mm * cos(rth),
                                      ry + p_ahead_mm * sin(rth), &idx);
        if (dev < 80.0)
        {
            g_hint = idx;
        }
        if ((idx > 150) && (idx < 390))
        {
            /* Settled after the start, before the first bend: zig-zag. */
            g_sq += dev * dev;
            g_sn++;
        }
        res.secs = ms / 1000.0;
        /* Lost: far from the line for 3 s (a search may pivot off it for a
         * while), or wandered right away. */
        off_sec = (dev > 45.0) ? off_sec + 0.001 : 0.0;
        if ((off_sec > 3.0) || (dev > 250.0))
        {
            break;
        }
        if (g_hint >= tn - 30) { res.ok = 1; break; }
    }
    res.pivot_secs = g_pivot_periods * (APP_CONTROLLER_PERIOD_MSEC / 1000.0);
    res.straight_mm = g_sn ? sqrt(g_sq / (double)g_sn) : 0.0;
    return res;
}

/* One trial in its own process: motion.c's state starts fresh each time. */
static result_t trial(int which, unsigned seed)
{
    int fd[2];
    result_t r = {0, 0.0, 0.0, 0.0};
    assert(0 == pipe(fd));
    pid_t const pid = fork();
    assert(pid >= 0);
    if (0 == pid)
    {
        close(fd[0]);
        course(which);
        r = run(seed);
        _exit(write(fd[1], &r, sizeof r) == (ssize_t)sizeof r ? 0 : 2);
    }
    close(fd[1]);
    if (read(fd[0], &r, sizeof r) != (ssize_t)sizeof r)
    {
        r.ok = 0;
    }
    close(fd[0]);
    (void)waitpid(pid, NULL, 0);
    return r;
}

/** @brief Run every test case; any failed assert aborts. */
int main(void)
{
    static char const *const names[] = {
        "S-bend R400", "S-bends R250", "curves R150", "curves R100",
        "90 deg corners R30", "sharp 90 deg corners"};
    /* Least runs out of TRIALS that must reach the end, per course. Set a
     * little under what the harness measures, so a small model change does
     * not fail it but a real regression does. */
#if SIM_RPM <= 40
    static int const need[6] = {8, 8, 8, 8, 7, 5};
    /* Zig-zag on a straight, mm RMS: 3.2 with the observer (the PI
     * steering 6.4, the original 4.5; the original at 15 RPM 3.0). */
    double const wander_max = 4.5;
#else
    static int const need[6] = {8, 8, 8, 8, 6, 4};
    double const wander_max = 6.0; /* 4.4 measured; PI steering 8.5 */
#endif
    double wander = 0.0;
    printf("Line following at %d RPM (closed-loop model, %d trials each)\n",
           SIM_RPM, TRIALS);
    for (int c = 0; c < 6; c++)
    {
        int ok = 0;
        double secs = 0.0;
        double pivot = 0.0;
        for (int t = 0; t < TRIALS; t++)
        {
            result_t const r = trial(c, (unsigned)(c * 1000 + t));
            ok += r.ok;
            if (r.ok)
            {
                secs += r.secs;
            }
            pivot += r.pivot_secs;
            if (c < 4)
            {
                wander += r.straight_mm / (4.0 * TRIALS);
            }
        }
        printf("  %-22s %d/%d reached the end, %.1f s each, %.2f s "
               "searching\n", names[c], ok, TRIALS, ok ? secs / ok : 0.0,
               pivot / TRIALS);
        assert(ok >= need[c]);
        if (c < 2)
        {
            /* Gentle curves: with the PID observer the line stays centred,
             * so after a long stretch with neither sensor touching the
             * search may make a short check pivot; the line is right there
             * between the sensors (measured 0.3-0.6 s a run). */
            assert(pivot / TRIALS <= 1.0);
        }
    }
    printf("  straight-line wander %.1f mm RMS (limit %.1f)\n", wander,
           wander_max);
    assert(wander <= wander_max);
    printf("PASS line following holds curves and corners at %d RPM\n",
           SIM_RPM);
    return 0;
}
/*** end of file ***/

