/* Host-only closed-loop simulation of the Buddy 5 bypass.
 *
 * The production obstacle task, bypass sequencer, scan planner, geometry,
 * shared state and servo driver run unchanged. This file supplies a world
 * of boxes and black stripes, a differential-drive car with encoders, an
 * HC-SR04 beam model, an SG90 that moves at a finite rate, and a stand-in
 * for the controller task. The two tasks run on a deterministic
 * cooperative scheduler driven by a simulated clock; tk_dly_tsk() advances
 * it. Results are a host simulation only, never a hardware qualification.
 *
 * Usage: buddy5_sim <scenario> [mirror] [verbose]
 */
#define _DEFAULT_SOURCE
#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board/board_config.h"
#include "app/tasks.h"
#include "modules/motion/motion.h"
#include "modules/obstacle/obs_config.h"
#include "modules/obstacle/obs_state.h"
#include "modules/obstacle/obstacle.h"
#include "modules/obstacle/servo.h"
#include "modules/obstacle/task_obstacle.h"
#include "modules/obstacle/ultrasonic.h"

#ifndef OBS_SERVO_ARM_MM
#define OBS_SERVO_ARM_MM 0
#endif
#ifndef OBS_SENSOR_LATERAL_MM
#define OBS_SENSOR_LATERAL_MM 0
#endif
#ifndef OBS_SIDE_NO_ECHO_LIMIT
#define OBS_SIDE_NO_ECHO_LIMIT 5U
#endif

#define PI 3.14159265358979
#define MAX_RECTS 8
#define MAX_STRIPES 8
#define CTRL_MS 50U
#define FOLLOW_RPM 30.0
#define CORRECT_SLOW_RPM 12.0
#define IR_HALF_SPACING 14.0
#define LINE_HALF_WIDTH 9.0
#define SPEC_MAX_DEG 40.0
#define CORNER_DIFFUSE_MM 20.0
#define SIM_LIMIT_MS 90000U
/* Half the HC-SR04 beam width: the beam is about 25 degrees wide. */
#define BEAM_HALF_DEG 12

typedef struct
{
    double x0, x1, y0, y1;
    double quiet_x0, quiet_x1; /* Part of the side faces that never echo. */
} rect_t;

typedef struct
{
    double x0, x1, y0, y1;
} stripe_t;

typedef struct
{
    const char *name;
    rect_t      rects[MAX_RECTS];
    int         nrect;
    stripe_t    stripes[MAX_STRIPES];
    int         nstripe;
    int         expect_side;    /* 1 left, -1 right, 0 either. */
    unsigned    expect_fault;   /* 0 for a completed bypass. */
    int         expect_bypasses;
    double      min_clear_mm;   /* Body clearance required throughout. */
    double      true_rear_x;    /* Rear of the obstacle to pass, or 0. */
    double      prompt_mm;      /* Most travel past the rear allowed. */
    double      servo_deg_per_ms;
    int         servo_stuck;
    int         side_none_from_ms; /* Into the pass, side pings are lost: */
    int         side_none_count;   /* this many in a row. */
    int         dead_after_ms;  /* Every ping lost from then on. */
    int         dead_in_pass_ms; /* Every ping lost this long into it. */
    int         spike_every;    /* Every Nth ping is a stray value. */
    int         expect_marks;   /* The IR must cross black off the line. */
    int         expect_inject;  /* The injected fault must really occur. */
    double      bias_y;         /* Shift of the whole world to the left. */
} scenario_t;

/* ---------- Simulated world and car ---------- */

static scenario_t g_sc;
static double     g_x, g_y, g_th;          /* Car pose: axle midpoint. */
static double     g_dist_l, g_dist_r;       /* Wheel travel in mm. */
static double     g_cmd_l, g_cmd_r;         /* Wheel commands in rpm. */
static double     g_servo_deg;              /* Physical bearing. */
static double     g_servo_target;
static double     g_min_clear = 1e9;
static int        g_track_clear;
static int        g_collided;
static uint32_t   g_now;
static int        g_mirror;
static int        g_verbose;
static uint32_t   g_ping_count;
static uint32_t   g_misaimed;    /* Pings taken with the servo still off. */
static uint32_t   g_marks_seen;  /* Periods with black away from the line. */
static uint32_t   g_injected;    /* Pings lost or silenced by the scenario. */
static uint32_t   g_pass_ms;     /* When the first pass alongside began. */
static int        g_ir_latch_l, g_ir_latch_r;
static uint32_t   g_lcg = 12345U;

static double rect_clearance (rect_t const *p_rect);

static double now_s (void)
{
    return g_now / 1000.0;
}

static double rpm_to_mm_per_ms (double rpm)
{
    return (rpm / 60.0) * PI * BOARD_WHEEL_DIAMETER_MM / 1000.0;
}

static int point_black (double px, double py)
{
    int black = (py >= -LINE_HALF_WIDTH) && (py <= LINE_HALF_WIDTH);

    for (int i = 0; i < g_sc.nstripe; i++)
    {
        stripe_t const *s = &g_sc.stripes[i];

        if ((px >= s->x0) && (px <= s->x1) && (py >= s->y0) && (py <= s->y1))
        {
            black = 1;
        }
    }

    return black;
}

static void ir_read (int *p_left, int *p_right)
{
    double const c = cos(g_th), s = sin(g_th);
    double const fx = g_x + (OBS_IR_TO_AXLE_MM * c);
    double const fy = g_y + (OBS_IR_TO_AXLE_MM * s);

    *p_left  = point_black(fx - (IR_HALF_SPACING * s), fy + (IR_HALF_SPACING * c));
    *p_right = point_black(fx + (IR_HALF_SPACING * s), fy - (IR_HALF_SPACING * c));
}

static void physics_step (void)
{
    double const vl = rpm_to_mm_per_ms(g_cmd_l);
    double const vr = rpm_to_mm_per_ms(g_cmd_r);
    double const v  = (vl + vr) / 2.0;
    double const w  = (vr - vl) / OBS_TRACK_WIDTH_MM;
    double const mid = g_th + (w / 2.0);
    double       target = g_servo_target;
    int          l = 0, r = 0;

    g_x += v * cos(mid);
    g_y += v * sin(mid);
    g_th += w;
    g_dist_l += vl;
    g_dist_r += vr;

    if (!g_sc.servo_stuck)
    {
        double const step = g_sc.servo_deg_per_ms;

        if (fabs(target - g_servo_deg) <= step)
        {
            g_servo_deg = target;
        }
        else
        {
            g_servo_deg += (target > g_servo_deg) ? step : -step;
        }
    }

    ir_read(&l, &r);
    g_ir_latch_l |= l;
    g_ir_latch_r |= r;

    if (g_track_clear && ((g_now % 5U) == 0U))
    {
        for (int i = 0; i < g_sc.nrect; i++)
        {
            double const clear = rect_clearance(&g_sc.rects[i]);

            if (clear < g_min_clear)
            {
                g_min_clear = clear;
            }

            if (clear <= 0.0)
            {
                g_collided = 1;
            }
        }
    }
}

/* ---------- Body clearance ---------- */

static double point_rect_distance (double px, double py, rect_t const *p)
{
    double const dx = (px < p->x0) ? (p->x0 - px) : ((px > p->x1) ? (px - p->x1) : 0.0);
    double const dy = (py < p->y0) ? (p->y0 - py) : ((py > p->y1) ? (py - p->y1) : 0.0);

    if ((dx == 0.0) && (dy == 0.0))
    {
        return -1.0;
    }

    return sqrt((dx * dx) + (dy * dy));
}

static double rect_clearance (rect_t const *p_rect)
{
    double const c = cos(g_th), s = sin(g_th);
    double const half = OBS_ROBOT_WIDTH_MM / 2.0;
    double best = 1e9;

    /* The body perimeter, sampled every 5 mm. */
    for (double f = -OBS_ROBOT_REAR_MM; f <= OBS_ROBOT_FRONT_MM; f += 5.0)
    {
        for (int side = -1; side <= 1; side += 2)
        {
            double const px = g_x + (f * c) - (side * half * s);
            double const py = g_y + (f * s) + (side * half * c);
            double const d  = point_rect_distance(px, py, p_rect);

            best = (d < best) ? d : best;
        }
    }

    for (double l = -half; l <= half; l += 5.0)
    {
        for (int end = 0; end < 2; end++)
        {
            double const f = end ? OBS_ROBOT_FRONT_MM : -OBS_ROBOT_REAR_MM;
            double const px = g_x + (f * c) - (l * s);
            double const py = g_y + (f * s) + (l * c);
            double const d  = point_rect_distance(px, py, p_rect);

            best = (d < best) ? d : best;
        }
    }

    /* A box corner inside the body. */
    double const cx[4] = {p_rect->x0, p_rect->x1, p_rect->x0, p_rect->x1};
    double const cy[4] = {p_rect->y0, p_rect->y0, p_rect->y1, p_rect->y1};

    for (int k = 0; k < 4; k++)
    {
        double const rx = cx[k] - g_x, ry = cy[k] - g_y;
        double const along = (rx * c) + (ry * s);
        double const across = (ry * c) - (rx * s);

        if ((along >= -OBS_ROBOT_REAR_MM) && (along <= OBS_ROBOT_FRONT_MM) &&
            (across >= -half) && (across <= half))
        {
            best = -1.0;
        }
    }

    return best;
}

/* ---------- Ultrasonic beam model ---------- */

static int g_ray_quiet;

static int ray_hit (double ox, double oy, double ang, double *p_dist)
{
    double const dx = cos(ang), dy = sin(ang);
    double best = 1e9;
    int found = 0;
    int quiet_best = 0;

    for (int i = 0; i < g_sc.nrect; i++)
    {
        rect_t const *r = &g_sc.rects[i];
        /* The four faces: x0 (normal -x), x1 (+x), y0 (-y), y1 (+y). */
        double const fx[2] = {r->x0, r->x1};
        double const fy[2] = {r->y0, r->y1};

        for (int k = 0; k < 2; k++)
        {
            if (fabs(dx) > 1e-9)
            {
                double const t = (fx[k] - ox) / dx;
                double const y = oy + (t * dy);

                if ((t > 0.0) && (t < best) && (y >= r->y0) && (y <= r->y1))
                {
                    double const nx = (k == 0) ? -1.0 : 1.0;
                    double const inc = acos(fabs(dx * nx)) * 180.0 / PI;
                    int const corner = (fabs(y - r->y0) < CORNER_DIFFUSE_MM) ||
                                       (fabs(y - r->y1) < CORNER_DIFFUSE_MM);

                    best = t;
                    found = (inc <= SPEC_MAX_DEG) || corner;
                    quiet_best = 0;
                }
            }

            if (fabs(dy) > 1e-9)
            {
                double const t = (fy[k] - oy) / dy;
                double const x = ox + (t * dx);

                if ((t > 0.0) && (t < best) && (x >= r->x0) && (x <= r->x1))
                {
                    double const ny = (k == 0) ? -1.0 : 1.0;
                    double const inc = acos(fabs(dy * ny)) * 180.0 / PI;
                    int const corner = (fabs(x - r->x0) < CORNER_DIFFUSE_MM) ||
                                       (fabs(x - r->x1) < CORNER_DIFFUSE_MM);
                    int const quiet = (x >= r->quiet_x0) && (x <= r->quiet_x1);

                    best = t;
                    found = ((inc <= SPEC_MAX_DEG) || corner) && !quiet;
                    quiet_best = quiet;
                }
            }
        }
    }

    *p_dist = best;
    g_ray_quiet |= quiet_best;
    return found;
}

static int beam_range (double *p_range)
{
    double const bearing = g_servo_deg * PI / 180.0;
    double const c = cos(g_th), s = sin(g_th);
    /* The true mount: the axis behind the face by the arm, rotated. */
    double const axis_f = OBS_SENSOR_TO_AXLE_MM - OBS_SERVO_ARM_MM;
    double const of = axis_f + (OBS_SERVO_ARM_MM * cos(bearing));
    double const ol = OBS_SENSOR_LATERAL_MM + (OBS_SERVO_ARM_MM * sin(bearing));
    double const ox = g_x + (of * c) - (ol * s);
    double const oy = g_y + (of * s) + (ol * c);
    double best = 1e9;
    int found = 0;

    g_ray_quiet = 0;

    for (int k = -BEAM_HALF_DEG; k <= BEAM_HALF_DEG; k++)
    {
        double d = 0.0;
        double const ang = g_th + bearing + (k * PI / 180.0);

        if (ray_hit(ox, oy, ang, &d) && (d < best))
        {
            best = d;
            found = 1;
        }
    }

    *p_range = best;
    return found && (best < OBS_FAR_MM);
}

/* ---------- Cooperative scheduler on the simulated clock ---------- */

static pthread_mutex_t g_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cv = PTHREAD_COND_INITIALIZER;
static int             g_running;
static uint32_t        g_wake[2];
static pthread_t       g_task;
static int             g_task_started;

static int self_id (void)
{
    return (g_task_started && pthread_equal(pthread_self(), g_task)) ? 1 : 0;
}

static void sim_sleep (uint32_t ms)
{
    int const self = self_id();

    pthread_mutex_lock(&g_mu);
    g_wake[self] = g_now + ms;

    int const next = (g_wake[0] <= g_wake[1]) ? 0 : 1;

    while (g_now < g_wake[next])
    {
        g_now++;
        physics_step();
    }

    g_running = next;
    pthread_cond_broadcast(&g_cv);

    while (g_running != self)
    {
        pthread_cond_wait(&g_cv, &g_mu);
    }

    pthread_mutex_unlock(&g_mu);
}

static void *task_main (void *p_arg)
{
    (void)p_arg;
    pthread_mutex_lock(&g_mu);

    while (g_running != 1)
    {
        pthread_cond_wait(&g_cv, &g_mu);
    }

    pthread_mutex_unlock(&g_mu);
    task_obstacle_task(0, NULL);
    return NULL;
}

/* ---------- Kernel, BSP and driver doubles ---------- */

static UINT g_flags;

INT tm_printf (const UB *format, ...)
{
    if (g_verbose)
    {
        va_list args;

        va_start(args, format);
        printf("%8.3f ", now_s());
        vprintf((const char *)format, args);
        va_end(args);
    }

    return 0;
}

ER tk_dly_tsk (UINT delay)
{
    sim_sleep(delay);
    return E_OK;
}

ER tk_get_otm (SYSTIM *time)
{
    time->hi = 0;
    time->lo = g_now;
    return E_OK;
}

ID tk_cre_sem (const T_CSEM *config) { (void)config; return 1; }
ER tk_wai_sem (ID sem, INT count, TMO timeout)
{
    (void)sem; (void)count; (void)timeout;
    return E_OK;
}
ER tk_sig_sem (ID sem, INT count) { (void)sem; (void)count; return E_OK; }
ID tk_cre_flg (const T_CFLG *config) { g_flags = config->iflgptn; return 2; }
ER tk_ref_flg (ID flag, T_RFLG *info)
{
    (void)flag;
    info->flgptn = g_flags;
    return E_OK;
}
ER tk_set_flg (ID flag, UINT bits) { (void)flag; g_flags |= bits; return E_OK; }
ER tk_clr_flg (ID flag, UINT keep) { (void)flag; g_flags &= keep; return E_OK; }
ER tk_sta_tsk (ID task, INT code) { (void)task; (void)code; return E_OK; }
ID tasks_create (tasks_entry_t p_entry, PRI priority)
{
    (void)p_entry; (void)priority;
    return 3;
}
void out_w (UW address, UW value) { (void)address; (void)value; }
UW in_w (UW address) { (void)address; return 0U; }
ER pwm_set_pin (UINT pin) { (void)pin; return E_OK; }
ER pwm_set_wrap (UINT pin, UW wrap) { (void)pin; (void)wrap; return E_OK; }
ER pwm_set_enabled (UINT pin, BOOL enabled)
{
    (void)pin; (void)enabled;
    return E_OK;
}
ER pwm_set_cc (UINT pin, UW compare)
{
    if (BOARD_SERVO_PIN == pin)
    {
        /* The servo's own calibration is the configured one. */
        g_servo_target = (OBS_SERVO_LEFT_SIGN *
                          ((double)compare - OBS_SERVO_FRONT_USEC)) /
                         OBS_SERVO_USEC_PER_DEG;
    }

    return E_OK;
}

ER ultrasonic_init (void) { return E_OK; }

ultrasonic_result_t ultrasonic_ping (int32_t *p_mm)
{
    double range = 0.0;
    int const ok = beam_range(&range);
    int const side = fabs(g_servo_deg) > 1.0;
    int lost = 0;
    uint32_t wait = OBS_ECHO_TIMEOUT_MSEC;
    ultrasonic_result_t result = ULTRASONIC_OUT_OF_RANGE;

    g_ping_count++;

    if (fabs(g_servo_deg - g_servo_target) > 3.0)
    {
        g_misaimed++;
    }

    if ((g_sc.dead_after_ms > 0) && (g_now >= (uint32_t)g_sc.dead_after_ms))
    {
        lost = 1;
    }

    if (side && (g_pass_ms > 0U) &&
        ((int)g_injected < g_sc.side_none_count) &&
        (g_now >= (g_pass_ms + (uint32_t)g_sc.side_none_from_ms)))
    {
        lost = 1;
    }

    if ((g_sc.dead_in_pass_ms > 0) && (g_pass_ms > 0U) &&
        (g_now >= (g_pass_ms + (uint32_t)g_sc.dead_in_pass_ms)))
    {
        lost = 1;
    }

    if (lost)
    {
        result = ULTRASONIC_NO_ECHO;
        g_injected++;
    }
    else if ((g_sc.spike_every > 0) &&
             ((g_ping_count % (uint32_t)g_sc.spike_every) == 0U))
    {
        /* A stray value: a very short false echo or a lost one. */
        g_lcg = (g_lcg * 1103515245U) + 12345U;

        if ((g_lcg >> 16) & 1U)
        {
            *p_mm  = 40;
            result = ULTRASONIC_OK;
        }
        else
        {
            result = ULTRASONIC_OUT_OF_RANGE;
        }
        wait = 3U;
    }
    else if (ok)
    {
        g_lcg = (g_lcg * 1103515245U) + 12345U;
        /* Echo timing in usec, as the PWM counter measures it. */
        uint32_t const ticks =
            (uint32_t)(((range + (double)((g_lcg >> 16) % 7U) - 3.0) * 2000.0) /
                       343.0);

        *p_mm  = (int32_t)((ticks * 343U) / 2000U);
        result = ULTRASONIC_OK;
        wait   = 1U + (ticks / 1000U);
    }
    else
    {
        /* Nothing in range: the echo pulse runs to the time-out. */
        if (g_ray_quiet && side)
        {
            g_injected++;
        }
    }

    tk_dly_tsk(wait);
    return result;
}

int32_t ultrasonic_median_of_three (int32_t a, int32_t b, int32_t c)
{
    int32_t lo = a, mid = b, hi = c, t;

    if (lo > mid) { t = lo; lo = mid; mid = t; }
    if (mid > hi) { t = mid; mid = hi; hi = t; }
    if (lo > mid) { mid = lo; }
    (void)hi;
    return mid;
}

ER motion_get_status (motion_status_t *p_output)
{
    double const circ = PI * BOARD_WHEEL_DIAMETER_MM;

    memset(p_output, 0, sizeof(*p_output));
    /* The left motor is mounted mirrored: its count falls going forward. */
    p_output->left_count  = -(int32_t)lround((g_dist_l / circ) * BOARD_ENC_COUNTS_PER_REV);
    p_output->right_count = (int32_t)lround((g_dist_r / circ) * BOARD_ENC_COUNTS_PER_REV);
    p_output->b_valid = true;
    return E_OK;
}

/* ---------- Scenarios ---------- */

static rect_t box (double x0, double x1, double y0, double y1)
{
    rect_t r = {x0, x1, y0, y1, 1e9, -1e9};
    return r;
}

static void add_rect (rect_t r) { g_sc.rects[g_sc.nrect++] = r; }

static void add_stripe (double x0, double x1, double y0, double y1)
{
    stripe_t s = {x0, x1, y0, y1};
    g_sc.stripes[g_sc.nstripe++] = s;
}

static int build_scenario (const char *name)
{
    memset(&g_sc, 0, sizeof(g_sc));
    g_sc.name             = name;
    g_sc.expect_bypasses  = 1;
    g_sc.min_clear_mm     = 40.0;
    g_sc.servo_deg_per_ms = 0.6; /* SG90: 0.1 s per 60 degrees. */
    g_sc.prompt_mm        = 0.0;

    if (0 == strcmp(name, "open"))
    {
        g_sc.bias_y      = 30.0;
        g_sc.expect_side = -1;
        /* 4: object ahead, both sides open. */
        add_rect(box(700, 850, -75, 75));
        g_sc.true_rear_x = 850;
    }
    else if (0 == strcmp(name, "left_blocked"))
    {
        /* 5: a second box beside the left of the object. */
        add_rect(box(700, 850, -75, 75));
        add_rect(box(560, 900, 190, 330));
        g_sc.expect_side = -1;
        g_sc.true_rear_x = 850;
    }
    else if (0 == strcmp(name, "outward_corridor"))
    {
        /* 7: the left corridor is blocked only out at the far bearing,
         * where the +/-40 degree profile and its beam cannot see it.
         */
        add_rect(box(700, 850, -60, 60));
        add_rect(box(440, 540, 290, 380));
        g_sc.expect_side = -1;
        g_sc.true_rear_x = 850;
    }
    else if (0 == strcmp(name, "departure_late"))
    {
        /* 7b: the only open side has a box beside the car, behind every
         * bearing the servo can reach. The departure watch must stop the
         * car before it reaches it; no collision is allowed.
         */
        add_rect(box(700, 850, -75, 75));
        add_rect(box(560, 900, 190, 330));
        add_rect(box(150, 330, -420, -300));
        g_sc.expect_fault    = OBS_FAULT_NO_ROUTE;
        g_sc.expect_bypasses = 0;
    }
    else if (0 == strcmp(name, "widens"))
    {
        /* 8: narrow face, wider body further back: the side watch must
         * push the car further out to keep its margin.
         */
        add_rect(box(700, 760, -60, 60));
        add_rect(box(1000, 1300, -60, 200));
        add_rect(box(760, 1000, -60, 60));
        add_rect(box(560, 900, -330, -190));
        g_sc.expect_side = 1;
        g_sc.true_rear_x = 1300;
    }
    else if (0 == strcmp(name, "short"))
    {
        g_sc.bias_y      = 30.0;
        g_sc.expect_side = -1;
        /* 9: a thin board across the line. */
        add_rect(box(700, 740, -75, 75));
        g_sc.true_rear_x = 740;
    }
    else if (0 == strcmp(name, "long"))
    {
        g_sc.bias_y      = 30.0;
        g_sc.expect_side = -1;
        /* 10: the rear is far beyond the first view. */
        add_rect(box(700, 1400, -75, 75));
        g_sc.true_rear_x = 1400;
    }
    else if (0 == strcmp(name, "side_dropout"))
    {
        g_sc.bias_y      = 30.0;
        g_sc.expect_inject = 1;
        g_sc.expect_side = -1;
        /* 11: side pings lost for a while beside a long object. */
        add_rect(box(700, 1300, -75, 75));
        g_sc.true_rear_x       = 1300;
        g_sc.side_none_from_ms = 1500;
        g_sc.side_none_count   = (int)OBS_SIDE_NO_ECHO_LIMIT - 1;
    }
    else if (0 == strcmp(name, "quiet_face"))
    {
        g_sc.bias_y      = 30.0;
        g_sc.expect_inject = 1;
        g_sc.expect_side = -1;
        /* 12: part of the side face gives no echo, so the side looks
         * beyond range there although the object goes on.
         */
        rect_t r = box(700, 1300, -75, 75);

        r.quiet_x0 = 930;
        r.quiet_x1 = 1090;
        add_rect(r);
        g_sc.true_rear_x = 1300;
    }
    else if (0 == strcmp(name, "quiet_long"))
    {
        g_sc.bias_y      = 30.0;
        g_sc.expect_inject = 1;
        g_sc.expect_side = -1;
        /* 12b: a quiet stretch long enough to look like the end; the
         * view ahead must find the object again before the turn.
         */
        rect_t r = box(700, 1400, -75, 75);

        r.quiet_x0 = 930;
        r.quiet_x1 = 1190;
        add_rect(r);
        g_sc.true_rear_x = 1400;
    }
    else if (0 == strcmp(name, "reappears"))
    {
        g_sc.bias_y      = 30.0;
        g_sc.expect_side = -1;
        /* 13: two boxes in a row with a gap the car cannot turn into. */
        add_rect(box(700, 850, -75, 75));
        add_rect(box(1000, 1150, -75, 75));
        g_sc.true_rear_x = 1150;
    }
    else if (0 == strcmp(name, "return_blocked"))
    {
        g_sc.bias_y      = 30.0;
        g_sc.expect_side = -1;
        /* 14: a post in the way back, between the path and the line. */
        add_rect(box(700, 850, -75, 75));
        add_rect(box(980, 1040, 20, 80));
        add_rect(box(980, 1040, -80, -20));
        g_sc.true_rear_x = 1040;
    }
    else if (0 == strcmp(name, "crossing"))
    {
        g_sc.bias_y      = 30.0;
        g_sc.expect_side = -1;
        /* 18: a crossing line and a stray mark on the way back. */
        add_rect(box(700, 850, -75, 75));
        add_stripe(1060, 1078, -600, 600);
        add_stripe(1000, 1300, 160, 178);
        add_stripe(1000, 1300, -178, -160);
        g_sc.true_rear_x  = 850;
        g_sc.expect_marks = 1;
    }
    else if (0 == strcmp(name, "repeated"))
    {
        g_sc.bias_y      = 30.0;
        g_sc.expect_side = -1;
        /* 19: two obstacles one after the other on the line. */
        add_rect(box(700, 850, -75, 75));
        add_rect(box(2300, 2450, -75, 75));
        g_sc.expect_bypasses = 2;
    }
    else if (0 == strcmp(name, "both_blocked"))
    {
        /* No usable side at all: stop with NO_ROUTE, never scrape. */
        add_rect(box(700, 850, -75, 75));
        add_rect(box(560, 900, 190, 330));
        add_rect(box(560, 900, -330, -190));
        g_sc.expect_fault = OBS_FAULT_NO_ROUTE;
        g_sc.expect_bypasses = 0;
    }
    else if (0 == strcmp(name, "sensor_dead"))
    {
        /* 20a: the sensor dies while passing. */
        add_rect(box(700, 1300, -75, 75));
        g_sc.dead_after_ms   = 9000;
        g_sc.expect_fault    = OBS_FAULT_NO_ECHO;
        g_sc.expect_bypasses = 0;
    }
    else if (0 == strcmp(name, "sensor_dead_pass"))
    {
        /* 20a': the sensor dies alongside the object. */
        add_rect(box(700, 1300, -75, 75));
        g_sc.bias_y          = 30.0;
        g_sc.dead_in_pass_ms = 1500;
        g_sc.expect_fault    = OBS_FAULT_NO_ECHO;
        g_sc.expect_bypasses = 0;
        g_sc.expect_inject   = 1;
    }
    else if (0 == strcmp(name, "servo_slow"))
    {
        g_sc.bias_y      = 30.0;
        g_sc.expect_side = -1;
        /* 20b: the servo settles slower than the settle model allows. */
        add_rect(box(700, 850, -75, 75));
        g_sc.servo_deg_per_ms = 0.15;
        g_sc.true_rear_x      = 850;
    }
    else if (0 == strcmp(name, "servo_stuck"))
    {
        /* 20c: the servo never moves; no side can be verified. */
        add_rect(box(700, 850, -75, 75));
        g_sc.servo_stuck     = 1;
        g_sc.expect_fault    = OBS_FAULT_NO_ROUTE;
        g_sc.expect_bypasses = 0;
    }
    else if (0 == strcmp(name, "spikes"))
    {
        g_sc.bias_y      = 30.0;
        g_sc.expect_side = -1;
        /* 20d: every seventh ping is a stray short or lost value. */
        add_rect(box(700, 850, -75, 75));
        g_sc.spike_every = 7;
        g_sc.true_rear_x = 850;
    }
    else
    {
        return 0;
    }

    for (int i = 0; i < g_sc.nrect; i++)
    {
        g_sc.rects[i].y0 += g_sc.bias_y;
        g_sc.rects[i].y1 += g_sc.bias_y;
    }

    /* With nominal sensing, the turn back must come promptly once the
     * rear is passed: no more than this beyond the true rear, and no
     * more than two stopped periods at the turn point.
     */
    if ((g_sc.true_rear_x > 0.0) && (0.0 == g_sc.prompt_mm) &&
        (0 == g_sc.spike_every) && (g_sc.servo_deg_per_ms >= 0.6))
    {
        g_sc.prompt_mm = 220.0;
    }

    if (g_mirror)
    {
        for (int i = 0; i < g_sc.nrect; i++)
        {
            double const y0 = g_sc.rects[i].y0;

            g_sc.rects[i].y0 = -g_sc.rects[i].y1;
            g_sc.rects[i].y1 = -y0;
        }

        for (int i = 0; i < g_sc.nstripe; i++)
        {
            double const y0 = g_sc.stripes[i].y0;

            g_sc.stripes[i].y0 = -g_sc.stripes[i].y1;
            g_sc.stripes[i].y1 = -y0;
        }

        g_sc.expect_side = -g_sc.expect_side;
    }

    return 1;
}

/* ---------- Controller stand-in and checks ---------- */

typedef enum { MODE_FOLLOW, MODE_BYPASS, MODE_FAULT } mode_t_;

int main (int argc, char **argv)
{
    mode_t_  mode = MODE_FOLLOW;
    int      bypasses = 0;
    int      route_side = 0;
    int      turned_back = 0;
    double   turn_back_x = 0.0;
    int      pass_end_stops = 0;
    double   max_line_err = 0.0;
    double   follow_after = 0.0;
    unsigned faults = 0U;
    int      failed = 0;

    if (argc < 2)
    {
        fprintf(stderr, "usage: %s scenario [mirror] [verbose]\n", argv[0]);
        return 2;
    }

    g_mirror  = (argc > 2) && (0 == strcmp(argv[2], "1"));
    g_verbose = (argc > 3) && (0 == strcmp(argv[3], "1"));

    if (!build_scenario(argv[1]))
    {
        fprintf(stderr, "unknown scenario %s\n", argv[1]);
        return 2;
    }

    if (obstacle_init() < E_OK)
    {
        return 3;
    }

    g_servo_deg   = g_servo_target;
    g_wake[1]     = 0U;
    g_running     = 0;
    pthread_create(&g_task, NULL, task_main, NULL);
    g_task_started = 1;

    while (g_now < SIM_LIMIT_MS)
    {
        int l = 0, r = 0;
        uint32_t const phase = obs_state_status_phase(obs_state_status_get());

        ir_read(&l, &r);
        l |= g_ir_latch_l;
        r |= g_ir_latch_r;
        g_ir_latch_l = 0;
        g_ir_latch_r = 0;

        if (MODE_FOLLOW == mode)
        {
            g_track_clear = 0;

            if (obstacle_has_fault())
            {
                faults |= OBS_FAULT_NO_ECHO;
                mode = MODE_FAULT;
            }
            else if (obstacle_is_blocked())
            {
                g_cmd_l = 0.0;
                g_cmd_r = 0.0;
                mode    = MODE_BYPASS;
            }
            else
            {
                g_cmd_l = (l && !r) ? CORRECT_SLOW_RPM : FOLLOW_RPM;
                g_cmd_r = (r && !l) ? CORRECT_SLOW_RPM : FOLLOW_RPM;

                if (bypasses > 0)
                {
                    follow_after += rpm_to_mm_per_ms(FOLLOW_RPM) * CTRL_MS;

                    if (follow_after > 300.0)
                    {
                        double const err = fabs(g_y);

                        max_line_err = (err > max_line_err) ? err : max_line_err;
                    }
                }
            }
        }
        else if (MODE_BYPASS == mode)
        {
            obstacle_drive_t const drive =
                obstacle_bypass_step((uint32_t)l, (uint32_t)r, g_now);
            obs_state_route_t const route =
                obs_state_status_route(obs_state_status_get());

            g_track_clear = 1;

            if (l || r)
            {
                double const ir_y = g_y + (OBS_IR_TO_AXLE_MM * sin(g_th));

                if (fabs(ir_y) > 60.0)
                {
                    g_marks_seen++;
                }
            }

            if ((OBS_ROUTE_LEFT == route) && (0 == route_side))
            {
                route_side = 1;
            }
            else if ((OBS_ROUTE_RIGHT == route) && (0 == route_side))
            {
                route_side = -1;
            }

            if ((OBS_PHASE_PASS_SIDE == phase) && (0U == g_pass_ms))
            {
                g_pass_ms = g_now;
            }

            if ((OBS_PHASE_TURN_BACK == phase) && !turned_back)
            {
                turned_back = 1;
                turn_back_x = g_x;
            }

            if ((OBS_PHASE_PASS_END == phase) && drive.b_stop)
            {
                pass_end_stops++;
            }

            if (drive.b_fault)
            {
                faults |= obs_state_status_faults(obs_state_status_get());
                g_cmd_l = 0.0;
                g_cmd_r = 0.0;
                mode    = MODE_FAULT;
            }
            else if (drive.b_done)
            {
                bypasses++;
                follow_after = 0.0;
                mode = MODE_FOLLOW;
                if (g_verbose)
                {
                    printf("bypass %d done at x=%.0f y=%.0f th=%.1f\n",
                           bypasses, g_x, g_y, g_th * 180.0 / PI);
                }
            }
            else if (drive.b_line_follow)
            {
                g_cmd_l = (l && !r) ? CORRECT_SLOW_RPM : FOLLOW_RPM;
                g_cmd_r = (r && !l) ? CORRECT_SLOW_RPM : FOLLOW_RPM;
            }
            else if (drive.b_stop)
            {
                g_cmd_l = 0.0;
                g_cmd_r = 0.0;
            }
            else
            {
                g_cmd_l = drive.left_rpm;
                g_cmd_r = drive.right_rpm;
            }
        }
        else
        {
            g_cmd_l = 0.0;
            g_cmd_r = 0.0;
            break;
        }

        if ((bypasses >= g_sc.expect_bypasses) && (g_sc.expect_bypasses > 0) &&
            (follow_after > 900.0))
        {
            break;
        }

        sim_sleep(CTRL_MS);
    }

    /* Verdict. */
    char why[256] = "";

    if (g_collided)
    {
        snprintf(why, sizeof(why), "collision");
        failed = 1;
    }
    else if (faults != g_sc.expect_fault)
    {
        snprintf(why, sizeof(why), "faults 0x%x, expected 0x%x", faults,
                 g_sc.expect_fault);
        failed = 1;
    }
    else if (bypasses != g_sc.expect_bypasses)
    {
        snprintf(why, sizeof(why), "%d bypasses, expected %d", bypasses,
                 g_sc.expect_bypasses);
        failed = 1;
    }
    else if ((0 != g_sc.expect_side) && (route_side != g_sc.expect_side))
    {
        snprintf(why, sizeof(why), "route %d, expected %d", route_side,
                 g_sc.expect_side);
        failed = 1;
    }
    else if (g_min_clear < g_sc.min_clear_mm)
    {
        snprintf(why, sizeof(why), "body clearance %.0f mm < %.0f",
                 g_min_clear, g_sc.min_clear_mm);
        failed = 1;
    }
    else if ((g_sc.expect_bypasses > 0) && (max_line_err > 25.0))
    {
        snprintf(why, sizeof(why), "line error %.0f mm after return",
                 max_line_err);
        failed = 1;
    }
    else if ((g_sc.true_rear_x > 0.0) && turned_back &&
             ((turn_back_x - OBS_ROBOT_REAR_MM) < g_sc.true_rear_x))
    {
        snprintf(why, sizeof(why), "turned back with rear %.0f mm short",
                 g_sc.true_rear_x - (turn_back_x - OBS_ROBOT_REAR_MM));
        failed = 1;
    }
    else if ((g_sc.prompt_mm > 0.0) &&
             ((turn_back_x - g_sc.true_rear_x) > g_sc.prompt_mm))
    {
        snprintf(why, sizeof(why), "turned back %.0f mm past the rear",
                 turn_back_x - g_sc.true_rear_x);
        failed = 1;
    }
    else if ((g_sc.prompt_mm > 0.0) && (pass_end_stops > 2))
    {
        snprintf(why, sizeof(why), "stopped %d periods at the turn point",
                 pass_end_stops);
        failed = 1;
    }
    else if (g_sc.expect_inject && (0U == g_injected))
    {
        snprintf(why, sizeof(why), "the injected fault never happened");
        failed = 1;
    }
    else if (g_sc.expect_marks && (0U == g_marks_seen))
    {
        snprintf(why, sizeof(why), "the IR never crossed the stray marks");
        failed = 1;
    }
    else if ((g_sc.servo_deg_per_ms >= 0.6) && !g_sc.servo_stuck &&
             (g_misaimed > 0U))
    {
        snprintf(why, sizeof(why), "%u pings before the servo settled",
                 g_misaimed);
        failed = 1;
    }

    printf("%s %-16s %-6s side=%-2d clear=%4.0f turn_x=%5.0f past_rear=%4.0f "
           "stops=%d line_err=%2.0f marks=%u misaim=%u inject=%u faults=0x%x "
           "t=%.1fs %s\n",
           failed ? "FAIL" : "PASS", g_sc.name, g_mirror ? "mirror" : "",
           route_side, (g_min_clear > 1e8) ? -1.0 : g_min_clear, turn_back_x,
           turned_back ? (turn_back_x - g_sc.true_rear_x) : 0.0,
           pass_end_stops, max_line_err, g_marks_seen, g_misaimed, g_injected,
           faults, now_s(), why);
    fflush(stdout);
    /* The obstacle task never returns; leave without joining it. */
    _Exit(failed ? 1 : 0);
}
