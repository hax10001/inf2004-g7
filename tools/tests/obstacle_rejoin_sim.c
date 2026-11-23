/* Closed-loop rejoin after a bypass: the real obstacle return state
 * machine drives a simulated robot back to a simulated line, including
 * stops where the robot was not heading along the line (angled approach,
 * mid-correction) and an obstacle on a curve. Odometry is exact; only the
 * line's true position differs from the straight-ahead assumption.
 * Success: after hand-back and simple line following, the line sensors
 * straddle the line and the robot faces the original travel direction. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "modules/obstacle/scan_plan.c"
#include "modules/obstacle/obstacle.c"

static UINT flags;
static uint32_t side_seq,front_seq;
INT tm_printf(UB const *format,...) {(void)format;return 0;}
UINT obs_state_flags_read(void) {return flags;}
void obs_state_flags_set(UINT bits) {flags|=bits;}
void obs_state_flags_clear(UINT bits) {flags&=~bits;}
void obs_state_set_phase(obs_state_phase_t phase) {(void)phase;}
void obs_state_set_route(obs_state_route_t route) {(void)route;}
void obs_state_set_width_cm(int32_t width) {(void)width;}
void obs_state_count_bypass(void) {}
void obs_state_faults_update(uint32_t set,uint32_t clear) {(void)set;(void)clear;}
void obs_state_plan_load(scan_plan_bypass_t *plan) {(void)plan;}
void obs_state_watch_angle_set(int32_t angle) {(void)angle;}
uint32_t obs_state_side_seq_get(void) {return ++side_seq;}
int32_t obs_state_side_range_get(void) {return 4000;}
uint32_t obs_state_front_seq_get(void) {return ++front_seq;}
int32_t obs_state_front_range_get(void) {return 4000;}
int32_t servo_left_limit_deg(void) {return 60;}
int32_t servo_right_limit_deg(void) {return 70;}

#define DT_S 0.05
#define TAPE_HALF_MM 9.0
#define SENSOR_HALF_GAP_MM 13.0 /* 4 mm clear of the tape each side */

typedef struct { double x, y, h; } pose_t;
/* The line: through the IR position at the stop (125, 0), heading
 * line_deg in the obstacle frame; bends with curve_mm radius (0 straight,
 * >0 to the left) after x = 150 along it. */
static double line_deg, curve_mm;

static void line_at(double s, double *x, double *y, double *dir)
{
    double const a0 = line_deg * M_PI / 180.0;
    double const bend = 150.0;
    if (curve_mm == 0.0 || s <= bend) {
        *x = 125.0 + s * cos(a0); *y = s * sin(a0); *dir = a0;
        return;
    }
    {
        double const bx = 125.0 + bend * cos(a0), by = bend * sin(a0);
        double const r = fabs(curve_mm), sg = curve_mm > 0 ? 1.0 : -1.0;
        double const t = (s - bend) / r;
        double const cx = bx - sg * r * sin(a0), cy = by + sg * r * cos(a0);
        *dir = a0 + sg * t;
        *x = cx + sg * r * sin(*dir); *y = cy - sg * r * cos(*dir);
    }
}
/* Distance from a point to the line and the line direction there. */
static double line_dist(double px, double py, double *dir_out)
{
    double best = 1e9, best_dir = 0.0;
    for (double s = -400.0; s <= 3000.0; s += 2.0) {
        double x, y, d;
        line_at(s, &x, &y, &d);
        double const dd = hypot(px - x, py - y);
        if (dd < best) { best = dd; best_dir = d; }
    }
    if (dir_out) *dir_out = best_dir;
    return best;
}
static void sensors(pose_t const *p, bool *l, bool *r)
{
    double const fx = p->x + OBS_IR_TO_AXLE_MM * cos(p->h);
    double const fy = p->y + OBS_IR_TO_AXLE_MM * sin(p->h);
    double const nx = -sin(p->h), ny = cos(p->h);
    *l = line_dist(fx + SENSOR_HALF_GAP_MM * nx, fy + SENSOR_HALF_GAP_MM * ny,
                   NULL) <= TAPE_HALF_MM;
    *r = line_dist(fx - SENSOR_HALF_GAP_MM * nx, fy - SENSOR_HALF_GAP_MM * ny,
                   NULL) <= TAPE_HALF_MM;
}
static void drive_rpm(pose_t *p, double left_rpm, double right_rpm)
{
    double const k = OBS_CIRC_MM / 60.0 * DT_S;
    double const l = left_rpm * k, r = right_rpm * k;
    double const turn = (r - l) / OBS_TRACK_WIDTH_MM;
    double const mid = p->h + turn / 2.0;
    p->x += (l + r) / 2.0 * cos(mid); p->y += (l + r) / 2.0 * sin(mid);
    p->h += turn;
}
/* Plain line following: steer away from the sensor on the tape; with
 * neither on it, spin towards the hand-back hint until one finds it. */
static void follow(pose_t *p, int hint, int periods)
{
    int spin = 0;
    for (int i = 0; i < periods; i++) {
        bool l, r;
        sensors(p, &l, &r);
        if (l && !r) drive_rpm(p, 12, 30);
        else if (r && !l) drive_rpm(p, 30, 12);
        else if (!l && !r && hint && spin < 40) {
            spin++;
            drive_rpm(p, hint > 0 ? -15 : 15, hint > 0 ? 15 : -15);
        } else { hint = 0; drive_rpm(p, 25, 25); }
    }
}

/* must: 1 must end on the line; 0 the line swings into the passing lane
 * behind the box (the robot is over it when it turns back), which neither
 * the old nor the new return finds: it must still hand back to line
 * following without a fault. */
typedef struct { const char *name; double line_deg, curve_mm; int side, must; } case_t;

static int run_case(case_t const *c, bool verbose)
{
    pose_t p;
    obstacle_drive_t out;
    uint32_t now = 1000U;
    int periods = 0;
    int last_phase = -1;
    bool const trace = getenv("REJOIN_TRACE") != NULL &&
                       strstr(c->name, getenv("REJOIN_TRACE")) != NULL;
    flags = 0U;
    line_deg = c->line_deg; curve_mm = c->curve_mm;
    /* Just past the box, out in the passing lane, still heading as at the
     * stop: x 650 (box face at 300, 200 deep plus the body), 260 aside. */
    g_seq = (obstacle_seq_t){0};
    g_seq.b_active = true; g_seq.b_planned = true; g_seq.side = c->side;
    g_seq.search_side = c->side; g_seq.lateral_mm = 260;
    g_seq.face_x_mm = 300.0F; g_seq.rear_x_mm = 500.0F;
    p.x = 650.0; p.y = c->side * 260.0; p.h = 0.0;
    g_seq.pose.x_mm = (float)p.x; g_seq.pose.y_mm = (float)p.y;
    g_seq.pose.heading_rad = 0.0F;
    enter_turn(&g_seq, OBS_PHASE_TURN_BACK,
               -c->side * OBS_RETURN_DEG * DDEG_PER_DEG, now);
    for (;;) {
        bool l, r;
        out = (obstacle_drive_t){0};
        sensors(&p, &l, &r);
        g_seq.b_ir_left = l; g_seq.b_ir_right = r;
        run_phase(&g_seq, &out, l || r, now);
        if (trace && ((int)g_seq.phase != last_phase || (periods > 95 && periods < 160 && (l || r || periods % 5 == 0)))) {
            last_phase = (int)g_seq.phase;
            printf("    t%4d phase %2d x%6.0f y%6.0f h%6.1f L%d R%d lineoff %5.0f fy %6.0f bs %d\n",
                   periods, last_phase, p.x, p.y, p.h * 180 / M_PI, l, r,
                   line_dist(p.x, p.y, NULL), g_seq.pose.y_mm, g_seq.b_black_seen);
        }
        if (out.b_done || out.b_fault || ++periods > 1200) break;
        {
            /* Odometry: the controller's own pose gets the same wheel
             * motion; it keeps any corrections it made to it. */
            pose_t const before = p;
            pose_t odo = {g_seq.pose.x_mm, g_seq.pose.y_mm,
                          g_seq.pose.heading_rad};
            double fwd, turn, c0, s0, lx, ly;
            if (out.b_line_follow) follow(&p, 0, 1);
            else if (!out.b_stop) drive_rpm(&p, out.left_rpm, out.right_rpm);
            /* The same displacement, in the robot's own frame. */
            c0 = cos(before.h); s0 = sin(before.h);
            lx = (p.x - before.x) * c0 + (p.y - before.y) * s0;
            ly = -(p.x - before.x) * s0 + (p.y - before.y) * c0;
            turn = p.h - before.h; fwd = 0.0; (void)fwd;
            odo.x += lx * cos(odo.h) - ly * sin(odo.h);
            odo.y += lx * sin(odo.h) + ly * cos(odo.h);
            odo.h += turn;
            g_seq.pose.x_mm = (float)odo.x; g_seq.pose.y_mm = (float)odo.y;
            g_seq.pose.heading_rad = (float)odo.h;
        }
        now += 50U;
    }
    if (!out.b_done) {
        printf("  %s: %s after %d periods\n", c->name,
               out.b_fault ? "FAULT" : "no hand-back", periods);
        return 0;
    }
    if (!c->must) {
        printf("  %-34s line in the passing lane: handed back after %d "
               "seek legs (known limit)\n", c->name, (int)g_seq.seek_legs);
        return 1;
    }
    {
        int const seeks = g_seq.seek_legs;
        double dir, h_err;
        follow(&p, out.line_hint, 60); /* 3 s of line following */
        double const fx = p.x + OBS_IR_TO_AXLE_MM * cos(p.h);
        double const fy = p.y + OBS_IR_TO_AXLE_MM * sin(p.h);
        double const off = line_dist(fx, fy, &dir);
        h_err = remainder(p.h - dir, 2.0 * M_PI) * 180.0 / M_PI;
        if (verbose)
            printf("  %-34s seek legs %d, off line %5.1f mm, heading "
                   "%+6.1f deg to the line\n", c->name, seeks, off, h_err);
        if (c->line_deg == 0.0 && c->curve_mm == 0.0)
            /* Head-on as before 23 Nov (4.5 mm, 0.1 deg; was 3.1, 0.5). */
            return seeks == 0 && off <= 6.0 && fabs(h_err) <= 2.0;
        return seeks == 0 && off <= 15.0 && fabs(h_err) <= 10.0;
    }
}

int main(void)
{
    static const case_t cases[] = {
        {"head-on, left bypass", 0.0, 0.0, 1, 1},
        {"head-on, right bypass", 0.0, 0.0, -1, 1},
        {"stopped 15 deg left of line, left", -15.0, 0.0, 1, 1},
        {"stopped 15 deg left of line, right", -15.0, 0.0, -1, 1},
        {"stopped 15 deg right of line, left", 15.0, 0.0, 1, 1},
        {"stopped 15 deg right of line, right", 15.0, 0.0, -1, 1},
        {"stopped 25 deg left of line, left", -25.0, 0.0, 1, 1},
        {"stopped 25 deg left of line, right", -25.0, 0.0, -1, 0},
        {"stopped 25 deg right of line, left", 25.0, 0.0, 1, 0},
        {"stopped 25 deg right of line, right", 25.0, 0.0, -1, 1},
        {"curve bending left (r 600), left", 0.0, 600.0, 1, 1},
        {"curve bending left (r 600), right", 0.0, 600.0, -1, 0},
        {"curve bending right (r 600), left", 0.0, -600.0, 1, 0},
        {"curve bending right (r 600), right", 0.0, -600.0, -1, 1},
    };
    int failed = 0;
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        int const ok = run_case(&cases[i], true);
        if (!ok) { printf("FAIL %s\n", cases[i].name); failed++; }
    }
    assert(0 == failed);
    puts("PASS rejoin: head-on, +-15/+-25 deg stops and curves bending away "
         "from the passing lane end on the line facing the travel direction");
    return 0;
}
/*** end of file ***/
