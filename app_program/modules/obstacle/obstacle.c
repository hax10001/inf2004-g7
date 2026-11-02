/** @file obstacle.c
 *
 * @brief Lifecycle and the bypass sequencer; shared status is obs_state.
 *
 * The bypass, in order:
 *  1. Stop and scan; take the clear side (a fault only if neither is).
 *  2. Swing out to that side far enough to clear the object.
 *  3. Turn parallel to the line and drive past the object, watching it from
 *     the side to see where it ends and glancing ahead for anything else.
 *  4. Turn back towards the line, which is assumed to run straight on, find
 *     it with the IR sensors, straighten up and hand back to line following.
 *
 * Distance and heading come from encoder counts since the bypass began, in
 * a frame where the line is y = 0 and the robot started along +x. Headings
 * are in tenths of a degree and positive is a left turn, matching motion.h.
 * All turns are arcs about a stopped wheel because the motor driver is
 * forward-only.
 */
#include "obstacle.h"
#include "obs_config.h"
#include "obs_state.h"
#include "scan_plan.h"
#include "servo.h"
#include "task_obstacle.h"
#include "ultrasonic.h"
#include "app/app_config.h"
#include "app/tasks.h"
#include "modules/motion/motion.h"
#include <tm/tmonitor.h>

#define MM_PER_CM 10
#define DDEG_PER_DEG 10
#define HALF 2
/* Wheel circumference in mm; folded at compile time. */
#define OBS_CIRC_MM (BOARD_WHEEL_DIAMETER_MM * 3.14159265F)
#define OBS_COUNTS_PER_REV ((float)BOARD_ENC_COUNTS_PER_REV)
#define OBS_DDEG_PER_RAD 572.958F
#define OBS_RAD_PER_DEG 0.0174533F
#define OBS_RAD_PER_DDEG 0.00174533F
/* <math.h> clashes with the kernel's size_t, so use the compiler's names
 * for the libm functions directly; -lm is already linked.
 */
#define OBS_COSF(x) __builtin_cosf(x)
#define OBS_SINF(x) __builtin_sinf(x)
/* Body side beyond the wheel centre: the corner a pivot must clear. */
#define OBS_BODY_PAST_WHEEL_MM                                                 \
    ((OBS_ROBOT_WIDTH_MM / HALF) - (OBS_TRACK_WIDTH_MM / HALF))
#define OBS_HALF_PATH_MM ((OBS_ROBOT_WIDTH_MM / HALF) + OBS_SAFETY_MARGIN_MM)

/** @brief Dead-reckoned pose: x along the line, y to its left, both from
 *  where the obstacle was confirmed; heading positive to the left.
 */
typedef struct
{
    float x_mm;
    float y_mm;
    float heading_rad;
} obstacle_pose_t;

typedef struct
{
    bool              b_active;
    obs_state_phase_t phase;
    uint32_t          enter_msec;
    int32_t           last_left;
    int32_t           last_right;
    obstacle_pose_t   pose;
    bool              b_planned; /**< A side has been chosen. */
    int32_t           side; /**< 1 bypasses on the left, -1 on the right. */
    int32_t           out_ddeg;
    int32_t           lateral_mm; /**< Offset from the line while passing. */
    int32_t           look_deg;   /**< Servo angle of the side watch. */
    int32_t           turn_target_ddeg;
    int32_t           turn_dir; /**< Arc direction of the current turn. */
    float             face_x_mm; /**< Front face of the object. */
    float             rear_x_mm; /**< Best estimate of the object's rear. */
    float             end_x_mm;  /**< Where the axle may turn back in. */
    float             beside_mm; /**< Closest side gap seen while passing. */
    bool              b_seen;     /**< The side watch has seen the object. */
    uint32_t          side_hits;  /**< Side pings that saw the object. */
    float             hit_rear_mm; /**< Furthest point the side echoes saw. */
    uint32_t          clear_hits; /**< Side pings since last seen. */
    uint32_t          side_seq;   /**< Last side ping handled. */
    uint32_t          front_seq;  /**< Last front ping handled. */
    uint32_t          guard_hits; /**< Front pings in a row that were near. */
    obs_state_phase_t resume_phase; /**< Phase a stop to look interrupted. */
    uint32_t          replans;
    int32_t           search_side; /**< Side of the line approached from. */
    int32_t           search_ddeg; /**< Heading of the run to the line. */
    uint32_t          retries;
    uint32_t          line_hits;
    uint32_t          verify_cycles;
    bool              b_black_seen;
} obstacle_seq_t;

static obstacle_seq_t g_seq;

/** @brief Read the signed wheel counts; false until motion has data. */
static bool counts_read (int32_t * p_left, int32_t * p_right);
/** @brief Advance the pose by the wheel travel since the last call.
 *
 * The motor driver is forward-only, so each wheel's travel is the size of
 * its count change. This holds whichever way an encoder is wired; the left
 * motor is mounted mirrored, so its raw count may fall as it drives forward.
 */
static void pose_update (obstacle_seq_t * p_seq, int32_t left_count,
                         int32_t right_count);
/** @brief Heading of the pose in tenths of a degree. */
static int32_t heading_ddeg (obstacle_pose_t const * p_pose);
/** @brief Start a bypass: zero the pose and enter STOPPING. */
static void seq_begin (obstacle_seq_t * p_seq, int32_t left_count,
                       int32_t right_count, uint32_t now_msec);
/** @brief Enter a phase, restart its timer, set the watches and log. */
static void enter_phase (obstacle_seq_t * p_seq, obs_state_phase_t phase,
                         uint32_t now_msec);
/** @brief Enter a turn phase towards an absolute heading. */
static void enter_turn (obstacle_seq_t * p_seq, obs_state_phase_t phase,
                        int32_t target_ddeg, uint32_t now_msec);
/** @brief Arc towards the turn target; true once it is reached. */
static bool turn_step (obstacle_seq_t const * p_seq, obstacle_drive_t * p_out);
/** @brief Run the current phase for one controller period. */
static void run_phase (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                       bool b_black, uint32_t now_msec);
/** @brief Drive alongside the object until the side watch says it ended. */
static void run_pass_side (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                           uint32_t now_msec);
/** @brief Handle a new side ping, if any; true if it saw the object. */
static bool side_ping_update (obstacle_seq_t * p_seq);
/** @brief Point the servo at the object for the side watch. */
static void aim_side_watch (obstacle_seq_t * p_seq);
/** @brief Heading that holds the passing offset from the line. */
static int32_t offset_heading (obstacle_seq_t const * p_seq);
/** @brief Black seen near where the IR sensors should meet the line. */
static bool line_near (obstacle_seq_t const * p_seq, bool b_black);
/** @brief The object is still in the way on the return: run parallel and
 *  carry on passing it, with the side watch deciding where it ends.
 */
static void pass_further (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                          uint32_t now_msec);
/** @brief The IR sensors are on the line: fix the pose from them and head
 *  on to where the turn onto the line starts.
 */
static void line_found (obstacle_seq_t * p_seq, uint32_t now_msec);
/** @brief Check the latest front ping; true once something is in the way.
 *  @param b_to_line Ignore objects well beyond the line.
 */
static bool guard_tripped (obstacle_seq_t * p_seq, bool b_to_line);
/** @brief Stop and scan again, remembering the interrupted phase. */
static void guard_stop (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                        uint32_t now_msec);
/** @brief Drive one wheel only; positive turn is left. */
static void set_arc (obstacle_drive_t * p_out, int32_t turn);
/** @brief Drive straight, trimming towards a target heading. */
static void set_straight (obstacle_drive_t * p_out, int32_t heading,
                          int32_t target_ddeg);
/** @brief Stop, record a fault bit and abort the obstacle task. */
static void raise_fault (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                         uint32_t fault_bit);
/** @brief End the bypass and hand control back. */
static void finish_bypass (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                           bool b_count);
/** @brief Read the plan from the scan and start swinging out. */
static void handle_plan (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                         uint32_t now_msec);

ER obstacle_init (void)
{
    ER error = E_OK;

    if (0 != APP_OBSTACLE_ENABLED)
    {
        error = obs_state_init();

        if (error >= E_OK)
        {
            error = ultrasonic_init();
        }

        if (error >= E_OK)
        {
            error = servo_init();
        }

        if (error < E_OK)
        {
            /* UB views the same string bytes; tm_printf does not modify. */
            tm_printf((UB *)"[OBS] Init failed: %d\n", error);
        }
    }

    return error;
}

ER obstacle_start (void)
{
    ER error = E_OK;

    if (0 != APP_OBSTACLE_ENABLED)
    {
        ID const h_task =
            tasks_create(task_obstacle_task, APP_OBSTACLE_PRIORITY);

        if (h_task <= E_OK)
        {
            /* UB views the same string bytes; tm_printf does not modify. */
            tm_printf((UB *)"[OBS] Task creation failed: %d\n", h_task);
            error = (E_OK == h_task) ? E_OBJ : h_task;
        }
        else
        {
            error = tk_sta_tsk(h_task, 0);
        }
    }

    return error;
}

bool obstacle_is_blocked (void)
{
    bool b_blocked = false;

    if (0 != APP_OBSTACLE_ENABLED)
    {
        b_blocked = (0U != (obs_state_flags_read() & OBS_EVT_BLOCKED));
    }

    return b_blocked;
}

bool obstacle_has_fault (void)
{
    bool b_fault = false;

    if ((0 != APP_OBSTACLE_ENABLED) && (0 != OBS_FAILSAFE_NO_ECHO))
    {
        uint32_t const faults = obs_state_status_faults(obs_state_status_get());

        b_fault = (0U != (faults & OBS_FAULT_NO_ECHO));
    }

    return b_fault;
}

void obstacle_abort (void)
{
    if (0 != APP_OBSTACLE_ENABLED)
    {
        obs_state_flags_set(OBS_EVT_ABORT);
        g_seq.b_active = false;
        obs_state_set_phase(OBS_PHASE_IDLE);
        obs_state_set_route(OBS_ROUTE_NONE);
    }
}

obstacle_drive_t obstacle_bypass_step (uint32_t line_left, uint32_t line_right,
                                       uint32_t now_msec)
{
    obstacle_drive_t out         = {0, 0, false, false, false, false};
    int32_t          left_count  = 0;
    int32_t          right_count = 0;

    if (0 == APP_OBSTACLE_ENABLED)
    {
        out.b_done = true;
    }
    else if (!counts_read(&left_count, &right_count))
    {
        out.b_stop = true;
    }
    else
    {
        obstacle_seq_t * const p_seq = &g_seq;

        if (!p_seq->b_active)
        {
            seq_begin(p_seq, left_count, right_count, now_msec);
        }

        pose_update(p_seq, left_count, right_count);

        bool const b_black = (0U != line_left) || (0U != line_right);

        run_phase(p_seq, &out, b_black, now_msec);
    }

    return out;
}

static void run_phase (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                       bool b_black, uint32_t now_msec)
{
    UINT const     pattern = obs_state_flags_read();
    uint32_t const elapsed = now_msec - p_seq->enter_msec;
    int32_t const  heading = heading_ddeg(&p_seq->pose);
    /* Offset measured towards the bypass side. */
    float const    out_y = ((float)p_seq->side) * p_seq->pose.y_mm;
    uint32_t const limit = (OBS_PHASE_SCANNING == p_seq->phase)
                               ? OBS_SCAN_TIMEOUT_MSEC
                               : OBS_STATE_TIMEOUT_MSEC;

    if (OBS_PHASE_FAULT == p_seq->phase)
    {
        p_out->b_stop  = true;
        p_out->b_fault = true;
    }
    else if ((0U != (pattern & OBS_EVT_ABORT)) || (elapsed > limit))
    {
        raise_fault(p_seq, p_out, OBS_FAULT_TIMEOUT);
    }
    else
    {
        switch (p_seq->phase)
        {
            case OBS_PHASE_STOPPING:
                p_out->b_stop = true;

                if (elapsed >= OBS_STOP_SETTLE_MSEC)
                {
                    obs_state_flags_clear(OBS_EVT_SCAN_DONE);
                    obs_state_flags_set(OBS_EVT_SCAN_REQ);
                    enter_phase(p_seq, OBS_PHASE_SCANNING, now_msec);
                }
            break;

            case OBS_PHASE_SCANNING:
                p_out->b_stop = true;

                if (0U != (pattern & OBS_EVT_SCAN_DONE))
                {
                    obs_state_flags_clear(OBS_EVT_SCAN_DONE);
                    handle_plan(p_seq, p_out, now_msec);
                }
            break;

            case OBS_PHASE_TURN_OUT:
                /* 1. Swing towards the clear side. */
                if (turn_step(p_seq, p_out))
                {
                    enter_phase(p_seq, OBS_PHASE_DIAG_OUT, now_msec);
                }
            break;

            case OBS_PHASE_DIAG_OUT:
            {
                /* 2. Drive out until far enough to the side. Turning
                 * parallel pivots on the inner wheel, which carries the
                 * robot a further half track times (1 - cos) outwards.
                 */
                float const turn_gain =
                    ((float)OBS_TRACK_WIDTH_MM / (float)HALF) *
                    (1.0F - OBS_COSF((float)heading * OBS_RAD_PER_DDEG));

                set_straight(p_out, heading, p_seq->side * p_seq->out_ddeg);

                if (guard_tripped(p_seq, false))
                {
                    /* The way out is blocked too: stop safely. */
                    raise_fault(p_seq, p_out, OBS_FAULT_NO_ROUTE);
                }
                else if ((out_y + turn_gain) >= (float)p_seq->lateral_mm)
                {
                    enter_turn(p_seq, OBS_PHASE_TURN_PARALLEL, 0, now_msec);
                }
                else
                {
                    /* Still moving out. */
                }
            }
            break;

            case OBS_PHASE_TURN_PARALLEL:
                /* 3. Turn back parallel to the line. */
                if (turn_step(p_seq, p_out))
                {
                    aim_side_watch(p_seq);
                    enter_phase(p_seq, OBS_PHASE_PASS_SIDE, now_msec);
                }
            break;

            case OBS_PHASE_PASS_SIDE:
                run_pass_side(p_seq, p_out, now_msec);
            break;

            case OBS_PHASE_PASS_END:
                /* Carry on until the body has cleared the rear, still
                 * watching the side in case the object goes on.
                 */
                set_straight(p_out, heading, offset_heading(p_seq));
                p_seq->line_hits =
                    line_near(p_seq, b_black) ? (p_seq->line_hits + 1U) : 0U;

                if (p_seq->line_hits >= OBS_LINE_CONFIRM_CYCLES)
                {
                    /* Past the object and already on the line. */
                    line_found(p_seq, now_msec);
                }
                else if (side_ping_update(p_seq))
                {
                    /* UB views the same bytes; tm_printf does not modify. */
                    tm_printf((UB *)"[OBS] Object goes on; still passing\n");
                    enter_phase(p_seq, OBS_PHASE_PASS_SIDE, now_msec);
                }
                else if (guard_tripped(p_seq, false))
                {
                    guard_stop(p_seq, p_out, now_msec);
                }
                else if (p_seq->pose.x_mm >= p_seq->end_x_mm)
                {
                    /* 4. Turn back towards the line. */
                    enter_turn(p_seq, OBS_PHASE_TURN_BACK,
                               -p_seq->side * OBS_RETURN_DEG * DDEG_PER_DEG,
                               now_msec);
                }
                else
                {
                    /* Not past the rear yet. */
                }
            break;

            case OBS_PHASE_TURN_BACK:
                p_seq->line_hits =
                    line_near(p_seq, b_black) ? (p_seq->line_hits + 1U) : 0U;

                if (p_seq->line_hits >= OBS_LINE_CONFIRM_CYCLES)
                {
                    /* Reached the line while still turning towards it. */
                    line_found(p_seq, now_msec);
                }
                else if (guard_tripped(p_seq, true))
                {
                    pass_further(p_seq, p_out, now_msec);
                }
                else if (turn_step(p_seq, p_out))
                {
                    p_seq->search_ddeg = p_seq->turn_target_ddeg;
                    enter_phase(p_seq, OBS_PHASE_SEARCH_LINE, now_msec);
                }
                else
                {
                    /* Still turning. */
                }
            break;

            case OBS_PHASE_SEARCH_LINE:
            {
                /* The line is straight, so it lies along y = 0: head back
                 * towards it at a fixed angle and expect to cross it there.
                 */
                float const near_y =
                    ((float)p_seq->search_side) * p_seq->pose.y_mm;

                set_straight(p_out, heading, p_seq->search_ddeg);
                p_seq->line_hits = b_black ? (p_seq->line_hits + 1U) : 0U;

                if ((!p_seq->b_black_seen) &&
                    (p_seq->line_hits >= OBS_LINE_CONFIRM_CYCLES))
                {
                    line_found(p_seq, now_msec);
                }
                else if (p_seq->b_black_seen)
                {
                    /* Turning onto the line pivots on the wheel away from
                     * it, which moves the axle (1 - cos) half a track
                     * further across. Start the turn that far short, so the
                     * axle ends on the line.
                     */
                    float const turn_at =
                        ((float)OBS_TRACK_WIDTH_MM / (float)HALF) *
                        (1.0F - OBS_COSF(p_seq->pose.heading_rad));

                    if (near_y <= turn_at)
                    {
                        /* The turn sweeps the IR back over the line; it
                         * must see it again for the hand-back.
                         */
                        p_seq->b_black_seen = false;
                        enter_turn(p_seq, OBS_PHASE_ALIGN, 0, now_msec);
                    }
                }
                else if (guard_tripped(p_seq, true))
                {
                    pass_further(p_seq, p_out, now_msec);
                }
                else if (near_y <= (float)(-OBS_SEARCH_MARGIN_MM))
                {
                    if (p_seq->retries < OBS_SEARCH_RETRIES)
                    {
                        /* Crossed it unseen: turn and cross back. */
                        p_seq->retries++;
                        p_seq->search_side = -p_seq->search_side;
                        /* UB views the same bytes; tm_printf keeps them. */
                        tm_printf((UB *)"[OBS] Line missed; crossing back\n");
                        enter_turn(p_seq, OBS_PHASE_TURN_BACK,
                                   -p_seq->search_side * OBS_RETURN_DEG *
                                       DDEG_PER_DEG,
                                   now_msec);
                    }
                    else
                    {
                        raise_fault(p_seq, p_out, OBS_FAULT_LINE_LOST);
                    }
                }
                else
                {
                    /* Keep closing on the line. */
                }
            }
            break;

            case OBS_PHASE_ALIGN:
                /* Turn along the line; line following finishes it. */
                p_seq->b_black_seen = p_seq->b_black_seen || b_black;

                if (turn_step(p_seq, p_out))
                {
                    p_seq->verify_cycles = 0U;
                    enter_phase(p_seq, OBS_PHASE_VERIFY, now_msec);
                }
            break;

            case OBS_PHASE_VERIFY:
                p_out->b_line_follow = true;
                p_seq->verify_cycles++;
                p_seq->b_black_seen = p_seq->b_black_seen || b_black;

                /* Another object ahead is left to the normal front monitor
                 * once line following resumes.
                 */
                if (p_seq->verify_cycles >= OBS_VERIFY_CYCLES)
                {
                    if (p_seq->b_black_seen)
                    {
                        finish_bypass(p_seq, p_out, true);
                    }
                    else
                    {
                        raise_fault(p_seq, p_out, OBS_FAULT_LINE_LOST);
                    }
                }
                else
                {
                    /* Let line following settle. */
                }
            break;

            default:
                raise_fault(p_seq, p_out, OBS_FAULT_TIMEOUT);
            break;
        }
    }
}

static void run_pass_side (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                           uint32_t now_msec)
{
    (void)side_ping_update(p_seq);
    set_straight(p_out, heading_ddeg(&p_seq->pose), offset_heading(p_seq));

    /* One stray echo is not enough to trust where the object ends. */
    bool const b_passed =
        p_seq->b_seen && (p_seq->clear_hits >= OBS_SIDE_CLEAR_HITS);
    /* Never seen from the side: trust the assumed depth. */
    bool const b_assumed =
        (!p_seq->b_seen) && (p_seq->pose.x_mm >= p_seq->rear_x_mm);

    if (b_passed || b_assumed)
    {
        /* On the way back the body side, OBS_BODY_PAST_WHEEL_MM outside the
         * pivot wheel, must stay clear of the object's rear corner:
         * past * sin(return) + gap * cos(return) >= that + margin, where
         * gap runs from the pivot wheel to the object's side.
         */
        float const ret_rad = (float)OBS_RETURN_DEG * OBS_RAD_PER_DEG;
        float const gap_mm =
            p_seq->b_seen
                ? (p_seq->beside_mm - ((float)OBS_TRACK_WIDTH_MM / (float)HALF))
                : 0.0F;
        float past_mm = (((float)(OBS_BODY_PAST_WHEEL_MM +
                                  OBS_SAFETY_MARGIN_MM)) -
                         (gap_mm * OBS_COSF(ret_rad))) /
                        OBS_SINF(ret_rad);

        if (past_mm < (float)OBS_PASS_MARGIN_MM)
        {
            past_mm = (float)OBS_PASS_MARGIN_MM;
        }

        p_seq->end_x_mm = p_seq->rear_x_mm + past_mm;

        if (b_assumed)
        {
            /* UB views the same string bytes; tm_printf does not modify. */
            tm_printf((UB *)"[OBS] Side never seen; using assumed depth\n");
        }

        /* Values fit INT on the 32-bit target; UB preserves string bytes. */
        tm_printf((UB *)"[OBS] Object ends at x=%d mm; turning back at %d mm\n",
                  (INT)p_seq->rear_x_mm, (INT)p_seq->end_x_mm);
        enter_phase(p_seq, OBS_PHASE_PASS_END, now_msec);
    }
    else if (guard_tripped(p_seq, false))
    {
        guard_stop(p_seq, p_out, now_msec);
    }
    else if (p_seq->pose.x_mm >= (p_seq->face_x_mm + (float)OBS_PASS_MAX_MM))
    {
        raise_fault(p_seq, p_out, OBS_FAULT_NO_ROUTE);
    }
    else
    {
        /* Still alongside the object. */
    }
}

static bool side_ping_update (obstacle_seq_t * p_seq)
{
    uint32_t const seq       = obs_state_side_seq_get();
    int32_t const  range_mm  = obs_state_side_range_get();
    bool           b_new_hit = false;

    if (seq != p_seq->side_seq)
    {
        float const heading = p_seq->pose.heading_rad;
        float const ray =
            heading + ((float)p_seq->look_deg * OBS_RAD_PER_DEG);
        /* Where the echo came from, in the line frame. */
        float const sensor_x =
            p_seq->pose.x_mm +
            ((float)OBS_SENSOR_TO_AXLE_MM * OBS_COSF(heading));
        float const sensor_y =
            p_seq->pose.y_mm +
            ((float)OBS_SENSOR_TO_AXLE_MM * OBS_SINF(heading));
        float const hit_x = sensor_x + ((float)range_mm * OBS_COSF(ray));
        float const hit_y = sensor_y + ((float)range_mm * OBS_SINF(ray));
        /* Gap from the robot's centre line to the echo, towards the line,
         * and how far the echo is from the line on the bypass side.
         */
        float const side    = (float)p_seq->side;
        float const gap_mm  = side * (p_seq->pose.y_mm - hit_y);
        float const from_ln = side * hit_y;
        bool const  b_hit =
            (range_mm > OBS_SIDE_MIN_MM) && (range_mm < OBS_SIDE_NEAR_MM) &&
            (from_ln >= -(float)OBS_SIDE_LINE_SLACK_MM);

        p_seq->side_seq = seq;

        if (b_hit)
        {
            /* Keep the body a full safety margin from the object. */
            if (gap_mm < (float)OBS_HALF_PATH_MM)
            {
                p_seq->lateral_mm +=
                    (int32_t)((float)OBS_HALF_PATH_MM - gap_mm);
            }

            if ((0U == p_seq->side_hits) || (hit_x > p_seq->hit_rear_mm))
            {
                p_seq->hit_rear_mm = hit_x;
            }

            if ((0U == p_seq->side_hits) || (gap_mm < p_seq->beside_mm))
            {
                p_seq->beside_mm = gap_mm;
            }

            if (p_seq->side_hits < OBS_SIDE_SEEN_HITS)
            {
                p_seq->side_hits++;
            }

            p_seq->b_seen     = (p_seq->side_hits >= OBS_SIDE_SEEN_HITS);
            p_seq->clear_hits = 0U;
            b_new_hit         = true;

            if (p_seq->b_seen)
            {
                /* Seen for sure: the echoes, not the assumed depth, now
                 * say where the object ends.
                 */
                p_seq->rear_x_mm = p_seq->hit_rear_mm;
            }
        }
        else if (p_seq->b_seen)
        {
            p_seq->clear_hits++;
        }
        else
        {
            /* Not seen enough yet: keep watching. */
        }
    }

    return b_new_hit;
}

static bool line_near (obstacle_seq_t const * p_seq, bool b_black)
{
    /* Where the IR sensors are, across the line. */
    float const ir_y =
        p_seq->pose.y_mm +
        ((float)OBS_IR_TO_AXLE_MM * OBS_SINF(p_seq->pose.heading_rad));

    return b_black && (ir_y < (float)OBS_LINE_NEAR_MM) &&
           (ir_y > -(float)OBS_LINE_NEAR_MM);
}

static void pass_further (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                          uint32_t now_msec)
{
    float const ahead_x =
        p_seq->pose.x_mm +
        ((float)(OBS_SENSOR_TO_AXLE_MM + obs_state_front_range_get()) *
         OBS_COSF(p_seq->pose.heading_rad));

    p_seq->replans++;

    if (p_seq->replans > OBS_REPLAN_MAX)
    {
        raise_fault(p_seq, p_out, OBS_FAULT_NO_ROUTE);
    }
    else
    {
        /* Usually the same object, longer than it looked; the side watch
         * finds its real end on the next pass.
         */
        p_seq->rear_x_mm = (ahead_x > p_seq->rear_x_mm) ? ahead_x
                                                        : p_seq->rear_x_mm;
        p_seq->b_seen    = false;
        p_seq->side_hits = 0U;
        /* Values fit INT on the target; UB preserves the string bytes. */
        tm_printf((UB *)"[OBS] Still in the way at x=%d mm on the way back; "
                        "passing further\n",
                  (INT)ahead_x);
        enter_turn(p_seq, OBS_PHASE_TURN_PARALLEL, 0, now_msec);
    }
}

static void aim_side_watch (obstacle_seq_t * p_seq)
{
    /* The object is on the side opposite the bypass. */
    int32_t const object_side = -p_seq->side;
    int32_t const reach =
        (object_side > 0) ? servo_left_limit_deg() : servo_right_limit_deg();
    int32_t const look =
        (reach < OBS_SIDE_LOOK_DEG) ? reach : OBS_SIDE_LOOK_DEG;

    p_seq->look_deg   = object_side * look;
    p_seq->clear_hits = 0U;
    p_seq->side_seq   = obs_state_side_seq_get();
    obs_state_watch_angle_set(p_seq->look_deg);
}

static int32_t offset_heading (obstacle_seq_t const * p_seq)
{
    float const out_y = ((float)p_seq->side) * p_seq->pose.y_mm;
    int32_t     turn  = (int32_t)(((float)p_seq->lateral_mm - out_y) *
                                  (float)OBS_OFFSET_GAIN_DDEG_PER_MM);

    if (turn > OBS_OFFSET_OUT_MAX_DDEG)
    {
        turn = OBS_OFFSET_OUT_MAX_DDEG;
    }

    if (turn < -OBS_OFFSET_IN_MAX_DDEG)
    {
        turn = -OBS_OFFSET_IN_MAX_DDEG;
    }

    return p_seq->side * turn;
}

static void line_found (obstacle_seq_t * p_seq, uint32_t now_msec)
{
    /* The IR sensors sit on y = 0, well ahead of the axle. */
    p_seq->b_black_seen = true;
    p_seq->pose.y_mm =
        -(float)OBS_IR_TO_AXLE_MM * OBS_SINF(p_seq->pose.heading_rad);
    if (p_seq->pose.y_mm < 0.0F)
    {
        p_seq->search_side = -1;
    }
    else if (p_seq->pose.y_mm > 0.0F)
    {
        p_seq->search_side = 1;
    }
    else
    {
        /* Exactly on it: keep the side the robot came from. */
    }
    /* Carry straight on at this heading to the start of the turn. */
    p_seq->search_ddeg = heading_ddeg(&p_seq->pose);
    /* Values fit INT on the target; UB preserves the string bytes. */
    tm_printf((UB *)"[OBS] Line found; axle %d mm from it\n",
              (INT)(((float)p_seq->search_side) * p_seq->pose.y_mm));

    if (OBS_PHASE_SEARCH_LINE != p_seq->phase)
    {
        enter_phase(p_seq, OBS_PHASE_SEARCH_LINE, now_msec);
    }
}

static bool guard_tripped (obstacle_seq_t * p_seq, bool b_to_line)
{
    uint32_t const seq = obs_state_front_seq_get();

    if (seq != p_seq->front_seq)
    {
        int32_t const range_mm = obs_state_front_range_get();
        bool          b_near   = (range_mm > 0) && (range_mm < OBS_GUARD_MM);

        p_seq->front_seq = seq;

        if (b_near && b_to_line)
        {
            /* Where the echo came from, measured towards the side the
             * robot comes from.
             */
            float const hit_y =
                p_seq->pose.y_mm +
                ((float)(OBS_SENSOR_TO_AXLE_MM + range_mm) *
                 OBS_SINF(p_seq->pose.heading_rad));

            b_near = (((float)p_seq->search_side) * hit_y) >
                     (float)(-OBS_GUARD_BEYOND_LINE_MM);
        }

        p_seq->guard_hits = b_near ? (p_seq->guard_hits + 1U) : 0U;
    }

    return p_seq->guard_hits >= OBS_GUARD_HITS;
}

static void guard_stop (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                        uint32_t now_msec)
{
    p_out->left_rpm  = 0;
    p_out->right_rpm = 0;
    p_out->b_stop    = true;
    p_seq->replans++;

    if (p_seq->replans > OBS_REPLAN_MAX)
    {
        raise_fault(p_seq, p_out, OBS_FAULT_NO_ROUTE);
    }
    else
    {
        /* Values fit INT on the 32-bit target; UB preserves string bytes. */
        tm_printf((UB *)"[OBS] Something ahead at %d mm; stopping to look\n",
                  (INT)obs_state_front_range_get());
        p_seq->resume_phase = p_seq->phase;
        enter_phase(p_seq, OBS_PHASE_STOPPING, now_msec);
    }
}

static bool counts_read (int32_t * p_left, int32_t * p_right)
{
    motion_status_t status = {0};
    bool const b_ok = (motion_get_status(&status) >= E_OK) && status.b_valid;

    if (b_ok)
    {
        *p_left  = status.left_count;
        *p_right = status.right_count;
    }

    return b_ok;
}

static void pose_update (obstacle_seq_t * p_seq, int32_t left_count,
                         int32_t right_count)
{
    int32_t const left_delta  = left_count - p_seq->last_left;
    int32_t const right_delta = right_count - p_seq->last_right;
    float const   left_mm =
        ((float)((left_delta < 0) ? -left_delta : left_delta) * OBS_CIRC_MM) /
        OBS_COUNTS_PER_REV;
    float const right_mm =
        ((float)((right_delta < 0) ? -right_delta : right_delta) *
         OBS_CIRC_MM) /
        OBS_COUNTS_PER_REV;
    float const travel = (left_mm + right_mm) / (float)HALF;
    float const turn   = (right_mm - left_mm) / (float)OBS_TRACK_WIDTH_MM;
    /* Integrate along the mean heading of the step. */
    float const mid = p_seq->pose.heading_rad + (turn / (float)HALF);

    p_seq->last_left  = left_count;
    p_seq->last_right = right_count;
    p_seq->pose.x_mm += travel * OBS_COSF(mid);
    p_seq->pose.y_mm += travel * OBS_SINF(mid);
    p_seq->pose.heading_rad += turn;
}

static int32_t heading_ddeg (obstacle_pose_t const * p_pose)
{
    /* Headings stay within a few turns, so the conversion cannot overflow. */
    return (int32_t)(p_pose->heading_rad * OBS_DDEG_PER_RAD);
}

static void seq_begin (obstacle_seq_t * p_seq, int32_t left_count,
                       int32_t right_count, uint32_t now_msec)
{
    p_seq->b_active         = true;
    p_seq->last_left        = left_count;
    p_seq->last_right       = right_count;
    p_seq->pose.x_mm        = 0.0F;
    p_seq->pose.y_mm        = 0.0F;
    p_seq->pose.heading_rad = 0.0F;
    p_seq->b_planned        = false;
    p_seq->side             = 1;
    p_seq->out_ddeg         = 0;
    p_seq->lateral_mm       = 0;
    p_seq->look_deg         = 0;
    p_seq->turn_target_ddeg = 0;
    p_seq->turn_dir         = 1;
    p_seq->face_x_mm        = 0.0F;
    p_seq->rear_x_mm        = 0.0F;
    p_seq->end_x_mm         = 0.0F;
    p_seq->beside_mm        = 0.0F;
    p_seq->b_seen           = false;
    p_seq->side_hits        = 0U;
    p_seq->hit_rear_mm      = 0.0F;
    p_seq->clear_hits       = 0U;
    p_seq->side_seq         = 0U;
    p_seq->front_seq        = 0U;
    p_seq->guard_hits       = 0U;
    p_seq->resume_phase     = OBS_PHASE_IDLE;
    p_seq->replans          = 0U;
    p_seq->search_side      = 1;
    p_seq->search_ddeg      = 0;
    p_seq->retries          = 0U;
    p_seq->line_hits        = 0U;
    p_seq->verify_cycles    = 0U;
    p_seq->b_black_seen     = false;
    enter_phase(p_seq, OBS_PHASE_STOPPING, now_msec);
}

static void enter_phase (obstacle_seq_t * p_seq, obs_state_phase_t phase,
                         uint32_t now_msec)
{
    bool const b_side = (OBS_PHASE_PASS_SIDE == phase) ||
                        (OBS_PHASE_PASS_END == phase);
    bool const b_front = b_side || (OBS_PHASE_DIAG_OUT == phase) ||
                         (OBS_PHASE_TURN_BACK == phase) ||
                         (OBS_PHASE_SEARCH_LINE == phase);

    p_seq->phase      = phase;
    p_seq->enter_msec = now_msec;
    p_seq->line_hits  = 0U;

    /* Pings taken before this phase are stale for its guard. */
    p_seq->guard_hits = 0U;
    p_seq->front_seq  = obs_state_front_seq_get();

    if (b_side)
    {
        obs_state_flags_set(OBS_EVT_SIDE_WATCH);
    }
    else
    {
        obs_state_flags_clear(OBS_EVT_SIDE_WATCH);
    }

    if (b_front)
    {
        obs_state_flags_set(OBS_EVT_FRONT_WATCH);
    }
    else
    {
        obs_state_flags_clear(OBS_EVT_FRONT_WATCH);
    }

    obs_state_set_phase(phase);
    /* Pose truncated to whole mm and ddeg for display; UB keeps the bytes. */
    tm_printf((UB *)"[OBS] Phase %d at x=%d y=%d heading=%d ddeg\n",
              (INT)phase, (INT)p_seq->pose.x_mm, (INT)p_seq->pose.y_mm,
              (INT)heading_ddeg(&p_seq->pose));
}

static void enter_turn (obstacle_seq_t * p_seq, obs_state_phase_t phase,
                        int32_t target_ddeg, uint32_t now_msec)
{
    p_seq->turn_target_ddeg = target_ddeg;
    p_seq->turn_dir =
        (target_ddeg >= heading_ddeg(&p_seq->pose)) ? 1 : -1;
    enter_phase(p_seq, phase, now_msec);
}

static bool turn_step (obstacle_seq_t const * p_seq, obstacle_drive_t * p_out)
{
    /* Ends a little early: the robot keeps turning while it slows. */
    int32_t const remaining =
        p_seq->turn_dir *
        (p_seq->turn_target_ddeg - heading_ddeg(&p_seq->pose));
    bool const b_done = remaining <= OBS_TURN_LEAD_DDEG;

    if (!b_done)
    {
        set_arc(p_out, p_seq->turn_dir);
    }

    return b_done;
}

static void set_arc (obstacle_drive_t * p_out, int32_t turn)
{
    p_out->left_rpm  = (turn > 0) ? 0 : OBS_DRIVE_RPM;
    p_out->right_rpm = (turn > 0) ? OBS_DRIVE_RPM : 0;
}

static void set_straight (obstacle_drive_t * p_out, int32_t heading,
                          int32_t target_ddeg)
{
    int32_t trim = (target_ddeg - heading) / OBS_HEADING_TRIM_DIV;

    if (trim > OBS_HEADING_TRIM_MAX)
    {
        trim = OBS_HEADING_TRIM_MAX;
    }

    if (trim < -OBS_HEADING_TRIM_MAX)
    {
        trim = -OBS_HEADING_TRIM_MAX;
    }

    p_out->left_rpm  = OBS_DRIVE_RPM - trim;
    p_out->right_rpm = OBS_DRIVE_RPM + trim;
}

static void raise_fault (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                         uint32_t fault_bit)
{
    p_seq->phase = OBS_PHASE_FAULT;
    obs_state_faults_update(fault_bit, 0U);
    obs_state_set_phase(OBS_PHASE_FAULT);
    obs_state_flags_set(OBS_EVT_ABORT);
    p_out->b_stop  = true;
    p_out->b_fault = true;
    /* Values fit INT on the 32-bit target; UB preserves string bytes. */
    tm_printf((UB *)"[OBS] FAULT 0x%x at x=%d y=%d heading=%d ddeg\n",
              (UINT)fault_bit, (INT)p_seq->pose.x_mm, (INT)p_seq->pose.y_mm,
              (INT)heading_ddeg(&p_seq->pose));
}

static void finish_bypass (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                           bool b_count)
{
    obs_state_flags_clear(OBS_EVT_BLOCKED | OBS_EVT_SCAN_REQ |
                          OBS_EVT_SCAN_DONE | OBS_EVT_SIDE_WATCH |
                          OBS_EVT_FRONT_WATCH);

    if (b_count)
    {
        obs_state_count_bypass();
    }

    obs_state_set_route(OBS_ROUTE_NONE);
    obs_state_set_phase(OBS_PHASE_IDLE);
    p_seq->b_active = false;
    p_out->b_done   = true;
}

static void handle_plan (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                         uint32_t now_msec)
{
    scan_plan_bypass_t plan = {0U, 0U};

    obs_state_plan_load(&plan);

    uint32_t const result = scan_plan_result(&plan);
    int32_t const  plan_side = scan_plan_side_is_left(&plan) ? 1 : -1;

    if (SCAN_PLAN_CLEAR == result)
    {
        if (!p_seq->b_planned)
        {
            /* Nothing there after all: back to line following. */
            finish_bypass(p_seq, p_out, false);
        }
        else
        {
            /* UB views the same string bytes; tm_printf does not modify. */
            tm_printf((UB *)"[OBS] Way is clear; carrying on\n");

            if (OBS_PHASE_PASS_SIDE == p_seq->resume_phase)
            {
                aim_side_watch(p_seq);
            }

            enter_phase(p_seq, p_seq->resume_phase, now_msec);
        }
    }
    else if ((SCAN_PLAN_ROUTE != result) ||
             (p_seq->b_planned && (plan_side != p_seq->side)))
    {
        /* Both sides blocked, or mid-bypass only the other side is clear,
         * which would mean crossing back over the object: stop safely.
         */
        raise_fault(p_seq, p_out, OBS_FAULT_NO_ROUTE);
    }
    else
    {
        int32_t const near_mm = scan_plan_near_cm(&plan) * MM_PER_CM;
        float const   face_x =
            p_seq->pose.x_mm +
            ((float)(OBS_SENSOR_TO_AXLE_MM + near_mm) *
             OBS_COSF(p_seq->pose.heading_rad));
        float const rear_x = face_x + (float)OBS_DEFAULT_DEPTH_MM;

        p_seq->b_planned = true;
        p_seq->side      = plan_side;
        p_seq->out_ddeg =
            scan_plan_angle_deg(scan_plan_angle_index(&plan)) * DDEG_PER_DEG;
        /* The plan's offset is from where the robot is now, which is on the
         * line at first and beside it after a stop to look again.
         */
        p_seq->lateral_mm =
            (int32_t)(((float)p_seq->side) * p_seq->pose.y_mm) +
            (scan_plan_lateral_cm(&plan) * MM_PER_CM);
        p_seq->face_x_mm   = face_x;
        p_seq->rear_x_mm   = (rear_x > p_seq->rear_x_mm) ? rear_x
                                                         : p_seq->rear_x_mm;
        p_seq->b_seen      = false;
        p_seq->side_hits   = 0U;
        p_seq->search_side = p_seq->side;
        obs_state_set_route((p_seq->side > 0) ? OBS_ROUTE_LEFT
                                              : OBS_ROUTE_RIGHT);
        obs_state_set_width_cm(scan_plan_width_cm(&plan));
        /* Values fit INT on the 32-bit target; UB preserves string bytes. */
        tm_printf((UB *)"[OBS] Bypass %s: out %d deg, offset %d mm, face at "
                        "%d mm\n",
                  (p_seq->side > 0) ? (UB *)"left" : (UB *)"right",
                  (INT)(p_seq->out_ddeg / DDEG_PER_DEG),
                  (INT)p_seq->lateral_mm, (INT)p_seq->face_x_mm);
        enter_turn(p_seq, OBS_PHASE_TURN_OUT, p_seq->side * p_seq->out_ddeg,
                   now_msec);
    }
}

/*** end of file ***/

