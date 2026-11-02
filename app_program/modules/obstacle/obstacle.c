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
 *
 * Side readings are used only when they were taken at the bearing asked
 * for, after it was asked for; each is placed in the line frame from where
 * the servo really holds the transducer. A reading is a hit on the object,
 * a clear view past where the object would be, or unknown. A missing echo
 * is unknown and never counts as clear. The object has ended only after it
 * was seen, then seen clear several times over enough axle travel. The car
 * turns back once its rear is past the object's estimated rear and a fresh
 * look down the way back, together with a swept-body check of the whole
 * return, finds nothing in the way; otherwise it keeps passing and looks
 * again after more travel.
 */
#include "obstacle.h"
#include "obs_config.h"
#include "obs_geometry.h"
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
#define CLOSE_HITS_LIMIT 2U
#define TURN_STEP_DEG 5
#define RUN_STEP_MM 20
#define RUN_LIMIT_MM 1500
#define REGION_REACH_MM 3000
/* Wheel circumference in mm; folded at compile time. */
#define OBS_CIRC_MM (BOARD_WHEEL_DIAMETER_MM * 3.14159265F)
#define OBS_COUNTS_PER_REV ((float)BOARD_ENC_COUNTS_PER_REV)
#define OBS_DDEG_PER_RAD 572.958F
#define OBS_DEG_PER_RAD 57.2958F
#define OBS_RAD_PER_DEG 0.0174533F
#define OBS_RAD_PER_DDEG 0.00174533F
/* <math.h> clashes with the kernel's size_t, so use the compiler's names
 * for the libm functions directly; -lm is already linked.
 */
#define OBS_COSF(x) __builtin_cosf(x)
#define OBS_SINF(x) __builtin_sinf(x)
/* Gap from the robot's centre line that keeps the body its margin, with
 * the range error allowed for. The side gap is measured directly, so no
 * position allowance is needed on top.
 */
#define OBS_HALF_PATH_MM                                                       \
    ((OBS_ROBOT_WIDTH_MM / HALF) + OBS_SAFETY_MARGIN_MM + OBS_RANGE_UNCERT_MM)
/* Offset the scan's plan adds beyond the object edge. */
#define OBS_PLAN_PATH_MM                                                       \
    ((OBS_ROBOT_WIDTH_MM / HALF) + OBS_SAFETY_MARGIN_MM + OBS_OUTWARD_EXTRA_MM)
/* Axle travel past the object's rear before the rear of the car is past. */
#define OBS_TURN_PAST_MM (OBS_ROBOT_REAR_MM + OBS_REAR_PASS_MARGIN_MM)

/** @brief Dead-reckoned pose: x along the line, y to its left, both from
 *  where the obstacle was confirmed; heading positive to the left.
 */
typedef struct
{
    float x_mm;
    float y_mm;
    float heading_rad;
} obstacle_pose_t;

/** @brief What one side reading says about the space beside the car. */
typedef enum
{
    SIDE_UNKNOWN = 0, /**< No echo, own body or an odd geometry. */
    SIDE_HIT,         /**< Something between the car and the line. */
    SIDE_CLEAR        /**< The view past the object's band was clear. */
} side_view_t;

/** @brief Where a side echo came from, in the line frame. */
typedef struct
{
    float x_mm;
    float y_mm;
    float gap_mm;  /**< From the car's centre line, towards the line. */
    bool  b_point; /**< A real echo, so x and y are a surface. */
} side_hit_t;

typedef struct
{
    bool              b_active;
    obs_state_phase_t phase;
    uint32_t          enter_msec;
    uint32_t          now_msec; /**< Time of the current step. */
    int32_t           last_left;
    int32_t           last_right;
    obstacle_pose_t   pose;
    bool              b_planned; /**< A side has been chosen. */
    int32_t           side; /**< 1 bypasses on the left, -1 on the right. */
    int32_t           out_ddeg;
    int32_t           lateral_mm; /**< Offset from the line while passing. */
    int32_t           lateral_max_mm; /**< Most the offset may grow to. */
    int32_t           look_deg;   /**< Servo bearing asked for. */
    uint32_t          look_msec;  /**< When that bearing was asked for. */
    int32_t           turn_target_ddeg;
    int32_t           turn_dir; /**< Arc direction of the current turn. */
    float             face_x_mm; /**< Front face of the object. */
    float             face_y_mm; /**< Object side nearest the path. */
    float             rear_x_mm; /**< Rear as far as evidence shows it. */
    float             rear_guess_mm; /**< Assumed depth, until disproved. */
    float             clear_axis_mm; /**< Furthest face point seen clear. */
    float             recheck_x_mm; /**< Pass at least this far first. */
    float             end_x_mm;  /**< Where the axle may turn back in. */
    float             hit_rear_mm; /**< Rear estimate from side echoes. */
    float             hit_axle_mm; /**< Axle x at the last side hit. */
    float             front_axle_mm; /**< Axle x at the last front ping. */
    float             probe_axle_mm; /**< Axle x at the last clear probe. */
    float             probe_x_mm; /**< Echo seen by the return probe. */
    float             probe_y_mm;
    bool              b_seen;     /**< The side watch has seen the object. */
    bool              b_clear_axis; /**< clear_axis_mm holds a value. */
    bool              b_too_close; /**< Needed offset beyond what is safe. */
    bool              b_front_clear; /**< The last front ping saw no block. */
    bool              b_probe;    /**< Looking down the way back. */
    bool              b_probe_clear; /**< The last probe found it clear. */
    bool              b_probe_point; /**< The last probe saw a surface. */
    bool              b_waiting;  /**< Stopped for a reading. */
    uint32_t          wait_msec;  /**< When that wait began. */
    uint32_t          side_hits;  /**< Side pings that saw the object. */
    uint32_t          clear_hits; /**< Clear side pings since last seen. */
    uint32_t          no_echo;    /**< Side pings in a row with no echo. */
    uint32_t          front_no_echo; /**< Front pings in a row, no echo. */
    uint8_t           side_seq;   /**< Last side ping handled. */
    uint8_t           front_seq;  /**< Last front ping handled. */
    uint8_t           outward_hits : 2; /**< 0..2 outward hits. */
    uint8_t           close_hits : 2; /**< 0..2 close-side hits. */
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
/** @brief Drive on past the rear and turn back once the way is clear. */
static void run_pass_end (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                          uint32_t now_msec);
/** @brief Take a new side ping, if it was taken at the bearing asked for,
 *  after it was asked for. Older or misaimed pings are dropped.
 */
static bool side_reading_take (obstacle_seq_t *      p_seq,
                               obs_state_reading_t * p_reading);
/** @brief Say what a side ping shows and where its echo came from. */
static side_view_t side_classify (obstacle_seq_t const *      p_seq,
                                  obs_state_reading_t const * p_reading,
                                  side_hit_t *                p_hit);
/** @brief Take in a side ping while passing; returns what it showed. */
static side_view_t side_update (obstacle_seq_t *            p_seq,
                                obs_state_reading_t const * p_reading);
/** @brief Record a confirmed or candidate hit on the object's side. */
static void side_hit_record (obstacle_seq_t * p_seq, side_hit_t const * p_hit);
/** @brief Take in a ping from the return probe.
 *  @return True if it shows the object still beside the car.
 */
static bool probe_update (obstacle_seq_t *            p_seq,
                          obs_state_reading_t const * p_reading);
/** @brief Ask the servo for a bearing, clamped to its reach. */
static void set_look (obstacle_seq_t * p_seq, int32_t angle_deg);
/** @brief Point the servo at the object for the side watch. */
static void aim_side_watch (obstacle_seq_t * p_seq);
/** @brief Point the servo down the way back for the return probe. */
static void aim_return_probe (obstacle_seq_t * p_seq);
/** @brief Stop a departure if new outward-side pings find an obstruction. */
static bool outward_blocked (obstacle_seq_t * p_seq);
/** @brief Evidence that the side echo passed the object's rear. */
static bool rear_clear (obstacle_seq_t const * p_seq);
/** @brief True if the fresh probe and the swept body allow the turn now. */
static bool return_turn_safe (obstacle_seq_t const * p_seq);
/** @brief Swept-body check of the pivot back and the run to the line. */
static bool return_path_clear (obstacle_seq_t const * p_seq);
/** @brief Note how far along the object's side a clear ping looked. */
static void clear_axis_note (obstacle_seq_t *            p_seq,
                             obs_state_reading_t const * p_reading);
/** @brief The rear to pass: the evidence, or the assumed depth while no
 *  clear view of the side has gone beyond it.
 */
static float effective_rear (obstacle_seq_t const * p_seq);
/** @brief Where the axle may turn back in. */
static float turn_point (obstacle_seq_t const * p_seq);
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
/** @brief True if a recent forward ping found the way ahead clear. */
static bool front_fresh (obstacle_seq_t const * p_seq);
/** @brief True once side or front pings have gone silent too long. */
static bool sensor_lost (obstacle_seq_t const * p_seq);
/** @brief Stop until a reading arrives; fault if it never does. */
static void wait_for_reading (obstacle_seq_t * p_seq, obstacle_drive_t * p_out);
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
/** @brief The integer pose nearest the float pose. */
static void pose_round (obstacle_pose_t const * p_pose,
                        obs_geometry_pose_t *   p_out);
/** @brief Nearest whole number. */
static int32_t round_int (float value);

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

        p_seq->now_msec = now_msec;
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
                    /* Monitor both the departure side and the front
                     * rather than trusting only the stationary scan.
                     */
                    set_look(p_seq, p_seq->side * OBS_SIDE_LOOK_DEG);
                    p_seq->outward_hits = 0U;
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

                /* Both watches take in their pings every period. */
                bool const b_outward = outward_blocked(p_seq);
                bool const b_ahead   = guard_tripped(p_seq, false);

                set_straight(p_out, heading, p_seq->side * p_seq->out_ddeg);

                if (sensor_lost(p_seq))
                {
                    raise_fault(p_seq, p_out, OBS_FAULT_NO_ECHO);
                }
                else if (b_ahead || b_outward)
                {
                    /* A new obstacle on the outward side needs another
                     * stationary decision before any more travel.
                     */
                    guard_stop(p_seq, p_out, now_msec);
                }
                else if ((out_y + turn_gain) >= (float)p_seq->lateral_mm)
                {
                    /* The side watch starts during the turn, so a short
                     * object is seen before the car is past it.
                     */
                    aim_side_watch(p_seq);
                    enter_turn(p_seq, OBS_PHASE_TURN_PARALLEL, 0, now_msec);
                }
                else
                {
                    /* Still moving out. */
                }
            }
            break;

            case OBS_PHASE_TURN_PARALLEL:
            {
                /* 3. Turn back parallel to the line. */
                obs_state_reading_t reading;

                if (side_reading_take(p_seq, &reading))
                {
                    (void)side_update(p_seq, &reading);
                }

                if (turn_step(p_seq, p_out))
                {
                    enter_phase(p_seq, OBS_PHASE_PASS_SIDE, now_msec);
                }
            }
            break;

            case OBS_PHASE_PASS_SIDE:
                run_pass_side(p_seq, p_out, now_msec);
            break;

            case OBS_PHASE_PASS_END:
                run_pass_end(p_seq, p_out, now_msec);
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
                else if (sensor_lost(p_seq))
                {
                    raise_fault(p_seq, p_out, OBS_FAULT_NO_ECHO);
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
                /* Black away from where the line should be is a crossing
                 * line, a mark or a barcode bar, not the original line.
                 */
                p_seq->line_hits =
                    line_near(p_seq, b_black) ? (p_seq->line_hits + 1U) : 0U;

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
                else if (sensor_lost(p_seq))
                {
                    raise_fault(p_seq, p_out, OBS_FAULT_NO_ECHO);
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
                p_seq->b_black_seen =
                    p_seq->b_black_seen || line_near(p_seq, b_black);

                if (turn_step(p_seq, p_out))
                {
                    p_seq->verify_cycles = 0U;
                    enter_phase(p_seq, OBS_PHASE_VERIFY, now_msec);
                }
            break;

            case OBS_PHASE_VERIFY:
                p_out->b_line_follow = true;
                p_seq->verify_cycles++;
                p_seq->b_black_seen =
                    p_seq->b_black_seen || line_near(p_seq, b_black);

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
    obs_state_reading_t reading;

    if (side_reading_take(p_seq, &reading))
    {
        (void)side_update(p_seq, &reading);
    }

    set_straight(p_out, heading_ddeg(&p_seq->pose), offset_heading(p_seq));

    if (sensor_lost(p_seq))
    {
        raise_fault(p_seq, p_out, OBS_FAULT_NO_ECHO);
    }
    else if ((p_seq->close_hits >= CLOSE_HITS_LIMIT) || p_seq->b_too_close)
    {
        /* UB views the same bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[OBS] Body-side clearance too small; "
                        "rechecking route\n");
        p_seq->b_too_close = false;
        p_seq->close_hits  = 0U;
        guard_stop(p_seq, p_out, now_msec);
    }
    else if (rear_clear(p_seq))
    {
        /* One stray echo is not enough to trust where the object ends. */
        p_seq->end_x_mm      = turn_point(p_seq);
        p_seq->b_probe       = false;
        p_seq->b_probe_clear = false;
        /* Values fit INT on the 32-bit target; UB preserves string bytes. */
        tm_printf((UB *)"[OBS] Object ends at x=%d mm; turning back at %d mm\n",
                  (INT)effective_rear(p_seq), (INT)p_seq->end_x_mm);
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
    else if (!front_fresh(p_seq))
    {
        /* Only drive on while a recent forward ping says it is clear. */
        wait_for_reading(p_seq, p_out);
    }
    else
    {
        /* Still alongside the object. */
        p_seq->b_waiting = false;
    }
}

static void run_pass_end (obstacle_seq_t * p_seq, obstacle_drive_t * p_out,
                          uint32_t now_msec)
{
    obs_state_reading_t reading;
    bool                b_goes_on = false;
    float const         x_mm      = p_seq->pose.x_mm;

    set_straight(p_out, heading_ddeg(&p_seq->pose), offset_heading(p_seq));

    if (side_reading_take(p_seq, &reading))
    {
        if (p_seq->b_probe)
        {
            b_goes_on = probe_update(p_seq, &reading);
        }
        else
        {
            b_goes_on = (SIDE_HIT == side_update(p_seq, &reading));
        }
    }

    /* New evidence can move the turn point either way: a hit pushes it on,
     * a clear view past the assumed depth brings it back to the evidence.
     */
    p_seq->end_x_mm = turn_point(p_seq);

    if (b_goes_on)
    {
        /* UB views the same bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[OBS] Object goes on; still passing\n");
        aim_side_watch(p_seq);
        enter_phase(p_seq, OBS_PHASE_PASS_SIDE, now_msec);
    }
    else if (sensor_lost(p_seq))
    {
        raise_fault(p_seq, p_out, OBS_FAULT_NO_ECHO);
    }
    else if (guard_tripped(p_seq, false))
    {
        guard_stop(p_seq, p_out, now_msec);
    }
    else if ((!p_seq->b_probe) &&
             (x_mm >= (p_seq->end_x_mm - (float)OBS_RETURN_PROBE_LEAD_MM)))
    {
        /* Look down the way back just before the turn point, so a fresh
         * reading is ready on arrival.
         */
        aim_return_probe(p_seq);
    }
    else if (x_mm >= p_seq->end_x_mm)
    {
        if (return_turn_safe(p_seq))
        {
            p_seq->b_waiting = false;
            /* UB views the same bytes; tm_printf does not modify them. */
            tm_printf((UB *)"[OBS] Way back clear at x=%d mm; return\n",
                      (INT)x_mm);
            enter_turn(p_seq, OBS_PHASE_TURN_BACK,
                       -p_seq->side * OBS_RETURN_DEG * DDEG_PER_DEG,
                       now_msec);
        }
        else if (p_seq->b_probe_clear)
        {
            /* The probe saw nothing in band, but the swept body does not
             * clear yet: pass on a little and look again.
             */
            p_seq->recheck_x_mm = x_mm + (float)OBS_RETURN_RECHECK_MM;
            p_seq->end_x_mm     = turn_point(p_seq);
            aim_side_watch(p_seq);
            p_seq->b_probe       = false;
            p_seq->b_probe_clear = false;
        }
        else
        {
            /* No fresh verdict yet: wait for the probe, never guess. */
            wait_for_reading(p_seq, p_out);
        }
    }
    else if (x_mm >= (p_seq->face_x_mm + (float)OBS_PASS_MAX_MM))
    {
        raise_fault(p_seq, p_out, OBS_FAULT_NO_ROUTE);
    }
    else if (!front_fresh(p_seq))
    {
        wait_for_reading(p_seq, p_out);
    }
    else
    {
        /* Not at the turn point yet. */
        p_seq->b_waiting = false;
    }
}

static bool side_reading_take (obstacle_seq_t *      p_seq,
                               obs_state_reading_t * p_reading)
{
    bool b_take = false;

    obs_state_side_reading_get(p_reading);

    if (p_reading->seq != p_seq->side_seq)
    {
        /* Modulo-2^32 difference; a reading before the request is
         * negative.
         */
        int32_t const age = (int32_t)(p_reading->time_msec - p_seq->look_msec);

        p_seq->side_seq = p_reading->seq;
        b_take = ((int32_t)p_reading->bearing_deg == p_seq->look_deg) &&
                 (age >= 0);
    }

    return b_take;
}

static side_view_t side_classify (obstacle_seq_t const *      p_seq,
                                  obs_state_reading_t const * p_reading,
                                  side_hit_t *                p_hit)
{
    side_view_t view = SIDE_UNKNOWN;

    p_hit->b_point = false;

    if (OBS_ECHO_BEYOND == p_reading->echo)
    {
        /* Nothing came back in range along the whole ray. */
        view = SIDE_CLEAR;
    }
    else if ((OBS_ECHO_VALID == p_reading->echo) &&
             (p_reading->range_mm > OBS_SIDE_MIN_MM))
    {
        int32_t origin_fwd = 0;
        int32_t origin_lat = 0;
        float const heading = p_seq->pose.heading_rad;
        float const ray =
            heading + ((float)p_reading->bearing_deg * OBS_RAD_PER_DEG);
        float const range = (float)p_reading->range_mm;

        /* The range starts where the servo holds the transducer. */
        obs_geometry_sensor_origin(p_reading->bearing_deg, &origin_fwd,
                                   &origin_lat);

        float const sensor_x =
            p_seq->pose.x_mm + ((float)origin_fwd * OBS_COSF(heading)) -
            ((float)origin_lat * OBS_SINF(heading));
        float const sensor_y =
            p_seq->pose.y_mm + ((float)origin_fwd * OBS_SINF(heading)) +
            ((float)origin_lat * OBS_COSF(heading));
        float const side = (float)p_seq->side;

        p_hit->x_mm    = sensor_x + (range * OBS_COSF(ray));
        p_hit->y_mm    = sensor_y + (range * OBS_SINF(ray));
        p_hit->gap_mm  = side * (p_seq->pose.y_mm - p_hit->y_mm);
        p_hit->b_point = true;

        /* How far the echo is from the line on the bypass side, and how
         * far the car is.
         */
        float const from_line = side * p_hit->y_mm;
        float const offset    = side * p_seq->pose.y_mm;
        bool const  b_in_band =
            (p_hit->gap_mm > 0.0F) &&
            (from_line >= -(float)OBS_SIDE_LINE_SLACK_MM) &&
            (from_line <= (offset + (float)OBS_SIDE_LINE_SLACK_MM));

        if ((p_reading->range_mm < OBS_SIDE_NEAR_MM) && b_in_band)
        {
            view = SIDE_HIT;
        }
        else if ((p_reading->range_mm >= OBS_SIDE_NEAR_MM) ||
                 (from_line < -(float)OBS_SIDE_LINE_SLACK_MM))
        {
            /* The ray crossed the object's band without an echo. */
            view = SIDE_CLEAR;
        }
        else
        {
            /* An echo outside the band in an odd place proves nothing. */
        }
    }
    else
    {
        /* No echo pulse, or the echo is the car's own body. */
    }

    return view;
}

static side_view_t side_update (obstacle_seq_t *            p_seq,
                                obs_state_reading_t const * p_reading)
{
    side_hit_t        hit  = {0.0F, 0.0F, 0.0F, false};
    side_view_t const view = side_classify(p_seq, p_reading, &hit);

    if (OBS_ECHO_NONE == p_reading->echo)
    {
        p_seq->no_echo++;
    }
    else
    {
        p_seq->no_echo = 0U;
    }

    if (SIDE_HIT == view)
    {
        side_hit_record(p_seq, &hit);
    }
    else if (SIDE_CLEAR == view)
    {
        if (p_seq->b_seen)
        {
            p_seq->close_hits = 0U;
            clear_axis_note(p_seq, p_reading);

            if (p_seq->clear_hits < OBS_SIDE_CLEAR_HITS)
            {
                p_seq->clear_hits++;
            }
        }
        else
        {
            /* Require consecutive evidence before declaring a side hit. */
            p_seq->side_hits = 0U;
        }
    }
    else
    {
        /* Unknown is not proof that the object ended, nor that it is
         * there: it neither adds to the evidence nor wipes it. Lost echoes
         * are counted above and become a sensor fault if they persist.
         */
    }

    return view;
}

static void side_hit_record (obstacle_seq_t * p_seq, side_hit_t const * p_hit)
{
    float const side       = (float)p_seq->side;
    float const offset     = side * p_seq->pose.y_mm;
    bool const  b_was_seen = p_seq->b_seen;
    /* The beam-axis point. The last echo from a side face comes from the
     * beam's rearmost ray, which meets the face behind the axis, so the
     * axis point never puts the rear too early.
     */
    float const hit_rear = p_hit->x_mm;

    if (p_hit->gap_mm < (float)OBS_SIDE_CRITICAL_MM)
    {
        if (p_seq->close_hits < CLOSE_HITS_LIMIT)
        {
            p_seq->close_hits++;
        }
    }
    else
    {
        p_seq->close_hits = 0U;
    }

    /* Keep the body its full margin from the side actually measured. The
     * target is an absolute offset, so the same error is never added
     * twice.
     */
    if (p_hit->gap_mm < (float)OBS_HALF_PATH_MM)
    {
        int32_t const required =
            round_int(offset + (float)OBS_HALF_PATH_MM - p_hit->gap_mm);

        if (required > p_seq->lateral_max_mm)
        {
            /* Further out than the scan checked: decide again. */
            p_seq->b_too_close = true;
        }
        else if (required > p_seq->lateral_mm)
        {
            p_seq->lateral_mm = required;
        }
        else
        {
            /* The offset already allows for this side. */
        }
    }

    if ((0U == p_seq->side_hits) || (hit_rear > p_seq->hit_rear_mm))
    {
        p_seq->hit_rear_mm = hit_rear;
    }

    if ((side * p_hit->y_mm) > (side * p_seq->face_y_mm))
    {
        p_seq->face_y_mm = p_hit->y_mm;
    }

    if (p_seq->side_hits < OBS_SIDE_SEEN_HITS)
    {
        p_seq->side_hits++;
    }

    p_seq->b_seen      = p_seq->b_seen ||
                         (p_seq->side_hits >= OBS_SIDE_SEEN_HITS);
    p_seq->clear_hits   = 0U;
    p_seq->b_clear_axis = false;
    p_seq->hit_axle_mm  = p_seq->pose.x_mm;

    if (p_seq->b_seen)
    {
        /* Evidence only moves the rear further back, never forward. */
        p_seq->rear_x_mm = (p_seq->hit_rear_mm > p_seq->rear_x_mm)
                               ? p_seq->hit_rear_mm
                               : p_seq->rear_x_mm;

        if (!b_was_seen)
        {
            /* Values fit INT on the target; UB preserves string bytes. */
            tm_printf((UB *)"[OBS] Side object confirmed; gap=%d mm\n",
                      (INT)p_hit->gap_mm);
        }
    }
}

static bool probe_update (obstacle_seq_t *            p_seq,
                          obs_state_reading_t const * p_reading)
{
    side_hit_t        hit  = {0.0F, 0.0F, 0.0F, false};
    side_view_t const view = side_classify(p_seq, p_reading, &hit);
    bool              b_goes_on = false;

    p_seq->b_probe_clear = false;
    p_seq->b_probe_point = false;

    if (OBS_ECHO_NONE == p_reading->echo)
    {
        /* No verdict from this ping; the next one is waited for. */
        p_seq->no_echo++;
    }
    else
    {
        p_seq->no_echo = 0U;
    }

    if (SIDE_HIT == view)
    {
        /* Something is between the car and the line on the way back. It
         * is passed like more of the object.
         */
        side_hit_record(p_seq, &hit);
        b_goes_on = true;
    }
    else if (SIDE_CLEAR == view)
    {
        p_seq->b_probe_clear = true;
        p_seq->probe_axle_mm = p_seq->pose.x_mm;

        if (hit.b_point)
        {
            /* A surface further on; the swept check decides if it is in
             * the way of the turn or the run to the line.
             */
            p_seq->b_probe_point = true;
            p_seq->probe_x_mm    = hit.x_mm;
            p_seq->probe_y_mm    = hit.y_mm;
        }
    }
    else
    {
        /* Unknown: not clear. */
    }

    return b_goes_on;
}

static void set_look (obstacle_seq_t * p_seq, int32_t angle_deg)
{
    int32_t const left  = servo_left_limit_deg();
    int32_t const right = servo_right_limit_deg();
    int32_t       look  = angle_deg;

    /* Ask only for a bearing the servo can reach, so the published bearing
     * matches the request exactly.
     */
    look = (look > left) ? left : look;
    look = (look < -right) ? -right : look;

    p_seq->look_deg  = look;
    p_seq->look_msec = p_seq->now_msec;
    obs_state_watch_angle_set(look);
}

static void aim_side_watch (obstacle_seq_t * p_seq)
{
    /* The object is on the side opposite the bypass. */
    p_seq->clear_hits = 0U;
    set_look(p_seq, -p_seq->side * OBS_SIDE_LOOK_DEG);
}

static void aim_return_probe (obstacle_seq_t * p_seq)
{
    /* The return run heads -side * OBS_RETURN_DEG from the line; look
     * along it, allowing for the car's present heading.
     */
    int32_t const heading_deg = heading_ddeg(&p_seq->pose) / DDEG_PER_DEG;

    p_seq->b_probe       = true;
    p_seq->b_probe_clear = false;
    p_seq->b_probe_point = false;
    set_look(p_seq, (-p_seq->side * OBS_RETURN_DEG) - heading_deg);
}

static bool outward_blocked (obstacle_seq_t * p_seq)
{
    obs_state_reading_t reading;

    if (side_reading_take(p_seq, &reading))
    {
        p_seq->no_echo = (OBS_ECHO_NONE == reading.echo)
                             ? (p_seq->no_echo + 1U)
                             : 0U;

        if ((OBS_ECHO_VALID == reading.echo) &&
            (reading.range_mm < OBS_OUTWARD_GUARD_MM))
        {
            if (p_seq->outward_hits < CLOSE_HITS_LIMIT)
            {
                p_seq->outward_hits++;
            }
        }
        else
        {
            p_seq->outward_hits = 0U;
        }

        if (p_seq->outward_hits >= CLOSE_HITS_LIMIT)
        {
            /* Values fit INT on the target; UB preserves string bytes. */
            tm_printf((UB *)"[OBS] Departure-side obstruction: %d mm\n",
                      (INT)reading.range_mm);
        }
    }

    return p_seq->outward_hits >= CLOSE_HITS_LIMIT;
}

static bool rear_clear (obstacle_seq_t const * p_seq)
{
    /* The object must have been seen, then seen clear several times over
     * enough axle travel since its last echo. A time-out alone cannot
     * imply the object ended; the return still needs a fresh probe.
     */
    return p_seq->b_seen && (p_seq->clear_hits >= OBS_SIDE_CLEAR_HITS) &&
           ((p_seq->pose.x_mm - p_seq->hit_axle_mm) >=
            (float)OBS_SIDE_CLEAR_TRAVEL_MM);
}

static bool return_turn_safe (obstacle_seq_t const * p_seq)
{
    /* The probe must be recent: taken within the lead of this spot. */
    bool const b_fresh =
        p_seq->b_probe && p_seq->b_probe_clear &&
        ((p_seq->pose.x_mm - p_seq->probe_axle_mm) <=
         (float)(HALF * OBS_RETURN_PROBE_LEAD_MM));

    return b_fresh && rear_clear(p_seq) &&
           (p_seq->pose.x_mm >= turn_point(p_seq)) && return_path_clear(p_seq);
}

static bool return_path_clear (obstacle_seq_t const * p_seq)
{
    obs_geometry_pose_t start = {0, 0, 0};
    obs_geometry_pose_t pose  = {0, 0, 0};
    obs_geometry_box_t  region = {0, 0, 0, 0};
    obs_geometry_box_t  point  = {0, 0, 0, 0};
    int32_t const face_y = round_int(p_seq->face_y_mm);
    bool          b_hit  = false;

    /* Everything behind the rear and inwards of the side face may be the
     * object, right across the line.
     */
    region.x_low_mm  = round_int(p_seq->face_x_mm) - REGION_REACH_MM;
    region.x_high_mm = round_int(effective_rear(p_seq));
    region.y_low_mm  = (p_seq->side > 0) ? (face_y - REGION_REACH_MM) : face_y;
    region.y_high_mm = (p_seq->side > 0) ? face_y : (face_y + REGION_REACH_MM);

    if (p_seq->b_probe_point)
    {
        point.x_low_mm  = round_int(p_seq->probe_x_mm);
        point.x_high_mm = point.x_low_mm;
        point.y_low_mm  = round_int(p_seq->probe_y_mm);
        point.y_high_mm = point.y_low_mm;
    }

    pose_round(&p_seq->pose, &start);
    pose = start;

    /* The pivot back on the wheel nearer the line, in small steps. */
    for (int32_t turn = 0; (turn <= OBS_RETURN_DEG) && (!b_hit);
         turn += TURN_STEP_DEG)
    {
        obs_geometry_pivot(&start, -p_seq->side, -p_seq->side * turn, &pose);
        b_hit = obs_geometry_body_hits_box(&pose, &region,
                                           OBS_SAFETY_MARGIN_MM) ||
                (p_seq->b_probe_point &&
                 obs_geometry_body_hits_box(&pose, &point,
                                            OBS_SAFETY_MARGIN_MM));
    }

    /* Then the straight run until the axle reaches the line. */
    start = pose;

    for (int32_t run = RUN_STEP_MM;
         (run <= RUN_LIMIT_MM) && (!b_hit) &&
         ((p_seq->side * pose.y_mm) > 0);
         run += RUN_STEP_MM)
    {
        obs_geometry_advance(&start, run, &pose);
        b_hit = obs_geometry_body_hits_box(&pose, &region,
                                           OBS_SAFETY_MARGIN_MM) ||
                (p_seq->b_probe_point &&
                 obs_geometry_body_hits_box(&pose, &point,
                                            OBS_SAFETY_MARGIN_MM));
    }

    return !b_hit;
}

static void clear_axis_note (obstacle_seq_t *            p_seq,
                             obs_state_reading_t const * p_reading)
{
    int32_t     origin_fwd = 0;
    int32_t     origin_lat = 0;
    float const heading    = p_seq->pose.heading_rad;
    float const ray =
        heading + ((float)p_reading->bearing_deg * OBS_RAD_PER_DEG);

    obs_geometry_sensor_origin(p_reading->bearing_deg, &origin_fwd,
                               &origin_lat);

    float const sensor_x = p_seq->pose.x_mm +
                           ((float)origin_fwd * OBS_COSF(heading)) -
                           ((float)origin_lat * OBS_SINF(heading));
    float const sensor_y = p_seq->pose.y_mm +
                           ((float)origin_fwd * OBS_SINF(heading)) +
                           ((float)origin_lat * OBS_COSF(heading));
    float const rise     = OBS_SINF(ray);
    float const to_face  = p_seq->face_y_mm - sensor_y;

    /* Where the beam axis crosses the line of the side seen so far. */
    if ((rise * to_face) > 0.0F)
    {
        float const reach_x =
            sensor_x + ((to_face / rise) * OBS_COSF(ray));

        if ((!p_seq->b_clear_axis) || (reach_x > p_seq->clear_axis_mm))
        {
            p_seq->clear_axis_mm = reach_x;
        }

        p_seq->b_clear_axis = true;
    }
}

static float effective_rear (obstacle_seq_t const * p_seq)
{
    bool const b_disproved = p_seq->b_clear_axis &&
                             (p_seq->clear_axis_mm >= p_seq->rear_guess_mm);
    float      rear        = p_seq->rear_x_mm;

    if ((!b_disproved) && (p_seq->rear_guess_mm > rear))
    {
        rear = p_seq->rear_guess_mm;
    }

    return rear;
}

static float turn_point (obstacle_seq_t const * p_seq)
{
    /* The rear of the car must be past the rear by the margin. */
    float const end_x = effective_rear(p_seq) + (float)OBS_TURN_PAST_MM;

    return (end_x > p_seq->recheck_x_mm) ? end_x : p_seq->recheck_x_mm;
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
    obs_state_reading_t reading;
    int32_t             origin_fwd = 0;
    int32_t             origin_lat = 0;

    obs_state_front_reading_get(&reading);
    obs_geometry_sensor_origin(0, &origin_fwd, &origin_lat);

    float const ahead_x =
        p_seq->pose.x_mm + ((float)(origin_fwd + reading.range_mm) *
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
        aim_side_watch(p_seq);
        enter_turn(p_seq, OBS_PHASE_TURN_PARALLEL, 0, now_msec);
    }
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
    obs_state_reading_t reading;

    obs_state_front_reading_get(&reading);

    if (reading.seq != p_seq->front_seq)
    {
        int32_t origin_fwd = 0;
        int32_t origin_lat = 0;
        bool    b_near     = (OBS_ECHO_VALID == reading.echo) &&
                      (reading.range_mm > 0) &&
                      (reading.range_mm < OBS_GUARD_MM);

        p_seq->front_seq     = reading.seq;
        p_seq->front_axle_mm = p_seq->pose.x_mm;
        p_seq->front_no_echo = (OBS_ECHO_NONE == reading.echo)
                                   ? (p_seq->front_no_echo + 1U)
                                   : 0U;
        /* A missing echo verifies nothing ahead. */
        p_seq->b_front_clear = (OBS_ECHO_NONE != reading.echo) && (!b_near);
        obs_geometry_sensor_origin(0, &origin_fwd, &origin_lat);

        if (b_near && b_to_line)
        {
            /* Where the echo came from, measured towards the side the
             * robot comes from.
             */
            float const hit_y =
                p_seq->pose.y_mm +
                ((float)(origin_fwd + reading.range_mm) *
                 OBS_SINF(p_seq->pose.heading_rad));

            b_near = (((float)p_seq->search_side) * hit_y) >
                     (float)(-OBS_GUARD_BEYOND_LINE_MM);
        }

        p_seq->guard_hits = b_near ? (p_seq->guard_hits + 1U) : 0U;
    }

    return p_seq->guard_hits >= OBS_GUARD_HITS;
}

static bool front_fresh (obstacle_seq_t const * p_seq)
{
    return p_seq->b_front_clear &&
           ((p_seq->pose.x_mm - p_seq->front_axle_mm) <=
            (float)OBS_FRONT_STALE_MM);
}

static bool sensor_lost (obstacle_seq_t const * p_seq)
{
    return (p_seq->no_echo >= OBS_SIDE_NO_ECHO_LIMIT) ||
           (p_seq->front_no_echo >= OBS_SIDE_NO_ECHO_LIMIT);
}

static void wait_for_reading (obstacle_seq_t * p_seq, obstacle_drive_t * p_out)
{
    p_out->left_rpm  = 0;
    p_out->right_rpm = 0;
    p_out->b_stop    = true;

    if (!p_seq->b_waiting)
    {
        p_seq->b_waiting = true;
        p_seq->wait_msec = p_seq->now_msec;
    }
    else if ((p_seq->now_msec - p_seq->wait_msec) > OBS_READING_WAIT_MSEC)
    {
        /* The servo or sensor never delivered: stop for good. */
        raise_fault(p_seq, p_out, OBS_FAULT_SERVO);
    }
    else
    {
        /* Still within the bounded wait. */
    }
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
        obs_state_reading_t reading;

        obs_state_front_reading_get(&reading);
        /* Values fit INT on the 32-bit target; UB preserves string bytes. */
        tm_printf((UB *)"[OBS] Something ahead at %d mm; stopping to look\n",
                  (INT)reading.range_mm);
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
    obs_state_reading_t side_reading;
    obs_state_reading_t front_reading;

    /* Pings from before this bypass are history, not evidence. */
    obs_state_side_reading_get(&side_reading);
    obs_state_front_reading_get(&front_reading);

    p_seq->b_active         = true;
    p_seq->now_msec         = now_msec;
    p_seq->last_left        = left_count;
    p_seq->last_right       = right_count;
    p_seq->pose.x_mm        = 0.0F;
    p_seq->pose.y_mm        = 0.0F;
    p_seq->pose.heading_rad = 0.0F;
    p_seq->b_planned        = false;
    p_seq->side             = 1;
    p_seq->out_ddeg         = 0;
    p_seq->lateral_mm       = 0;
    p_seq->lateral_max_mm   = 0;
    p_seq->look_deg         = 0;
    p_seq->look_msec        = now_msec;
    p_seq->turn_target_ddeg = 0;
    p_seq->turn_dir         = 1;
    p_seq->face_x_mm        = 0.0F;
    p_seq->face_y_mm        = 0.0F;
    p_seq->rear_x_mm        = 0.0F;
    p_seq->rear_guess_mm    = 0.0F;
    p_seq->clear_axis_mm    = 0.0F;
    p_seq->recheck_x_mm     = 0.0F;
    p_seq->end_x_mm         = 0.0F;
    p_seq->hit_rear_mm      = 0.0F;
    p_seq->hit_axle_mm      = 0.0F;
    p_seq->front_axle_mm    = 0.0F;
    p_seq->probe_axle_mm    = 0.0F;
    p_seq->probe_x_mm       = 0.0F;
    p_seq->probe_y_mm       = 0.0F;
    p_seq->b_seen           = false;
    p_seq->b_clear_axis     = false;
    p_seq->b_too_close      = false;
    p_seq->b_front_clear    = false;
    p_seq->b_probe          = false;
    p_seq->b_probe_clear    = false;
    p_seq->b_probe_point    = false;
    p_seq->b_waiting        = false;
    p_seq->wait_msec        = now_msec;
    p_seq->side_hits        = 0U;
    p_seq->clear_hits       = 0U;
    p_seq->no_echo          = 0U;
    p_seq->front_no_echo    = 0U;
    p_seq->side_seq         = side_reading.seq;
    p_seq->front_seq        = front_reading.seq;
    p_seq->outward_hits     = 0U;
    p_seq->close_hits       = 0U;
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
                        (OBS_PHASE_PASS_END == phase) ||
                        (OBS_PHASE_DIAG_OUT == phase) ||
                        (OBS_PHASE_TURN_PARALLEL == phase);
    bool const b_front = (OBS_PHASE_PASS_SIDE == phase) ||
                         (OBS_PHASE_PASS_END == phase) ||
                         (OBS_PHASE_DIAG_OUT == phase) ||
                         (OBS_PHASE_TURN_BACK == phase) ||
                         (OBS_PHASE_SEARCH_LINE == phase);

    p_seq->phase      = phase;
    p_seq->enter_msec = now_msec;
    p_seq->line_hits  = 0U;
    p_seq->b_waiting  = false;

    /* Pings taken before this phase are stale for its guard. */
    p_seq->guard_hits = 0U;

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
    scan_plan_bypass_t   plan = {0U, 0U};
    obs_state_aim_info_t aim  = {0U, 0, OBS_AIM_UNAVAILABLE};

    obs_state_plan_load(&plan);
    obs_state_aim_get(&aim);

    uint32_t const result = scan_plan_result(&plan);
    int32_t const  plan_side = scan_plan_side_is_left(&plan) ? 1 : -1;

    if (OBS_AIM_FORWARD != aim.aim)
    {
        /* The scan always ends at FRONT; anything else is a servo fault. */
        raise_fault(p_seq, p_out, OBS_FAULT_SERVO);
    }
    else if (SCAN_PLAN_CLEAR == result)
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

            if ((OBS_PHASE_PASS_SIDE == p_seq->resume_phase) ||
                (OBS_PHASE_PASS_END == p_seq->resume_phase))
            {
                p_seq->b_probe = false;
                aim_side_watch(p_seq);
            }
            else if (OBS_PHASE_DIAG_OUT == p_seq->resume_phase)
            {
                set_look(p_seq, p_seq->side * OBS_SIDE_LOOK_DEG);
            }
            else
            {
                /* Other phases use no side watch. */
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
        /* The plan's distances are from the axle midpoint. */
        int32_t const near_mm  = scan_plan_near_cm(&plan) * MM_PER_CM;
        int32_t const need_mm  = scan_plan_lateral_cm(&plan) * MM_PER_CM;
        int32_t const slack_mm = scan_plan_slack_cm(&plan) * MM_PER_CM;
        float const   face_x =
            p_seq->pose.x_mm +
            ((float)near_mm * OBS_COSF(p_seq->pose.heading_rad));
        float const rear_guess = face_x + (float)OBS_DEFAULT_DEPTH_MM;
        /* The object's side as the scan saw it, nearest the path. */
        float const face_y =
            p_seq->pose.y_mm +
            ((float)plan_side * (float)(need_mm - OBS_PLAN_PATH_MM));

        p_seq->b_planned = true;
        p_seq->side      = plan_side;
        p_seq->out_ddeg =
            scan_plan_angle_deg(scan_plan_angle_index(&plan)) * DDEG_PER_DEG;
        /* The plan's offset is from where the robot is now, which is on the
         * line at first and beside it after a stop to look again.
         */
        p_seq->lateral_mm =
            (int32_t)(((float)p_seq->side) * p_seq->pose.y_mm) + need_mm;
        /* Corrections stay inside the corridor the scan checked. */
        p_seq->lateral_max_mm =
            p_seq->lateral_mm + ((slack_mm < OBS_PASS_CORRECT_MAX_MM)
                                     ? slack_mm
                                     : OBS_PASS_CORRECT_MAX_MM);
        p_seq->face_x_mm   = face_x;
        p_seq->face_y_mm   = face_y;
        p_seq->rear_guess_mm = (rear_guess > p_seq->rear_guess_mm)
                                   ? rear_guess
                                   : p_seq->rear_guess_mm;
        p_seq->b_seen       = false;
        p_seq->b_clear_axis = false;
        p_seq->b_too_close = false;
        p_seq->side_hits   = 0U;
        p_seq->close_hits  = 0U;
        p_seq->no_echo     = 0U;
        p_seq->search_side = p_seq->side;
        obs_state_set_route((p_seq->side > 0) ? OBS_ROUTE_LEFT
                                              : OBS_ROUTE_RIGHT);
        obs_state_set_width_cm(scan_plan_width_cm(&plan));
        /* Values fit INT on the 32-bit target; UB preserves string bytes. */
        tm_printf((UB *)"[OBS] Bypass %s: out %d deg, offset %d mm (max %d), "
                        "face at %d mm\n",
                  (p_seq->side > 0) ? (UB *)"left" : (UB *)"right",
                  (INT)(p_seq->out_ddeg / DDEG_PER_DEG),
                  (INT)p_seq->lateral_mm, (INT)p_seq->lateral_max_mm,
                  (INT)p_seq->face_x_mm);
        enter_turn(p_seq, OBS_PHASE_TURN_OUT, p_seq->side * p_seq->out_ddeg,
                   now_msec);
    }
}

static void pose_round (obstacle_pose_t const * p_pose,
                        obs_geometry_pose_t *   p_out)
{
    p_out->x_mm        = round_int(p_pose->x_mm);
    p_out->y_mm        = round_int(p_pose->y_mm);
    p_out->heading_deg = round_int(p_pose->heading_rad * OBS_DEG_PER_RAD);
}

static int32_t round_int (float value)
{
    /* Poses stay within a few metres, so the conversion cannot overflow. */
    return (int32_t)((value < 0.0F) ? (value - 0.5F) : (value + 0.5F));
}

/*** end of file ***/

