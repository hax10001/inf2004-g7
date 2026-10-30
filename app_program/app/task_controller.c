/** @file task_controller.c
 * @brief Preserve controller transitions, freshness checks, and motor actions.
 */
#include "task_controller.h"
#include "app_config.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/line_barcode/line_follow.h"
#include "modules/motion/motion.h"
#include "modules/terrain/terrain.h"
#include "modules/obstacle/obstacle.h"
#include "modules/obstacle/obs_state.h"
#include <tm/tmonitor.h>

#define JUNCTION_HOLD_CYCLES                                                   \
    (APP_JUNCTION_HOLD_MSEC / APP_CONTROLLER_PERIOD_MSEC)
#define JUNCTION_SETTLE_CYCLES                                                 \
    (APP_JUNCTION_SETTLE_MSEC / APP_CONTROLLER_PERIOD_MSEC)

typedef enum
{
    ROBOT_IDLE,
    ROBOT_LINE_FOLLOWING,
    ROBOT_AT_JUNCTION,
    ROBOT_OBSTACLE,
    ROBOT_STOPPED,
    ROBOT_ERROR
} robot_state_t;

typedef struct
{
    line_follow_action_t previous_action;
    bool                 b_have_previous_action;
    uint32_t             junction_count;
    uint32_t             junction_cycles; /**< Periods since stopping. */
    uint32_t             settle_cycles;   /**< Periods back on a line. */
    bool                 b_rearmed;       /**< Both black after the hold. */
} controller_history_t;

/** @brief Wait for startup calibration and fresh IR data. */
static robot_state_t handle_idle (void);
/** @brief Follow the line, preserving stop and error behaviour. */
static robot_state_t handle_following (controller_history_t * p_history);
/** @brief Stay stopped at a junction until placed back on a line.
 *
 * The IR sensors read black when lifted. Both black seen after the hold
 * period means the robot is still on the junction or has been picked up;
 * a coast off the junction ends within the hold and is not counted. A
 * steady line reading after that resumes following.
 */
static robot_state_t handle_junction (controller_history_t * p_history);
/** @brief Run one period of scan, bypass and line reacquisition. */
static robot_state_t handle_obstacle (controller_history_t * p_history);
/** @brief Apply and log an action.
 * @return ROBOT_AT_JUNCTION when both sensors are black.
 */
static robot_state_t apply_action (ir_shared_snapshot_t const * p_reading,
                                   controller_history_t *       p_history);

void task_controller_task (INT start_code, void * p_context)
{
    robot_state_t        state   = ROBOT_IDLE;
    controller_history_t history = {LINE_UNCERTAIN, false, 0U, 0U, 0U, false};

    /* Casting unused arguments to void cannot alter their values. */
    (void)start_code;
    (void)p_context;

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[CTRL] Controller started\n");

#if APP_IMU_ENABLED
    motion_apply(LINE_UNCERTAIN);
    /* UB views read-only string bytes; the monitor does not modify them. */
    tm_printf((UB *)"[CTRL] Waiting for IMU level reference\n");
#endif

    for (;;)
    {
        switch (state)
        {
            case ROBOT_IDLE:
                state = handle_idle();
            break;

            case ROBOT_LINE_FOLLOWING:
                state = handle_following(&history);
            break;

            case ROBOT_AT_JUNCTION:
                state = handle_junction(&history);
            break;

            case ROBOT_OBSTACLE:
                state = handle_obstacle(&history);
            break;

            case ROBOT_STOPPED:
                /* Remain stopped until a future restart mechanism exists. */
            break;

            case ROBOT_ERROR:
                /* Remain in error until reset or future recovery. */
            break;

            default:
                state = ROBOT_ERROR;
                motion_apply(LINE_UNCERTAIN);
            break;
        }

        tk_dly_tsk(APP_CONTROLLER_PERIOD_MSEC);
    }
}

static robot_state_t handle_idle (void)
{
#if APP_IMU_ENABLED
    if (!terrain_is_calibrated())
    {
        return ROBOT_IDLE;
    }
#endif

    ir_shared_snapshot_t reading = {0};
    ER                   error   = ir_shared_get_latest(&reading);

    if (error < E_OK)
    {
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] IR read error: %d\n", error);
        return ROBOT_ERROR;
    }

    if (!reading.b_valid)
    {
        return ROBOT_IDLE;
    }

    SYSTIM now = {0};
    error      = tk_get_otm(&now);

    if (error < E_OK)
    {
        return ROBOT_ERROR;
    }

    /* UW is the BSP's unsigned 32-bit type; subtraction wraps as before. */
    if ((UW)(now.lo - reading.timestamp_msec) <= APP_IR_MAX_AGE_MSEC)
    {
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] IDLE -> LINE_FOLLOWING\n");
        return ROBOT_LINE_FOLLOWING;
    }

    return ROBOT_IDLE;
}

static robot_state_t handle_following (controller_history_t * p_history)
{
    ir_shared_snapshot_t reading = {0};
    ER                   error   = ir_shared_get_latest(&reading);

    if (error < E_OK)
    {
        motion_apply(LINE_UNCERTAIN);
        return ROBOT_ERROR;
    }

    SYSTIM now = {0};
    error      = tk_get_otm(&now);

    if (error < E_OK)
    {
        motion_apply(LINE_UNCERTAIN);
        return ROBOT_ERROR;
    }

    /* UW is unsigned 32-bit; elapsed time retains modulo-2^32 behaviour. */
    if ((!reading.b_valid) ||
        ((UW)(now.lo - reading.timestamp_msec) > APP_IR_MAX_AGE_MSEC))
    {
        motion_apply(LINE_UNCERTAIN);
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] IR data stale -> STOPPED\n");
        return ROBOT_STOPPED;
    }

    if (obstacle_has_fault())
    {
        motion_apply(LINE_UNCERTAIN);
        obstacle_abort();
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] Obstacle sensor fault -> ERROR\n");
        return ROBOT_ERROR;
    }

    if (obstacle_is_blocked())
    {
        motion_apply(LINE_UNCERTAIN);
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] Obstacle confirmed -> OBSTACLE\n");
        return ROBOT_OBSTACLE;
    }

    bool b_junction = false;
    bool b_left     = false;
    bool b_right    = false;
    error           = ir_shared_take_junction(&b_junction);

    if (error >= E_OK)
    {
        error = ir_shared_take_sides(&b_left, &b_right);
    }

    if (error < E_OK)
    {
        motion_apply(LINE_UNCERTAIN);
        return ROBOT_ERROR;
    }

    if (b_junction && ((1U != reading.left) || (1U != reading.right)))
    {
        /* Both-black passed between reads; act on it as if still present. */
        reading.left  = 1U;
        reading.right = 1U;
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] Junction seen between reads\n");
    }
    else if ((b_left != b_right) && (0U == reading.left) &&
             (0U == reading.right))
    {
        /* One edge was touched between reads; still correct for it. Touches
         * on both sides at different times are left to the latest reading.
         */
        reading.left  = b_left ? 1U : 0U;
        reading.right = b_right ? 1U : 0U;
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] Line touch seen between reads\n");
    }
    else
    {
        /* The latest reading already reflects what was seen. */
    }

    return apply_action(&reading, p_history);
}

static robot_state_t handle_junction (controller_history_t * p_history)
{
    ir_shared_snapshot_t reading = {0};
    bool                 b_black = false;
    ER                   error   = ir_shared_get_latest(&reading);

    if (error >= E_OK)
    {
        /* Also clears the latch, so stale both-black cannot leak out. */
        error = ir_shared_take_junction(&b_black);
    }

    SYSTIM now = {0};

    if (error >= E_OK)
    {
        error = tk_get_otm(&now);
    }

    if (error < E_OK)
    {
        /* Motors are already stopped. */
        return ROBOT_ERROR;
    }

    /* UW is unsigned 32-bit; elapsed time retains modulo-2^32 behaviour. */
    bool const b_fresh =
        reading.b_valid &&
        ((UW)(now.lo - reading.timestamp_msec) <= APP_IR_MAX_AGE_MSEC);

    if (p_history->junction_cycles < JUNCTION_HOLD_CYCLES)
    {
        /* Ignore readings while the robot may still be coasting. */
        p_history->junction_cycles++;
        return ROBOT_AT_JUNCTION;
    }

    if (b_black)
    {
        if (!p_history->b_rearmed)
        {
            /* UB views the same string bytes; tm_printf does not modify
             * them.
             */
            tm_printf((UB *)"[CTRL] Lifted -> waiting to be placed on a "
                            "line\n");
        }

        p_history->b_rearmed     = true;
        p_history->settle_cycles = 0U;
        return ROBOT_AT_JUNCTION;
    }

    if ((!p_history->b_rearmed) || (!b_fresh))
    {
        p_history->settle_cycles = 0U;
        return ROBOT_AT_JUNCTION;
    }

    p_history->settle_cycles++;

    if (1U == p_history->settle_cycles)
    {
        /* Printed again whenever a both-black reading restarts the wait.
         * UB views the same string bytes; tm_printf does not modify them.
         */
        tm_printf((UB *)"[CTRL] Placed -> starting if the line stays\n");
    }

    if (p_history->settle_cycles < JUNCTION_SETTLE_CYCLES)
    {
        return ROBOT_AT_JUNCTION;
    }

    bool b_left  = false;
    bool b_right = false;

    /* Discard touches from being lifted and placed; they are not steering. */
    if (ir_shared_take_sides(&b_left, &b_right) < E_OK)
    {
        return ROBOT_ERROR;
    }

    /* Log the first action after resuming, whatever it is. */
    p_history->b_have_previous_action = false;
    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[CTRL] Back on line -> LINE_FOLLOWING\n");
    return ROBOT_LINE_FOLLOWING;
}

static robot_state_t handle_obstacle (controller_history_t * p_history)
{
    ir_shared_snapshot_t reading = {0};
    SYSTIM               now     = {0};
    ER                   error   = ir_shared_get_latest(&reading);

    if (error >= E_OK)
    {
        error = tk_get_otm(&now);
    }

    if ((error < E_OK) || (!reading.b_valid) ||
        ((UW)(now.lo - reading.timestamp_msec) > APP_IR_MAX_AGE_MSEC))
    {
        obstacle_abort();
        motion_apply(LINE_UNCERTAIN);
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] IR unavailable during bypass -> STOPPED\n");
        return ROBOT_STOPPED;
    }

    obstacle_drive_t const drive =
        obstacle_bypass_step(reading.left, reading.right, (uint32_t)now.lo);

    if (drive.b_fault)
    {
        motion_apply(LINE_UNCERTAIN);
        /* UINT matches %x on the 32-bit target; UB preserves string bytes. */
        tm_printf((UB *)"[CTRL] Bypass fault 0x%x -> ERROR\n",
                  (UINT)obs_state_status_faults(obs_state_status_get()));
        return ROBOT_ERROR;
    }

    if (drive.b_done)
    {
        bool b_junction = false;
        bool b_left     = false;
        bool b_right    = false;

        /* Discard latches from the bypass; they are not steering. */
        (void)ir_shared_take_junction(&b_junction);
        (void)ir_shared_take_sides(&b_left, &b_right);
        p_history->b_have_previous_action = false;
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[CTRL] Bypass done -> LINE_FOLLOWING\n");
        return ROBOT_LINE_FOLLOWING;
    }

    if (drive.b_line_follow)
    {
        line_follow_action_t action =
            line_follow_decide(reading.left, reading.right);

        motion_apply((LINE_UNCERTAIN == action) ? LINE_FORWARD : action);
    }
    else if (drive.b_stop)
    {
        motion_apply(LINE_UNCERTAIN);
    }
    else
    {
        motion_drive((float)drive.left_rpm, (float)drive.right_rpm);
    }

    return ROBOT_OBSTACLE;
}

static robot_state_t apply_action (ir_shared_snapshot_t const * p_reading,
                                   controller_history_t *       p_history)
{
    line_follow_action_t action =
        line_follow_decide(p_reading->left, p_reading->right);

    bool const b_new_junction =
        (LINE_UNCERTAIN == action) &&
        ((!p_history->b_have_previous_action) ||
         (LINE_UNCERTAIN != p_history->previous_action));

    if ((LINE_UNCERTAIN == action) && (0 == APP_JUNCTION_STOP_ENABLED))
    {
        /* Stopping is overridden: drive straight across the junction. */
        motion_apply(LINE_FORWARD);

        if (b_new_junction)
        {
            p_history->junction_count++;
            /* UINT matches %u on the 32-bit target; UB preserves bytes. */
            tm_printf((UB *)"[CTRL] Junction %u -> crossing (stop "
                            "disabled)\n",
                      (UINT)p_history->junction_count);
        }

        p_history->previous_action        = LINE_UNCERTAIN;
        p_history->b_have_previous_action = true;
        return ROBOT_LINE_FOLLOWING;
    }

    if (LINE_UNCERTAIN == action)
    {
        motion_apply(LINE_UNCERTAIN);
        p_history->junction_count++;
        p_history->junction_cycles = 0U;
        p_history->settle_cycles   = 0U;
        p_history->b_rearmed       = false;
        /* UINT matches %u on the 32-bit target; UB preserves string bytes. */
        tm_printf((UB *)"[CTRL] Junction %u -> STOPPED until placed on a "
                        "line\n",
                  (UINT)p_history->junction_count);
        return ROBOT_AT_JUNCTION;
    }

    motion_apply(action);

    if ((!p_history->b_have_previous_action) ||
        (action != p_history->previous_action))
    {
        /* UINT matches the original %u arguments on the 32-bit target.
         * UB views string bytes; tm_printf does not modify the string.
         */
        tm_printf((UB *)"[CTRL] L=%u R=%u ACTION=%s\n", (UINT)p_reading->left,
                  (UINT)p_reading->right, line_follow_text(action));

        p_history->previous_action        = action;
        p_history->b_have_previous_action = true;
    }

    return ROBOT_LINE_FOLLOWING;
}

/*** end of file ***/

