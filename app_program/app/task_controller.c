/** @file task_controller.c
 * @brief Existing line and junction policy with immediate bypass handoff.
 */
#include "task_controller.h"
#include "app_config.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/line_barcode/line_follow.h"
#include "modules/motion/motion.h"
#include "modules/terrain/terrain.h"
#include "modules/obstacle/obstacle.h"
#include <tm/tmonitor.h>

#define JUNCTION_HOLD_CYCLES \
    (APP_JUNCTION_HOLD_MSEC / APP_CONTROLLER_PERIOD_MSEC)
#define JUNCTION_SETTLE_CYCLES \
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
    bool b_have_previous_action;
    uint32_t junction_count;
    uint32_t junction_cycles;
    uint32_t settle_cycles;
    bool b_rearmed;
} controller_history_t;

/** @brief Wait for startup calibration and fresh IR. */
static robot_state_t handle_idle (void);
/** @brief Apply the original line and junction policy. */
static robot_state_t handle_following (controller_history_t * p_history);
/** @brief Preserve the original stopped-junction rearming sequence. */
static robot_state_t handle_junction (controller_history_t * p_history);
/** @brief Run one bounded bypass step. */
static robot_state_t handle_obstacle (controller_history_t * p_history);
/** @brief Preserve the original action mapping and junction option. */
static robot_state_t apply_action (ir_shared_snapshot_t const * p_reading,
                                   controller_history_t * p_history);

void task_controller_task (INT start_code, void * p_context)
{
    robot_state_t state = ROBOT_IDLE;
    controller_history_t history = {LINE_UNCERTAIN, false, 0U, 0U, 0U, false};

    (void)start_code;
    (void)p_context;
    tm_printf((UB *)"[CTRL] Controller started\n");
#if APP_IMU_ENABLED
    motion_apply(LINE_UNCERTAIN);
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
            case ROBOT_ERROR:
                /* Remain stopped until reset or a future restart command. */
            break;
            default:
                motion_apply(LINE_UNCERTAIN);
                obstacle_abort();
                state = ROBOT_ERROR;
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
    ER error = ir_shared_get_latest(&reading);

    if (error < E_OK)
    {
        tm_printf((UB *)"[CTRL] IR read error: %d\n", error);
        return ROBOT_ERROR;
    }
    if (!reading.b_valid)
    {
        return ROBOT_IDLE;
    }
    SYSTIM now = {0};

    if (tk_get_otm(&now) < E_OK)
    {
        return ROBOT_ERROR;
    }
    if ((UW)(now.lo - reading.timestamp_msec) <= APP_IR_MAX_AGE_MSEC)
    {
        tm_printf((UB *)"[CTRL] IDLE -> LINE_FOLLOWING\n");
        return ROBOT_LINE_FOLLOWING;
    }
    return ROBOT_IDLE;
}

static robot_state_t handle_following (controller_history_t * p_history)
{
    ir_shared_snapshot_t reading = {0};
    ER error = ir_shared_get_latest(&reading);
    SYSTIM now = {0};

    if (error >= E_OK)
    {
        error = tk_get_otm(&now);
    }
    if (error < E_OK)
    {
        motion_apply(LINE_UNCERTAIN);
        return ROBOT_ERROR;
    }
    if (!reading.b_valid ||
        ((UW)(now.lo - reading.timestamp_msec) > APP_IR_MAX_AGE_MSEC))
    {
        motion_apply(LINE_UNCERTAIN);
        tm_printf((UB *)"[CTRL] IR data stale -> STOPPED\n");
        return ROBOT_STOPPED;
    }
    if (obstacle_has_fault())
    {
        motion_apply(LINE_UNCERTAIN);
        obstacle_abort();
        tm_printf((UB *)"[CTRL] Obstacle sensor fault -> ERROR\n");
        return ROBOT_ERROR;
    }
    if (obstacle_is_blocked())
    {
        motion_apply(LINE_UNCERTAIN);
        tm_printf((UB *)"[CTRL] Obstacle confirmed -> OBSTACLE\n");
        return ROBOT_OBSTACLE;
    }

    bool junction = false;
    bool left = false;
    bool right = false;

    error = ir_shared_take_junction(&junction);
    if (error >= E_OK)
    {
        error = ir_shared_take_sides(&left, &right);
    }
    if (error < E_OK)
    {
        motion_apply(LINE_UNCERTAIN);
        return ROBOT_ERROR;
    }
    if (junction && ((reading.left != 1U) || (reading.right != 1U)))
    {
        reading.left = 1U;
        reading.right = 1U;
        tm_printf((UB *)"[CTRL] Junction seen between reads\n");
    }
    else if ((left != right) && (reading.left == 0U) &&
             (reading.right == 0U))
    {
        reading.left = left ? 1U : 0U;
        reading.right = right ? 1U : 0U;
        tm_printf((UB *)"[CTRL] Line touch seen between reads\n");
    }
    return apply_action(&reading, p_history);
}

static robot_state_t handle_junction (controller_history_t * p_history)
{
    ir_shared_snapshot_t reading = {0};
    bool black = false;
    ER error = ir_shared_get_latest(&reading);
    SYSTIM now = {0};

    if (error >= E_OK)
    {
        error = ir_shared_take_junction(&black);
    }
    if (error >= E_OK)
    {
        error = tk_get_otm(&now);
    }
    if (error < E_OK)
    {
        return ROBOT_ERROR;
    }
    bool const fresh = reading.b_valid &&
        ((UW)(now.lo - reading.timestamp_msec) <= APP_IR_MAX_AGE_MSEC);

    if (p_history->junction_cycles < JUNCTION_HOLD_CYCLES)
    {
        p_history->junction_cycles++;
        return ROBOT_AT_JUNCTION;
    }
    if (black)
    {
        if (!p_history->b_rearmed)
        {
            tm_printf((UB *)"[CTRL] Lifted -> waiting to be placed\n");
        }
        p_history->b_rearmed = true;
        p_history->settle_cycles = 0U;
        return ROBOT_AT_JUNCTION;
    }
    if (!p_history->b_rearmed || !fresh)
    {
        p_history->settle_cycles = 0U;
        return ROBOT_AT_JUNCTION;
    }
    p_history->settle_cycles++;
    if (p_history->settle_cycles < JUNCTION_SETTLE_CYCLES)
    {
        return ROBOT_AT_JUNCTION;
    }
    bool left = false;
    bool right = false;

    if (ir_shared_take_sides(&left, &right) < E_OK)
    {
        return ROBOT_ERROR;
    }
    p_history->b_have_previous_action = false;
    tm_printf((UB *)"[CTRL] Back on line -> LINE_FOLLOWING\n");
    return ROBOT_LINE_FOLLOWING;
}

static robot_state_t handle_obstacle (controller_history_t * p_history)
{
    ir_shared_snapshot_t reading = {0};
    SYSTIM now = {0};
    ER error = ir_shared_get_latest(&reading);

    if (error >= E_OK)
    {
        error = tk_get_otm(&now);
    }
    if ((error < E_OK) || !reading.b_valid ||
        ((UW)(now.lo - reading.timestamp_msec) > APP_IR_MAX_AGE_MSEC))
    {
        motion_apply(LINE_UNCERTAIN);
        obstacle_abort();
        return ROBOT_STOPPED;
    }
    obstacle_drive_t const drive =
        obstacle_bypass_step(reading.left, reading.right, now.lo);

    if (drive.b_fault)
    {
        motion_apply(LINE_UNCERTAIN);
        tm_printf((UB *)"[CTRL] Bypass fault 0x%x -> ERROR\n",
                  (UINT)obs_state_status_faults(obs_state_status_get()));
        return ROBOT_ERROR;
    }
    if (drive.b_done)
    {
        bool junction = false;
        bool left = false;
        bool right = false;

        /* First stop the previous wheel command before normal steering. */
        motion_apply(LINE_UNCERTAIN);
        error = ir_shared_take_junction(&junction);
        if (error >= E_OK)
        {
            error = ir_shared_take_sides(&left, &right);
        }
        if (error < E_OK)
        {
            return ROBOT_ERROR;
        }
        p_history->b_have_previous_action = false;
        tm_printf((UB *)"[CTRL] Bypass done -> normal line control\n");
        return apply_action(&reading, p_history);
    }
    if (drive.b_stop)
    {
        motion_apply(LINE_UNCERTAIN);
    }
    else if (drive.b_line_follow)
    {
        return apply_action(&reading, p_history);
    }
    else
    {
        motion_drive((float)drive.left_rpm, (float)drive.right_rpm);
    }
    return ROBOT_OBSTACLE;
}

static robot_state_t apply_action (ir_shared_snapshot_t const * p_reading,
                                   controller_history_t * p_history)
{
    line_follow_action_t const action =
        line_follow_decide(p_reading->left, p_reading->right);
    bool const new_junction = (action == LINE_UNCERTAIN) &&
        (!p_history->b_have_previous_action ||
         (p_history->previous_action != LINE_UNCERTAIN));

    if ((action == LINE_UNCERTAIN) && !APP_JUNCTION_STOP_ENABLED)
    {
        motion_apply(LINE_FORWARD);
        if (new_junction)
        {
            p_history->junction_count++;
            tm_printf((UB *)"[CTRL] Junction %u -> crossing\n",
                      (UINT)p_history->junction_count);
        }
        p_history->previous_action = LINE_UNCERTAIN;
        p_history->b_have_previous_action = true;
        return ROBOT_LINE_FOLLOWING;
    }
    if (action == LINE_UNCERTAIN)
    {
        motion_apply(LINE_UNCERTAIN);
        p_history->junction_count++;
        p_history->junction_cycles = 0U;
        p_history->settle_cycles = 0U;
        p_history->b_rearmed = false;
        tm_printf((UB *)"[CTRL] Junction %u -> STOPPED\n",
                  (UINT)p_history->junction_count);
        return ROBOT_AT_JUNCTION;
    }
    motion_apply(action);
    if (!p_history->b_have_previous_action ||
        (action != p_history->previous_action))
    {
        tm_printf((UB *)"[CTRL] L=%u R=%u ACTION=%s\n",
                  (UINT)p_reading->left, (UINT)p_reading->right,
                  line_follow_text(action));
        p_history->previous_action = action;
        p_history->b_have_previous_action = true;
    }
    return ROBOT_LINE_FOLLOWING;
}

/*** end of file ***/

