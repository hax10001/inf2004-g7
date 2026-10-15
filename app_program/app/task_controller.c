/** @file task_controller.c
 * @brief Preserve controller transitions, freshness checks, and motor actions.
 */
#include "task_controller.h"
#include "app_config.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/line_barcode/line_follow.h"
#include "modules/motion/motion.h"
#include <tm/tmonitor.h>

typedef enum
{
    ROBOT_IDLE,
    ROBOT_LINE_FOLLOWING,
    ROBOT_STOPPED,
    ROBOT_ERROR
} robot_state_t;

typedef struct
{
    line_follow_action_t previous_action;
    bool                 b_have_previous_action;
} controller_history_t;

/** @brief Remain idle until fresh IR data is available. */
static robot_state_t handle_idle (void);
/** @brief Follow the line, preserving stop and error behaviour. */
static robot_state_t handle_following (controller_history_t * p_history);
/** @brief Apply and log an action without changing controller state. */
static void apply_action (ir_shared_snapshot_t const * p_reading,
                          controller_history_t *       p_history);

void task_controller_task (INT start_code, void * p_context)
{
    robot_state_t        state   = ROBOT_IDLE;
    controller_history_t history = {LINE_UNCERTAIN, false};

    /* Casting unused arguments to void cannot alter their values. */
    (void)start_code;
    (void)p_context;

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[CTRL] Controller started\n");

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

    apply_action(&reading, p_history);
    return ROBOT_LINE_FOLLOWING;
}

static void apply_action (ir_shared_snapshot_t const * p_reading,
                          controller_history_t *       p_history)
{
    line_follow_action_t action =
        line_follow_decide(p_reading->left, p_reading->right);

    if (LINE_UNCERTAIN == action)
    {
        motion_apply(LINE_UNCERTAIN);
        /* WARNING: Preserve the tested behaviour: this does not latch STOPPED.
         * The existing log describes a stop command, not a state transition.
         * UB views the same string bytes; tm_printf does not modify them.
         */
        tm_printf((UB *)"[CTRL] UNCERTAIN -> STOPPED\n");
        return;
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
}

/*** end of file ***/

