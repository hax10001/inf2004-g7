#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "controller_task.h"

#define CONTROLLER_INTERVAL_MS    50U
#define IR_MAX_AGE_MS             200U


typedef enum {

    ROBOT_IDLE,
    ROBOT_LINE_FOLLOWING,
    ROBOT_STOPPED,
    ROBOT_ERROR

} ROBOT_STATE;

EXPORT void controller_task(INT stacd, void *exinf)
{
    ROBOT_STATE state = ROBOT_IDLE;
    LINE_ACTION previous_action = LINE_UNCERTAIN;
    BOOL have_previous_action = FALSE;

    (void)stacd;
    (void)exinf;

    tm_printf((UB *)"[CTRL] Controller started\n");


    while (1) {

        IR_SNAPSHOT ir;
        SYSTIM now;
        ER er;

        switch (state) {
            case ROBOT_IDLE:
                er = ir_get_latest(&ir);

                if (er < E_OK) {
                    state = ROBOT_ERROR;
                    tm_printf(
                        (UB *)"[CTRL] IR read error: %d\n",
                        er
                    );
                    break;
                }

                if (!ir.valid) {
                    break;
                }

                er = tk_get_otm(&now);
                if (er < E_OK) {
                    state = ROBOT_ERROR;
                    break;
                }

                /* Only start after receiving fresh readings. */
                if ((UW)(now.lo - ir.timestamp_ms)
                    <= IR_MAX_AGE_MS) {
                    state = ROBOT_LINE_FOLLOWING;

                    tm_printf(
                        (UB *)"[CTRL] IDLE -> LINE_FOLLOWING\n"
                    );
                }
                break;
            case ROBOT_LINE_FOLLOWING:
                er = ir_get_latest(&ir);

                if (er < E_OK) {
                    state = ROBOT_ERROR;
                    motor_controller_apply(LINE_UNCERTAIN);
                    break;
                }

                er = tk_get_otm(&now);

                if (er < E_OK) {
                    state = ROBOT_ERROR;
                    motor_controller_apply(LINE_UNCERTAIN);
                    break;
                }

                /* Do not trust stale IR measurements. */
                if (!ir.valid ||
                    (UW)(now.lo - ir.timestamp_ms)
                        > IR_MAX_AGE_MS) {

                    state = ROBOT_STOPPED;
                    motor_controller_apply(LINE_UNCERTAIN);
                    tm_printf(
                        (UB *)"[CTRL] IR data stale -> STOPPED\n"
                    );
                    break;
                }

                LINE_ACTION action;

                action = line_follow_decide(
                    ir.left,
                    ir.right
                );

                if (action == LINE_UNCERTAIN) {
                    //state = ROBOT_STOPPED;
                    motor_controller_apply(LINE_UNCERTAIN);

                    tm_printf(
                        (UB *)"[CTRL] UNCERTAIN -> STOPPED\n"
                    );
                    break;
                }

                motor_controller_apply(action);

                /* Avoid repeated serial output. */
                if (!have_previous_action ||
                    action != previous_action) {
                    
                    tm_printf(
                        (UB *)"[CTRL] L=%u R=%u ACTION=%s\n",
                        ir.left,
                        ir.right,
                        line_follow_text(action)
                    );

                    previous_action = action;
                    have_previous_action = TRUE;
                }
                break;
            case ROBOT_STOPPED:
                /* Remain stopped until a future restart event. */
                break;
            case ROBOT_ERROR:
                /* Remain in error state until reset/recovery. */
                break;
            default:
                state = ROBOT_ERROR;
                motor_controller_apply(LINE_UNCERTAIN);
                break;
        }

        tk_dly_tsk(CONTROLLER_INTERVAL_MS);
    }
}