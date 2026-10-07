#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include <stdbool.h>
#include "usb_console_compat.h"
#include "ir_task.h"
#include "ir_shared.h"

#define IR_LEFT_PIN 6
#define IR_RIGHT_PIN 2
// This pin is currently conflicting with another device using GP26 as well.
#define IR_BARCODE_PIN 27

/* -------------------------------------------------
 * IR application task - IR sensor monitoring
 * ------------------------------------------------- */

EXPORT void ir_task(INT stacd, void *exinf)
{
    INT i;
    ER er;

    (void)stacd;
    (void)exinf;

    /* Wait for USB CDC connection */
    for (i = 0; i < 200 && tm_usb_state() < 2; i++) {
        tk_dly_tsk(100);
    }

    tk_dly_tsk(500);

    tm_printf((UB *)"\n=== RoboPico IR Task ===\n");

    /* Initialize Grove 1 IR sensor */
    er = gpio_set_pin(IR_LEFT_PIN, GPIO_MODE_IN);

    if (er < E_OK) {
        tm_printf((UB *)"IR LEFT initialization failed: %d\n", er);
        tk_slp_tsk(TMO_FEVR);
        return;
    }

    /* Initialize Grove 6 */
    er = gpio_set_pin(IR_BARCODE_PIN, GPIO_MODE_IN);

    if (er < E_OK) {
        tm_printf((UB *)"IR BARCODE initialization failed: %d\n", er);
        return;
    }

    /* Initialize Grove 7 IR sensor */
    er = gpio_set_pin(IR_RIGHT_PIN, GPIO_MODE_IN);

    if (er < E_OK) {
        tm_printf((UB *)"IR RIGHT initialization failed: %d\n", er);
        tk_slp_tsk(TMO_FEVR);
        return;
    }

    tm_printf((UB *)"IR sensors initialized.\n");

    /* Main IR monitoring loop */
    while (1) {
        UINT left_raw  = gpio_get_val(IR_LEFT_PIN);  // GP6 = LEFT side of robot (this is temporary as Gp26 is still not useable need replace the IR sensor.)
        UINT right_raw = gpio_get_val(IR_RIGHT_PIN);  // GP2 = RIGHT side of robot
        UINT barcode_raw = gpio_get_val(IR_BARCODE_PIN); // GP26 = BARCODE sensor

        // Pass the readings to ir_shared for the controller to read from.
        ER pub_er = ir_shared_publish(left_raw, right_raw, barcode_raw);

        if (pub_er < E_OK) {

            tm_printf(
                (UB *)"[IR] Publish error: %d\n",
                pub_er
            );
        }

        tk_dly_tsk(100);
    }
}