/** @file task_ir.c
 * @brief Periodic sampling; GPIO operations remain in the IR driver.
 */
#include "task_ir.h"
#include "ir_shared.h"
#include "drivers/ir.h"
#include "app/app_config.h"
#include "modules/barcode_scanning/bc_setup.h"
#include "modules/barcode_scanning/bc_cal.h"
#include "platform/console.h"
#include <tm/tmonitor.h>

void task_ir_task (INT start_code, void * p_context)
{
    /* Casting unused arguments to void cannot alter their values. */
    (void)start_code;
    (void)p_context;

    for (INT attempt = 0;
         (attempt < APP_USB_WAIT_ATTEMPTS) && (console_state() < 2U); attempt++)
    {
        tk_dly_tsk(APP_USB_WAIT_PERIOD_MSEC);
    }

    tk_dly_tsk(APP_USB_SETTLE_MSEC);

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"\n=== RoboPico IR Task ===\n");

    bool b_sleep_on_error = true;
    ER   error            = ir_init(&b_sleep_on_error);

    if (error < E_OK)
    {
        if (b_sleep_on_error)
        {
            tk_slp_tsk(TMO_FEVR);
        }

        return;
    }

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"IR sensors initialized.\n");

    for (;;)
    {
        uint32_t left_raw    = 0U;
        uint32_t right_raw   = 0U;
        uint32_t barcode_raw = 0U;

        ir_read(&left_raw, &right_raw, &barcode_raw);
        error = ir_shared_publish(left_raw, right_raw, barcode_raw);

#if BC_CAL_READY
        /* Capture one GP27 reading; never decode or print here. */
        if (error >= E_OK)
        {
            SYSTIM sample_time = {0};

            if (tk_get_otm(&sample_time) >= E_OK)
            {
                (void)bc_setup_sample((uint8_t)barcode_raw,
                                      sample_time.lo);
            }
        }
#endif

        if (error < E_OK)
        {
            /* UB views the same bytes; tm_printf does not modify them. */
            tm_printf((UB *)"[IR] Publish error: %d\n", error);
        }

        tk_dly_tsk(APP_IR_PERIOD_MSEC);
    }
}

/*** end of file ***/

