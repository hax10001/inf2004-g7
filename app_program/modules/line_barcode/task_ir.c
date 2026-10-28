/** @file task_ir.c
 * @brief Bounded three-IR sampling and perception loop (task context).
 */
#include "task_ir.h"
#include "ir_shared.h"
#include "line_barcode.h"
#include "drivers/ir.h"
#include <stdbool.h>
#include <stdint.h>

/* Requested 4 ms, but current kernel is configured for 10 ms SysTick.
 * The scheduler cannot guarantee 4 ms until the kernel timer is reviewed.
 * The team must validate the actual interval before physical scanning.
 */
#define B3_IR_REQUEST_PERIOD_MSEC 4U

void task_ir_task (INT start_code, void * p_context)
{
    bool b_sleep_on_error = true;
    ER error = E_OK;

    (void)start_code;
    (void)p_context;
    error = ir_init(&b_sleep_on_error);
    if (error < E_OK)
    {
        (void)b_sleep_on_error;
        (void)line_barcode_report_fault();
        return;
    }
    for (;;)
    {
        uint32_t left = 0U;
        uint32_t right = 0U;
        uint32_t barcode = 0U;
        uint8_t bits = 0U;

        ir_read(&left, &right, &barcode);
        if (((left | right | barcode) & ~1U) != 0U)
        {
            /* Invalid electrical reading: do not publish or steer. */
            (void)line_barcode_report_fault();
            return;
        }
        error = ir_shared_publish(left, right, barcode);
        if (error < E_OK)
        {
            /* A failed update must not be mistaken for a fresh sample. */
            (void)line_barcode_report_fault();
            return;
        }
        bits = (uint8_t)(left | (right << 1U) | (barcode << 2U));
        error = line_barcode_process(bits);
        if (error < E_OK)
        {
            (void)line_barcode_report_fault();
            return;
        }
        (void)tk_dly_tsk(B3_IR_REQUEST_PERIOD_MSEC);
    }
}

/*** end of file ***/

