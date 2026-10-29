/** @file bc_setup.c
 * @brief Calibrated lifecycle and low-priority barcode diagnostics.
 */
#include "bc_setup.h"
#include "bc_cal.h"
#include "bc_port.h"
#include "bc_uart.h"
#include "bc_fast.h"
#include "app/tasks.h"

#if BC_CAL_READY
static void bc_setup_diag_task (INT start_code, void * p_context)
{
    (void)start_code;
    (void)p_context;

    for (;;)
    {
        (void)bc_fast_drain();
        (void)bc_port_process();
        (void)bc_uart_poll();
        tk_dly_tsk(BC_CAL_DIAG_PERIOD_MSEC);
    }
}
#endif

ER bc_setup_init (void)
{
#if BC_CAL_READY
    bc_port_config_t const config =
    {
        {
            BC_CAL_MIN_RUN_UNITS,
            BC_CAL_LEADING_UNITS,
            BC_CAL_TRAILING_UNITS,
            BC_CAL_MIN_BARCODE_UNITS,
            BC_CAL_MAX_BARCODE_UNITS,
            BC_CAL_TIMEOUT_MSEC
        },
        BC_CAL_LEFT_FORWARD_SIGN,
        BC_CAL_RIGHT_FORWARD_SIGN,
        BC_CAL_MAX_STEP_DIFFERENCE,
        BC_CAL_MAX_SAMPLE_UNITS,
        BC_CAL_MAX_SAMPLE_MSEC,
        BC_CAL_BLACK_LEVEL
    };

    ER error = bc_port_init(&config);
    if (error < E_OK)
    {
        return error;
    }
    return bc_fast_init();
#else
    return E_OK;
#endif
}

ER bc_setup_start (void)
{
#if BC_CAL_READY
    ID id = tasks_create(bc_setup_diag_task, BC_CAL_DIAG_PRIORITY);

    if (id <= E_OK)
    {
        return id;
    }
    return tk_sta_tsk(id, 0);
#else
    return E_OK;
#endif
}

ER bc_setup_sample (uint8_t raw_gpio, uint32_t now_msec)
{
#if BC_CAL_READY
    /* Existing 10 ms IR task only arms the hardware timer once.
     * Subsequent barcode readings come exclusively from TIMER ALARM2.
     */
    (void)raw_gpio;
    (void)now_msec;
    return bc_fast_arm();
#else
    (void)raw_gpio;
    (void)now_msec;
    return E_OK;
#endif
}

/*** end of file ***/

