/** @file bc_setup.c
 * @brief Barcode lifecycle, standalone result consumer and measurements.
 */
#include "bc_setup.h"
#include "bc_cal.h"
#include "bc_port.h"
#include "bc_uart.h"
#include "bc_fast.h"
#include "bc_fastq.h"
#include "app/tasks.h"
#include "app/app_config.h"
#include "platform/console.h"
#include <tm/tmonitor.h>

#if BC_CAL_READY
#define BC_MEASURE_LOG_USEC 500000U

typedef enum
{
    BC_SETUP_OFF,
    BC_SETUP_MEASURE,
    BC_SETUP_SCAN,
    BC_SETUP_FAILED
} bc_setup_mode_t;

typedef struct
{
    uint8_t ready;
    uint8_t level;
    uint8_t run_valid;
    int32_t left;
    int32_t right;
    uint32_t time_usec;
    uint32_t log_usec;
    uint32_t run_usec;
    uint32_t run_units;
    uint32_t max_gap_usec;
    uint32_t max_step_units;
    uint32_t max_skew_units;
    uint32_t invalid;
} bc_measure_t;

static uint8_t volatile g_setup_mode = BC_SETUP_OFF;
static ER volatile g_setup_error = E_OK;
static uint8_t gb_fast_ready = 0U;
static uint8_t volatile gb_worker_ready = 0U;
static uint8_t g_reported_mode = BC_SETUP_OFF;
static bc_measure_t g_measure;

static void bc_setup_fail (ER error)
{
    g_setup_error = error;
    g_setup_mode = BC_SETUP_FAILED;
    if (0U != gb_fast_ready)
    {
        bc_fast_stop();
    }
}

/** @brief Count absolute wheel travel without assuming encoder signs. */
static uint32_t bc_measure_step (int32_t count, int32_t previous)
{
    int64_t difference = (int64_t)count - previous;
    uint64_t size = (uint64_t)((difference < 0) ?
                               -difference : difference);
    return (size > UINT32_MAX) ? UINT32_MAX : (uint32_t)size;
}

static void bc_measure_sample (bc_fastq_sample_t const * p_sample)
{
    if (0U == g_measure.ready)
    {
        g_measure.ready = 1U;
        g_measure.log_usec = p_sample->time_usec;
        g_measure.run_usec = p_sample->time_usec;
        g_measure.run_valid = 1U;
        g_measure.level = p_sample->gpio_level;
    }
    else
    {
        uint32_t gap = p_sample->time_usec - g_measure.time_usec;
        uint32_t left = bc_measure_step(p_sample->left_count,
                                        g_measure.left);
        uint32_t right = bc_measure_step(p_sample->right_count,
                                         g_measure.right);
        uint64_t total = (uint64_t)left + right;
        uint32_t step = (total > UINT32_MAX) ?
                        UINT32_MAX : (uint32_t)total;
        uint32_t skew = (left > right) ? left - right : right - left;
        if (gap > g_measure.max_gap_usec)
        {
            g_measure.max_gap_usec = gap;
        }
        if (step > g_measure.max_step_units)
        {
            g_measure.max_step_units = step;
        }
        if (skew > g_measure.max_skew_units)
        {
            g_measure.max_skew_units = skew;
        }
        if (step <= UINT32_MAX - g_measure.run_units)
        {
            g_measure.run_units += step;
        }
        else
        {
            g_measure.run_valid = 0U;
        }
    }
    if (0U == p_sample->valid)
    {
        if (g_measure.invalid < UINT32_MAX)
        {
            ++g_measure.invalid;
        }
        g_measure.run_valid = 0U;
    }
    if (p_sample->gpio_level != g_measure.level)
    {
        tm_printf((UB *)"[BC CAL EDGE] level=%u units=%u us=%u ok=%u\n",
                  (UINT)g_measure.level, (UINT)g_measure.run_units,
                  (UINT)(p_sample->time_usec - g_measure.run_usec),
                  (UINT)g_measure.run_valid);
        g_measure.level = p_sample->gpio_level;
        g_measure.run_units = 0U;
        g_measure.run_usec = p_sample->time_usec;
        g_measure.run_valid = p_sample->valid;
    }
    g_measure.left = p_sample->left_count;
    g_measure.right = p_sample->right_count;
    g_measure.time_usec = p_sample->time_usec;
}

static void bc_measure_poll (void)
{
    bc_fastq_sample_t sample = {0};
    for (uint32_t i = 0U; i < BC_FASTQ_CAPACITY; ++i)
    {
        if (0U != bc_fastq_pop(&sample))
        {
            bc_measure_sample(&sample);
        }
    }
    if ((0U != g_measure.ready) &&
        ((uint32_t)(g_measure.time_usec - g_measure.log_usec) >=
         BC_MEASURE_LOG_USEC))
    {
        tm_printf((UB *)"[BC CAL] raw=%u L=%d R=%d gap_us=%u "
                          "step=%u skew=%u invalid=%u\n",
                  (UINT)g_measure.level, (INT)g_measure.left,
                  (INT)g_measure.right, (UINT)g_measure.max_gap_usec,
                  (UINT)g_measure.max_step_units,
                  (UINT)g_measure.max_skew_units, (UINT)g_measure.invalid);
        g_measure.log_usec = g_measure.time_usec;
    }
}

static void bc_setup_diag_task (INT start_code, void * p_context)
{
    (void)start_code;
    (void)p_context;
    for (INT attempt = 0;
         (attempt < APP_USB_WAIT_ATTEMPTS) && (console_state() < 2U);
         ++attempt)
    {
        tk_dly_tsk(APP_USB_WAIT_PERIOD_MSEC);
    }
    tk_dly_tsk(APP_USB_SETTLE_MSEC);
    for (;;)
    {
        (void)bc_setup_poll();
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
    ER error = E_PAR;
    if (0 == BC_CAL_MEASURE)
    {
        error = bc_port_init(&config);
    }
    if ((error < E_OK) && (E_PAR != error))
    {
        bc_setup_fail(error);
    }
    else
    {
        g_setup_mode = (E_OK == error) ? BC_SETUP_SCAN : BC_SETUP_MEASURE;
        error = bc_fast_init();
        if (error < E_OK)
        {
            bc_setup_fail(error);
        }
        else
        {
            gb_fast_ready = 1U;
        }
    }
#endif
    /* A barcode failure must not trigger usermain's shutdown path. */
    return E_OK;
}

ER bc_setup_start (void)
{
#if BC_CAL_READY
    ID id = tasks_create(bc_setup_diag_task, BC_CAL_DIAG_PRIORITY);
    ER error = (id <= E_OK) ? ((E_OK == id) ? E_OBJ : id) :
                             tk_sta_tsk(id, 0);
    if (error < E_OK)
    {
        bc_setup_fail(error);
        tm_printf((UB *)"[BARCODE] Worker unavailable: %d; disabled\n",
                  error);
    }
#endif
    return E_OK;
}

ER bc_setup_poll (void)
{
#if BC_CAL_READY
    gb_worker_ready = 1U;
    if ((0U != gb_fast_ready) && (bc_fast_error() < E_OK) &&
        (BC_SETUP_FAILED != g_setup_mode))
    {
        bc_setup_fail(bc_fast_error());
    }
    if (g_reported_mode != g_setup_mode)
    {
        g_reported_mode = g_setup_mode;
        if (BC_SETUP_MEASURE == g_setup_mode)
        {
            tm_printf((UB *)"[BARCODE] Measurement mode; decoding off\n");
        }
        else if (BC_SETUP_SCAN == g_setup_mode)
        {
            tm_printf((UB *)"[BARCODE] Scanning enabled\n");
        }
        else if (BC_SETUP_FAILED == g_setup_mode)
        {
            tm_printf((UB *)"[BARCODE] Disabled: error %d; robot continues\n",
                      g_setup_error);
        }
    }
    if (BC_SETUP_MEASURE == g_setup_mode)
    {
        bc_measure_poll();
    }
    else if (BC_SETUP_SCAN == g_setup_mode)
    {
        (void)bc_fast_drain();
        (void)bc_port_process();
        (void)bc_uart_poll();
#if BC_CAL_AUTO_ACK
        bc_port_result_t result = {0};
        if (E_OK == bc_port_peek(&result))
        {
            (void)bc_port_ack(result.sequence);
        }
#endif
    }
    else if (BC_SETUP_FAILED == g_setup_mode)
    {
        return g_setup_error;
    }
#endif
    return E_OK;
}

ER bc_setup_sample (uint8_t raw_gpio, uint32_t now_msec)
{
    (void)raw_gpio;
    (void)now_msec;
#if BC_CAL_READY
    if ((0U != gb_worker_ready) &&
        ((BC_SETUP_SCAN == g_setup_mode) ||
         (BC_SETUP_MEASURE == g_setup_mode)))
    {
        ER error = bc_fast_arm();
        if (error < E_OK)
        {
            bc_setup_fail(error);
        }
    }
#endif
    return E_OK;
}

/*** end of file ***/

