/** @file bc_setup.c
 * @brief Barcode lifecycle, standalone result consumer and measurements.
 */
#include "bc_setup.h"
#include "bc_cal.h"
#include "bc_port.h"
#include "bc_uart.h"
#include "bc_fast.h"
#include "bc_fastq.h"
#include "bc_trace.h"
#include "bc_wave.h"
#include "app/tasks.h"
#include "app/app_config.h"
#include "platform/console.h"
#include "drivers/encoder.h"
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
    uint32_t adc_raw;
    uint32_t adc_min;
    uint32_t adc_max;
    uint32_t adc_samples;
    uint32_t adc_errors;
} bc_measure_t;

static uint8_t volatile g_setup_mode = BC_SETUP_OFF;
static ER volatile g_setup_error = E_OK;
static uint8_t gb_fast_ready = 0U;
static uint8_t volatile gb_worker_ready = 0U;
static uint8_t g_reported_mode = BC_SETUP_OFF;
static bc_measure_t g_measure;
static uint32_t g_input_report_usec;
#if !APP_BARCODE_ANALOG_ENABLED || !APP_BARCODE_BUFFERED_ENABLED
static uint32_t g_reported_event;
static uint32_t g_frame_report_usec;

static void bc_setup_frame_report (bc_port_stats_t const * p_stats,
                                   uint32_t now)
{
    if (g_reported_event != p_stats->last_event)
    {
        g_reported_event = p_stats->last_event;
        if ((BC_SCAN_ERROR_INPUT != p_stats->last_error) ||
            ((uint32_t)(now - g_frame_report_usec) >= 1000000U))
        {
            char line[384] = {0};
            uint32_t used = 0U;
            g_frame_report_usec = now;
            tm_printf((UB *)"[BC FRAME] event=%u capture_error=%u "
                              "runs=%u/29 travel_units=%u\n",
                      (UINT)p_stats->last_event,
                      (UINT)p_stats->last_error,
                      (UINT)p_stats->last_runs,
                      (UINT)p_stats->last_scan_units);
            for (uint32_t i = 0U;
                 (i < p_stats->last_runs) && (i < BC_CODE_ELEMENTS); ++i)
            {
                char digits[10] = {0};
                uint32_t count = 0U;
                uint32_t value = p_stats->last_widths[i];
                do
                {
                    digits[count++] = (char)('0' + value % 10U);
                    value /= 10U;
                } while (value > 0U);
                if (used + count + 1U >= sizeof(line))
                {
                    break;
                }
                if (i > 0U)
                {
                    line[used++] = ',';
                }
                for (; count > 0U; --count)
                {
                    line[used++] = digits[count - 1U];
                }
                line[used] = '\0';
            }
            tm_printf((UB *)"[BC WIDTHS] event=%u %s\n",
                      (UINT)p_stats->last_event, line);
        }
    }
}
#endif

#if APP_BARCODE_ANALOG_ENABLED
static void bc_setup_adc_report (void)
{
    bc_fast_adc_stats_t adc = {0};
    bc_fast_adc_report(&adc);
    tm_printf((UB *)"[BC ADC] raw=%u min=%u max=%u n=%u errors=%u "
                      "filtered=%u threshold=%u hysteresis=%u black_high=%u\n",
              (UINT)adc.latest, (UINT)adc.minimum, (UINT)adc.maximum,
              (UINT)adc.samples, (UINT)adc.errors,
              (UINT)adc.filtered_spikes,
              (UINT)APP_BARCODE_ADC_THRESHOLD,
              (UINT)APP_BARCODE_ADC_HYSTERESIS,
              (UINT)APP_BARCODE_ADC_BLACK_HIGH);
}
#endif

#if !APP_BARCODE_ANALOG_ENABLED || !APP_BARCODE_BUFFERED_ENABLED
static void bc_setup_input_report (void)
{
    uint32_t now = encoder_now_usec();
    bc_port_stats_t stats = {0};
    if (E_OK != bc_port_stats(&stats))
    {
        return;
    }
    bc_setup_frame_report(&stats, now);
    if ((uint32_t)(now - g_input_report_usec) >= 1000000U)
    {
        g_input_report_usec = now;
#if APP_BARCODE_ANALOG_ENABLED
        bc_setup_adc_report();
#endif
        {
            tm_printf((UB *)"[BC INPUT] raw=%u n=%u irq=%u gap=%u "
                              "dir=%u skew=%u step=%u stale=%u lock=%u "
                              "max_gap_us=%u\n",
                      (UINT)stats.raw, (UINT)stats.samples,
                      (UINT)stats.capture_invalid, (UINT)stats.gap_invalid,
                      (UINT)stats.direction_invalid,
                      (UINT)stats.skew_invalid, (UINT)stats.step_invalid,
                      (UINT)stats.stale_edge, (UINT)stats.lock_lost,
                      (UINT)stats.max_gap_usec);
            tm_printf((UB *)"[BC SCAN] edges=%u state=%u runs=%u/29 "
                              "travel_units=%u\n",
                      (UINT)stats.edges, (UINT)stats.state,
                      (UINT)stats.runs, (UINT)stats.scan_units);
        }
    }
}
#endif

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
#if APP_BARCODE_ANALOG_ENABLED
            if (0U != sample.adc_valid)
            {
                if (0U == g_measure.adc_samples)
                {
                    g_measure.adc_min = sample.adc_raw;
                    g_measure.adc_max = sample.adc_raw;
                }
                if (sample.adc_raw < g_measure.adc_min)
                {
                    g_measure.adc_min = sample.adc_raw;
                }
                if (sample.adc_raw > g_measure.adc_max)
                {
                    g_measure.adc_max = sample.adc_raw;
                }
                g_measure.adc_raw = sample.adc_raw;
                ++g_measure.adc_samples;
            }
            else
            {
                ++g_measure.adc_errors;
            }
#endif
            bc_measure_sample(&sample);
        }
    }
    if ((0U != g_measure.ready) &&
        ((uint32_t)(g_measure.time_usec - g_measure.log_usec) >=
         BC_MEASURE_LOG_USEC))
    {
#if APP_BARCODE_ANALOG_ENABLED
        tm_printf((UB *)"[BC ADC] raw=%u min=%u max=%u n=%u errors=%u "
                          "threshold=%u hysteresis=%u black_high=%u\n",
                  (UINT)g_measure.adc_raw, (UINT)g_measure.adc_min,
                  (UINT)g_measure.adc_max, (UINT)g_measure.adc_samples,
                  (UINT)g_measure.adc_errors,
                  (UINT)APP_BARCODE_ADC_THRESHOLD,
                  (UINT)APP_BARCODE_ADC_HYSTERESIS,
                  (UINT)APP_BARCODE_ADC_BLACK_HIGH);
        g_measure.adc_min = 0U;
        g_measure.adc_max = 0U;
        g_measure.adc_samples = 0U;
        g_measure.adc_errors = 0U;
#endif
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
#if APP_BARCODE_ANALOG_ENABLED
#if APP_BARCODE_BUFFERED_ENABLED
            tm_printf((UB *)"[BARCODE] Buffered waveform mode; "
                              "start on white; fixed threshold compares\n");
#ifdef B1_LIVE_COMMANDS
            tm_printf((UB *)"[BARCODE] Continuous scan auto rearm; "
                              "motor owned by navigation controller\n");
#else
            tm_printf((UB *)"[BARCODE] One diagnostic trial per boot\n");
#endif
#else
            tm_printf((UB *)"[BARCODE] Analog AO on GP27/ADC1; "
                              "software threshold enabled\n");
#endif
#if APP_BARCODE_TRACE_ENABLED
            tm_printf((UB *)"[BARCODE] Buffered trace armed; "
                              "one pass per boot, wait for TRACE END\n");
#endif
#if APP_BARCODE_EDGE_FILTER_ENABLED && !APP_BARCODE_BUFFERED_ENABLED
            tm_printf((UB *)"[BARCODE] Two-sample edge filter enabled; "
                              "first-sample position retained\n");
#endif
#endif
#if BC_CAL_TEST_IGNORE_INPUT_ERRORS
            tm_printf((UB *)"[BARCODE] TEST MODE: "
                              "input validity checks bypassed\n");
#endif
#if BC_CAL_TEST_RELAXED_CAPTURE
            tm_printf((UB *)"[BARCODE] TEST MODE: "
                              "broad capture; separate bar/space decoding\n");
#endif
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
#if APP_BARCODE_ANALOG_ENABLED && APP_BARCODE_BUFFERED_ENABLED
        bc_wave_poll();
        uint32_t now = encoder_now_usec();
        if ((uint32_t)(now - g_input_report_usec) >= 1000000U)
        {
            g_input_report_usec = now;
            bc_setup_adc_report();
        }
#else
        bc_setup_input_report();
#endif
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
#if APP_BARCODE_ANALOG_ENABLED && APP_BARCODE_TRACE_ENABLED
    if (BC_SETUP_SCAN == g_setup_mode)
    {
        bc_trace_poll();
    }
#endif
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

