/** @file line_barcode.c
 * @brief Barcode service using the existing IR task and shared mutex.
 */
#include "line_barcode.h"
#include "ir_shared.h"
#include "task_ir.h"
#include "bc_config.h"
#include "drivers/encoder.h"
#include "app/app_config.h"
#include "app/tasks.h"
#include <limits.h>
#if BC_DIAGNOSTICS
#include <tm/tmonitor.h>
#endif

/* Access after startup is protected by the existing IR mutex. The scanner
 * allocates no extra RTOS object: all four configured mutexes are in use.
 */
static bc_scan_t g_scan;
static barcode_result_t g_result;
static barcode_diagnostic_t g_diagnostic;
static barcode_failure_t g_failure;
static bool gb_have_failure = false;
static bool gb_configured = false;
static bool gb_enabled = true;
static bool gb_pending = false;
static bool gb_measured = false;
static struct
{
    uint32_t b_avoidance : 1;
} g_motion_gate = {0U};
static uint32_t g_sequence = 0U;
static uint32_t g_measure_run = 0U;
#if BC_DIAGNOSTICS
static uint32_t g_printed_failure = 0U;
static void log_failure (void);
static char const * failure_reason (barcode_failure_t const * p_failure);
static char const * decimal_u64 (uint64_t value, char digits[21]);
#endif

static uint32_t absolute_delta (int32_t current, int32_t previous);
static void measure_sample (uint32_t raw, int32_t left, int32_t right,
                            uint32_t now_usec);
static void save_failure (bc_state_t failed_state, uint32_t now_usec);

ER line_barcode_init (void)
{
    ER const error = ir_shared_init();

    if (error < E_OK)
    {
        return error;
    }

    bc_scan_config_t const config = {
        .black_level = BC_BLACK_LEVEL,
        .left_sign = BC_LEFT_FORWARD_SIGN,
        .right_sign = BC_RIGHT_FORWARD_SIGN,
        .max_gap_usec = BC_MAX_GAP_USEC,
        .timeout_usec = BC_SCAN_TIMEOUT_USEC,
        .stop_usec = BC_STOP_TIMEOUT_USEC,
        .max_skew_units = BC_MAX_SKEW_UNITS,
        .max_skew_percent = BC_MAX_SKEW_PERCENT};

    bool const b_valid = bc_scan_init(&g_scan, &config);

    gb_configured = (0 != BC_CAL_READY) && b_valid;
    gb_enabled = true;
    gb_pending = false;
    gb_measured = false;
    g_motion_gate.b_avoidance = false;
    g_sequence = 0U;
    g_measure_run = 0U;
    g_result = (barcode_result_t){0};
    g_diagnostic = (barcode_diagnostic_t){0};
    g_failure = (barcode_failure_t){0};
    gb_have_failure = false;
#if BC_DIAGNOSTICS
    g_printed_failure = 0U;
#endif
    g_diagnostic.mode = gb_configured ? BARCODE_ENABLED : BARCODE_MEASURING;

    /* Unknown barcode settings must never abort working robot startup. */
    return E_OK;
}

ER line_barcode_start (void)
{
    ID h_task = tasks_create(task_ir_task, APP_IR_PRIORITY);

    if (h_task <= E_OK)
    {
        /* Keep the original rejection of a zero task handle. */
        return (E_OK == h_task) ? E_OBJ : h_task;
    }

    return tk_sta_tsk(h_task, 0);
}

ER line_barcode_get_result (barcode_result_t * p_result)
{
    if (NULL == p_result)
    {
        return E_PAR;
    }

    ER const error = ir_shared_lock(TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    ER result = E_OBJ;

    if (gb_pending)
    {
        *p_result = g_result;
        result = E_OK;
    }

    ER const unlock_error = ir_shared_unlock();

    return (unlock_error < E_OK) ? unlock_error : result;
}

ER line_barcode_ack (uint32_t sequence)
{
    ER const error = ir_shared_lock(TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    ER result = E_OBJ;

    if (gb_pending && (sequence == g_result.sequence))
    {
        gb_pending = false;
        bc_scan_reset(&g_scan);
        result = E_OK;
    }

    ER const unlock_error = ir_shared_unlock();

    return (unlock_error < E_OK) ? unlock_error : result;
}

ER line_barcode_set_enabled (bool b_enabled)
{
    ER const error = ir_shared_lock(TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    ER result = E_OK;

    if (b_enabled && !gb_configured)
    {
        result = E_PAR;
    }
    else if (gb_enabled != b_enabled)
    {
        gb_enabled = b_enabled;
        bc_scan_reset(&g_scan);
    }

    ER const unlock_error = ir_shared_unlock();

    return (unlock_error < E_OK) ? unlock_error : result;
}

ER line_barcode_set_avoidance (bool b_active)
{
    ER const error = ir_shared_lock(TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    if (g_motion_gate.b_avoidance != b_active)
    {
        g_motion_gate.b_avoidance = b_active;
        bc_scan_reset(&g_scan);
        gb_measured = false;
    }

    return ir_shared_unlock();
}

ER line_barcode_get_diagnostic (barcode_diagnostic_t * p_output)
{
    if (NULL == p_output)
    {
        return E_PAR;
    }

    ER const error = ir_shared_lock(TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    *p_output = g_diagnostic;
    p_output->mode = (!gb_enabled || g_motion_gate.b_avoidance) ?
                     BARCODE_DISABLED :
                     (gb_configured ? BARCODE_ENABLED : BARCODE_MEASURING);
    p_output->state = g_scan.state;
    p_output->status = g_scan.status;
    p_output->run_count = g_scan.run_count;
    for (uint32_t index = 0U; index < BC_RUN_COUNT; index++)
    {
        p_output->runs[index] = g_scan.runs[index];
    }
    return ir_shared_unlock();
}

ER line_barcode_get_failure (barcode_failure_t * p_output)
{
    if (NULL == p_output)
    {
        return E_PAR;
    }

    ER const error = ir_shared_lock(TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    ER result = E_OBJ;

    if (gb_have_failure)
    {
        *p_output = g_failure;
        result = E_OK;
    }

    ER const unlock_error = ir_shared_unlock();

    return (unlock_error < E_OK) ? unlock_error : result;
}

ER line_barcode_ack_failure (uint32_t sequence)
{
    ER const error = ir_shared_lock(TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    ER result = E_OBJ;

    if (gb_have_failure && g_failure.b_pending &&
        (sequence == g_failure.sequence))
    {
        g_failure.b_pending = false;
        result = E_OK;
    }

    ER const unlock_error = ir_shared_unlock();

    return (unlock_error < E_OK) ? unlock_error : result;
}

void line_barcode_sample (uint32_t barcode_raw, uint32_t sample_usec)
{
    uint32_t const now_usec = sample_usec;
    encoder_sample_t left = {0};
    encoder_sample_t right = {0};

    encoder_read(ENCODER_LEFT, &left);
    encoder_read(ENCODER_RIGHT, &right);
    uint32_t const read_usec = encoder_now_usec() - now_usec;

    /* A busy consumer must not add a wait before publishing line sensors.
     * The next successful sample detects the enlarged sampling gap.
     */
    if (ir_shared_lock(0) < E_OK)
    {
        return;
    }

    if (!g_motion_gate.b_avoidance)
    {
        measure_sample(barcode_raw, left.count, right.count, now_usec);
    }
    if (!g_motion_gate.b_avoidance && gb_enabled &&
        gb_configured && !gb_pending)
    {
        bc_state_t const previous_state = g_scan.state;
        bool const b_active = (BC_SCANNING == previous_state) ||
                              (BC_TRAILING_WHITE == previous_state);
        uint32_t const raw = (read_usec <= BC_MAX_READ_USEC) ?
                             barcode_raw : 2U;
        bool const b_complete = bc_scan_feed(&g_scan, raw, left.count,
                                             right.count, now_usec);

        if (b_active && (BC_ERROR == g_scan.state) && !g_scan.b_seeded)
        {
            g_diagnostic.rejected++;
            save_failure(previous_state, now_usec);
        }
        if (b_complete)
        {
            SYSTIM now = {0};

            /* Timestamp failure discards this frame, not IR operation. */
            if (tk_get_otm(&now) >= E_OK)
            {
                g_sequence++;
                g_result.letter = g_scan.decoded.letter;
                g_result.status = BC_OK;
                g_result.sequence = g_sequence;
                g_result.timestamp_msec = now.lo;
                g_result.narrow_units = g_scan.decoded.narrow_units;
                g_result.width_units = g_scan.decoded.width_units;
                float const units_per_mm = BC_UNITS_PER_MM;

                g_result.b_distance_valid = units_per_mm > 0.0f;
                g_result.width_mm = g_result.b_distance_valid ?
                    (float)g_result.width_units / units_per_mm : 0.0f;
                gb_pending = true;
                g_diagnostic.completed++;
            }
            else
            {
                bc_scan_reset(&g_scan);
            }
        }
    }

    (void)ir_shared_unlock();
}

void line_barcode_log (void)
{
#if BC_DIAGNOSTICS
    /* This function has a single caller: task_ir after snapshot publish. */
    static uint32_t last_log_usec = 0U;
    static uint32_t last_sequence = 0U;
    barcode_diagnostic_t reading = {0};

    if (line_barcode_get_diagnostic(&reading) < E_OK)
    {
        return;
    }
    if ((BC_SCANNING == reading.state) ||
        (BC_TRAILING_WHITE == reading.state))
    {
        return;
    }

    /* Print a terminal failure immediately, once, outside capture/locks.
     * Do not throttle it with the periodic summary: another attempt could
     * otherwise begin before its report has been consumed.
     */
    log_failure();
    if ((reading.timestamp_usec - last_log_usec) < BC_LOG_PERIOD_USEC)
    {
        return;
    }

    last_log_usec = reading.timestamp_usec;
    /* UB views string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[BC] mode=%u state=%u raw=%u "
           "L=%d R=%d gap=%u step=%u skew=%u "
           "runs=%u rejected=%u completed=%u err=%u "
           "fail_seq=%u fail_err=%u dropped=%u\n",
            (UINT)reading.mode,
            (UINT)reading.state,
            (UINT)reading.raw,
            (INT)reading.left_count,
            (INT)reading.right_count,
            (UINT)reading.max_gap_usec,
            (UINT)reading.max_step_units,
            (UINT)reading.max_skew_units,
            (UINT)reading.run_count,
            (UINT)reading.rejected,
            (UINT)reading.completed,
            (UINT)reading.status,
            (UINT)reading.failure_sequence,
            (UINT)reading.failure_status,
            (UINT)reading.failure_reports_dropped);

    barcode_result_t result = {0};

    if ((E_OK == line_barcode_get_result(&result)) &&
        (result.sequence != last_sequence))
    {
        last_sequence = result.sequence;
        tm_printf((UB *)"[BC RESULT] %c seq=%u narrow=%u width=%u\n",
                  (INT)result.letter, (UINT)result.sequence,
                  (UINT)result.narrow_units, (UINT)result.width_units);
    }
#endif
}

static void save_failure (bc_state_t failed_state, uint32_t now_usec)
{
    /* A failure outside an active attempt must not relabel old run data.
     * The caller increments rejected once on the active-to-error edge.
     */
    if (gb_have_failure && g_failure.b_pending)
    {
        g_diagnostic.failure_reports_dropped++;
        return;
    }

    g_failure = (barcode_failure_t){0};
    g_failure.decode = g_scan.decode_report;
    g_failure.status = g_scan.status;
    g_failure.failed_state = failed_state;
    g_failure.sequence = g_diagnostic.rejected;
    g_failure.timestamp_usec = now_usec;
    g_failure.start_usec = g_scan.start_usec;
    g_failure.run_count = g_scan.run_count;
    for (uint32_t index = 0U; index < BC_RUN_COUNT; index++)
    {
        if (index < g_scan.run_count)
        {
            g_failure.runs[index] = g_scan.runs[index];
        }
    }
    g_failure.max_step_units = g_scan.max_step_units;
    g_failure.approach_max_step = g_scan.approach_max_step;
    g_failure.barcode_max_step = g_scan.barcode_max_step;
    g_failure.trailing_max_step = g_scan.trailing_max_step;
    g_failure.max_confirm_usec = g_scan.max_confirm_usec;
    g_failure.max_confirm_units = g_scan.max_confirm_units;
    g_failure.unconfirmed_edges = g_scan.unconfirmed_edges;
    g_failure.start_level = g_scan.start_level;
    g_failure.black_level = g_scan.config.black_level;
    g_failure.b_started_black =
        g_scan.start_level == g_scan.config.black_level;
    g_failure.b_from_white =
        g_scan.start_from_level != g_scan.config.black_level;
    g_failure.rejected = g_diagnostic.rejected;
    g_failure.completed = g_diagnostic.completed;
    g_failure.b_pending = true;
    gb_have_failure = true;
    g_diagnostic.failure_sequence = g_failure.sequence;
    g_diagnostic.failure_status = g_failure.status;
}

#if BC_DIAGNOSTICS
static void log_failure (void)
{
    barcode_failure_t failure = {0};

    if ((line_barcode_get_failure(&failure) < E_OK) || !failure.b_pending)
    {
        return;
    }
    if (g_printed_failure == failure.sequence)
    {
        (void)line_barcode_ack_failure(failure.sequence);
        return;
    }

    /* Each short line releases the console's interrupt-masking lock.
     * The capture routine and encoder ISR never print or format text.
     */
    tm_printf((UB *)"[BC FAIL] reason=%s err=%u seq=%u state=%u "
              "runs=%u rejected=%u completed=%u\n",
              failure_reason(&failure), (UINT)failure.status,
              (UINT)failure.sequence, (UINT)failure.failed_state,
              (UINT)failure.run_count, (UINT)failure.rejected,
              (UINT)failure.completed);
    tm_printf((UB *)"max_step=%u approach_max=%u barcode_max=%u "
              "trailing_max=%u\n", (UINT)failure.max_step_units,
              (UINT)failure.approach_max_step, (UINT)failure.barcode_max_step,
              (UINT)failure.trailing_max_step);
    tm_printf((UB *)"start_black=%u from_white=%u start_raw=%u "
              "expected_black=%u start_us=%u end_us=%u\n",
              (UINT)failure.b_started_black, (UINT)failure.b_from_white,
              (UINT)failure.start_level, (UINT)failure.black_level,
              (UINT)failure.start_usec, (UINT)failure.timestamp_usec);
    tm_printf((UB *)"confirm_max_us=%u confirm_max_units=%u "
              "unconfirmed_edges=%u decoder=%s metrics=%u\n",
              (UINT)failure.max_confirm_usec, (UINT)failure.max_confirm_units,
              (UINT)failure.unconfirmed_edges,
              bc_check_name(failure.decode.check),
              (UINT)failure.decode.b_metrics_valid);

    if (failure.decode.b_metrics_valid)
    {
        char minimum[21];
        char separation[21];
        char narrow[21];
        char wide[21];

        tm_printf((UB *)"decoder_step=%u min_width=%u required_min=%s\n",
                  (UINT)failure.decode.max_step_units,
                  (UINT)failure.decode.min_width,
                  decimal_u64(failure.decode.required_min, minimum));
        tm_printf((UB *)"narrow_max=%u wide_min=%u separation=%u "
                  "required_separation_gt=%s\n",
                  (UINT)failure.decode.narrow_max,
                  (UINT)failure.decode.wide_min,
                  (UINT)(failure.decode.wide_min - failure.decode.narrow_max),
                  decimal_u64(failure.decode.required_separation, separation));
        tm_printf((UB *)"narrow_sum=%s narrow_count=20 "
                  "wide_sum=%s wide_count=9\n",
                  decimal_u64(failure.decode.narrow_sum, narrow),
                  decimal_u64(failure.decode.wide_sum, wide));
        tm_printf((UB *)"start_pattern=0x%03x letter_pattern=0x%03x "
                  "stop_pattern=0x%03x expected_star=0x094\n",
                  (UINT)failure.decode.start_pattern,
                  (UINT)failure.decode.letter_pattern,
                  (UINT)failure.decode.stop_pattern);
    }
    if (failure.decode.failed_index < BC_RUN_COUNT)
    {
        tm_printf((UB *)"failed_index=%u\n", (UINT)failure.decode.failed_index);
    }
    tm_printf((UB *)"[BC RUNS] seq=%u\n", (UINT)failure.sequence);
    for (uint32_t index = 0U;
         (index < failure.run_count) && (index < BC_RUN_COUNT); index++)
    {
        tm_printf((UB *)"%u: %u %c\n", (UINT)index,
                  (UINT)failure.runs[index], (0U == (index % 2U)) ? 'B' : 'W');
    }
    tm_printf((UB *)"[BC END] seq=%u\n", (UINT)failure.sequence);
    g_printed_failure = failure.sequence;
    (void)line_barcode_ack_failure(failure.sequence);
}

static char const * failure_reason (barcode_failure_t const * p_failure)
{
    if ((BC_CHECK_NOT_RUN != p_failure->decode.check) &&
        (BC_CHECK_OK != p_failure->decode.check))
    {
        return bc_check_name(p_failure->decode.check);
    }

    switch (p_failure->status)
    {
        case BC_ERR_QUIET:    return "QUIET";
        case BC_ERR_TIMEOUT:  return "TIMEOUT";
        case BC_ERR_OVERFLOW: return "CAPTURE_OVERFLOW";
        case BC_ERR_REVERSE:  return "REVERSE";
        case BC_ERR_SKEW:     return "SKEW";
        case BC_ERR_SAMPLE:   return "SAMPLE";
        case BC_ERR_STOPPED:  return "STOPPED";
        default:             return "UNKNOWN";
    }
}

static char const * decimal_u64 (uint64_t value, char digits[21])
{
    uint32_t index = 20U;

    digits[index] = '\0';
    do
    {
        index--;
        digits[index] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value > 0U);

    return &digits[index];
}
#endif

static uint32_t absolute_delta (int32_t current, int32_t previous)
{
    uint32_t const delta = (uint32_t)current - (uint32_t)previous;

    return (delta <= INT32_MAX) ? delta : (0U - delta);
}

static void measure_sample (uint32_t raw, int32_t left, int32_t right,
                            uint32_t now_usec)
{
    if (gb_measured)
    {
        uint32_t const dl = absolute_delta(left, g_diagnostic.left_count);
        uint32_t const dr = absolute_delta(right, g_diagnostic.right_count);
        uint64_t const sum = (uint64_t)dl + dr;
        uint32_t const step = (sum > UINT32_MAX) ? UINT32_MAX : (uint32_t)sum;
        uint32_t const skew = (dl > dr) ? dl - dr : dr - dl;
        uint32_t const gap = now_usec - g_diagnostic.timestamp_usec;
        uint64_t const run = (uint64_t)g_measure_run + step;

        g_measure_run = (run > UINT32_MAX) ? UINT32_MAX : (uint32_t)run;
        if (gap > g_diagnostic.max_gap_usec)
        {
            g_diagnostic.max_gap_usec = gap;
        }
        if (step > g_diagnostic.max_step_units)
        {
            g_diagnostic.max_step_units = step;
        }
        if (skew > g_diagnostic.max_skew_units)
        {
            g_diagnostic.max_skew_units = skew;
        }
        if (raw != g_diagnostic.raw)
        {
            g_diagnostic.last_run_level = g_diagnostic.raw;
            g_diagnostic.last_run_units = g_measure_run;
            g_measure_run = 0U;
        }
    }

    gb_measured = true;
    g_diagnostic.raw = raw;
    g_diagnostic.left_count = left;
    g_diagnostic.right_count = right;
    g_diagnostic.timestamp_usec = now_usec;
}

/*** end of file ***/

