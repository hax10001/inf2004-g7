/** @file bc_port.c
 * @brief Nonblocking capture feed and semaphore-protected decoded-letter slot.
 */
#include "bc_port.h"
#include "bc_cal.h"
#include "drivers/encoder.h"

#define BC_PORT_ERROR_PATTERN 7U
#define BC_PORT_ERROR_BUSY 8U
#define BC_PORT_DIAG_SLOTS 4U

static ID gh_port_lock = 0;
static bc_port_config_t g_port_config;
static bc_scan_t g_port_scan;
static bc_port_result_t g_port_result;
static bc_port_result_t g_diag_queue[BC_PORT_DIAG_SLOTS];
static uint8_t g_diag_head = 0U;
static uint8_t g_diag_count = 0U;
static int32_t g_last_left = 0;
static int32_t g_last_right = 0;
static uint32_t g_travel_units = 0U;
static uint16_t g_event_sequence = 0U;
static uint8_t gb_have_counts = 0U;
static uint8_t gb_have_time = 0U;
static uint32_t g_last_time_usec = 0U;
static uint64_t g_clock_usec = 0U;
static uint8_t gb_have_clock = 0U;
static uint8_t gb_have_result = 0U;
static uint8_t volatile gb_sample_lost = 0U;
static bc_port_stats_t g_input_stats;
static uint8_t gb_scan_active;

static ER bc_port_store (bc_code_result_t decoded, uint32_t timestamp,
                         uint8_t error_code, uint8_t event);


static uint8_t bc_port_ticks (int32_t left, int32_t right)
{
    int64_t step_l;
    int64_t step_r;
    uint64_t difference;
    uint64_t sum;

    if (0U == gb_have_counts)
    {
        g_last_left = left;
        g_last_right = right;
        gb_have_counts = 1U;
        return 1U;
    }

    step_l = ((int64_t)left - g_last_left) *
             g_port_config.left_forward_sign;
    step_r = ((int64_t)right - g_last_right) *
             g_port_config.right_forward_sign;
    g_last_left = left;
    g_last_right = right;

    if ((step_l < 0) || (step_r < 0))
    {
        ++g_input_stats.direction_invalid;
#if BC_CAL_TEST_IGNORE_INPUT_ERRORS
        /* Test bypass retains forward travel without inventing travel
         * from backward steps. Direction failures remain in diagnostics.
         */
        step_l = (step_l < 0) ? 0 : step_l;
        step_r = (step_r < 0) ? 0 : step_r;
#else
        return 0U;
#endif
    }

    difference = (step_l > step_r) ?
                 (uint64_t)(step_l - step_r) :
                 (uint64_t)(step_r - step_l);
    sum = (uint64_t)step_l + (uint64_t)step_r;

    if (difference > g_port_config.max_wheel_step_delta)
    {
        ++g_input_stats.skew_invalid;
#if !BC_CAL_TEST_IGNORE_INPUT_ERRORS
        return 0U;
#endif
    }
    /* Keep arithmetic bounded even during permissive testing. */
    if (sum > UINT32_MAX - g_travel_units)
    {
        return 0U;
    }

    g_travel_units += (uint32_t)sum;
    if (sum > g_port_config.max_sample_travel_units)
    {
        ++g_input_stats.step_invalid;
    }
    /* If any sample spans too much travel, a narrow element might
     * have been missed: reject rather than invent missing transitions.
     */
    return (sum <= g_port_config.max_sample_travel_units) ? 1U : 0U;
}

ER bc_port_init (bc_port_config_t const * p_config)
{
    T_CSEM lock_config = {0};
    ID lock;

    if ((0 == p_config) ||
        ((1 != p_config->left_forward_sign) &&
         (-1 != p_config->left_forward_sign)) ||
        ((1 != p_config->right_forward_sign) &&
         (-1 != p_config->right_forward_sign)) ||
        (p_config->black_gpio_level > 1U) ||
        (0U == p_config->max_sample_travel_units) ||
        (0U == p_config->max_sample_interval_msec))
    {
        return E_PAR;
    }

    if (gh_port_lock > E_OK)
    {
        return E_OBJ;
    }

    if (0U == bc_scan_init(&g_port_scan, &p_config->scan))
    {
        return E_PAR;
    }

    /* Motion, IR and terrain use all four configured mutex slots.
     * A binary semaphore preserves task-context exclusion without taking
     * a fifth slot. No I/O, delays or decoding occur while holding it.
     * Priority ordering does not provide mutex priority inheritance.
     */
    lock_config.sematr = TA_TPRI;
    lock_config.isemcnt = 1;
    lock_config.maxsem = 1;
    lock = tk_cre_sem(&lock_config);

    if (lock <= E_OK)
    {
        return (E_OK == lock) ? E_OBJ : lock;
    }

    g_port_config = *p_config;
    g_input_stats = (bc_port_stats_t){0};
    gb_scan_active = 0U;
    g_port_result = (bc_port_result_t){0U, 0U, 0U};
    gb_have_counts = 0U;
    gb_have_time = 0U;
    gb_have_clock = 0U;
    g_clock_usec = 0U;
    gb_have_result = 0U;
    g_diag_head = 0U;
    g_diag_count = 0U;
    gb_sample_lost = 0U;
    g_travel_units = 0U;
    gh_port_lock = lock;
    return E_OK;
}

ER bc_port_sample_counts (uint8_t raw_gpio_level,
                          uint32_t now_usec, int32_t left,
                          int32_t right, uint32_t left_edge_usec,
                          uint32_t right_edge_usec, uint8_t b_valid)
{
    uint32_t now_msec;
    uint32_t const edge_age_limit = (uint32_t)
        g_port_config.max_sample_interval_msec * 1000U;
    ER error;

    if (gh_port_lock <= E_OK)
    {
        return E_OBJ;
    }
    if (raw_gpio_level > 1U)
    {
        gb_sample_lost = 1U;
        return E_PAR;
    }
    error = tk_wai_sem(gh_port_lock, 1, 0);
    if (error < E_OK)
    {
        gb_sample_lost = 1U;
        ++g_input_stats.lock_lost;
        return error;
    }

    if ((0U != g_input_stats.samples) &&
        (g_input_stats.raw != raw_gpio_level))
    {
        ++g_input_stats.edges;
    }
    ++g_input_stats.samples;
    g_input_stats.raw = raw_gpio_level;
    if (0U == b_valid)
    {
        ++g_input_stats.capture_invalid;
    }

    if ((0U != gb_have_counts) &&
        (((left != g_last_left) &&
          ((uint32_t)(now_usec - left_edge_usec) > edge_age_limit)) ||
         ((right != g_last_right) &&
          ((uint32_t)(now_usec - right_edge_usec) > edge_age_limit))))
    {
        ++g_input_stats.stale_edge;
        b_valid = 0U;
    }
    if (0U == bc_port_ticks(left, right))
    {
        b_valid = 0U;
    }
    /* Subtract microseconds before converting: the hardware low word
     * wraps every 72 minutes. Keep event/timeout milliseconds continuous.
     */
    uint32_t const elapsed_usec = now_usec - g_last_time_usec;
    if ((0U != gb_have_time) && (elapsed_usec > edge_age_limit))
    {
        ++g_input_stats.gap_invalid;
        b_valid = 0U;
    }
    if ((0U != gb_have_time) &&
        (elapsed_usec > g_input_stats.max_gap_usec))
    {
        g_input_stats.max_gap_usec = elapsed_usec;
    }
    if (0U == gb_have_clock)
    {
        g_clock_usec = now_usec;
        gb_have_clock = 1U;
    }
    else
    {
        g_clock_usec += elapsed_usec;
    }
    now_msec = (uint32_t)(g_clock_usec / 1000U);
    g_last_time_usec = now_usec;
    gb_have_time = 1U;

    if (0U != gb_sample_lost)
    {
        b_valid = 0U;
        gb_sample_lost = 0U;
    }
    (void)bc_scan_feed(&g_port_scan,
                       (raw_gpio_level ==
                        g_port_config.black_gpio_level) ? 1U : 0U,
                       g_travel_units, now_msec,
                       BC_CAL_TEST_IGNORE_INPUT_ERRORS ? 1U : b_valid);
    if ((BC_SCAN_CAPTURING == g_port_scan.state) ||
        (BC_SCAN_END_CANDIDATE == g_port_scan.state) ||
        (BC_SCAN_COMPLETE == g_port_scan.state))
    {
        gb_scan_active = 1U;
    }
    g_input_stats.state = g_port_scan.state;
    g_input_stats.runs = g_port_scan.run_count;
    g_input_stats.scan_units = (0U != gb_scan_active) ?
        g_travel_units - g_port_scan.start_units : 0U;
    return tk_sig_sem(gh_port_lock, 1);
}

ER bc_port_sample (uint8_t raw_gpio_level, uint32_t now_msec)
{
    encoder_sample_t left = {0};
    encoder_sample_t right = {0};

    (void)now_msec;
    encoder_read(ENCODER_LEFT, &left);
    encoder_read(ENCODER_RIGHT, &right);
    return bc_port_sample_counts(raw_gpio_level, encoder_now_usec(),
                                 left.count, right.count,
                                 left.edge_usec, right.edge_usec, 1U);
}

ER bc_port_process (void)
{
    uint32_t widths[BC_CODE_ELEMENTS] = {0U};
    uint32_t timestamp = 0U;
    uint8_t error_code = BC_SCAN_ERROR_NONE;
    uint8_t event;
    bc_code_result_t decoded;
    ER error;

    if (gh_port_lock <= E_OK)
    {
        return E_OBJ;
    }

    error = tk_wai_sem(gh_port_lock, 1, 0);
    if (error < E_OK)
    {
        return error;
    }

    if ((BC_SCAN_COMPLETE == g_port_scan.state) ||
        (BC_SCAN_INVALID == g_port_scan.state))
    {
        ++g_input_stats.last_event;
        g_input_stats.last_error = g_port_scan.error;
        g_input_stats.last_runs = g_port_scan.run_count;
        g_input_stats.last_scan_units = g_input_stats.scan_units;
        for (uint8_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
        {
            g_input_stats.last_widths[i] = (i < g_port_scan.run_count) ?
                g_port_scan.widths[i] : 0U;
        }
        gb_scan_active = 0U;
    }
    event = bc_scan_take(&g_port_scan, widths, &error_code,
                         &timestamp);
    error = tk_sig_sem(gh_port_lock, 1);

    if (error < E_OK)
    {
        return error;
    }
    if (BC_SCAN_EVENT_NONE == event)
    {
        return E_OK;
    }

    decoded = (bc_code_result_t){BC_CODE_LETTER_INVALID, 0U, 0U, 0U};
    if (BC_SCAN_EVENT_READY == event)
    {
        decoded = bc_code_decode(widths,
                                  g_port_config.scan.min_run_units);
        if (0U == decoded.valid)
        {
            error_code = BC_PORT_ERROR_PATTERN;
        }
    }

    return bc_port_store(decoded, timestamp, error_code, event);
}

static ER bc_port_store (bc_code_result_t decoded, uint32_t timestamp,
                         uint8_t error_code, uint8_t event)
{
    bc_port_result_t result = {0U, 0U, 0U};
    ER error;
    result.timestamp_msec = timestamp;
    result.info = (uint16_t)((0U != decoded.valid) ?
                             decoded.letter : BC_CODE_LETTER_INVALID);
    result.info |= (uint16_t)((decoded.quality & 3U) <<
                              BC_PORT_QUALITY_SHIFT);
    result.info |= (uint16_t)((error_code & 15U) <<
                              BC_PORT_ERROR_SHIFT);
    if (BC_SCAN_EVENT_READY == event)
    {
        result.info |= BC_PORT_COMPLETE_FLAG;
    }
    if (0U != decoded.valid)
    {
        result.info |= BC_PORT_VALID_FLAG;
    }
    if (0U != decoded.reverse)
    {
        result.info |= BC_PORT_REVERSE_FLAG;
    }

    error = tk_wai_sem(gh_port_lock, 1, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }

    uint8_t b_busy = gb_have_result;
    ++g_event_sequence;
    result.sequence = g_event_sequence;
    if ((0U == b_busy) ||
        (0U == (g_port_result.info & BC_PORT_VALID_FLAG)))
    {
        /* A rejection must not indefinitely block a later valid letter.
         * Never replace a still-pending valid letter until acknowledged.
         */
        g_port_result = result;
        gb_have_result = 1U;
        b_busy = 0U;
    }
    else
    {
        /* Preserve pending valid letter; report the busy event only. */
        result.info = BC_CODE_LETTER_INVALID |
            (uint16_t)(BC_PORT_ERROR_BUSY << BC_PORT_ERROR_SHIFT);
    }

    if (g_diag_count < BC_PORT_DIAG_SLOTS)
    {
        uint8_t tail = (uint8_t)
            ((g_diag_head + g_diag_count) % BC_PORT_DIAG_SLOTS);
        g_diag_queue[tail] = result;
        ++g_diag_count;
    }
    error = tk_sig_sem(gh_port_lock, 1);
    return (0U != b_busy) ? E_OBJ : error;
}

ER bc_port_publish_buffered (bc_code_result_t const * p_result,
                              uint32_t timestamp_msec)
{
    if ((0 == p_result) || (0U == p_result->valid) ||
        (p_result->letter >= 26U))
    {
        return E_PAR;
    }
    if (gh_port_lock <= E_OK)
    {
        return E_OBJ;
    }
    return bc_port_store(*p_result, timestamp_msec,
                         BC_SCAN_ERROR_NONE, BC_SCAN_EVENT_READY);
}

ER bc_port_peek (bc_port_result_t * p_output)
{
    ER error;

    if (0 == p_output)
    {
        return E_PAR;
    }
    if (gh_port_lock <= E_OK)
    {
        return E_OBJ;
    }

    error = tk_wai_sem(gh_port_lock, 1, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }

    if (0U == gb_have_result)
    {
        (void)tk_sig_sem(gh_port_lock, 1);
        return E_OBJ;
    }

    *p_output = g_port_result;
    return tk_sig_sem(gh_port_lock, 1);
}

ER bc_port_diag_pop (bc_port_result_t * p_output)
{
    ER error;

    if (0 == p_output)
    {
        return E_PAR;
    }
    if (gh_port_lock <= E_OK)
    {
        return E_OBJ;
    }
    error = tk_wai_sem(gh_port_lock, 1, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }
    if (0U == g_diag_count)
    {
        (void)tk_sig_sem(gh_port_lock, 1);
        return E_OBJ;
    }
    *p_output = g_diag_queue[g_diag_head];
    g_diag_head = (uint8_t)((g_diag_head + 1U) % BC_PORT_DIAG_SLOTS);
    --g_diag_count;
    return tk_sig_sem(gh_port_lock, 1);
}

ER bc_port_ack (uint16_t sequence)
{
    ER error;

    if (gh_port_lock <= E_OK)
    {
        return E_OBJ;
    }
    error = tk_wai_sem(gh_port_lock, 1, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }

    if ((0U == gb_have_result) ||
        (sequence != g_port_result.sequence))
    {
        (void)tk_sig_sem(gh_port_lock, 1);
        return E_OBJ;
    }

    gb_have_result = 0U;
    return tk_sig_sem(gh_port_lock, 1);
}

ER bc_port_clear (void)
{
    ER error;

    if (gh_port_lock <= E_OK)
    {
        return E_OBJ;
    }
    error = tk_wai_sem(gh_port_lock, 1, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }

    (void)bc_scan_init(&g_port_scan, &g_port_config.scan);
    gb_have_result = 0U;
    g_diag_head = 0U;
    g_diag_count = 0U;
    gb_have_counts = 0U;
    gb_have_time = 0U;
    gb_sample_lost = 0U;
    gb_scan_active = 0U;
    g_input_stats.state = BC_SCAN_IDLE;
    g_input_stats.runs = 0U;
    g_input_stats.scan_units = 0U;
    return tk_sig_sem(gh_port_lock, 1);
}

ER bc_port_snapshot (bc_scan_t * p_output)
{
    if (0 == p_output)
    {
        return E_PAR;
    }
    if (gh_port_lock <= E_OK)
    {
        return E_OBJ;
    }
    ER error = tk_wai_sem(gh_port_lock, 1, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }
    *p_output = g_port_scan;
    return tk_sig_sem(gh_port_lock, 1);
}

ER bc_port_stats (bc_port_stats_t * p_output)
{
    if (0 == p_output)
    {
        return E_PAR;
    }
    if (gh_port_lock <= E_OK)
    {
        return E_OBJ;
    }
    ER error = tk_wai_sem(gh_port_lock, 1, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }
    *p_output = g_input_stats;
    return tk_sig_sem(gh_port_lock, 1);
}

/*** end of file ***/

