/** @file bc_port.c
 * @brief Nonblocking capture feed and mutex-protected decoded-letter slot.
 */
#include "bc_port.h"
#include "drivers/encoder.h"

#define BC_PORT_ERROR_PATTERN 7U
#define BC_PORT_ERROR_BUSY 8U
#define BC_PORT_DIAG_SLOTS 4U

static ID gh_port_mutex = 0;
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
        return 0U;
    }

    difference = (step_l > step_r) ?
                 (uint64_t)(step_l - step_r) :
                 (uint64_t)(step_r - step_l);
    sum = (uint64_t)step_l + (uint64_t)step_r;

    if ((difference > g_port_config.max_wheel_step_delta) ||
        (sum > UINT32_MAX - g_travel_units))
    {
        return 0U;
    }

    g_travel_units += (uint32_t)sum;
    /* If any sample spans too much travel, a narrow element might
     * have been missed: reject rather than invent missing transitions.
     */
    return (sum <= g_port_config.max_sample_travel_units) ? 1U : 0U;
}

ER bc_port_init (bc_port_config_t const * p_config)
{
    T_CMTX mutex_config = {0};
    ID mutex;

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

    if (gh_port_mutex > E_OK)
    {
        return E_OBJ;
    }

    if (0U == bc_scan_init(&g_port_scan, &p_config->scan))
    {
        return E_PAR;
    }

    mutex_config.mtxatr = TA_INHERIT;
    mutex = tk_cre_mtx(&mutex_config);

    if (mutex <= E_OK)
    {
        return mutex;
    }

    g_port_config = *p_config;
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
    gh_port_mutex = mutex;
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

    if (gh_port_mutex <= E_OK)
    {
        return E_OBJ;
    }
    if (raw_gpio_level > 1U)
    {
        gb_sample_lost = 1U;
        return E_PAR;
    }
    error = tk_loc_mtx(gh_port_mutex, 0);
    if (error < E_OK)
    {
        gb_sample_lost = 1U;
        return error;
    }

    if ((0U != gb_have_counts) &&
        (((left != g_last_left) &&
          ((uint32_t)(now_usec - left_edge_usec) > edge_age_limit)) ||
         ((right != g_last_right) &&
          ((uint32_t)(now_usec - right_edge_usec) > edge_age_limit))))
    {
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
        b_valid = 0U;
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
                       g_travel_units, now_msec, b_valid);
    return tk_unl_mtx(gh_port_mutex);
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
    bc_port_result_t result = {0U, 0U, 0U};
    ER error;

    if (gh_port_mutex <= E_OK)
    {
        return E_OBJ;
    }

    error = tk_loc_mtx(gh_port_mutex, 0);
    if (error < E_OK)
    {
        return error;
    }

    event = bc_scan_take(&g_port_scan, widths, &error_code,
                         &timestamp);
    error = tk_unl_mtx(gh_port_mutex);

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

    error = tk_loc_mtx(gh_port_mutex, TMO_FEVR);
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
    error = tk_unl_mtx(gh_port_mutex);
    return (0U != b_busy) ? E_OBJ : error;
}

ER bc_port_peek (bc_port_result_t * p_output)
{
    ER error;

    if (0 == p_output)
    {
        return E_PAR;
    }
    if (gh_port_mutex <= E_OK)
    {
        return E_OBJ;
    }

    error = tk_loc_mtx(gh_port_mutex, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }

    if (0U == gb_have_result)
    {
        (void)tk_unl_mtx(gh_port_mutex);
        return E_OBJ;
    }

    *p_output = g_port_result;
    return tk_unl_mtx(gh_port_mutex);
}

ER bc_port_diag_pop (bc_port_result_t * p_output)
{
    ER error;

    if (0 == p_output)
    {
        return E_PAR;
    }
    if (gh_port_mutex <= E_OK)
    {
        return E_OBJ;
    }
    error = tk_loc_mtx(gh_port_mutex, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }
    if (0U == g_diag_count)
    {
        (void)tk_unl_mtx(gh_port_mutex);
        return E_OBJ;
    }
    *p_output = g_diag_queue[g_diag_head];
    g_diag_head = (uint8_t)((g_diag_head + 1U) % BC_PORT_DIAG_SLOTS);
    --g_diag_count;
    return tk_unl_mtx(gh_port_mutex);
}

ER bc_port_ack (uint16_t sequence)
{
    ER error;

    if (gh_port_mutex <= E_OK)
    {
        return E_OBJ;
    }
    error = tk_loc_mtx(gh_port_mutex, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }

    if ((0U == gb_have_result) ||
        (sequence != g_port_result.sequence))
    {
        (void)tk_unl_mtx(gh_port_mutex);
        return E_OBJ;
    }

    gb_have_result = 0U;
    return tk_unl_mtx(gh_port_mutex);
}

ER bc_port_clear (void)
{
    ER error;

    if (gh_port_mutex <= E_OK)
    {
        return E_OBJ;
    }
    error = tk_loc_mtx(gh_port_mutex, TMO_FEVR);
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
    return tk_unl_mtx(gh_port_mutex);
}

/*** end of file ***/

