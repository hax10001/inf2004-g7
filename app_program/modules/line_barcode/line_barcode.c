/** @file line_barcode.c
 * @brief Filter, line tracker and barcode detection, with bounded events.
 */
#include "line_barcode.h"
#include "bc_capture.h"
#include "ir_filter.h"
#include "ir_shared.h"
#include "line_track.h"
#include "task_ir.h"
#include "app/app_config.h"
#include "app/tasks.h"
//#include <stddef.h>

static ID gh_lbar_mutex = 0;
static ir_filter_t g_filter = {0};
static line_track_t g_track = {0};
static bc_capture_t g_capture = {0};
static lbar_status_t g_status = {0};
static uint8_t g_b_configured = 0U;
static uint8_t g_b_started = 0U;
static uint8_t g_b_active = 0U;
static uint32_t g_pending_ticks = 0U;

ER line_barcode_init (void)
{
    T_CMTX mutex_config = {0};
    ER error = ir_shared_init();

    if (error < E_OK)
    {
        return error;
    }
    mutex_config.mtxatr = TA_INHERIT;
    gh_lbar_mutex = tk_cre_mtx(&mutex_config);
    if (gh_lbar_mutex <= E_OK)
    {
        return (gh_lbar_mutex == E_OK) ? E_OBJ : gh_lbar_mutex;
    }
    (void)ir_filter_init(&g_filter, IR_FILTER_BITS);
    line_track_init(&g_track);
    g_status = (lbar_status_t){0};
    g_status.pending_letter = BC_INVALID_LETTER;
    g_b_configured = 0U;
    g_b_started = 0U;
    g_b_active = 0U;
    return E_OK;
}

ER line_barcode_configure (bc_capture_config_t const * p_config,
                           uint8_t polarity_bits)
{
    if ((NULL == p_config) || (0U != g_b_started))
    {
        return E_PAR;
    }
    if (!ir_filter_init(&g_filter, polarity_bits) ||
        !bc_capture_init(&g_capture, p_config))
    {
        return E_PAR;
    }
    g_b_configured = 1U;
    return E_OK;
}

ER line_barcode_start (void)
{
    ID h_task = 0;

    g_b_started = 1U;
    h_task = tasks_create(task_ir_task, APP_IR_PRIORITY);
    if (h_task <= E_OK)
    {
        g_b_started = 0U;
        return (h_task == E_OK) ? E_OBJ : h_task;
    }
    return tk_sta_tsk(h_task, 0);
}

ER line_barcode_set_active (bool b_active)
{
    ER error = E_OK;

    if (gh_lbar_mutex <= E_OK)
    {
        return E_OBJ;
    }
    if (b_active && (0U == g_b_configured))
    {
        return E_OBJ;
    }
    error = tk_loc_mtx(gh_lbar_mutex, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }
    g_b_active = b_active ? 1U : 0U;
    if (b_active)
    {
        line_track_init(&g_track);
        bc_capture_cancel(&g_capture);
        g_status.event_flags = 0U;
        g_status.pending_letter = BC_INVALID_LETTER;
    }
    if (!b_active)
    {
        bc_capture_cancel(&g_capture);
        g_status.pending_letter = BC_INVALID_LETTER;
        g_status.event_flags = 0U;
    }
    return tk_unl_mtx(gh_lbar_mutex);
}

ER line_barcode_set_recovery (bool b_recovering)
{
    SYSTIM now = {0};
    ER error = E_OK;

    if (gh_lbar_mutex <= E_OK)
    {
        return E_OBJ;
    }
    error = tk_get_otm(&now);
    if (error < E_OK)
    {
        return error;
    }
    error = tk_loc_mtx(gh_lbar_mutex, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }
    line_track_recovery(&g_track, b_recovering, now.lo);
    bc_capture_cancel(&g_capture);
    if (b_recovering &&
        ((g_status.event_flags & LBAR_EVENT_PENDING) != 0U))
    {
        g_status.pending_letter = BC_INVALID_LETTER;
        g_status.event_flags &= (uint8_t)~LBAR_EVENT_PENDING;
        g_status.event_flags |= LBAR_EVENT_FAULT;
    }
    return tk_unl_mtx(gh_lbar_mutex);
}

ER line_barcode_process (uint8_t raw_bits)
{
    ir_shared_compact_t snapshot = {0};
    bc_decode_result_t decode = {0};
    bc_capture_event_t event = BC_PENDING;
    bool b_junction = false;
    bool b_was_recovering = false;
    uint8_t black_bits = 0U;
    ER error = E_OK;

    if (gh_lbar_mutex <= E_OK)
    {
        return E_OBJ;
    }
    error = ir_shared_get_compact(&snapshot);
    if (error < E_OK)
    {
        return error;
    }
    /* No wait while another consumer reads a status word. */
    error = tk_loc_mtx(gh_lbar_mutex, 0);
    if (error < E_OK)
    {
        return error;
    }
    if (ir_filter_step(&g_filter, raw_bits, &black_bits))
    {
        b_was_recovering = (0U != g_track.b_recovering);
        g_status.line_state = (uint8_t)line_track_step_distance(
            &g_track, (uint8_t)(black_bits & 3U),
            snapshot.timestamp_msec, snapshot.travel_ticks,
            g_capture.config.junction_min_ticks,
            (0U != g_b_configured) &&
            ((snapshot.flags & IR_SHARED_ENCODER_BIT) != 0U), &b_junction);
        if (b_was_recovering && (0U == g_track.b_recovering))
        {
            g_status.event_flags |= LBAR_EVENT_REACQUIRED;
        }
        if (LT_FAULT == g_status.line_state)
        {
            g_status.event_flags |= LBAR_EVENT_FAULT;
        }
        g_status.sensor_flags = black_bits;
        g_status.timestamp_msec = snapshot.timestamp_msec;
        if (b_junction && (0U != g_b_active))
        {
            if (g_status.event_flags & LBAR_EVENT_JUNCTION)
            {
                g_status.event_flags |= LBAR_EVENT_OVERFLOW;
            }
            g_status.event_flags |= LBAR_EVENT_JUNCTION;
            ++g_status.sequence;
        }
        /* Pending commands expire after bounded travel without a turn. */
        if ((0U != g_b_active) &&
            ((g_status.event_flags & LBAR_EVENT_PENDING) != 0U) &&
            ((snapshot.flags & IR_SHARED_ENCODER_BIT) != 0U) &&
            ((snapshot.travel_ticks - g_pending_ticks) >
             g_capture.config.pending_expiry_ticks))
        {
            g_status.pending_letter = BC_INVALID_LETTER;
            g_status.event_flags &= (uint8_t)~LBAR_EVENT_PENDING;
            g_status.event_flags |= LBAR_EVENT_FAULT;
        }
        if ((0U != g_b_active) && (0U != g_b_configured) &&
            (0U == g_track.b_recovering) &&
            ((snapshot.flags & IR_SHARED_ENCODER_BIT) != 0U))
        {
            event = bc_capture_step(&g_capture,
                                    (black_bits & 4U) != 0U,
                                    snapshot.travel_ticks,
                                    snapshot.timestamp_msec, true, &decode);
            if ((BC_COMPLETE == event) &&
                !(g_status.event_flags & LBAR_EVENT_PENDING))
            {
                g_status.pending_letter = (uint8_t)(decode.letter &
                                                     BC_LETTER_MASK);
                g_pending_ticks = snapshot.travel_ticks;
                g_status.event_flags |= LBAR_EVENT_BARCODE |
                                        LBAR_EVENT_PENDING;
                if (decode.b_reverse)
                {
                    g_status.event_flags |= LBAR_EVENT_REVERSE;
                }
                ++g_status.sequence;
            }
            else if ((BC_COMPLETE == event) &&
                     (g_status.pending_letter != decode.letter))
            {
                g_status.event_flags |= LBAR_EVENT_OVERFLOW;
            }
            else if (BC_REJECTED == event)
            {
                g_status.event_flags |= LBAR_EVENT_FAULT;
            }
        }
        else if (0U != g_b_configured)
        {
            if (0U != g_capture.phase)
            {
                g_status.event_flags |= LBAR_EVENT_FAULT;
            }
            bc_capture_cancel(&g_capture);
        }
    }
    return tk_unl_mtx(gh_lbar_mutex);
}

ER line_barcode_report_fault (void)
{
    ER error = E_OK;

    if (gh_lbar_mutex <= E_OK)
    {
        return E_OBJ;
    }
    error = tk_loc_mtx(gh_lbar_mutex, 0);
    if (error < E_OK)
    {
        return error;
    }
    g_status.event_flags |= LBAR_EVENT_FAULT;
    bc_capture_cancel(&g_capture);
    ++g_status.sequence;
    return tk_unl_mtx(gh_lbar_mutex);
}

ER line_barcode_get_status (lbar_status_t * p_output)
{
    ER error = E_OK;

    if (NULL == p_output)
    {
        return E_PAR;
    }
    if (gh_lbar_mutex <= E_OK)
    {
        return E_OBJ;
    }
    error = tk_loc_mtx(gh_lbar_mutex, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }
    *p_output = g_status;
    return tk_unl_mtx(gh_lbar_mutex);
}

ER line_barcode_ack (uint8_t event_flags)
{
    ER error = E_OK;

    if (gh_lbar_mutex <= E_OK)
    {
        return E_OBJ;
    }
    if (event_flags & ~(LBAR_EVENT_BARCODE | LBAR_EVENT_JUNCTION |
                        LBAR_EVENT_FAULT | LBAR_EVENT_OVERFLOW |
                        LBAR_EVENT_REACQUIRED))
    {
        return E_PAR;
    }
    error = tk_loc_mtx(gh_lbar_mutex, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }
    g_status.event_flags &= (uint8_t)~event_flags;
    return tk_unl_mtx(gh_lbar_mutex);
}

ER line_barcode_cancel (void)
{
    ER error = E_OK;

    if (gh_lbar_mutex <= E_OK)
    {
        return E_OBJ;
    }
    error = tk_loc_mtx(gh_lbar_mutex, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }
    bc_capture_cancel(&g_capture);
    g_status.pending_letter = BC_INVALID_LETTER;
    g_pending_ticks = 0U;
    g_status.event_flags = 0U;
    ++g_status.sequence;
    return tk_unl_mtx(gh_lbar_mutex);
}

/*** end of file ***/

