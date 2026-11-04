/** @file ir_shared.c
 * @brief Mutex-protected latest IR snapshot.
 * @note Volatile marks shared state; the mutex provides synchronisation.
 */
#include <tk/tkernel.h>
#include "ir_shared.h"

/* Only this module can directly access these variables. */
static ID volatile gh_ir_mutex = 0;

static ir_shared_snapshot_t volatile g_latest_ir = {.left           = 0U,
                                                    .right          = 0U,
                                                    .barcode        = 0U,
                                                    .timestamp_msec = 0U,
                                                    .b_valid        = false};

/* Set by any both-black sample; held until the controller takes it, so a
 * junction shorter than the controller period is not lost between reads.
 */
static bool volatile gb_junction_latched = false;
/* Set by any black sample on that side, for the same reason: a brief touch
 * of the line edge would otherwise be overwritten before it is acted on.
 */
static bool volatile gb_left_latched  = false;
static bool volatile gb_right_latched = false;

/* Initialize once from usermain(), before starting tasks. */
ER ir_shared_init (void)
{
    T_CMTX mutex_config = {0};

    mutex_config.mtxatr = TA_INHERIT;

    ID h_mutex = tk_cre_mtx(&mutex_config);

    if (h_mutex <= E_OK)
    {
        return h_mutex;
    }

    gh_ir_mutex = h_mutex;

    return E_OK;
}

/* The IR sampling task is the only publisher. */
ER ir_shared_publish (uint32_t left, uint32_t right, uint32_t barcode)
{
    SYSTIM now = {0};
    ER     error;

    if (gh_ir_mutex <= E_OK)
    {
        return E_OBJ;
    }

    error = tk_get_otm(&now);

    if (error < E_OK)
    {
        return error;
    }

    error = tk_loc_mtx(gh_ir_mutex, TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    g_latest_ir.left           = left;
    g_latest_ir.right          = right;
    g_latest_ir.barcode        = barcode;
    g_latest_ir.timestamp_msec = now.lo;
    g_latest_ir.b_valid        = true;

    if ((1U == left) && (1U == right))
    {
        gb_junction_latched = true;
    }

    if (1U == left)
    {
        gb_left_latched = true;
    }

    if (1U == right)
    {
        gb_right_latched = true;
    }

    return tk_unl_mtx(gh_ir_mutex);
}

/* The controller is the only consumer; taking the latch clears it. */
ER ir_shared_take_junction (bool * p_b_seen)
{
    if (NULL == p_b_seen)
    {
        return E_PAR;
    }

    if (gh_ir_mutex <= E_OK)
    {
        return E_OBJ;
    }

    ER error = tk_loc_mtx(gh_ir_mutex, TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    *p_b_seen           = gb_junction_latched;
    gb_junction_latched = false;

    return tk_unl_mtx(gh_ir_mutex);
}

/* The controller is the only consumer; taking the latches clears them. */
ER ir_shared_take_sides (bool * p_b_left, bool * p_b_right)
{
    if ((NULL == p_b_left) || (NULL == p_b_right))
    {
        return E_PAR;
    }

    if (gh_ir_mutex <= E_OK)
    {
        return E_OBJ;
    }

    ER error = tk_loc_mtx(gh_ir_mutex, TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    *p_b_left        = gb_left_latched;
    *p_b_right       = gb_right_latched;
    gb_left_latched  = false;
    gb_right_latched = false;

    return tk_unl_mtx(gh_ir_mutex);
}

/* Consumers receive their own copy after acquiring the mutex. */
ER ir_shared_get_latest (ir_shared_snapshot_t * p_output)
{
    ER error;

    if (NULL == p_output)
    {
        return E_PAR;
    }

    if (gh_ir_mutex <= E_OK)
    {
        return E_OBJ;
    }

    error = tk_loc_mtx(gh_ir_mutex, TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    *p_output = g_latest_ir;

    return tk_unl_mtx(gh_ir_mutex);
}

ER ir_shared_lock (TMO timeout)
{
    if (gh_ir_mutex <= E_OK)
    {
        return E_OBJ;
    }

    return tk_loc_mtx(gh_ir_mutex, timeout);
}

ER ir_shared_unlock (void)
{
    return tk_unl_mtx(gh_ir_mutex);
}

/*** end of file ***/

