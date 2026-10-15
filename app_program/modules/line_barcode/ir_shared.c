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

/*** end of file ***/

