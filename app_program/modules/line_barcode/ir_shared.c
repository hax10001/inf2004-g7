/** @file ir_shared.c
 * @brief Mutex-protected latest IR snapshot.
 * @note Volatile marks shared state; the mutex provides synchronisation.
 */
#include <tk/tkernel.h>
#include "ir_shared.h"
#include "app/app_config.h"

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
/* Set once APP_IR_BOTH_GAP_SAMPLES consecutive samples were not both
 * black: a 50 ms controller can then tell a continuous both-black run
 * (junction) from black readings that were broken by white in between.
 */
static bool volatile     gb_both_gap_latched = false;
static uint32_t volatile g_not_both_samples  = 0U;
/* Junction diagnostics: sample counts since the controller last took them. */
static ir_shared_diag_t volatile g_diag = {0U, 0U, 0U, 0U};

/* The SMP port's context checks read per-core state; a task that moves
 * between cores during the check can get a spurious E_CTX (seen during
 * heavy USB output). The check runs before the mutex is touched, so a
 * prompt retry is safe for both lock and unlock.
 */
#define IR_SHARED_CTX_RETRIES 4U

static ER ir_shared_lock_retry (TMO timeout)
{
    ER error = E_CTX;

    for (UINT attempt = 0U;
         (attempt < IR_SHARED_CTX_RETRIES) && (E_CTX == error); ++attempt)
    {
        error = tk_loc_mtx(gh_ir_mutex, timeout);
    }

    return error;
}

static ER ir_shared_unlock_retry (void)
{
    ER error = E_CTX;

    for (UINT attempt = 0U;
         (attempt < IR_SHARED_CTX_RETRIES) && (E_CTX == error); ++attempt)
    {
        error = tk_unl_mtx(gh_ir_mutex);
    }

    return error;
}

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

    error = ir_shared_lock_retry(TMO_FEVR);

    /* E_ILUSE: this task already holds the mutex because an earlier unlock
     * failed (rare E_CTX seen under heavy console output). Continue and
     * unlock below rather than failing every cycle with the lock held,
     * which left the controller's IR data stale for good. */
    if (E_ILUSE == error)
    {
        error = E_OK;
    }

    if (error < E_OK)
    {
        return error;
    }

    g_latest_ir.left           = left;
    g_latest_ir.right          = right;
    g_latest_ir.barcode        = barcode;
    g_latest_ir.timestamp_msec = now.lo;
    g_latest_ir.b_valid        = true;

    if (g_diag.samples < UINT16_MAX)
    {
        g_diag.samples++;
        g_diag.left  += (uint16_t)(1U == left);
        g_diag.right += (uint16_t)(1U == right);
        g_diag.both  += (uint16_t)((1U == left) && (1U == right));
    }

    if ((1U == left) && (1U == right))
    {
        gb_junction_latched = true;
        g_not_both_samples  = 0U;
    }
    else if (g_not_both_samples < APP_IR_BOTH_GAP_SAMPLES)
    {
        g_not_both_samples++;

        if (APP_IR_BOTH_GAP_SAMPLES == g_not_both_samples)
        {
            gb_both_gap_latched = true;
        }
    }
    else
    {
        /* Gap already recorded; the count stays saturated. */
    }

    if (1U == left)
    {
        gb_left_latched = true;
    }

    if (1U == right)
    {
        gb_right_latched = true;
    }

    return ir_shared_unlock_retry();
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

    ER error = ir_shared_lock_retry(TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    *p_b_seen           = gb_junction_latched;
    gb_junction_latched = false;

    return ir_shared_unlock_retry();
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

    ER error = ir_shared_lock_retry(TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    *p_b_left        = gb_left_latched;
    *p_b_right       = gb_right_latched;
    gb_left_latched  = false;
    gb_right_latched = false;

    return ir_shared_unlock_retry();
}

/* The controller is the only consumer; taking the latch clears it. */
ER ir_shared_take_both_gap (bool * p_b_gap)
{
    if (NULL == p_b_gap)
    {
        return E_PAR;
    }

    if (gh_ir_mutex <= E_OK)
    {
        return E_OBJ;
    }

    ER error = ir_shared_lock_retry(TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    *p_b_gap            = gb_both_gap_latched;
    gb_both_gap_latched = false;

    return ir_shared_unlock_retry();
}

/* The controller is the only consumer; taking the counts clears them. */
ER ir_shared_take_diag (ir_shared_diag_t * p_diag)
{
    if (NULL == p_diag)
    {
        return E_PAR;
    }

    if (gh_ir_mutex <= E_OK)
    {
        return E_OBJ;
    }

    ER error = ir_shared_lock_retry(TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    p_diag->samples = g_diag.samples;
    p_diag->left    = g_diag.left;
    p_diag->right   = g_diag.right;
    p_diag->both    = g_diag.both;
    g_diag.samples  = 0U;
    g_diag.left     = 0U;
    g_diag.right    = 0U;
    g_diag.both     = 0U;

    return ir_shared_unlock_retry();
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

    error = ir_shared_lock_retry(TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    *p_output = g_latest_ir;

    return ir_shared_unlock_retry();
}

ER ir_shared_lock (TMO timeout)
{
    if (gh_ir_mutex <= E_OK)
    {
        return E_OBJ;
    }

    return ir_shared_lock_retry(timeout);
}

ER ir_shared_unlock (void)
{
    return ir_shared_unlock_retry();
}

/*** end of file ***/

