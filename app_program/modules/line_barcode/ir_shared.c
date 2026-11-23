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

/* Four latched observations and the saturating 0..2 gap count are
 * stored in six bits. Every read/modify/write occurs under gh_ir_mutex;
 * no ISR accesses this state. Unlike independent bool variables, this
 * does not allocate one byte (or word) per application flag. */
#define IR_LATCH_JUNCTION 0x01U
#define IR_LATCH_LEFT 0x02U
#define IR_LATCH_RIGHT 0x04U
#define IR_LATCH_BOTH_GAP 0x08U
#define IR_GAP_COUNT_SHIFT 4U
#define IR_GAP_COUNT_MASK 0x30U
static volatile uint8_t g_ir_latched_and_gap = 0U;
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
        g_ir_latched_and_gap |= IR_LATCH_JUNCTION;
        g_ir_latched_and_gap &= (uint8_t)~IR_GAP_COUNT_MASK;
    }
    else if (((g_ir_latched_and_gap & IR_GAP_COUNT_MASK) >>
              IR_GAP_COUNT_SHIFT) < APP_IR_BOTH_GAP_SAMPLES)
    {
        uint8_t count = (uint8_t)((g_ir_latched_and_gap &
            IR_GAP_COUNT_MASK) >> IR_GAP_COUNT_SHIFT);
        ++count;
        g_ir_latched_and_gap = (uint8_t)((g_ir_latched_and_gap &
            (uint8_t)~IR_GAP_COUNT_MASK) |
            (uint8_t)(count << IR_GAP_COUNT_SHIFT));
        if (APP_IR_BOTH_GAP_SAMPLES == count)
        {
            g_ir_latched_and_gap |= IR_LATCH_BOTH_GAP;
        }
    }
    else
    {
        /* Gap already recorded; the count stays saturated. */
    }

    if (1U == left)
    {
        g_ir_latched_and_gap |= IR_LATCH_LEFT;
    }

    if (1U == right)
    {
        g_ir_latched_and_gap |= IR_LATCH_RIGHT;
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

    *p_b_seen = (g_ir_latched_and_gap & IR_LATCH_JUNCTION) != 0U;
    g_ir_latched_and_gap &= (uint8_t)~IR_LATCH_JUNCTION;

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

    *p_b_left = (g_ir_latched_and_gap & IR_LATCH_LEFT) != 0U;
    *p_b_right = (g_ir_latched_and_gap & IR_LATCH_RIGHT) != 0U;
    g_ir_latched_and_gap &= (uint8_t)~(IR_LATCH_LEFT | IR_LATCH_RIGHT);

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

    *p_b_gap = (g_ir_latched_and_gap & IR_LATCH_BOTH_GAP) != 0U;
    g_ir_latched_and_gap &= (uint8_t)~IR_LATCH_BOTH_GAP;

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

