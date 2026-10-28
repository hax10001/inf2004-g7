/** @file ir_shared.c
 * @brief Single packed IR state with mutex and legacy layout conversion.
 */
#include "ir_shared.h"
//#include <stddef.h>

#define IR_ENCODER_MAX_AGE_MSEC 40U

static ID gh_ir_mutex = 0;
static ir_shared_compact_t g_packed_ir = {0};
static uint32_t g_encoder_update_msec = 0U;
static uint32_t g_encoder_ticks = 0U;
static uint8_t g_b_have_encoder = 0U;

ER ir_shared_init (void)
{
    T_CMTX mutex_config = {0};
    ID h_mutex = 0;

    mutex_config.mtxatr = TA_INHERIT;
    h_mutex = tk_cre_mtx(&mutex_config);
    if (h_mutex <= E_OK)
    {
        return (h_mutex == E_OK) ? E_OBJ : h_mutex;
    }
    gh_ir_mutex = h_mutex;
    g_packed_ir = (ir_shared_compact_t){0};
    g_encoder_update_msec = 0U;
    g_encoder_ticks = 0U;
    g_b_have_encoder = 0U;
    return E_OK;
}

ER ir_shared_publish (uint32_t left, uint32_t right, uint32_t barcode)
{
    SYSTIM now = {0};
    ER error = E_OK;
    uint8_t packed = 0U;

    if (gh_ir_mutex <= E_OK)
    {
        return E_OBJ;
    }
    if (((left | right | barcode) & ~1U) != 0U)
    {
        return E_PAR;
    }
    error = tk_get_otm(&now);
    if (error < E_OK)
    {
        return error;
    }
    error = tk_loc_mtx(gh_ir_mutex, 0);
    if (error < E_OK)
    {
        return error;
    }
    packed = (uint8_t)((left * IR_SHARED_LEFT_BIT) |
                       (right * IR_SHARED_RIGHT_BIT) |
                       (barcode * IR_SHARED_BARCODE_BIT) |
                       IR_SHARED_VALID_BIT);
    if ((0U != g_b_have_encoder) &&
        ((now.lo - g_encoder_update_msec) <=
         IR_ENCODER_MAX_AGE_MSEC))
    {
        packed |= IR_SHARED_ENCODER_BIT;
    }
    g_packed_ir.flags = packed;
    g_packed_ir.timestamp_msec = now.lo;
    g_packed_ir.travel_ticks = g_encoder_ticks;
    ++g_packed_ir.sample_number;
    return tk_unl_mtx(gh_ir_mutex);
}

ER ir_shared_get_compact (ir_shared_compact_t * p_output)
{
    ER error = E_OK;

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
    *p_output = g_packed_ir;
    return tk_unl_mtx(gh_ir_mutex);
}

ER ir_shared_get_latest (ir_shared_snapshot_t * p_output)
{
    ir_shared_compact_t compact = {0};
    ER error = E_OK;

    if (NULL == p_output)
    {
        return E_PAR;
    }
    error = ir_shared_get_compact(&compact);
    if (error < E_OK)
    {
        return error;
    }
    p_output->left = (compact.flags & IR_SHARED_LEFT_BIT) ? 1U : 0U;
    p_output->right = (compact.flags & IR_SHARED_RIGHT_BIT) ? 1U : 0U;
    p_output->barcode = (compact.flags & IR_SHARED_BARCODE_BIT) ? 1U : 0U;
    p_output->timestamp_msec = compact.timestamp_msec;
    p_output->b_valid = (compact.flags & IR_SHARED_VALID_BIT) != 0U;
    return E_OK;
}

ER ir_shared_encoder_update (uint32_t ticks, bool b_valid)
{
    SYSTIM now = {0};
    ER error = E_OK;

    if (gh_ir_mutex <= E_OK)
    {
        return E_OBJ;
    }
    error = tk_get_otm(&now);
    if (error < E_OK)
    {
        return error;
    }
    /* Poll instead of blocking the Buddy 2 motor task. */
    error = tk_loc_mtx(gh_ir_mutex, 0);
    if (error < E_OK)
    {
        return error;
    }
    if (b_valid && (0U != g_b_have_encoder) &&
        (ticks < g_encoder_ticks))
    {
        b_valid = false;
    }
    g_encoder_ticks = ticks;
    g_b_have_encoder = b_valid ? 1U : 0U;
    g_encoder_update_msec = now.lo;
    return tk_unl_mtx(gh_ir_mutex);
}

/*** end of file ***/

