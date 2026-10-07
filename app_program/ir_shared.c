#include <tk/tkernel.h>
#include "ir_shared.h"

/* Only this module can directly access these variables. */
static ID ir_mutex = 0;

static IR_SNAPSHOT latest_ir = {
    .left = 0U,
    .right = 0U,
    .barcode = 0U,
    .timestamp_ms = 0U,
    .valid = FALSE
};


/* Initialize once from usermain(), before starting tasks. */
ER ir_shared_init(void)
{
    T_CMTX cmtx = {0};

    cmtx.mtxatr = TA_INHERIT;

    ID id = tk_cre_mtx(&cmtx);

    if (id <= E_OK) {
        return (ER)id;
    }

    ir_mutex = id;

    return E_OK;
}


/* Called by ir_task(). */
ER ir_shared_publish(UINT left, UINT right, UINT barcode)
{
    SYSTIM now;
    ER er;

    if (ir_mutex <= E_OK) {
        return E_OBJ;
    }

    er = tk_get_otm(&now);

    if (er < E_OK) {
        return er;
    }

    er = tk_loc_mtx(ir_mutex, TMO_FEVR);

    if (er < E_OK) {
        return er;
    }

    latest_ir.left = left;
    latest_ir.right = right;
    latest_ir.barcode = barcode;
    latest_ir.timestamp_ms = now.lo;
    latest_ir.valid = TRUE;

    return tk_unl_mtx(ir_mutex);
}


/* Called by controller_task(). */
ER ir_get_latest(IR_SNAPSHOT *out)
{
    ER er;

    if (out == NULL) {
        return E_PAR;
    }

    if (ir_mutex <= E_OK) {
        return E_OBJ;
    }

    er = tk_loc_mtx(ir_mutex, TMO_FEVR);

    if (er < E_OK) {
        return er;
    }

    *out = latest_ir;

    return tk_unl_mtx(ir_mutex);
}