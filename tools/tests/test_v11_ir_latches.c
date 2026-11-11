/* Host-only proof that packed IR latches survive and clear independently. */
#include "modules/line_barcode/ir_shared.h"
#include <assert.h>
#include <stdio.h>

static unsigned held=0;
static uint32_t now=0;
ID tk_cre_mtx(const T_CMTX *config) {
    assert(config->mtxatr == TA_INHERIT);
    return 4;
}
ER tk_loc_mtx(ID mutex,TMO timeout) {
    assert(mutex==4 && timeout==TMO_FEVR && !held);
    held=1;return E_OK;
}
ER tk_unl_mtx(ID mutex) {
    assert(mutex==4 && held);
    held=0;return E_OK;
}
ER tk_get_otm(SYSTIM *time) {
    time->hi=0;time->lo=++now;return E_OK;
}
int main(void) {
    bool left=false,right=false,gap=false,junction=false;
    ir_shared_snapshot_t snapshot={0};
    ir_shared_diag_t d={0};
    assert(ir_shared_init()==E_OK);
    assert(ir_shared_publish(1,1,0)==E_OK);
    assert(ir_shared_get_latest(&snapshot)==E_OK);
    assert(snapshot.left&&snapshot.right&&!snapshot.barcode);
    assert(ir_shared_publish(0,0,0)==E_OK);
    assert(ir_shared_publish(0,1,0)==E_OK);
    assert(ir_shared_take_sides(&left,&right)==E_OK);
    assert(left&&right);
    assert(ir_shared_take_sides(&left,&right)==E_OK);
    assert(!left&&!right);
    assert(ir_shared_take_junction(&junction)==E_OK);
    assert(junction);
    assert(ir_shared_take_junction(&junction)==E_OK);
    assert(!junction);
    assert(ir_shared_take_both_gap(&gap)==E_OK);
    assert(gap);
    assert(ir_shared_take_both_gap(&gap)==E_OK);
    assert(!gap);
    assert(ir_shared_take_diag(&d)==E_OK);
    assert(d.samples==3u&&d.left==1u&&d.right==2u&&d.both==1u);
    assert(!held);
    puts("PASS V11 IR packed latches: no lost events; independent take/clear");
    return 0;
}
