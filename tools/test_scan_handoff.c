/* Exercise the actual port semaphore and independent navigation reader. */
#include <assert.h>
#include <stdio.h>
#include "modules/barcode_scanning/bc_port.c"

static int busy, locked;
ER tk_get_otm(SYSTIM *now) {*now=(SYSTIM){.lo=500U};return E_OK;}
ER tk_wai_sem(ID id, INT count, TMO timeout)
{
    assert(id==42 && count==1 && !locked);
    if (busy && timeout==TMO_POL) return E_TMOUT;
    locked=1;return E_OK;
}
ER tk_sig_sem(ID id, INT count)
{assert(id==42 && count==1 && locked);locked=0;return E_OK;}

int main(void)
{
    scan_handoff_t scan={0};
    bc_port_result_t telemetry={0};
    bc_code_result_t g={6U,2U,0U,1U};
    gh_port_lock=42;
    assert(sizeof(scan_handoff_t)==12U);
    assert(bc_port_nav_enable(1U)==E_OK);
    assert(bc_port_nav_begin(100U)==E_OK);
    assert(bc_port_nav_take(&scan)==E_OK && scan.busy && !scan.ready);
    assert(bc_port_publish_buffered(&g,100U)==E_OK);
    assert(bc_port_nav_finish()==E_OK); /* no second event */
    assert(bc_port_peek(&telemetry)==E_OK);
    assert(bc_port_ack(telemetry.sequence)==E_OK);
    /* Telemetry ACK and decoder idle cannot erase the controller result. */
    busy=1;assert(bc_port_nav_take(&scan)==E_TMOUT);busy=0;
    assert(bc_port_nav_take(&scan)==E_OK && scan.ready && !scan.busy);
    assert(scan.letter==6U && scan.sequence==telemetry.sequence);
    assert(scan.captured_msec==100U);
    assert(scan.completed_msec==500U);
    assert(bc_port_nav_take(&scan)==E_OK && !scan.ready);
    for (unsigned i=0;i<10;i++)
    {
        assert(bc_port_nav_begin(200U+i*2000U)==E_OK);
        assert(bc_port_publish_buffered(&g,200U+i*2000U)==E_OK);
        assert(bc_port_nav_take(&scan)==E_OK && scan.ready);
        assert(scan.sequence!=(uint16_t)telemetry.sequence);
        assert(bc_port_peek(&telemetry)==E_OK);
        assert(bc_port_ack(telemetry.sequence)==E_OK);
    }
    /* Old worker completing after STOP then START has no ownership. */
    assert(bc_port_nav_begin(30000U)==E_OK);
    assert(bc_port_nav_enable(0U)==E_OK);
    assert(bc_port_nav_enable(1U)==E_OK);
    assert(bc_port_publish_buffered(&g,30000U)==E_OK);
    assert(bc_port_nav_finish()==E_OK);
    assert(bc_port_nav_take(&scan)==E_OK && !scan.ready && !scan.busy);
    /* Failure and ignored marks terminate explicitly; no invented letter. */
    assert(bc_port_nav_begin(40000U)==E_OK);
    assert(bc_port_nav_finish()==E_OK);
    assert(bc_port_nav_take(&scan)==E_OK && scan.ready && scan.letter==31U);
    /* Sequence zero is a genuine wrapped event, not an invalid sentinel. */
    g_event_sequence=65535U;
    assert(bc_port_nav_begin(45000U)==E_OK);
    (void)bc_port_publish_buffered(&g,45000U);
    assert(bc_port_nav_take(&scan)==E_OK && scan.sequence==0U && scan.ready);
    /* If producer outruns its consumer, fail closed rather than choose. */
    assert(bc_port_nav_begin(50000U)==E_OK);
    (void)bc_port_publish_buffered(&g,50000U);
    assert(bc_port_nav_begin(51000U)==E_OK);
    (void)bc_port_publish_buffered(&g,51000U);
    assert(bc_port_nav_take(&scan)==E_OK && scan.overflow && scan.letter==31U);
    puts("V15 scan port: independent ACK, idle/result atomicity, contention, "
         "cancelled worker, repeated scans, failure, overflow and wrap PASS");
    return 0;
}
