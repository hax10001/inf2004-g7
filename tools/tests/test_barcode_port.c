/* Host-only barcode port ownership/configuration regression. */
#include "modules/barcode_scanning/bc_port.h"
#include "drivers/encoder.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static int sem_taken=0;
ER tk_get_otm(SYSTIM *now) {*now=(SYSTIM){0};return E_OK;}
ID tk_cre_sem(T_CSEM const *cfg) {
    assert(cfg && cfg->isemcnt==1 && cfg->maxsem==1);
    return 8;
}
ER tk_wai_sem(ID id, W count,TMO timeout) {
    (void)timeout; assert(id==8 && count==1 && !sem_taken);
    sem_taken=1; return E_OK;
}
ER tk_sig_sem(ID id,W count) {
    assert(id==8 && count==1 && sem_taken);
    sem_taken=0; return E_OK;
}
void encoder_read(encoder_wheel_t wheel,encoder_sample_t *p) {
    (void)wheel; p->count=0; p->edge_usec=0;
}
uint32_t encoder_now_usec(void) {return 0U;}
int main(void) {
    bc_port_config_t cfg={0};
    bc_scan_t snapshot={0};
    bc_port_stats_t stats={0};
    cfg.scan.min_run_units=3;
    cfg.scan.leading_units=24;
    cfg.scan.trailing_units=24;
    cfg.scan.min_barcode_units=87;
    cfg.scan.max_barcode_units=240;
    cfg.scan.timeout_msec=2500;
    cfg.left_forward_sign=-1;
    cfg.right_forward_sign=1;
    cfg.max_wheel_step_delta=7;
    cfg.max_sample_travel_units=11;
    cfg.max_sample_interval_msec=18;
    cfg.black_gpio_level=0;
    assert(bc_port_init(&cfg)==E_OK);
    assert(bc_port_snapshot(&snapshot)==E_OK);
    assert(memcmp(&snapshot.cfg,&cfg.scan,sizeof(cfg.scan))==0);
    assert(bc_port_sample_counts(1,1000,0,0,1000,1000,1)==E_OK);
    assert(bc_port_sample_counts(1,2000,-2,2,2000,2000,1)==E_OK);
    assert(bc_port_stats(&stats)==E_OK);
    assert(stats.direction_invalid==0 && stats.samples==2);
    /* Clear must preserve the only copy of the scan configuration. */
    assert(bc_port_clear()==E_OK);
    memset(&snapshot,0,sizeof(snapshot));
    assert(bc_port_snapshot(&snapshot)==E_OK);
    assert(memcmp(&snapshot.cfg,&cfg.scan,sizeof(cfg.scan))==0);
    assert(snapshot.state==BC_SCAN_IDLE);
    assert(bc_port_sample_counts(1,3000,0,0,3000,3000,1)==E_OK);
    assert(bc_port_sample_counts(1,4000,-1,1,4000,4000,1)==E_OK);
    assert(bc_port_stats(&stats)==E_OK);
    assert(stats.direction_invalid==0);
    assert(!sem_taken);
    puts("PASS V11 barcode port: reverse wheel config, persisted scan thresholds, clear and no semaphore leak");
    return 0;
}
