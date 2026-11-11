/* Real sampler: held readiness survives START and catches the first stripe. */
#include <assert.h>
#include <stdio.h>
#define B1_LIVE_COMMANDS 1
#include "modules/barcode_scanning/bc_wave.c"

static uint32_t now;
static unsigned int captures;
INT tm_printf(UB const *f,...) {(void)f;return 0;}
uint32_t encoder_now_usec(void) {return now;}
ER bc_port_nav_begin(uint32_t ms) {(void)ms;++captures;return E_OK;}
ER bc_port_nav_captured(uint32_t ms) {(void)ms;return E_OK;}

static void sample(uint16_t adc, int32_t count)
{
    now += BC_FAST_PERIOD_USEC;
    bc_fastq_sample_t s = {.time_usec=now,.left_count=count,
        .right_count=count,.adc_raw=adc,.valid=1U,.adc_valid=1U};
    bc_wave_sample(&s);
}

int main(void)
{
    bc_wave_status_t status={0};
    bc_wave_set_suppressed(1U);
    assert(!bc_wave_is_ready());
    for(unsigned i=0;i<BC_WAVE_WHITE_SAMPLES;i++) sample(200U,0);
    assert(bc_wave_is_ready());
    for(unsigned i=0;i<1000;i++) sample(200U,0);
    assert(bc_wave_is_ready() && !captures);
    uint32_t const white=g_white_mean, count=g_count, sequence=g_sequence;
    /* START only lifts suppression; no reset or relearning window. */
    bc_wave_set_suppressed(0U);
    assert(bc_wave_is_ready() && g_white_mean==white && g_count==count);
    sample(1600U,1);
    assert(g_phase==BC_WAVE_CAPTURE && captures==1U);
    assert(g_sequence==sequence+1U && !bc_wave_is_ready());
    g_phase=BC_WAVE_DONE;
    assert(bc_wave_can_rearm() && !bc_wave_is_ready());
    bc_wave_request_rearm();
    wave_rearm_completed_capture();
    assert(g_phase==BC_WAVE_CLEAR && !bc_wave_is_ready());
    for(unsigned i=0;i<BC_WAVE_WHITE_SAMPLES*2;i++) sample(200U,1);
    assert(bc_wave_is_ready());
    bc_wave_request_cancel();
    assert(!bc_wave_is_ready());
    bc_wave_status(&status);
    assert(status.state!=BC_WAVE_STATE_READY);
    g_cancel_requested=0U;
    bc_wave_request_relearn();
    assert(!bc_wave_is_ready());
    puts("PASS real scanner: persistent held READY, first-stripe capture, "
         "CLEAR/DONE/cancel/relearn cannot authorize START");
    return 0;
}
