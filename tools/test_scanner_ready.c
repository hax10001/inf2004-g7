/* Real sampler: held readiness survives START and catches the first stripe. */
#include <assert.h>
#include <stdio.h>
#define B1_LIVE_COMMANDS 1
#include "modules/barcode_scanning/bc_wave.c"

static uint32_t now;
static unsigned int captures;
static unsigned int rejections;
bc_match_result_t bc_match_result(void) {return (bc_match_result_t){0};}
bc_layout_result_t bc_layout_result(void)
{return (bc_layout_result_t){.reason=BC_LAYOUT_FEW_BARS,.letter=255U};}
char const *bc_layout_reason_text(uint8_t reason)
{(void)reason;return "few bars";}
ER bc_port_publish_buffered(bc_code_result_t const *result,uint32_t ms)
{(void)result;(void)ms;assert(0 && "unreadable capture cannot publish a letter");return E_OK;}
ER bc_port_publish_buffered_rejection(uint8_t code,uint32_t ms)
{(void)ms;assert(code==BC_SCAN_ERROR_PATTERN);++rejections;return E_OK;}
ER bc_port_nav_finish(void) {return E_OK;}
INT tm_printf(UB const *f,...) {(void)f;return 0;}
uint32_t encoder_now_usec(void) {return now;}
ER bc_port_nav_begin(uint32_t ms) {(void)ms;++captures;return E_OK;}
static unsigned int aborts;
ER bc_port_nav_abort(void) {++aborts;return E_OK;}
ER bc_port_nav_captured(uint32_t ms) {(void)ms;return E_OK;}
/* bc_wave_poll() is called only with no capture to decode, so the decoder
 * is never reached; these satisfy the linker and fail loudly if it is. */
bc_code_result_t bc_code_decode(uint32_t const *w,uint32_t m)
{(void)w;(void)m;assert(0 && "no decode expected");
 return (bc_code_result_t){0};}
void bc_layout_begin(uint32_t n,bc_layout_read_t r,uint8_t l)
{(void)n;(void)r;(void)l;assert(0 && "no decode expected");}
uint8_t bc_layout_poll(void) {assert(0 && "no decode expected");return 1U;}
uint8_t bc_scan_init(bc_scan_t *s,bc_scan_config_t const *c)
{(void)s;(void)c;return 1U;}
uint8_t bc_scan_feed(bc_scan_t *s,uint8_t b,uint32_t u,uint32_t ms,uint8_t v)
{(void)s;(void)b;(void)u;(void)ms;(void)v;return 0U;}
uint8_t bc_scan_take(bc_scan_t *s,uint32_t *w,uint8_t *e,uint32_t *ts)
{(void)s;(void)w;(void)e;(void)ts;return 0U;}

static unsigned int captures_seen;
/* Captures started since the last call. */
static unsigned int captures_since(void)
{unsigned int const n=captures-captures_seen;captures_seen=captures;return n;}
static int32_t g_cnt;
/* Drive n samples forward at `step` encoder units per 2 ms sample. */
static void drive(uint16_t adc, unsigned int n, int32_t step);
static void sample(uint16_t adc, int32_t count)
{
    now += BC_FAST_PERIOD_USEC;
    bc_fastq_sample_t s = {.time_usec=now,.left_count=count,
        .right_count=count,.adc_raw=adc,.valid=1U,.adc_valid=1U};
    bc_wave_sample(&s);
}

static void drive(uint16_t adc, unsigned int n, int32_t step)
{
    for (unsigned int i = 0U; i < n; i++)
    {
        g_cnt += step;
        sample(adc, g_cnt);
    }
}

/* Issue 1 fixes: a shadow cannot start a capture, and a line or tape
 * crossing is dropped early while the scanner stays armed. */
static void trigger_and_drop(void)
{
    /* Armed on a white of 300 while driving. Clear requests left by the
     * gating checks above. */
    g_relearn_requested=0U;g_cancel_requested=0U;g_white_mean=300U;
    g_phase=BC_WAVE_DONE;bc_wave_request_rearm();
    wave_rearm_completed_capture();
    g_cnt=g_left;
    drive(300U, BC_WAVE_WHITE_SAMPLES*2U+10U, 3);
    assert(bc_wave_is_ready() && g_white_mean==300U);
    (void)captures_since();
    /* Shadow / dirty floor: 500 counts darker than white. */
    drive(800U, 100U, 3);
    drive(300U, 50U, 3);
#if APP_BARCODE_TRIGGER_RISE >= 500U
    assert(!captures_since() && bc_wave_is_ready());
#else
    assert(captures_since()==1U); /* previous behaviour */
    g_phase=BC_WAVE_WAIT;
#endif
    /* Tape crossing: 60 units of ink, then white past the end gap. */
    {
        unsigned int const aborts_before=aborts;
        drive(3000U, 20U, 3);
        assert(g_phase==BC_WAVE_CAPTURE && captures_since()==1U);
        drive(300U, 200U, 3);
#if APP_BARCODE_MIN_DARK_UNITS
        assert(g_phase==BC_WAVE_WAIT && bc_wave_is_ready());
        assert(aborts==aborts_before+1U);
        assert(g_count<=BC_WAVE_PRETRIGGER);
#else
        assert(g_phase==BC_WAVE_CAPTURE && aborts==aborts_before);
        g_phase=BC_WAVE_WAIT;
#endif
    }
    /* Barcode-like: bars across 1500 units, then white. Not dropped:
     * it is frozen for the decoder. */
    {
        unsigned int const aborts_before=aborts;
        for (unsigned int bar=0U; bar<12U; bar++)
        {
            drive(3000U, 20U, 3);
            drive(300U, 20U, 3);
        }
        drive(3000U, 20U, 3);
        assert(g_phase==BC_WAVE_CAPTURE && captures_since()==1U);
        drive(300U, 200U, 3);
        assert(g_phase==BC_WAVE_SMOOTH && aborts==aborts_before);
        assert(g_freeze_reason==4U);
    }
    g_phase=BC_WAVE_DONE;
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
#if APP_BARCODE_KEEP_STARTUP_WHITE
    assert(g_startup_white_valid && g_startup_white_mean==200U);
#endif
    (void)captures_since(); /* Only count captures from here on. */
    /* After a barcode the same floor reads slightly differently: the
     * startup reference is kept (or re-learned with the setting off). */
    g_phase=BC_WAVE_DONE;bc_wave_request_rearm();
    wave_rearm_completed_capture();
    for(unsigned i=0;i<BC_WAVE_WHITE_SAMPLES*2;i++) sample(205U,1);
    assert(bc_wave_is_ready());
    assert(g_white_mean==(APP_BARCODE_KEEP_STARTUP_WHITE ? 200U : 205U));
    /* Lighter floor beyond sigma + 8 (here 230 vs 200, sigma 0) is not
     * the startup white: using 200 there would read the floor as ink. */
    g_phase=BC_WAVE_DONE;bc_wave_request_rearm();
    wave_rearm_completed_capture();
    for(unsigned i=0;i<BC_WAVE_CLEAR_MAX_SAMPLES+BC_WAVE_WHITE_SAMPLES+10U;i++)
        sample(230U,1);
    assert(bc_wave_is_ready() && g_white_mean==230U && !captures_since());
    /* A floor that no longer matches uses its own reading, so the scanner
     * cannot lock up triggering on it; the startup reference survives. */
    g_phase=BC_WAVE_DONE;bc_wave_request_rearm();
    wave_rearm_completed_capture();
    for(unsigned i=0;i<BC_WAVE_CLEAR_MAX_SAMPLES+BC_WAVE_WHITE_SAMPLES+10U;i++)
        sample(400U,1);
    assert(bc_wave_is_ready() && g_white_mean==400U && !captures_since());
#if APP_BARCODE_KEEP_STARTUP_WHITE
    assert(g_startup_white_mean==200U);
#endif
    /* Back on the original floor: the startup reference applies again. */
    g_phase=BC_WAVE_DONE;bc_wave_request_rearm();
    wave_rearm_completed_capture();
    for(unsigned i=0;i<BC_WAVE_CLEAR_MAX_SAMPLES+BC_WAVE_WHITE_SAMPLES+10U;i++)
        sample(205U,1);
    assert(bc_wave_is_ready());
    assert(g_white_mean==(APP_BARCODE_KEEP_STARTUP_WHITE ? 200U : 205U));
    /* An explicit recalibration replaces the startup reference. */
    bc_wave_request_relearn();
    bc_wave_poll();
    assert(g_phase==BC_WAVE_LEARN);
    for(unsigned i=0;i<BC_WAVE_WHITE_SAMPLES;i++) sample(300U,1);
    assert(bc_wave_is_ready() && g_white_mean==300U);
#if APP_BARCODE_KEEP_STARTUP_WHITE
    assert(g_startup_white_valid && g_startup_white_mean==300U);
#endif
    bc_wave_request_cancel();
    assert(!bc_wave_is_ready());
    bc_wave_status(&status);
    assert(status.state!=BC_WAVE_STATE_READY);
    g_cancel_requested=0U;
    bc_wave_request_relearn();
    assert(!bc_wave_is_ready());
    /* Even a capture filtered as non-barcode-like must have a failed result.
     * This exercises the real report path, not a guessed UI timeout. */
    g_count=0U;g_sequence=7U;g_phase=BC_WAVE_REPORT;
    wave_report();
    bc_wave_status(&status);
    assert(rejections==1U && !status.last_valid && status.sequence==7U);
    assert(status.last_reason==BC_WAVE_REASON_IGNORED);
    trigger_and_drop();
    puts("PASS scanner capture gate: shadow ignored, tape crossing dropped "
         "and still armed, barcode-like capture frozen for decode");
    puts(APP_BARCODE_KEEP_STARTUP_WHITE ?
         "PASS real scanner: persistent held READY, first-stripe capture, "
         "startup white kept across rearms, drifted floor and recalibration, "
         "CLEAR/DONE/cancel/relearn cannot authorize START" :
         "PASS real scanner (re-learn white each rearm): held READY, "
         "first-stripe capture, CLEAR/DONE/cancel/relearn gating");
    return 0;
}
