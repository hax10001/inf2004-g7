/* Host-only startup/logger regression. Never enters firmware discovery. */
#include "app_main.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "platform/console.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static jmp_buf g_complete;
static char const *g_case;
static int g_motor_init;
static int g_motor_stop;
static int g_encoder_init;
static int g_go;
static int g_rows;
static int g_invalid;
static int g_done;
static int g_finished;
static unsigned g_ticks;
static int32_t g_left;
static int32_t g_right;
static unsigned g_prep_reads;
static uint32_t g_usec = 0xFFFFF000U;

void motor_init(void) { ++g_motor_init; }
void motor_stop(void) { ++g_motor_stop; }
ER encoder_init(void)
{
    assert(g_motor_init == 1 && g_motor_stop == 1);
    ++g_encoder_init;
    return strcmp(g_case, "failure") == 0 ? E_OBJ : E_OK;
}
void encoder_read(encoder_wheel_t wheel, encoder_sample_t *sample)
{
    sample->count = wheel == ENCODER_LEFT ? g_left : g_right;
    sample->edge_usec = 0U;
}
uint32_t encoder_now_usec(void) { return g_usec; }
UW console_state(void)
{
    /* Simulate USB arriving late: the experiment must not start first. */
    ++g_prep_reads;
    assert(g_prep_reads > 3 || g_go == 0);
    return g_prep_reads > 3 ? 3U : 2U;
}
ER tk_dly_tsk(UINT delay)
{
    /* Include scheduler overshoot; start near timer wrap deliberately. */
    g_usec += (delay + 10U) * 1000U;
    if (delay == 20U)
    {
        assert(g_go > 0 && g_motor_stop == 1);
        ++g_ticks;
        if (g_ticks <= 25 || strcmp(g_case, "moving") == 0)
        {
            g_left += 2;
            if (strcmp(g_case, "zero") != 0) { g_right -= 3; }
        }
    }
    return E_OK;
}
ER tk_slp_tsk(TMO timeout)
{
    assert(timeout == TMO_FEVR && g_finished == 1);
    longjmp(g_complete, 1);
}
INT tm_printf(UB const *format, ...)
{
    char text[512];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), (char const *)format, args);
    va_end(args);
    if (strstr(text, "[ENC CAL] GO pass="))
    {
        ++g_go;
        g_ticks = 0;
        g_usec = 0xFFFFF000U;
    }
    if (strstr(text, "[ENC CAL CSV]") && !strstr(text, "requested_mm"))
    {
        unsigned pass, requested, still;
        int ls, le, rs, re, dl, dr;
        assert(sscanf(text, "[ENC CAL CSV] %u,%u,%d,%d,%d,%d,%d,%d,%u",
                      &pass, &requested, &ls, &le, &rs, &re, &dl, &dr,
                      &still) == 9);
        ++g_rows;
        assert(pass == (unsigned)g_rows && requested == 100);
        assert(le - ls == dl && re - rs == dr);
        if (strcmp(g_case, "normal") == 0)
        {
            assert(dl == 50 && dr == -75 && still >= 1000);
        }
    }
    g_invalid += strstr(text, "INVALID pass=") != NULL;
    g_done += strstr(text, "DONE pass=") != NULL;
    g_finished += strstr(text, "SESSION COMPLETE") != NULL;
    return (INT)strlen(text);
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    g_case = argv[1];
    if (setjmp(g_complete) == 0)
    {
        int result = usermain();
        assert(strcmp(g_case, "failure") == 0 && result == 1);
    }
    assert(g_motor_init == 1 && g_motor_stop == 1 && g_encoder_init == 1);
    if (strcmp(g_case, "failure") == 0)
    {
        assert(g_go == 0 && g_rows == 0 && g_finished == 0);
    }
    else
    {
        assert(g_go == 3 && g_rows == 3 && g_finished == 1);
        assert(g_done == (strcmp(g_case, "normal") == 0 ? 3 : 0));
        assert(g_invalid == (strcmp(g_case, "normal") == 0 ? 0 : 3));
    }
    printf("PASS encoder-only startup/logger: %s\n", g_case);
    return 0;
}
