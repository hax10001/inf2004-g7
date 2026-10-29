/** @file test_live.c
 * @brief Register-model regression tests of actual barcode integration.
 */
#include "test_live.h"

#ifdef BC_LIVE_TEST_MAIN
#include "bc_setup.h"
#include "bc_fast.h"
#include "bc_fastq.h"
#include "bc_port.h"
#include "drivers/encoder.h"
#include "app/tasks.h"
#include "platform/console.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static uint32_t g_now = 100000U;
static uint32_t g_alarm;
static uint32_t g_inte;
static uint32_t g_armed;
static uint32_t g_capture_delay;
static uint32_t g_write_delay;
static uint8_t g_raw;
static int32_t g_left;
static int32_t g_right;
static void (*g_handler)(UINT);
static ER g_create_error = 7;
static ER g_start_error = E_OK;
static int g_fail;
static int g_checks;
static int g_detected;
static int g_rejected;
static int g_measure_messages;
static int g_edges;
static char g_message[256];

static void check (uint8_t ok, char const * p_label)
{
    ++g_checks;
    if (0U == ok)
    {
        ++g_fail;
        printf("FAIL: %s\n", p_label);
    }
}

ID tk_cre_mtx (T_CMTX const * p_config)
{
    (void)p_config;
    return 7;
}

ER tk_loc_mtx (ID mutex, TMO timeout)
{
    (void)mutex;
    (void)timeout;
    return E_OK;
}

ER tk_unl_mtx (ID mutex)
{
    (void)mutex;
    return E_OK;
}

ID tasks_create (tasks_entry_t entry, PRI priority)
{
    (void)entry;
    (void)priority;
    return g_create_error;
}

ER tk_sta_tsk (ID id, INT code)
{
    (void)id;
    (void)code;
    return g_start_error;
}

ER tk_dly_tsk (UINT delay)
{
    (void)delay;
    return E_OK;
}

UW console_state (void)
{
    return 3U;
}

INT tm_printf (UB const * p_format, ...)
{
    va_list args;
    va_start(args, p_format);
    (void)vsnprintf(g_message, sizeof(g_message),
                    (char const *)p_format, args);
    va_end(args);
    g_detected += (0 != strstr(g_message, "Detected: F"));
    g_rejected += (0 != strstr(g_message, "Rejected:"));
    g_measure_messages += (0 != strstr(g_message, "Measurement mode"));
    g_edges += (0 != strstr(g_message, "[BC CAL EDGE]"));
    return (INT)strlen(g_message);
}

void encoder_read (encoder_wheel_t wheel, encoder_sample_t * p_sample)
{
    p_sample->count = (ENCODER_LEFT == wheel) ? g_left : g_right;
    p_sample->edge_usec = g_now;
}

uint32_t encoder_now_usec (void)
{
    return g_now;
}

UW in_w (UW address)
{
    if (TIMER_INTE == address)
    {
        return g_inte;
    }
    if (TIMER_ARMED == address)
    {
        return g_armed;
    }
    if (GPIO_IN == address)
    {
        g_now += g_capture_delay;
        return (UW)g_raw << 27U;
    }
    return 0U;
}

void out_w (UW address, UW value)
{
    if (TIMER_ALARM2 == address)
    {
        g_now += g_write_delay;
        g_alarm = value;
        g_armed |= 4U;
    }
    if (TIMER_ARMED == address)
    {
        g_armed &= ~value;
    }
    if (TIMER_INTE == address)
    {
        g_inte = value;
    }
}

ER tk_def_int (UINT number, T_DINT const * p_handler)
{
    (void)number;
    g_handler = (void (*)(UINT))p_handler->inthdr;
    return E_OK;
}

void ClearInt (UINT number)
{
    (void)number;
}

void EnableInt (UINT number, INT priority)
{
    (void)number;
    (void)priority;
}

static void capture (uint8_t raw, int32_t left, int32_t right)
{
    g_now = g_alarm;
    g_left = left;
    g_right = right;
    g_raw = raw;
    g_armed = 0U;
    g_handler(2U);
    check(E_OK == bc_setup_poll(), "worker service");
}

static void frame_f (void)
{
    uint32_t widths[29] = {0};
    uint16_t symbols[3] = {0x94U, 0x58U, 0x94U};
    uint8_t index = 0U;
    for (uint8_t symbol = 0U; symbol < 3U; ++symbol)
    {
        for (uint8_t bit = 0U; bit < 9U; ++bit)
        {
            widths[index++] =
                (symbols[symbol] & (1U << (8U - bit))) ? 60U : 20U;
        }
        if (symbol < 2U)
        {
            widths[index++] = 20U;
        }
    }
    int32_t base_l = g_left;
    int32_t base_r = g_right;
    for (uint32_t position = 0U; position <= 1180U; position += 4U)
    {
        uint8_t raw = 0U;
        if ((position >= 120U) && (position < 1060U))
        {
            uint32_t end = 120U;
            for (uint8_t run = 0U; run < 29U; ++run)
            {
                end += widths[run];
                if (position < end)
                {
                    raw = (0U == (run & 1U)) ? 1U : 0U;
                    break;
                }
            }
        }
        capture(raw, base_l + (int32_t)(position / 2U),
                     base_r + (int32_t)(position / 2U));
    }
}

int test_live_run (char const * p_case)
{
    bc_port_result_t result = {0};
    if (0 == strcmp(p_case, "wrap"))
    {
        g_now = 0xFFFFF000U;
    }
    if (0 == strcmp(p_case, "conflict"))
    {
        g_inte = 4U;
    }
    check(E_OK == bc_setup_init(), "barcode init never aborts robot");
    if (0 == strcmp(p_case, "worker"))
    {
        g_create_error = E_OBJ;
    }
    check(E_OK == bc_setup_start(), "barcode start never aborts robot");
    if ((0 == strcmp(p_case, "conflict")) ||
        (0 == strcmp(p_case, "worker")))
    {
        check(bc_setup_poll() < E_OK, "failure reported by worker");
        check(E_OK == bc_setup_sample(0U, 0U), "inactive hook succeeds");
        check(0 == strstr(g_message, "Detected:"), "no false result");
        if (0 == strcmp(p_case, "conflict"))
        {
            check(4U == g_inte, "other alarm owner is untouched");
        }
    }
    else
    {
        check(E_OK == bc_setup_sample(0U, 0U), "early IR hook");
        check(0U == g_armed, "wait for worker before arming");
        check(E_OK == bc_setup_poll(), "worker ready");
        check(E_OK == bc_setup_sample(0U, 0U), "arm after worker ready");
        if (0 == BC_CAL_READY)
        {
            check(0U == g_armed, "disabled mode owns no alarm");
        }
        else if (0 == strcmp(p_case, "late"))
        {
            g_now = g_alarm + 3999U;
            g_capture_delay = 3U;
            g_handler(2U);
            check((int32_t)(g_alarm - g_now) > 0,
                  "late ISR programs an alarm in the future");
            check(E_OK == bc_fast_error(), "late ISR recovers");
            g_capture_delay = 0U;
            check(E_OK == bc_setup_poll(), "late record handled");
            frame_f();
            check(1 == g_detected, "scanning continues after late IRQ");
        }
        else if (0 == strcmp(p_case, "alarm"))
        {
            g_now = g_alarm;
            g_write_delay = 5000U;
            g_handler(2U);
            check(bc_setup_poll() < E_OK, "stalled hardware reported");
            check(0U == g_inte, "failed timer interrupt disabled");
            check(0U == g_armed, "failed timer disarmed");
            check(E_OK == bc_setup_sample(0U, 0U), "robot hook survives");
        }
        else if ((0 != BC_CAL_MEASURE) || (0U == BC_CAL_MIN_RUN_UNITS))
        {
            check(1 == g_measure_messages, "measurement fallback reported");
            capture(0U, 0, 0);
            capture(1U, 2, 2);
            check(0 != strstr(g_message, "units=4"),
                  "calibration edge reports actual encoder travel");
            check(0 != strstr(g_message, "level=0"),
                  "calibration reports raw polarity");
            for (uint32_t i = 0U; i < 125U; ++i)
            {
                capture(1U, (int32_t)i + 2, (int32_t)i + 2);
            }
            check(0 != strstr(g_message, "gap_us=4000"),
                  "calibration measures real capture gaps");
            check(g_edges > 0, "calibration edge log");
            check(0 == g_detected, "measurement mode never decodes");
        }
        else
        {
            frame_f();
#if BC_CAL_AUTO_ACK
            check(E_OBJ == bc_port_peek(&result), "first event released");
            frame_f();
            check(2 == g_detected, "two consecutive barcodes reported");
            check(0 == g_rejected, "no busy or wrap rejection");
            check(E_OBJ == bc_port_peek(&result), "second event released");
#else
            check(E_OK == bc_port_peek(&result), "mission retains result");
            check(E_OK == bc_port_ack(result.sequence), "mission ACK");
            frame_f();
            check(2 == g_detected, "mission can accept next barcode");
#endif
        }
    }
    printf("Barcode integration (%s): %d assertions, %d failed\n",
           p_case, g_checks, g_fail);
    return g_fail;
}

int main (int argc, char ** pp_argv)
{
    return (0 == test_live_run((argc > 1) ? pp_argv[1] : "scan")) ? 0 : 1;
}
#endif

/*** end of file ***/

