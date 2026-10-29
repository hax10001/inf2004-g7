/** @file test_port.c
 * @brief Mocked integration verification; firmware does not run this file.
 */
#include "test_port.h"

#ifdef BC_PORT_HOST_TEST_MAIN
#include "bc_port.h"
#include "bc_uart.h"
#include "bc_code.h"
#include "drivers/encoder.h"
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define BC_FAST_TEST_INTERVAL_USEC 4000U

static int32_t g_left_count = 0;
static int32_t g_right_count = 0;
static int g_print_count = 0;
static int g_fail = 0;
static int g_checks = 0;
static char g_last_message[128];
static uint32_t g_time = 0U;
static uint8_t gb_stale_edge = 0U;
static pthread_mutex_t g_test_mutex = PTHREAD_MUTEX_INITIALIZER;

static void check (uint8_t ok, char const * p_name)
{
    ++g_checks;
    if (0U == ok)
    {
        ++g_fail;
        printf("FAIL: %s\n", p_name);
    }
}

ID tk_cre_mtx (T_CMTX const * p_config)
{
    (void)p_config;
    return 7;
}

ER tk_loc_mtx (ID mutex, TMO timeout)
{
    int rc;
    if (7 != mutex)
    {
        return E_OBJ;
    }
    rc = (0 == timeout) ? pthread_mutex_trylock(&g_test_mutex) :
         pthread_mutex_lock(&g_test_mutex);
    return (0 == rc) ? E_OK : E_OBJ;
}

ER tk_unl_mtx (ID mutex)
{
    if (7 != mutex)
    {
        return E_OBJ;
    }
    return (0 == pthread_mutex_unlock(&g_test_mutex)) ? E_OK : E_OBJ;
}

void encoder_read (encoder_wheel_t wheel, encoder_sample_t * p_sample)
{
    p_sample->count = (ENCODER_LEFT == wheel) ?
                      g_left_count : g_right_count;
    p_sample->edge_usec = g_time * 1000U -
        ((0U != gb_stale_edge) ? 200000U : 0U);
}

uint32_t encoder_now_usec (void)
{
    return g_time * 1000U;
}

INT tm_printf (UB const * p_format, ...)
{
    va_list args;
    va_start(args, p_format);
    (void)vsnprintf(g_last_message, sizeof(g_last_message),
                    (char const *)p_format, args);
    va_end(args);
    ++g_print_count;
    return (INT)strlen(g_last_message);
}

static void tick (uint32_t units, uint8_t b_black)
{
    /* A transition may occur at the boundary, with zero travel; otherwise
     * simulate multiple 10 ms polls, never crossing 4 travel units.
     */
    uint32_t remaining = units;
    do
    {
        uint32_t part = (remaining > 4U) ? 4U : remaining;
        g_left_count += (int32_t)(part / 2U);
        g_right_count += (int32_t)(part / 2U);
        g_time += 10U;
        check(E_OK == bc_port_sample(b_black, g_time), "sample");
        check(E_OK == bc_port_process(), "decode process");
        remaining -= part;
    } while (remaining > 0U);
}

static void send_symbol (uint16_t pattern)
{
    for (uint8_t i = 0U; i < 9U; ++i)
    {
        uint32_t width = (0U != (pattern & (1U << (8U - i)))) ?
                         60U : 20U;
        uint8_t black = ((i & 1U) == 0U) ? 1U : 0U;
        tick(width, black);
        tick(0U, (uint8_t)(black ^ 1U));
    }
}

typedef struct
{
    uint16_t expected;
    uint32_t errors;
} test_reader_t;

static void * test_reader (void * p_arg)
{
    test_reader_t * p_reader = (test_reader_t *)p_arg;
    for (uint16_t i = 0U; i < 2000U; ++i)
    {
        bc_port_result_t value = {0U, 0U, 0U};
        ER result = bc_port_peek(&value);
        if ((result != E_OK) ||
            (value.sequence != p_reader->expected) ||
            ((value.info & BC_PORT_LETTER_MASK) != 5U))
        {
            ++p_reader->errors;
        }
    }
    return 0;
}

static void test_parallel_reads (uint16_t sequence)
{
    pthread_t workers[3];
    test_reader_t readers[3] = {{sequence, 0U},
                                {sequence, 0U},
                                {sequence, 0U}};
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        check(0 == pthread_create(&workers[i], 0,
                                   test_reader, &readers[i]),
              "reader started");
    }
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        (void)pthread_join(workers[i], 0);
        check(readers[i].errors == 0U, "parallel snapshot consistent");
    }
}

static void test_full_scan (void)
{
    bc_port_config_t cfg = {
        {8U, 100U, 100U, 600U, 1500U, 12000U},
        1, 1, 8U, 4U, 15U, 1U
    };
    bc_code_pattern_t pattern;
    bc_port_result_t result = {0U, 0U, 0U};
    uint16_t sequence;

    check(E_OK == bc_port_init(&cfg), "init");
    check(E_OBJ == bc_port_peek(&result), "no early result");
    tick(0U, 0U);
    tick(120U, 0U);
    tick(0U, 1U);
    send_symbol(0x94U);
    tick(20U, 0U);
    tick(0U, 1U);
    check(bc_code_get(5U, &pattern), "pattern F");
    send_symbol(pattern.element_pattern);
    tick(20U, 0U);
    tick(0U, 1U);
    send_symbol(0x94U);
    tick(120U, 0U);

    check(E_OK == bc_port_peek(&result), "pending result");
    check((result.info & BC_PORT_LETTER_MASK) == 5U,
          "pending letter F only");
    check(0U != (result.info & BC_PORT_VALID_FLAG), "valid flag");
    check(0U != (result.info & BC_PORT_COMPLETE_FLAG),
          "complete flag");
    check(0U == (result.info & 0xC000U), "reserved bits zero");
    sequence = result.sequence;
    test_parallel_reads(sequence);
    check(E_OBJ == bc_port_ack((uint16_t)(sequence + 1U)),
          "wrong ack ignored");
    check(E_OK == bc_port_ack(sequence), "correct ack");
    check(E_OBJ == bc_port_peek(&result), "result cleared");
    /* ACK must not discard the separate once-only debug queue. */
    check(E_OK == bc_uart_poll(), "diag first");
    check(1 == g_print_count, "one diagnostic");
    check(0 == strcmp(g_last_message, "[BARCODE] Detected: F\n"),
          "UART exact letter");
    check(E_OK == bc_uart_poll(), "diag repeated call");
    check(1 == g_print_count, "diagnostic not repeated");
    check(E_OK == bc_port_clear(), "reset after stop");
    check(sizeof(bc_port_result_t) == 8U, "compact result eight bytes");
}

static void test_input_rejections (void)
{
    bc_port_result_t result = {0U, 0U, 0U};
    uint8_t error;

    /* Impossible four-unit bar shorter than calibrated eight-unit floor. */
    check(E_OK == bc_port_clear(), "clear before invalid width");
    tick(0U, 0U);
    tick(120U, 0U);
    tick(0U, 1U);
    tick(4U, 1U);
    tick(0U, 0U);
    check(E_OK == bc_port_peek(&result), "bad width recorded");
    check(0U == (result.info & BC_PORT_VALID_FLAG),
          "invalid width cannot become a valid letter");
    error = (uint8_t)((result.info & BC_PORT_ERROR_MASK) >>
                      BC_PORT_ERROR_SHIFT);
    check(error == BC_SCAN_ERROR_WIDTH, "explicit width error");
    check(E_OK == bc_uart_poll(), "invalid diagnostic");
    check(0 == strcmp(g_last_message,
                      "[BARCODE] Rejected: code 2\n"),
          "clear invalid UART reason");
    check(E_OK == bc_port_ack(result.sequence), "ack rejection");

    check(E_OK == bc_port_clear(), "clear before over-speed sample");
    tick(0U, 0U);
    tick(120U, 0U);
    g_left_count += 20;
    g_right_count += 20;
    g_time += 10U;
    check(E_OK == bc_port_sample(1U, g_time), "speed check fed");
    check(E_OK == bc_port_process(), "speed check processed");
    check(E_OK == bc_port_peek(&result), "fast sample rejected");
    check(0U == (result.info & BC_PORT_VALID_FLAG),
          "too-fast polling never creates a letter");
    check(E_OK == bc_port_ack(result.sequence), "ack speed issue");

    check(E_OK == bc_port_clear(), "clear before stale encoder");
    tick(0U, 0U);
    gb_stale_edge = 1U;
    g_left_count += 2;
    g_right_count += 2;
    g_time += 10U;
    check(E_OK == bc_port_sample(0U, g_time), "stale sample fed");
    check(E_OK == bc_port_process(), "stale sample processed");
    check(E_OK == bc_port_peek(&result), "stale encoder rejected");
    check(0U == (result.info & BC_PORT_VALID_FLAG),
          "stale measurement invalid");
    gb_stale_edge = 0U;
}

static void test_four_millisecond_capture (void)
{
    bc_port_result_t result = {0U, 0U, 0U};
    uint32_t usec = g_time * 1000U;

    check(E_OK == bc_port_clear(), "clear for 4 ms capture");
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        usec += BC_FAST_TEST_INTERVAL_USEC;
        check(E_OK == bc_port_sample_counts(
            0U, usec, g_left_count, g_right_count,
            usec, usec, 1U), "4 ms encoder snapshot accepted");
        check(E_OK == bc_port_process(), "4 ms processing accepted");
    }
    check(E_OBJ == bc_port_peek(&result),
          "quiet 4 ms samples cannot fabricate a letter");
    usec += BC_FAST_TEST_INTERVAL_USEC;
    check(E_OK == bc_port_sample_counts(
        0U, usec, g_left_count, g_right_count,
        usec, usec, 0U), "missed IRQ invalid sample fed");
    check(E_OK == bc_port_process(), "missed IRQ rejected safely");
    check(E_OK == bc_port_peek(&result), "invalid record available");
    check(0U == (result.info & BC_PORT_VALID_FLAG),
          "invalid 4 ms record cannot become valid letter");
    check(E_OK == bc_port_ack(result.sequence), "invalid ACK");
}

static void test_microsecond_wrap (void)
{
    bc_port_result_t result = {0};
    uint32_t before = 0xFFFFFF00U;
    uint32_t after = before + 4000U;
    check(E_OK == bc_port_clear(), "clear before timer wrap");
    check(E_OK == bc_port_sample_counts(0U, before, 0, 0,
                                        before, before, 1U), "before wrap");
    check(E_OK == bc_port_process(), "process before wrap");
    check(E_OK == bc_port_sample_counts(0U, after, 0, 0,
                                        after, after, 1U), "after wrap");
    check(E_OK == bc_port_process(), "process after wrap");
    check(E_OBJ == bc_port_peek(&result),
          "ordinary 4 ms gap across wrap remains valid");
}

int test_port_run (void)
{
    test_full_scan();
    test_input_rejections();
    test_four_millisecond_capture();
    test_microsecond_wrap();
    printf("Barcode adapter: %d assertions, %d failed\n",
           g_checks, g_fail);
    return g_fail;
}

int main (void)
{
    return (test_port_run() == 0) ? 0 : 1;
}
#endif

/*** end of file ***/

