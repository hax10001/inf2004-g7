/* Host-only integration tests. The normal robot build does not enter host/.
 * Keep a guard because the older repository host runner discovers .c files
 * recursively. This test intentionally supplies no motor or GPIO driver.
 */
#ifdef BC_HOST_SERVICE_TEST
#include "modules/line_barcode/line_barcode.h"
#include "modules/line_barcode/bc_config.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/line_barcode/line_follow.h"
#include "modules/line_barcode/task_ir.h"
#include "drivers/encoder.h"
#include "app/tasks.h"
#include "app/app_config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define TEST_ERROR (-57)
#define TEST_LIMIT (-34)
#define TEST_STEP 4U
#define TEST_NARROW 40U
#define CHECK(expression) do { \
    g_checks++; \
    if (!(expression)) { \
        fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #expression); \
        g_failures++; \
    } \
} while (0)

static unsigned int g_checks;
static unsigned int g_failures;
static unsigned int g_mutex_count = 1U; /* Existing motion mutex. */
static unsigned int g_create_calls;
static unsigned int g_start_calls;
static unsigned int g_print_calls;
static bool g_locked;
static ER g_create_error;
static ER g_lock_error;
static ER g_unlock_error;
static ER g_time_error;
static ER g_start_error;
static ID g_task_result = 7;
static uint32_t g_now_usec = 100000U;
static int32_t g_left;
static int32_t g_right;
static uint32_t g_travel_step = TEST_STEP;
static char g_log[32768];
static size_t g_log_used;

ID tk_cre_mtx (T_CMTX const * config)
{
    CHECK(NULL != config);
    CHECK(TA_INHERIT == config->mtxatr);
    if (g_create_error < E_OK)
    {
        return g_create_error;
    }
    if (g_mutex_count >= 4U)
    {
        return TEST_LIMIT;
    }
    g_mutex_count++;
    return (ID)g_mutex_count;
}

ER tk_loc_mtx (ID mutex, TMO timeout)
{
    CHECK(2 == mutex);
    CHECK((0 == timeout) || (TMO_FEVR == timeout));
    if (g_lock_error < E_OK)
    {
        return g_lock_error;
    }
    CHECK(!g_locked);
    if (g_locked)
    {
        return E_OBJ;
    }
    g_locked = true;
    return E_OK;
}

ER tk_unl_mtx (ID mutex)
{
    CHECK(2 == mutex);
    CHECK(g_locked);
    g_locked = false;
    return g_unlock_error;
}

ER tk_get_otm (SYSTIM * time)
{
    if (g_time_error < E_OK)
    {
        return g_time_error;
    }
    time->hi = 0;
    time->lo = g_now_usec / 1000U;
    return E_OK;
}

ID tasks_create (tasks_entry_t entry, PRI priority)
{
    CHECK(task_ir_task == entry);
    CHECK(APP_IR_PRIORITY == priority);
    CHECK(!g_locked);
    g_create_calls++;
    return g_task_result;
}

ER tk_sta_tsk (ID task, INT code)
{
    CHECK(7 == task);
    CHECK(0 == code);
    g_start_calls++;
    return g_start_error;
}

void task_ir_task (INT code, void * context)
{
    (void)code;
    (void)context;
    CHECK(false); /* Creating a task must not run it synchronously. */
}

void encoder_read (encoder_wheel_t wheel, encoder_sample_t * sample)
{
    CHECK(!g_locked); /* Hardware reads stay outside the snapshot mutex. */
    CHECK((ENCODER_LEFT == wheel) || (ENCODER_RIGHT == wheel));
    sample->count = (ENCODER_LEFT == wheel) ? g_left : g_right;
    sample->edge_usec = g_now_usec;
}

uint32_t encoder_now_usec (void)
{
    CHECK(!g_locked);
    return g_now_usec;
}

INT tm_printf (UB const * format, ...)
{
    CHECK(!g_locked);
    g_print_calls++;
    va_list args;
    va_start(args, format);
    int const count = vsnprintf(g_log + g_log_used,
        sizeof(g_log) - g_log_used, (char const *)format, args);
    va_end(args);
    if ((count > 0) && ((size_t)count < sizeof(g_log) - g_log_used))
    {
        g_log_used += (size_t)count;
    }
    return count;
}

static void sample (uint32_t raw, uint32_t step)
{
    CHECK(0U == (step % 2U));
    g_left += (int32_t)(step / 2U);
    g_right -= (int32_t)(step / 2U);
    g_now_usec += 10000U;
    line_barcode_sample(raw, g_now_usec);
}

static void travel (uint32_t raw, uint32_t units)
{
    CHECK(0U == (units % g_travel_step));
    for (uint32_t distance = 0U; distance < units; distance += g_travel_step)
    {
        sample(raw, g_travel_step);
    }
}

static void marking_a (void)
{
    /* Independent Code 39 strings, also matching the PDF's vector bars.
     * Letter A has wide elements 1, 6 and 9; stars have 2, 5 and 7.
     */
    char const * const symbols[3] = {
        "nwnnwnwnn", "wnnnnwnnw", "nwnnwnwnn"};

    for (uint32_t symbol = 0U; symbol < 3U; symbol++)
    {
        for (uint32_t element = 0U; element < 9U; element++)
        {
            uint32_t const units =
                ('w' == symbols[symbol][element]) ?
                3U * TEST_NARROW : TEST_NARROW;
            uint32_t const raw = (0U == (element % 2U)) ?
                BC_BLACK_LEVEL : 1U - BC_BLACK_LEVEL;
            travel(raw, units);
        }
        if (symbol < 2U)
        {
            travel(1U - BC_BLACK_LEVEL, TEST_NARROW);
        }
    }
}

static void scan_a (void)
{
    travel(1U - BC_BLACK_LEVEL, 12U * TEST_NARROW);
    marking_a();
    travel(1U - BC_BLACK_LEVEL, 12U * TEST_NARROW);
}

static void check_no_result (void)
{
    barcode_result_t result = {.letter = '?', .sequence = 12345U};
    CHECK(E_OBJ == line_barcode_get_result(&result));
    CHECK('?' == result.letter);
    CHECK(12345U == result.sequence);
}

static barcode_result_t check_result (uint32_t sequence)
{
    barcode_result_t result = {0};
    CHECK(E_OK == line_barcode_get_result(&result));
    CHECK('A' == result.letter);
    CHECK(BC_OK == result.status);
    CHECK(sequence == result.sequence);
    CHECK(TEST_NARROW == result.narrow_units);
    CHECK(47U * TEST_NARROW == result.width_units);
    CHECK(result.timestamp_msec > 0U);
    CHECK(result.timestamp_msec <= g_now_usec / 1000U);
    CHECK((BC_UNITS_PER_MM > 0.0f) == result.b_distance_valid);
    if (BC_UNITS_PER_MM > 0.0f)
    {
        float const expected = (float)result.width_units / BC_UNITS_PER_MM;
        CHECK((result.width_mm >= expected - 0.001f) &&
              (result.width_mm <= expected + 0.001f));
    }
    else
    {
        CHECK(0.0f == result.width_mm);
    }
    return result;
}

static void test_startup (void)
{
    barcode_result_t result = {0};
    barcode_diagnostic_t diagnostic = {0};
    barcode_failure_t failure = {0};
    CHECK(E_PAR == line_barcode_get_result(NULL));
    CHECK(E_PAR == line_barcode_get_diagnostic(NULL));
    CHECK(E_PAR == line_barcode_get_failure(NULL));
    CHECK(E_OBJ == line_barcode_get_result(&result));
    CHECK(E_OBJ == line_barcode_get_diagnostic(&diagnostic));
    CHECK(E_OBJ == line_barcode_get_failure(&failure));
    CHECK(E_OBJ == line_barcode_ack_failure(1U));
    CHECK(E_OBJ == line_barcode_ack(1U));
    CHECK(E_OBJ == line_barcode_set_enabled(false));

    g_create_error = TEST_ERROR;
    CHECK(TEST_ERROR == line_barcode_init());
    CHECK(1U == g_mutex_count);
    g_create_error = E_OK;
    CHECK(E_OK == line_barcode_init());
    CHECK(E_OBJ == line_barcode_get_failure(&failure));
    CHECK(E_OBJ == line_barcode_ack_failure(1U));
    CHECK(2U == g_mutex_count);
    CHECK(0U == g_create_calls);

    /* Simulate the terrain subsystem allocating its two existing mutexes.
     * A hidden fifth barcode mutex would prevent this startup sequence.
     */
    T_CMTX const config = {.mtxatr = TA_INHERIT};
    CHECK(3 == tk_cre_mtx(&config));
    CHECK(4 == tk_cre_mtx(&config));
    CHECK(TEST_LIMIT == tk_cre_mtx(&config));

    g_task_result = 0;
    CHECK(E_OBJ == line_barcode_start());
    CHECK(0U == g_start_calls);
    g_task_result = TEST_ERROR;
    CHECK(TEST_ERROR == line_barcode_start());
    CHECK(0U == g_start_calls);
    g_task_result = 7;
    g_start_error = TEST_ERROR;
    CHECK(TEST_ERROR == line_barcode_start());
    g_start_error = E_OK;
    CHECK(E_OK == line_barcode_start());
    CHECK(4U == g_create_calls);
    CHECK(2U == g_start_calls);
}

static void test_ir_preserved (void)
{
    CHECK(LINE_FORWARD == line_follow_decide(0U, 0U));
    CHECK(LINE_CORRECT_LEFT == line_follow_decide(1U, 0U));
    CHECK(LINE_CORRECT_RIGHT == line_follow_decide(0U, 1U));
    CHECK(LINE_UNCERTAIN == line_follow_decide(1U, 1U));
    CHECK(E_OK == ir_shared_publish(1U, 0U, 1U));
    CHECK(E_OK == ir_shared_publish(0U, 1U, 0U));
    CHECK(E_OK == ir_shared_publish(1U, 1U, 1U));
    CHECK(E_OK == ir_shared_publish(0U, 0U, 0U));
    sample(1U - BC_BLACK_LEVEL, 0U);

    bool junction = false;
    bool left = false;
    bool right = false;
    CHECK(E_OK == ir_shared_take_junction(&junction));
    CHECK(junction);
    CHECK(E_OK == ir_shared_take_junction(&junction));
    CHECK(!junction);
    CHECK(E_OK == ir_shared_take_sides(&left, &right));
    CHECK(left && right);
    CHECK(E_OK == ir_shared_take_sides(&left, &right));
    CHECK(!left && !right);

    ir_shared_snapshot_t snapshot = {0};
    CHECK(E_OK == ir_shared_get_latest(&snapshot));
    CHECK(snapshot.b_valid);
    CHECK((0U == snapshot.left) && (0U == snapshot.right));
    CHECK(0U == snapshot.barcode);
}

static void test_error_returns (void)
{
    barcode_result_t result = {0};
    barcode_diagnostic_t diagnostic = {0};
    barcode_failure_t failure = {0};
    g_lock_error = TEST_ERROR;
    CHECK(TEST_ERROR == line_barcode_get_result(&result));
    CHECK(TEST_ERROR == line_barcode_get_diagnostic(&diagnostic));
    CHECK(TEST_ERROR == line_barcode_get_failure(&failure));
    CHECK(TEST_ERROR == line_barcode_ack_failure(1U));
    CHECK(TEST_ERROR == line_barcode_ack(1U));
    CHECK(TEST_ERROR == line_barcode_set_enabled(false));
    sample(1U - BC_BLACK_LEVEL, 0U);
    CHECK(!g_locked);
    g_lock_error = E_OK;
    g_unlock_error = TEST_ERROR;
    CHECK(TEST_ERROR == line_barcode_get_result(&result));
    CHECK(TEST_ERROR == line_barcode_get_diagnostic(&diagnostic));
    CHECK(TEST_ERROR == line_barcode_get_failure(&failure));
    CHECK(TEST_ERROR == line_barcode_ack_failure(1U));
    CHECK(!g_locked);
    g_unlock_error = E_OK;
}

static void test_measurement_mode (void)
{
    barcode_diagnostic_t diagnostic = {0};
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    CHECK(BARCODE_MEASURING == diagnostic.mode);
    CHECK(E_PAR == line_barcode_set_enabled(true));
    scan_a();
    check_no_result();
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    CHECK(g_left == diagnostic.left_count);
    CHECK(g_right == diagnostic.right_count);
    CHECK(4U == diagnostic.max_step_units);
    CHECK(0U == diagnostic.max_skew_units);
    CHECK(10000U <= diagnostic.max_gap_usec);
    CHECK(0U == diagnostic.completed);
    CHECK(E_OK == line_barcode_set_enabled(false));
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    CHECK(BARCODE_DISABLED == diagnostic.mode);
    CHECK(E_PAR == line_barcode_set_enabled(true));
}

static void test_scanning_mode (void)
{
    barcode_diagnostic_t diagnostic = {0};
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    CHECK(BARCODE_ENABLED == diagnostic.mode);
    CHECK(E_OK == line_barcode_set_enabled(true));
    check_no_result();
    unsigned int const prints_before = g_print_calls;
    scan_a();
    CHECK(prints_before == g_print_calls);
    barcode_result_t const first = check_result(1U);
    barcode_result_t const repeated = check_result(1U);
    CHECK(first.timestamp_msec == repeated.timestamp_msec);

    CHECK(E_OBJ == line_barcode_ack(first.sequence + 1U));
    (void)check_result(1U);
    CHECK(E_OK == line_barcode_set_enabled(false));
    (void)check_result(1U);
    CHECK(E_OK == line_barcode_set_enabled(true));
    (void)check_result(1U);
    for (unsigned int index = 0U; index < 60U; index++)
    {
        sample(1U - BC_BLACK_LEVEL, 0U);
    }
    (void)check_result(1U);
    line_barcode_log();
    (void)check_result(1U); /* Diagnostics never consume a mission result. */

    CHECK(E_OK == line_barcode_ack(first.sequence));
    CHECK(E_OBJ == line_barcode_ack(first.sequence));
    check_no_result();
    for (unsigned int index = 0U; index < 80U; index++)
    {
        sample((index < 40U) ? 1U - BC_BLACK_LEVEL : BC_BLACK_LEVEL, 0U);
    }
    check_no_result();
    scan_a();
    barcode_result_t const second = check_result(2U);
    CHECK(E_OK == line_barcode_ack(second.sequence));

    /* An enable transition discards a partial barcode. */
    travel(1U - BC_BLACK_LEVEL, 12U * TEST_NARROW);
    travel(BC_BLACK_LEVEL, TEST_NARROW);
    travel(1U - BC_BLACK_LEVEL, 3U * TEST_NARROW);
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    CHECK(BC_SCANNING == diagnostic.state);
    unsigned int const active_prints = g_print_calls;
    line_barcode_log();
    CHECK(active_prints == g_print_calls);
    CHECK(E_OK == line_barcode_set_enabled(false));
    marking_a();
    check_no_result();
    CHECK(E_OK == line_barcode_set_enabled(true));
    travel(1U - BC_BLACK_LEVEL, 2U * TEST_NARROW);
    marking_a();
    travel(1U - BC_BLACK_LEVEL, 12U * TEST_NARROW);
    check_no_result(); /* Fresh leading quiet zone was too small. */
    scan_a();
    barcode_result_t const third = check_result(3U);
    CHECK(E_OK == line_barcode_ack(third.sequence));

    /* Contention drops barcode samples, then rejects the enlarged gap;
     * it never leaves the shared lock held or publishes a guessed result.
     */
    travel(1U - BC_BLACK_LEVEL, 12U * TEST_NARROW);
    travel(BC_BLACK_LEVEL, TEST_NARROW);
    g_lock_error = TEST_ERROR;
    travel(1U - BC_BLACK_LEVEL, 6U * TEST_STEP);
    g_lock_error = E_OK;
    sample(1U - BC_BLACK_LEVEL, TEST_STEP);
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    CHECK(BC_ERR_SAMPLE == diagnostic.status);
    CHECK(diagnostic.rejected > 0U);
    check_no_result();

    /* Delayed IR/count acquisition includes preemption before the hook. */
    travel(1U - BC_BLACK_LEVEL, 12U * TEST_NARROW);
    travel(BC_BLACK_LEVEL, TEST_NARROW);
    g_now_usec += 10000U;
    line_barcode_sample(1U - BC_BLACK_LEVEL,
                        g_now_usec - BC_MAX_READ_USEC - 1U);
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    CHECK(BC_ERR_SAMPLE == diagnostic.status);
    check_no_result();

    /* Failed timestamp publication loses only this frame. */
    g_time_error = TEST_ERROR;
    scan_a();
    check_no_result();
    g_time_error = E_OK;
    scan_a();
    (void)check_result(4U);
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    CHECK(4U == diagnostic.completed);
    CHECK(4U == g_mutex_count);
}

#if BC_DIAGNOSTICS
static void test_first_failure_log (void)
{
    CHECK(E_OK == line_barcode_ack(4U));
    barcode_failure_t previous = {0};
    if ((E_OK == line_barcode_get_failure(&previous)) && previous.b_pending)
    {
        CHECK(E_OK == line_barcode_ack_failure(previous.sequence));
    }
    g_log[0] = '\0';
    g_log_used = 0;
    g_travel_step = 10U;
    scan_a(); /* All narrow runs are 40, but the required minimum is 42. */
    barcode_diagnostic_t diagnostic = {0};
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    CHECK(diagnostic.status == BC_ERR_RESOLUTION);
    CHECK(diagnostic.run_count == BC_RUN_COUNT);
    uint32_t const rejected_before_reverse = diagnostic.rejected;
    barcode_failure_t original = {0};
    CHECK(E_OK == line_barcode_get_failure(&original));
    CHECK(original.status == BC_ERR_RESOLUTION);
    CHECK(original.decode.check == BC_CHECK_MIN_WIDTH);
    CHECK(original.run_count == BC_RUN_COUNT);
    CHECK(original.decode.max_step_units == 10U);
    CHECK(original.decode.min_width == 40U);
    CHECK(original.decode.narrow_max == 40U);
    CHECK(original.decode.wide_min == 120U);
    CHECK(original.decode.required_min == 42U);
    CHECK(original.decode.required_separation == 22U);
    CHECK(original.b_started_black && original.b_from_white);
    CHECK(original.barcode_max_step == 10U);
    CHECK(original.approach_max_step == 10U);
    CHECK(original.b_pending);
    for (unsigned index = 0; index < BC_RUN_COUNT; ++index)
    {
        char const * const pattern = "nwnnwnwnnnwnnnnwnnwnnwnnwnwnn";
        uint32_t const expected =
            ('w' == pattern[index]) ? 3U * TEST_NARROW : TEST_NARROW;
        CHECK(original.runs[index] == expected);
    }

    /* Incidental reverse movement must not replace the scan's failure. */
    g_now_usec += 10000U;
    g_left--;
    g_right++;
    line_barcode_sample(1U - BC_BLACK_LEVEL, g_now_usec);
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    CHECK(diagnostic.status == BC_ERR_REVERSE);
    CHECK(diagnostic.rejected == rejected_before_reverse);
    CHECK(diagnostic.failure_status == BC_ERR_RESOLUTION);
    barcode_failure_t preserved = {0};
    CHECK(E_OK == line_barcode_get_failure(&preserved));
    CHECK(preserved.sequence == original.sequence);
    CHECK(preserved.status == original.status);
    CHECK(preserved.decode.check == original.decode.check);
    CHECK(preserved.b_pending);
    CHECK(0 == memcmp(original.runs, preserved.runs, sizeof(original.runs)));
    CHECK(E_OBJ == line_barcode_ack_failure(original.sequence + 1U));
    line_barcode_log();
    CHECK(strstr(g_log, "[BC FAIL] reason=MIN_WIDTH") != NULL);
    CHECK(strstr(g_log, "max_step=10") != NULL);
    CHECK(strstr(g_log, "min_width=40") != NULL);
    CHECK(strstr(g_log, "required_min=42") != NULL);
    CHECK(strstr(g_log, "[BC RUNS]") != NULL);
    CHECK(strstr(g_log, "0: 40") != NULL);
    CHECK(strstr(g_log, "28: 40") != NULL);
    CHECK(E_OK == line_barcode_get_failure(&preserved));
    CHECK(!preserved.b_pending);
    CHECK(preserved.status == BC_ERR_RESOLUTION);
    CHECK(0 == memcmp(original.runs, preserved.runs, sizeof(original.runs)));
    g_log[0] = '\0';
    g_log_used = 0;
    for (unsigned i = 0; i < 120; ++i)
    {
        sample(1U - BC_BLACK_LEVEL, 0U);
        line_barcode_log();
    }
    CHECK(strstr(g_log, "[BC FAIL]") == NULL);

    /* New live run data must not mutate the retained report. */
    g_travel_step = TEST_STEP;
    travel(1U - BC_BLACK_LEVEL, 12U * TEST_NARROW);
    travel(BC_BLACK_LEVEL, 2U * TEST_NARROW);
    travel(1U - BC_BLACK_LEVEL, TEST_NARROW);
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    CHECK(diagnostic.runs[0] == 2U * TEST_NARROW);
    CHECK(E_OK == line_barcode_get_failure(&preserved));
    CHECK(preserved.runs[0] == TEST_NARROW);
    g_log[0] = '\0';
    g_log_used = 0;
    line_barcode_log();
    CHECK(g_log_used == 0); /* No formatting/printing during active capture. */
    CHECK(E_OK == line_barcode_set_enabled(false));
    CHECK(E_OK == line_barcode_set_enabled(true));

    /* A later rejected attempt gets a distinct once-only report. */
    g_travel_step = 10U;
    scan_a();
    CHECK(E_OK == line_barcode_get_failure(&preserved));
    CHECK(preserved.sequence > original.sequence);
    CHECK(preserved.b_pending);
    /* If logging is not serviced, retain the first unread failure. */
    uint32_t const unread_sequence = preserved.sequence;
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    uint32_t const dropped_before = diagnostic.failure_reports_dropped;
    scan_a();
    CHECK(E_OK == line_barcode_get_failure(&preserved));
    CHECK(preserved.sequence == unread_sequence);
    CHECK(preserved.b_pending);
    CHECK(E_OK == line_barcode_get_diagnostic(&diagnostic));
    CHECK(diagnostic.rejected == unread_sequence + 1U);
    CHECK(diagnostic.failure_reports_dropped == dropped_before + 1U);
    line_barcode_log();
    CHECK(strstr(g_log, "[BC FAIL] reason=MIN_WIDTH") != NULL);
    char const * const first = strstr(g_log, "[BC FAIL]");
    if (NULL != first)
    {
        CHECK(strstr(first + 1, "[BC FAIL]") == NULL);
    }
    g_travel_step = TEST_STEP;
}
#endif

int main (void)
{
    test_startup();
    test_ir_preserved();
    test_error_returns();
    if ((0 != BC_CAL_READY) &&
        ((1 == BC_LEFT_FORWARD_SIGN) || (-1 == BC_LEFT_FORWARD_SIGN)) &&
        ((1 == BC_RIGHT_FORWARD_SIGN) || (-1 == BC_RIGHT_FORWARD_SIGN)))
    {
        test_scanning_mode();
#if BC_DIAGNOSTICS
        test_first_failure_log();
#endif
    }
    else
    {
        test_measurement_mode();
    }
    CHECK(!g_locked);
    printf("Barcode service: %u checks, %u failures.\n", g_checks, g_failures);
    return (0U == g_failures) ? 0 : 1;
}
#endif /* BC_HOST_SERVICE_TEST */
