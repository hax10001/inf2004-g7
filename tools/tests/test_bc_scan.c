/** @file test_bc_scan.c
 * @brief Host tests of the real run-length capture bc_scan.c feeding the
 *        real bc_code.c (FR5: a clean pass yields one letter in either
 *        direction; interrupted or malformed passes yield no letter).
 */
#include "bc_scan.h"
#include "bc_code.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define T_MIN_RUN 3U
#define T_LEADING 24U
#define T_TRAILING 24U
#define T_MIN_CODE 87U
#define T_MAX_CODE 400U
#define T_TIMEOUT 2500U
#define T_NARROW 4U
#define T_WIDE 12U
#define T_SYMBOL 9U
#define T_MSEC_PER_UNIT 2U

/* Code 39 reference, '1' = wide; letters used by these tests. */
static char const g_ref_star[] = "010010100";
static char const g_ref_b[] = "001001001";
static char const g_ref_r[] = "100000110";

typedef struct
{
    bc_scan_t scan;
    uint32_t units;
    uint32_t msec;
    uint8_t last_event;
    uint8_t ready_count;
    uint8_t reject_count;
} t_feed_t;

/** @brief A valid configuration used by most tests. */
static bc_scan_config_t make_config (void)
{
    bc_scan_config_t cfg;

    cfg.min_run_units = T_MIN_RUN;
    cfg.leading_units = T_LEADING;
    cfg.trailing_units = T_TRAILING;
    cfg.min_barcode_units = T_MIN_CODE;
    cfg.max_barcode_units = T_MAX_CODE;
    cfg.timeout_msec = T_TIMEOUT;
    return cfg;
}

/** @brief Build *letter* widths from a reference string. */
static void build_widths (uint32_t * p_width, char const * p_letter)
{
    char const * const symbols[BC_CODE_SYMBOLS] =
    {
        g_ref_star, p_letter, g_ref_star
    };
    uint32_t out = 0U;

    for (uint32_t s = 0U; s < BC_CODE_SYMBOLS; ++s)
    {
        if (s > 0U)
        {
            p_width[out] = T_NARROW;
            ++out;
        }
        for (uint32_t i = 0U; i < T_SYMBOL; ++i)
        {
            p_width[out] = ('1' == symbols[s][i]) ? T_WIDE : T_NARROW;
            ++out;
        }
    }
}

/** @brief Start a feeder at a given distance and time. */
static void feed_start (t_feed_t * p_feed, uint32_t units, uint32_t msec)
{
    bc_scan_config_t const cfg = make_config();

    memset(p_feed, 0, sizeof(*p_feed));
    assert(1U == bc_scan_init(&p_feed->scan, &cfg));
    p_feed->units = units;
    p_feed->msec = msec;
}

/** @brief Feed one colour for a number of unit steps. */
static void feed_run (t_feed_t * p_feed, uint8_t b_black, uint32_t length)
{
    for (uint32_t i = 0U; i < length; ++i)
    {
        uint8_t const event = bc_scan_feed(&p_feed->scan, b_black,
                                           p_feed->units, p_feed->msec, 1U);
        if (BC_SCAN_EVENT_READY == event)
        {
            ++p_feed->ready_count;
        }
        if (BC_SCAN_EVENT_REJECTED == event)
        {
            ++p_feed->reject_count;
        }
        p_feed->last_event = event;
        ++p_feed->units;
        p_feed->msec += T_MSEC_PER_UNIT;
    }
}

/** @brief Feed leading white, the runs, then trailing white. */
static void feed_code (t_feed_t * p_feed, uint32_t const * p_width,
                       uint32_t runs, uint32_t trailing)
{
    feed_run(p_feed, 0U, T_LEADING + 1U);
    for (uint32_t i = 0U; i < runs; ++i)
    {
        feed_run(p_feed, (uint8_t)((0U == (i % 2U)) ? 1U : 0U),
                 p_width[i]);
    }
    feed_run(p_feed, 0U, trailing);
}

/** @brief Take a result and check event and error code. */
static void expect_take (t_feed_t * p_feed, uint8_t event, uint8_t error,
                         uint32_t * p_width)
{
    uint8_t got_error = 0xFFU;
    uint32_t stamp = 0U;

    assert(event == bc_scan_take(&p_feed->scan, p_width, &got_error,
                                 &stamp));
    assert(error == got_error);
    assert(BC_SCAN_IDLE == p_feed->scan.state);
}

/** @brief Every contradictory configuration is refused. */
static void test_init_rejects_bad_config (void)
{
    bc_scan_t scan;
    bc_scan_config_t cfg = make_config();

    assert(1U == bc_scan_init(&scan, &cfg));
    assert(0U == bc_scan_init((bc_scan_t *)0, &cfg));
    assert(0U == bc_scan_init(&scan, (bc_scan_config_t const *)0));
    cfg.min_run_units = 0U;
    assert(0U == bc_scan_init(&scan, &cfg));
    cfg = make_config();
    cfg.leading_units = T_MIN_RUN;
    assert(0U == bc_scan_init(&scan, &cfg));
    cfg = make_config();
    cfg.trailing_units = T_MIN_RUN * 4U;
    assert(0U == bc_scan_init(&scan, &cfg));
    cfg.trailing_units = T_MIN_RUN * 4U + 1U;
    assert(1U == bc_scan_init(&scan, &cfg));
    cfg = make_config();
    cfg.min_barcode_units = T_MIN_RUN * BC_CODE_ELEMENTS - 1U;
    assert(0U == bc_scan_init(&scan, &cfg));
    cfg = make_config();
    cfg.max_barcode_units = T_MIN_CODE - 1U;
    assert(0U == bc_scan_init(&scan, &cfg));
    cfg = make_config();
    cfg.timeout_msec = 0U;
    assert(0U == bc_scan_init(&scan, &cfg));
    cfg = make_config();
    cfg.max_barcode_units = UINT32_MAX - T_TRAILING + 1U;
    assert(0U == bc_scan_init(&scan, &cfg));
    cfg = make_config();
    cfg.min_run_units = UINT32_MAX / 5U + 1U;
    cfg.leading_units = UINT32_MAX;
    cfg.trailing_units = UINT32_MAX;
    assert(0U == bc_scan_init(&scan, &cfg));
    puts("PASS bc_scan init: null, zero run, short leading/trailing, "
         "short/long code, zero timeout, overflow refused");
}

/** @brief A clean pass captures 29 widths that decode forwards. */
static void test_clean_capture_forward (void)
{
    t_feed_t feed;
    uint32_t width[BC_CODE_ELEMENTS];
    uint32_t got[BC_CODE_ELEMENTS] = {0U};
    uint8_t error = 0xFFU;
    uint32_t stamp = 0U;
    uint32_t ready_msec;
    bc_code_result_t r;

    build_widths(width, g_ref_b);
    feed_start(&feed, 1000U, 50000U);
    feed_code(&feed, width, BC_CODE_ELEMENTS, T_TRAILING + 1U);
    assert(1U == feed.ready_count);
    assert(BC_SCAN_EVENT_READY == feed.last_event);
    ready_msec = feed.msec - T_MSEC_PER_UNIT;
    /* Held until taken: further samples keep reporting READY. */
    assert(BC_SCAN_EVENT_READY ==
           bc_scan_feed(&feed.scan, 1U, feed.units, feed.msec, 1U));
    assert(BC_SCAN_EVENT_READY ==
           bc_scan_take(&feed.scan, got, &error, &stamp));
    assert(BC_SCAN_ERROR_NONE == error);
    assert(ready_msec == stamp);
    assert(0 == memcmp(width, got, sizeof(width)));
    r = bc_code_decode(got, T_MIN_RUN);
    assert((1U == r.valid) && (1U == r.letter) && (0U == r.reverse));
    /* Take rearms: nothing more to take. */
    assert(BC_SCAN_EVENT_NONE ==
           bc_scan_take(&feed.scan, got, &error, &stamp));
    puts("PASS bc_scan FR5: clean forward pass -> READY once, 29 exact "
         "widths, timestamp, decodes 'B'");
}

/** @brief A reversed pass captures widths that decode backwards. */
static void test_clean_capture_reverse (void)
{
    t_feed_t feed;
    uint32_t width[BC_CODE_ELEMENTS];
    uint32_t back[BC_CODE_ELEMENTS];
    uint32_t got[BC_CODE_ELEMENTS] = {0U};
    bc_code_result_t r;

    build_widths(width, g_ref_r);
    for (uint32_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        back[i] = width[BC_CODE_ELEMENTS - 1U - i];
    }
    feed_start(&feed, 0U, 0U);
    feed_code(&feed, back, BC_CODE_ELEMENTS, T_TRAILING + 1U);
    expect_take(&feed, BC_SCAN_EVENT_READY, BC_SCAN_ERROR_NONE, got);
    r = bc_code_decode(got, T_MIN_RUN);
    assert((1U == r.valid) && (17U == r.letter) && (1U == r.reverse));
    puts("PASS bc_scan FR5: reversed pass decodes 'R' with reverse flag");
}

/** @brief Two passes in a row each need fresh leading white. */
static void test_rearm_after_take (void)
{
    t_feed_t feed;
    uint32_t width[BC_CODE_ELEMENTS];
    uint32_t got[BC_CODE_ELEMENTS] = {0U};

    build_widths(width, g_ref_b);
    feed_start(&feed, 0U, 0U);
    feed_code(&feed, width, BC_CODE_ELEMENTS, T_TRAILING + 1U);
    expect_take(&feed, BC_SCAN_EVENT_READY, BC_SCAN_ERROR_NONE, got);
    /* Black immediately after take: still idle, no capture starts. */
    feed_run(&feed, 1U, 10U);
    assert(BC_SCAN_IDLE == feed.scan.state);
    feed.ready_count = 0U;
    feed_code(&feed, width, BC_CODE_ELEMENTS, T_TRAILING + 1U);
    assert(1U == feed.ready_count);
    expect_take(&feed, BC_SCAN_EVENT_READY, BC_SCAN_ERROR_NONE, got);
    puts("PASS bc_scan rearm: take returns to idle; second pass captured");
}

/** @brief Too little leading white never starts a capture. */
static void test_short_leading_ignored (void)
{
    t_feed_t feed;

    feed_start(&feed, 0U, 0U);
    feed_run(&feed, 0U, T_LEADING - 1U);
    feed_run(&feed, 1U, T_NARROW);
    assert(BC_SCAN_IDLE == feed.scan.state);
    assert(0U == feed.reject_count);
    feed_run(&feed, 0U, T_LEADING + 1U);
    assert(BC_SCAN_START_CANDIDATE == feed.scan.state);
    feed_run(&feed, 1U, 1U);
    assert(BC_SCAN_CAPTURING == feed.scan.state);
    puts("PASS bc_scan start: black after < leading white stays idle; "
         "after >= leading starts capture");
}

/** @brief Invalid samples during a pass reject it with INPUT. */
static void test_invalid_sample_rejects (void)
{
    t_feed_t feed;
    uint32_t got[BC_CODE_ELEMENTS] = {0U};
    uint32_t keep[BC_CODE_ELEMENTS] = {0U};
    uint8_t error = 0U;
    uint32_t stamp = 0U;

    feed_start(&feed, 0U, 0U);
    feed_run(&feed, 0U, T_LEADING + 1U);
    feed_run(&feed, 1U, T_NARROW);
    assert(BC_SCAN_EVENT_REJECTED ==
           bc_scan_feed(&feed.scan, 0U, feed.units, 777U, 0U));
    /* Held: a good sample cannot revive the rejected pass. */
    assert(BC_SCAN_EVENT_REJECTED ==
           bc_scan_feed(&feed.scan, 0U, feed.units, 778U, 1U));
    memset(got, 0xA5, sizeof(got));
    memcpy(keep, got, sizeof(got));
    assert(BC_SCAN_EVENT_REJECTED ==
           bc_scan_take(&feed.scan, got, &error, &stamp));
    assert((BC_SCAN_ERROR_INPUT == error) && (777U == stamp));
    assert(0 == memcmp(got, keep, sizeof(got)));

    feed_start(&feed, 0U, 0U);
    assert(BC_SCAN_EVENT_REJECTED ==
           bc_scan_feed(&feed.scan, 2U, 0U, 0U, 1U));
    expect_take(&feed, BC_SCAN_EVENT_REJECTED, BC_SCAN_ERROR_INPUT, got);
    assert(BC_SCAN_EVENT_REJECTED ==
           bc_scan_feed(&feed.scan, 0U, 0U, 0U, 2U));
    expect_take(&feed, BC_SCAN_EVENT_REJECTED, BC_SCAN_ERROR_INPUT, got);
    assert(BC_SCAN_EVENT_REJECTED ==
           bc_scan_feed((bc_scan_t *)0, 0U, 0U, 0U, 1U));
    puts("PASS bc_scan input: invalid sample/colour rejects with INPUT, "
         "held until take, widths untouched");
}

/** @brief A run narrower than the minimum rejects with WIDTH. */
static void test_glitch_rejects_width (void)
{
    t_feed_t feed;
    uint32_t width[BC_CODE_ELEMENTS];
    uint32_t got[BC_CODE_ELEMENTS] = {0U};

    build_widths(width, g_ref_b);
    width[7] = T_MIN_RUN - 1U;
    feed_start(&feed, 0U, 0U);
    feed_code(&feed, width, BC_CODE_ELEMENTS, T_TRAILING + 1U);
    assert(0U == feed.ready_count);
    expect_take(&feed, BC_SCAN_EVENT_REJECTED, BC_SCAN_ERROR_WIDTH, got);
    puts("PASS bc_scan width: run below min_run rejects with WIDTH, no "
         "READY");
}

/** @brief Travel going backwards rejects with DISTANCE. */
static void test_backward_travel_rejects (void)
{
    t_feed_t feed;
    uint32_t got[BC_CODE_ELEMENTS] = {0U};

    feed_start(&feed, 500U, 0U);
    feed_run(&feed, 0U, T_LEADING + 1U);
    feed_run(&feed, 1U, T_NARROW);
    assert(BC_SCAN_EVENT_REJECTED ==
           bc_scan_feed(&feed.scan, 0U, feed.units - T_NARROW - 1U,
                        feed.msec, 1U));
    expect_take(&feed, BC_SCAN_EVENT_REJECTED, BC_SCAN_ERROR_DISTANCE,
                got);
    puts("PASS bc_scan distance: travel moving backwards rejects with "
         "DISTANCE");
}

/** @brief A pass that takes too long rejects with TIMEOUT. */
static void test_timeout_rejects (void)
{
    t_feed_t feed;
    uint32_t got[BC_CODE_ELEMENTS] = {0U};

    feed_start(&feed, 0U, 0U);
    feed_run(&feed, 0U, T_LEADING + 1U);
    feed_run(&feed, 1U, T_NARROW);
    feed.msec += T_TIMEOUT;
    feed_run(&feed, 1U, 1U);
    assert(BC_SCAN_EVENT_REJECTED == feed.last_event);
    expect_take(&feed, BC_SCAN_EVENT_REJECTED, BC_SCAN_ERROR_TIMEOUT, got);
    puts("PASS bc_scan timeout: stalled pass rejects with TIMEOUT");
}

/** @brief The millisecond clock wrapping mid-pass is not a timeout. */
static void test_clock_wrap_no_timeout (void)
{
    t_feed_t feed;
    uint32_t width[BC_CODE_ELEMENTS];
    uint32_t got[BC_CODE_ELEMENTS] = {0U};

    build_widths(width, g_ref_b);
    feed_start(&feed, 0U, UINT32_MAX - 100U);
    feed_code(&feed, width, BC_CODE_ELEMENTS, T_TRAILING + 1U);
    assert(feed.msec < 1000U);
    expect_take(&feed, BC_SCAN_EVENT_READY, BC_SCAN_ERROR_NONE, got);
    assert(0 == memcmp(width, got, sizeof(width)));
    puts("PASS bc_scan clock wrap: uint32 msec rollover mid-pass still "
         "READY");
}

/** @brief A 30th element (black in the trailing zone) is OVERFLOW. */
static void test_extra_bar_overflow (void)
{
    t_feed_t feed;
    uint32_t width[BC_CODE_ELEMENTS];
    uint32_t got[BC_CODE_ELEMENTS] = {0U};

    build_widths(width, g_ref_b);
    feed_start(&feed, 0U, 0U);
    feed_code(&feed, width, BC_CODE_ELEMENTS, T_TRAILING - 2U);
    assert(BC_SCAN_END_CANDIDATE == feed.scan.state);
    feed_run(&feed, 1U, 1U);
    assert(BC_SCAN_EVENT_REJECTED == feed.last_event);
    expect_take(&feed, BC_SCAN_EVENT_REJECTED, BC_SCAN_ERROR_OVERFLOW,
                got);
    puts("PASS bc_scan overflow: bar inside trailing zone after 29 runs "
         "rejects with OVERFLOW");
}

/** @brief Codes too short or too long for the window are refused. */
static void test_length_window (void)
{
    t_feed_t feed;
    uint32_t width[BC_CODE_ELEMENTS];
    uint32_t got[BC_CODE_ELEMENTS] = {0U};

    /* All runs at min_run: 87 units = min, accepted. */
    for (uint32_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        width[i] = T_MIN_RUN;
    }
    feed_start(&feed, 0U, 0U);
    feed_code(&feed, width, BC_CODE_ELEMENTS, T_TRAILING + 1U);
    expect_take(&feed, BC_SCAN_EVENT_READY, BC_SCAN_ERROR_NONE, got);

    /* Same min-width code below a stricter minimum length. */
    feed_start(&feed, 0U, 0U);
    feed.scan.cfg.min_barcode_units = T_MIN_RUN * BC_CODE_ELEMENTS + 1U;
    feed_code(&feed, width, BC_CODE_ELEMENTS, T_TRAILING + 1U);
    expect_take(&feed, BC_SCAN_EVENT_REJECTED, BC_SCAN_ERROR_DISTANCE,
                got);

    /* Total 414 > max 400 (but < max + trailing) at the 29th run. */
    build_widths(width, g_ref_b);
    width[0] = T_MAX_CODE - 170U;
    feed_start(&feed, 0U, 0U);
    feed_code(&feed, width, BC_CODE_ELEMENTS, T_TRAILING + 1U);
    expect_take(&feed, BC_SCAN_EVENT_REJECTED, BC_SCAN_ERROR_DISTANCE,
                got);

    /* A bar that never ends: exceeds max + trailing while capturing. */
    feed_start(&feed, 0U, 0U);
    feed_run(&feed, 0U, T_LEADING + 1U);
    feed_run(&feed, 1U, T_MAX_CODE + T_TRAILING + 2U);
    assert(BC_SCAN_EVENT_REJECTED == feed.last_event);
    expect_take(&feed, BC_SCAN_EVENT_REJECTED, BC_SCAN_ERROR_DISTANCE,
                got);
    puts("PASS bc_scan length: min length accepted; under min, over max "
         "and endless bar rejected with DISTANCE");
}

/** @brief Interrupt cancels an active pass but not an idle scanner. */
static void test_interrupt (void)
{
    t_feed_t feed;
    uint32_t width[BC_CODE_ELEMENTS];
    uint32_t got[BC_CODE_ELEMENTS] = {0U};
    uint8_t error = 0U;
    uint32_t stamp = 0U;

    feed_start(&feed, 0U, 0U);
    bc_scan_interrupt(&feed.scan, 5U);
    assert(BC_SCAN_IDLE == feed.scan.state);
    feed_run(&feed, 0U, 3U);
    bc_scan_interrupt(&feed.scan, 6U);
    assert(BC_SCAN_START_CANDIDATE == feed.scan.state);
    bc_scan_interrupt((bc_scan_t *)0, 7U);

    feed_run(&feed, 0U, T_LEADING);
    feed_run(&feed, 1U, T_NARROW);
    bc_scan_interrupt(&feed.scan, 4321U);
    assert(BC_SCAN_EVENT_REJECTED ==
           bc_scan_take(&feed.scan, got, &error, &stamp));
    assert((BC_SCAN_ERROR_INTERRUPTED == error) && (4321U == stamp));

    /* Interrupt in the trailing zone loses an otherwise good read. */
    build_widths(width, g_ref_b);
    feed_start(&feed, 0U, 0U);
    feed_code(&feed, width, BC_CODE_ELEMENTS, 2U);
    assert(BC_SCAN_END_CANDIDATE == feed.scan.state);
    bc_scan_interrupt(&feed.scan, 9U);
    expect_take(&feed, BC_SCAN_EVENT_REJECTED, BC_SCAN_ERROR_INTERRUPTED,
                got);

    /* A completed read is not cancelled by a late interrupt. */
    feed_start(&feed, 0U, 0U);
    feed_code(&feed, width, BC_CODE_ELEMENTS, T_TRAILING + 1U);
    bc_scan_interrupt(&feed.scan, 10U);
    expect_take(&feed, BC_SCAN_EVENT_READY, BC_SCAN_ERROR_NONE, got);
    puts("PASS bc_scan interrupt: active/trailing pass -> INTERRUPTED; "
         "idle and completed passes unaffected");
}

/** @brief Take with any null output pointer does nothing. */
static void test_take_null_guards (void)
{
    t_feed_t feed;
    uint32_t width[BC_CODE_ELEMENTS];
    uint32_t got[BC_CODE_ELEMENTS] = {0U};
    uint8_t error = 0U;
    uint32_t stamp = 0U;

    build_widths(width, g_ref_b);
    feed_start(&feed, 0U, 0U);
    feed_code(&feed, width, BC_CODE_ELEMENTS, T_TRAILING + 1U);
    assert(BC_SCAN_EVENT_NONE ==
           bc_scan_take((bc_scan_t *)0, got, &error, &stamp));
    assert(BC_SCAN_EVENT_NONE ==
           bc_scan_take(&feed.scan, (uint32_t *)0, &error, &stamp));
    assert(BC_SCAN_EVENT_NONE ==
           bc_scan_take(&feed.scan, got, (uint8_t *)0, &stamp));
    assert(BC_SCAN_EVENT_NONE ==
           bc_scan_take(&feed.scan, got, &error, (uint32_t *)0));
    /* The ready result survived the bad calls. */
    expect_take(&feed, BC_SCAN_EVENT_READY, BC_SCAN_ERROR_NONE, got);
    puts("PASS bc_scan take: null arguments return NONE and keep the "
         "pending result");
}

/** @brief Run every bc_scan test. */
int main (void)
{
    test_init_rejects_bad_config();
    test_clean_capture_forward();
    test_clean_capture_reverse();
    test_rearm_after_take();
    test_short_leading_ignored();
    test_invalid_sample_rejects();
    test_glitch_rejects_width();
    test_backward_travel_rejects();
    test_timeout_rejects();
    test_clock_wrap_no_timeout();
    test_extra_bar_overflow();
    test_length_window();
    test_interrupt();
    test_take_null_guards();
    return 0;
}

/*** end of file ***/

