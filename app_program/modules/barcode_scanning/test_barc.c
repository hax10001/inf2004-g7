/** @file test_barc.c
 * @brief Deterministic Code 39 and distance-capture regression tests.
 */
#include "test_barc.h"
#include "bc_code.h"
#include "bc_scan.h"
#include <stdio.h>

#define TEST_NARROW 10U
#define TEST_WIDE 30U
#define TEST_FIRST_GAP 70U
#define TEST_END_GAP 80U
#define TEST_MAX_DISTANCE 900U

static int g_failures = 0;
static int g_assertions = 0;

static void test_check (uint8_t b_condition, char const * p_label)
{
    ++g_assertions;
    if (0U == b_condition)
    {
        ++g_failures;
        printf("FAIL: %s\n", p_label);
    }
}

static void test_symbol (uint16_t bits, uint32_t * p_width)
{
    for (uint8_t i = 0U; i < 9U; ++i)
    {
        p_width[i] = (0U != (bits & (1U << (8U - i)))) ?
                     TEST_WIDE : TEST_NARROW;
    }
}

static void test_make (uint8_t code, uint32_t * p_width)
{
    bc_code_pattern_t pattern = {0U};

    test_check(bc_code_get(code, &pattern), "letter lookup");
    test_symbol(0x94U, &p_width[0U]);
    p_width[9U] = TEST_NARROW;
    test_symbol(pattern.element_pattern, &p_width[10U]);
    p_width[19U] = TEST_NARROW;
    test_symbol(0x94U, &p_width[20U]);
}

static uint32_t test_feed (bc_scan_t * p_scan,
                            uint32_t const * p_width,
                            uint32_t step_ticks, uint8_t b_reverse)
{
    uint32_t ticks = 0U;
    uint32_t msec = 0U;
    uint8_t b_black = 0U;
    bc_scan_config_t const config = {
        4U, 50U, 50U, 300U, TEST_MAX_DISTANCE, 10000U
    };

    test_check(bc_scan_init(p_scan, &config), "scan init");
    (void)bc_scan_feed(p_scan, 0U, 0U, msec, 1U);

    for (uint32_t k = 0U; k < TEST_FIRST_GAP; k += step_ticks)
    {
        ticks += step_ticks;
        msec += 10U;
        (void)bc_scan_feed(p_scan, 0U, ticks, msec, 1U);
    }

    b_black = 1U;
    (void)bc_scan_feed(p_scan, b_black, ticks, msec, 1U);

    for (uint8_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        uint32_t width = b_reverse ?
            p_width[BC_CODE_ELEMENTS - 1U - i] : p_width[i];
        uint32_t delta = 0U;

        /* Simulated 10 ms polling with movement per sampling interval. */
        while (delta < width)
        {
            uint32_t part = (width - delta < step_ticks) ?
                            (width - delta) : step_ticks;
            delta += part;
            ticks += part;
            msec += 10U;
            (void)bc_scan_feed(p_scan, b_black, ticks, msec, 1U);
        }

        b_black ^= 1U;
        (void)bc_scan_feed(p_scan, b_black, ticks, msec, 1U);
    }

    for (uint32_t k = 0U; k < TEST_END_GAP; k += step_ticks)
    {
        ticks += step_ticks;
        msec += 10U;
        (void)bc_scan_feed(p_scan, 0U, ticks, msec, 1U);
    }
    return msec;
}

static void test_letters (void)
{
    for (uint8_t letter = 0U; letter < 26U; ++letter)
    {
        uint32_t widths[BC_CODE_ELEMENTS] = {0U};
        uint32_t reverse_widths[BC_CODE_ELEMENTS] = {0U};
        bc_code_result_t result;
        bc_scan_t capture;
        uint8_t error = 0U;
        uint32_t when = 0U;
        uint32_t copied[BC_CODE_ELEMENTS] = {0U};

        test_make(letter, widths);
        result = bc_code_decode(widths, 4U);
        test_check(result.valid && (result.letter == letter) &&
                   !result.reverse, "A-Z forward");

        for (uint8_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
        {
            reverse_widths[i] = widths[BC_CODE_ELEMENTS - 1U - i];
        }
        result = bc_code_decode(reverse_widths, 4U);
        test_check(result.valid && (result.letter == letter) &&
                   result.reverse, "A-Z reverse");

        (void)test_feed(&capture, widths, 2U, 0U);
        test_check(BC_SCAN_EVENT_READY ==
                   bc_scan_take(&capture, copied, &error, &when),
                   "full scan complete");
        result = bc_code_decode(copied, 4U);
        test_check(result.valid && result.letter == letter,
                   "full scan decodes");

        (void)test_feed(&capture, widths, 2U, 1U);
        test_check(BC_SCAN_EVENT_READY ==
                   bc_scan_take(&capture, copied, &error, &when),
                   "reverse scan complete");
        result = bc_code_decode(copied, 4U);
        test_check(result.valid && result.letter == letter &&
                   result.reverse, "reverse scan decodes");
    }
}

static void test_invalid (void)
{
    bc_code_pattern_t pattern = {0U};
    uint32_t widths[BC_CODE_ELEMENTS] = {0U};
    bc_code_result_t result;
    bc_scan_t capture;
    bc_scan_config_t config = {4U, 50U, 50U, 300U,
                                TEST_MAX_DISTANCE, 200U};

    test_check(!bc_code_get(26U, &pattern), "letter 26 reserved");
    test_check(!bc_code_get(31U, &pattern), "letter 31 invalid");
    test_make(0U, widths);
    widths[1U] = TEST_NARROW;
    result = bc_code_decode(widths, 4U);
    test_check(!result.valid, "corrupted start rejected");
    test_make(5U, widths);
    widths[9U] = TEST_WIDE;
    result = bc_code_decode(widths, 4U);
    test_check(!result.valid, "wide separator rejected");
    test_make(25U, widths);
    widths[28U] = 1U;
    result = bc_code_decode(widths, 4U);
    test_check(!result.valid, "narrow corrupt rejected");

    test_check(bc_scan_init(&capture, &config), "bad scan setup");
    (void)bc_scan_feed(&capture, 0U, 0U, 0U, 1U);
    (void)bc_scan_feed(&capture, 0U, 80U, 10U, 1U);
    (void)bc_scan_feed(&capture, 1U, 80U, 20U, 1U);
    (void)bc_scan_feed(&capture, 1U, 80U, 230U, 1U);
    test_check(capture.state == BC_SCAN_INVALID,
               "timeout on stationary black");

    (void)bc_scan_init(&capture, &config);
    (void)bc_scan_feed(&capture, 0U, 0U, 0U, 1U);
    (void)bc_scan_feed(&capture, 0U, 80U, 10U, 1U);
    (void)bc_scan_feed(&capture, 1U, 80U, 20U, 1U);
    (void)bc_scan_feed(&capture, 1U, 80U, 30U, 0U);
    test_check(capture.error == BC_SCAN_ERROR_INPUT,
               "encoder fault during scan");
}

static void test_sampled_pdfs (void)
{
    /* Widths extracted from a horizontal scan of the uploaded A, F, Z
     * barcode drawings, rounded to whole PDF page pixels. These fixtures
     * are independent of the lookup table in bc_code.c.
     */
    uint32_t const fixtures[3][BC_CODE_ELEMENTS] =
    {
        {19U,60U,19U,20U,59U,21U,59U,20U,19U,21U,
         59U,20U,19U,20U,20U,60U,19U,20U,59U,21U,
         19U,60U,19U,20U,60U,20U,59U,20U,19U},
        {19U,60U,19U,20U,59U,21U,59U,20U,19U,21U,
         19U,20U,59U,20U,60U,59U,20U,20U,19U,21U,
         19U,60U,19U,20U,60U,20U,59U,20U,19U},
        {19U,60U,19U,20U,59U,21U,59U,20U,19U,21U,
         19U,60U,59U,20U,59U,20U,20U,20U,19U,21U,
         19U,60U,19U,20U,60U,20U,59U,20U,19U}
    };
    uint8_t const letters[3] = {0U, 5U, 25U};

    for (uint8_t i = 0U; i < 3U; ++i)
    {
        bc_code_result_t result = bc_code_decode(fixtures[i], 4U);
        test_check(result.valid && (result.letter == letters[i]),
                   "uploaded PDF A/F/Z widths");
    }
}

static void test_recorded_reverse_a (void)
{
    /* Actual complete COM8 frame supplied by the user, 2026-10-29.
     * Independent of the symbol lookup table; narrow widths span 35..118.
     */
    uint32_t const recorded[BC_CODE_ELEMENTS] = {
        82U,46U,252U,35U,252U,49U,83U,222U,73U,63U,
        239U,39U,69U,223U,62U,100U,64U,79U,213U,90U,
        87U,45U,223U,76U,208U,118U,35U,248U,71U
    };
    uint32_t changed[BC_CODE_ELEMENTS] = {0U};
    bc_code_result_t result = bc_code_decode(recorded, 20U);
    test_check(result.valid && result.letter == 0U && result.reverse,
               "recorded moving reverse A");
    for (uint8_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        changed[i] = recorded[BC_CODE_ELEMENTS - 1U - i];
    }
    result = bc_code_decode(changed, 20U);
    test_check(result.valid && result.letter == 0U && !result.reverse,
               "recorded A in forward order");
    for (uint8_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        changed[i] = recorded[i] * 2U;
    }
    result = bc_code_decode(changed, 40U);
    test_check(result.valid && result.letter == 0U && result.reverse,
               "recorded A at doubled scale");
    for (uint8_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        changed[i] = recorded[i];
    }
    changed[3U] = 20U;
    result = bc_code_decode(changed, 20U);
    test_check(!result.valid, "extreme short narrow run still rejected");
    changed[3U] = recorded[3U];
    changed[25U] = 190U;
    result = bc_code_decode(changed, 20U);
    test_check(!result.valid, "ambiguous wide/narrow separation rejected");
}

static void test_narrow_variation (void)
{
    uint32_t const narrow[20] = {
        5U,8U,9U,10U,11U,12U,16U,10U,9U,11U,
        10U,8U,12U,10U,9U,11U,10U,9U,10U,10U
    };
    for (uint8_t letter = 0U; letter < 26U; ++letter)
    {
        uint32_t widths[BC_CODE_ELEMENTS] = {0U};
        uint32_t reversed[BC_CODE_ELEMENTS] = {0U};
        uint8_t next = 0U;
        test_make(letter, widths);
        for (uint8_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
        {
            if (TEST_NARROW == widths[i])
            {
                widths[i] = narrow[next++];
            }
            reversed[BC_CODE_ELEMENTS - 1U - i] = widths[i];
        }
        bc_code_result_t result = bc_code_decode(widths, 4U);
        test_check(result.valid && result.letter == letter &&
                   !result.reverse, "A-Z with median-bounded narrow runs");
        result = bc_code_decode(reversed, 4U);
        test_check(result.valid && result.letter == letter &&
                   result.reverse, "reverse A-Z with narrow variation");
    }
}

static void test_pauses_and_tilt (void)
{
    bc_scan_t capture;
    bc_scan_config_t config = {4U, 50U, 50U, 300U, 900U, 600U};
    uint32_t widths[BC_CODE_ELEMENTS] = {0U};
    bc_code_result_t result;

    test_check(bc_scan_init(&capture, &config), "pause setup");
    (void)bc_scan_feed(&capture, 0U, 0U, 0U, 1U);
    (void)bc_scan_feed(&capture, 0U, 70U, 10U, 1U);
    (void)bc_scan_feed(&capture, 1U, 70U, 20U, 1U);
    (void)bc_scan_feed(&capture, 1U, 90U, 30U, 1U);
    (void)bc_scan_feed(&capture, 0U, 90U, 40U, 1U);
    for (uint32_t msec = 50U; msec <= 250U; msec += 10U)
    {
        uint8_t event = bc_scan_feed(&capture, 0U, 90U,
                                     msec, 1U);
        test_check(event == BC_SCAN_EVENT_NONE,
                   "stopped on white is not completion");
    }
    (void)bc_scan_feed(&capture, 0U, 160U, 260U, 1U);
    test_check(capture.state == BC_SCAN_CAPTURING,
               "long internal white not barcode end");

    test_make(0U, widths);
    widths[12U] = 100U;
    result = bc_code_decode(widths, 4U);
    test_check(!result.valid, "tilt width distortion rejected");

    bc_scan_interrupt(&capture, 270U);
    test_check(capture.error == BC_SCAN_ERROR_INTERRUPTED,
               "interrupted scan rejected");
}

static void test_recorded_z (void)
{
    /* User-confirmed Z: black/white widths have different baselines. */
    uint32_t widths[BC_CODE_ELEMENTS] = {
        34U, 257U, 15U, 126U, 159U, 123U, 167U, 126U, 15U,
        163U, 9U, 253U, 165U, 119U, 155U, 120U, 14U, 135U, 15U,
        126U, 16U, 264U, 12U, 139U, 139U, 109U, 203U, 136U, 12U
    };
    uint32_t reverse[BC_CODE_ELEMENTS] = {0U};
    bc_code_result_t result = bc_code_decode(widths, 3U);
    test_check(result.valid && (25U == result.letter) && !result.reverse,
               "recorded biased-width Z forward");
    for (uint8_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        reverse[i] = widths[BC_CODE_ELEMENTS - 1U - i];
    }
    result = bc_code_decode(reverse, 3U);
    test_check(result.valid && (25U == result.letter) && result.reverse,
               "recorded biased-width Z reverse");

    widths[21U] = 190U;
    result = bc_code_decode(widths, 3U);
    test_check(!result.valid, "ambiguous space separation rejected");
    widths[21U] = 264U;
    widths[9U] = 253U;
    result = bc_code_decode(widths, 3U);
    test_check(!result.valid, "recorded Z wide separator rejected");
    widths[9U] = 163U;
    widths[10U] = 0U;
    result = bc_code_decode(widths, 3U);
    test_check(!result.valid, "zero measured width rejected");
}

int test_barc_run (void)
{
    test_letters();
    test_invalid();
    test_sampled_pdfs();
    test_recorded_reverse_a();
    test_recorded_z();
    test_narrow_variation();
    test_pauses_and_tilt();
    printf("Barcode core: %d assertions, %d failed\n",
           g_assertions, g_failures);
    return g_failures;
}

#ifdef BC_BARCODE_TEST_MAIN
int main (void)
{
    return (test_barc_run() == 0) ? 0 : 1;
}
#endif

/*** end of file ***/

