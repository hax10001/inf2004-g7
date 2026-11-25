/** @file test_bc_code.c
 * @brief Host tests of the real Code 39 element decoder bc_code.c
 *        (FR5: A-Z read in both directions; bad codes give no letter).
 *
 * The expected patterns come from an independent Code 39 reference
 * written below as wide/narrow strings, not from the firmware table.
 */
#include "bc_code.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define T_NARROW 10U       /* narrow element width, travel units */
#define T_WIDE 30U         /* wide element width (3:1 print) */
#define T_MIN_WIDTH 3U     /* smallest trustworthy run */
#define T_SYMBOL 9U        /* elements per symbol */
#define T_STRIDE 10U       /* symbol plus separator */
#define T_LETTER_AT 10U    /* first element of the letter symbol */
#define T_STOP_AT 20U      /* first element of the stop star */

/* Code 39 reference, element order bar,space,bar,...; '1' = wide. */
static char const * const g_ref_letter[BC_CODE_LETTERS] =
{
    "100001001", "001001001", "101001000", "000011001", "100011000",
    "001011000", "000001101", "100001100", "001001100", "000011100",
    "100000011", "001000011", "101000010", "000010011", "100010010",
    "001010010", "000000111", "100000110", "001000110", "000010110",
    "110000001", "011000001", "111000000", "010010001", "110010000",
    "011010000"
};
static char const g_ref_star[] = "010010100";
static char const g_ref_digit0[] = "000110100";
static char const g_ref_dash[] = "010000101";

/** @brief Write one symbol's nine widths from a reference string. */
static void put_symbol (uint32_t * p_width, uint32_t at,
                        char const * p_ref, uint32_t narrow,
                        uint32_t wide)
{
    for (uint32_t i = 0U; i < T_SYMBOL; ++i)
    {
        p_width[at + i] = ('1' == p_ref[i]) ? wide : narrow;
    }
}

/** @brief Build *X* widths (start, letter, stop) with chosen widths. */
static void build_frame (uint32_t * p_width, char const * p_start,
                         char const * p_letter, char const * p_stop,
                         uint32_t narrow, uint32_t wide)
{
    put_symbol(p_width, 0U, p_start, narrow, wide);
    p_width[T_SYMBOL] = narrow;
    put_symbol(p_width, T_LETTER_AT, p_letter, narrow, wide);
    p_width[T_SYMBOL + T_STRIDE] = narrow;
    put_symbol(p_width, T_STOP_AT, p_stop, narrow, wide);
}

/** @brief Build an ideal *X* capture for letter index 0..25. */
static void build_letter (uint32_t * p_width, uint8_t letter,
                          uint32_t narrow, uint32_t wide)
{
    build_frame(p_width, g_ref_star, g_ref_letter[letter], g_ref_star,
                narrow, wide);
}

/** @brief Reverse a capture as seen when driving the other way. */
static void reverse_widths (uint32_t * p_width)
{
    for (uint32_t i = 0U; i < BC_CODE_ELEMENTS / 2U; ++i)
    {
        uint32_t const keep = p_width[i];
        p_width[i] = p_width[BC_CODE_ELEMENTS - 1U - i];
        p_width[BC_CODE_ELEMENTS - 1U - i] = keep;
    }
}

/** @brief Assert that a capture is refused with no letter. */
static void expect_refused (uint32_t const * p_width)
{
    bc_code_result_t const r = bc_code_decode(p_width, T_MIN_WIDTH);

    assert(0U == r.valid);
    assert(BC_CODE_LETTER_INVALID == r.letter);
    assert(0U == r.quality);
}

/** @brief The firmware table matches the independent Code 39 reference. */
static void test_table_matches_reference (void)
{
    bc_code_pattern_t pattern = {0U};

    for (uint8_t letter = 0U; letter < BC_CODE_LETTERS; ++letter)
    {
        uint16_t expect = 0U;
        for (uint32_t i = 0U; i < T_SYMBOL; ++i)
        {
            if ('1' == g_ref_letter[letter][i])
            {
                expect |= (uint16_t)(1U << (BC_CODE_TOP_BIT - i));
            }
        }
        assert(bc_code_get(letter, &pattern));
        assert(expect == pattern.element_pattern);
    }
    assert(!bc_code_get(BC_CODE_LETTERS, &pattern));
    assert(!bc_code_get(BC_CODE_LETTER_INVALID, &pattern));
    assert(!bc_code_get(0U, (bc_code_pattern_t *)0));
    puts("PASS bc_code table: A-Z patterns equal Code 39 reference, "
         "out-of-range letter refused");
}

/** @brief Every letter decodes forwards from ideal widths. */
static void test_all_letters_forward (void)
{
    uint32_t width[BC_CODE_ELEMENTS];

    for (uint8_t letter = 0U; letter < BC_CODE_LETTERS; ++letter)
    {
        bc_code_result_t r;
        build_letter(width, letter, T_NARROW, T_WIDE);
        r = bc_code_decode(width, T_MIN_WIDTH);
        assert(1U == r.valid);
        assert(letter == r.letter);
        assert(0U == r.reverse);
        assert(3U == r.quality);
    }
    puts("PASS bc_code FR5: all 26 letters A-Z decode forwards, "
         "strong quality");
}

/** @brief Every letter decodes when scanned backwards. */
static void test_all_letters_reverse (void)
{
    uint32_t width[BC_CODE_ELEMENTS];

    for (uint8_t letter = 0U; letter < BC_CODE_LETTERS; ++letter)
    {
        bc_code_result_t r;
        build_letter(width, letter, T_NARROW, T_WIDE);
        reverse_widths(width);
        r = bc_code_decode(width, T_MIN_WIDTH);
        assert(1U == r.valid);
        assert(letter == r.letter);
        assert(1U == r.reverse);
    }
    puts("PASS bc_code FR5: all 26 letters A-Z decode backwards with "
         "reverse flag set");
}

/** @brief Unequal print/scan widths within tolerance still decode. */
static void test_uneven_widths_decode (void)
{
    uint32_t width[BC_CODE_ELEMENTS];
    bc_code_result_t r;

    /* 'K' at 2.5:1 with jittered narrow and wide runs. */
    build_letter(width, 10U, 20U, 50U);
    width[2] = 23U;
    width[3] = 17U;
    width[T_LETTER_AT] = 55U;
    width[T_LETTER_AT + 8U] = 47U;
    r = bc_code_decode(width, T_MIN_WIDTH);
    assert((1U == r.valid) && (10U == r.letter));
    puts("PASS bc_code: jittered 2.5:1 widths still decode the letter");
}

/** @brief Quality grades at the 1.4x and 2.3x separation boundaries. */
static void test_quality_boundaries (void)
{
    uint32_t width[BC_CODE_ELEMENTS];
    bc_code_result_t r;

    build_letter(width, 0U, 100U, 230U);
    r = bc_code_decode(width, T_MIN_WIDTH);
    assert((1U == r.valid) && (3U == r.quality));

    build_letter(width, 0U, 100U, 229U);
    r = bc_code_decode(width, T_MIN_WIDTH);
    assert((1U == r.valid) && (2U == r.quality));

    build_letter(width, 0U, 100U, 140U);
    r = bc_code_decode(width, T_MIN_WIDTH);
    assert((1U == r.valid) && (0U == r.letter) && (2U == r.quality));

    build_letter(width, 0U, 100U, 139U);
    expect_refused(width);

    /* One weak symbol downgrades the whole read. */
    build_letter(width, 0U, 100U, 300U);
    put_symbol(width, T_STOP_AT, g_ref_star, 100U, 200U);
    r = bc_code_decode(width, T_MIN_WIDTH);
    assert((1U == r.valid) && (2U == r.quality));
    puts("PASS bc_code quality: 2.3x strong, 2.29x good, 1.4x good, "
         "1.39x refused");
}

/** @brief A wide run far wider than the other wide run is refused. */
static void test_wide_spread_refused (void)
{
    uint32_t width[BC_CODE_ELEMENTS];
    bc_code_result_t r;

    /* Letter A: wide bars at elements 0 and 8 of the symbol. */
    build_letter(width, 0U, T_NARROW, T_WIDE);
    width[T_LETTER_AT] = 2U * T_WIDE;
    r = bc_code_decode(width, T_MIN_WIDTH);
    assert((1U == r.valid) && (0U == r.letter));
    width[T_LETTER_AT] = 2U * T_WIDE + 1U;
    expect_refused(width);
    puts("PASS bc_code ratio: widest run > 2x narrowest wide is refused, "
         "exactly 2x accepted");
}

/** @brief Wrong wide-element counts or a corrupted element are refused. */
static void test_corrupted_elements_refused (void)
{
    uint32_t width[BC_CODE_ELEMENTS];

    /* Three wide bars in the letter symbol (narrow bar made wide). */
    build_letter(width, 0U, T_NARROW, T_WIDE);
    width[T_LETTER_AT + 2U] = T_WIDE;
    expect_refused(width);

    /* One wide bar only (wide bar made narrow). */
    build_letter(width, 0U, T_NARROW, T_WIDE);
    width[T_LETTER_AT] = T_NARROW;
    expect_refused(width);

    /* Two wide spaces in the start star. */
    build_letter(width, 0U, T_NARROW, T_WIDE);
    width[5] = T_WIDE;
    expect_refused(width);

    /* No wide space in the stop star. */
    build_letter(width, 0U, T_NARROW, T_WIDE);
    width[T_STOP_AT + 1U] = T_NARROW;
    expect_refused(width);

    /* Separator as wide as a wide space (merged gap). */
    build_letter(width, 0U, T_NARROW, T_WIDE);
    width[T_SYMBOL] = T_WIDE;
    expect_refused(width);
    build_letter(width, 0U, T_NARROW, T_WIDE);
    width[T_SYMBOL + T_STRIDE] = 22U;
    expect_refused(width);
    build_letter(width, 0U, T_NARROW, T_WIDE);
    width[T_SYMBOL + T_STRIDE] = 21U;
    assert(1U == bc_code_decode(width, T_MIN_WIDTH).valid);

    /* One run below the trustworthy minimum (glitch). */
    build_letter(width, 0U, T_NARROW, T_WIDE);
    width[T_LETTER_AT + 3U] = T_MIN_WIDTH - 1U;
    expect_refused(width);
    puts("PASS bc_code reject: 3 or 1 wide bars, 2 or 0 wide spaces, "
         "wide separator, sub-minimum run");
}

/** @brief Valid Code 39 symbols that are not *letter* give no letter. */
static void test_unknown_patterns_refused (void)
{
    uint32_t width[BC_CODE_ELEMENTS];

    build_frame(width, g_ref_star, g_ref_digit0, g_ref_star,
                T_NARROW, T_WIDE);
    expect_refused(width);
    reverse_widths(width);
    expect_refused(width);

    build_frame(width, g_ref_star, g_ref_dash, g_ref_star,
                T_NARROW, T_WIDE);
    expect_refused(width);

    build_frame(width, g_ref_star, g_ref_star, g_ref_star,
                T_NARROW, T_WIDE);
    expect_refused(width);
    puts("PASS bc_code reject: digit 0, '-' and '*' between stars give "
         "no letter");
}

/** @brief Start and stop stars are both required. */
static void test_framing_required (void)
{
    uint32_t width[BC_CODE_ELEMENTS];

    build_frame(width, g_ref_letter[0], g_ref_letter[1], g_ref_star,
                T_NARROW, T_WIDE);
    expect_refused(width);
    reverse_widths(width);
    expect_refused(width);

    build_frame(width, g_ref_star, g_ref_letter[1], g_ref_letter[0],
                T_NARROW, T_WIDE);
    expect_refused(width);

    /* "A" framed by stars but read in an order that breaks the stars. */
    build_frame(width, g_ref_letter[2], g_ref_letter[0],
                g_ref_letter[2], T_NARROW, T_WIDE);
    expect_refused(width);
    puts("PASS bc_code framing: missing start or stop '*' gives no letter "
         "in either direction");
}

/** @brief Null input, zero minimum width and min-width gate. */
static void test_input_guards (void)
{
    uint32_t width[BC_CODE_ELEMENTS];
    bc_code_result_t r;

    r = bc_code_decode((uint32_t const *)0, T_MIN_WIDTH);
    assert((0U == r.valid) && (BC_CODE_LETTER_INVALID == r.letter));
    build_letter(width, 3U, T_NARROW, T_WIDE);
    r = bc_code_decode(width, 0U);
    assert(0U == r.valid);
    r = bc_code_decode(width, T_NARROW);
    assert((1U == r.valid) && (3U == r.letter));
    r = bc_code_decode(width, T_NARROW + 1U);
    assert(0U == r.valid);
    puts("PASS bc_code guards: null widths, zero min width and runs below "
         "min width refused");
}

/** @brief Run every bc_code test. */
int main (void)
{
    test_table_matches_reference();
    test_all_letters_forward();
    test_all_letters_reverse();
    test_uneven_widths_decode();
    test_quality_boundaries();
    test_wide_spread_refused();
    test_corrupted_elements_refused();
    test_unknown_patterns_refused();
    test_framing_required();
    test_input_guards();
    return 0;
}

/*** end of file ***/

