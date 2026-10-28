/** @file bc_decode.c
 * @brief Decode Code 39 A-Z with a measured wide/narrow separation.
 */
#include "bc_decode.h"
#include <stddef.h>

/* Code 39: nine alternating elements, three wide per symbol. */
static char const * const g_letter_patterns[26] =
{
    "wnnnnwnnw", "nnwnnwnnw", "wnwnnwnnn", "nnnnwwnnw",
    "wnnnwwnnn", "nnwnwwnnn", "nnnnnwwnw", "wnnnnwwnn",
    "nnwnnwwnn", "nnnnwwwnn", "wnnnnnnww", "nnwnnnnww",
    "wnwnnnnwn", "nnnnwnnww", "wnnnwnnwn", "nnwnwnnwn",
    "nnnnnnwww", "wnnnnnwwn", "nnwnnnwwn", "nnnnwnwwn",
    "wwnnnnnnw", "nwwnnnnnw", "wwwnnnnnn", "nwnnwnnnw",
    "wwnnwnnnn", "nwwnwnnnn"
};
static char const g_star_pattern[] = "nwnnwnwnn";

static bool pattern_equal (char const * p_left, char const * p_right)
{
    for (uint8_t index = 0U; index < 9U; ++index)
    {
        if (p_left[index] != p_right[index])
        {
            return false;
        }
    }
    return true;
}

bool bc_decode_pattern (uint8_t letter, char * p_pattern)
{
    if ((letter >= 26U) || (NULL == p_pattern))
    {
        return false;
    }
    for (uint8_t index = 0U; index < 9U; ++index)
    {
        p_pattern[index] = g_letter_patterns[letter][index];
    }
    return true;
}

bc_decode_result_t bc_decode_runs (uint32_t const * p_run_ticks,
                                    uint32_t min_ticks)
{
    bc_decode_result_t result = {BC_INVALID_LETTER, 0U, false, false};
    uint32_t sorted[BC_RUN_COUNT] = {0U};
    char pattern[BC_RUN_COUNT] = {0};
    uint8_t wide_count = 0U;

    if ((NULL == p_run_ticks) || (0U == min_ticks))
    {
        return result;
    }
    for (uint8_t index = 0U; index < BC_RUN_COUNT; ++index)
    {
        uint32_t value = p_run_ticks[index];
        uint8_t slot = index;

        if (value < min_ticks)
        {
            return result;
        }
        while ((slot > 0U) && (sorted[slot - 1U] > value))
        {
            sorted[slot] = sorted[slot - 1U];
            --slot;
        }
        sorted[slot] = value;
    }

    /* Exactly nine wide and twenty narrow elements, including two gaps. */
    if (((uint64_t)sorted[20] * 10U) <
         ((uint64_t)sorted[19] * 17U))
    {
        return result;
    }
    if (((uint64_t)sorted[28] * 10U) >
         ((uint64_t)sorted[0] * 48U))
    {
        return result;
    }
    for (uint8_t index = 0U; index < BC_RUN_COUNT; ++index)
    {
        pattern[index] = (p_run_ticks[index] >= sorted[20]) ? 'w' : 'n';
        if ('w' == pattern[index])
        {
            ++wide_count;
        }
    }
    if ((9U != wide_count) || ('n' != pattern[9]) ||
        ('n' != pattern[19]))
    {
        return result;
    }

    for (uint8_t direction = 0U; direction < 2U; ++direction)
    {
        char normalized[BC_RUN_COUNT] = {0};
        for (uint8_t index = 0U; index < BC_RUN_COUNT; ++index)
        {
            normalized[index] = (0U == direction) ? pattern[index] :
                                pattern[BC_RUN_COUNT - 1U - index];
        }
        if (pattern_equal(normalized, g_star_pattern) &&
            pattern_equal(&normalized[20], g_star_pattern))
        {
            for (uint8_t letter = 0U; letter < 26U; ++letter)
            {
                if (pattern_equal(&normalized[10],
                                  g_letter_patterns[letter]))
            {
                    result.letter = letter;
                    result.confidence = 1U;
                    result.b_reverse = (1U == direction);
                    result.b_valid = true;
                    return result;
                }
            }
        }
    }
    return result;
}

/*** end of file ***/

