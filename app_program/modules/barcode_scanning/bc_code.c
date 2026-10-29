/** @file bc_code.c
 * @brief Fixed nine-bit Code 39 pattern table and strict classification.
 */
#include "bc_code.h"

#define BC_CODE_STAR 0x094U
#define BC_CODE_WIDE_PER_SYMBOL 3U
#define BC_CODE_WIDE_TOTAL 9U
#define BC_CODE_SORT_SPLIT 20U

/* Independent Code 39 reference: b/B=bar and s/S=space, upper is wide.
 * The symbol table is A..Z; separators at indexes 9 and 19 are narrow.
 */
static bc_code_pattern_t const g_bc_patterns[26] =
{
    {0x109U}, {0x049U}, {0x148U}, {0x019U}, {0x118U}, {0x058U},
    {0x00DU}, {0x10CU}, {0x04CU}, {0x01CU}, {0x103U}, {0x043U},
    {0x142U}, {0x013U}, {0x112U}, {0x052U}, {0x007U}, {0x106U},
    {0x046U}, {0x016U}, {0x181U}, {0x0C1U}, {0x1C0U}, {0x091U},
    {0x190U}, {0x0D0U}
};

static uint16_t bc_code_mask (uint8_t const * p_wide, uint8_t offset)
{
    uint16_t mask = 0U;

    for (uint8_t index = 0U; index < 9U; ++index)
    {
        if (0U != p_wide[offset + index])
        {
            mask |= (uint16_t)(1U << (8U - index));
        }
    }

    return mask;
}

static uint8_t bc_code_count (uint16_t bits)
{
    uint8_t count = 0U;

    for (uint8_t index = 0U; index < 9U; ++index)
    {
        count += (uint8_t)((bits >> index) & 1U);
    }

    return count;
}

bool bc_code_get (uint8_t letter, bc_code_pattern_t * p_pattern)
{
    if ((letter >= 26U) || (0 == p_pattern))
    {
        return false;
    }

    *p_pattern = g_bc_patterns[letter];
    return true;
}

bc_code_result_t bc_code_decode (uint32_t const * p_width,
                                  uint32_t min_width)
{
    bc_code_result_t result = {BC_CODE_LETTER_INVALID, 0U, 0U, 0U};
    uint32_t sorted[BC_CODE_ELEMENTS] = {0U};
    uint8_t  wide[BC_CODE_ELEMENTS] = {0U};
    uint32_t narrow_min;
    uint32_t narrow_max;
    uint32_t wide_min;
    uint32_t wide_max;
    uint8_t  wide_count = 0U;

    if ((0 == p_width) || (0U == min_width))
    {
        return result;
    }

    for (uint8_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        uint8_t slot = i;

        if (p_width[i] < min_width)
        {
            return result;
        }

        while ((slot > 0U) && (sorted[slot - 1U] > p_width[i]))
        {
            sorted[slot] = sorted[slot - 1U];
            --slot;
        }

        sorted[slot] = p_width[i];
    }

    narrow_min = sorted[0U];
    narrow_max = sorted[BC_CODE_SORT_SPLIT - 1U];
    wide_min = sorted[BC_CODE_SORT_SPLIT];
    wide_max = sorted[BC_CODE_ELEMENTS - 1U];

    /* Strict separation and coherent widths, using 64-bit products to
     * avoid overflow. Reject tilt-induced or partial bar distortion.
     */
    if (((uint64_t)wide_min * 10U < (uint64_t)narrow_max * 17U) ||
        ((uint64_t)narrow_max > (uint64_t)narrow_min * 2U) ||
        ((uint64_t)wide_max > (uint64_t)wide_min * 2U))
    {
        return result;
    }

    for (uint8_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        wide[i] = (p_width[i] >= wide_min) ? 1U : 0U;
        wide_count += wide[i];
    }

    if ((BC_CODE_WIDE_TOTAL != wide_count) || (0U != wide[9U]) ||
        (0U != wide[19U]))
    {
        return result;
    }

    for (uint8_t direction = 0U; direction < 2U; ++direction)
    {
        uint8_t normalized[BC_CODE_ELEMENTS] = {0U};

        for (uint8_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
        {
            normalized[i] = (0U == direction) ? wide[i] :
                           wide[BC_CODE_ELEMENTS - 1U - i];
        }

        uint16_t first = bc_code_mask(normalized, 0U);
        uint16_t last = bc_code_mask(normalized, 20U);
        uint16_t letter_mask = bc_code_mask(normalized, 10U);

        if ((BC_CODE_STAR == first) && (BC_CODE_STAR == last) &&
            (BC_CODE_WIDE_PER_SYMBOL == bc_code_count(letter_mask)))
        {
            for (uint8_t letter = 0U; letter < 26U; ++letter)
            {
                if (g_bc_patterns[letter].element_pattern == letter_mask)
                {
                    result.letter = letter;
                    result.reverse = direction;
                    result.valid = 1U;
                    result.quality =
                        ((uint64_t)wide_min * 10U >=
                         (uint64_t)narrow_max * 23U) ? 3U : 2U;
                    return result;
                }
            }
        }
    }

    return result;
}

/*** end of file ***/

