/** @file ir_filter.c
 * @brief Constant-time three-channel digital IR debouncer.
 */
#include "ir_filter.h"
#include <stddef.h>

bool ir_filter_init (ir_filter_t * p_filter, uint8_t polarity_bits)
{
    if ((NULL == p_filter) || (0U != (polarity_bits & ~IR_FILTER_BITS)))
    {
        return false;
    }
    *p_filter = (ir_filter_t){0};
    p_filter->polarity_bits = polarity_bits;
    return true;
}

bool ir_filter_step (ir_filter_t * p_filter, uint8_t raw_bits,
                     uint8_t * p_black_bits)
{
    uint8_t blacks = 0U;
    bool b_all_ready = true;

    if ((NULL == p_filter) || (NULL == p_black_bits) ||
        (0U != (raw_bits & ~IR_FILTER_BITS)))
    {
        return false;
    }
    blacks = raw_bits ^ ((uint8_t)~p_filter->polarity_bits &
                         IR_FILTER_BITS);
    for (uint8_t index = 0U; index < 3U; ++index)
    {
        uint8_t mask = (uint8_t)(1U << index);
        if ((blacks & mask) != (p_filter->previous_bits & mask))
        {
            p_filter->counts[index] = 1U;
        }
        else if (p_filter->counts[index] < IR_FILTER_STABLE_SAMPLES)
        {
            ++p_filter->counts[index];
        }
        if (p_filter->counts[index] >= IR_FILTER_STABLE_SAMPLES)
        {
            p_filter->stable_bits = (uint8_t)(
                (p_filter->stable_bits & (uint8_t)~mask) |
                (blacks & mask));
        }
        else
        {
            b_all_ready = false;
        }
    }
    p_filter->previous_bits = blacks;
    if (b_all_ready)
    {
        p_filter->b_valid = true;
    }
    *p_black_bits = p_filter->stable_bits;
    return p_filter->b_valid;
}

/*** end of file ***/

