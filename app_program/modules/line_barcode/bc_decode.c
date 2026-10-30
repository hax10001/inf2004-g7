/** @file bc_decode.c
 * @brief Validate Code 39 start, uppercase data and stop using width ratios.
 */
#include "bc_decode.h"
#include <stddef.h>

#define BC_NARROW_COUNT 20U
#define BC_WIDE_COUNT 9U
#define BC_SYMBOL_RUNS 9U
#define BC_STAR_PATTERN 0x094U

/* EXPERIMENT ONLY: restore 4U when sensing/sampling is calibrated. */
#define BC_MIN_WIDTH_STEP_FACTOR 1U
#define BC_NARROW_BLACK_TOL_PERCENT 55U
#define BC_OTHER_TOL_PERCENT 30U

/* Standard Code 39 A-Z encodings, also used by ZXing-C++ Code39Reader.
 * Bit 8 is the first black bar; one means wide and zero means narrow.
 */
static uint16_t const g_letter_patterns[26] = {
    0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U, 0x00DU,
    0x10CU, 0x04CU, 0x01CU, 0x103U, 0x043U, 0x142U, 0x013U,
    0x112U, 0x052U, 0x007U, 0x106U, 0x046U, 0x016U, 0x181U,
    0x0C1U, 0x1C0U, 0x091U, 0x190U, 0x0D0U};

/** @brief Sort a small, fixed-size copy without changing the measurements. */
static void sort_widths (uint32_t widths[BC_RUN_COUNT]);
/** @brief Build a nine-bit narrow/wide pattern, first element in bit eight. */
static uint16_t make_pattern (uint32_t const * p_runs,
                              uint32_t narrow_max);
/** @brief Map a failing sorted width back to its first original index. */
static uint32_t width_index (uint32_t const runs[BC_RUN_COUNT],
                             uint32_t width);

bc_status_t bc_decode (uint32_t const runs[BC_RUN_COUNT],
                       uint32_t max_step_units, bc_decoded_t * p_result)
{
    return bc_decode_inspect(runs, max_step_units, p_result, NULL);
}

bc_status_t bc_decode_inspect (uint32_t const runs[BC_RUN_COUNT],
                               uint32_t max_step_units,
                               bc_decoded_t * p_result,
                               bc_decode_report_t * p_report)
{
    uint32_t sorted[BC_RUN_COUNT];
    uint64_t total = 0U;
    uint64_t narrow_sum = 0U;
    uint64_t wide_sum = 0U;
    bc_decode_report_t discarded;
    bc_decode_report_t * const p_trace =
        (NULL != p_report) ? p_report : &discarded;

    *p_trace = (bc_decode_report_t){
        .check = BC_CHECK_NOT_RUN,
        .failed_index = BC_RUN_COUNT,
        .max_step_units = max_step_units,
        .required_min = (BC_MIN_WIDTH_STEP_FACTOR *
                         (uint64_t)max_step_units) + 2U,
        .required_separation = (2U * (uint64_t)max_step_units) + 2U};

    if ((NULL == runs) || (NULL == p_result))
    {
        p_trace->check = BC_CHECK_INPUT;
        return BC_ERR_PATTERN;
    }

    for (uint32_t index = 0U; index < BC_RUN_COUNT; index++)
    {
        if (0U == runs[index])
        {
            p_trace->check = BC_CHECK_ZERO_WIDTH;
            p_trace->failed_index = index;
            return BC_ERR_PATTERN;
        }

        sorted[index] = runs[index];
        total += runs[index];
    }

    if (total > UINT32_MAX)
    {
        p_trace->check = BC_CHECK_TOTAL_OVERFLOW;
        return BC_ERR_OVERFLOW;
    }

    sort_widths(sorted);

    /* Three characters contain 18 narrow and nine wide elements. The two
     * inter-character white gaps supply the remaining two narrow widths.
     */
    for (uint32_t index = 0U; index < BC_NARROW_COUNT; index++)
    {
        narrow_sum += sorted[index];
    }

    for (uint32_t index = BC_NARROW_COUNT; index < BC_RUN_COUNT; index++)
    {
        wide_sum += sorted[index];
    }

    uint32_t const narrow_max = sorted[BC_NARROW_COUNT - 1U];
    uint32_t const wide_min = sorted[BC_NARROW_COUNT];

    /* Capture every comparison value before either resolution check. The
     * bit patterns here describe classification, not accepted symbols.
     */
    p_trace->min_width = sorted[0];
    p_trace->narrow_max = narrow_max;
    p_trace->wide_min = wide_min;
    p_trace->narrow_sum = narrow_sum;
    p_trace->wide_sum = wide_sum;
    p_trace->start_pattern = make_pattern(&runs[0], narrow_max);
    p_trace->letter_pattern = make_pattern(&runs[10], narrow_max);
    p_trace->stop_pattern = make_pattern(&runs[20], narrow_max);
    p_trace->b_metrics_valid = true;

    if ((uint64_t)sorted[0] < p_trace->required_min)
    {
        p_trace->check = BC_CHECK_MIN_WIDTH;
        p_trace->failed_index = width_index(runs, sorted[0]);
        return BC_ERR_RESOLUTION;
    }

    if ((uint64_t)wide_min <=
        (uint64_t)narrow_max + p_trace->required_separation)
    {
        p_trace->check = BC_CHECK_SEPARATION;
        return BC_ERR_RESOLUTION;
    }

    /* Compare cluster means without rounding. Printed ratios are normally
     * 2:1 to 3:1; 1.8:1 to 3.5:1 allows modest measurement/printing error.
     */
    uint64_t const scaled_wide = wide_sum * BC_NARROW_COUNT * 10U;
    uint64_t const scaled_narrow = narrow_sum * BC_WIDE_COUNT;

    if ((scaled_wide < (18U * scaled_narrow)) ||
        (scaled_wide > (35U * scaled_narrow)))
    {
        p_trace->check = BC_CHECK_WIDTH_RATIO;
        return BC_ERR_PATTERN;
    }

    /* EXPERIMENT: compare widths against their own colour and size class.
     * Group 0: narrow black, 1: narrow white,
     *       2: wide black,   3: wide white.
     * In an ideal barcode, equivalent black and white elements have equal
     * widths; separate means tolerate the measured IR threshold bias.
     */
    uint64_t group_sums[4] = {0U, 0U, 0U, 0U};
    uint32_t group_counts[4] = {0U, 0U, 0U, 0U};

    for (uint32_t index = 0U; index < BC_RUN_COUNT; index++)
    {
        uint32_t const group =
            ((runs[index] > narrow_max) ? 2U : 0U) + (index & 1U);

        group_sums[group] += runs[index];
        group_counts[group]++;
    }

    for (uint32_t index = 0U; index < BC_RUN_COUNT; index++)
    {
        uint32_t const group =
            ((runs[index] > narrow_max) ? 2U : 0U) + (index & 1U);
        uint32_t const tolerance = (0U == group) ?
            BC_NARROW_BLACK_TOL_PERCENT : BC_OTHER_TOL_PERCENT;
        uint64_t const scaled =
            (uint64_t)runs[index] * group_counts[group] * 100U;
        uint64_t const min_allowed =
            group_sums[group] * (100U - tolerance);
        uint64_t const max_allowed =
            group_sums[group] * (100U + tolerance);

        if ((scaled < min_allowed) || (scaled > max_allowed))
        {
            p_trace->check = (group < 2U) ?
                BC_CHECK_NARROW_TOLERANCE : BC_CHECK_WIDE_TOLERANCE;
            p_trace->failed_index = index;
            return BC_ERR_PATTERN;
        }
    }

    if (runs[9] > narrow_max)
    {
        p_trace->check = BC_CHECK_GAP_FIRST;
        p_trace->failed_index = 9U;
        return BC_ERR_PATTERN;
    }
    if (runs[19] > narrow_max)
    {
        p_trace->check = BC_CHECK_GAP_SECOND;
        p_trace->failed_index = 19U;
        return BC_ERR_PATTERN;
    }
    if (BC_STAR_PATTERN != p_trace->start_pattern)
    {
        p_trace->check = BC_CHECK_START_SYMBOL;
        p_trace->failed_index = 0U;
        return BC_ERR_PATTERN;
    }
    if (BC_STAR_PATTERN != p_trace->stop_pattern)
    {
        p_trace->check = BC_CHECK_STOP_SYMBOL;
        p_trace->failed_index = 20U;
        return BC_ERR_PATTERN;
    }

    uint16_t const pattern = p_trace->letter_pattern;

    for (uint32_t index = 0U; index < 26U; index++)
    {
        if (pattern == g_letter_patterns[index])
        {
            p_result->letter = (char)('A' + index);
            p_result->narrow_units =
                (uint32_t)((narrow_sum + (BC_NARROW_COUNT / 2U)) /
                           BC_NARROW_COUNT);
            p_result->width_units = (uint32_t)total;
            p_trace->check = BC_CHECK_OK;
            return BC_OK;
        }
    }

    p_trace->check = BC_CHECK_LETTER;
    p_trace->failed_index = 10U;
    return BC_ERR_PATTERN;
}

char const * bc_check_name (bc_check_t check)
{
    switch (check)
    {
        case BC_CHECK_NOT_RUN:
            return "NOT_RUN";
        case BC_CHECK_OK:
            return "OK";
        case BC_CHECK_INPUT:
            return "INPUT";
        case BC_CHECK_ZERO_WIDTH:
            return "ZERO_WIDTH";
        case BC_CHECK_TOTAL_OVERFLOW:
            return "TOTAL_OVERFLOW";
        case BC_CHECK_MIN_WIDTH:
            return "MIN_WIDTH";
        case BC_CHECK_SEPARATION:
            return "SEPARATION";
        case BC_CHECK_WIDTH_RATIO:
            return "WIDTH_RATIO";
        case BC_CHECK_NARROW_TOLERANCE:
            return "NARROW_TOLERANCE";
        case BC_CHECK_WIDE_TOLERANCE:
            return "WIDE_TOLERANCE";
        case BC_CHECK_GAP_FIRST:
            return "GAP_FIRST";
        case BC_CHECK_GAP_SECOND:
            return "GAP_SECOND";
        case BC_CHECK_START_SYMBOL:
            return "START_SYMBOL";
        case BC_CHECK_STOP_SYMBOL:
            return "STOP_SYMBOL";
        case BC_CHECK_LETTER:
            return "LETTER";
        default:
            return "UNKNOWN";
    }
}

static void sort_widths (uint32_t widths[BC_RUN_COUNT])
{
    for (uint32_t index = 1U; index < BC_RUN_COUNT; index++)
    {
        uint32_t const value = widths[index];
        uint32_t position = index;

        while ((position > 0U) && (widths[position - 1U] > value))
        {
            widths[position] = widths[position - 1U];
            position--;
        }

        widths[position] = value;
    }
}

static uint16_t make_pattern (uint32_t const * p_runs, uint32_t narrow_max)
{
    uint16_t pattern = 0U;

    for (uint32_t index = 0U; index < BC_SYMBOL_RUNS; index++)
    {
        pattern = (uint16_t)(pattern << 1U);

        if (p_runs[index] > narrow_max)
        {
            pattern |= 1U;
        }
    }

    return pattern;
}

static uint32_t width_index (uint32_t const runs[BC_RUN_COUNT],
                             uint32_t width)
{
    for (uint32_t index = 0U; index < BC_RUN_COUNT; index++)
    {
        if (width == runs[index])
        {
            return index;
        }
    }

    return BC_RUN_COUNT;
}

/*** end of file ***/

