/** @file bc_match.c
 * @brief Shared geometry and weak-feature search for all A-Z templates.
 */
#include "bc_match.h"
#include "bc_code.h"
#include "bc_cal.h"
#include "app/app_config.h"
#include <tm/tmonitor.h>

#if APP_BARCODE_ANALOG_ENABLED && APP_BARCODE_BUFFERED_ENABLED && \
    APP_BARCODE_TEMPLATE_ENABLED && BC_CAL_READY && !BC_CAL_MEASURE
#define BC_MATCH_PEAKS 12U
#define BC_MATCH_LEVELS 6U
#define BC_MATCH_BAD_SCORE (-1000000)
#define BC_MATCH_MAX_GEOMETRY 250U
#define BC_MATCH_DIRECTIONS 2U       /* forward and reverse reading */
#define BC_MATCH_JOBS 52U            /* letters times directions */
#define BC_MATCH_JOBS_PER_POLL 2U    /* jobs run per poll call */
#define BC_MATCH_LOGGED_MODELS 2U    /* best and runner-up are logged */
#define BC_MATCH_ANCHORS 6U          /* wide bars fitted per letter */
#define BC_MATCH_ANCHORS_I 6         /* BC_MATCH_ANCHORS, signed */
#define BC_MATCH_MIN_POSITIVE 6U     /* positive features to qualify */
#define BC_MATCH_Q8_ONE 256U         /* 1.0 in Q8 fixed point */
#define BC_MATCH_Q8_ONE_I 256        /* BC_MATCH_Q8_ONE, signed */
#define BC_MATCH_STAR 0x094U         /* Code 39 '*' wide-element mask */
#define BC_MATCH_FRAME_SYMBOLS 3     /* start star, letter, end star */
#define BC_MATCH_SYMBOL_ELEMENTS 10U /* symbol elements plus the gap */
#define BC_MATCH_CHAR_ELEMENTS 9U    /* bars and spaces in one symbol */
#define BC_MATCH_CHAR_TOP_BIT 8U     /* mask bit of a symbol's 1st element */
#define BC_MATCH_END_STAR_FIRST 20U  /* first element of the end star */
#define BC_MATCH_NARROW_X16 16U      /* narrow width, 1/16 units */
#define BC_MATCH_NARROW_X16_I 16     /* BC_MATCH_NARROW_X16, signed */
#define BC_MATCH_HALF_NARROW 8U      /* half a narrow width, 1/16 units */
#define BC_MATCH_HALF_NARROW_I 8     /* BC_MATCH_HALF_NARROW, signed */
#define BC_MATCH_RADIUS_DIV 5        /* mean radius = narrow / 5 */
#define BC_MATCH_PERCENT 100U        /* percent denominator */
#define BC_MATCH_PERCENT_I 100       /* BC_MATCH_PERCENT, signed */
#define BC_MATCH_PERMILLE 1000U      /* per-mille geometry error scale */
#define BC_MATCH_NOISE_FACTOR 4U     /* anchor threshold floor, x noise */
#define BC_MATCH_MIN_NOISE 4U        /* floor on the noise estimate */
#define BC_MATCH_LOCAL_SPAN_PCT 35   /* diagnostic shift range, +/- % */
#define BC_MATCH_LOCAL_STEP_PCT 5    /* diagnostic shift step, % */
#define BC_MATCH_EVIDENCE_CAP 600    /* max |z100| from one feature */
#define BC_MATCH_EVIDENCE_MIN 50     /* z100 of a positive feature */
#define BC_MATCH_GEOMETRY_DIV 10     /* geometry penalty divisor */
#define BC_MATCH_RATIO_MIN 40U       /* first wide:narrow ratio, x16 */
#define BC_MATCH_RATIO_MAX 56U       /* last wide:narrow ratio, x16 */
#define BC_MATCH_RATIO_STEP 8U       /* wide:narrow ratio step, x16 */
#define BC_MATCH_SHIFT_PCT 15        /* offset nudge, % of narrow */
#define BC_MATCH_MIN_SAMPLES 32U     /* fewest samples worth matching */

typedef struct
{
    uint32_t centre;
    uint32_t width;
} bc_match_peak_t;

typedef struct
{
    int32_t score;
    int32_t offset_q8;
    int32_t scale_q8;
    uint16_t geometry;
    uint8_t ratio;
    uint8_t reverse;
    uint8_t positive;
    uint8_t eligible;
} bc_match_model_t;

static bc_match_read_t g_read;
static uint32_t g_count;
static uint32_t g_noise;
static uint32_t g_maximum;
static uint32_t g_extent;
static uint32_t g_level;
static uint32_t g_job;
static uint32_t g_peak_count;
/* Low levels retain weak wide bars when amplitudes differ across a print. */
static uint8_t const g_anchor_percent[BC_MATCH_LEVELS] = {
    15U, 20U, 25U, 40U, 55U, 70U
};
static uint32_t g_previous_threshold;
static uint8_t g_duplicate_level;
static uint8_t g_peak_overflow;
static uint8_t g_done = 1U;
static bc_match_peak_t g_peaks[BC_MATCH_PEAKS];
static bc_match_model_t g_models[BC_CODE_LETTERS];
static int32_t g_level_score[BC_CODE_LETTERS];
static uint8_t g_support[BC_CODE_LETTERS];
static bc_match_result_t g_result;
static uint32_t g_geometry_reject;
static uint32_t g_clearance_reject;
static uint32_t g_weak_reject;
static int32_t * gp_prefix;
static uint8_t g_prefix_enabled;
static uint32_t g_read_calls;
static uint32_t g_mean_calls;
static uint32_t g_cached_means;

/** @brief Read one capture sample through the reader callback. */
static void match_fetch (uint32_t index, uint32_t * p_units,
                          int32_t * p_signal)
{
    g_read(index, p_units, p_signal);
    ++g_read_calls;
}

/** @brief Return the travel position of one capture sample. */
static uint32_t match_position (uint32_t index)
{
    uint32_t units = 0U;
    int32_t signal = 0;
    match_fetch(index, &units, &signal);
    return units;
}

/** @brief Binary search for the first sample at a position. */
static uint32_t match_lower_bound (uint32_t position)
{
    uint32_t lo = 0U;
    uint32_t hi = g_count;
    while (lo < hi)
    {
        uint32_t mid = lo + (hi - lo) / 2U;
        if (match_position(mid) < position)
        {
            lo = mid + 1U;
        }
        else
        {
            hi = mid;
        }
    }
    return lo;
}

/** @brief Signal integral up to a sample, from the prefix table.
 * The table holds every second sample; an odd index adds one interval.
 */
static int32_t match_integral (uint32_t index)
{
    int32_t area = gp_prefix[index / 2U];
    if (0U != (index % 2U))
    {
        uint32_t position = 0U;
        int32_t signal = 0;
        match_fetch(index - 1U, &position, &signal);
        area += signal * (int32_t)(match_position(index) - position);
    }
    return area;
}

/** @brief Mean signal over a window; 0 if it leaves the capture. */
static uint8_t match_mean (int32_t centre_q8, int32_t radius_q8,
                            int32_t * p_mean)
{
    ++g_mean_calls;
    if ((centre_q8 < radius_q8) ||
        ((uint32_t)(centre_q8 + radius_q8) > g_extent * BC_MATCH_Q8_ONE))
    {
        return 0U;
    }
    uint32_t lo = match_lower_bound(
        (uint32_t)(centre_q8 - radius_q8) / BC_MATCH_Q8_ONE);
    uint32_t hi = match_lower_bound(
        (uint32_t)(centre_q8 + radius_q8) / BC_MATCH_Q8_ONE + 1U);
    if (hi <= lo)
    {
        return 0U;
    }
    if (0U != g_prefix_enabled)
    {
        uint32_t weight = match_position(hi - 1U) - match_position(lo);
        if ((0U == weight) || (weight > INT32_MAX))
        {
            return 0U;
        }
        *p_mean = (match_integral(hi - 1U) - match_integral(lo)) /
                  (int32_t)weight;
        ++g_cached_means;
        return 1U;
    }
    /* Weight by travel, not repeated samples while a wheel is stopped. */
    int64_t sum = 0;
    uint32_t weight = 0U;
    for (uint32_t i = lo; i + 1U < hi; ++i)
    {
        uint32_t position = 0U;
        int32_t signal = 0;
        match_fetch(i, &position, &signal);
        uint32_t step = match_position(i + 1U) - position;
        sum += (int64_t)signal * step;
        weight += step;
    }
    if (0U == weight)
    {
        return 0U;
    }
    *p_mean = (int32_t)(sum / weight);
    return 1U;
}

/** @brief Diagnostic: strongest nearby dark peak around a narrow bar.
 * Logged only; never fed back into the template score, so a peak found
 * in noise cannot become evidence.
 */
static void match_local_peak (int32_t centre, int32_t left, int32_t right,
                              int32_t radius, int32_t narrow,
                              int32_t * p_delta, int32_t * p_shift)
{
    for (int32_t percent = -BC_MATCH_LOCAL_SPAN_PCT;
         percent <= BC_MATCH_LOCAL_SPAN_PCT;
         percent += BC_MATCH_LOCAL_STEP_PCT)
    {
        int32_t shift = narrow * percent / BC_MATCH_PERCENT_I;
        int32_t black = 0;
        int32_t white_left = 0;
        int32_t white_right = 0;
        if ((0U != match_mean(centre + shift, radius, &black)) &&
            (0U != match_mean(left + shift, radius, &white_left)) &&
            (0U != match_mean(right + shift, radius, &white_right)))
        {
            int32_t baseline = (int32_t)
                (((int64_t)white_left * (right - centre) +
                  (int64_t)white_right * (centre - left)) / (right - left));
            int32_t delta = black - baseline;
            int32_t magnitude = (percent < 0) ? -percent : percent;
            int32_t best_magnitude = (*p_shift < 0) ? -*p_shift : *p_shift;
            if ((delta > *p_delta) ||
                ((delta == *p_delta) && (magnitude < best_magnitude)))
            {
                *p_delta = delta;
                *p_shift = percent;
            }
        }
    }
}

/** @brief Build the expected element widths and centres of a letter. */
static void match_pattern (uint8_t letter, uint8_t reverse, uint8_t ratio,
                            uint16_t * p_width, uint32_t * p_centres)
{
    bc_code_pattern_t pattern = {0U};
    (void)bc_code_get(letter, &pattern);
    uint16_t masks[BC_MATCH_FRAME_SYMBOLS] = {
        BC_MATCH_STAR, pattern.element_pattern, BC_MATCH_STAR
    };
    for (uint32_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        uint32_t target = reverse ? BC_CODE_ELEMENTS - 1U - i : i;
        uint32_t symbol = i / BC_MATCH_SYMBOL_ELEMENTS;
        uint32_t element = i % BC_MATCH_SYMBOL_ELEMENTS;
        uint8_t wide = (element < BC_MATCH_CHAR_ELEMENTS) &&
            (0U != (masks[symbol] & (1U << (BC_MATCH_CHAR_TOP_BIT - element))));
        p_width[target] = wide ? (uint16_t)ratio : BC_MATCH_NARROW_X16;
    }
    uint32_t cursor = 0U;
    uint32_t peak = 0U;
    for (uint32_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        if (((i % 2U) == 0U) && (p_width[i] > BC_MATCH_NARROW_X16))
        {
            p_centres[peak] = cursor + p_width[i] / 2U;
            ++peak;
        }
        cursor += p_width[i];
    }
}

/** @brief Find the anchor peaks above the noise threshold. */
static void match_peaks (void)
{
    uint32_t threshold = g_maximum * g_anchor_percent[g_level] /
                         BC_MATCH_PERCENT;
    if (threshold < g_noise * BC_MATCH_NOISE_FACTOR)
    {
        threshold = g_noise * BC_MATCH_NOISE_FACTOR;
    }
    g_duplicate_level = (g_level > 0U) &&
                        (threshold == g_previous_threshold);
    g_previous_threshold = threshold;
    uint8_t active = 0U;
    uint32_t start = 0U;
    uint32_t last = 0U;
    uint64_t weighted = 0U;
    uint64_t weights = 0U;
    g_peak_count = 0U;
    g_peak_overflow = 0U;
    for (uint32_t i = 0U; i <= g_count; ++i)
    {
        uint32_t position = last;
        int32_t signal = 0;
        if (i < g_count)
        {
            match_fetch(i, &position, &signal);
        }
        if ((i < g_count) && (signal > (int32_t)threshold))
        {
            if (0U == active)
            {
                active = 1U;
                start = position;
                weighted = 0U;
                weights = 0U;
            }
            uint32_t weight = (uint32_t)signal - threshold;
            /* Include spatial weight so dwell time does not move a peak. */
            uint32_t step = (position > last) ? position - last : 0U;
            weighted += (uint64_t)position * weight * step;
            weights += (uint64_t)weight * step;
            last = position;
        }
        else if (0U != active)
        {
            if ((last - start >= BC_CAL_MIN_RUN_UNITS) && (0U != weights))
            {
                if (g_peak_count < BC_MATCH_PEAKS)
                {
                    g_peaks[g_peak_count].centre =
                        (uint32_t)(weighted / weights);
                    g_peaks[g_peak_count].width = last - start;
                    ++g_peak_count;
                }
                else
                {
                    g_peak_overflow = 1U;
                }
            }
            active = 0U;
        }
        if (i < g_count)
        {
            last = position;
        }
    }
    for (uint32_t i = 0U; i < BC_CODE_LETTERS; ++i)
    {
        g_level_score[i] = BC_MATCH_BAD_SCORE;
    }
#ifndef B1_BARCODE_TEST /* quiet barcode test log */
    tm_printf((UB *)"[BC MATCH ANCHORS] level=%u percent=%u threshold=%u "
                      "peaks=%u overflow=%u duplicate=%u\n",
              (UINT)g_level, (UINT)g_anchor_percent[g_level],
              (UINT)threshold, (UINT)g_peak_count,
              (UINT)g_peak_overflow, (UINT)g_duplicate_level);
#endif
}

/** @brief Fit scale and offset from the anchor peaks to a letter. */
static uint8_t match_geometry (uint32_t first, uint32_t const * p_centres,
                               uint16_t const * p_width,
                               bc_match_model_t * p_model)
{
    int64_t sx = 0;
    int64_t sy = 0;
    int64_t xx = 0;
    int64_t xy = 0;
    for (uint32_t i = 0U; i < BC_MATCH_ANCHORS; ++i)
    {
        int64_t x = p_centres[i];
        int64_t y = g_peaks[first + i].centre;
        sx += x;
        sy += y;
        xx += x * x;
        xy += x * y;
    }
    int64_t denominator = BC_MATCH_ANCHORS_I * xx - sx * sx;
    int64_t numerator = BC_MATCH_ANCHORS_I * xy - sx * sy;
    if ((denominator <= 0) || (numerator <= 0))
    {
        return 0U;
    }
    p_model->scale_q8 = (int32_t)(numerator * BC_MATCH_Q8_ONE_I /
                                  denominator);
    p_model->offset_q8 = (int32_t)((sy * BC_MATCH_Q8_ONE_I -
        p_model->scale_q8 * sx) / BC_MATCH_ANCHORS_I);
    int32_t narrow = p_model->scale_q8 * BC_MATCH_NARROW_X16_I;
    if (narrow <= 0)
    {
        return 0U;
    }
    uint32_t total = 0U;
    for (uint32_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        total += p_width[i];
    }
    int64_t span_q8 = (int64_t)p_model->scale_q8 * total;
    int64_t end_q8 = p_model->offset_q8 + span_q8;
    if ((span_q8 < BC_CAL_MIN_BARCODE_UNITS * BC_MATCH_Q8_ONE) ||
        (span_q8 > BC_CAL_MAX_BARCODE_UNITS * BC_MATCH_Q8_ONE) ||
        (p_model->offset_q8 < (int32_t)(BC_CAL_LEADING_UNITS *
                                         BC_MATCH_Q8_ONE)) ||
        (end_q8 + BC_CAL_TRAILING_UNITS * BC_MATCH_Q8_ONE >
         g_extent * BC_MATCH_Q8_ONE))
    {
        ++g_clearance_reject;
        return 0U;
    }
    /* Other strong peaks inside the predicted frame contradict its layout. */
    if (((first > 0U) &&
         ((int64_t)g_peaks[first - 1U].centre * BC_MATCH_Q8_ONE_I >
          p_model->offset_q8)) ||
        ((first + BC_MATCH_ANCHORS < g_peak_count) &&
         ((int64_t)g_peaks[first + BC_MATCH_ANCHORS].centre *
          BC_MATCH_Q8_ONE_I < end_q8)))
    {
        ++g_geometry_reject;
        return 0U;
    }
    uint64_t error = 0U;
    for (uint32_t i = 0U; i < BC_MATCH_ANCHORS; ++i)
    {
        int64_t residual =
            (int64_t)g_peaks[first + i].centre * BC_MATCH_Q8_ONE_I -
            (p_model->offset_q8 + (int64_t)p_model->scale_q8 * p_centres[i]);
        error += (uint64_t)((residual < 0) ? -residual : residual);
    }
    p_model->geometry = (uint16_t)(error * BC_MATCH_PERMILLE /
        (BC_MATCH_ANCHORS * (uint32_t)narrow));
    if (p_model->geometry > BC_MATCH_MAX_GEOMETRY)
    {
        ++g_geometry_reject;
        return 0U;
    }
    return 1U;
}

/** @brief Score a letter against the capture at a fitted geometry. */
static int32_t match_features (uint8_t letter, uint16_t const * p_width,
                                bc_match_model_t * p_model, uint8_t log)
{
    uint32_t cursor = 0U;
    uint32_t positive = 0U;
    uint32_t star_first = 0U;
    uint32_t star_last = 0U;
    int32_t score = 0;
    int32_t radius = p_model->scale_q8 * BC_MATCH_NARROW_X16_I /
                     BC_MATCH_RADIUS_DIV;
    if (radius < BC_MATCH_Q8_ONE_I)
    {
        radius = BC_MATCH_Q8_ONE_I;
    }
    for (uint32_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        if (((i % 2U) == 0U) && (BC_MATCH_NARROW_X16 == p_width[i]))
        {
            int32_t centre = p_model->offset_q8 +
                p_model->scale_q8 * (int32_t)(cursor + BC_MATCH_HALF_NARROW);
            int32_t left = p_model->offset_q8 + p_model->scale_q8 *
                ((int32_t)cursor - ((i > 0U) ? p_width[i - 1U] / 2
                                    : BC_MATCH_HALF_NARROW_I));
            int32_t right = p_model->offset_q8 + p_model->scale_q8 *
                (int32_t)(cursor + BC_MATCH_NARROW_X16 +
                          ((i + 1U < BC_CODE_ELEMENTS) ? p_width[i + 1U] / 2U
                                                       : BC_MATCH_HALF_NARROW));
            int32_t black = 0;
            int32_t white_left = 0;
            int32_t white_right = 0;
            if ((0U == match_mean(centre, radius, &black)) ||
                (0U == match_mean(left, radius, &white_left)) ||
                (0U == match_mean(right, radius, &white_right)))
            {
                ++g_clearance_reject;
                return BC_MATCH_BAD_SCORE;
            }
            int32_t baseline = (int32_t)
                (((int64_t)white_left * (right - centre) +
                  (int64_t)white_right * (centre - left)) / (right - left));
            int32_t delta = black - baseline;
            int32_t evidence = delta * BC_MATCH_PERCENT_I / (int32_t)g_noise;
            /* Bound leverage of any one feature; wide-bar spill cannot
             * dominate all the other narrow bars by arbitrary amplitude.
             */
            if (evidence > BC_MATCH_EVIDENCE_CAP)
            {
                evidence = BC_MATCH_EVIDENCE_CAP;
            }
            if (evidence < -BC_MATCH_EVIDENCE_CAP)
            {
                evidence = -BC_MATCH_EVIDENCE_CAP;
            }
            score += evidence;
            if (evidence >= BC_MATCH_EVIDENCE_MIN)
            {
                ++positive;
                star_first += (i < BC_MATCH_CHAR_ELEMENTS) ? 1U : 0U;
                star_last += (i >= BC_MATCH_END_STAR_FIRST) ? 1U : 0U;
            }
            if (0U != log)
            {
                int32_t local_delta = delta;
                int32_t local_shift = 0;
                match_local_peak(centre, left, right, radius,
                                  p_model->scale_q8 * BC_MATCH_NARROW_X16_I,
                                  &local_delta, &local_shift);
                tm_printf((UB *)"[BC MATCH FEATURE] letter=%c rev=%u i=%u "
                                  "u=%u black=%d white=%d delta=%d z100=%d "
                                  "local_delta=%d local_shift_pct=%d\n",
                          (INT)('A' + letter), (UINT)p_model->reverse,
                          (UINT)i, (UINT)(centre / BC_MATCH_Q8_ONE_I),
                          (INT)black,
                          (INT)baseline, (INT)delta, (INT)evidence,
                          (INT)local_delta, (INT)local_shift);
            }
        }
        cursor += p_width[i];
    }
    p_model->positive = (uint8_t)positive;
    p_model->eligible = (positive >= BC_MATCH_MIN_POSITIVE) &&
                        (star_first > 0U) && (star_last > 0U);
    if ((positive < BC_MATCH_MIN_POSITIVE) || (0U == star_first) ||
        (0U == star_last))
    {
        ++g_weak_reject;
    }
    /* Keep weak competitors in the score ranking. Dropping them would
     * artificially inflate the winner's margin around an eligibility cut.
     */
    return score - (int32_t)p_model->geometry / BC_MATCH_GEOMETRY_DIV;
}

/** @brief Score one letter and direction against the capture. */
static void match_job (void)
{
    uint8_t letter = (uint8_t)(g_job / BC_MATCH_DIRECTIONS);
    uint8_t reverse = (uint8_t)(g_job % BC_MATCH_DIRECTIONS);
    for (uint32_t first = 0U; first + BC_MATCH_ANCHORS <= g_peak_count; ++first)
    {
        for (uint8_t ratio = BC_MATCH_RATIO_MIN; ratio <= BC_MATCH_RATIO_MAX;
             ratio += BC_MATCH_RATIO_STEP)
        {
            uint16_t widths[BC_CODE_ELEMENTS] = {0U};
            uint32_t centres[BC_MATCH_ANCHORS] = {0U};
            bc_match_model_t model = {0};
            match_pattern(letter, reverse, ratio, widths, centres);
            if (0U != match_geometry(first, centres, widths, &model))
            {
                int32_t origin = model.offset_q8;
                for (int32_t shift = -1; shift <= 1; ++shift)
                {
                    model.offset_q8 = origin +
                        shift * model.scale_q8 * BC_MATCH_NARROW_X16_I *
                        BC_MATCH_SHIFT_PCT / BC_MATCH_PERCENT_I;
                    model.ratio = ratio;
                    model.reverse = reverse;
                    uint64_t residual_sum = 0U;
                    for (uint32_t i = 0U; i < BC_MATCH_ANCHORS; ++i)
                    {
                        int64_t residual =
                            (int64_t)g_peaks[first + i].centre *
                            BC_MATCH_Q8_ONE_I -
                            (model.offset_q8 +
                             (int64_t)model.scale_q8 * centres[i]);
                        residual_sum += (uint64_t)
                            ((residual < 0) ? -residual : residual);
                    }
                    model.geometry = (uint16_t)(residual_sum *
                        BC_MATCH_PERMILLE / (BC_MATCH_ANCHORS *
                        (uint32_t)model.scale_q8 * BC_MATCH_NARROW_X16));
                    model.score = BC_MATCH_BAD_SCORE;
                    if (model.geometry <= BC_MATCH_MAX_GEOMETRY)
                    {
                        model.score = match_features(letter, widths,
                                                     &model, 0U);
                    }
                    if (model.score > g_level_score[letter])
                    {
                        g_level_score[letter] = model.score;
                    }
                    if (model.score > g_models[letter].score)
                    {
                        g_models[letter] = model;
                    }
                }
            }
        }
    }
    ++g_job;
}

/** @brief Return the index of the best score, skipping one entry. */
static uint32_t match_best (int32_t const * p_scores, uint32_t exclude)
{
    uint32_t best = BC_CODE_LETTERS;
    for (uint32_t i = 0U; i < BC_CODE_LETTERS; ++i)
    {
        if ((i != exclude) &&
            ((BC_CODE_LETTERS == best) || (p_scores[i] > p_scores[best])))
        {
            best = i;
        }
    }
    return best;
}

/** @brief Pick the best and runner-up letters and their margin. */
static void match_finish (void)
{
    int32_t scores[BC_CODE_LETTERS] = {0};
    for (uint32_t i = 0U; i < BC_CODE_LETTERS; ++i)
    {
        scores[i] = g_models[i].score;
    }
    uint32_t best = match_best(scores, BC_CODE_LETTERS);
    uint32_t runner = match_best(scores, best);
    int32_t margin = scores[best] -
        ((scores[runner] == BC_MATCH_BAD_SCORE) ? 0 : scores[runner]);
    g_result.letter = (scores[best] == BC_MATCH_BAD_SCORE) ?
                      BC_CODE_LETTER_INVALID : (uint8_t)best;
    g_result.reverse = g_models[best].reverse;
    g_result.score_z100 = scores[best];
    g_result.margin_z100 = margin;
    g_result.support = g_support[best];
    g_result.runner_letter = (scores[runner] == BC_MATCH_BAD_SCORE) ?
                             BC_CODE_LETTER_INVALID : (uint8_t)runner;
    g_result.runner_reverse = g_models[runner].reverse;
    g_result.runner_score_z100 = (scores[runner] == BC_MATCH_BAD_SCORE) ?
                                 0 : scores[runner];
    g_result.positive = g_models[best].positive;
    g_result.eligible = g_models[best].eligible;
    g_result.valid = (scores[best] >= APP_BARCODE_TEMPLATE_MIN_SCORE_Z100) &&
        (0U != g_models[best].eligible) &&
        (margin >= APP_BARCODE_TEMPLATE_MIN_MARGIN_Z100) &&
        (g_support[best] >= APP_BARCODE_TEMPLATE_MIN_SUPPORT);
    tm_printf((UB *)"[BC MATCH RESULT] valid=%u best=%c rev=%u score_z100=%d "
                      "runner=%c runner_z100=%d margin_z100=%d support=%u "
                      "positive=%u eligible=%u "
                      "geometry_reject=%u clearance_reject=%u weak_reject=%u\n",
              (UINT)g_result.valid,
              (INT)((scores[best] == BC_MATCH_BAD_SCORE) ? '?' : 'A' + best),
              (UINT)g_result.reverse, (INT)scores[best],
              (INT)((scores[runner] == BC_MATCH_BAD_SCORE) ?
                    '?' : 'A' + runner),
              (INT)scores[runner], (INT)margin, (UINT)g_support[best],
              (UINT)g_models[best].positive, (UINT)g_models[best].eligible,
              (UINT)g_geometry_reject, (UINT)g_clearance_reject,
              (UINT)g_weak_reject);
    for (uint32_t rank = 0U; rank < BC_MATCH_LOGGED_MODELS; ++rank)
    {
        uint32_t candidate = (0U == rank) ? best : runner;
        if (scores[candidate] != BC_MATCH_BAD_SCORE)
        {
            uint16_t widths[BC_CODE_ELEMENTS] = {0U};
            uint32_t centres[BC_MATCH_ANCHORS] = {0U};
            match_pattern((uint8_t)candidate, g_models[candidate].reverse,
                          g_models[candidate].ratio, widths, centres);
#ifndef B1_LIVE_COMMANDS
            tm_printf((UB *)"[BC MATCH MODEL] letter=%c offset_q8=%d "
                              "scale_q8=%d ratio_x16=%u geom_permille=%u\n",
                      (INT)('A' + candidate),
                      (INT)g_models[candidate].offset_q8,
                      (INT)g_models[candidate].scale_q8,
                      (UINT)g_models[candidate].ratio,
                      (UINT)g_models[candidate].geometry);
#endif
            (void)match_features((uint8_t)candidate, widths,
#ifdef B1_LIVE_COMMANDS
                                 &g_models[candidate], 0U);
#else
                                 &g_models[candidate], 1U);
#endif
        }
    }
    g_done = 1U;
#ifndef B1_BARCODE_TEST /* quiet barcode test log */
    tm_printf((UB *)"[BC MATCH COST] prefix=%u means=%u cached=%u reads=%u\n",
              (UINT)g_prefix_enabled, (UINT)g_mean_calls,
              (UINT)g_cached_means, (UINT)g_read_calls);
#endif
}

void bc_match_begin (uint32_t count, uint32_t noise,
                      bc_match_read_t read_sample, int32_t * p_prefix,
                      uint32_t prefix_capacity)
{
    g_read = read_sample;
    g_count = count;
    g_noise = (noise > BC_MATCH_MIN_NOISE) ? noise : BC_MATCH_MIN_NOISE;
    g_maximum = 0U;
    g_extent = 0U;
    g_level = 0U;
    g_previous_threshold = 0U;
    g_duplicate_level = 0U;
    g_job = 0U;
    g_done = 0U;
    g_geometry_reject = 0U;
    g_clearance_reject = 0U;
    g_weak_reject = 0U;
    g_result = (bc_match_result_t){
        .letter = BC_CODE_LETTER_INVALID,
        .runner_letter = BC_CODE_LETTER_INVALID
    };
    g_read_calls = 0U;
    g_mean_calls = 0U;
    g_cached_means = 0U;
    gp_prefix = p_prefix;
    g_prefix_enabled = (0 != p_prefix) && (prefix_capacity >= count / 2U + 1U);
    if ((0 == read_sample) || (count < BC_MATCH_MIN_SAMPLES))
    {
        g_done = 1U;
        return;
    }
    for (uint32_t i = 0U; i < BC_CODE_LETTERS; ++i)
    {
        g_models[i] = (bc_match_model_t){0};
        g_models[i].score = BC_MATCH_BAD_SCORE;
        g_support[i] = 0U;
    }
    int64_t area = 0;
    uint32_t previous_position = 0U;
    int32_t previous_signal = 0;
    for (uint32_t i = 0U; i < count; ++i)
    {
        uint32_t position = 0U;
        int32_t signal = 0;
        match_fetch(i, &position, &signal);
        if (i > 0U)
        {
            area += (int64_t)previous_signal * (position - previous_position);
        }
        if ((signal < 0) || (area < 0) || (area > INT32_MAX))
        {
            g_prefix_enabled = 0U;
        }
        if ((0U != g_prefix_enabled) && (0U == (i % 2U)))
        {
            gp_prefix[i / 2U] = (int32_t)area;
        }
        previous_position = position;
        previous_signal = signal;
        if (signal > (int32_t)g_maximum)
        {
            g_maximum = (uint32_t)signal;
        }
        g_extent = position;
    }
    match_peaks();
}

uint8_t bc_match_poll (void)
{
    for (uint32_t work = 0U;
         (work < BC_MATCH_JOBS_PER_POLL) && (0U == g_done); ++work)
    {
        if ((g_peak_count >= BC_MATCH_ANCHORS) && (0U == g_peak_overflow) &&
            (0U == g_duplicate_level))
        {
            match_job();
        }
        else
        {
            g_job = BC_MATCH_JOBS;
        }
        if (g_job >= BC_MATCH_JOBS)
        {
            uint32_t best = match_best(g_level_score, BC_CODE_LETTERS);
            uint32_t runner = match_best(g_level_score, best);
            if ((g_level_score[best] >= APP_BARCODE_TEMPLATE_MIN_SCORE_Z100) &&
                (g_level_score[best] > g_level_score[runner]))
            {
                ++g_support[best];
            }
            ++g_level;
            if (g_level >= BC_MATCH_LEVELS)
            {
                match_finish();
            }
            else
            {
                g_job = 0U;
                match_peaks();
            }
        }
    }
    return g_done;
}

/** @brief Return the latest template-match result. */
bc_match_result_t bc_match_result (void)
{
    return g_result;
}
#else
void bc_match_begin (uint32_t count, uint32_t noise,
                      bc_match_read_t read_sample, int32_t * p_prefix,
                      uint32_t prefix_capacity)
{
    (void)count;
    (void)noise;
    (void)read_sample;
    (void)p_prefix;
    (void)prefix_capacity;
}

uint8_t bc_match_poll (void)
{
    return 1U;
}

/** @brief Return the latest template-match result. */
bc_match_result_t bc_match_result (void)
{
    bc_match_result_t result = {
        .letter = BC_CODE_LETTER_INVALID,
        .runner_letter = BC_CODE_LETTER_INVALID
    };
    return result;
}
#endif

/*** end of file ***/

