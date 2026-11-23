/** @file bc_layout.c
 * @brief Wide-bar layout and blurred-model Code 39 decoder.
 *
 * Steps per distance variant (each poll does a bounded share):
 *   PEAKS   one pass: separated local peaks; sixth-tallest sets the level
 *   BARS    one pass: dark segments above half that level, centroids
 *   LAYOUT  every ordered choice of six bars x 26 letters x 2 directions:
 *           least-squares scale/offset, worst wide-bar error in modules
 *   PICK    best layout; refuse when another layout fits about as well
 *   INSIDE  per group letter: samples over the symbol, sample stride
 *   MODEL   per group letter: 3 scales x 11 shifts of a blurred *X* model
 */
#include "bc_layout.h"
#include "bc_code.h"
#ifdef BC_LAYOUT_DEBUG
#include <stdio.h>
#endif

#define BC_LAYOUT_STAR 0x094U
#ifndef BC_LAYOUT_RATIO
#define BC_LAYOUT_RATIO 3.0F          /* wide:narrow of the prints */
#endif
#define BC_LAYOUT_TOTAL 47.0F         /* modules in *X* */
#define BC_LAYOUT_ELEMENTS 29U
#define BC_LAYOUT_BLACKS 15U
#define BC_LAYOUT_WIDE 6U
#define BC_LAYOUT_SEP 60.0F           /* merge peaks closer than this */
#define BC_LAYOUT_MAX_RAW 96U         /* dark segments considered */
#define BC_LAYOUT_MAX_BARS 10U        /* candidates kept for the search */
/* *X* has 15 bars. Every correct replayed read had 6-11 dark segments; a
 * small T read as F had 92 (noise fitted at the wrong scale). Refuse
 * captures this messy rather than decode them. */
#define BC_LAYOUT_MAX_CLEAN_BARS 24U
#define BC_LAYOUT_FIT_TOL 0.8F        /* modules */
#define BC_LAYOUT_CLASS_MARGIN 0.25F  /* modules */
/* Narrow module, L+R units. The course uses any print size, unstated
 * range; the smallest seen are ~10 cm (~37 units, ~17 units/mm). Accept
 * one continuous range: 30 (~8 cm) to 100 (~27 cm, the capture stage's
 * 4800-unit limit / 47). Two bands (33-41, 66-80) refused every
 * mid-size print. Replay of all passes with 25-100: no wrong reads. */
#define BC_LAYOUT_MODULE_MIN 30.0F
#define BC_LAYOUT_MODULE_MAX 100.0F
#ifndef BC_LAYOUT_BLUR
#define BC_LAYOUT_BLUR 0.7F           /* model blur sigma, modules */
#endif
#define BC_LAYOUT_EDGE 3.0F           /* model margin, modules */
#define BC_LAYOUT_SAMPLES 512U        /* per model fit */
#define BC_LAYOUT_SCALES 3U
#define BC_LAYOUT_SHIFTS 11U
#define BC_LAYOUT_FITS (BC_LAYOUT_SCALES * BC_LAYOUT_SHIFTS)
#define BC_LAYOUT_MIN_POINTS 20U
/* Sensor 70 mm left of centre, 110 mm track: k = 70 / 110. */
#define BC_LAYOUT_SENSOR_K 0.636364F
#define BC_LAYOUT_COMBOS_PER_POLL 30U
#define BC_LAYOUT_FITS_PER_POLL 8U
#define BC_LAYOUT_NONE 255U
#define BC_LAYOUT_PHI_STEPS 32.0F     /* table entries per unit z */
#define BC_LAYOUT_PHI_COUNT 145U
#define BC_LAYOUT_BIG_ERR 1.0e9F
/* Blurred model tabulated per letter: x from -3 to 50 modules. */
#define BC_LAYOUT_GRID_STEPS 16.0F    /* points per module */
#define BC_LAYOUT_GRID 850U
#define BC_LAYOUT_Q 16384.0F          /* model table scale (Q14) */
/* Sampled points kept per letter: the fit window plus room for every
 * scale and shift tried (1.5% of 50 modules + 0.5 < 1.5 modules). */
#define BC_LAYOUT_CACHE 768U
#define BC_LAYOUT_CACHE_MARGIN 1.5F
/* Local comparison of the top two group letters (see layout_local). */
#define BC_LAYOUT_LOCAL_DIFF 0.25F    /* of one bar's model contrast */
#define BC_LAYOUT_LOCAL_MIN 6U        /* samples where the models differ */
/* Local misfit ratio - 1 needed to accept. Whole-symbol gaps had been
 * 0-1.6%, which is how a big P was read as F. Replay of 81 correct reads:
 * lowest 7.1%, 9.4%, 9.7%; a big T scanned backwards was read as G at
 * 6.6%. 10% refuses that and three correct reads, never a wrong one. */
#define BC_LAYOUT_LOCAL_MARGIN 0.10F
/* Local gap (per mille) the sensor-distance read needs to overrule a
 * centre-distance tie (see layout_combine). */
#ifndef BC_LAYOUT_OVERRULE_PERMILLE
#define BC_LAYOUT_OVERRULE_PERMILLE 250U
#endif

typedef enum
{
    LAYOUT_IDLE,
    LAYOUT_PEAKS,
    LAYOUT_BARS,
    LAYOUT_SEARCH,
    LAYOUT_PICK,
    LAYOUT_INSIDE,
    LAYOUT_MODEL,
    LAYOUT_DONE
} layout_phase_t;

typedef struct
{
    float err;
    float a;
    float b;
} layout_fit_t;

/* Normal CDF at z = k / 32 for k = 0..144 (z 0..4.5). */
static float const g_phi[BC_LAYOUT_PHI_COUNT] = {
    0.500000F, 0.512465F, 0.524918F, 0.537346F, 0.549738F, 0.562082F,
    0.574366F, 0.586578F, 0.598706F, 0.610741F, 0.622670F, 0.634483F,
    0.646170F, 0.657721F, 0.669126F, 0.680376F, 0.691462F, 0.702377F,
    0.713112F, 0.723660F, 0.734014F, 0.744168F, 0.754116F, 0.763853F,
    0.773373F, 0.782672F, 0.791748F, 0.800595F, 0.809213F, 0.817598F,
    0.825749F, 0.833665F, 0.841345F, 0.848788F, 0.855996F, 0.862968F,
    0.869705F, 0.876211F, 0.882485F, 0.888530F, 0.894350F, 0.899947F,
    0.905324F, 0.910485F, 0.915434F, 0.920175F, 0.924712F, 0.929050F,
    0.933193F, 0.937146F, 0.940915F, 0.944504F, 0.947919F, 0.951164F,
    0.954246F, 0.957170F, 0.959941F, 0.962564F, 0.965046F, 0.967390F,
    0.969604F, 0.971691F, 0.973658F, 0.975509F, 0.977250F, 0.978885F,
    0.980420F, 0.981859F, 0.983207F, 0.984468F, 0.985647F, 0.986748F,
    0.987776F, 0.988733F, 0.989625F, 0.990455F, 0.991226F, 0.991941F,
    0.992605F, 0.993221F, 0.993790F, 0.994317F, 0.994804F, 0.995253F,
    0.995668F, 0.996049F, 0.996401F, 0.996724F, 0.997020F, 0.997292F,
    0.997542F, 0.997771F, 0.997980F, 0.998171F, 0.998346F, 0.998505F,
    0.998650F, 0.998782F, 0.998903F, 0.999012F, 0.999111F, 0.999201F,
    0.999282F, 0.999356F, 0.999423F, 0.999483F, 0.999538F, 0.999587F,
    0.999631F, 0.999671F, 0.999706F, 0.999739F, 0.999767F, 0.999793F,
    0.999816F, 0.999837F, 0.999856F, 0.999872F, 0.999887F, 0.999900F,
    0.999912F, 0.999922F, 0.999931F, 0.999939F, 0.999947F, 0.999953F,
    0.999959F, 0.999964F, 0.999968F, 0.999972F, 0.999976F, 0.999979F,
    0.999981F, 0.999984F, 0.999986F, 0.999988F, 0.999989F, 0.999991F,
    0.999992F, 0.999993F, 0.999994F, 0.999995F, 0.999995F, 0.999996F,
    0.999997F
};

static float const g_scales[BC_LAYOUT_SCALES] = {0.985F, 1.0F, 1.015F};

static layout_phase_t g_phase = LAYOUT_IDLE;
static bc_layout_read_t g_read;
static uint32_t g_count;
static uint8_t g_has_left;
static uint8_t g_variant;
static uint8_t g_tables_ready;

static float g_wide[26][BC_LAYOUT_WIDE];
static float g_wide_mean[26];
static float g_wide_sxx[26];
/* Wide-bar centres along the scan for letter x direction (index
 * 2 * letter + reverse), shifted to start at 0. Equal shapes cannot be
 * told apart by wide bars alone (Z backwards = G forwards). */
static float g_shape[52][BC_LAYOUT_WIDE];
static float g_black[BC_LAYOUT_BLACKS][2];
static int16_t g_grid[BC_LAYOUT_GRID];
static float g_cache_pos[BC_LAYOUT_CACHE];
static int16_t g_cache_sig[BC_LAYOUT_CACHE];
static uint32_t g_cache_count;

static int32_t g_top[BC_LAYOUT_WIDE];
static uint32_t g_peaks;

static float g_raw_centre[BC_LAYOUT_MAX_RAW];
static int32_t g_raw_height[BC_LAYOUT_MAX_RAW];
static uint32_t g_raw_count;
static uint8_t g_raw_overflow;
static float g_bar[BC_LAYOUT_MAX_BARS];
static uint32_t g_bars;

static uint32_t g_pick[BC_LAYOUT_WIDE];
static uint8_t g_combos_done;
static layout_fit_t g_fit[26][2];

static uint8_t g_group[52];             /* 2 * letter + reverse */
static uint32_t g_group_count;
static uint32_t g_group_index;
static uint8_t g_best_reverse;
static float g_best_err;
static uint32_t g_stride;
static uint32_t g_fit_index;
static double g_mse[52];
static uint8_t g_mse_valid[52];
/* Best model fit per candidate: scale/shift index, gain and offset. */
static uint8_t g_mfit[52];
static double g_gain[52];
static double g_icpt[52];
static double g_var[52];               /* signal variance in that fit */
/* Second model table: the runner-up during the local comparison. */
static int16_t g_grid2[BC_LAYOUT_GRID];

static bc_layout_result_t g_variant_result[BC_LAYOUT_VARIANTS];
static bc_layout_result_t g_result;

/* ------------------------------------------------------------ geometry */

static float layout_abs (float value)
{
    return (value < 0.0F) ? -value : value;
}

static float layout_phi (float z)
{
    float magnitude = layout_abs(z) * BC_LAYOUT_PHI_STEPS;
    float value = 1.0F;
    if (magnitude < (float)(BC_LAYOUT_PHI_COUNT - 1U))
    {
        uint32_t index = (uint32_t)magnitude;
        float part = magnitude - (float)index;
        value = g_phi[index] + part * (g_phi[index + 1U] - g_phi[index]);
    }
    return (z < 0.0F) ? 1.0F - value : value;
}

/** Element widths and black flags of *letter*, in module units. */
static void layout_elements (uint8_t letter, float * p_x0, float * p_x1,
                             uint8_t * p_black, uint8_t * p_wide)
{
    bc_code_pattern_t pattern = {0U};
    uint16_t patterns[3];
    float x = 0.0F;
    uint32_t out = 0U;

    (void)bc_code_get(letter, &pattern);
    patterns[0] = BC_LAYOUT_STAR;
    patterns[1] = pattern.element_pattern;
    patterns[2] = BC_LAYOUT_STAR;
    for (uint32_t c = 0U; c < 3U; ++c)
    {
        if (c > 0U)
        {
            x += 1.0F; /* inter-character gap */
        }
        for (uint32_t i = 0U; i < 9U; ++i)
        {
            uint8_t wide = (uint8_t)((patterns[c] >> (8U - i)) & 1U);
            float width = (0U != wide) ? BC_LAYOUT_RATIO : 1.0F;
            p_x0[out] = x;
            p_x1[out] = x + width;
            p_black[out] = (uint8_t)((i % 2U) == 0U);
            p_wide[out] = wide;
            x += width;
            ++out;
        }
    }
}

static void layout_tables (void)
{
    float x0[BC_LAYOUT_ELEMENTS];
    float x1[BC_LAYOUT_ELEMENTS];
    uint8_t black[BC_LAYOUT_ELEMENTS];
    uint8_t wide[BC_LAYOUT_ELEMENTS];

    for (uint8_t letter = 0U; letter < 26U; ++letter)
    {
        uint32_t n = 0U;
        float sum = 0.0F;
        float sxx = 0.0F;
        layout_elements(letter, x0, x1, black, wide);
        for (uint32_t i = 0U; i < BC_LAYOUT_ELEMENTS; ++i)
        {
            if ((0U != black[i]) && (0U != wide[i]) &&
                (n < BC_LAYOUT_WIDE))
            {
                g_wide[letter][n] = (x0[i] + x1[i]) / 2.0F;
                sum += g_wide[letter][n];
                ++n;
            }
        }
        g_wide_mean[letter] = sum / (float)BC_LAYOUT_WIDE;
        for (uint32_t k = 0U; k < BC_LAYOUT_WIDE; ++k)
        {
            float d = g_wide[letter][k] - g_wide_mean[letter];
            sxx += d * d;
        }
        g_wide_sxx[letter] = sxx;
        for (uint32_t k = 0U; k < BC_LAYOUT_WIDE; ++k)
        {
            /* Reversed: mirrored centres, ascending along the scan. */
            uint32_t const back = BC_LAYOUT_WIDE - 1U - k;
            g_shape[2U * letter][k] = g_wide[letter][k] - g_wide[letter][0];
            g_shape[2U * letter + 1U][k] =
                (BC_LAYOUT_TOTAL - g_wide[letter][back]) -
                (BC_LAYOUT_TOTAL - g_wide[letter][BC_LAYOUT_WIDE - 1U]);
        }
    }
    g_tables_ready = 1U;
}

/* ------------------------------------------------------------- samples */

static float layout_position (uint32_t units, uint32_t left)
{
    float u = (float)units;
    float position = u;
    if (0U != g_variant)
    {
        /* Travel at the sensor: (L+R) - 2k(R-L), with R = u - L. */
        position = u - 2.0F * BC_LAYOUT_SENSOR_K *
                   (u - 2.0F * (float)left);
    }
    return position;
}

/** Read sample i; *p_left carries the running left-wheel sum. */
static void layout_sample (uint32_t index, uint32_t * p_left,
                           float * p_position, int32_t * p_signal)
{
    uint32_t units = 0U;
    uint32_t step = 0U;
    int32_t signal = 0;
    g_read(index, &units, &step, &signal);
    if (index > 0U)
    {
        *p_left += step;
    }
    *p_position = layout_position(units, *p_left);
    *p_signal = (signal > 0) ? signal : 0;
}

static void layout_top_insert (int32_t height)
{
    ++g_peaks;
    for (uint32_t k = 0U; k < BC_LAYOUT_WIDE; ++k)
    {
        if (height > g_top[k])
        {
            int32_t moved = g_top[k];
            g_top[k] = height;
            height = moved;
        }
    }
}

static void layout_peaks (void)
{
    uint32_t left = 0U;
    int32_t s_prev = 0;
    int32_t s_cur = 0;
    float p_cur = 0.0F;
    uint8_t have = 0U;
    float cluster_pos = 0.0F;
    int32_t cluster_height = 0;

    g_peaks = 0U;
    for (uint32_t k = 0U; k < BC_LAYOUT_WIDE; ++k)
    {
        g_top[k] = -1;
    }
    for (uint32_t i = 0U; i < g_count; ++i)
    {
        float p_next = 0.0F;
        int32_t s_next = 0;
        layout_sample(i, &left, &p_next, &s_next);
        if ((i >= 2U) && (s_cur >= s_prev) && (s_cur > s_next))
        {
            if ((0U != have) && ((p_cur - cluster_pos) < BC_LAYOUT_SEP))
            {
                if (s_cur > cluster_height)
                {
                    cluster_pos = p_cur;
                    cluster_height = s_cur;
                }
            }
            else
            {
                if (0U != have)
                {
                    layout_top_insert(cluster_height);
                }
                have = 1U;
                cluster_pos = p_cur;
                cluster_height = s_cur;
            }
        }
        s_prev = s_cur;
        s_cur = s_next;
        p_cur = p_next;
    }
    if (0U != have)
    {
        layout_top_insert(cluster_height);
    }
}

static void layout_keep_tallest (void)
{
    uint8_t chosen[BC_LAYOUT_MAX_RAW] = {0U};
    uint32_t n = 0U;

    for (uint32_t round = 0U; round < BC_LAYOUT_MAX_BARS; ++round)
    {
        uint32_t best = BC_LAYOUT_MAX_RAW;
        for (uint32_t i = 0U; i < g_raw_count; ++i)
        {
            if ((0U == chosen[i]) &&
                ((best == BC_LAYOUT_MAX_RAW) ||
                 (g_raw_height[i] > g_raw_height[best])))
            {
                best = i;
            }
        }
        chosen[best] = 1U;
    }
    for (uint32_t i = 0U; i < g_raw_count; ++i)
    {
        if (0U != chosen[i])
        {
            /* Insertion by centre, as the replay sorts kept bars. */
            uint32_t j = n;
            while ((j > 0U) && (g_bar[j - 1U] > g_raw_centre[i]))
            {
                g_bar[j] = g_bar[j - 1U];
                --j;
            }
            g_bar[j] = g_raw_centre[i];
            ++n;
        }
    }
    g_bars = n;
}

static void layout_bars (void)
{
    float const ref = (float)g_top[BC_LAYOUT_WIDE - 1U];
    float const clip_at = 1.5F * ref;
    float const on = ref / 2.0F;
    float const off = ref * 3.0F / 8.0F;
    uint32_t left = 0U;
    uint8_t black = 0U;
    double sum_pw = 0.0;
    double sum_w = 0.0;
    int32_t height = 0;

    g_raw_count = 0U;
    g_raw_overflow = 0U;
    for (uint32_t i = 0U; i < g_count; ++i)
    {
        float position = 0.0F;
        int32_t signal = 0;
        float clipped;
        uint8_t opened = 0U;
        layout_sample(i, &left, &position, &signal);
        clipped = ((float)signal < clip_at) ? (float)signal : clip_at;
        if ((0U == black) && (clipped >= on))
        {
            black = 1U;
            opened = 1U;
            sum_pw = 0.0;
            sum_w = 0.0;
            height = 0;
        }
        if (0U != black)
        {
            sum_pw += (double)position * (double)clipped;
            sum_w += (double)clipped;
            if (signal > height)
            {
                height = signal;
            }
        }
        /* A sample may open a bar or close one, not both (replay order). */
        if ((0U != black) && (0U == opened) &&
            ((clipped <= off) || (i + 1U == g_count)))
        {
            black = 0U;
            if (g_raw_count < BC_LAYOUT_MAX_RAW)
            {
                g_raw_centre[g_raw_count] = (float)(sum_pw /
                    ((sum_w > 0.0) ? sum_w : 1.0));
                g_raw_height[g_raw_count] = height;
                ++g_raw_count;
            }
            else
            {
                g_raw_overflow = 1U;
            }
        }
    }
    if (g_raw_count > BC_LAYOUT_MAX_BARS)
    {
        layout_keep_tallest();
    }
    else
    {
        for (uint32_t i = 0U; i < g_raw_count; ++i)
        {
            g_bar[i] = g_raw_centre[i];
        }
        g_bars = g_raw_count;
    }
}

/* -------------------------------------------------------- wide layout */

static uint8_t layout_module_ok (float a)
{
    return (uint8_t)((a >= BC_LAYOUT_MODULE_MIN) &&
                     (a <= BC_LAYOUT_MODULE_MAX));
}

static void layout_try_combo (void)
{
    for (uint8_t reverse = 0U; reverse < 2U; ++reverse)
    {
        float cs[BC_LAYOUT_WIDE];
        float c_mean = 0.0F;
        for (uint32_t k = 0U; k < BC_LAYOUT_WIDE; ++k)
        {
            uint32_t at = (0U != reverse) ? (BC_LAYOUT_WIDE - 1U - k) : k;
            cs[k] = g_bar[g_pick[at]];
            c_mean += cs[k];
        }
        c_mean /= (float)BC_LAYOUT_WIDE;
        for (uint8_t letter = 0U; letter < 26U; ++letter)
        {
            float m_mean = (0U != reverse) ?
                BC_LAYOUT_TOTAL - g_wide_mean[letter] : g_wide_mean[letter];
            float sxy = 0.0F;
            float a;
            float b;
            for (uint32_t k = 0U; k < BC_LAYOUT_WIDE; ++k)
            {
                float m = (0U != reverse) ?
                    BC_LAYOUT_TOTAL - g_wide[letter][k] : g_wide[letter][k];
                sxy += (m - m_mean) * (cs[k] - c_mean);
            }
            a = sxy / g_wide_sxx[letter];
            b = c_mean - a * m_mean;
            if (0U != layout_module_ok(a))
            {
                float err = 0.0F;
                layout_fit_t * const p_fit = &g_fit[letter][reverse];
                for (uint32_t k = 0U; k < BC_LAYOUT_WIDE; ++k)
                {
                    float m = (0U != reverse) ?
                        BC_LAYOUT_TOTAL - g_wide[letter][k] :
                        g_wide[letter][k];
                    float e = layout_abs((cs[k] - b) / a - m);
                    if (e > err)
                    {
                        err = e;
                    }
                }
                if ((err < p_fit->err) ||
                    ((err == p_fit->err) && (a < p_fit->a)))
                {
                    p_fit->err = err;
                    p_fit->a = a;
                    p_fit->b = b;
                }
            }
        }
    }
}

/** Advance g_pick to the next ordered six of g_bars; 0 when exhausted. */
static uint8_t layout_next_combo (void)
{
    uint32_t k = BC_LAYOUT_WIDE;
    uint8_t found = 0U;
    while ((k > 0U) && (0U == found))
    {
        --k;
        if (g_pick[k] < g_bars - BC_LAYOUT_WIDE + k)
        {
            found = 1U;
        }
    }
    if (0U != found)
    {
        ++g_pick[k];
        for (uint32_t j = k + 1U; j < BC_LAYOUT_WIDE; ++j)
        {
            g_pick[j] = g_pick[j - 1U] + 1U;
        }
    }
    return found;
}

static void layout_search (void)
{
    for (uint32_t n = 0U;
         (n < BC_LAYOUT_COMBOS_PER_POLL) && (0U == g_combos_done); ++n)
    {
        layout_try_combo();
        if (0U == layout_next_combo())
        {
            g_combos_done = 1U;
        }
    }
}

static uint8_t layout_key_differs (uint32_t candidate, uint32_t best)
{
    uint8_t differs = 0U;
    for (uint32_t k = 0U; k < BC_LAYOUT_WIDE; ++k)
    {
        if (layout_abs(g_shape[candidate][k] - g_shape[best][k]) > 0.001F)
        {
            differs = 1U;
        }
    }
    return differs;
}

/** Choose the wide-bar layout; fill the group or record a refusal. */
static void layout_pick (bc_layout_result_t * p_out)
{
    uint8_t best_letter = BC_LAYOUT_NONE;
    g_best_err = BC_LAYOUT_BIG_ERR;
    for (uint8_t letter = 0U; letter < 26U; ++letter)
    {
        for (uint8_t reverse = 0U; reverse < 2U; ++reverse)
        {
            float err = g_fit[letter][reverse].err;
            if (err < g_best_err)
            {
                g_best_err = err;
                best_letter = letter;
                g_best_reverse = reverse;
            }
        }
    }
#ifdef BC_LAYOUT_DEBUG
    /* Host replay only: the five best wide-bar layouts. */
    {
        uint8_t shown[52] = {0U};
        printf("    [v%u layouts]", (unsigned)g_variant);
        for (uint32_t r = 0U; r < 5U; ++r)
        {
            uint32_t pick = 52U;
            for (uint32_t c = 0U; c < 52U; ++c)
            {
                if ((0U == shown[c]) && ((52U == pick) ||
                    (g_fit[c / 2U][c % 2U].err <
                     g_fit[pick / 2U][pick % 2U].err)))
                {
                    pick = c;
                }
            }
            shown[pick] = 1U;
            printf(" %c%s=%.2f", 'A' + (int)(pick / 2U),
                   (pick % 2U) ? "r" : "", g_fit[pick / 2U][pick % 2U].err);
        }
        printf("\n");
    }
#endif
    g_group_count = 0U;
    if ((BC_LAYOUT_NONE == best_letter) || (g_best_err > BC_LAYOUT_FIT_TOL))
    {
        p_out->reason = BC_LAYOUT_NO_FIT;
        p_out->err_centi = (g_best_err < 600.0F) ?
                           (uint16_t)(g_best_err * 100.0F) : 60000U;
    }
    else
    {
        float other_err = BC_LAYOUT_BIG_ERR;
        p_out->err_centi = (uint16_t)(g_best_err * 100.0F);
        p_out->module_x10 = (uint16_t)
            (g_fit[best_letter][g_best_reverse].a * 10.0F);
        for (uint8_t letter = 0U; letter < 26U; ++letter)
        {
            for (uint8_t reverse = 0U; reverse < 2U; ++reverse)
            {
                float err = g_fit[letter][reverse].err;
                if ((0U != layout_key_differs(2U * letter + reverse,
                                              2U * best_letter +
                                              g_best_reverse)) &&
                    (err <= g_best_err + BC_LAYOUT_CLASS_MARGIN) &&
                    (err < other_err))
                {
                    other_err = err;
                    p_out->other = letter;
                }
            }
        }
        if (BC_LAYOUT_NONE != p_out->other)
        {
            p_out->reason = BC_LAYOUT_AMBIGUOUS;
        }
        else
        {
            /* Same wide-bar shape, either direction: the model decides. */
            for (uint32_t c = 0U; c < 52U; ++c)
            {
                if ((0U == layout_key_differs(c, 2U * best_letter +
                                              g_best_reverse)) &&
                    (g_fit[c / 2U][c % 2U].err < BC_LAYOUT_BIG_ERR))
                {
                    g_group[g_group_count] = (uint8_t)c;
                    g_mse_valid[c] = 0U;
                    ++g_group_count;
                }
            }
        }
    }
}

/* ---------------------------------------------------- blurred model */

static void layout_load_blacks (uint8_t letter)
{
    float x0[BC_LAYOUT_ELEMENTS];
    float x1[BC_LAYOUT_ELEMENTS];
    uint8_t black[BC_LAYOUT_ELEMENTS];
    uint8_t wide[BC_LAYOUT_ELEMENTS];
    uint32_t n = 0U;
    layout_elements(letter, x0, x1, black, wide);
    for (uint32_t i = 0U; i < BC_LAYOUT_ELEMENTS; ++i)
    {
        if ((0U != black[i]) && (n < BC_LAYOUT_BLACKS))
        {
            g_black[n][0] = x0[i];
            g_black[n][1] = x1[i];
            ++n;
        }
    }
}

static float layout_model_value (float x)
{
    float value = 0.0F;
    for (uint32_t k = 0U; k < BC_LAYOUT_BLACKS; ++k)
    {
        float x0 = g_black[k][0];
        float x1 = g_black[k][1];
        if ((x > x0 - BC_LAYOUT_EDGE) && (x < x1 + BC_LAYOUT_EDGE))
        {
            value += layout_phi((x1 - x) / BC_LAYOUT_BLUR) -
                     layout_phi((x0 - x) / BC_LAYOUT_BLUR);
        }
    }
    return value;
}

/** Tabulate the blurred model of letter into p_grid (Q14). */
static void layout_build_grid (uint8_t letter, int16_t * p_grid)
{
    layout_load_blacks(letter);
    for (uint32_t k = 0U; k < BC_LAYOUT_GRID; ++k)
    {
        float const value = layout_model_value(-BC_LAYOUT_EDGE +
            (float)k / BC_LAYOUT_GRID_STEPS);
        p_grid[k] = (int16_t)(value * BC_LAYOUT_Q + 0.5F);
    }
}

/** Cache every stride-th sample near the symbol, placed by candidate's
 * wide-bar fit, so the model fits reuse them. */
static void layout_cache (uint8_t candidate)
{
    layout_fit_t const * const p_fit =
        &g_fit[candidate / 2U][candidate % 2U];
    uint32_t left = 0U;
    uint32_t inside = 0U;
    uint32_t phase = 0U;
    for (uint32_t i = 0U; i < g_count; ++i)
    {
        float position = 0.0F;
        int32_t signal = 0;
        float x;
        layout_sample(i, &left, &position, &signal);
        x = (position - p_fit->b) / p_fit->a;
        if ((x > -BC_LAYOUT_EDGE) && (x < BC_LAYOUT_TOTAL + BC_LAYOUT_EDGE))
        {
            ++inside;
        }
    }
    g_stride = (inside + BC_LAYOUT_SAMPLES - 1U) / BC_LAYOUT_SAMPLES;
    if (0U == g_stride)
    {
        g_stride = 1U;
    }
    /* Keep every stride-th sample near the symbol once; the 33 fits then
     * reuse them instead of rereading the whole capture. */
    left = 0U;
    g_cache_count = 0U;
    for (uint32_t i = 0U; i < g_count; ++i)
    {
        uint32_t units = 0U;
        uint32_t step = 0U;
        int32_t signal = 0;
        g_read(i, &units, &step, &signal);
        if (i > 0U)
        {
            left += step;
        }
        if (0U == phase)
        {
            float const position = layout_position(units, left);
            float const x = (position - p_fit->b) / p_fit->a;
            if ((x > -BC_LAYOUT_EDGE - BC_LAYOUT_CACHE_MARGIN) &&
                (x < BC_LAYOUT_TOTAL + BC_LAYOUT_EDGE +
                     BC_LAYOUT_CACHE_MARGIN) &&
                (g_cache_count < BC_LAYOUT_CACHE))
            {
                g_cache_pos[g_cache_count] = position;
                g_cache_sig[g_cache_count] = (int16_t)((signal < 0) ? 0 :
                    ((signal > 32767) ? 32767 : signal));
                ++g_cache_count;
            }
        }
        ++phase;
        if (phase >= g_stride)
        {
            phase = 0U;
        }
    }
}

static void layout_inside (void)
{
    uint8_t const candidate = g_group[g_group_index];
    layout_cache(candidate);
    g_fit_index = 0U;
    layout_build_grid((uint8_t)(candidate / 2U), g_grid);
}

/** Tabulated model (Q14) at x, -3 < x < 50 modules, linear between. */
static int32_t layout_table_value (int16_t const * p_grid, float x)
{
    int32_t const at = (int32_t)((x + BC_LAYOUT_EDGE) *
                                 BC_LAYOUT_GRID_STEPS * 256.0F);
    int32_t const index = at / 256;
    int32_t value = p_grid[BC_LAYOUT_GRID - 1U];
    if (at <= 0)
    {
        value = p_grid[0];
    }
    else if (index < (int32_t)BC_LAYOUT_GRID - 1)
    {
        int32_t const low = p_grid[index];
        int32_t const high = p_grid[index + 1];
        value = low + ((high - low) * (at - index * 256)) / 256;
    }
    else
    {
        /* Beyond the table: last point. */
    }
    return value;
}

static int32_t layout_grid_value (float x)
{
    return layout_table_value(g_grid, x);
}

/** One scale/shift fit of the current group letter (integer sums). */
static void layout_model_fit (void)
{
    uint8_t const candidate = g_group[g_group_index];
    uint8_t const reverse = (uint8_t)(candidate % 2U);
    layout_fit_t const * const p_fit = &g_fit[candidate / 2U][reverse];
    float const scale = g_scales[g_fit_index / BC_LAYOUT_SHIFTS];
    float const shift = 0.1F * ((float)(g_fit_index % BC_LAYOUT_SHIFTS) -
                                5.0F);
    float const inverse = 1.0F / (p_fit->a * scale);
    float const offset = p_fit->b;
    uint32_t n = 0U;
    int32_t sm = 0;
    int32_t sy = 0;
    int64_t smm = 0;
    int64_t smy = 0;
    int64_t syy = 0;

    for (uint32_t k = 0U; k < g_cache_count; ++k)
    {
        float x = (g_cache_pos[k] - offset) * inverse + shift;
        if ((x > -BC_LAYOUT_EDGE) && (x < BC_LAYOUT_TOTAL + BC_LAYOUT_EDGE))
        {
            int32_t m;
            int32_t const y = g_cache_sig[k];
            if (0U != reverse)
            {
                x = BC_LAYOUT_TOTAL - x;
            }
            m = layout_grid_value(x);
            sm += m;
            sy += y;
            smm += (int64_t)(m * m);
            smy += (int64_t)(m * y);
            syy += (int64_t)(y * y);
            ++n;
        }
    }
    if (n >= BC_LAYOUT_MIN_POINTS)
    {
        double const count = (double)n;
        double const dsm = (double)sm;
        double const dsy = (double)sy;
        double sxx = (double)smm - dsm * dsm / count;
        double sxy = (double)smy - dsm * dsy / count;
        double const syy_c = (double)syy - dsy * dsy / count;
        if (sxx <= 0.0)
        {
            sxx = 1.0e-9;
        }
        if (sxy / sxx > 0.0)
        {
            double const mse = (syy_c - (sxy / sxx) * sxy) / count;
            if ((0U == g_mse_valid[candidate]) ||
                (mse < g_mse[candidate]))
            {
                double const gain = sxy / sxx;
                g_var[candidate] = syy_c / count;
                g_mse[candidate] = mse;
                g_mse_valid[candidate] = 1U;
                g_mfit[candidate] = (uint8_t)g_fit_index;
                g_gain[candidate] = gain;
                g_icpt[candidate] = (dsy - gain * dsm) / count;
            }
        }
    }
    ++g_fit_index;
}

/** Model coordinate of a cached position under candidate's best fit. */
static float layout_model_x (uint8_t candidate, float position)
{
    layout_fit_t const * const p_fit =
        &g_fit[candidate / 2U][candidate % 2U];
    uint8_t const k = g_mfit[candidate];
    float const scale = g_scales[k / BC_LAYOUT_SHIFTS];
    float const shift = 0.1F * ((float)(k % BC_LAYOUT_SHIFTS) - 5.0F);
    float x = (position - p_fit->b) / (p_fit->a * scale) + shift;
    if (0U != (candidate % 2U))
    {
        x = BC_LAYOUT_TOTAL - x;
    }
    return x;
}

/** Misfit of two candidates only where their models differ.
 *
 * Group members share every wide bar, so the whole-symbol misfit differs
 * by well under 1% (F/P, Y/R, Z backwards/G): the deciding narrow detail
 * is a few samples among hundreds. Summing squared residuals only where
 * the two blurred models differ by BC_LAYOUT_LOCAL_DIFF of a bar's
 * contrast compares exactly that detail. Each keeps its own whole-symbol
 * scale, shift, gain and offset, so the window cannot refit freely.
 * @return Samples compared.
 */
static uint32_t layout_local (uint8_t c1, uint8_t c2,
                              double * p_s1, double * p_s2)
{
    int32_t const min_diff = (int32_t)(BC_LAYOUT_LOCAL_DIFF * BC_LAYOUT_Q);
    uint32_t n = 0U;
    *p_s1 = 0.0;
    *p_s2 = 0.0;
    layout_cache(c1);
    layout_build_grid((uint8_t)(c1 / 2U), g_grid);
    layout_build_grid((uint8_t)(c2 / 2U), g_grid2);
    for (uint32_t k = 0U; k < g_cache_count; ++k)
    {
        float const x1 = layout_model_x(c1, g_cache_pos[k]);
        float const x2 = layout_model_x(c2, g_cache_pos[k]);
        if ((x1 > -BC_LAYOUT_EDGE) && (x1 < BC_LAYOUT_TOTAL + BC_LAYOUT_EDGE)
            && (x2 > -BC_LAYOUT_EDGE) &&
            (x2 < BC_LAYOUT_TOTAL + BC_LAYOUT_EDGE))
        {
            int32_t const m1 = layout_table_value(g_grid, x1);
            int32_t const m2 = layout_table_value(g_grid2, x2);
            int32_t const diff = (m1 > m2) ? (m1 - m2) : (m2 - m1);
            if (diff >= min_diff)
            {
                double const y = (double)g_cache_sig[k];
                double const r1 = y - (g_gain[c1] * m1 + g_icpt[c1]);
                double const r2 = y - (g_gain[c2] * m2 + g_icpt[c2]);
                *p_s1 += r1 * r1;
                *p_s2 += r2 * r2;
                ++n;
            }
        }
    }
    return n;
}

/** Decide among the group: rank by whole-symbol misfit, then let the
 * local comparison of the top two decide, refusing a near tie. */
static void layout_decide (bc_layout_result_t * p_out)
{
    uint8_t best = BC_LAYOUT_NONE;
    uint8_t runner = BC_LAYOUT_NONE;
    /* best and runner are candidates: 2 * letter + reverse. */
    for (uint32_t k = 0U; k < g_group_count; ++k)
    {
        uint8_t c = g_group[k];
        if ((0U != g_mse_valid[c]) &&
            ((BC_LAYOUT_NONE == best) || (g_mse[c] < g_mse[best]) ||
             ((g_mse[c] == g_mse[best]) && (c < best))))
        {
            best = c;
        }
    }
    /* Runner-up: the best-fitting candidate that is a different letter
     * (the same letter read backwards is not a competing answer). */
    for (uint32_t k = 0U; k < g_group_count; ++k)
    {
        uint8_t c = g_group[k];
        if ((BC_LAYOUT_NONE != best) && (0U != g_mse_valid[c]) &&
            ((c / 2U) != (best / 2U)) &&
            ((BC_LAYOUT_NONE == runner) || (g_mse[c] < g_mse[runner])))
        {
            runner = c;
        }
    }
    if (BC_LAYOUT_NONE == best)
    {
        p_out->reason = BC_LAYOUT_MODEL_FAIL;
        return;
    }
#ifdef BC_LAYOUT_DEBUG
    printf("    [v%u model] best=%c r2=%.3f\n", (unsigned)g_variant,
           'A' + (int)(best / 2U),
           (g_var[best] > 0.0) ? 1.0 - g_mse[best] / g_var[best] : 0.0);
#endif
    if ((BC_LAYOUT_NONE != runner) && ((best / 2U) != (runner / 2U)))
    {
        double s_best = 0.0;
        double s_runner = 0.0;
        uint32_t const n = layout_local(best, runner, &s_best, &s_runner);
        double const low = (s_best < s_runner) ? s_best : s_runner;
        double const high = (s_best < s_runner) ? s_runner : s_best;
        double gap = (low > 0.0) ? (high / low - 1.0) * 1000.0 : 60000.0;
        if (s_runner < s_best)
        {
            /* The deciding detail fits the runner-up better. */
            uint8_t const swap = best;
            best = runner;
            runner = swap;
        }
        p_out->gap_permille = (gap < 60000.0) ? (uint16_t)gap : 60000U;
        if ((n < BC_LAYOUT_LOCAL_MIN) ||
            (gap < BC_LAYOUT_LOCAL_MARGIN * 1000.0))
        {
            /* Too close to call: a wrong letter misroutes the robot. */
            p_out->reason = BC_LAYOUT_AMBIGUOUS;
            p_out->other = (uint8_t)(runner / 2U);
            p_out->runner = (uint8_t)(best / 2U);
            return;
        }
    }
    p_out->valid = 1U;
    p_out->reason = BC_LAYOUT_OK;
    p_out->letter = (uint8_t)(best / 2U);
    p_out->reverse = (uint8_t)(best % 2U);
    p_out->runner = (BC_LAYOUT_NONE == runner) ? BC_LAYOUT_NONE :
                    (uint8_t)(runner / 2U);
}

/* -------------------------------------------------------------- driver */

static void layout_start_variant (void)
{
    bc_layout_result_t * const p_out = &g_variant_result[g_variant];
    *p_out = (bc_layout_result_t){0U};
    p_out->runner = BC_LAYOUT_NONE;
    p_out->other = BC_LAYOUT_NONE;
    p_out->letter = BC_LAYOUT_NONE;
    g_phase = LAYOUT_PEAKS;
}

static void layout_combine (void)
{
    bc_layout_result_t const * const p_centre = &g_variant_result[0];
    bc_layout_result_t const * const p_sensor = &g_variant_result[1];
    g_result = *p_centre;
    if (0U != g_has_left)
    {
        if ((0U != p_centre->valid) && (0U != p_sensor->valid) &&
            (p_centre->letter != p_sensor->letter))
        {
            g_result.valid = 0U;
            g_result.reason = BC_LAYOUT_DISAGREE;
            g_result.other = p_sensor->letter;
        }
        else if ((0U == p_centre->valid) && (0U != p_sensor->valid) &&
                 ((BC_LAYOUT_AMBIGUOUS != p_centre->reason) ||
                  (p_sensor->gap_permille >= BC_LAYOUT_OVERRULE_PERMILLE)))
        {
            /* A centre tie between two letters is real doubt: the sensor
             * distance may overrule it only when clearly decisive (a big
             * T backwards was read as G at 16% that way). Other centre
             * refusals may still fall back. */
            g_result = *p_sensor;
        }
        else
        {
            /* Centre result (read, or both refused) stands. */
        }
    }
    g_phase = LAYOUT_DONE;
}

static void layout_finish_variant (void)
{
    if ((0U == g_variant) && (0U != g_has_left))
    {
        g_variant = 1U;
        layout_start_variant();
    }
    else
    {
        layout_combine();
    }
}

void bc_layout_begin (uint32_t count, bc_layout_read_t read_sample,
                      uint8_t b_has_left)
{
    if (0U == g_tables_ready)
    {
        layout_tables();
    }
    g_read = read_sample;
    g_count = (0 == read_sample) ? 0U : count;
    g_has_left = (0U != b_has_left) ? 1U : 0U;
    g_variant = 0U;
    g_result = (bc_layout_result_t){0U};
    g_result.reason = BC_LAYOUT_FEW_BARS;
    layout_start_variant();
}

uint8_t bc_layout_poll (void)
{
    bc_layout_result_t * const p_out = &g_variant_result[g_variant];
    if (LAYOUT_PEAKS == g_phase)
    {
        layout_peaks();
        if (g_peaks < BC_LAYOUT_WIDE)
        {
            p_out->reason = BC_LAYOUT_FEW_BARS;
            layout_finish_variant();
        }
        else
        {
            g_phase = LAYOUT_BARS;
        }
    }
    else if (LAYOUT_BARS == g_phase)
    {
        layout_bars();
        p_out->bars = (uint8_t)g_raw_count;
        if ((0U != g_raw_overflow) ||
            (g_raw_count > BC_LAYOUT_MAX_CLEAN_BARS))
        {
            p_out->reason = BC_LAYOUT_MANY_BARS;
            layout_finish_variant();
        }
        else if (g_bars < BC_LAYOUT_WIDE)
        {
            p_out->reason = BC_LAYOUT_FEW_BARS;
            layout_finish_variant();
        }
        else
        {
            for (uint32_t k = 0U; k < BC_LAYOUT_WIDE; ++k)
            {
                g_pick[k] = k;
            }
            for (uint32_t letter = 0U; letter < 26U; ++letter)
            {
                for (uint32_t reverse = 0U; reverse < 2U; ++reverse)
                {
                    g_fit[letter][reverse].err = BC_LAYOUT_BIG_ERR;
                    g_fit[letter][reverse].a = 0.0F;
                    g_fit[letter][reverse].b = 0.0F;
                }
            }
            g_combos_done = 0U;
            g_phase = LAYOUT_SEARCH;
        }
    }
    else if (LAYOUT_SEARCH == g_phase)
    {
        layout_search();
        if (0U != g_combos_done)
        {
            g_phase = LAYOUT_PICK;
        }
    }
    else if (LAYOUT_PICK == g_phase)
    {
        layout_pick(p_out);
        if (0U == g_group_count)
        {
            layout_finish_variant();
        }
        else
        {
            g_group_index = 0U;
            g_phase = LAYOUT_INSIDE;
        }
    }
    else if (LAYOUT_INSIDE == g_phase)
    {
        layout_inside();
        g_phase = LAYOUT_MODEL;
    }
    else if (LAYOUT_MODEL == g_phase)
    {
        for (uint32_t n = 0U;
             (n < BC_LAYOUT_FITS_PER_POLL) && (g_fit_index < BC_LAYOUT_FITS);
             ++n)
        {
            layout_model_fit();
        }
        if (g_fit_index >= BC_LAYOUT_FITS)
        {
            ++g_group_index;
            if (g_group_index < g_group_count)
            {
                g_phase = LAYOUT_INSIDE;
            }
            else
            {
                layout_decide(p_out);
                layout_finish_variant();
            }
        }
    }
    else
    {
        /* Idle or done: nothing to do. */
    }
    return (uint8_t)((LAYOUT_DONE == g_phase) || (LAYOUT_IDLE == g_phase));
}

bc_layout_result_t bc_layout_result (void)
{
    return g_result;
}

char const * bc_layout_reason_text (uint8_t reason)
{
    static char const * const texts[] = {
        "ok", "few_bars", "no_fit", "ambiguous", "model_fail",
        "centre_sensor_disagree", "many_bars"
    };
    char const * text = "unknown";
    if (reason < (sizeof(texts) / sizeof(texts[0])))
    {
        text = texts[reason];
    }
    return text;
}

/*** end of file ***/

