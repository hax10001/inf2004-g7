/** @file bc_wave.c
 * @brief Distance-based waveform segmentation, exact decoding and diagnostics.
 */
#include "bc_wave.h"
#include "bc_code.h"
#include "bc_scan.h"
#include "bc_port.h"
#include "bc_match.h"
#include "bc_layout.h"
#include "bc_fast.h"
#include "bc_cal.h"
#include "app/app_config.h"
#include "drivers/encoder.h"
#include <tm/tmonitor.h>

#if APP_BARCODE_ANALOG_ENABLED && APP_BARCODE_BUFFERED_ENABLED && \
    BC_CAL_READY && !BC_CAL_MEASURE
#define BC_WAVE_CAPACITY 8192U
#define BC_WAVE_PRETRIGGER 512U
#define BC_WAVE_WHITE_SAMPLES 250U
#define BC_WAVE_MAX_USEC 16000000U
#define BC_WAVE_MAX_GAP_USEC 20000U
#define BC_WAVE_RUNS 256U
#if APP_BARCODE_LAYOUT_ENABLED
/* Larger steps: each poll of the 5 ms barcode task did ~1 ms of work. */
#define BC_WAVE_CHUNK 2048U
#else
#define BC_WAVE_CHUNK 256U
#endif
#define BC_WAVE_RADIUS (BC_CAL_MAX_BARCODE_UNITS / 12U)
#define BC_WAVE_END_WHITE_UNITS (BC_CAL_MAX_BARCODE_UNITS / 12U)
#define BC_WAVE_OFFSET_TRIALS 19U
#define BC_WAVE_TRIALS 26U

typedef enum
{
    BC_WAVE_LEARN,
    BC_WAVE_WAIT,
    BC_WAVE_CAPTURE,
    BC_WAVE_SMOOTH,
    BC_WAVE_ENVELOPE,
    BC_WAVE_UPPER,
    BC_WAVE_LEGACY,
    BC_WAVE_SEARCH,
    BC_WAVE_MATCH_INIT,
    BC_WAVE_MATCH,
    BC_WAVE_LAYOUT_INIT,
    BC_WAVE_LAYOUT,
    BC_WAVE_REPORT,
    BC_WAVE_DUMP,
    BC_WAVE_DONE,
    BC_WAVE_CLEAR /**< Wait for real white background before rearming. */
} bc_wave_phase_t;

typedef struct
{
    uint32_t units;
    uint16_t adc : 12;      /**< 12-bit ADC reading. */
    uint16_t left_step : 4; /**< Left counts since the previous row, <= 15. */
    uint16_t usec; /**< Low word; adjacent valid gaps must be below 65 ms. */
} bc_wave_row_t;

/* Not barcode-like: too few or too many strong bars, or one wider than
 * twice the widest Code 39 bar the span limits allow. Such captures (line
 * edges, junction bars, dark floor, ramps) are not counted as failures.
 */
#define BC_WAVE_LIKE_MIN_BARS 3U
#define BC_WAVE_LIKE_MAX_BARS 16U
#define BC_WAVE_LIKE_MAX_BAR_UNITS (BC_CAL_MAX_BARCODE_UNITS * 6U / 47U)
#define BC_WAVE_LIKE_MIN_SIGMAS 8U
/* Re-learn white if the old baseline is not seen again for this many valid
 * samples (6 s): the surface, light or height may have changed for good.
 */
#define BC_WAVE_CLEAR_MAX_SAMPLES 3000U
/* White reads ~150-250 on this sensor and ink 600-3900. A learned "white"
 * darker than this means the sensor sat on the line or a barcode at boot;
 * learning repeats instead of arming with black as the baseline. */
#define BC_WAVE_WHITE_MAX_DARKNESS 800U
/* A capture must end by its 16 s / 8192-sample limits. If one is still
 * open this long by the task's own clock, log the state and restart. */
#define BC_WAVE_CAPTURE_WATCHDOG_USEC 20000000U
#ifdef B1_BARCODE_TEST
#define BC_WAVE_DUMP_BATCH 6U
#else
#define BC_WAVE_DUMP_BATCH 2U
#endif

/* Approximately 132 KiB, static rather than on the task's 4 KiB stack.
 * No allocation, waits or printing in sample collection.
 */
static bc_wave_row_t g_rows[BC_WAVE_CAPACITY];
static uint16_t g_smooth[BC_WAVE_CAPACITY];
static uint16_t g_floor[BC_WAVE_CAPACITY];
static uint16_t g_contrast[BC_WAVE_CAPACITY];
/* Envelope indices and half-rate integral cache have disjoint lifetimes.
 * Reuse the storage, preserving raw/smoothed arrays for the final dump.
 */
static union
{
    uint16_t envelope[BC_WAVE_CAPACITY];
    int32_t integral[BC_WAVE_CAPACITY / 2U + 1U];
} g_workspace;
static uint32_t g_envelope_head;
static uint32_t g_envelope_tail;
static uint32_t g_envelope_next;
static uint32_t g_run_width[BC_WAVE_RUNS];
static uint8_t g_run_black[BC_WAVE_RUNS];
static uint32_t g_widths[26][BC_CODE_ELEMENTS];
static uint16_t g_fit[26];
static uint8_t g_reverse[26];
static uint8_t g_votes[26];
static uint16_t const g_offsets[BC_WAVE_OFFSET_TRIALS] = {
    8U, 16U, 24U, 32U, 48U, 64U, 96U, 128U, 192U, 256U,
    384U, 512U, 768U, 1024U, 1280U, 1536U, 2048U, 2560U, 3200U
};
static bc_wave_phase_t g_phase;
static uint32_t g_write;
static uint32_t g_count;
static uint32_t g_start;
static uint32_t g_index;
static uint32_t g_trial;
static uint32_t g_units;
static int32_t g_left;
static int32_t g_right;
static uint32_t g_last_usec;
static uint32_t g_trigger_usec;
static uint32_t g_trigger_msec;
static uint32_t g_freeze_usec;
static uint32_t g_capture_msec;
static uint32_t g_last_signal_units;
static uint32_t g_tail_level;
static volatile uint32_t g_motion_hold;
static volatile uint32_t g_rearm_requested;
static uint64_t g_clock_usec;
static uint32_t g_clock_last;
static uint8_t g_clock_ready;
static uint32_t g_trigger_units;
static uint32_t g_white_count;
static uint64_t g_white_sum;
static uint64_t g_white_square;
static uint32_t g_white_mean;
static uint32_t g_white_sigma;
static uint32_t g_white_min = 4095U;
static uint32_t g_white_max;
static uint32_t g_gap_max;
static uint32_t g_invalid;
static uint32_t g_adc_errors;
static uint32_t g_backward;
static uint32_t g_busy_samples;
static uint32_t g_sequence;
static uint32_t g_saturated;
static uint32_t g_run_count;
static uint32_t g_trial_frames;
static uint32_t g_trial_mask;
static uint32_t g_last_widths[BC_CODE_ELEMENTS];
static uint32_t g_legacy_edges;
static uint8_t g_have_counts;
static uint8_t g_moved;
static uint8_t g_freeze_reason;
static uint32_t g_clear_white_samples;
static uint32_t g_clear_total_samples;
static volatile uint32_t g_suppressed;
static uint32_t g_left_units;
static uint32_t g_row_left_units;
static uint32_t g_white_rejects;
static uint32_t g_capture_seen_usec;
static uint8_t g_white_dark;              /* last learn refused as dark */
static uint32_t g_dark_adc;               /* reading that was refused */
static volatile uint32_t g_relearn_requested;
/* Last finished pass, for MQTT diagnostics. */
static uint8_t g_last_valid;
static uint8_t g_last_letter = BC_CODE_LETTER_INVALID;
static uint8_t g_last_reason;
static uint16_t g_last_process_ms;
static uint16_t g_last_sequence;
#ifdef B1_BARCODE_TEST
static uint32_t g_clear_report_usec;
#endif
static uint32_t g_ignored;
static uint32_t g_scan_guard_until_usec;
static uint8_t g_run_overflow;
static bc_code_result_t g_legacy;
#if APP_BARCODE_WAVE_DUMP_ENABLED
static uint32_t g_dump_index;
static uint32_t g_dump_usec;
static uint32_t g_dump_left;
#endif

uint8_t bc_wave_motion_hold (void)
{
    return (uint8_t)g_motion_hold;
}

uint8_t bc_wave_can_rearm (void)
{
    return (uint8_t)(g_motion_hold == 0U || g_phase == BC_WAVE_DONE);
}
uint8_t bc_wave_capture_active (void)
{
    return (uint8_t)(g_phase == BC_WAVE_CAPTURE);
}

uint8_t bc_wave_navigation_guard_active(void)
{
    /* Recently observed scan stripes are not junctions by themselves.
     * This time window is NEVER authorization for unbounded movement. */
    uint32_t now = encoder_now_usec();
    return (uint8_t)(g_phase == BC_WAVE_CAPTURE ||
        g_phase == BC_WAVE_CLEAR ||
        (g_scan_guard_until_usec != 0U &&
         (int32_t)(g_scan_guard_until_usec - now) > 0));
}

void bc_wave_set_suppressed (uint8_t b_suppressed)
{
    g_suppressed = (0U != b_suppressed) ? 1U : 0U;
}

void bc_wave_request_rearm (void)
{
    /* The barcode owner task alone changes scan state. The controller
     * requests re-arm only after an explicit remote START. */
    g_rearm_requested = 1U;
}

void bc_wave_request_relearn (void)
{
    /* Acted on by bc_wave_poll() in the barcode task. */
    g_relearn_requested = 1U;
}

uint8_t bc_wave_decode_pending (void)
{
    bc_wave_phase_t const phase = g_phase;
    return (uint8_t)((phase >= BC_WAVE_SMOOTH) && (phase <= BC_WAVE_REPORT));
}

void bc_wave_status (bc_wave_status_t * p_status)
{
    bc_wave_phase_t const phase = g_phase;
    uint8_t state = BC_WAVE_STATE_DECODING;
    if (0 == p_status)
    {
        return;
    }
    if (BC_WAVE_LEARN == phase)
    {
        state = (0U != g_white_dark) ? BC_WAVE_STATE_NOT_WHITE :
                                       BC_WAVE_STATE_LEARNING;
    }
    else if (BC_WAVE_WAIT == phase)
    {
        state = BC_WAVE_STATE_READY;
    }
    else if (BC_WAVE_CAPTURE == phase)
    {
        state = BC_WAVE_STATE_CAPTURE;
    }
    else if (BC_WAVE_CLEAR == phase)
    {
        state = BC_WAVE_STATE_CLEARING;
    }
    else
    {
        /* Processing, reporting or dumping a recorded pass. */
    }
    p_status->state = state;
    p_status->white_rejects = (uint8_t)((g_white_rejects > 255U) ?
                                        255U : g_white_rejects);
    /* Not on white: report the dark reading, not the stale baseline. */
    p_status->white_mean = (uint16_t)((BC_WAVE_STATE_NOT_WHITE == state) ?
                                      g_dark_adc : g_white_mean);
    p_status->white_sigma = (uint16_t)g_white_sigma;
    p_status->sequence = g_last_sequence;
    p_status->last_valid = g_last_valid;
    p_status->last_letter = g_last_letter;
    p_status->last_reason = g_last_reason;
    p_status->last_process_ms = g_last_process_ms;
}

static void wave_rearm_completed_capture (void)
{
    if (g_phase != BC_WAVE_DONE || g_rearm_requested == 0U)
    {
        return;
    }
    /* All frozen samples have been processed, and the published result
     * remains in bc_port's mailbox. Keep g_sequence for deduplication. */
    g_write = 0U;
    g_count = 0U;
    g_units = 0U;
    g_have_counts = 0U;
    g_moved = 0U;
    g_clock_ready = 0U;
    g_white_count = 0U;
    g_white_sum = 0U;
    g_white_square = 0U;
    /* Keep previous measured white baseline while waiting for clearance.
     * Reset it only after the sensor has actually left the printed symbol. */
    g_white_min = 4095U;
    g_white_max = 0U;
    g_gap_max = 0U;
    g_invalid = 0U;
    g_adc_errors = 0U;
    g_backward = 0U;
    g_saturated = 0U;
    g_run_count = 0U;
    g_last_signal_units = 0U;
    g_clear_white_samples = 0U;
    g_clear_total_samples = 0U;
    g_left_units = 0U;
    g_row_left_units = 0U;
    g_phase = BC_WAVE_CLEAR;
    g_rearm_requested = 0U;
    g_motion_hold = 0U;
    tm_printf((UB *)"[BC WAVE] Scan worker rearmed capture; "
                    "motor hold released\n");
}

static bc_wave_row_t const * wave_row (uint32_t index)
{
    return &g_rows[(g_start + index) % BC_WAVE_CAPACITY];
}

static uint32_t wave_sqrt (uint32_t value)
{
    uint32_t result = 0U;
    uint32_t bit = 1U << 30U;
    for (uint32_t i = 0U; i < 16U; ++i)
    {
        if (value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1U) + bit;
        }
        else
        {
            result >>= 1U;
        }
        bit >>= 2U;
    }
    return result;
}

static void wave_freeze (uint8_t reason)
{
#ifndef B1_LIVE_COMMANDS
    /* Bench waveform capture intentionally holds motion. */
    g_motion_hold = 1U;
#endif
    g_freeze_usec = encoder_now_usec();
    g_scan_guard_until_usec = g_freeze_usec + 600000U;
    g_capture_msec = (g_last_usec - g_trigger_usec) / 1000U;
    g_freeze_reason = reason;
    g_start = (g_write + BC_WAVE_CAPACITY - g_count) % BC_WAVE_CAPACITY;
    g_index = 0U;
    g_trial = 0U;
    for (uint32_t i = 0U; i < 26U; ++i)
    {
        g_votes[i] = 0U;
        g_fit[i] = 65535U;
    }
    g_legacy = (bc_code_result_t){BC_CODE_LETTER_INVALID, 0U, 0U, 0U};
    g_phase = BC_WAVE_SMOOTH;
}

void bc_wave_sample (bc_fastq_sample_t const * p_sample)
{
    if (BC_WAVE_CLEAR == g_phase)
    {
        /* Do NOT learn a new white baseline while still sitting over
         * barcode ink. Repeated black/white oscillation resets the guard.
         * Consecutive valid samples on the old white baseline are required.
         */
        uint32_t tolerance = g_white_sigma * 3U + 24U;
        uint32_t adc = p_sample->adc_raw;
        uint32_t difference = (adc > g_white_mean) ?
            adc - g_white_mean : g_white_mean - adc;
        if (p_sample->valid && p_sample->adc_valid &&
            difference <= tolerance)
        {
            ++g_clear_white_samples;
        }
        else g_clear_white_samples = 0U;
        if (p_sample->valid && p_sample->adc_valid)
        {
            ++g_clear_total_samples;
        }
        if ((g_clear_white_samples >= BC_WAVE_WHITE_SAMPLES) ||
            (g_clear_total_samples >= BC_WAVE_CLEAR_MAX_SAMPLES))
        {
            if (g_clear_white_samples < BC_WAVE_WHITE_SAMPLES)
            {
                tm_printf((UB *)"[BC WAVE] Old white not seen again; "
                                "re-learning white\n");
            }
            g_white_mean = 0U;
            g_white_sigma = 0U;
            g_clear_white_samples = 0U;
            g_clear_total_samples = 0U;
            g_phase = BC_WAVE_LEARN;
        }
        return;
    }
    if (0U == g_clock_ready)
    {
        g_clock_usec = p_sample->time_usec;
        g_clock_ready = 1U;
    }
    else
    {
        g_clock_usec += (uint32_t)(p_sample->time_usec - g_clock_last);
    }
    g_clock_last = p_sample->time_usec;
    if (g_phase > BC_WAVE_CAPTURE)
    {
        if (BC_WAVE_DONE != g_phase)
        {
            ++g_busy_samples;
        }
    }
    else
    {
        if (0U != g_have_counts)
        {
            uint32_t gap = p_sample->time_usec - g_last_usec;
            int64_t dl = ((int64_t)p_sample->left_count - g_left) *
                         BC_CAL_LEFT_FORWARD_SIGN;
            int64_t dr = ((int64_t)p_sample->right_count - g_right) *
                         BC_CAL_RIGHT_FORWARD_SIGN;
            if (gap > g_gap_max)
            {
                g_gap_max = gap;
            }
            if ((dl < 0) || (dr < 0))
            {
                ++g_backward;
            }
            /* Match the existing forward-only travel estimate and expose
             * backward increments in logs instead of hiding their presence.
             */
            uint64_t step = (uint64_t)((dl > 0) ? dl : 0) +
                            (uint64_t)((dr > 0) ? dr : 0);
            g_left_units += (uint32_t)((dl > 0) ? dl : 0);
            if (step > UINT32_MAX - g_units)
            {
                ++g_adc_errors;
            }
            else
            {
                g_units += (uint32_t)step;
                if (0U != step)
                {
                    g_moved = 1U;
                }
            }
        }
        g_left = p_sample->left_count;
        g_right = p_sample->right_count;
        g_last_usec = p_sample->time_usec;
        g_have_counts = 1U;
        if (0U == p_sample->valid)
        {
            ++g_invalid;
        }
        if (0U == p_sample->adc_valid)
        {
            ++g_adc_errors;
        }
        else
        {
            uint32_t adc = p_sample->adc_raw;
            if ((adc <= 4U) || (adc >= 4091U))
            {
                ++g_saturated;
            }
            if ((BC_WAVE_LEARN == g_phase) &&
                ((APP_BARCODE_ADC_BLACK_HIGH ? adc : 4095U - adc) >
                 BC_WAVE_WHITE_MAX_DARKNESS))
            {
                /* Dark sample (line, barcode, junction): white is learned
                 * only from consecutive white samples, so restart. */
                if (0U == g_white_dark)
                {
                    ++g_white_rejects;
                }
                g_white_dark = 1U;
                g_dark_adc = adc;
                g_white_count = 0U;
                g_white_sum = 0U;
                g_white_square = 0U;
                g_white_min = 4095U;
                g_white_max = 0U;
            }
            else if (BC_WAVE_LEARN == g_phase)
            {
                g_white_sum += adc;
                g_white_square += (uint64_t)adc * adc;
                if (adc < g_white_min)
                {
                    g_white_min = adc;
                }
                if (adc > g_white_max)
                {
                    g_white_max = adc;
                }
                ++g_white_count;
                if (g_white_count >= BC_WAVE_WHITE_SAMPLES)
                {
                    g_white_mean = (uint32_t)(g_white_sum / g_white_count);
                    uint32_t variance = (uint32_t)
                        ((g_white_square * g_white_count -
                          g_white_sum * g_white_sum) /
                         ((uint64_t)g_white_count * g_white_count));
                    g_white_sigma = wave_sqrt(variance);
                    {
                        uint32_t const darkness = APP_BARCODE_ADC_BLACK_HIGH ?
                            g_white_mean : 4095U - g_white_mean;
                        if (darkness > BC_WAVE_WHITE_MAX_DARKNESS)
                        {
                            /* Not white: learn again (reported by poll). */
                            if (0U == g_white_dark)
                            {
                                ++g_white_rejects;
                            }
                            g_white_dark = 1U;
                            g_dark_adc = g_white_mean;
                            g_white_count = 0U;
                            g_white_sum = 0U;
                            g_white_square = 0U;
                            g_white_min = 4095U;
                            g_white_max = 0U;
                        }
                        else
                        {
                            g_white_dark = 0U;
                            g_phase = BC_WAVE_WAIT;
                        }
                    }
                }
            }
            bc_wave_row_t * const p_row = &g_rows[g_write];
            p_row->units = g_units;
            p_row->adc = (uint16_t)(adc & 0x0FFFU);
            {
                /* Per-row left-wheel step: the layout decoder rebuilds
                 * distance at the sensor from it. 4 bits is ~3x the
                 * fastest 2 ms step. */
                uint32_t step = g_left_units - g_row_left_units;
                p_row->left_step = (uint16_t)((step > 15U) ? 15U : step);
                g_row_left_units = g_left_units;
            }
            p_row->usec = (uint16_t)g_clock_usec;
            g_write = (g_write + 1U) % BC_WAVE_CAPACITY;
            if (g_count < BC_WAVE_CAPACITY)
            {
                ++g_count;
            }
            uint32_t trigger = g_white_sigma * 3U + 8U;
            uint32_t difference = (adc > g_white_mean) ?
                                   adc - g_white_mean : g_white_mean - adc;
            uint8_t black_side = APP_BARCODE_ADC_BLACK_HIGH ?
                                (adc > g_white_mean) : (adc < g_white_mean);
            if ((BC_WAVE_WAIT == g_phase) && (0U != g_moved) &&
                (0U == g_suppressed) &&
                (0U != black_side) && (difference > trigger))
            {
                g_trigger_units = g_units;
                g_trigger_usec = p_sample->time_usec;
                g_trigger_msec = (uint32_t)(g_clock_usec / 1000U);
                g_last_signal_units = g_units;
                g_tail_level = adc;
                ++g_sequence;
                g_phase = BC_WAVE_CAPTURE;
            }
            if (BC_WAVE_CAPTURE == g_phase)
            {
                /* Light causal smoothing only for the end-of-pass check.
                 * Full-resolution raw ADC remains in the saved waveform.
                 */
                g_tail_level = (g_tail_level * 3U + adc) / 4U;
                uint32_t tail_difference = (g_tail_level > g_white_mean) ?
                    g_tail_level - g_white_mean :
                    g_white_mean - g_tail_level;
                uint8_t tail_black_side = APP_BARCODE_ADC_BLACK_HIGH ?
                    (g_tail_level > g_white_mean) :
                    (g_tail_level < g_white_mean);
                if ((0U != tail_black_side) && (tail_difference > trigger))
                {
                    g_last_signal_units = g_units;
                }
            }
            if (g_phase < BC_WAVE_CAPTURE)
            {
                if (g_count > BC_WAVE_PRETRIGGER)
                {
                    g_count = BC_WAVE_PRETRIGGER;
                }
            }
        }
        if (BC_WAVE_CAPTURE == g_phase)
        {
            /* Clearance exceeds the widest nominal internal space for
             * the configured maximum 3:1 Code 39 print span. It is not
             * the legacy 60-unit quiet zone, which could cut inside a code.
             */
            if (((g_units - g_trigger_units) >= BC_CAL_MIN_BARCODE_UNITS) &&
                ((g_units - g_last_signal_units) >= BC_WAVE_END_WHITE_UNITS))
            {
                wave_freeze(4U);
            }
            else if (g_count >= BC_WAVE_CAPACITY)
            {
                wave_freeze(1U);
            }
            else if ((g_units - g_trigger_units) >=
                     BC_CAL_MAX_BARCODE_UNITS + 2U * BC_CAL_TRAILING_UNITS)
            {
                wave_freeze(2U);
            }
            else if ((uint32_t)(p_sample->time_usec - g_trigger_usec) >=
                     BC_WAVE_MAX_USEC)
            {
                wave_freeze(3U);
            }
        }
    }
}

static void wave_smooth (void)
{
    uint32_t end = g_index + BC_WAVE_CHUNK;
    for (; (g_index < g_count) && (g_index < end); ++g_index)
    {
        uint32_t lo = (g_index > 0U) ? g_index - 1U : g_index;
        uint32_t hi = (g_index + 1U < g_count) ? g_index + 1U : g_index;
        uint32_t sum = 0U;
        for (uint32_t j = lo; j <= hi; ++j)
        {
            uint32_t adc = wave_row(j)->adc;
            sum += APP_BARCODE_ADC_BLACK_HIGH ? adc : 4095U - adc;
        }
        g_smooth[g_index] = (uint16_t)(sum / (hi - lo + 1U));
    }
    if (g_index == g_count)
    {
        g_index = 0U;
        g_envelope_head = 0U;
        g_envelope_tail = 0U;
        g_envelope_next = 0U;
        g_phase = BC_WAVE_ENVELOPE;
    }
}

static void wave_envelope (void)
{
    uint8_t upper = (BC_WAVE_UPPER == g_phase);
    uint32_t end = g_index + BC_WAVE_CHUNK;
    for (; (g_index < g_count) && (g_index < end); ++g_index)
    {
        uint32_t position = wave_row(g_index)->units;
        while ((g_envelope_next < g_count) &&
               ((wave_row(g_envelope_next)->units <= position) ||
                (wave_row(g_envelope_next)->units - position <=
                 BC_WAVE_RADIUS)))
        {
            uint16_t value = g_smooth[g_envelope_next];
            uint8_t b_remove = 1U;
            while ((g_envelope_tail > g_envelope_head) &&
                   (0U != b_remove))
            {
                uint16_t previous =
                    g_smooth[g_workspace.envelope[g_envelope_tail - 1U]];
                b_remove = upper ? (previous <= value) : (previous >= value);
                if (0U != b_remove)
                {
                    --g_envelope_tail;
                }
            }
            g_workspace.envelope[g_envelope_tail] = (uint16_t)g_envelope_next;
            ++g_envelope_tail;
            ++g_envelope_next;
        }
        uint32_t lower_limit = (position > BC_WAVE_RADIUS) ?
                               position - BC_WAVE_RADIUS : 0U;
        while ((g_envelope_head < g_envelope_tail) &&
               (wave_row(g_workspace.envelope[g_envelope_head])->units <
                lower_limit))
        {
            ++g_envelope_head;
        }
        uint16_t extreme = g_smooth[g_workspace.envelope[g_envelope_head]];
        if (0U != upper)
        {
            g_contrast[g_index] = (uint16_t)(extreme - g_floor[g_index]);
        }
        else
        {
            g_floor[g_index] = extreme;
#if APP_BARCODE_LAYOUT_ENABLED
            g_contrast[g_index] = 0U; /* upper envelope not computed */
#endif
        }
    }
    if (g_index == g_count)
    {
        g_index = 0U;
        g_envelope_head = 0U;
        g_envelope_tail = 0U;
        g_envelope_next = 0U;
#if APP_BARCODE_LAYOUT_ENABLED
        /* Only the old segmentation search used the upper envelope. */
        (void)upper;
        g_phase = BC_WAVE_LEGACY;
#else
        g_phase = upper ? BC_WAVE_LEGACY : BC_WAVE_UPPER;
#endif
    }
}

static uint16_t wave_fit (uint32_t const * p_width)
{
    uint32_t error = 0U;
    for (uint32_t symbol = 0U; symbol < 3U; ++symbol)
    {
        for (uint32_t colour = 0U; colour < 2U; ++colour)
        {
            uint32_t sorted[5] = {0U};
            uint32_t count = 0U;
            for (uint32_t i = colour; i < 9U; i += 2U)
            {
                uint32_t width = p_width[symbol * 10U + i];
                uint32_t slot = count;
                while ((slot > 0U) && (sorted[slot - 1U] > width))
                {
                    sorted[slot] = sorted[slot - 1U];
                    --slot;
                }
                sorted[slot] = width;
                ++count;
            }
            uint32_t narrow = (sorted[0] + sorted[1] + sorted[2]) / 3U;
            uint32_t wide = sorted[3];
            if (5U == count)
            {
                wide = (wide + sorted[4]) / 2U;
            }
            for (uint32_t i = 0U; i < count; ++i)
            {
                uint32_t centre = (i < 3U) ? narrow : wide;
                uint32_t delta = (sorted[i] > centre) ?
                                  sorted[i] - centre : centre - sorted[i];
                if (0U != centre)
                {
                    error += delta * 100U / centre;
                }
            }
        }
    }
    return (uint16_t)(error / 27U);
}

static void wave_candidate (uint32_t start)
{
    uint32_t span = 0U;
    for (uint32_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
    {
        g_last_widths[i] = g_run_width[start + i];
        span += g_last_widths[i];
    }
    if ((span >= BC_CAL_MIN_BARCODE_UNITS) &&
        (span <= BC_CAL_MAX_BARCODE_UNITS))
    {
        bc_code_result_t decoded = bc_code_decode(g_last_widths,
                                                   BC_CAL_MIN_RUN_UNITS);
        if (0U != decoded.valid)
        {
            uint16_t fit = wave_fit(g_last_widths);
            if (fit <= APP_BARCODE_WAVE_MAX_FIT_PERCENT)
            {
                ++g_trial_frames;
                g_trial_mask |= 1U << decoded.letter;
                if (fit < g_fit[decoded.letter])
                {
                    g_fit[decoded.letter] = fit;
                    g_reverse[decoded.letter] = decoded.reverse;
                    for (uint32_t i = 0U; i < BC_CODE_ELEMENTS; ++i)
                    {
                        g_widths[decoded.letter][i] = g_last_widths[i];
                    }
                }
            }
        }
    }
}

static void wave_search (void)
{
    g_run_count = 0U;
    g_run_overflow = 0U;
    g_trial_mask = 0U;
    g_trial_frames = 0U;
    uint8_t black = 0U;
    uint32_t start = wave_row(0U)->units;
    for (uint32_t i = 0U; i < g_count; ++i)
    {
        uint32_t value = g_smooth[i] - g_floor[i];
        uint32_t threshold = (g_trial < BC_WAVE_OFFSET_TRIALS) ?
            g_offsets[g_trial] :
            g_contrast[i] * (15U + (g_trial - BC_WAVE_OFFSET_TRIALS) * 10U)
            / 100U;
        uint32_t noise = (g_white_sigma > 4U) ? g_white_sigma : 4U;
        if (threshold < noise)
        {
            threshold = noise;
        }
        uint32_t hysteresis = noise / 2U;
        if (hysteresis > threshold / 4U)
        {
            hysteresis = threshold / 4U;
        }
        uint8_t next = black;
        if (value >= threshold + hysteresis)
        {
            next = 1U;
        }
        else if (value <= threshold - hysteresis)
        {
            next = 0U;
        }
        if (next != black)
        {
            if (g_run_count < BC_WAVE_RUNS)
            {
                g_run_width[g_run_count] = wave_row(i)->units - start;
                g_run_black[g_run_count] = black;
                ++g_run_count;
            }
            else
            {
                g_run_overflow = 1U;
            }
            start = wave_row(i)->units;
            black = next;
        }
    }
    if (g_run_count < BC_WAVE_RUNS)
    {
        g_run_width[g_run_count] = wave_row(g_count - 1U)->units - start;
        g_run_black[g_run_count] = black;
        ++g_run_count;
    }
    else
    {
        g_run_overflow = 1U;
    }
    if (0U == g_run_overflow)
    {
        for (uint32_t i = 1U; i + BC_CODE_ELEMENTS < g_run_count; ++i)
        {
            if ((0U != g_run_black[i]) &&
                (g_run_width[i - 1U] >= BC_CAL_LEADING_UNITS) &&
                (g_run_width[i + BC_CODE_ELEMENTS] >= BC_CAL_TRAILING_UNITS))
            {
                wave_candidate(i);
            }
        }
    }
    /* One vote per threshold, only if that threshold found one character.
     * Repeated windows do not inflate support for the same threshold.
     */
    uint8_t duplicate_floor = (g_trial > 0U) &&
                             (g_trial < BC_WAVE_OFFSET_TRIALS) &&
                             (g_offsets[g_trial] <= g_white_sigma);
    if ((0U == duplicate_floor) && (0U != g_trial_mask) &&
        (0U == (g_trial_mask & (g_trial_mask - 1U))))
    {
        for (uint32_t i = 0U; i < 26U; ++i)
        {
            if (0U != (g_trial_mask & (1U << i)))
            {
                ++g_votes[i];
            }
        }
    }
#ifndef B1_LIVE_COMMANDS
#ifndef B1_BARCODE_TEST /* quiet barcode test log */
    tm_printf((UB *)"[BC WAVE TRY] pass=%u trial=%u mode=%u value=%u "
                      "runs=%u overflow=%u matches=%u mask=%u\n",
              (UINT)g_sequence, (UINT)g_trial,
              (UINT)(g_trial >= BC_WAVE_OFFSET_TRIALS),
              (UINT)((g_trial < BC_WAVE_OFFSET_TRIALS) ?
                      g_offsets[g_trial] :
                      15U + (g_trial - BC_WAVE_OFFSET_TRIALS) * 10U),
              (UINT)g_run_count, (UINT)g_run_overflow,
              (UINT)g_trial_frames, (UINT)g_trial_mask);
#endif
#endif
    ++g_trial;
    if (g_trial >= BC_WAVE_TRIALS)
    {
#if APP_BARCODE_TEMPLATE_ENABLED
        g_phase = BC_WAVE_MATCH_INIT;
#else
        g_phase = BC_WAVE_LAYOUT_INIT;
#endif
    }
}

#if APP_BARCODE_TEMPLATE_ENABLED
static void wave_match_read (uint32_t index, uint32_t * p_units,
                              int32_t * p_signal)
{
    *p_units = wave_row(index)->units - wave_row(0U)->units;
    *p_signal = (int32_t)g_smooth[index] - g_floor[index];
}
#endif

#if APP_BARCODE_LAYOUT_ENABLED
static void wave_layout_read (uint32_t index, uint32_t * p_units,
                              uint32_t * p_left_step, int32_t * p_signal)
{
    bc_wave_row_t const * const p_row = wave_row(index);
    *p_units = p_row->units - wave_row(0U)->units;
    *p_left_step = p_row->left_step;
    *p_signal = (int32_t)g_smooth[index] - g_floor[index];
}
#endif

static void wave_legacy (void)
{
    bc_scan_config_t const config = {
        BC_CAL_MIN_RUN_UNITS, BC_CAL_LEADING_UNITS, BC_CAL_TRAILING_UNITS,
        BC_CAL_MIN_BARCODE_UNITS, BC_CAL_MAX_BARCODE_UNITS, BC_CAL_TIMEOUT_MSEC
    };
    bc_scan_t scan = {0};
    (void)bc_scan_init(&scan, &config);
    uint8_t stable = 0U;
    uint8_t previous = 0U;
    uint32_t elapsed_usec = 0U;
    g_legacy_edges = 0U;
    uint32_t rejected[9] = {0U};
    /* Use original ADC, original fixed hysteresis and the existing
     * two-observation rule. Retain the first valid legacy detection.
     */
    for (uint32_t i = 0U; i + 1U < g_count; ++i)
    {
        if (i > 0U)
        {
            elapsed_usec += (uint16_t)(wave_row(i)->usec -
                                       wave_row(i - 1U)->usec);
        }
        uint32_t adc = wave_row(i)->adc;
        uint8_t raw = previous;
        if (adc >= APP_BARCODE_ADC_THRESHOLD + APP_BARCODE_ADC_HYSTERESIS)
        {
            raw = APP_BARCODE_ADC_BLACK_HIGH ? 1U : 0U;
        }
        else if (adc <= APP_BARCODE_ADC_THRESHOLD - APP_BARCODE_ADC_HYSTERESIS)
        {
            raw = APP_BARCODE_ADC_BLACK_HIGH ? 0U : 1U;
        }
        uint32_t next_adc = wave_row(i + 1U)->adc;
        uint8_t next = raw;
        if (next_adc >= APP_BARCODE_ADC_THRESHOLD +
                        APP_BARCODE_ADC_HYSTERESIS)
        {
            next = APP_BARCODE_ADC_BLACK_HIGH ? 1U : 0U;
        }
        else if (next_adc <= APP_BARCODE_ADC_THRESHOLD -
                             APP_BARCODE_ADC_HYSTERESIS)
        {
            next = APP_BARCODE_ADC_BLACK_HIGH ? 0U : 1U;
        }
        uint16_t gap = (uint16_t)(wave_row(i + 1U)->usec - wave_row(i)->usec);
        if ((raw != stable) && (!APP_BARCODE_EDGE_FILTER_ENABLED ||
            ((next == raw) && (gap <= 8000U))))
        {
            stable = raw;
            ++g_legacy_edges;
        }
        previous = raw;
        uint8_t event = bc_scan_feed(&scan, stable, wave_row(i)->units,
                                     elapsed_usec / 1000U, 1U);
        if (BC_SCAN_EVENT_NONE != event)
        {
            uint32_t widths[BC_CODE_ELEMENTS] = {0U};
            uint32_t timestamp = 0U;
            uint8_t error = 0U;
            (void)bc_scan_take(&scan, widths, &error, &timestamp);
            if (BC_SCAN_EVENT_READY == event)
            {
                bc_code_result_t result = bc_code_decode(widths,
                                                    BC_CAL_MIN_RUN_UNITS);
                if ((0U != result.valid) && (0U == g_legacy.valid))
                {
                    g_legacy = result;
                }
                if (0U == result.valid)
                {
                    error = BC_SCAN_ERROR_PATTERN;
                }
            }
            if ((error > 0U) && (error < 9U))
            {
                ++rejected[error];
            }
        }
    }
    tm_printf((UB *)"[BC WAVE LEGACY] pass=%u valid=%u letter=%u "
                      "reverse=%u edges=%u pending_runs=%u "
                      "code2=%u code3=%u code5=%u code7=%u\n",
              (UINT)g_sequence, (UINT)g_legacy.valid, (UINT)g_legacy.letter,
              (UINT)g_legacy.reverse, (UINT)g_legacy_edges,
              (UINT)scan.run_count, (UINT)rejected[2], (UINT)rejected[3],
              (UINT)rejected[5], (UINT)rejected[7]);
#if APP_BARCODE_LAYOUT_ENABLED
    /* The layout decoder replaces the 26-threshold segmentation vote. */
    g_phase = BC_WAVE_LAYOUT_INIT;
#else
    g_phase = BC_WAVE_SEARCH;
#endif
}

#if APP_BARCODE_TEMPLATE_ENABLED && APP_BARCODE_ESTIMATES_ENABLED
/* A diagnostic estimate never enters bc_port's confirmed result queue. */
static void wave_estimate (bc_match_result_t const * p_match,
                           uint8_t conflict)
{
    char const * reason = "weak_evidence";
    uint8_t available = (p_match->letter < 26U) &&
                        (p_match->score_z100 > 0) &&
                        (p_match->positive > 0U);
    if ((0U != g_adc_errors) || (g_gap_max > BC_WAVE_MAX_GAP_USEC))
    {
        reason = "capture_quality";
        available = 0U;
    }
    else if (0U != conflict)
    {
        reason = "decoder_conflict";
        available = 0U;
    }
    else if (0U == available)
    {
        reason = (p_match->letter < 26U) ? "insufficient_evidence" :
                                         "no_template_fit";
    }
    if (0U != available)
    {
        tm_printf((UB *)"[BARCODE ESTIMATE] pass=%u potential=%c "
                          "runnerup=%c score_z100=%d runnerup_z100=%d "
                          "margin_z100=%d reverse=%u runnerup_reverse=%u "
                          "support=%u positive=%u eligible=%u "
                          "reason=%s published=0\n",
                  (UINT)g_sequence, (INT)('A' + p_match->letter),
                  (INT)((p_match->runner_letter < 26U) ?
                        'A' + p_match->runner_letter : '?'),
                  (INT)p_match->score_z100, (INT)p_match->runner_score_z100,
                  (INT)p_match->margin_z100, (UINT)p_match->reverse,
                  (UINT)p_match->runner_reverse, (UINT)p_match->support,
                  (UINT)p_match->positive, (UINT)p_match->eligible, reason);
    }
    else
    {
        tm_printf((UB *)"[BARCODE UNREADABLE] pass=%u reason=%s "
                          "published=0\n", (UINT)g_sequence, reason);
    }
}
#endif

/** @brief True if the capture holds a plausible set of strong bars. */
static uint8_t wave_barcode_like (uint32_t * p_bars, uint32_t * p_widest)
{
    uint32_t peak = 0U;
    for (uint32_t i = 0U; i < g_count; ++i)
    {
        uint32_t value = (g_smooth[i] > g_floor[i]) ?
                         (uint32_t)(g_smooth[i] - g_floor[i]) : 0U;
        if (value > peak)
        {
            peak = value;
        }
    }
    uint32_t const noise = (g_white_sigma > 4U) ? g_white_sigma : 4U;
    uint32_t const on = peak / 2U;
    uint32_t const off = peak * 3U / 8U;
    uint32_t bars = 0U;
    uint32_t widest = 0U;
    uint32_t start = 0U;
    uint8_t black = 0U;
    for (uint32_t i = 0U; i < g_count; ++i)
    {
        uint32_t value = (g_smooth[i] > g_floor[i]) ?
                         (uint32_t)(g_smooth[i] - g_floor[i]) : 0U;
        uint32_t units = wave_row(i)->units;
        if ((0U == black) && (value >= on))
        {
            black = 1U;
            start = units;
            ++bars;
        }
        else if ((0U != black) && ((value <= off) || (i + 1U == g_count)))
        {
            black = 0U;
            if (units - start > widest)
            {
                widest = units - start;
            }
        }
    }
    *p_bars = bars;
    *p_widest = widest;
    return (uint8_t)((peak >= noise * BC_WAVE_LIKE_MIN_SIGMAS) &&
                     (bars >= BC_WAVE_LIKE_MIN_BARS) &&
                     (bars <= BC_WAVE_LIKE_MAX_BARS) &&
                     (widest <= BC_WAVE_LIKE_MAX_BAR_UNITS));
}

#if APP_BARCODE_LAYOUT_ENABLED
static void wave_layout_print (bc_layout_result_t const * p_layout)
{
    tm_printf((UB *)"[BC LAYOUT] pass=%u valid=%u letter=%c reverse=%u "
                      "distance=%s reason=%s other=%c runner=%c bars=%u "
                      "err_centi=%u module_x10=%u gap_permille=%u\n",
              (UINT)g_sequence, (UINT)p_layout->valid,
              (INT)((p_layout->letter < 26U) ? 'A' + p_layout->letter : '?'),
              (UINT)p_layout->reverse,
              (0U != p_layout->variant) ? "sensor" : "centre",
              bc_layout_reason_text(p_layout->reason),
              (INT)((p_layout->other < 26U) ? 'A' + p_layout->other : '-'),
              (INT)((p_layout->runner < 26U) ? 'A' + p_layout->runner : '-'),
              (UINT)p_layout->bars, (UINT)p_layout->err_centi,
              (UINT)p_layout->module_x10, (UINT)p_layout->gap_permille);
}
#endif

static void wave_report (void)
{
    uint32_t best = 0U;
    uint32_t runner = 0U;
    for (uint32_t i = 1U; i < 26U; ++i)
    {
        if (g_votes[i] > g_votes[best])
        {
            best = i;
        }
    }
    for (uint32_t i = 0U; i < 26U; ++i)
    {
        if ((i != best) && (g_votes[i] > runner))
        {
            runner = g_votes[i];
        }
    }
    uint8_t valid = (g_votes[best] >= APP_BARCODE_WAVE_MIN_VOTES) &&
                    (g_votes[best] >= 2U * (runner + 1U)) &&
                    (runner < 2U) &&
                    (0U == g_adc_errors) &&
                    (g_gap_max <= BC_WAVE_MAX_GAP_USEC);
    uint8_t selected_reverse = g_reverse[best];
    uint8_t source = valid ? 1U : 0U;
#if APP_BARCODE_TEMPLATE_ENABLED
    bc_match_result_t matched = bc_match_result();
    if (0U != matched.valid)
    {
        if ((0U != valid) && (best != matched.letter))
        {
            source = 3U;
            valid = 0U;
        }
        else if (0U == valid)
        {
            best = matched.letter;
            selected_reverse = matched.reverse;
            source = 2U;
            valid = 1U;
        }
    }
#endif
    /* Neither template search nor its score can bypass missing-data gates. */
    valid = valid && (0U == g_adc_errors) &&
            (g_gap_max <= BC_WAVE_MAX_GAP_USEC);
    uint8_t comparison = 0U; /* neither */
    if (3U == source)
    {
        comparison = 4U;
    }
    else if ((0U != valid) && (0U != g_legacy.valid))
    {
        comparison = (best == g_legacy.letter) ? 1U : 4U;
        if (4U == comparison)
        {
            valid = 0U;
        }
    }
    else if (0U != valid)
    {
        comparison = 2U; /* buffered only */
    }
    else if (0U != g_legacy.valid)
    {
        comparison = 3U; /* legacy only */
    }
#if APP_BARCODE_LAYOUT_ENABLED
    {
        /* The layout decoder decides. Missing-data gates still apply. */
        bc_layout_result_t const layout = bc_layout_result();
        wave_layout_print(&layout);
        valid = (uint8_t)((0U != layout.valid) && (0U == g_adc_errors) &&
                          (g_gap_max <= BC_WAVE_MAX_GAP_USEC));
        if (0U != layout.valid)
        {
            best = layout.letter;
            selected_reverse = layout.reverse;
        }
        source = 4U;
        g_last_reason = layout.reason;
    }
#endif
    char const * const comparisons[5] = {
        "neither", "both_same", "buffered_only", "legacy_only", "disagree"
    };
    char const * const sources[5] = {
        "none", "segmentation", "template", "conflict", "layout"
    };
    tm_printf((UB *)"[BC WAVE RESULT] pass=%u valid=%u letter=%u "
                      "candidate=%c reverse=%u votes=%u runner=%u "
                      "fit_pct=%u compare=%s source=%s "
                      "busy_samples=%u capture_ms=%u process_ms=%u\n",
              (UINT)g_sequence, (UINT)valid,
              (UINT)((0U != valid) ? best : BC_CODE_LETTER_INVALID),
              (INT)(((0U != g_votes[best]) || (2U == source) ||
                     (0U != valid)) ? 'A' + best : '?'),
              (UINT)selected_reverse, (UINT)g_votes[best], (UINT)runner,
              (UINT)g_fit[best], comparisons[comparison], sources[source],
              (UINT)g_busy_samples, (UINT)g_capture_msec,
              (UINT)((encoder_now_usec() - g_freeze_usec) / 1000U));
    {
        uint32_t const process_ms =
            (encoder_now_usec() - g_freeze_usec) / 1000U;
        g_last_valid = valid;
        g_last_letter = (uint8_t)((0U != valid) ?
                                  best : BC_CODE_LETTER_INVALID);
        g_last_process_ms = (uint16_t)((process_ms > 65535U) ?
                                       65535U : process_ms);
        g_last_sequence = (uint16_t)g_sequence;
    }
    if (0U != valid)
    {
        bc_code_result_t result = {
            (uint8_t)best, (uint8_t)((2U == source) ? 1U : 2U),
            selected_reverse, 1U
        };
        ER error = bc_port_publish_buffered(&result, g_trigger_msec);
        tm_printf((UB *)"[BC WAVE PUBLISH] pass=%u status=%d\n",
                  (UINT)g_sequence, error);
#ifndef B1_LIVE_COMMANDS
        for (uint32_t i = 0U;
             (1U == source) && (i < BC_CODE_ELEMENTS); ++i)
        {
            tm_printf((UB *)"[BC WAVE WIDTH] pass=%u i=%u units=%u\n",
                      (UINT)g_sequence, (UINT)i, (UINT)g_widths[best][i]);
        }
#endif
        if (2U == source)
        {
            tm_printf((UB *)"[BC WAVE TEMPLATE] Analog template accepted; "
                              "no measured 29-run widths claimed\n");
        }
    }
    else
    {
        uint32_t like_bars = 0U;
        uint32_t like_widest = 0U;
        if (0U == wave_barcode_like(&like_bars, &like_widest))
        {
            /* Not a barcode attempt: no failure is counted or sent. */
            ++g_ignored;
            g_last_reason = BC_WAVE_REASON_IGNORED;
            tm_printf((UB *)"[BARCODE] Ignored capture %u: not barcode-like "
                              "(strong_bars=%u widest=%u units); not a scan "
                              "failure\n",
                      (UINT)g_ignored, (UINT)like_bars, (UINT)like_widest);
        }
        else
        {
        uint8_t error_code = BC_SCAN_ERROR_PATTERN;
        if (g_freeze_reason == 3U) error_code = BC_SCAN_ERROR_TIMEOUT;
        else if (g_adc_errors || g_invalid) error_code = BC_SCAN_ERROR_INPUT;
        else if (g_run_overflow) error_code = BC_SCAN_ERROR_OVERFLOW;
        (void)bc_port_publish_buffered_rejection(error_code,
                                                 g_trigger_msec);
        tm_printf((UB *)"[BARCODE] Buffered pass unconfirmed; "
                          "failure counter published to MQTT\n");
#if APP_BARCODE_TEMPLATE_ENABLED && APP_BARCODE_ESTIMATES_ENABLED
        wave_estimate(&matched, (uint8_t)(4U == comparison));
#endif
        }
    }
#if APP_BARCODE_WAVE_DUMP_ENABLED
    g_dump_index = 0U;
    g_dump_usec = 0U;
    g_dump_left = 0U;
    g_phase = BC_WAVE_DUMP;
    tm_printf((UB *)"[BC WAVE RAW BEGIN] pass=%u rows=%u origin_units=%u "
                      "time_valid=%u\n",
              (UINT)g_sequence, (UINT)g_count, (UINT)wave_row(0U)->units,
              (UINT)(g_gap_max < 65536U));
#ifdef B1_BARCODE_TEST
    tm_printf((UB *)"[BC WAVE RAW COLUMNS] "
                      "pass,i,dt_us,units,adc,smooth,floor,contrast,left\n");
#else
    tm_printf((UB *)"[BC WAVE RAW COLUMNS] "
                      "pass,i,dt_us,units,adc,smooth,floor,contrast\n");
#endif
#else
    g_phase = BC_WAVE_DONE;
#ifdef B1_LIVE_COMMANDS
    /* No persistent one-shot hold in autonomous mode. The scanner's owner
     * rearms after decoding while the controller owns all motor decisions. */
    g_rearm_requested = 1U;
    tm_printf((UB *)"[BC WAVE DONE] Auto rearm queued\n");
#else
    tm_printf((UB *)"[BC WAVE DONE] Capture complete; explicit START "
                    "required to rearm\n");
#endif
#endif
}

#if APP_BARCODE_WAVE_DUMP_ENABLED
static void wave_dump (void)
{
    /* Bounded output, after collection and result publication. Samples,
     * envelopes and the motor hold remain frozen throughout this dump.
     */
    for (uint32_t batch = 0U;
         (batch < BC_WAVE_DUMP_BATCH) && (g_dump_index < g_count); ++batch)
    {
        bc_wave_row_t const * const p_row = wave_row(g_dump_index);
        if (g_dump_index > 0U)
        {
            g_dump_usec += (uint16_t)(p_row->usec -
                                      wave_row(g_dump_index - 1U)->usec);
            g_dump_left += p_row->left_step;
        }
#ifdef B1_BARCODE_TEST
        tm_printf((UB *)"[BC WAVE RAW] %u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                  (UINT)g_sequence, (UINT)g_dump_index, (UINT)g_dump_usec,
                  (UINT)(p_row->units - wave_row(0U)->units),
                  (UINT)p_row->adc, (UINT)g_smooth[g_dump_index],
                  (UINT)g_floor[g_dump_index],
                  (UINT)g_contrast[g_dump_index],
                  (UINT)(uint16_t)g_dump_left);
#else
        tm_printf((UB *)"[BC WAVE RAW] %u,%u,%u,%u,%u,%u,%u,%u\n",
                  (UINT)g_sequence, (UINT)g_dump_index, (UINT)g_dump_usec,
                  (UINT)(p_row->units - wave_row(0U)->units),
                  (UINT)p_row->adc, (UINT)g_smooth[g_dump_index],
                  (UINT)g_floor[g_dump_index],
                  (UINT)g_contrast[g_dump_index]);
#endif
        ++g_dump_index;
    }
    if (g_dump_index == g_count)
    {
        tm_printf((UB *)"[BC WAVE RAW END] pass=%u rows=%u\n",
                  (UINT)g_sequence, (UINT)g_dump_index);
        g_phase = BC_WAVE_DONE;
#ifdef B1_BARCODE_TEST
        g_rearm_requested = 1U;
        tm_printf((UB *)"[BC WAVE DONE] Test mode: re-arming for the next "
                          "pass\n");
#else
        tm_printf((UB *)"[BC WAVE DONE] Capture complete; explicit START "
                          "required to rearm\n");
#endif
    }
}
#endif

void bc_wave_poll (void)
{
    static bc_wave_phase_t reported = BC_WAVE_REPORT;
#ifdef B1_LIVE_COMMANDS
    /* Buffer processing is complete before reset; never free raw sample
     * storage while a matcher is using it. */
    if (g_phase == BC_WAVE_DONE) g_rearm_requested = 1U;
#endif
    if ((0U != g_relearn_requested) &&
        ((g_phase <= BC_WAVE_CAPTURE) || (BC_WAVE_CLEAR == g_phase) ||
         (BC_WAVE_DONE == g_phase)))
    {
        /* Remote recalibration: drop any capture (never a decode in
         * progress) and learn white again from the next samples. */
        g_relearn_requested = 0U;
        g_phase = BC_WAVE_DONE;
        g_rearm_requested = 1U;
        wave_rearm_completed_capture();
        g_white_mean = 0U;
        g_white_sigma = 0U;
        g_white_dark = 0U;
        g_phase = BC_WAVE_LEARN;
        tm_printf((UB *)"[BC WAVE] Recalibration requested: "
                          "learning white\n");
    }
    wave_rearm_completed_capture();
    {
        static uint32_t white_rejects_reported;
        if (white_rejects_reported != g_white_rejects)
        {
            white_rejects_reported = g_white_rejects;
            tm_printf((UB *)"[BC WAVE] Sensor not on white (read %u); "
                              "waiting for white before arming\n",
                      (UINT)g_dark_adc);
        }
    }
    if ((BC_WAVE_CAPTURE == g_phase) && (BC_WAVE_CAPTURE == reported) &&
        ((uint32_t)(encoder_now_usec() - g_capture_seen_usec) >=
         BC_WAVE_CAPTURE_WATCHDOG_USEC))
    {
        tm_printf((UB *)"[BC WAVE] Capture stuck: pass=%u n=%u units=%u "
                          "trigger=%u last_signal=%u white=%u gap_us=%u; "
                          "restarting scan\n",
                  (UINT)g_sequence, (UINT)g_count, (UINT)g_units,
                  (UINT)g_trigger_units, (UINT)g_last_signal_units,
                  (UINT)g_white_mean, (UINT)g_gap_max);
        g_motion_hold = 0U;
        g_phase = BC_WAVE_DONE;
        g_rearm_requested = 1U;
    }
#ifdef B1_BARCODE_TEST
    if (BC_WAVE_CLEAR == g_phase)
    {
        /* Shows whether samples still arrive while waiting for white. */
        uint32_t const now = encoder_now_usec();
        if ((uint32_t)(now - g_clear_report_usec) >= 2000000U)
        {
            g_clear_report_usec = now;
            tm_printf((UB *)"[BC WAVE CLEAR] waiting for white: run=%u/%u "
                              "samples=%u fast_error=%d\n",
                      (UINT)g_clear_white_samples,
                      (UINT)BC_WAVE_WHITE_SAMPLES,
                      (UINT)g_clear_total_samples, (INT)bc_fast_error());
        }
    }
#endif
    if (reported != g_phase)
    {
        reported = g_phase;
        if (BC_WAVE_LEARN == g_phase)
        {
            tm_printf((UB *)"[BC WAVE] Learning white: keep sensor on white\n");
        }
        else if (BC_WAVE_WAIT == g_phase)
        {
            tm_printf((UB *)"[BC WAVE READY] white_mean=%u sigma=%u "
                              "min=%u max=%u; awaiting contrast and motion\n",
                      (UINT)g_white_mean, (UINT)g_white_sigma,
                      (UINT)g_white_min, (UINT)g_white_max);
        }
        else if (BC_WAVE_CAPTURE == g_phase)
        {
            g_capture_seen_usec = encoder_now_usec();
            tm_printf((UB *)"[BC WAVE START] pass=%u units=%u\n",
                      (UINT)g_sequence, (UINT)g_trigger_units);
        }
        else if (BC_WAVE_SMOOTH == g_phase)
        {
            uint32_t span = wave_row(g_count - 1U)->units - wave_row(0)->units;
            tm_printf((UB *)"[BC WAVE FROZEN] pass=%u reason=%u n=%u "
                              "span=%u gap_us=%u invalid=%u adc_errors=%u "
                              "backward=%u saturated=%u capture_ms=%u "
                              "tail_quiet_units=%u\n",
                      (UINT)g_sequence, (UINT)g_freeze_reason, (UINT)g_count,
                      (UINT)span,
                      (UINT)g_gap_max, (UINT)g_invalid, (UINT)g_adc_errors,
                      (UINT)g_backward, (UINT)g_saturated,
                      (UINT)g_capture_msec,
                      (UINT)(g_units - g_last_signal_units));
        }
    }
    if (BC_WAVE_SMOOTH == g_phase)
    {
        wave_smooth();
    }
    else if ((BC_WAVE_ENVELOPE == g_phase) || (BC_WAVE_UPPER == g_phase))
    {
        wave_envelope();
    }
    else if (BC_WAVE_LEGACY == g_phase)
    {
        wave_legacy();
    }
    else if (BC_WAVE_SEARCH == g_phase)
    {
        wave_search();
    }
#if APP_BARCODE_TEMPLATE_ENABLED
    else if (BC_WAVE_MATCH_INIT == g_phase)
    {
        bc_match_begin(g_count, g_white_sigma, wave_match_read,
                       g_workspace.integral, BC_WAVE_CAPACITY / 2U + 1U);
        g_phase = BC_WAVE_MATCH;
    }
    else if (BC_WAVE_MATCH == g_phase)
    {
        if (0U != bc_match_poll())
        {
            g_phase = BC_WAVE_LAYOUT_INIT;
        }
    }
#endif
    else if (BC_WAVE_LAYOUT_INIT == g_phase)
    {
#if APP_BARCODE_LAYOUT_ENABLED
        bc_layout_begin(g_count, wave_layout_read, 1U);
        g_phase = BC_WAVE_LAYOUT;
#else
        g_phase = BC_WAVE_REPORT;
#endif
    }
    else if (BC_WAVE_LAYOUT == g_phase)
    {
        if (0U != bc_layout_poll())
        {
            g_phase = BC_WAVE_REPORT;
        }
    }
    else if (BC_WAVE_REPORT == g_phase)
    {
        wave_report();
    }
#if APP_BARCODE_WAVE_DUMP_ENABLED
    else if (BC_WAVE_DUMP == g_phase)
    {
        wave_dump();
    }
#endif
}
#else
uint8_t bc_wave_motion_hold (void)
{
    return 0U;
}
uint8_t bc_wave_can_rearm (void)
{
    return 1U;
}
uint8_t bc_wave_capture_active (void)
{
    return 0U;
}
uint8_t bc_wave_navigation_guard_active(void)
{
    return 0U;
}
void bc_wave_request_rearm (void)
{
}
void bc_wave_set_suppressed (uint8_t b_suppressed)
{
    (void)b_suppressed;
}
void bc_wave_request_relearn (void)
{
}
uint8_t bc_wave_decode_pending (void)
{
    return 0U;
}
void bc_wave_status (bc_wave_status_t * p_status)
{
    if (0 != p_status)
    {
        *p_status = (bc_wave_status_t){0U};
        p_status->state = BC_WAVE_STATE_OFF;
        p_status->last_letter = 31U;
    }
}

void bc_wave_sample (bc_fastq_sample_t const * p_sample)
{
    (void)p_sample;
}

void bc_wave_poll (void)
{
}
#endif

/*** end of file ***/

