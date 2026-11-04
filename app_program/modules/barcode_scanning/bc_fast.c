/** @file bc_fast.c
 * @brief RP2040 alarm-2 captures barcode ADC/GPIO and encoders each 2000 us.
 * @note Must reserve alarm 2 and TIMER_IRQ_2 with platform owner.
 */
#include "bc_fast.h"
#include "bc_fastq.h"
#include "bc_port.h"
#include "bc_trace.h"
#include "bc_wave.h"
#include "drivers/encoder.h"
#include "drivers/ir.h"
#include "app/app_config.h"
#include "board/board_config.h"
#include <bsp/libbsp.h>

static bc_fast_adc_stats_t g_adc_stats;

void bc_fast_adc_report (bc_fast_adc_stats_t * p_stats)
{
    *p_stats = g_adc_stats;
    g_adc_stats = (bc_fast_adc_stats_t){0};
}

#if defined(_PICO_RP2040_)

/* RP2040 TIMER_IRQ_2 has external interrupt number 2. */
#define BC_FAST_IRQ_NUMBER 2U
#define BC_FAST_ALARM_MASK (1U << 2U)
#define BC_FAST_PRIORITY 2
#define BC_FAST_ARM_MARGIN_USEC 100U

static uint8_t gb_fast_registered = 0U;
static uint8_t gb_fast_armed = 0U;
static uint32_t g_next_alarm_usec = 0U;
static ER volatile g_fast_error = E_OK;

#if APP_BARCODE_ANALOG_ENABLED && APP_BARCODE_EDGE_FILTER_ENABLED && \
    !APP_BARCODE_BUFFERED_ENABLED
/* Preserve the existing jitter allowance when sampling faster. */
#define BC_FAST_FILTER_MAX_GAP_USEC 8000U
static bc_fastq_sample_t g_filter_pending;
static uint8_t gb_filter_pending;
static uint8_t g_filter_level;
static uint8_t gb_filter_initialized;

/** Delay every sample by one observation so encoder time never moves
 * backwards. Two matching observations confirm a change at the FIRST
 * observation's position. An isolated pulse retains the previous colour.
 * Raw ADC diagnostics are collected before this function changes colour.
 */
static uint8_t bc_fast_filter (bc_fastq_sample_t * p_sample)
{
    bc_fastq_sample_t const current = *p_sample;
    uint8_t b_output = 0U;

    if (0U == gb_filter_initialized)
    {
        g_filter_level = current.gpio_level;
        gb_filter_initialized = 1U;
    }
    if (0U != gb_filter_pending)
    {
        uint32_t const gap = current.time_usec - g_filter_pending.time_usec;
        if ((g_filter_pending.gpio_level != g_filter_level) &&
            (current.gpio_level == g_filter_pending.gpio_level) &&
            (gap <= BC_FAST_FILTER_MAX_GAP_USEC))
        {
            g_filter_level = g_filter_pending.gpio_level;
        }
        else if (g_filter_pending.gpio_level != g_filter_level)
        {
            ++g_adc_stats.filtered_spikes;
        }
        *p_sample = g_filter_pending;
        p_sample->gpio_level = g_filter_level;
        b_output = 1U;
    }
    g_filter_pending = current;
    gb_filter_pending = 1U;
    return b_output;
}
#endif

/** @brief Program a future alarm, with local preemption excluded.
 * Recheck after writing: an overdue equality alarm would wait for wrap.
 * Two attempts bound interrupt work; a hardware failure disables capture.
 */
static uint8_t bc_fast_program (uint32_t deadline, uint8_t * p_valid)
{
    UINT intsts = 0U;
    DI(intsts);
    for (uint8_t attempt = 0U; attempt < 2U; ++attempt)
    {
        uint32_t now = encoder_now_usec();
        if ((int32_t)(deadline - now) <
            (int32_t)BC_FAST_ARM_MARGIN_USEC)
        {
            deadline = now + BC_FAST_PERIOD_USEC;
            *p_valid = 0U;
        }
        out_w(TIMER_ALARM2, deadline);
        now = encoder_now_usec();
        if ((int32_t)(deadline - now) > 0)
        {
            g_next_alarm_usec = deadline;
            EI(intsts);
            return 1U;
        }
        *p_valid = 0U;
        deadline = now + BC_FAST_PERIOD_USEC;
    }
    out_w(TIMER_ARMED, BC_FAST_ALARM_MASK);
    clr_w(TIMER_INTE, BC_FAST_ALARM_MASK);
    g_fast_error = E_OBJ;
    EI(intsts);
    return 0U;
}

static void bc_fast_isr (UINT interrupt_number)
{
    bc_fastq_sample_t sample = {0};
    encoder_sample_t left = {0};
    encoder_sample_t right = {0};
    uint8_t valid = 1U;
    uint32_t now_usec;

    /* Acknowledge the alarm before doing bounded capture work. */
    out_w(TIMER_INTR, BC_FAST_ALARM_MASK);
    encoder_read(ENCODER_LEFT, &left);
    encoder_read(ENCODER_RIGHT, &right);
    now_usec = encoder_now_usec();

    sample.time_usec = now_usec;
    sample.left_count = left.count;
    sample.right_count = right.count;
    sample.left_edge_usec = left.edge_usec;
    sample.right_edge_usec = right.edge_usec;
#if APP_BARCODE_ANALOG_ENABLED
    sample.adc_valid = ir_barcode_capture(&sample.adc_raw,
                                           &sample.gpio_level) ? 1U : 0U;
    if (0U == sample.adc_valid)
    {
        valid = 0U;
    }
#else
    sample.gpio_level = (uint8_t)(
        (in_w(GPIO_IN) >> BOARD_IR_BARCODE_PIN) & 1U);
#endif
    g_next_alarm_usec = bc_fastq_next(g_next_alarm_usec,
                                      now_usec, &valid);
    if (0U == bc_fast_program(g_next_alarm_usec, &valid))
    {
        valid = 0U;
    }
    sample.valid = valid;
    (void)bc_fastq_push(&sample);
    ClearInt(interrupt_number);
}

ER bc_fast_init (void)
{
    T_DINT handler = {0};
    ER error;

    if (0U != gb_fast_registered)
    {
        return E_OBJ;
    }
    /* Fail rather than steal an already armed/enabled hardware alarm. */
    if ((0U != (in_w(TIMER_INTE) & BC_FAST_ALARM_MASK)) ||
        (0U != (in_w(TIMER_ARMED) & BC_FAST_ALARM_MASK)))
    {
        return E_OBJ;
    }
    bc_fastq_reset();
#if APP_BARCODE_ANALOG_ENABLED && APP_BARCODE_EDGE_FILTER_ENABLED && \
    !APP_BARCODE_BUFFERED_ENABLED
    gb_filter_pending = 0U;
    gb_filter_initialized = 0U;
#endif
    handler.intatr = TA_HLNG;
    handler.inthdr = (FP)bc_fast_isr;
    error = tk_def_int(BC_FAST_IRQ_NUMBER, &handler);
    if (error < E_OK)
    {
        return error;
    }
    gb_fast_registered = 1U;
    return E_OK;
}

ER bc_fast_arm (void)
{
    if ((0U == gb_fast_registered) || (g_fast_error < E_OK))
    {
        return E_OBJ;
    }
    if (0U == gb_fast_armed)
    {
        uint8_t valid = 1U;
        /* Initialization and the first IR hook can be far apart. Recheck
         * ownership before touching an alarm enabled by another module.
         */
        if ((0U != (in_w(TIMER_INTE) & BC_FAST_ALARM_MASK)) ||
            (0U != (in_w(TIMER_ARMED) & BC_FAST_ALARM_MASK)))
        {
            return E_OBJ;
        }
        /* Caller has now observed one successful initialized IR read. */
        g_next_alarm_usec = encoder_now_usec() +
                            BC_FAST_PERIOD_USEC;
        out_w(TIMER_INTR, BC_FAST_ALARM_MASK);
        ClearInt(BC_FAST_IRQ_NUMBER);
        if (0U == bc_fast_program(g_next_alarm_usec, &valid))
        {
            return g_fast_error;
        }
        set_w(TIMER_INTE, BC_FAST_ALARM_MASK);
        EnableInt(BC_FAST_IRQ_NUMBER, BC_FAST_PRIORITY);
        gb_fast_armed = 1U;
    }
    return E_OK;
}

void bc_fast_stop (void)
{
    /* Registration alone does not mean this module armed the hardware. */
    if (0U == gb_fast_armed)
    {
        return;
    }
    clr_w(TIMER_INTE, BC_FAST_ALARM_MASK);
    out_w(TIMER_ARMED, BC_FAST_ALARM_MASK);
    ClearInt(BC_FAST_IRQ_NUMBER);
    gb_fast_armed = 0U;
}

ER bc_fast_error (void)
{
    return g_fast_error;
}

ER bc_fast_drain (void)
{
    bc_fastq_sample_t sample = {0};
    ER first_error = E_OK;

    /* Queue is bounded: at most 64 samples per diagnostic cycle. */
    for (uint32_t i = 0U; i < BC_FASTQ_CAPACITY; ++i)
    {
        ER error;
        uint8_t b_process_sample = 1U;
        if (0U == bc_fastq_pop(&sample))
        {
            break;
        }
#if APP_BARCODE_ANALOG_ENABLED && APP_BARCODE_TRACE_ENABLED
        bc_trace_record(&sample);
#endif
#if APP_BARCODE_ANALOG_ENABLED && APP_BARCODE_BUFFERED_ENABLED
        bc_wave_sample(&sample);
#endif
#if APP_BARCODE_ANALOG_ENABLED
        if (0U == sample.adc_valid)
        {
            /* Never feed an invented colour on an ADC hardware failure,
             * even with the experimental input-validity bypass enabled.
             */
            ++g_adc_stats.errors;
            b_process_sample = 0U;
#if APP_BARCODE_EDGE_FILTER_ENABLED && !APP_BARCODE_BUFFERED_ENABLED
            /* An unreadable observation cannot confirm a pending edge. */
            gb_filter_pending = 0U;
#endif
        }
        else
        {
            if (0U == g_adc_stats.samples)
            {
                g_adc_stats.minimum = sample.adc_raw;
                g_adc_stats.maximum = sample.adc_raw;
            }
            if (sample.adc_raw < g_adc_stats.minimum)
            {
                g_adc_stats.minimum = sample.adc_raw;
            }
            if (sample.adc_raw > g_adc_stats.maximum)
            {
                g_adc_stats.maximum = sample.adc_raw;
            }
            g_adc_stats.latest = sample.adc_raw;
            ++g_adc_stats.samples;
        }
#endif
#if APP_BARCODE_ANALOG_ENABLED && APP_BARCODE_BUFFERED_ENABLED
        /* Buffered mode consumes raw ADC; live fixed-threshold capture is
         * replayed offline on the same samples for the comparison log.
         */
        b_process_sample = 0U;
#endif
#if APP_BARCODE_ANALOG_ENABLED && APP_BARCODE_EDGE_FILTER_ENABLED && \
    !APP_BARCODE_BUFFERED_ENABLED
        if (0U != b_process_sample)
        {
            b_process_sample = bc_fast_filter(&sample);
        }
#endif
        if (0U != b_process_sample)
        {
            error = bc_port_sample_counts(
                sample.gpio_level, sample.time_usec,
                sample.left_count, sample.right_count,
                sample.left_edge_usec, sample.right_edge_usec,
                sample.valid);
            if ((error < E_OK) && (first_error >= E_OK))
            {
                first_error = error;
            }
            /* Consume terminal states before feeding the next sample.
             * Otherwise terminal states hide the rest of this batch,
             * including the next barcode's leading white region.
             */
            error = bc_port_process();
            if ((error < E_OK) && (first_error >= E_OK))
            {
                first_error = error;
            }
        }
    }
    return first_error;
}

#else
/* Host-only builds do not have memory-mapped RP2040 timer registers. */
void bc_fast_stop (void)
{
}

ER bc_fast_error (void)
{
    return E_OBJ;
}

ER bc_fast_init (void)
{
    return E_OBJ;
}

ER bc_fast_arm (void)
{
    return E_OBJ;
}

ER bc_fast_drain (void)
{
    return E_OK;
}
#endif

/*** end of file ***/

