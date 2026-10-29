/** @file bc_fast.c
 * @brief RP2040 TIMER ALARM2 captures GP27 and two encoders each 4000 us.
 * @note Must reserve alarm 2 and TIMER_IRQ_2 with platform owner.
 */
#include "bc_fast.h"
#include "bc_fastq.h"
#include "bc_port.h"
#include "drivers/encoder.h"
#include "board/board_config.h"
#include <bsp/libbsp.h>

#if defined(_PICO_RP2040_)

/* RP2040 TIMER_IRQ_2 has external interrupt number 2. */
#define BC_FAST_IRQ_NUMBER 2U
#define BC_FAST_ALARM_MASK (1U << 2U)
#define BC_FAST_PRIORITY 2

static uint8_t gb_fast_registered = 0U;
static uint8_t gb_fast_armed = 0U;
static uint32_t g_next_alarm_usec = 0U;

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
    sample.gpio_level = (uint8_t)(
        (in_w(GPIO_IN) >> BOARD_IR_BARCODE_PIN) & 1U);
    g_next_alarm_usec = bc_fastq_next(g_next_alarm_usec,
                                      now_usec, &valid);
    sample.valid = valid;
    (void)bc_fastq_push(&sample);
    out_w(TIMER_ALARM2, g_next_alarm_usec);
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
    if (0U == gb_fast_registered)
    {
        return E_OBJ;
    }
    if (0U == gb_fast_armed)
    {
        /* Caller has now observed one successful initialized IR read. */
        g_next_alarm_usec = encoder_now_usec() +
                            BC_FAST_PERIOD_USEC;
        out_w(TIMER_INTR, BC_FAST_ALARM_MASK);
        ClearInt(BC_FAST_IRQ_NUMBER);
        out_w(TIMER_ALARM2, g_next_alarm_usec);
        set_w(TIMER_INTE, BC_FAST_ALARM_MASK);
        EnableInt(BC_FAST_IRQ_NUMBER, BC_FAST_PRIORITY);
        gb_fast_armed = 1U;
    }
    return E_OK;
}

ER bc_fast_drain (void)
{
    bc_fastq_sample_t sample = {0};
    ER first_error = E_OK;

    /* Queue is bounded: at most 32 samples per diagnostic cycle. */
    for (uint32_t i = 0U; i < BC_FASTQ_CAPACITY; ++i)
    {
        ER error;
        if (0U == bc_fastq_pop(&sample))
        {
            break;
        }
        error = bc_port_sample_counts(
            sample.gpio_level, sample.time_usec,
            sample.left_count, sample.right_count,
            sample.left_edge_usec, sample.right_edge_usec,
            sample.valid);
        if ((error < E_OK) && (first_error >= E_OK))
        {
            first_error = error;
        }
    }
    return first_error;
}

#else
/* Host-only builds do not have memory-mapped RP2040 timer registers. */
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

