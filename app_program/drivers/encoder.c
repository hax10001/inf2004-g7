/** @file encoder.c
 * @brief Count quadrature edges in a minimal GPIO interrupt handler.
 *
 * The supplied BSP has no GPIO interrupt API, so this driver programs the
 * RP2040 IO_BANK0 interrupt registers through the kernel's register access.
 */
#include "encoder.h"
#include "board/board_config.h"
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

/* RP2040 registers that the kernel's sysdef.h does not name. */
#define ENC_IO_IRQ_BANK0 13U
#define ENC_INT_PRIORITY 2
#define ENC_INTR_BASE (IO_BANK0_BASE + 0x0F0U)
#define ENC_PROC0_INTE_BASE (IO_BANK0_BASE + 0x100U)
#define ENC_PROC_STRIDE 0x30U
#define ENC_INTS_OFFSET 0x20U
#define ENC_SIO_CPUID (SIO_BASE + 0x000U)
#define ENC_WATCHDOG_TICK 0x4005802CU
#define ENC_TICK_ENABLE (1U << 9)

/* Each GPIO has four event bits; eight GPIOs share one 32-bit register. */
#define ENC_BANK_OFFSET(pin) (((pin) / 8U) * 4U)
#define ENC_EDGE_LOW(pin) (1U << ((((pin) % 8U) * 4U) + 2U))
#define ENC_EDGE_HIGH(pin) (1U << ((((pin) % 8U) * 4U) + 3U))
#define ENC_EDGES(pin) (ENC_EDGE_LOW(pin) | ENC_EDGE_HIGH(pin))

typedef struct
{
    UINT a_pin;
    UINT b_pin;
} encoder_pins_t;

typedef struct
{
    volatile int32_t  count;
    volatile uint32_t edge_usec;
} encoder_state_t;

static encoder_pins_t const g_pins[ENCODER_WHEEL_COUNT] = {
    {BOARD_ENC_LEFT_A_PIN, BOARD_ENC_LEFT_B_PIN},
    {BOARD_ENC_RIGHT_A_PIN, BOARD_ENC_RIGHT_B_PIN}};

static encoder_state_t g_state[ENCODER_WHEEL_COUNT];
/* INTS register block of the core whose NVIC services the interrupt. */
static UW g_ints_base;

/** @brief Count each pending A edge; B's level gives the direction. */
static void encoder_isr (UINT intno);
/** @brief Start the 1 MHz TIMER if the kernel has left it in reset. */
static void start_usec_timer (void);
/** @brief Make one pin a pulled-up SIO input, taking it from UART or ADC. */
static ER claim_input (UINT pin);

ER encoder_init (void)
{
    start_usec_timer();

    for (uint32_t wheel = 0U; wheel < (uint32_t)ENCODER_WHEEL_COUNT; wheel++)
    {
        ER error = claim_input(g_pins[wheel].a_pin);

        if (error >= E_OK)
        {
            error = claim_input(g_pins[wheel].b_pin);
        }

        if (error < E_OK)
        {
            /* UB views the same bytes; tm_printf does not modify them. */
            tm_printf((UB *)"[ENCODER] Pin setup failed: %d\n", error);
            return error;
        }

        g_state[wheel].count     = 0;
        g_state[wheel].edge_usec = encoder_now_usec();
        out_w(ENC_INTR_BASE + ENC_BANK_OFFSET(g_pins[wheel].a_pin),
              ENC_EDGES(g_pins[wheel].a_pin));
    }

    T_DINT const dint = {.intatr = TA_HLNG, .inthdr = (FP)encoder_isr};
    ER           error = tk_def_int(ENC_IO_IRQ_BANK0, &dint);

    if (error < E_OK)
    {
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[ENCODER] Interrupt definition failed: %d\n", error);
        return error;
    }

    /* WARNING: NVIC and INTE are per core; stay on one core while pairing. */
    UINT intsts = 0U;
    DI(intsts);
    UW const inte_base =
        ENC_PROC0_INTE_BASE + (in_w(ENC_SIO_CPUID) * ENC_PROC_STRIDE);
    g_ints_base = inte_base + ENC_INTS_OFFSET;

    for (uint32_t wheel = 0U; wheel < (uint32_t)ENCODER_WHEEL_COUNT; wheel++)
    {
        set_w(inte_base + ENC_BANK_OFFSET(g_pins[wheel].a_pin),
              ENC_EDGES(g_pins[wheel].a_pin));
    }

    ClearInt(ENC_IO_IRQ_BANK0);
    EnableInt(ENC_IO_IRQ_BANK0, ENC_INT_PRIORITY);
    EI(intsts);

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[ENCODER INIT] Left GP%d/GP%d, right GP%d/GP%d.\n",
              (INT)BOARD_ENC_LEFT_A_PIN, (INT)BOARD_ENC_LEFT_B_PIN,
              (INT)BOARD_ENC_RIGHT_A_PIN, (INT)BOARD_ENC_RIGHT_B_PIN);
    return E_OK;
}

void encoder_read (encoder_wheel_t wheel, encoder_sample_t * p_sample)
{
    if ((uint32_t)wheel >= (uint32_t)ENCODER_WHEEL_COUNT)
    {
        return;
    }

    encoder_state_t const * p_state = &g_state[wheel];
    int32_t                 count   = 0;
    uint32_t                edge    = 0U;

    /* The ISR stores the time before the count, so a stable count means the
     * time belongs to it. This also holds when the ISR runs on another core.
     */
    do
    {
        count = p_state->count;
        edge  = p_state->edge_usec;
    } while (count != p_state->count);

    p_sample->count     = count;
    p_sample->edge_usec = edge;
}

uint32_t encoder_now_usec (void)
{
    return in_w(TIMER_TIMERAWL);
}

static void encoder_isr (UINT intno)
{
    uint32_t const now  = in_w(TIMER_TIMERAWL);
    UW const       gpio = in_w(GPIO_IN);

    for (uint32_t wheel = 0U; wheel < (uint32_t)ENCODER_WHEEL_COUNT; wheel++)
    {
        UINT const a_pin  = g_pins[wheel].a_pin;
        UW const   offset = ENC_BANK_OFFSET(a_pin);
        UW const   edges  = in_w(g_ints_base + offset) & ENC_EDGES(a_pin);

        if (0U != edges)
        {
            /* Edge bits are write-one-to-clear. */
            out_w(ENC_INTR_BASE + offset, edges);

            uint32_t const b     = (gpio >> g_pins[wheel].b_pin) & 1U;
            int32_t        delta = 0;

            /* Quadrature: A equal to B after the edge is one direction. */
            if (0U != (edges & ENC_EDGE_HIGH(a_pin)))
            {
                delta += (1U == b) ? 1 : -1;
            }

            if (0U != (edges & ENC_EDGE_LOW(a_pin)))
            {
                delta += (0U == b) ? 1 : -1;
            }

            g_state[wheel].edge_usec = now;
            g_state[wheel].count     = g_state[wheel].count + delta;
        }
    }

    ClearInt(intno);
}

static void start_usec_timer (void)
{
    if (0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER))
    {
        clr_w(RESETS_RESET, RESETS_RESET_TIMER);

        while (0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER))
        {
        }
    }

    /* The TIMER counts watchdog ticks: divide the XOSC reference to 1 MHz. */
    out_w(ENC_WATCHDOG_TICK, ENC_TICK_ENABLE | (UW)XOSC_MHz);
}

static ER claim_input (UINT pin)
{
    ER const error = gpio_set_pin(pin, GPIO_MODE_IN);

    if (error >= E_OK)
    {
        clr_w(GPIO(pin), GPIO_PDE);
        set_w(GPIO(pin), GPIO_PUE);
    }

    return error;
}

/*** end of file ***/

