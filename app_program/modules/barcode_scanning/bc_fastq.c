/** @file bc_fastq.c
 * @brief Bounded single-producer single-consumer cross-core sample queue.
 */
#include "bc_fastq.h"

#define BC_FASTQ_MASK (BC_FASTQ_CAPACITY - 1U)

static bc_fastq_sample_t g_samples[BC_FASTQ_CAPACITY];
/* RP2040 uses single-copy atomic, aligned 32-bit word accesses.
 * DMB orders the record fields before publishing an index on both cores.
 */
static volatile uint32_t g_write_index = 0U;
static volatile uint32_t g_read_index = 0U;
static volatile uint32_t g_drop_count = 0U;
static uint32_t g_last_drop_seen = 0U;

static void bc_fastq_barrier (void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile ("dmb" ::: "memory");
#else
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#endif
}

void bc_fastq_reset (void)
{
    g_write_index = 0U;
    g_read_index = 0U;
    g_drop_count = 0U;
    g_last_drop_seen = 0U;
}

uint8_t bc_fastq_push (bc_fastq_sample_t const * p_sample)
{
    uint32_t write_index;
    uint32_t read_index;

    if (0 == p_sample)
    {
        return 0U;
    }
    write_index = g_write_index;
    read_index = g_read_index;
    bc_fastq_barrier();
    if ((uint32_t)(write_index - read_index) >= BC_FASTQ_CAPACITY)
    {
        ++g_drop_count;
        return 0U;
    }
    g_samples[write_index & BC_FASTQ_MASK] = *p_sample;
    bc_fastq_barrier();
    g_write_index = write_index + 1U;
    return 1U;
}

uint8_t bc_fastq_pop (bc_fastq_sample_t * p_sample)
{
    uint32_t read_index;
    uint32_t write_index;

    if (0 == p_sample)
    {
        return 0U;
    }
    read_index = g_read_index;
    write_index = g_write_index;
    bc_fastq_barrier();
    if (read_index == write_index)
    {
        return 0U;
    }
    *p_sample = g_samples[read_index & BC_FASTQ_MASK];
    /* A dropped sample makes the next observed scan invalid. */
    if (g_last_drop_seen != g_drop_count)
    {
        g_last_drop_seen = g_drop_count;
        p_sample->valid = 0U;
    }
    bc_fastq_barrier();
    g_read_index = read_index + 1U;
    return 1U;
}

uint32_t bc_fastq_next (uint32_t deadline, uint32_t now_usec,
                        uint8_t * p_valid)
{
    uint32_t next = deadline + BC_FAST_PERIOD_USEC;

    if (0 != p_valid)
    {
        if ((int32_t)(now_usec - deadline) >
            (int32_t)BC_FAST_MAX_LATE_USEC)
        {
            *p_valid = 0U;
        }
        if ((int32_t)(now_usec - next) >= 0)
        {
            /* Do not burst-catch-up or accept a missed sample. */
            next = now_usec + BC_FAST_PERIOD_USEC;
            *p_valid = 0U;
        }
    }
    return next;
}

/*** end of file ***/

