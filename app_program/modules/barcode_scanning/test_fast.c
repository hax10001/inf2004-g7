/** @file test_fast.c
 * @brief Portable queue and scheduler tests for 4 ms barcode sampling.
 */
#include "test_fast.h"
#include "bc_fastq.h"

#ifdef BC_FAST_HOST_TEST_MAIN
#include <pthread.h>
#include <stdio.h>

#define BC_FAST_TEST_EVENTS 12000U

static int g_checks = 0;
static int g_failures = 0;
static uint32_t g_thread_errors = 0U;

static void check (uint8_t condition, char const * p_description)
{
    ++g_checks;
    if (0U == condition)
    {
        ++g_failures;
        printf("FAIL: %s\n", p_description);
    }
}

static void test_deadlines (void)
{
    uint8_t valid = 1U;
    uint32_t deadline = 10000U;

    deadline = bc_fastq_next(deadline, 10000U, &valid);
    check(14000U == deadline, "exactly 4 ms ahead");
    check(1U == valid, "on-time valid");
    deadline = bc_fastq_next(deadline, 14800U, &valid);
    check(18000U == deadline, "small delay keeps 4 ms phase");
    check(1U == valid, "under 1 ms jitter remains valid");
    valid = 1U;
    deadline = bc_fastq_next(deadline, 19400U, &valid);
    check(22000U == deadline, "late interrupt phase advanced");
    check(0U == valid, "excessive latency invalid");
    valid = 1U;
    deadline = bc_fastq_next(deadline, 33000U, &valid);
    check(37000U == deadline, "missed period reschedules ahead");
    check(0U == valid, "missed interval invalid");
    valid = 1U;
    deadline = bc_fastq_next(0xFFFFFF00U, 0xFFFFFF00U, &valid);
    check(0x00000EA0U == deadline, "timer wrap is supported");
    check(1U == valid, "wrap does not reject good sample");
}

static void test_fifo_and_overflow (void)
{
    bc_fastq_sample_t sample = {0};
    bc_fastq_sample_t result = {0};

    bc_fastq_reset();
    check(0U == bc_fastq_pop(&result), "empty queue");
    for (uint32_t i = 0U; i < BC_FASTQ_CAPACITY; ++i)
    {
        sample.time_usec = i * BC_FAST_PERIOD_USEC;
        sample.valid = 1U;
        check(1U == bc_fastq_push(&sample), "bounded enqueue");
    }
    check(0U == bc_fastq_push(&sample), "full queue rejects");
    for (uint32_t i = 0U; i < BC_FASTQ_CAPACITY; ++i)
    {
        check(1U == bc_fastq_pop(&result), "queued read");
        check(i * BC_FAST_PERIOD_USEC == result.time_usec,
              "strict FIFO order");
        if (0U == i)
        {
            check(0U == result.valid, "overflow marks invalid");
        }
    }
    check(0U == bc_fastq_pop(&result), "fully drained");
    bc_fastq_reset();
    sample.valid = 1U;
    check(1U == bc_fastq_push(&sample), "rearmed");
    check(1U == bc_fastq_pop(&result), "new read");
    check(1U == result.valid, "reset clears overflow state");
}

static void * producer (void * p_context)
{
    bc_fastq_sample_t sample = {0};
    (void)p_context;
    for (uint32_t i = 1U; i <= BC_FAST_TEST_EVENTS; ++i)
    {
        sample.time_usec = i;
        sample.left_count = (int32_t)i;
        sample.right_count = (int32_t)(i * 2U);
        sample.valid = 1U;
        /* Retrying is for host testing only; real ISR never retries. */
        while (0U == bc_fastq_push(&sample))
        {
        }
    }
    return 0;
}

static void * consumer (void * p_context)
{
    bc_fastq_sample_t sample = {0};
    (void)p_context;
    for (uint32_t i = 1U; i <= BC_FAST_TEST_EVENTS; ++i)
    {
        while (0U == bc_fastq_pop(&sample))
        {
        }
        if ((sample.time_usec != i) ||
            (sample.left_count != (int32_t)i) ||
            (sample.right_count != (int32_t)(i * 2U)))
        {
            ++g_thread_errors;
        }
    }
    return 0;
}

static void test_concurrency (void)
{
    pthread_t p;
    pthread_t c;
    int const p_status = pthread_create(&p, 0, producer, 0);
    int const c_status = pthread_create(&c, 0, consumer, 0);

    check(0 == p_status, "producer started");
    check(0 == c_status, "consumer started");
    if ((0 == p_status) && (0 == c_status))
    {
        (void)pthread_join(p, 0);
        (void)pthread_join(c, 0);
        check(0U == g_thread_errors, "no torn concurrent records");
    }
}

int test_fast_run (void)
{
    test_deadlines();
    test_fifo_and_overflow();
    bc_fastq_reset();
    test_concurrency();
    printf("Barcode fast sampling: %d assertions, %d failed\n",
           g_checks, g_failures);
    return (0 == g_failures) ? 0 : 1;
}
int main (void)
{
    return test_fast_run();
}
#endif

/*** end of file ***/

