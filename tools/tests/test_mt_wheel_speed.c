/** @file test_mt_wheel_speed.c
 * @brief Host test (FR7 wheel speed): the real wheel_speed.c edge-timed
 *        RPM, its stop timeout, direction handling, wraparound and reset.
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "modules/motion/wheel_speed.h"
#include "board/board_config.h"

/* Float comparison tolerance, RPM. */
#define T_EPS 0.001f
/* 88 edges in 50 ms with 1056 counts per revolution is exactly 100 RPM. */
#define T_EDGES 88
#define T_SPAN_USEC 50000U

/** @brief True when two floats agree within T_EPS. */
static bool near (float a, float b)
{
    return fabsf(a - b) < T_EPS;
}

/** @brief RPM for @p edges over @p usec, from first principles. */
static float expect_rpm (float edges, float usec)
{
    return (edges * 60000000.0f) / (usec * (float)BOARD_ENC_COUNTS_PER_REV);
}

/** @brief Edge timing gives the exact rate; sign of travel is ignored. */
static void test_rate_and_direction (void)
{
    wheel_speed_t w;
    float         rpm = 0.0f;

    wheel_speed_reset(&w, 1000, 5000U);
    assert(0.0f == w.rpm);
    rpm = wheel_speed_update(&w, 1000 + T_EDGES, 5000U + T_SPAN_USEC,
                             5000U + T_SPAN_USEC);
    assert(near(rpm, 100.0f));
    assert(near(rpm, expect_rpm((float)T_EDGES, (float)T_SPAN_USEC)));
    /* At 100 RPM an edge comes every 568 us: 500 us without one says
     * nothing, 1000 us without one caps the speed at one edge per ms. */
    rpm = wheel_speed_update(&w, 1000 + T_EDGES, 5000U + T_SPAN_USEC,
                             5000U + T_SPAN_USEC + 500U);
    assert(near(rpm, 100.0f));
    rpm = wheel_speed_update(&w, 1000 + T_EDGES, 5000U + T_SPAN_USEC,
                             5000U + T_SPAN_USEC + 1000U);
    assert(near(rpm, expect_rpm(1.0f, 1000.0f)));
    puts("PASS wheel speed: 88 edges in 50 ms = 100 RPM; 1 ms without an "
         "edge caps it at 56.8 RPM");

    /* Reverse (or the mirrored left encoder): same positive speed. */
    wheel_speed_reset(&w, 0, 0U);
    rpm = wheel_speed_update(&w, -T_EDGES, T_SPAN_USEC, T_SPAN_USEC);
    assert(near(rpm, 100.0f) && (rpm > 0.0f));
    puts("PASS wheel speed: counting down reports +100 RPM, never "
         "negative");
}

/** @brief Restart from rest: the first interval is not a measurement. */
static void test_restart_boundary (void)
{
    wheel_speed_t w;

    /* Exactly WHEEL_SPEED_STOP_USEC since the last edge is measured. */
    wheel_speed_reset(&w, 0, 0U);
    assert(near(wheel_speed_update(&w, 1, WHEEL_SPEED_STOP_USEC,
                                   WHEEL_SPEED_STOP_USEC),
                expect_rpm(1.0f, (float)WHEEL_SPEED_STOP_USEC)));
    /* One microsecond longer is a restart: zero, not a tiny speed. */
    wheel_speed_reset(&w, 0, 0U);
    assert(0.0f == wheel_speed_update(&w, 1, WHEEL_SPEED_STOP_USEC + 1U,
                                      WHEEL_SPEED_STOP_USEC + 1U));
    /* The restart sample becomes the reference: next interval counts. */
    assert(near(wheel_speed_update(&w, 1 + T_EDGES,
                                   WHEEL_SPEED_STOP_USEC + 1U + T_SPAN_USEC,
                                   WHEEL_SPEED_STOP_USEC + 1U + T_SPAN_USEC),
                100.0f));
    /* Edges stamped in the same microsecond as the reference: no
     * division by zero, reported as a restart. */
    wheel_speed_reset(&w, 0, 777U);
    assert(0.0f == wheel_speed_update(&w, 5, 777U, 777U));
    puts("PASS wheel speed: 200000 us gap measured, 200001 us restarts "
         "at 0, zero interval safe");
}

/** @brief No edges: decay to the ceiling, then zero after the timeout. */
static void test_timeout (void)
{
    wheel_speed_t w;
    float         rpm = 0.0f;

    wheel_speed_reset(&w, 0, 0U);
    (void)wheel_speed_update(&w, T_EDGES, T_SPAN_USEC, T_SPAN_USEC);
    /* Same microsecond as the last edge: estimate kept. */
    assert(near(wheel_speed_update(&w, T_EDGES, T_SPAN_USEC, T_SPAN_USEC),
                100.0f));
    /* 100 ms with no edge: at most one edge per 100 ms. */
    rpm = wheel_speed_update(&w, T_EDGES, T_SPAN_USEC,
                             T_SPAN_USEC + 100000U);
    assert(near(rpm, expect_rpm(1.0f, 100000.0f)) && (rpm < 1.0f));
    /* The ceiling never raises a lower estimate back up. */
    rpm = wheel_speed_update(&w, T_EDGES, T_SPAN_USEC,
                             T_SPAN_USEC + 50000U);
    assert(near(rpm, expect_rpm(1.0f, 100000.0f)));
    /* At exactly the timeout it is still the (small) ceiling... */
    rpm = wheel_speed_update(&w, T_EDGES, T_SPAN_USEC,
                             T_SPAN_USEC + WHEEL_SPEED_STOP_USEC);
    assert(rpm > 0.0f);
    /* ...and one microsecond later it is stopped. */
    rpm = wheel_speed_update(&w, T_EDGES, T_SPAN_USEC,
                             T_SPAN_USEC + WHEEL_SPEED_STOP_USEC + 1U);
    assert(0.0f == rpm);
    puts("PASS wheel speed: no edges decays to the 1-edge ceiling, 0 RPM "
         "after 200 ms");
}

/** @brief Counter and microsecond clock wraparound. */
static void test_wraparound (void)
{
    wheel_speed_t w;

    wheel_speed_reset(&w, INT32_MAX - 10, 0xFFFFFFFFU - 9999U);
    /* +88 counts across INT32_MAX, 50 ms across the 32-bit clock wrap. */
    assert(near(wheel_speed_update(&w, (int32_t)(INT32_MIN + 77),
                                   T_SPAN_USEC - 10000U,
                                   T_SPAN_USEC - 10000U),
                100.0f));
    puts("PASS wheel speed: count and clock wraparound give 100 RPM");

    /* Reset forgets the previous speed and reference. */
    wheel_speed_reset(&w, 5, 123U);
    assert((0.0f == w.rpm) && (5 == w.last_count) &&
           (123U == w.last_edge_usec));
    assert(0.0f == wheel_speed_update(&w, 5, 123U, 124U));
    puts("PASS wheel speed: reset reports zero from the new reference");
}

/** @brief Run every wheel-speed case. */
int main (void)
{
    test_rate_and_direction();
    test_restart_boundary();
    test_timeout();
    test_wraparound();
    return 0;
}

/*** end of file ***/

