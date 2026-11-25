/** @file test_mt_steering.c
 * @brief Host test (FR4 line follow): the real steering.c PI steering and
 *        line observer, and the real line_follow.c truth table and labels.
 *
 * Checks the sign and size of each correction, the +-STEERING_MAX clamp,
 * the speed-scale bound, nudge/curve/repeat/hold/bend-memory behaviour at
 * their period boundaries, and the observer's band corrections, heading
 * limit and lost-outside-a-sensor case.
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "modules/motion/steering.h"
#include "modules/line_barcode/line_follow.h"

/* The controller period the gains were tuned at, seconds. */
#define T_PERIOD_SEC 0.05f
/* Float comparison tolerance. */
#define T_EPS 0.0001f

/** @brief True when two floats agree within T_EPS. */
static bool near (float a, float b)
{
    return fabsf(a - b) < T_EPS;
}

/** @brief Run @p n periods of one action at speed scale 1. */
static float run (steering_t * p_s, line_follow_action_t action, uint32_t n)
{
    float out = 0.0f;

    for (uint32_t i = 0U; i < n; i++)
    {
        out = steering_update(p_s, action, T_PERIOD_SEC, 1.0f);
    }

    return out;
}

/** @brief All four sensor combinations, out-of-range levels and labels. */
static void test_line_follow (void)
{
    assert(LINE_FORWARD == line_follow_decide(0U, 0U));
    assert(LINE_CORRECT_LEFT == line_follow_decide(1U, 0U));
    assert(LINE_CORRECT_RIGHT == line_follow_decide(0U, 1U));
    assert(LINE_UNCERTAIN == line_follow_decide(1U, 1U));
    /* A corrupt level is never read as a steering command. */
    assert(LINE_UNCERTAIN == line_follow_decide(2U, 0U));
    assert(LINE_UNCERTAIN == line_follow_decide(0U, 0xFFFFFFFFU));
    puts("PASS line_follow_decide: 4 sensor combinations, bad level "
         "-> UNCERTAIN");

    assert(0 == strcmp(line_follow_text(LINE_FORWARD), "FORWARD"));
    assert(0 == strcmp(line_follow_text(LINE_CORRECT_LEFT),
                       "CORRECT LEFT"));
    assert(0 == strcmp(line_follow_text(LINE_CORRECT_RIGHT),
                       "CORRECT RIGHT"));
    assert(0 == strcmp(line_follow_text(LINE_UNCERTAIN),
                       "BOTH BLACK - CHECK JUNCTION"));
    assert(0 == strcmp(line_follow_text((line_follow_action_t)7),
                       "UNKNOWN ACTION"));
    puts("PASS line_follow_text: four names and UNKNOWN ACTION default");
}

/** @brief PI: sign, symmetry, exact first-period size and the clamp. */
static void test_pi_sign_and_clamp (void)
{
    steering_t s;
    float      left  = 0.0f;
    float      right = 0.0f;
    float      out   = 0.0f;

    steering_reset(&s);
    assert(0.0f == run(&s, LINE_FORWARD, 5U));
    left = run(&s, LINE_CORRECT_LEFT, 1U);
    steering_reset(&s);
    right = run(&s, LINE_CORRECT_RIGHT, 1U);
    /* KP 0.3 plus one period of KI 2.0 * 0.05 s. */
    assert(near(left, 0.4f) && near(right, -0.4f));
    puts("PASS steering PI: left +0.4, right -0.4 after one period");

    steering_reset(&s);

    for (uint32_t i = 0U; i < 200U; i++)
    {
        out = steering_update(&s, LINE_CORRECT_LEFT, T_PERIOD_SEC, 1.0f);
        assert(out <= STEERING_MAX);
    }

    assert(STEERING_MAX == out);
    assert(s.integral <= STEERING_MAX);
    steering_reset(&s);

    for (uint32_t i = 0U; i < 200U; i++)
    {
        out = steering_update(&s, LINE_CORRECT_RIGHT, 1.0f, 3.0f);
        assert(out >= -STEERING_MAX);
    }

    assert(-STEERING_MAX == out);
    puts("PASS steering PI: output and integral clamped to +-STEERING_MAX");

    /* Scale outside 0..STEERING_SCALE_MAX is bounded, not trusted. */
    steering_t a;
    steering_t b;

    steering_reset(&a);
    steering_reset(&b);
    assert(near(steering_update(&a, LINE_CORRECT_LEFT, T_PERIOD_SEC, 10.0f),
                steering_update(&b, LINE_CORRECT_LEFT, T_PERIOD_SEC,
                                STEERING_SCALE_MAX)));
    steering_reset(&a);

    for (uint32_t i = 0U; i < 50U; i++)
    {
        out = steering_update(&a, LINE_CORRECT_LEFT, T_PERIOD_SEC, -1.0f);
        assert(near(out, 0.3f));
    }

    puts("PASS steering PI: scale >3 acts as 3, negative scale freezes the "
         "integral");
}

/** @brief A nudge ends at once; a curve's turn decays steadily. */
static void test_nudge_and_curve (void)
{
    steering_t s;
    float      out  = 0.0f;
    float      prev = 0.0f;

    steering_reset(&s);
    (void)run(&s, LINE_CORRECT_LEFT, 3U); /* integral 0.3 < 0.35 */
    assert(0.0f == run(&s, LINE_FORWARD, 1U));
    puts("PASS steering: 3-period nudge drops its turn on release");

    steering_reset(&s);
    (void)run(&s, LINE_CORRECT_LEFT, 4U); /* integral 0.4: a curve */
    out = run(&s, LINE_FORWARD, 1U);
    assert(near(out, 0.4f * 0.85f));
    prev = out;

    for (uint32_t i = 0U; i < 40U; i++)
    {
        out = run(&s, LINE_FORWARD, 1U);
        assert((out < prev) && (out > 0.0f));
        prev = out;
    }

    assert(out < 0.01f);
    puts("PASS steering: 4-period curve kept, decays 0.85 per centred "
         "period");

    /* Crossing to the other side drops the turn, never unwinds it. */
    steering_reset(&s);
    (void)run(&s, LINE_CORRECT_LEFT, 6U);
    out = run(&s, LINE_CORRECT_RIGHT, 1U);
    assert(near(out, -0.4f));
    puts("PASS steering: crossing the line restarts from zero (-0.4)");
}

/** @brief Repeat touch on one side: kept up to 8 centred periods apart. */
static float repeat_after (uint32_t gap, line_follow_action_t second)
{
    steering_t s;

    steering_reset(&s);
    (void)run(&s, LINE_CORRECT_LEFT, 1U);
    (void)run(&s, LINE_FORWARD, 1U + gap); /* end event, then gap */
    (void)run(&s, second, 1U);
    return run(&s, LINE_FORWARD, 1U);
}

/** @brief STEERING_REPEAT_PERIODS boundary and alternating sides. */
static void test_repeat (void)
{
    assert(repeat_after(8U, LINE_CORRECT_LEFT) > 0.0f);
    assert(0.0f == repeat_after(9U, LINE_CORRECT_LEFT));
    assert(0.0f == repeat_after(2U, LINE_CORRECT_RIGHT));
    puts("PASS steering: same-side repeat after 8 centred periods kept, "
         "9 dropped, alternating sides dropped");
}

/** @brief Full-strength turn: held 12 periods at scale 1, 6 at scale 2. */
static void test_hold (void)
{
    steering_t s;
    uint32_t   held = 0U;
    float      out  = 0.0f;

    steering_reset(&s);
    (void)run(&s, LINE_CORRECT_LEFT, 10U);

    do
    {
        out = steering_update(&s, LINE_FORWARD, T_PERIOD_SEC, 1.0f);
        held++;
    } while ((STEERING_MAX == out) && (held < 100U));

    /* Release period plus 12 hold periods at full turn. */
    assert(14U == held && out < STEERING_MAX);
    steering_reset(&s);
    (void)steering_update(&s, LINE_CORRECT_LEFT, 0.2f, 2.0f);
    (void)steering_update(&s, LINE_CORRECT_LEFT, 0.2f, 2.0f);
    held = 0U;

    do
    {
        out = steering_update(&s, LINE_FORWARD, T_PERIOD_SEC, 2.0f);
        held++;
    } while ((STEERING_MAX == out) && (held < 100U));

    assert(8U == held);
    puts("PASS steering hold: full turn held 1+12 periods at scale 1, "
         "1+6 at scale 2 (same distance)");

    /* A sensor finding the line ends the hold at once. */
    steering_reset(&s);
    (void)run(&s, LINE_CORRECT_LEFT, 10U);
    (void)run(&s, LINE_FORWARD, 3U);
    out = run(&s, LINE_CORRECT_RIGHT, 1U);
    assert(near(out, -0.4f) && (0.0f == s.hold_count));
    puts("PASS steering hold: other sensor ends the hold (-0.4)");
}

/** @brief Full turn from a touch of @p touch periods of @p period_sec,
 *  then a 1-period same-side touch released @p gap calls later. */
static float bend_after (uint32_t touch, float period_sec, uint32_t gap)
{
    steering_t s;

    steering_reset(&s);

    for (uint32_t i = 0U; i < touch; i++)
    {
        (void)steering_update(&s, LINE_CORRECT_LEFT, period_sec, 1.0f);
    }

    (void)run(&s, LINE_FORWARD, 1U);       /* full-turn release */
    (void)run(&s, LINE_FORWARD, gap - 2U); /* hold, then decay */
    (void)run(&s, LINE_CORRECT_LEFT, 1U);
    return run(&s, LINE_FORWARD, 1U);      /* call number gap */
}

/** @brief Bend memory: 25-period window, needs a 5-period touch. */
static void test_bend_memory (void)
{
    assert(STEERING_MAX == bend_after(10U, T_PERIOD_SEC, 25U));
    assert(0.0f == bend_after(10U, T_PERIOD_SEC, 26U));
    /* Saturated in two long periods (touch < 5): no bend remembered. */
    assert(0.0f == bend_after(2U, 0.2f, 25U));
    puts("PASS steering bend memory: short touch 25 periods after a full "
         "turn resumes it, 26 does not, a 2-period full turn sets none");
}

/** @brief Observer: centred, left/right bands, both black, reset. */
static void test_observer_bands (void)
{
    line_track_t t;
    float        left  = 0.0f;
    float        right = 0.0f;

    line_track_reset(&t);

    for (uint32_t i = 0U; i < 50U; i++)
    {
        assert(0.0f == line_track_update(&t, false, false, 2.0f, 2.0f));
    }

    line_track_reset(&t);
    left = line_track_update(&t, true, false, 2.0f, 2.0f);
    /* Line seen under the left sensor: at least 4 mm left (gap edge),
     * heading corrected by 0.5 * 4 / 20 mm. */
    assert(near(t.offset_mm, 4.0f) && near(t.heading_rad, -0.1f));
    assert(left > 0.25f && left < 0.35f);
    line_track_reset(&t);
    right = line_track_update(&t, false, true, 2.0f, 2.0f);
    assert(near(t.offset_mm, -4.0f) && near(right, -left));
    puts("PASS line observer: left sensor -> offset +4 mm, steer left; "
         "right mirrors it");

    /* Both black (junction): keep the prediction, no correction. */
    float const before  = t.offset_mm;
    float const heading = t.heading_rad;

    (void)line_track_update(&t, true, true, 2.0f, 2.0f);
    assert(near(t.offset_mm, before - (2.0f * heading)));
    assert(t.offset_mm < -4.0f); /* outside the band: not snapped back */
    line_track_reset(&t);
    assert((0.0f == t.offset_mm) && (0.0f == t.heading_rad) &&
           (0.0f == t.integral) && (0.0f == t.since_mm));
    puts("PASS line observer: both black keeps prediction; reset clears");
}

/** @brief Observer: lost outside a sensor, heading limit, steer clamp. */
static void test_observer_limits (void)
{
    line_track_t t;
    float        out = 0.0f;

    /* Swerving right with no sensor: the line ends up outside the left
     * sensor (beyond 22 mm) within one period, where "neither sees it"
     * is not the gap. */
    line_track_reset(&t);

    out = line_track_update(&t, false, false, 30.0f, 0.0f);

    assert(t.offset_mm > 22.0f);
    float const kept = t.offset_mm;

    out = line_track_update(&t, false, false, 0.0f, 0.0f);
    assert(near(t.offset_mm, kept));
    assert(STEERING_MAX == out);
    puts("PASS line observer: line lost outside a sensor is not snapped "
         "into the gap; steer clamps at +STEERING_MAX");

    for (uint32_t i = 0U; i < 40U; i++)
    {
        out = line_track_update(&t, false, false, 10.0f, 0.0f);
        assert(t.heading_rad >= -0.8f && out <= STEERING_MAX);
        assert(t.integral <= STEERING_MAX);
    }

    assert(-0.8f == t.heading_rad);
    puts("PASS line observer: heading bounded at -0.8 rad, integral and "
         "steer within +-STEERING_MAX");

    /* Inside the outer edge but outside the gap with no sensor: the line
     * must be in the gap, so the estimate snaps to the 4 mm edge. */
    line_track_reset(&t);
    t.offset_mm = 10.0f;
    t.since_mm  = 100.0f;
    (void)line_track_update(&t, false, false, 0.0f, 0.0f);
    assert(near(t.offset_mm, 4.0f));
    /* Spread over the 100 mm it built up over: 0.5 * -6 / 100. */
    assert(near(t.heading_rad, 0.03f) && (0.0f == t.since_mm));
    puts("PASS line observer: 10 mm with no sensor snaps to the 4 mm gap "
         "edge, heading fixed over 100 mm travelled");
}

/** @brief Run every steering and line-follow case. */
int main (void)
{
    test_line_follow();
    test_pi_sign_and_clamp();
    test_nudge_and_curve();
    test_repeat();
    test_hold();
    test_bend_memory();
    test_observer_bands();
    test_observer_limits();
    return 0;
}

/*** end of file ***/

