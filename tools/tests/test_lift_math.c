/** @file test_lift_math.c
 * @brief Host test of the lifted-height estimator
 *        (modules/terrain/lift_math.c). Pure arithmetic: no kernel, I2C or
 *        motor code is involved.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "modules/terrain/lift_math.h"

#define STEP_MSEC 20U

static lift_math_state_t g_state;

/** @brief Host stub of run(). */
static uint16_t run(int16_t excess_mg, unsigned samples, bool floor)
{
    uint16_t height = 0U;
    for (unsigned i = 0U; i < samples; i++)
    {
        height = lift_math_step(&g_state, excess_mg, STEP_MSEC, floor);
    }
    return height;
}

/* +200 mg for 200 ms, coast 200 ms, -200 mg for 200 ms: about 157 mm. */
static uint16_t lift(void)
{
    (void)run(200, 5U, true);       /* still within sensor range */
    (void)run(200, 5U, false);
    (void)run(0, 10U, false);
    return run(-200, 10U, false);
}

/** @brief Run every test case; any failed assert aborts. */
int main(void)
{
    uint16_t height;

    /* NULL is tolerated. */
    lift_math_reset(NULL);
    assert(lift_math_step(NULL, 500, STEP_MSEC, false) == 0U);
    assert(!lift_math_is_active(NULL));

    /* Resting on the floor with sensor noise never reads as a lift. */
    lift_math_reset(&g_state);
    for (unsigned i = 0U; i < 500U; i++)
    {
        height = lift_math_step(&g_state, (int16_t)((i % 7U) - 3U),
                                STEP_MSEC, true);
        assert(height == 0U && !lift_math_is_active(&g_state));
    }

    /* Motor vibration below the start level, and a two-sample spike. */
    for (unsigned i = 0U; i < 500U; i++)
    {
        height = lift_math_step(&g_state, (i & 1U) ? 45 : -45, STEP_MSEC,
                                true);
        assert(height == 0U && !lift_math_is_active(&g_state));
    }
    assert(run(120, 2U, true) < 3U && !lift_math_is_active(&g_state));
    assert(run(0, 1U, true) == 0U);

    /* A real lift is measured to within 15 % of the true 157 mm. */
    height = lift();
    assert(lift_math_is_active(&g_state));
    assert(height >= 133U && height <= 181U);

    /* Held still in the air: the estimate neither drifts nor resets. */
    {
        uint16_t const held = run(0, 250U, false);
        assert(abs((int)held - (int)height) <= 3);
        assert(lift_math_is_active(&g_state));
        /* Tilted in the hand: a constant offset is learned, not summed. */
        (void)run(30, 60U, false);
        height = run(30, 500U, false);
        assert(height == run(30, 500U, false));
        assert(height < held + 60U);
    }

    /* A sensor seeing the floor means it is not up: the height reads 0
     * while it does. A brief touch (a hand under the sensors) does not end
     * the estimate, so it carries on when the floor is lost again. */
    assert(run(30, LIFT_MATH_FLOOR_SAMPLES - 1U, true) == 0U);
    assert(lift_math_is_active(&g_state));
    (void)run(30, 1U, false);
    assert(lift_math_is_active(&g_state));

    /* Put down: lowered, then the floor seen for half a second. */
    (void)run(-150, 8U, false);
    (void)run(180, 8U, false);
    assert(run(0, LIFT_MATH_FLOOR_SAMPLES, true) == 0U);
    assert(!lift_math_is_active(&g_state));

    /* A bump on the floor may start an estimate but cannot keep it. */
    (void)run(90, 3U, true);
    assert(run(-90, 3U, true) < 20U);
    assert(run(0, LIFT_MATH_FLOOR_SAMPLES, true) == 0U);
    assert(!lift_math_is_active(&g_state));

    /* Logged false lift: a bump while driving switches the estimate on,
     * vibration's small upward bias (+30 mg) builds speed while the line
     * sensors still see the floor, then the robot stops on a junction
     * (both black: no floor) and sits still. That used to read as a lift
     * of ~100 mm within half a second. */
    lift_math_reset(&g_state);
    (void)run(0, 40U, true);
    (void)run(80, LIFT_MATH_START_SAMPLES, true);  /* bump: active */
    assert(run(30, 17U, true) == 0U);              /* floor: height 0 */
    height = run(0, 30U, false);                   /* stopped on junction */
    assert(height < 20U);
    assert(run(0, 100U, false) < 20U && lift_math_is_still(&g_state));

    /* It cannot sink below the floor. */
    lift_math_reset(&g_state);
    (void)run(80, 3U, false);
    assert(run(-400, 50U, false) == 0U);

    /* Saturates at the largest value the telemetry field can carry. */
    lift_math_reset(&g_state);
    assert(run(1500, 200U, false) == LIFT_MATH_MAX_MM);

    /* A gap in the samples restarts the estimate. */
    assert(lift_math_step(&g_state, 0, LIFT_MATH_MAX_STEP_MSEC + 1U,
                          false) == 0U);
    assert(!lift_math_is_active(&g_state));
    assert(lift_math_step(&g_state, 0, 0U, false) == 0U);

    puts("Lift height: rest, vibration, measured lift, hold, tilt offset, "
         "floor clamp, landing, bump, driving into a junction, saturation "
         "and sample gap PASS");
    return 0;
}
/*** end of file ***/

