/* Host-only tests for the supplied Buddy 5 scan planner. */
#include <assert.h>
#include <stdio.h>
#include "modules/obstacle/scan_plan.h"
#include "modules/obstacle/obs_config.h"

static void fill(scan_plan_point_t *points, int32_t range)
{
    int32_t i = 0;
    for (int32_t angle = -OBS_SCAN_RIGHT_DEG;
         angle <= OBS_SCAN_LEFT_DEG;
         angle += OBS_SCAN_STEP_DEG)
    {
        points[i].angle_deg = angle;
        points[i].range_mm = range;
        i++;
    }
    assert(i == OBS_SCAN_POINTS);
}

int main(void)
{
    scan_plan_point_t points[OBS_SCAN_POINTS];
    scan_plan_bypass_t plan = {0U, 0U};
    const int32_t zero = OBS_SCAN_RIGHT_DEG / OBS_SCAN_STEP_DEG;

    fill(points, 1500);
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_CLEAR);
    puts("PASS empty scan");

    fill(points, 1500);
    points[zero - 1].range_mm = 354;
    points[zero].range_mm = 350;
    points[zero + 1].range_mm = 354;
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
    puts("PASS centred object yields bypass");

    fill(points, 1500);
    scan_plan_build_confirmed(points, OBS_SCAN_POINTS, 350, &plan);
    assert(points[zero].range_mm == 350);
    assert(scan_plan_result(&plan) != SCAN_PLAN_CLEAR);
    puts("PASS retained 350 mm front obstacle");

    fill(points, 1500);
    scan_plan_build_confirmed(points, OBS_SCAN_POINTS, 4095, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_CLEAR);
    puts("PASS invalid previous detection not trusted");

    fill(points, 1500);
    points[zero - 2].range_mm = 400;
    points[zero - 1].range_mm = 254;
    points[zero].range_mm = 250;
    points[zero + 1].range_mm = 254;
    points[zero + 2].range_mm = 400;
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
    assert(scan_plan_side_is_left(&plan) == (OBS_PREFERRED_SIDE >= 0));
    /* Outside the 400 mm sample at 20 deg: 137 + 35 pad + 116 half path. */
    assert(scan_plan_lateral_cm(&plan) >= 28);
    puts("PASS both sides crowded widens the preferred side, no NO_ROUTE");

    /* Object offset to the left, so the right edge is much nearer the line:
     * the preferred left side is still used while it is usable.
     */
    fill(points, 1500);
    points[zero].range_mm = 300;
    points[zero + 1].range_mm = 304;
    points[zero + 2].range_mm = 310;
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
    assert(scan_plan_side_is_left(&plan) == (OBS_PREFERRED_SIDE >= 0));
    puts("PASS preferred side used even when the other edge is nearer");

    /* Something else just left of the object: fall back to the right. */
    fill(points, 1500);
    points[zero - 1].range_mm = 254;
    points[zero].range_mm = 250;
    points[zero + 1].range_mm = 254;
    points[zero + 2].range_mm = 400;
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
    assert(scan_plan_side_is_left(&plan) == (OBS_PREFERRED_SIDE < 0));
    puts("PASS crowded preferred side falls back to the other side");

    /* Mid-bypass on the left, the same crowded scan keeps the left side. */
    scan_plan_build_side(points, OBS_SCAN_POINTS, 0, 1, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
    assert(scan_plan_side_is_left(&plan));
    scan_plan_build_side(points, OBS_SCAN_POINTS, 0, -1, &plan);
    assert(!scan_plan_side_is_left(&plan));
    puts("PASS mid-bypass rescan keeps the side already taken");

    /* Logged wide flat obstacle: the echo stops between 30 and 40 deg. The
     * edge may reach the empty 40 deg ray, so the offset covers that: 238
     * mm edge + 116 mm half path, not the old half-step 192 + 116.
     */
    {
        static const int32_t logged[OBS_SCAN_POINTS] = {
            1183, 330, 320, 315, 314, 314, 318, 328, 4000};
        fill(points, 1500);
        for (int32_t i = 0; i < OBS_SCAN_POINTS; i++)
        {
            points[i].range_mm = logged[i];
        }
        scan_plan_build(points, OBS_SCAN_POINTS, &plan);
        assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
        assert(scan_plan_side_is_left(&plan));
        assert(scan_plan_lateral_cm(&plan) >= 35);
        puts("PASS edge reaches the next empty ray, widening the offset");
    }

    /* A narrow obstacle (echo only straight ahead) keeps its measured,
     * tighter offset; the wide-obstacle floor does not apply.
     */
    fill(points, 1500);
    points[zero].range_mm = 300;
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
    assert(scan_plan_lateral_cm(&plan) * 10 < OBS_MIN_LATERAL_MM);
    puts("PASS narrow obstacle is not widened to the minimum offset");

    /* Echoes out to 20 deg: the boundary may be a flat-face dropout, so the
     * minimum offset applies.
     */
    fill(points, 1500);
    points[zero - 2].range_mm = 170;
    points[zero - 1].range_mm = 162;
    points[zero].range_mm = 160;
    points[zero + 1].range_mm = 162;
    points[zero + 2].range_mm = 170;
    scan_plan_build(points, OBS_SCAN_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
    assert(scan_plan_lateral_cm(&plan) * 10 >= OBS_MIN_LATERAL_MM - 5);
    puts("PASS wide-looking obstacle gets at least the minimum offset");

    puts("All planner scenarios passed.");
    return 0;
}
