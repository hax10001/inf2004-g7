/* Host-only tests for the supplied Buddy 5 scan planner. */
#include <assert.h>
#include <stdio.h>
#include "modules/obstacle/scan_plan.h"
#include "modules/obstacle/obs_config.h"

/* These scenarios use plain 10 degree data; the planner takes any bearing
 * sequence. The firmware's finer front sequence is tested at the end. */
#define UNIFORM_POINTS \
    ((OBS_SCAN_LEFT_DEG + OBS_SCAN_RIGHT_DEG) / OBS_SCAN_STEP_DEG + 1)

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
    assert(i == UNIFORM_POINTS);
}

int main(void)
{
    scan_plan_point_t points[UNIFORM_POINTS];
    scan_plan_bypass_t plan = {0U, 0U};
    const int32_t zero = OBS_SCAN_RIGHT_DEG / OBS_SCAN_STEP_DEG;

    fill(points, 1500);
    scan_plan_build(points, UNIFORM_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_CLEAR);
    puts("PASS empty scan");

    fill(points, 1500);
    points[zero - 1].range_mm = 354;
    points[zero].range_mm = 350;
    points[zero + 1].range_mm = 354;
    scan_plan_build(points, UNIFORM_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
    puts("PASS centred object yields bypass");

    fill(points, 1500);
    scan_plan_build_confirmed(points, UNIFORM_POINTS, OBS_STOP_MM, &plan);
    assert(points[zero].range_mm == OBS_STOP_MM);
    assert(scan_plan_result(&plan) != SCAN_PLAN_CLEAR);
    puts("PASS retained confirmed front obstacle at the closer trigger");

    fill(points, 1500);
    scan_plan_build_confirmed(points, UNIFORM_POINTS, 4095, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_CLEAR);
    puts("PASS invalid previous detection not trusted");

    fill(points, 1500);
    points[zero - 2].range_mm = 400;
    points[zero - 1].range_mm = 254;
    points[zero].range_mm = 250;
    points[zero + 1].range_mm = 254;
    points[zero + 2].range_mm = 400;
    scan_plan_build(points, UNIFORM_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_NO_ROUTE);
    puts("PASS both corridors blocked stops instead of assuming a wider route");

    /* Object offset to the left, so the right edge is much nearer the line:
     * the preferred left side is still used while it is usable.
     */
    fill(points, 1500);
    points[zero].range_mm = 300;
    points[zero + 1].range_mm = 304;
    points[zero + 2].range_mm = 310;
    scan_plan_build(points, UNIFORM_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
    assert(!scan_plan_side_is_left(&plan));
    puts("PASS nearer clear right corridor wins over the old left preference");

    /* Something else just left of the object: fall back to the right. */
    fill(points, 1500);
    points[zero - 1].range_mm = 254;
    points[zero].range_mm = 250;
    points[zero + 1].range_mm = 254;
    points[zero + 2].range_mm = 400;
    scan_plan_build(points, UNIFORM_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
    assert(scan_plan_side_is_left(&plan) == (OBS_PREFERRED_SIDE < 0));
    puts("PASS crowded preferred side falls back to the other side");

    /* Mid-bypass on the left, the same crowded scan keeps the left side. */
    scan_plan_build_side(points, UNIFORM_POINTS, 0, 1, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_NO_ROUTE);
    scan_plan_build_side(points, UNIFORM_POINTS, 0, -1, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
    assert(!scan_plan_side_is_left(&plan));
    puts("PASS mid-bypass keeps its side or stops if its corridor is blocked");

    /* Logged wide flat obstacle: the echo stops between 30 and 40 deg. The
     * edge may reach the empty 40 deg ray, so the offset covers that: 238
     * mm edge + 116 mm half path, not the old half-step 192 + 116.
     */
    {
        static const int32_t logged[9] = {
            1183, 330, 320, 315, 314, 314, 318, 328, 4000};
        fill(points, 1500);
        for (int32_t i = 0; i < 9; i++)
        {
            points[zero - 4 + i].range_mm = logged[i];
        }
        scan_plan_build(points, UNIFORM_POINTS, &plan);
        assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
        assert(scan_plan_lateral_cm(&plan) >= 35);
        puts("PASS edge reaches the next empty ray, widening the offset");
    }

    /* A narrow obstacle (echo only straight ahead) keeps its measured,
     * tighter offset; the wide-obstacle floor does not apply.
     */
    fill(points, 1500);
    points[zero].range_mm = 300;
    scan_plan_build(points, UNIFORM_POINTS, &plan);
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
    scan_plan_build(points, UNIFORM_POINTS, &plan);
    assert(scan_plan_result(&plan) == SCAN_PLAN_ROUTE);
    assert(scan_plan_lateral_cm(&plan) * 10 >= OBS_MIN_LATERAL_MM - 5);
    puts("PASS wide-looking obstacle gets at least the minimum offset");

    /* These lateral obstructions were outside the old +/-40 degree scan. */
    for(int32_t side=-1;side<=1;side+=2)
    {
        fill(points,1500);
        points[zero].range_mm=250;
        int32_t const extreme=side>0?UNIFORM_POINTS-1:0;
        points[extreme].range_mm=250;
        assert(points[extreme].angle_deg==side*70);
        scan_plan_build(points,UNIFORM_POINTS,&plan);
        assert(scan_plan_result(&plan)==SCAN_PLAN_ROUTE);
        assert(scan_plan_side_is_left(&plan)==(side<0));
        points[side>0?0:UNIFORM_POINTS-1].range_mm=250;
        scan_plan_build(points,UNIFORM_POINTS,&plan);
        assert(scan_plan_result(&plan)==SCAN_PLAN_NO_ROUTE);
    }
    assert(sizeof(scan_plan_point_t)==4U && sizeof(scan_plan_bypass_t)==8U);
    puts("PASS extreme-left/right blockages select the opposite clear side");

    /* Side choice by where the object sits, with the reported needs. */
    {
        scan_plan_sides_t sides={0};
        int32_t const diff_limit=OBS_SIDE_PREFERENCE_THRESHOLD_MM;

        /* Object offset to the left of the path: go round the right. */
        fill(points,1500);
        points[zero].range_mm=300;points[zero+1].range_mm=304;
        points[zero+2].range_mm=310;
        scan_plan_build_side_report(points,UNIFORM_POINTS,0,0,&plan,&sides);
        assert(scan_plan_result(&plan)==SCAN_PLAN_ROUTE);
        assert(!scan_plan_side_is_left(&plan) && !sides.b_tie);
        assert(sides.b_object && sides.b_left_ok && sides.b_right_ok);
        assert((int32_t)sides.left_need_mm-(int32_t)sides.right_need_mm >
               diff_limit);
        puts("PASS obstacle offset left: bypass right, the side it blocks less");

        /* Mirror: object offset to the right, go round the left. */
        fill(points,1500);
        points[zero].range_mm=300;points[zero-1].range_mm=304;
        points[zero-2].range_mm=310;
        scan_plan_build_side_report(points,UNIFORM_POINTS,0,0,&plan,&sides);
        assert(scan_plan_result(&plan)==SCAN_PLAN_ROUTE);
        assert(scan_plan_side_is_left(&plan) && !sides.b_tie);
        assert((int32_t)sides.right_need_mm-(int32_t)sides.left_need_mm >
               diff_limit);
        puts("PASS obstacle offset right: bypass left, the side it blocks less");

        /* Centred object: equal needs, so the configured default side. */
        fill(points,1500);
        points[zero-1].range_mm=354;points[zero].range_mm=350;
        points[zero+1].range_mm=354;
        scan_plan_build_side_report(points,UNIFORM_POINTS,0,0,&plan,&sides);
        assert(scan_plan_result(&plan)==SCAN_PLAN_ROUTE);
        assert(sides.b_tie && sides.left_need_mm==sides.right_need_mm);
        assert(scan_plan_side_is_left(&plan)==(OBS_SIDE_DEFAULT>0));
        puts("PASS centred obstacle takes OBS_SIDE_DEFAULT");

        /* Slightly off centre (a few mm of edge noise): still a tie. */
        fill(points,1500);
        points[zero-1].range_mm=360;points[zero].range_mm=350;
        points[zero+1].range_mm=354;
        scan_plan_build_side_report(points,UNIFORM_POINTS,0,0,&plan,&sides);
        assert(sides.left_need_mm!=sides.right_need_mm);
        assert(sides.b_tie);
        assert(scan_plan_side_is_left(&plan)==(OBS_SIDE_DEFAULT>0));
        puts("PASS near-equal sides within the threshold use the default side");

        /* A blocked corridor is reported and never treated as a tie. */
        fill(points,1500);
        points[zero].range_mm=250;
        points[UNIFORM_POINTS-1].range_mm=250;
        scan_plan_build_side_report(points,UNIFORM_POINTS,0,0,&plan,&sides);
        assert(!sides.b_left_ok && sides.b_right_ok && !sides.b_tie);
        assert(!scan_plan_side_is_left(&plan));
        puts("PASS blocked side is reported and the clear side is taken");

        /* The debug report names the blocker and where the object is. */
        assert(sides.left.b_blocked && !sides.right.b_blocked);
        assert(sides.left.blocker_deg==OBS_SCAN_LEFT_DEG);
        assert(sides.left.blocker_mm==250U);
        assert(sides.low_deg==0 && sides.high_deg==0 && sides.near_mm==250U);
        assert(sides.left.b_seen && sides.right.b_seen);
        assert(sides.left.open_mm==250U && sides.right.open_mm==1500U);
        puts("PASS side detail reports the blocker, edges and open space");

        /* An object filling the sweep on one side: unseen-edge floor. */
        fill(points,1500);
        for(int32_t i=zero;i<UNIFORM_POINTS;i++) points[i].range_mm=300;
        scan_plan_build_side_report(points,UNIFORM_POINTS,0,0,&plan,&sides);
        assert(!sides.left.b_seen && sides.right.b_seen);
        assert(sides.left.reason==SCAN_PLAN_NEED_UNSEEN ||
               sides.left_need_mm>OBS_LATERAL_UNKNOWN_MM);
        assert(sides.left.edge_deg==OBS_SCAN_LEFT_DEG);
        assert(!scan_plan_side_is_left(&plan));
        puts("PASS side detail flags an edge beyond the sweep");

        /* No object: the report says so and the plan is clear. */
        fill(points,1500);
        scan_plan_build_side_report(points,UNIFORM_POINTS,0,0,&plan,&sides);
        assert(scan_plan_result(&plan)==SCAN_PLAN_CLEAR && !sides.b_object);
        assert(sizeof(scan_plan_side_info_t)<=12U);
        puts("PASS side report is empty when nothing is ahead");
    }
    /* Firmware sequence: 10 degree steps outside the front zone, finer
     * inside it, from the right end to the left end. */
    {
        scan_plan_point_t fine[OBS_SCAN_POINTS];
        scan_plan_sides_t sides={0};
        int32_t n=0;
        for(int32_t a=-OBS_SCAN_RIGHT_DEG;a<=OBS_SCAN_LEFT_DEG;
            a=scan_plan_next_angle(a))
        {
            assert(n<OBS_SCAN_POINTS);
            fine[n].angle_deg=a;fine[n].range_mm=4000;n++;
        }
        assert(n==OBS_SCAN_POINTS);
        assert(fine[n-1].angle_deg==OBS_SCAN_LEFT_DEG);
        assert(scan_plan_next_angle(0)==OBS_SCAN_FINE_STEP_DEG);
        assert(scan_plan_next_angle(-OBS_SCAN_RIGHT_DEG)==
               -OBS_SCAN_RIGHT_DEG+OBS_SCAN_STEP_DEG);
        puts("PASS scan sequence is fine in front, coarse at the sides");

        /* A 150 mm face 230 mm ahead whose echoes span x_lo..x_hi mm:
         * a ray echoes (at ~240 mm) where 230 * tan(a) falls inside. */
        {
            static const int32_t tan_milli[15]={0,87,176,268,364,466,577,
                700,839,1000,1192,1428,1732,2145,2747};
            static const int32_t spans[3][2]={{-50,100},{-100,50},
                                              {20,170}};
            for(int32_t k=0;k<3;k++)
            {
                for(int32_t i=0;i<n;i++)
                {
                    int32_t const a=fine[i].angle_deg;
                    int32_t const x=(a<0?-1:1)*230*
                        tan_milli[(a<0?-a:a)/5]/1000;
                    fine[i].range_mm=(x>=spans[k][0] && x<=spans[k][1])?
                        240:4000;
                }
                scan_plan_build_side_report(fine,n,0,0,&plan,&sides);
                assert(scan_plan_result(&plan)==SCAN_PLAN_ROUTE);
                if(k<2)
                {
                    /* 25 mm off centre: below what the beam resolves,
                     * so centred, OBS_SIDE_DEFAULT. */
                    assert(sides.b_tie);
                    assert(scan_plan_side_is_left(&plan)==
                           (OBS_SIDE_DEFAULT>0));
                }
                else
                {
                    /* Wholly left of the line: still goes right. */
                    assert(!sides.b_tie && !scan_plan_side_is_left(&plan));
                }
            }
        }
        puts("PASS small offsets take the default side, clear ones choose");

        /* Logged 22:15 scan: edges at -15 and +20 degrees. The +20 edge is
         * far enough off centre for the OBS_MIN_LATERAL_MM floor, which
         * used to add ~35 mm to that side alone and decide the side. The
         * floor now only sets the drive; the sides compare measured. */
        {
            static const int32_t logged[][2]={{-15,233},{-10,216},{-5,208},
                {0,210},{5,207},{10,207},{15,211},{20,215}};
            for(int32_t i=0;i<n;i++) fine[i].range_mm=4000;
            for(unsigned k=0;k<sizeof logged/sizeof logged[0];k++)
                for(int32_t i=0;i<n;i++)
                    if(fine[i].angle_deg==logged[k][0])
                        fine[i].range_mm=(uint16_t)logged[k][1];
            scan_plan_build_side_report(fine,n,0,0,&plan,&sides);
            assert(sides.left.reason==SCAN_PLAN_NEED_MIN);
            assert(sides.left_need_mm==OBS_MIN_LATERAL_MM);
            assert(sides.left.compare_mm<sides.left_need_mm);
            assert(sides.right.compare_mm==sides.right_need_mm);
            {
                int32_t const diff=(int32_t)sides.left.compare_mm-
                                   (int32_t)sides.right.compare_mm;
                int32_t const mag=diff<0?-diff:diff;
                assert(sides.b_tie==(mag<=OBS_SIDE_PREFERENCE_THRESHOLD_MM));
                assert(mag<40); /* was 48 with the floor */
            }
            /* If chosen, the left still drives the full floored offset. */
            scan_plan_build_side_report(fine,n,0,1,&plan,&sides);
            assert(scan_plan_side_is_left(&plan));
            assert(scan_plan_lateral_cm(&plan)*10>=OBS_MIN_LATERAL_MM-5);
        }
        puts("PASS the MIN_LATERAL floor sets the drive, not the side choice");
    }
    puts("All planner scenarios passed.");
    return 0;
}
