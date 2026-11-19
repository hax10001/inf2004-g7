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
        points[i].lone_mm = 0U;
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
        int32_t const diff_limit=0; /* no tie band: any difference */

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

        /* Centred object: equal moves. No default side: the more open half
         * of the sweep decides, either way. */
        for(int32_t open_side=-1;open_side<=1;open_side+=2)
        {
            fill(points,1500);
            points[zero-1].range_mm=354;points[zero].range_mm=350;
            points[zero+1].range_mm=354;
            points[open_side>0?UNIFORM_POINTS-1:0].range_mm=1700;
            scan_plan_build_side_report(points,UNIFORM_POINTS,0,0,&plan,
                                        &sides);
            assert(scan_plan_result(&plan)==SCAN_PLAN_ROUTE);
            assert(sides.b_tie && sides.left_need_mm==sides.right_need_mm);
            assert(scan_plan_side_is_left(&plan)==(open_side>0));
        }
        puts("PASS centred obstacle goes to the more open side, no default");

        /* Slightly off centre (a few mm): no tie band, the smaller move
         * wins however small the difference. */
        fill(points,1500);
        points[zero-1].range_mm=360;points[zero].range_mm=350;
        points[zero+1].range_mm=354;
        scan_plan_build_side_report(points,UNIFORM_POINTS,0,0,&plan,&sides);
        assert(sides.left_need_mm!=sides.right_need_mm);
        if(sides.left.compare_mm!=sides.right.compare_mm)
        {
            assert(!sides.b_tie);
            assert(scan_plan_side_is_left(&plan)==
                   (sides.left.compare_mm<sides.right.compare_mm));
        }
        else
        {
            /* Equal at the beam's resolution: the smaller drive decides. */
            assert(sides.b_tie);
            assert(scan_plan_side_is_left(&plan)==
                   (sides.left_need_mm<sides.right_need_mm));
        }
        puts("PASS near-equal sides: the smaller move wins, no threshold");

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
        scan_plan_point_t fine[OBS_SCAN_POINTS]={{0}};
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
                    /* 25 mm off centre: no tie band, so the side with the
                     * smaller move, however small. */
                    if(!sides.b_tie)
                        assert(scan_plan_side_is_left(&plan)==
                               (sides.left.compare_mm<
                                sides.right.compare_mm));
                }
                else
                {
                    /* Wholly left of the line: still goes right. */
                    assert(!sides.b_tie && !scan_plan_side_is_left(&plan));
                }
            }
        }
        puts("PASS small offsets take the smaller move, clear ones too");

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
#if OBS_SIDE_COMPARE_MEASURED
            /* Sides compare the beam-corrected edges, never more than the
             * padded need that sets the drive. */
            assert(sides.right.compare_mm<=sides.right_need_mm);
#else
            assert(sides.right.compare_mm==sides.right_need_mm);
#endif
            {
                int32_t const diff=(int32_t)sides.left.compare_mm-
                                   (int32_t)sides.right.compare_mm;
                int32_t const mag=diff<0?-diff:diff;
                /* Within the centred band the room past it decides. */
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
    /* Reported width with a real beam: a 150 mm face 210 mm ahead, centred
     * and 40 mm left of centre. A ray echoes while any part of its beam
     * (+-OBS_BEAM_HALF_DEG) falls on the face, so edges appear wide. Face
     * edges in degrees: atan(-75/210)=-19.7, atan(75/210)=19.7 centred;
     * atan(-35/210)=-9.5, atan(115/210)=28.7 offset. */
    {
        static const int32_t edges_x10[2][2]={{-197,197},{-95,287}};
        /* Range: on-axis rays see the face at 210 / cos(a); rays that only
         * catch it with the beam's edge see the nearer corner:
         * sqrt(210^2+75^2)=223 centred; 213 (right) / 239 (left) offset. */
        static const int32_t corner_mm[2][2]={{223,223},{213,239}};
        static const int32_t cos_milli[15]={1000,996,985,966,940,906,866,
            819,766,707,643,574,500,423,342};
        scan_plan_point_t pts[OBS_SCAN_POINTS]={{0}};
        scan_plan_sides_t sides;
        for(int32_t k=0;k<2;k++)
        {
            int32_t n=0;
            for(int32_t a=-OBS_SCAN_RIGHT_DEG;a<=OBS_SCAN_LEFT_DEG &&
                n<OBS_SCAN_POINTS;a=scan_plan_next_angle(a))
            {
                bool const hit=(10*(a+OBS_BEAM_HALF_DEG)>edges_x10[k][0]) &&
                               (10*(a-OBS_BEAM_HALF_DEG)<edges_x10[k][1]);
                bool const on_face=(10*a>edges_x10[k][0]) &&
                                   (10*a<edges_x10[k][1]);
                int32_t const range=on_face?
                    210*1000/cos_milli[(a<0?-a:a)/5]:
                    corner_mm[k][a>0?1:0];
                pts[n].angle_deg=a;
                pts[n].range_mm=hit?(unsigned)range:4000U;
                n++;
            }
            scan_plan_build_side_report(pts,n,0,0,&plan,&sides);
            assert(scan_plan_result(&plan)==SCAN_PLAN_ROUTE);
            int32_t const w=scan_plan_width_cm(&plan)*10;
            printf("  150 mm face %s: reported width %d mm\n",
                   k?"offset 40 mm":"centred",(int)w);
            assert(w>=105 && w<=195);
            /* The log's mm width and the plan's cm width agree (rounding). */
            assert((int)sides.width_mm>=w-5 && (int)sides.width_mm<=w+5);
        }
        puts("PASS reported width corrects for the ultrasonic beam spread");
    }
    /* Side choice on the fair (beam-corrected) comparison and the tie
     * break, from logged geometry. */
    {
        scan_plan_point_t pts[OBS_SCAN_POINTS]={{0}};
        scan_plan_sides_t sides;
        int32_t n=0;
        for(int32_t a=-OBS_SCAN_RIGHT_DEG;a<=OBS_SCAN_LEFT_DEG &&
            n<OBS_SCAN_POINTS;a=scan_plan_next_angle(a))
        {pts[n].angle_deg=a;pts[n].range_mm=4000U;++n;}
        /* Logged: box echoing 5..30 deg (wholly left of centre) at ~155
         * mm. Going right is the short way; it must not be a tie. */
        for(int32_t i=0;i<n;i++)
            pts[i].range_mm=(pts[i].angle_deg>=5&&pts[i].angle_deg<=30)?
                155U:4000U;
        scan_plan_build_side_report(pts,n,0,0,&plan,&sides);
        assert(scan_plan_result(&plan)==SCAN_PLAN_ROUTE);
        assert(!sides.b_tie && !scan_plan_side_is_left(&plan));
        /* Mirror: wholly right of centre goes left. */
        for(int32_t i=0;i<n;i++)
            pts[i].range_mm=(pts[i].angle_deg>=-30&&pts[i].angle_deg<=-5)?
                155U:4000U;
        scan_plan_build_side_report(pts,n,0,0,&plan,&sides);
        assert(!sides.b_tie && scan_plan_side_is_left(&plan));
        /* Centred box: equal moves. Something 700 mm away on one side,
         * beyond the corridor: still clear, but less room, so it goes the
         * other way, whichever side it is on. */
        for(int32_t far_side=-60;far_side<=60;far_side+=120)
        {
            for(int32_t i=0;i<n;i++)
                pts[i].range_mm=(pts[i].angle_deg>=-20&&
                                 pts[i].angle_deg<=20)?220U:4000U;
            for(int32_t i=0;i<n;i++)
                if(pts[i].angle_deg==far_side) pts[i].range_mm=700U;
            scan_plan_build_side_report(pts,n,0,0,&plan,&sides);
            assert(sides.b_tie && sides.b_left_ok && sides.b_right_ok);
            assert(scan_plan_side_is_left(&plan)==(far_side<0));
        }
        /* Perfectly symmetric: nothing differs; still a route either way. */
        for(int32_t i=0;i<n;i++)
            pts[i].range_mm=(pts[i].angle_deg>=-20&&pts[i].angle_deg<=20)?
                220U:4000U;
        scan_plan_build_side_report(pts,n,0,0,&plan,&sides);
        assert(sides.b_tie && scan_plan_result(&plan)==SCAN_PLAN_ROUTE);
        puts("PASS one-sided boxes go the short way; equal moves go to the "
             "more open side");
    }
#if OBS_SIDE_CORNER_ECHO
    /* Logged 18 Nov 18:47 (box on the left): the sensor faces the box
     * square on, so -30..30 all read ~225 mm wherever the box sits; only a
     * corner echo past 30 degrees shows where it is. */
    {
        static const int32_t prof_deg[13]={-30,-25,-20,-15,-10,-5,0,5,10,15,
                                           20,25,30};
        static const int32_t prof_mm[13]={246,238,232,231,229,227,229,229,
                                          228,232,233,238,249};
        scan_plan_point_t pts[OBS_SCAN_POINTS]={{0}};
        scan_plan_sides_t sides;
        int32_t n=0;
        for(int32_t a=-OBS_SCAN_RIGHT_DEG;a<=OBS_SCAN_LEFT_DEG &&
            n<OBS_SCAN_POINTS;a=scan_plan_next_angle(a))
        {
            pts[n].angle_deg=a;pts[n].range_mm=4000U;
            for(int32_t k=0;k<13;k++)
                if(prof_deg[k]==a) pts[n].range_mm=(unsigned)prof_mm[k];
            ++n;
        }
        for(int32_t sign=-1;sign<=1;sign+=2)
        {
            /* Logged corner at +40: 320 mm. Mirrored for the right. */
            scan_plan_point_t m[OBS_SCAN_POINTS]={{0}};
            for(int32_t i=0;i<n;i++)
            {
                m[i]=pts[i];
                if(sign<0)
                    m[i].range_mm=pts[n-1-i].range_mm;
                if(m[i].angle_deg==40*sign) m[i].range_mm=320U;
            }
            scan_plan_build_side_report(m,n,0,0,&plan,&sides);
            assert(scan_plan_result(&plan)==SCAN_PLAN_ROUTE);
            assert(sides.b_centre);
            /* sqrt(320^2 - 227^2) = 225 mm out on that side. */
            scan_plan_side_info_t const *seen=sign>0?&sides.left:&sides.right;
            assert(seen->b_corner && seen->corner_mm>=215 &&
                   seen->corner_mm<=235);
            assert(sign*sides.centre_mm>100);
            /* Corner on the left: go right, and vice versa. */
            assert(!sides.b_tie);
            assert(scan_plan_side_is_left(&plan)==(sign<0));
        }
        /* No corner echo: the box is within the square-on zone. */
        scan_plan_build_side_report(pts,n,0,0,&plan,&sides);
        assert(!sides.b_centre);
        /* Barely beyond the face (logged 18:59: 227 mm at 40 degrees, face
         * 214): the face itself still echoing, not a corner. */
        for(int32_t i=0;i<n;i++)
            if(pts[i].angle_deg==40) pts[i].range_mm=240U;
        scan_plan_build_side_report(pts,n,0,0,&plan,&sides);
        assert(!sides.b_centre);
        /* A far echo in the corner band is another object, not a corner. */
        for(int32_t i=0;i<n;i++)
            if(pts[i].angle_deg==40) pts[i].range_mm=1135U;
        scan_plan_build_side_report(pts,n,0,0,&plan,&sides);
        assert(!sides.b_centre);
        /* Both corners: the centre is midway between them. */
        for(int32_t i=0;i<n;i++)
        {
            if(pts[i].angle_deg==40) pts[i].range_mm=320U;  /* ~225 left */
            if(pts[i].angle_deg==-40) pts[i].range_mm=265U; /* ~137 right */
        }
        scan_plan_build_side_report(pts,n,0,0,&plan,&sides);
        assert(sides.b_centre && sides.left.b_corner && sides.right.b_corner);
        assert(sides.centre_mm>30 && sides.centre_mm<60);
        assert(!scan_plan_side_is_left(&plan));
        /* The robot drives out far enough to clear a seen corner. */
        assert(sides.right_need_mm>=sides.right.corner_mm+
               OBS_ROBOT_WIDTH_MM/2+OBS_SAFETY_MARGIN_MM);
        /* A corner heard on one ping only (lone): kept when the next
         * corner bearing heard the same range, not on its own. */
        for(int32_t i=0;i<n;i++)
            if(pts[i].angle_deg==40||pts[i].angle_deg==-40)
                pts[i].range_mm=4000U;
        for(int32_t i=0;i<n;i++)
            if(pts[i].angle_deg==40) pts[i].lone_mm=279U;
        scan_plan_build_side_report(pts,n,0,0,&plan,&sides);
        assert(!sides.b_centre);
        for(int32_t i=0;i<n;i++)
            if(pts[i].angle_deg==45) pts[i].lone_mm=290U;
        scan_plan_build_side_report(pts,n,0,0,&plan,&sides);
        assert(sides.b_centre && sides.left.b_corner && !sides.right.b_corner);
        assert(!scan_plan_side_is_left(&plan));
        /* Neighbour at a different range: a different thing, not kept. */
        for(int32_t i=0;i<n;i++)
            if(pts[i].angle_deg==45) pts[i].lone_mm=400U;
        scan_plan_build_side_report(pts,n,0,0,&plan,&sides);
        assert(!sides.b_centre);
        for(int32_t i=0;i<n;i++) pts[i].lone_mm=0U;
        /* The robot stopped 7.5 degrees off square, the normal to the
         * left: the face still echoes 35 degrees out on the left (delayed,
         * as at the zone's edge), which is not a corner. The same echo at
         * -35 is 42.5 degrees off the normal: a corner. */
        {
            scan_plan_point_t t[OBS_SCAN_POINTS]={{0}};
            for(int32_t i=0;i<n;i++)
            {
                int32_t const a=pts[i].angle_deg;
                int32_t const off=(a>7?a-7:7-a); /* about the normal */
                t[i].angle_deg=a;
                t[i].range_mm=(a>=-25&&a<=35)?(unsigned)(214+off*off/25):4000U;
            }
            scan_plan_build_side_report(t,n,0,0,&plan,&sides);
            assert(sides.tilt_x2>=10 && sides.tilt_x2<=20);
            assert(!sides.left.b_corner);
            for(int32_t i=0;i<n;i++)
            {
                int32_t const a=t[i].angle_deg;
                if(a==35) t[i].range_mm=4000U;
                if(a==-35) t[i].range_mm=280U;
                if(a==-25||a==-30) t[i].range_mm=4000U;
            }
            scan_plan_build_side_report(t,n,0,0,&plan,&sides);
            assert(sides.right.b_corner && !sides.left.b_corner);
            assert(scan_plan_side_is_left(&plan));
        }
        puts("PASS corner echoes locate an off-centre box the square-on "
             "face hides");
    }
#endif
    puts("All planner scenarios passed.");
    return 0;
}
