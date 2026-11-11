/** @file test_autonav.c
 * @brief Host-only deterministic robot navigation regression harness.
 */
#include "app/nav_core.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } \
    ++pass; } while (0)
static int pass;
/* Junction = 96 counts (~20 mm) of unbroken both-black, or 1500 ms. */
static const nav_config_t cfg={150U,250U,2500U,1800U,120U,96U,1500U,0U,0U,0U,0U,0U,0U};
static const nav_config_t cfg_wait={150U,250U,2500U,1800U,120U,96U,1500U,
                                    8000U,1500U,0U,0U,0U,0U};
static nav_controller_t n;
static nav_sample_t s;
static nav_result_t result;
static void tick(uint32_t now,uint32_t distance,uint8_t left,uint8_t right)
{
    s.now_msec=now;s.travel_counts=distance;
    s.left_black=left;s.right_black=right;
    result=nav_step(&n,&s);
}
static void ready(void)
{
    nav_init(&n,&cfg);
    nav_start(&n,1000U);
    s=(nav_sample_t){0};
    CHECK(n.state==NAV_FOLLOW);
}
int main(void)
{
    ready();
    CHECK(nav_barcode(&n,25U,NAV_LEFT,1U,1200U,1000U)==1);
    CHECK(n.state==NAV_APPROACH);
    tick(1300U,1070U,0U,0U);
    CHECK(result.drive==NAV_OUTPUT_FOLLOW);
    /* A short both-black pulse must not drive a turn. */
    tick(1400U,1240U,1U,1U);
    CHECK(n.state==NAV_APPROACH && result.drive==NAV_OUTPUT_FORWARD);
    tick(1450U,1250U,0U,0U);
    CHECK(n.state==NAV_APPROACH);
    /* Properly confirmed junction, with a measured barcode gap. */
    tick(1600U,1300U,1U,1U);
    tick(1700U,1380U,1U,1U);
    CHECK(n.state==NAV_APPROACH && result.drive==NAV_OUTPUT_FORWARD);
    tick(1770U,1400U,1U,1U);
    CHECK(n.state==NAV_TURN);
    CHECK(result.drive==NAV_OUTPUT_LEFT);
    CHECK(result.event==NAV_EVENT_JUNCTION);
    tick(1950U,1400U,1U,1U);
    CHECK(result.drive==NAV_OUTPUT_LEFT);
    tick(2080U,1440U,1U,0U);
    CHECK(n.state==NAV_REACQUIRE);
    tick(2250U,1480U,1U,0U);
    CHECK(n.state==NAV_RESUME);
    CHECK(result.event==NAV_EVENT_TURN_COMPLETED);
    CHECK(result.drive==NAV_OUTPUT_FOLLOW);
    tick(2310U,1500U,0U,0U);
    CHECK(n.state==NAV_FOLLOW);
    CHECK(result.event==NAV_EVENT_LINE_REACQUIRED);
    tick(2360U,1520U,0U,0U);
    CHECK(result.event==NAV_EVENT_FOLLOWING);
    /* Repeated scans can be queued without a reboot or START. */
    CHECK(nav_barcode(&n,1U,NAV_RIGHT,2U,2400U,1550U)==1);
    CHECK(nav_barcode(&n,1U,NAV_RIGHT,2U,2401U,1550U)==0);
    CHECK(nav_barcode(&n,2U,NAV_UTURN,3U,2420U,1570U)==-1);
    nav_stop(&n);
    CHECK(n.state==NAV_WAIT && n.pending==0U);
    tick(4000U,2000U,1U,1U);
    CHECK(result.drive==NAV_OUTPUT_STOP);
    /* No motors start on reconnect unless explicit nav_start. */
    ready();
    tick(1300U,800U,1U,1U);
    tick(1470U,900U,1U,1U);
    CHECK(n.state==NAV_FAULT && result.drive==NAV_OUTPUT_STOP);
    /* Failed barcode scan by itself is not a navigation fault. Keep
     * following the line, even if the sensor reported an undecodable mark. */
    ready();
    tick(1300U,120U,0U,1U);
    CHECK(n.state==NAV_FOLLOW && result.drive==NAV_OUTPUT_FOLLOW);
    /* During a *reported* scanner window, allow only bounded travel over
     * both-black barcode ink. Do not turn or enter NAV_FAULT for this. */
    s.barcode_capturing=1U;
    tick(1400U,200U,1U,1U);
    CHECK(n.state==NAV_FOLLOW && result.drive==NAV_OUTPUT_FORWARD);
    tick(1630U,260U,1U,1U);
    CHECK(n.state==NAV_FOLLOW && result.drive==NAV_OUTPUT_FORWARD);
    /* Line seen again after a failed decode, no STOP/START required. */
    s.barcode_capturing=0U;
    tick(1710U,275U,1U,0U);
    CHECK(n.state==NAV_FOLLOW && result.drive==NAV_OUTPUT_FOLLOW);
    /* A scan window cannot grant unlimited forward travel past a junction:
     * the confirmation distance still applies. */
    ready();
    s.barcode_capturing=1U;
    tick(1300U,100U,1U,1U);
    tick(1400U,196U,1U,1U);
    CHECK(n.state==NAV_FAULT && result.drive==NAV_OUTPUT_STOP);
    s.barcode_capturing=0U;
    /* Time bound is independent of distance: a robot that barely moves
     * (stall, lifted) is stopped, never left rolling. */
    ready();
    tick(1300U,100U,1U,1U);
    tick(2700U,110U,1U,1U);
    CHECK(result.drive==NAV_OUTPUT_FORWARD);
    tick(2800U,112U,1U,1U);
    CHECK(n.state==NAV_FAULT && result.drive==NAV_OUTPUT_STOP);
    /* Timed-out turn stops; stale sensor/hardware safety stays in owner. */
    ready();
    CHECK(nav_barcode(&n,0U,NAV_UTURN,10U,1100U,100U)==1);
    tick(1300U,240U,1U,1U);tick(1460U,340U,1U,1U);
    CHECK(result.drive==NAV_OUTPUT_UTURN);
    tick(4050U,360U,0U,0U);
    CHECK(n.state==NAV_FAULT && result.drive==NAV_OUTPUT_STOP);
    /* Straight traverses junction then returns to ordinary following. */
    ready();
    CHECK(nav_barcode(&n,3U,NAV_STRAIGHT,11U,1100U,100U)==1);
    tick(1300U,280U,1U,1U);tick(1460U,380U,1U,1U);
    CHECK(result.drive==NAV_OUTPUT_FORWARD);
    tick(1750U,450U,0U,0U);tick(1920U,520U,0U,0U);
    CHECK(n.state==NAV_FOLLOW);
    /* A physical barcode close to junction needs configured clearance. */
    ready();
    CHECK(nav_barcode(&n,4U,NAV_RIGHT,12U,1100U,100U)==1);
    tick(1300U,105U,1U,1U);tick(1460U,205U,1U,1U);
    CHECK(n.state==NAV_FAULT);
    /* Logged: the junction tape passed the barcode sensor and started a
     * capture right at the junction. With an action queued (barcode well
     * behind) that is still a junction: take the action, do not fault. */
    ready();
    CHECK(nav_barcode(&n,5U,NAV_RIGHT,13U,1100U,100U)==1);
    s.barcode_capturing=1U;
    tick(1300U,800U,1U,1U);tick(1460U,900U,1U,1U);
    CHECK(n.state==NAV_TURN && result.event==NAV_EVENT_JUNCTION);
    s.barcode_capturing=0U;
    /* Decode wait disabled (0): junction during a decode faults at once. */
    ready();
    s.barcode_decoding=1U;
    tick(1300U,800U,1U,1U);tick(1470U,900U,1U,1U);
    CHECK(n.state==NAV_FAULT && n.fault_reason==1U);
    s.barcode_decoding=0U;
    /* Junction reached while the barcode is still decoding: stop and wait,
     * then take the mapped action when the letter arrives. */
    nav_init(&n,&cfg_wait);nav_start(&n,1000U);s=(nav_sample_t){0};
    s.barcode_decoding=1U;
    s.barcode_capturing=1U; /* guard window overlaps the decode start */
    tick(1300U,800U,1U,1U);
    CHECK(result.drive==NAV_OUTPUT_FORWARD); /* still confirming */
    tick(1470U,900U,1U,1U);
    CHECK(n.decode_hold==1U && result.drive==NAV_OUTPUT_STOP);
    CHECK(result.event==NAV_EVENT_DECODE_WAIT && n.state!=NAV_FAULT);
    s.barcode_capturing=0U;
    tick(3000U,800U,1U,1U);
    CHECK(n.decode_hold==1U && result.drive==NAV_OUTPUT_STOP);
    s.barcode_decoding=0U; /* decode done; letter reaches navigation */
    tick(3100U,800U,1U,1U);
    CHECK(n.decode_hold==1U && result.drive==NAV_OUTPUT_STOP);
    CHECK(nav_barcode(&n,24U,NAV_LEFT,30U,3150U,800U)==1);
    tick(3160U,800U,1U,1U);
    CHECK(n.decode_hold==0U && n.state==NAV_TURN);
    CHECK(result.event==NAV_EVENT_JUNCTION && result.drive==NAV_OUTPUT_LEFT);
    /* Decode ends with no letter: safety stop after the grace period. */
    nav_init(&n,&cfg_wait);nav_start(&n,1000U);s=(nav_sample_t){0};
    s.barcode_decoding=1U;
    tick(1300U,800U,1U,1U);tick(1470U,900U,1U,1U);
    CHECK(n.decode_hold==1U);
    s.barcode_decoding=0U;
    tick(2000U,800U,1U,1U);
    CHECK(n.decode_hold==1U && n.state!=NAV_FAULT);
    tick(3600U,800U,1U,1U);
    CHECK(n.state==NAV_FAULT && n.fault_reason==5U);
    CHECK(result.drive==NAV_OUTPUT_STOP && result.event==NAV_EVENT_FAULT);
    /* A decode that never finishes is bounded by the wait limit. */
    nav_init(&n,&cfg_wait);nav_start(&n,1000U);s=(nav_sample_t){0};
    s.barcode_decoding=1U;
    tick(1300U,800U,1U,1U);tick(1470U,900U,1U,1U);
    tick(9000U,800U,1U,1U);
    CHECK(n.decode_hold==1U);
    tick(9480U,800U,1U,1U);
    CHECK(n.state==NAV_FAULT && n.fault_reason==5U);
    /* STOP clears a hold; a new START does not inherit it. */
    nav_init(&n,&cfg_wait);nav_start(&n,1000U);s=(nav_sample_t){0};
    s.barcode_decoding=1U;
    tick(1300U,800U,1U,1U);tick(1470U,900U,1U,1U);
    nav_stop(&n);CHECK(n.decode_hold==0U);
    nav_start(&n,2000U);CHECK(n.decode_hold==0U && n.state==NAV_FOLLOW);
    /* Issue: a curve stopped the robot. The line crossed diagonally under
     * both sensors for one reading; the robot used to stop at once and
     * then "confirm" the frozen reading. Now it rolls on and keeps
     * following without any fault. */
    ready();
    tick(1300U,100U,1U,0U);
    CHECK(result.drive==NAV_OUTPUT_FOLLOW);
    tick(1350U,113U,1U,1U);
    CHECK(n.state==NAV_FOLLOW && result.drive==NAV_OUTPUT_FORWARD);
    tick(1400U,126U,0U,1U);
    CHECK(n.state==NAV_FOLLOW && result.drive==NAV_OUTPUT_FOLLOW);
    /* Even several both-black readings on a tight bend stay below the
     * confirmation distance. */
    tick(1450U,139U,1U,1U);tick(1500U,152U,1U,1U);tick(1550U,165U,1U,1U);
    CHECK(n.state==NAV_FOLLOW && result.drive==NAV_OUTPUT_FORWARD);
    tick(1600U,178U,1U,0U);
    CHECK(n.state==NAV_FOLLOW && result.drive==NAV_OUTPUT_FOLLOW);
    /* Black readings broken by white between reads (seen by the 1 ms
     * sampler) restart the measurement, so barcode bars or a bend swept
     * twice cannot add up to a junction. */
    ready();
    tick(1300U,100U,1U,1U);
    s.both_interrupted=1U;
    tick(1350U,150U,1U,1U);
    s.both_interrupted=0U;
    tick(1400U,205U,1U,1U); /* 105 counts since 100, only 55 since 150 */
    CHECK(n.state==NAV_FOLLOW && result.drive==NAV_OUTPUT_FORWARD);
    tick(1450U,246U,1U,1U);
    CHECK(n.state==NAV_FAULT && result.drive==NAV_OUTPUT_STOP);
    /* Junction square (265 mm) at ~13 counts per 50 ms: the robot drives
     * in, confirms after ~20 mm and stops (no letter queued: safety stop)
     * well inside the square. */
    ready();
    {
        uint32_t c=1000U, t=1300U;
        int steps=0;
        tick(t,c,0U,0U);
        while (n.state==NAV_FOLLOW && steps<40)
        {
            t+=50U; c+=13U; ++steps;
            tick(t,c,1U,1U);
        }
        CHECK(n.state==NAV_FAULT && result.drive==NAV_OUTPUT_STOP);
        CHECK(c-1013U>=96U && c-1013U<96U+13U); /* stops ~20 mm in */
    }
    /* With a letter queued the same square starts the mapped turn. */
    ready();
    CHECK(nav_barcode(&n,5U,NAV_RIGHT,40U,1100U,500U)==1);
    {
        uint32_t c=1000U, t=1300U;
        int steps=0;
        while (n.state==NAV_APPROACH && steps<40)
        {
            t+=50U; c+=13U; ++steps;
            tick(t,c,1U,1U);
        }
        CHECK(n.state==NAV_TURN && result.drive==NAV_OUTPUT_RIGHT);
        CHECK(result.event==NAV_EVENT_JUNCTION);
    }
    /* Logged run (auto_run1, junction-debug build): the crossing bar is
     * tape, both-black for only ~15 mm, read at run=0,15,29,45,59. With
     * the 20 mm threshold (96) it was never confirmed; with 5 mm (24) it
     * is confirmed on the third read, still on the tape. */
    {
        static const nav_config_t tape={150U,250U,2500U,1800U,120U,24U,
                                        1500U,0U,0U,0U,0U,0U,0U};
        nav_init(&n,&tape);nav_start(&n,1000U);s=(nav_sample_t){0};
        tick(1300U,1000U,0U,1U);
        tick(1350U,1000U,1U,1U);
        CHECK(result.drive==NAV_OUTPUT_FORWARD);
        tick(1400U,1015U,1U,1U);
        CHECK(result.drive==NAV_OUTPUT_FORWARD);
        tick(1450U,1029U,1U,1U);
        CHECK(n.state==NAV_FAULT && result.drive==NAV_OUTPUT_STOP);
        /* One both-black read on a curve still does not stop it. */
        nav_init(&n,&tape);nav_start(&n,1000U);s=(nav_sample_t){0};
        tick(1300U,1000U,0U,1U);
        tick(1350U,1015U,1U,1U);
        tick(1400U,1030U,0U,1U);
        CHECK(n.state==NAV_FOLLOW && result.drive==NAV_OUTPUT_FOLLOW);
    }
    /* Pivot turns (firmware config): drive on 643 counts from the bar's
     * first both-black reading to the centre, then pivot by encoder angle,
     * 826 counts (both wheels) per 90 degrees. */
    {
        static const nav_config_t piv={150U,250U,6000U,1800U,120U,24U,
                                       1500U,8000U,1500U,643U,5000U,826U,0U};
        uint32_t t, c, k;
        /* Left: junction confirmed, drive to the centre, pivot. */
        nav_init(&n,&piv);nav_start(&n,1000U);s=(nav_sample_t){0};
        CHECK(nav_barcode(&n,11U,NAV_LEFT,50U,1100U,0U)==1);
        tick(1300U,1000U,1U,1U);            /* bar first seen at 1000 */
        tick(1350U,1015U,1U,1U);
        tick(1400U,1030U,1U,1U);
        CHECK(n.state==NAV_CENTRE && result.event==NAV_EVENT_JUNCTION);
        CHECK(result.drive==NAV_OUTPUT_FORWARD);
        tick(1450U,1045U,0U,0U);            /* past the bar: follow arm */
        CHECK(n.state==NAV_CENTRE && result.drive==NAV_OUTPUT_FOLLOW);
        tick(1500U,1500U,0U,1U);
        CHECK(n.state==NAV_CENTRE && result.drive==NAV_OUTPUT_FOLLOW);
        s.turn_counts=5000U;
        tick(1550U,1643U,0U,0U);            /* axle over the centre */
        CHECK(n.state==NAV_TURN && result.drive==NAV_OUTPUT_LEFT);
        CHECK(result.event==NAV_EVENT_TURN_STARTED);
        /* The arm straight ahead passes under a sensor early: ignored. */
        s.turn_counts=5100U;tick(1600U,1643U,0U,1U);
        CHECK(n.state==NAV_TURN && result.drive==NAV_OUTPUT_LEFT);
        s.turn_counts=5500U;tick(1650U,1643U,0U,0U);
        CHECK(n.state==NAV_TURN);
        s.turn_counts=5560U;tick(1700U,1643U,1U,0U); /* 560 >= 550 gate */
        CHECK(n.state==NAV_RESUME && result.event==NAV_EVENT_TURN_COMPLETED);
        CHECK(result.drive==NAV_OUTPUT_STOP && n.pending==0U);
        tick(1750U,1643U,1U,0U);
        CHECK(n.state==NAV_FOLLOW && result.drive==NAV_OUTPUT_FOLLOW);
        /* U-turn: the side arm at 90 degrees is ignored, the arm behind
         * (180) ends it, driven by the right output. */
        nav_init(&n,&piv);nav_start(&n,1000U);s=(nav_sample_t){0};
        CHECK(nav_barcode(&n,20U,NAV_UTURN,51U,1100U,0U)==1);
        tick(1300U,1000U,1U,1U);tick(1350U,1015U,1U,1U);
        tick(1400U,1030U,1U,1U);
        tick(1450U,1643U,0U,0U);
        CHECK(n.state==NAV_TURN && result.drive==NAV_OUTPUT_UTURN);
        for (k=1U, t=1500U; k<=40U; ++k, t+=50U)
        {
            s.turn_counts=k*32U;            /* 32 counts per period */
            /* Black at 90 degrees (826) and again at 180 (1652). */
            tick(t,1643U,(uint8_t)(k==26U||k==52U),0U);
            if (n.state!=NAV_TURN) break;
        }
        CHECK(n.state==NAV_TURN && result.drive==NAV_OUTPUT_UTURN);
        for (; k<=60U && n.state==NAV_TURN; ++k, t+=50U)
        {
            s.turn_counts=k*32U;
            tick(t,1643U,(uint8_t)(k==52U),0U);
        }
        CHECK(n.state==NAV_RESUME && s.turn_counts==52U*32U);
        /* Right turn that never finds the new arm faults past 135 deg. */
        nav_init(&n,&piv);nav_start(&n,1000U);s=(nav_sample_t){0};
        CHECK(nav_barcode(&n,17U,NAV_RIGHT,52U,1100U,0U)==1);
        tick(1300U,1000U,1U,1U);tick(1350U,1015U,1U,1U);
        tick(1400U,1030U,1U,1U);tick(1450U,1643U,0U,0U);
        CHECK(n.state==NAV_TURN && result.drive==NAV_OUTPUT_RIGHT);
        s.turn_counts=1239U;tick(1500U,1643U,0U,0U);
        CHECK(n.state==NAV_TURN);
        s.turn_counts=1240U;tick(1550U,1643U,0U,0U);
        CHECK(n.state==NAV_FAULT && n.fault_reason==2U);
        CHECK(result.drive==NAV_OUTPUT_STOP);
        /* Logged U-turn: the sensor reached the line on the same reading
         * the count limit tripped. That is a finished turn, not a fault. */
        nav_init(&n,&piv);nav_start(&n,1000U);s=(nav_sample_t){0};
        CHECK(nav_barcode(&n,24U,NAV_UTURN,56U,1100U,0U)==1);
        tick(1300U,1000U,1U,1U);tick(1350U,1015U,1U,1U);
        tick(1400U,1030U,1U,1U);tick(1450U,1643U,0U,0U);
        CHECK(n.state==NAV_TURN && result.drive==NAV_OUTPUT_UTURN);
        s.turn_counts=2107U;tick(1500U,1643U,1U,0U); /* > 2065 limit */
        CHECK(n.state==NAV_RESUME && result.event==NAV_EVENT_TURN_COMPLETED);
        /* Drive to the centre is time bounded (stall, lifted). */
        nav_init(&n,&piv);nav_start(&n,1000U);s=(nav_sample_t){0};
        CHECK(nav_barcode(&n,17U,NAV_RIGHT,53U,1100U,0U)==1);
        tick(1300U,1000U,1U,1U);tick(1350U,1015U,1U,1U);
        tick(1400U,1030U,1U,1U);
        CHECK(n.state==NAV_CENTRE);
        tick(6500U,1200U,0U,0U);
        CHECK(n.state==NAV_FAULT && n.fault_reason==3U);
        /* Straight is unchanged: drives over, no centring or pivot. */
        nav_init(&n,&piv);nav_start(&n,1000U);s=(nav_sample_t){0};
        CHECK(nav_barcode(&n,18U,NAV_STRAIGHT,54U,1100U,0U)==1);
        tick(1300U,1000U,1U,1U);tick(1350U,1015U,1U,1U);
        tick(1400U,1030U,1U,1U);
        CHECK(n.state==NAV_CROSS && result.drive==NAV_OUTPUT_FORWARD);
        /* Letter arrives while stopped at the junction for the decode:
         * the centre is still measured from the bar, not from zero. */
        nav_init(&n,&piv);nav_start(&n,1000U);s=(nav_sample_t){0};
        s.barcode_decoding=1U;
        tick(1300U,1000U,1U,1U);tick(1350U,1015U,1U,1U);
        tick(1400U,1030U,1U,1U);
        CHECK(n.decode_hold==1U && result.drive==NAV_OUTPUT_STOP);
        s.barcode_decoding=0U;
        CHECK(nav_barcode(&n,11U,NAV_LEFT,55U,2000U,1040U)==1);
        tick(2050U,1040U,1U,1U);
        CHECK(n.state==NAV_CENTRE && result.drive==NAV_OUTPUT_FORWARD);
        c=1500U;tick(2100U,c,0U,0U);
        CHECK(n.state==NAV_CENTRE);
        tick(2150U,1643U,0U,0U);
        CHECK(n.state==NAV_TURN && result.drive==NAV_OUTPUT_LEFT);
    }
    /* Junction action mode. Single (0): the action is used once, so the
     * next junction without a new barcode is a safety stop. Repeat (1): it
     * is done at every junction until another barcode replaces it. */
    {
        static const nav_config_t one={150U,250U,6000U,1800U,120U,24U,
                                       1500U,8000U,1500U,643U,5000U,826U,0U};
        static const nav_config_t rep={150U,250U,6000U,1800U,120U,24U,
                                       1500U,8000U,1500U,643U,5000U,826U,1U};
        /* Junction 1 at 1000 counts, crossed straight; junction 2 at 3000. */
        #define JUNCTION_AT(t0,c0) do { tick((t0),(c0),1U,1U); \
            tick((t0)+50U,(c0)+15U,1U,1U); tick((t0)+100U,(c0)+30U,1U,1U); \
        } while (0)
        #define CROSS_DONE(t0,c0) do { tick((t0),(c0),0U,0U); \
            tick((t0)+200U,(c0)+50U,0U,0U); } while (0)
        nav_init(&n,&one);nav_start(&n,1000U);s=(nav_sample_t){0};
        CHECK(nav_barcode(&n,18U,NAV_STRAIGHT,60U,1100U,0U)==1);
        JUNCTION_AT(1300U,1000U);
        CHECK(n.state==NAV_CROSS);
        CROSS_DONE(1700U,1100U);
        CHECK(n.state==NAV_FOLLOW && n.pending==0U);
        JUNCTION_AT(2500U,3000U);
        CHECK(n.state==NAV_FAULT && n.fault_reason==1U);
        /* Repeat: the same straight is done at junction 2 as well. */
        nav_init(&n,&rep);nav_start(&n,1000U);s=(nav_sample_t){0};
        CHECK(nav_barcode(&n,18U,NAV_STRAIGHT,60U,1100U,0U)==1);
        JUNCTION_AT(1300U,1000U);
        CROSS_DONE(1700U,1100U);
        CHECK(n.state==NAV_FOLLOW && n.pending==1U);
        JUNCTION_AT(2500U,3000U);
        CHECK(n.state==NAV_CROSS && result.event==NAV_EVENT_JUNCTION);
        CROSS_DONE(2900U,3100U);
        /* A new barcode replaces it (single mode would refuse with -1). */
        CHECK(nav_barcode(&n,5U,NAV_LEFT,61U,3200U,3200U)==1);
        CHECK(n.action==NAV_LEFT && n.held_action==NAV_LEFT);
        JUNCTION_AT(3500U,5000U);
        CHECK(n.state==NAV_CENTRE && result.value==NAV_LEFT);
        /* An obstacle bypass (nav_stop, later nav_start) keeps it ... */
        nav_stop(&n);nav_start(&n,4000U);
        CHECK(n.pending==1U && n.action==NAV_LEFT);
        JUNCTION_AT(4100U,7000U);
        CHECK(n.state==NAV_CENTRE);
        /* ... STOP forgets it: the next START has nothing queued. */
        nav_forget(&n);nav_start(&n,5000U);
        CHECK(n.pending==0U && n.held==0U);
        JUNCTION_AT(5100U,9000U);
        CHECK(n.state==NAV_FAULT && n.fault_reason==1U);
        /* A barcode still decoding at a junction is waited for even with an
         * action remembered; a failed decode falls back to that action. */
        nav_init(&n,&rep);nav_start(&n,1000U);s=(nav_sample_t){0};
        CHECK(nav_barcode(&n,18U,NAV_STRAIGHT,62U,1100U,0U)==1);
        s.barcode_decoding=1U;
        JUNCTION_AT(1300U,1000U);
        CHECK(n.decode_hold==1U && result.drive==NAV_OUTPUT_STOP);
        CHECK(n.state!=NAV_CROSS);
        s.barcode_decoding=0U;
        tick(2000U,1030U,1U,1U);
        CHECK(n.decode_hold==1U);
        tick(3600U,1030U,1U,1U);                /* grace over, no letter */
        CHECK(n.state==NAV_CROSS && n.decode_hold==0U);
        /* ... and a letter that does arrive is used instead. */
        nav_init(&n,&rep);nav_start(&n,1000U);s=(nav_sample_t){0};
        CHECK(nav_barcode(&n,18U,NAV_STRAIGHT,63U,1100U,0U)==1);
        s.barcode_decoding=1U;
        JUNCTION_AT(1300U,1000U);
        CHECK(n.decode_hold==1U);
        s.barcode_decoding=0U;
        CHECK(nav_barcode(&n,20U,NAV_UTURN,64U,2000U,1030U)==1);
        tick(2050U,1030U,1U,1U);
        CHECK(n.state==NAV_CENTRE && result.value==NAV_UTURN);
        #undef JUNCTION_AT
        #undef CROSS_DONE
    }
    /* V15: boundary results, genuine sequence wrap and confirmed junction
     * evidence must survive a result arriving just before confirmation. */
    for (uint32_t offset=0U;offset<=8001U;offset+=8000U)
    {
        nav_init(&n,&cfg_wait);nav_start(&n,1000U);s=(nav_sample_t){0};
        s.barcode_decoding=1U;
        tick(1300U,1000U,1U,1U);tick(1400U,1100U,1U,1U);
        CHECK(n.decode_hold && result.drive==NAV_OUTPUT_STOP);
        CHECK(nav_barcode(&n,6U,NAV_LEFT,100U,1400U+offset,1100U)==1);
        s.barcode_decoding=0U;
        tick(1400U+offset,1100U,1U,1U);
        CHECK(n.state==NAV_TURN && result.drive==NAV_OUTPUT_LEFT);
    }
    nav_init(&n,&cfg_wait);nav_start(&n,1000U);s=(nav_sample_t){0};
    s.barcode_decoding=1U;
    tick(1300U,1000U,1U,1U);tick(1400U,1100U,1U,1U);
    CHECK(nav_barcode(&n,6U,NAV_LEFT,101U,9401U,1100U)==-3);
    tick(9401U,1100U,1U,1U);
    CHECK(n.state==NAV_FAULT && result.drive==NAV_OUTPUT_STOP);
    CHECK(nav_barcode(&n,6U,NAV_LEFT,102U,9450U,1100U)==-2);
    nav_forget(&n);tick(9500U,1100U,1U,1U);
    CHECK(result.drive==NAV_OUTPUT_STOP && n.state==NAV_WAIT);
    nav_start(&n,9600U);
    CHECK(nav_barcode(&n,6U,NAV_LEFT,65535U,9700U,100U)==1);
    nav_stop(&n);nav_start(&n,9800U);
    CHECK(nav_barcode(&n,6U,NAV_LEFT,0U,12000U,400U)==1);
    CHECK(nav_barcode(&n,6U,NAV_LEFT,0U,12001U,400U)==0);
    nav_stop(&n);nav_start(&n,12100U);
    CHECK(nav_barcode(&n,6U,NAV_LEFT,1U,14000U,800U)==1);
    CHECK(nav_barcode(&n,6U,NAV_LEFT,2U,14010U,801U)==0);
    CHECK(nav_barcode(&n,26U,NAV_LEFT,3U,14100U,1000U)==-2);
    CHECK(nav_barcode(&n,6U,4U,3U,14100U,1000U)==-2);
    /* A fresh letter does not erase the crossing's measured origin. */
    nav_init(&n,&cfg_wait);nav_start(&n,1000U);s=(nav_sample_t){0};
    tick(1300U,1000U,1U,1U);
    CHECK(nav_barcode(&n,6U,NAV_LEFT,4U,1350U,500U)==1);
    tick(1400U,1100U,1U,1U);
    CHECK(n.state==NAV_TURN && n.centre_from==1000U);
    /* A wait straddling the millisecond timer wrap has the same bound. */
    nav_init(&n,&cfg_wait);nav_start(&n,UINT32_MAX-200U);
    s=(nav_sample_t){0};s.barcode_decoding=1U;
    tick(UINT32_MAX-100U,1000U,1U,1U);
    tick(UINT32_MAX-50U,1100U,1U,1U);
    CHECK(n.decode_hold);
    CHECK(nav_barcode(&n,6U,NAV_LEFT,5U,25U,1100U)==1);
    tick(25U,1100U,1U,1U);
    CHECK(result.drive==NAV_OUTPUT_LEFT);
    /* Accumulated encoder travel is modulo uint32 too. */
    nav_init(&n,&cfg);nav_start(&n,1000U);s=(nav_sample_t){0};
    CHECK(nav_barcode(&n,6U,NAV_LEFT,6U,1100U,UINT32_MAX-200U)==1);
    tick(1300U,UINT32_MAX-50U,1U,1U);
    tick(1400U,50U,1U,1U);
    CHECK(n.state==NAV_TURN && result.drive==NAV_OUTPUT_LEFT);
    /* Completion was before the deadline; consumption can be just after
     * it without a false timeout, provided no fault was already entered. */
    nav_init(&n,&cfg_wait);nav_start(&n,1000U);s=(nav_sample_t){0};
    s.barcode_decoding=1U;
    tick(1300U,1000U,1U,1U);tick(1400U,1100U,1U,1U);
    CHECK(nav_barcode(&n,6U,NAV_LEFT,7U,9399U,1100U)==1);
    s.barcode_decoding=0U;tick(9450U,1100U,1U,1U);
    CHECK(n.state==NAV_TURN && result.drive==NAV_OUTPUT_LEFT);
    printf("AUTONAV PASS: %d assertions\n",pass);
    return 0;
}
