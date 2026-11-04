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
static const nav_config_t cfg={150U,250U,2500U,1800U,120U,400U,180U};
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
    CHECK(n.state==NAV_APPROACH);
    tick(1450U,1240U,0U,0U);
    CHECK(n.state==NAV_APPROACH);
    /* Properly confirmed junction, with a measured barcode gap. */
    tick(1600U,1300U,1U,1U);
    tick(1770U,1380U,1U,1U);
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
    /* A scan window cannot grant unlimited forward travel past a junction. */
    ready();
    s.barcode_capturing=1U;
    tick(1300U,100U,1U,1U);
    tick(1750U,150U,1U,1U);
    CHECK(n.state==NAV_FAULT && result.drive==NAV_OUTPUT_STOP);
    /* Distance bound is independent of elapsed time bound. */
    ready();
    s.barcode_capturing=1U;
    tick(1300U,100U,1U,1U);
    tick(1400U,290U,1U,1U);
    CHECK(n.state==NAV_FAULT && result.drive==NAV_OUTPUT_STOP);
    s.barcode_capturing=0U;
    /* Timed-out turn stops; stale sensor/hardware safety stays in owner. */
    ready();
    CHECK(nav_barcode(&n,0U,NAV_UTURN,10U,1100U,100U)==1);
    tick(1300U,240U,1U,1U);tick(1460U,290U,1U,1U);
    CHECK(result.drive==NAV_OUTPUT_UTURN);
    tick(4050U,360U,0U,0U);
    CHECK(n.state==NAV_FAULT && result.drive==NAV_OUTPUT_STOP);
    /* Straight traverses junction then returns to ordinary following. */
    ready();
    CHECK(nav_barcode(&n,3U,NAV_STRAIGHT,11U,1100U,100U)==1);
    tick(1300U,280U,1U,1U);tick(1460U,340U,1U,1U);
    CHECK(result.drive==NAV_OUTPUT_FORWARD);
    tick(1750U,450U,0U,0U);tick(1920U,520U,0U,0U);
    CHECK(n.state==NAV_FOLLOW);
    /* A physical barcode close to junction needs configured clearance. */
    ready();
    CHECK(nav_barcode(&n,4U,NAV_RIGHT,12U,1100U,100U)==1);
    tick(1300U,120U,1U,1U);tick(1460U,140U,1U,1U);
    CHECK(n.state==NAV_FAULT);
    printf("AUTONAV PASS: %d assertions\n",pass);
    return 0;
}
