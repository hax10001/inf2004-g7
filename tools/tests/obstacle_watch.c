/* Run the real ultrasonic worker's side/front watch with fake hardware. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <tk/tkernel.h>
typedef UINT RELTIM; /* Kernel time type needed by the embedded worker. */
#include "modules/obstacle/task_obstacle.c"
#include "modules/obstacle/scan_plan.c"

static uint32_t clock_ms, side_count, front_count, side_time, max_gap;
static int32_t current_angle, published_side;
static int32_t watch_angle=-OBS_SIDE_LOOK_DEG;
static ultrasonic_result_t echo_result=ULTRASONIC_OK;
static int32_t front_range=1000, min_angle, max_angle;
static UINT worker_flags;
static scan_plan_bypass_t stored_plan;
static unsigned int plan_count;

INT tm_printf(UB const *format, ...) {(void)format;return 0;}
ER tk_dly_tsk(UINT delay) {clock_ms+=delay;return E_OK;}
ER tk_get_otm(SYSTIM *now) {now->hi=0;now->lo=clock_ms;return E_OK;}
/* Trigger times, to check scan ping spacing. */
static uint32_t last_trigger, min_trigger_gap=UINT32_MAX, pings;
static bool have_trigger;
/* A bearing whose echo comes and goes (logged at an object edge). */
static int32_t flaky_angle=999;
static unsigned int flaky_n;
bool ultrasonic_last_trigger(uint32_t *msec)
{if(!have_trigger)return false;*msec=last_trigger;return true;}
/* Simulated servo lag: it stops lag_deg short of each target, so the
 * real bearing depends on the direction it arrived from. */
static int32_t lag_deg, move_dir, lag_lo=-10, lag_hi=10;
static unsigned int left_moves, right_moves;
/* First bearing a scan pings (set by the first ultrasonic_ping after
 * scan_order_armed). */
static int32_t scan_order_first=999;
static bool scan_order_armed;
uint32_t servo_set_angle(int32_t angle)
{
    unsigned movement=(unsigned)abs(angle-current_angle);
    if(angle>current_angle) {move_dir=1;++left_moves;}
    if(angle<current_angle) {move_dir=-1;++right_moves;}
    current_angle=angle;
    if(angle<min_angle) min_angle=angle;
    if(angle>max_angle) max_angle=angle;
    return OBS_SERVO_SETTLE_BASE_MSEC + movement*OBS_SERVO_SETTLE_PER_DEG_MSEC;
}
int32_t obs_state_watch_angle_get(void) {return watch_angle;}
ultrasonic_result_t ultrasonic_ping(int32_t *range)
{
    if(have_trigger && clock_ms-last_trigger<min_trigger_gap)
        min_trigger_gap=clock_ms-last_trigger;
    last_trigger=clock_ms;have_trigger=true;++pings;
    if(scan_order_armed){scan_order_first=current_angle;scan_order_armed=false;}
    clock_ms+=OBS_ECHO_TIMEOUT_MSEC;
    if(current_angle==flaky_angle)
    {
        /* Echo, miss, echo, ... like 18 Nov's +20 degree edge. */
        if((flaky_n++)%2U) return ULTRASONIC_OUT_OF_RANGE;
        *range=232;
        return ULTRASONIC_OK;
    }
    if(lag_deg!=0)
    {
        int32_t const actual=current_angle-move_dir*lag_deg;
        if(actual<lag_lo || actual>lag_hi) return ULTRASONIC_OUT_OF_RANGE;
        *range=240;
        return ULTRASONIC_OK;
    }
    *range=current_angle==0?front_range:240;
    return echo_result;
}
UINT obs_state_flags_read(void) {return worker_flags;}
void obs_state_flags_set(UINT bits) {worker_flags|=bits;}
void obs_state_flags_clear(UINT bits) {worker_flags&=~bits;}
void obs_state_set_phase(obs_state_phase_t phase) {(void)phase;}
void obs_state_set_distance_mm(int32_t mm) {(void)mm;}
void obs_state_faults_update(uint32_t set,uint32_t clear)
{(void)set;(void)clear;}
uint32_t obs_state_status_get(void) {return 0U;}
uint32_t obs_state_status_distance_cm(uint32_t word)
{(void)word;return OBS_STOP_MM/10U;}
obs_state_route_t obs_state_status_route(uint32_t word)
{(void)word;return OBS_ROUTE_NONE;}
void obs_state_plan_store(scan_plan_bypass_t const *plan)
{stored_plan=*plan;++plan_count;}
int32_t servo_left_limit_deg(void) {return OBS_SERVO_LEFT_LIMIT_DEG;}
int32_t servo_right_limit_deg(void) {return OBS_SERVO_RIGHT_LIMIT_DEG;}
int32_t ultrasonic_median_of_three(int32_t a,int32_t b,int32_t c)
{
    if(a>b) {int32_t t=a;a=b;b=t;}
    if(b>c) {int32_t t=b;b=c;c=t;}
    return a>b?a:b;
}
void obs_state_side_range_set(int32_t range)
{
    assert(current_angle==watch_angle);
    if(side_count && clock_ms-side_time>max_gap) max_gap=clock_ms-side_time;
    side_time=clock_ms;published_side=range;++side_count;
}
void obs_state_front_range_set(int32_t range)
{
    assert(current_angle==0 && range==1000);
    ++front_count;
}

int main(void)
{
    side_watch_t watch={false,0U,0};
    UINT pattern=OBS_EVT_SIDE_WATCH|OBS_EVT_FRONT_WATCH;
    for(unsigned i=0;i<OBS_GLANCE_EVERY;++i)
    {
        run_side_watch(&watch,pattern);
        tk_dly_tsk(OBS_PING_PERIOD_MSEC);
    }
    assert(side_count==OBS_GLANCE_EVERY && front_count==0U);
    assert(published_side==240 && watch.b_aimed);
    puts("PASS worker keeps the ultrasonic aimed at the obstacle while passing");

#if (0 != OBS_PASS_FRONT_GLANCE)
    run_side_watch(&watch,pattern);
    tk_dly_tsk(OBS_PING_PERIOD_MSEC);
    assert(front_count==1U && side_count==OBS_GLANCE_EVERY);
    assert(current_angle==0 && !watch.b_aimed);
    run_side_watch(&watch,pattern);
    tk_dly_tsk(OBS_PING_PERIOD_MSEC);
    assert(side_count==OBS_GLANCE_EVERY+1U && watch.b_aimed);
    assert(current_angle==-OBS_SIDE_LOOK_DEG);
    assert(max_gap<OBS_SIDE_MAX_AGE_MSEC);
    puts("PASS front glance re-aims at the object within the freshness limit");
#else
    for(unsigned i=0;i<4U*OBS_GLANCE_EVERY;++i)
    {
        run_side_watch(&watch,pattern);
        tk_dly_tsk(OBS_PING_PERIOD_MSEC);
        assert(current_angle==-OBS_SIDE_LOOK_DEG);
    }
    assert(front_count==0U && side_count==5U*OBS_GLANCE_EVERY);
    assert(max_gap<OBS_SIDE_MAX_AGE_MSEC);
    puts("PASS with front glances off the servo never leaves the obstacle");
#endif

    /* The sequencer moves the aim while turning out: the worker follows. */
    watch_angle=-35;
    uint32_t before=side_count;
    run_side_watch(&watch,OBS_EVT_SIDE_WATCH);
    assert(current_angle==-35 && side_count==before+1U);
    watch_angle=-OBS_SIDE_LOOK_DEG;
    run_side_watch(&watch,OBS_EVT_SIDE_WATCH);
    assert(current_angle==-OBS_SIDE_LOOK_DEG);
    puts("PASS worker re-aims whenever the requested watch angle changes");

    echo_result=ULTRASONIC_NO_ECHO;
    before=side_count;
    run_side_watch(&watch,OBS_EVT_SIDE_WATCH);
    assert(side_count==before+1U && published_side==-1);
    puts("PASS a missing echo is published once for sequencer debounce");

    before=side_count;
    leave_side_watch(&watch);
    assert(current_angle==0 && !watch.b_aimed && watch.pings==0U);
    assert(side_count==before);
    puts("PASS leaving side watch parks the servo without a false clear sample");
    echo_result=ULTRASONIC_OK;
    monitor_t monitor={{0,0,0},0U,0U,0U,0U};
    front_range=OBS_STOP_MM+1;
    for(unsigned i=0;i<10;i++) run_monitor(&monitor);
    assert(!(worker_flags&OBS_EVT_BLOCKED));
    front_range=OBS_STOP_MM-1;
    for(unsigned i=0;i<10;i++) run_monitor(&monitor);
    assert(worker_flags&OBS_EVT_BLOCKED);
    assert(OBS_STOP_MM==250);
    puts("PASS front monitor triggers below 250 mm, 100 mm closer than before");
    worker_flags=OBS_EVT_SCAN_REQ;
    uint32_t const scan_started=clock_ms;
    uint32_t const pings_before=pings;
    min_trigger_gap=UINT32_MAX;
    run_scan();
    assert(plan_count==1U && scan_plan_result(&stored_plan)==SCAN_PLAN_ROUTE);
    assert(min_angle==-70 && max_angle==70 && current_angle==0);
    assert((worker_flags&OBS_EVT_SCAN_DONE) &&
           !(worker_flags&OBS_EVT_SCAN_REQ));
    assert(clock_ms-scan_started<OBS_SCAN_TIMEOUT_MSEC);
    puts("PASS real scan worker covers both servo extremes within its timeout");
    {
        /* Faster scan, same measurement: every ping still at least
         * OBS_PING_PERIOD_MSEC after the previous trigger, same pings per
         * bearing, and the turnaround bearing is not pinged twice. */
        /* Return pass (two-way) re-reads only bearings within
         * +-OBS_SCAN_RETURN_DEG, and never the end bearing twice. */
        uint32_t returning=0U;
        if (0==OBS_SCAN_DIRECTION)
            for (int32_t a=-OBS_SCAN_RIGHT_DEG;a<=OBS_SCAN_LEFT_DEG;
                 a=scan_plan_next_angle(a))
                if (a>=-OBS_SCAN_RETURN_DEG && a<=OBS_SCAN_RETURN_DEG &&
                    a!=-OBS_SCAN_RIGHT_DEG) ++returning;
        uint32_t const bearings=(uint32_t)OBS_SCAN_POINTS+returning;
        uint32_t const took=clock_ms-scan_started;
        assert(min_trigger_gap>=OBS_PING_PERIOD_MSEC);
        assert(pings-pings_before==bearings*OBS_SCAN_PINGS);
        /* Previous timing, as a lower bound: the smallest servo settle at
         * every bearing, then each ping's echo plus a full period after it
         * (and it pinged the turnaround bearing twice, not counted here). */
        uint32_t const old_per_ping=OBS_ECHO_TIMEOUT_MSEC+OBS_PING_PERIOD_MSEC;
        uint32_t const old_settle=OBS_SERVO_SETTLE_BASE_MSEC+
            OBS_SCAN_FINE_STEP_DEG*OBS_SERVO_SETTLE_PER_DEG_MSEC;
        /* Old: both full passes (41 bearings) and 3 pings each. */
        uint32_t const old_bearings=2U*(uint32_t)OBS_SCAN_POINTS;
        uint32_t const old_estimate=old_bearings*old_settle+
            old_bearings*3U*old_per_ping;
        assert(took*100U < old_estimate*80U); /* at least 20% faster */
        /* Target: a scan in about 5 s even with every echo timing out
         * (the worst case for ping time). */
        assert(took <= 5500U);
        printf("PASS scan %u bearings, %u pings in %u ms (pings >= %u ms "
               "apart; old timing at least %u ms)\n",(unsigned)bearings,
               (unsigned)(pings-pings_before),(unsigned)took,
               (unsigned)min_trigger_gap,(unsigned)old_estimate);
    }

    /* An edge that echoes every other ping: the third ping and median keep
     * it (keeping the larger of two would have dropped it). Agreeing
     * bearings still take two pings. */
    {
        flaky_angle=20;flaky_n=0U;
        uint32_t const before=pings;
        worker_flags=OBS_EVT_SCAN_REQ;
        run_scan();
        /* Each visit to +20 (once per pass) costs one extra ping. */
        uint32_t const visits=(0==OBS_SCAN_DIRECTION &&
                               20<=OBS_SCAN_RETURN_DEG)?2U:1U;
        uint32_t const base=(uint32_t)(pings-before)-visits;
        assert(base%OBS_SCAN_PINGS==0U);
        /* Directly at the edge: echo, miss, echo -> median 232 mm. */
        (void)servo_set_angle(20);flaky_n=0U;
        uint32_t const p0=pings;
        assert(scan_range(20)==232 && pings-p0==3U);
        /* Miss first: miss, echo, miss -> median says no echo (4000). */
        flaky_n=1U;
        assert(scan_range(20)==OBS_FAR_MM);
        flaky_angle=999;
        /* Agreeing echoes: two pings only, their average. */
        (void)servo_set_angle(10);
        uint32_t const p1=pings;
        assert(scan_range(10)==240 && pings-p1==2U);
        puts("PASS an intermittent edge echo is kept: third ping, median");
    }

#if (0 == OBS_SCAN_DIRECTION)
    /* Both ways: the sweep runs left to right first, then back. */
    assert(left_moves>0U && right_moves>1U);
    {
        /* Left first: the first ping of a scan is at the left end. */
        (void)servo_set_angle(0);
        worker_flags=OBS_EVT_SCAN_REQ;
        scan_order_armed=true;
        run_scan();
        assert(scan_order_first==OBS_SCAN_LEFT_DEG);
        puts("PASS two-way scan starts at the left end, then sweeps right");
    }
    {
        /* Corner band: a weak corner echo that only every other ping
         * catches is kept (two agreeing echoes); empty bearings there take
         * OBS_CORNER_PINGS pings; outside it still two. */
        flaky_angle=40;flaky_n=1U; /* miss, echo, miss, echo */
        (void)servo_set_angle(40);
        uint32_t p0=pings;
        assert(scan_range(40)==232 && pings-p0==4U);
        flaky_n=0U; /* echo, miss, echo: stops once two agree */
        p0=pings;
        assert(scan_range(40)==232 && pings-p0==3U);
        flaky_angle=999;
        echo_result=ULTRASONIC_NO_ECHO;
        (void)servo_set_angle(-50);
        p0=pings;
        assert(scan_range(-50)==OBS_FAR_MM && pings-p0==OBS_CORNER_PINGS);
        (void)servo_set_angle(-60);
        p0=pings;
        assert(scan_range(-60)==OBS_FAR_MM && pings-p0==2U);
        echo_result=ULTRASONIC_OK;
        puts("PASS corner band pings until two agree; a lone echo is kept out");
    }
    /* A centred object and a servo that lags 5 degrees: one-way it would
     * lean (edges -5..15 or -15..5); both ways keep -15..15, so the sides
     * stay equal whichever way the servo lags. */
    bool b_first_left=false;
    for(int32_t sign=-1;sign<=1;sign+=2)
    {
        lag_deg=5*sign;
        worker_flags=OBS_EVT_SCAN_REQ;
        run_scan();
        assert(scan_plan_result(&stored_plan)==SCAN_PLAN_ROUTE);
        /* The same side whichever way the servo lags. */
        if(sign<0) b_first_left=scan_plan_side_is_left(&stored_plan);
        else assert(scan_plan_side_is_left(&stored_plan)==b_first_left);
    }
    lag_deg=0;
    puts("PASS two-way scan cancels a servo lag in either direction");
#endif
    return 0;
}
