/** @file obstacle_watch.c
 * @brief Run the real ultrasonic worker's side/front watch with fake
 *        hardware.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <tk/tkernel.h>
typedef UINT RELTIM; /* Kernel time type needed by the embedded worker. */
#include "modules/obstacle/task_obstacle.c"
#include "modules/obstacle/scan_plan.c"

static uint32_t clock_ms;
static uint32_t side_count;
static uint32_t front_count;
static uint32_t side_time;
static uint32_t max_gap;
static int32_t current_angle;
static int32_t published_side;
static int32_t watch_angle=-OBS_SIDE_LOOK_DEG;
static ultrasonic_result_t echo_result=ULTRASONIC_OK;
static int32_t front_range=1000;
static int32_t min_angle;
static int32_t max_angle;
static UINT worker_flags;
static scan_plan_bypass_t stored_plan;
static unsigned int plan_count;

/** @brief Host stub of tm_printf(). */
INT tm_printf(UB const *format, ...) {(void)format;return 0;}
/** @brief Host stub of tk_dly_tsk(). */
ER tk_dly_tsk(UINT delay) {clock_ms+=delay;return E_OK;}
/** @brief Host stub of tk_get_otm(). */
ER tk_get_otm(SYSTIM *now) {now->hi=0;now->lo=clock_ms;return E_OK;}
/* Trigger times, to check scan ping spacing. */
static uint32_t last_trigger;
static uint32_t min_trigger_gap=UINT32_MAX;
static uint32_t pings;
/* Lone corner echo out of scan_range(). */
static int32_t lone;
static bool have_trigger;
/* A bearing whose echo comes and goes (logged at an object edge). */
static int32_t flaky_angle=999;
static unsigned int flaky_n;
/* A bearing that echoes on its first ping only. */
static int32_t lone_echo_angle=999;
static unsigned int lone_echo_n;
/** @brief Host stub of ultrasonic_last_trigger(). */
bool ultrasonic_last_trigger(uint32_t *msec)
{
    if (!have_trigger)
    {
        return false;
    }
    *msec = last_trigger;
    return true;
}
/* Simulated servo lag: it stops lag_deg short of each target, so the
 * real bearing depends on the direction it arrived from. */
static int32_t lag_deg;
static int32_t move_dir;
static int32_t lag_lo=-10;
static int32_t lag_hi=10;
/* Pings taken right after a move to the left (off the left limit). */
static uint32_t left_approach_pings;
static unsigned int left_moves;
static unsigned int right_moves;
/* First bearing a scan pings (set by the first ultrasonic_ping after
 * scan_order_armed). */
static int32_t scan_order_first=999;
static int32_t scan_order_first_sweep=999;
static bool scan_order_armed;
/** @brief Host stub of servo_set_angle(). */
uint32_t servo_set_angle(int32_t angle)
{
    unsigned movement=(unsigned)abs(angle-current_angle);
    if(angle>current_angle) {move_dir=1;++left_moves;}
    if(angle<current_angle) {move_dir=-1;++right_moves;}
    current_angle=angle;
    if(angle<min_angle)
    {
        min_angle=angle;
    }
    if(angle>max_angle)
    {
        max_angle=angle;
    }
    return OBS_SERVO_SETTLE_BASE_MSEC + movement*OBS_SERVO_SETTLE_PER_DEG_MSEC;
}
/** @brief Host stub of servo_angle_get(). */
int32_t servo_angle_get(void) {return current_angle;}
/** @brief Host stub of obs_state_watch_angle_get(). */
int32_t obs_state_watch_angle_get(void) {return watch_angle;}
/** @brief Host stub of ultrasonic_ping(). */
ultrasonic_result_t ultrasonic_ping(int32_t *range)
{
    if(have_trigger && clock_ms-last_trigger<min_trigger_gap)
    {
        min_trigger_gap=clock_ms-last_trigger;
    }
    last_trigger=clock_ms;have_trigger=true;++pings;
    if(move_dir==1 && current_angle<OBS_SERVO_LEFT_LIMIT_DEG)
    {
        ++left_approach_pings;
    }
    if(scan_order_armed)
    {
        if(scan_order_first==999)
        {
            scan_order_first=current_angle;
        }
        if(current_angle!=0)
        {
            scan_order_first_sweep=current_angle;
            scan_order_armed=false;
        }
    }
    clock_ms+=OBS_ECHO_TIMEOUT_MSEC;
    if(current_angle==lone_echo_angle)
    {
        /* One echo, then nothing (a weak corner heard once). */
        if(lone_echo_n++)
        {
            return ULTRASONIC_OUT_OF_RANGE;
        }
        *range=232;
        return ULTRASONIC_OK;
    }
    if(current_angle==flaky_angle)
    {
        /* Echo, miss, echo, ... like 18 Nov's +20 degree edge. */
        if((flaky_n++)%2U)
        {
            return ULTRASONIC_OUT_OF_RANGE;
        }
        *range=232;
        return ULTRASONIC_OK;
    }
    if(lag_deg!=0)
    {
        int32_t const actual=current_angle-move_dir*lag_deg;
        if(actual<lag_lo || actual>lag_hi)
        {
            return ULTRASONIC_OUT_OF_RANGE;
        }
        *range=240;
        return ULTRASONIC_OK;
    }
    *range=current_angle==0?front_range:240;
    return echo_result;
}
/** @brief Host stub of obs_state_flags_read(). */
UINT obs_state_flags_read(void) {return worker_flags;}
/** @brief Host stub of obs_state_flags_set(). */
void obs_state_flags_set(UINT bits) {worker_flags|=bits;}
/** @brief Host stub of obs_state_flags_clear(). */
void obs_state_flags_clear(UINT bits) {worker_flags&=~bits;}
/** @brief Host stub of obs_state_set_phase(). */
void obs_state_set_phase(obs_state_phase_t phase) {(void)phase;}
/** @brief Host stub of obs_state_set_distance_mm(). */
void obs_state_set_distance_mm(int32_t mm) {(void)mm;}
/** @brief Host stub of obs_state_faults_update(). */
void obs_state_faults_update(uint32_t set,uint32_t clear)
{(void)set;(void)clear;}
/** @brief Host stub of obs_state_status_get(). */
uint32_t obs_state_status_get(void) {return 0U;}
/** @brief Host stub of obs_state_status_distance_cm(). */
uint32_t obs_state_status_distance_cm(uint32_t word)
{(void)word;return OBS_STOP_MM/10U;}
/** @brief Host stub of obs_state_status_route(). */
obs_state_route_t obs_state_status_route(uint32_t word)
{(void)word;return OBS_ROUTE_NONE;}
/** @brief Host stub of obs_state_plan_store(). */
void obs_state_plan_store(scan_plan_bypass_t const *plan)
{stored_plan=*plan;++plan_count;}
/** @brief Host stub of servo_left_limit_deg(). */
int32_t servo_left_limit_deg(void) {return OBS_SERVO_LEFT_LIMIT_DEG;}
/** @brief Host stub of servo_right_limit_deg(). */
int32_t servo_right_limit_deg(void) {return OBS_SERVO_RIGHT_LIMIT_DEG;}
/** @brief Host stub of ultrasonic_median_of_three(). */
int32_t ultrasonic_median_of_three(int32_t a,int32_t b,int32_t c)
{
    if(a>b) {int32_t t=a;a=b;b=t;}
    if(b>c) {int32_t t=b;b=c;c=t;}
    return a>b?a:b;
}
/** @brief Host stub of obs_state_side_range_set(). */
void obs_state_side_range_set(int32_t range)
{
    assert(current_angle==watch_angle);
    if(side_count && clock_ms-side_time>max_gap)
    {
        max_gap=clock_ms-side_time;
    }
    side_time=clock_ms;published_side=range;++side_count;
}
/** @brief Host stub of obs_state_front_range_set(). */
void obs_state_front_range_set(int32_t range)
{
    assert(current_angle==0 && range==1000);
    ++front_count;
}

/** @brief Run every test case; any failed assert aborts. */
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
    puts(
        "PASS worker keeps the ultrasonic aimed at the obstacle while passing");

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
    puts(
        "PASS leaving side watch parks the servo without a false clear sample");
    echo_result=ULTRASONIC_OK;
    monitor_t monitor={{0,0,0},0U,0U,0U,0U};
    front_range=OBS_STOP_MM+1;
    for(unsigned i=0;i<10;i++)
    {
        run_monitor(&monitor);
    }
    assert(!(worker_flags&OBS_EVT_BLOCKED));
    front_range=OBS_STOP_MM-1;
    for(unsigned i=0;i<10;i++)
    {
        run_monitor(&monitor);
    }
    assert(worker_flags&OBS_EVT_BLOCKED);
    assert(OBS_STOP_MM==250);
    puts("PASS front monitor triggers below 250 mm, 100 mm closer than before");
    worker_flags=OBS_EVT_SCAN_REQ;
    uint32_t const scan_started=clock_ms;
    uint32_t const pings_before=pings;
    min_trigger_gap=UINT32_MAX;
    run_scan();
    assert(plan_count==1U && scan_plan_result(&stored_plan)==SCAN_PLAN_ROUTE);
    assert(min_angle==-OBS_SCAN_RIGHT_DEG && max_angle==OBS_SCAN_LEFT_DEG &&
           current_angle==0);
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
        {
            for (int32_t a=-OBS_SCAN_RIGHT_DEG;a<=OBS_SCAN_LEFT_DEG;
                 a=scan_plan_next_angle(a))
            {
                if (a>=-OBS_SCAN_RETURN_DEG && a<=OBS_SCAN_RETURN_DEG &&
                    a!=-OBS_SCAN_RIGHT_DEG)
                {
                    ++returning;
                }
            }
        }
        uint32_t const anchor=OBS_FRESH_SCAN_CENTRE!=0?1U:0U;
        uint32_t const bearings=(uint32_t)OBS_SCAN_POINTS+returning+anchor;
        /* Slow edge vote (object ahead): OBS_EDGE_VOTE_PINGS more pings at
         * each scan bearing OBS_EDGE_VOTE_MIN..MAX_DEG either side. Its
         * guaranteed waits are taken off before the sweep speed checks. */
        uint32_t voted=0U;
        if (0!=OBS_EDGE_VOTE)
        {
            for (int32_t a=-OBS_SCAN_RIGHT_DEG;a<=OBS_SCAN_LEFT_DEG;
                 a=scan_plan_next_angle(a))
            {
                if (abs(a)>=OBS_EDGE_VOTE_MIN_DEG &&
                    abs(a)<=OBS_EDGE_VOTE_MAX_DEG)
                {
                    ++voted;
                }
            }
        }
        /* At least: out along the left band, across to the right band's
         * start and out along it (servo travel), then per voted bearing a
         * settle, the extra settle, each ping's echo wait and the gaps. */
        uint32_t const vote_travel=(0U==voted)?0U:(uint32_t)(
            2*(OBS_EDGE_VOTE_MAX_DEG-OBS_EDGE_VOTE_MIN_DEG)+
            (OBS_EDGE_VOTE_MAX_DEG+OBS_EDGE_VOTE_MIN_DEG));
        uint32_t const vote_wait=vote_travel*OBS_SERVO_SETTLE_PER_DEG_MSEC+
            voted*(OBS_SERVO_SETTLE_BASE_MSEC+OBS_EDGE_VOTE_SETTLE_MSEC+
            OBS_EDGE_VOTE_PINGS*OBS_ECHO_TIMEOUT_MSEC+
            (OBS_EDGE_VOTE_PINGS-1U)*OBS_EDGE_VOTE_GAP_MSEC);
        uint32_t const took_all=clock_ms-scan_started;
        assert(took_all>=vote_wait);
        uint32_t const took=took_all-vote_wait;
        assert(min_trigger_gap>=OBS_PING_PERIOD_MSEC);
        assert(pings-pings_before==bearings*OBS_SCAN_PINGS+
               voted*OBS_EDGE_VOTE_PINGS);
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
        /* Target: about 6 s with every bearing echoing (5.5 s before the
         * corner band went to 5 degree steps for off-centre boxes). */
        uint32_t const anchor_delay=anchor*(OBS_SERVO_SETTLE_BASE_MSEC+
            OBS_SCAN_PINGS*OBS_PING_PERIOD_MSEC);
        /* +1.8 s (21 Nov): every bearing is now reached the same way
         * (always arriving moving right, ramped big moves),
         * measured 7.85 s here. */
        assert(took <= 8000U+anchor_delay);
        printf("PASS scan %u bearings + %u voted, %u pings in %u ms "
               "(sweep %u ms; pings >= %u ms apart; old timing at least %u "
               "ms)\n",(unsigned)bearings,(unsigned)voted,
               (unsigned)(pings-pings_before),(unsigned)took_all,
               (unsigned)took,
               (unsigned)min_trigger_gap,(unsigned)old_estimate);
    }

    /* Worst case: nothing echoes, so each corner band bearing takes
     * OBS_CORNER_PINGS pings. Still well inside the scan timeout. */
    {
        uint32_t const t0=clock_ms;
        echo_result=ULTRASONIC_NO_ECHO;
        worker_flags=OBS_EVT_SCAN_REQ;
        uint32_t const p0=pings;
        run_scan();
        echo_result=ULTRASONIC_OK;
        uint32_t const took=clock_ms-t0;
        /* Nothing in front after the one sweep: clear at once, parked at
         * centre, with no edge vote (no more pings than the sweep's). */
        assert(scan_plan_result(&stored_plan)==SCAN_PLAN_CLEAR);
        assert((worker_flags&OBS_EVT_SCAN_DONE) &&
               !(worker_flags&OBS_EVT_SCAN_REQ) && current_angle==0);
        assert(pings-p0<=(uint32_t)OBS_SCAN_POINTS*OBS_CORNER_PINGS+
               OBS_CORNER_PINGS);
        puts("PASS nothing in front after the sweep: clear at once, no "
             "edge vote");
        printf("PASS silent scan (corner band pinged %u times per bearing) "
               "in %u ms\n",(unsigned)OBS_CORNER_PINGS,(unsigned)took);
        uint32_t const anchor_delay=(OBS_FRESH_SCAN_CENTRE!=0)
            ? OBS_SERVO_SETTLE_BASE_MSEC+OBS_SCAN_PINGS*OBS_PING_PERIOD_MSEC
            : 0U;
        /* Same-way servo approach adds ~0.8 s (measured 8.27 s). */
        assert(took <= 8500U+anchor_delay && took < OBS_SCAN_TIMEOUT_MSEC);
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
        assert(scan_range(20,&lone)==232 && pings-p0==3U);
        /* Miss first: miss, echo, miss -> median says no echo (4000). */
        flaky_n=1U;
        assert(scan_range(20,&lone)==OBS_FAR_MM);
        flaky_angle=999;
        /* Agreeing echoes: two pings only, their average. */
        (void)servo_set_angle(10);
        uint32_t const p1=pings;
        assert(scan_range(10,&lone)==240 && pings-p1==2U);
        puts("PASS an intermittent edge echo is kept: third ping, median");
    }

#if (0 == OBS_SCAN_DIRECTION)
    /* Both ways: the sweep runs left to right first, then back. */
    assert(left_moves>0U && right_moves>1U);
    {
        /* Servo zero is measured before the left-to-right sweep. */
        (void)servo_set_angle(0);
        worker_flags=OBS_EVT_SCAN_REQ;
        scan_order_armed=true;
        run_scan();
        assert(scan_order_first==(OBS_FRESH_SCAN_CENTRE!=0
                                      ?0:OBS_SCAN_LEFT_DEG));
        assert(scan_order_first_sweep==OBS_SCAN_LEFT_DEG);
        puts("PASS the fresh zero precedes the left-to-right sweep");
    }
    {
        /* Corner band: a weak corner echo that only every other ping
         * catches is kept (two agreeing echoes); empty bearings there take
         * OBS_CORNER_PINGS pings; outside it still two. */
        flaky_angle=40;flaky_n=1U; /* miss, echo, miss, echo */
        (void)servo_set_angle(40);
        uint32_t p0=pings;
        assert(scan_range(40,&lone)==232 && pings-p0==4U);
        flaky_n=0U; /* echo, miss, echo: stops once two agree */
        p0=pings;
        assert(scan_range(40,&lone)==232 && pings-p0==3U);
        /* Echo once, then misses: the reading stays far, the echo is
         * kept apart as lone for the planner's neighbour check. */
        flaky_n=0U;
        lone_echo_angle=40; lone_echo_n=0U;
        p0=pings;
        assert(scan_range(40,&lone)==OBS_FAR_MM && lone==232 &&
               pings-p0==OBS_CORNER_PINGS);
        lone_echo_angle=999;
        flaky_angle=999;
        echo_result=ULTRASONIC_NO_ECHO;
        (void)servo_set_angle(-50);
        p0=pings;
        assert(scan_range(-50,&lone)==OBS_FAR_MM && pings-p0==OBS_CORNER_PINGS);
        (void)servo_set_angle(-60);
        p0=pings;
        assert(scan_range(-60,&lone)==OBS_FAR_MM && pings-p0==2U);
        echo_result=ULTRASONIC_OK;
        puts("PASS corner band pings until two agree; a lone echo is kept out");
    }
#endif
    /* A servo that stops 5 degrees short of its target, either way. Every
     * scan bearing is reached moving right (a move left first swings to
     * the left limit), so the lag is one constant offset for the whole
     * scan, which the FRONT trim absorbs, not a direction-dependent error
     * that tilts the edges (logged 21 Nov: after looking left the servo
     * came back to centre 5-10 degrees off). Same plan on a repeat scan. */
    for(int32_t sign=-1;sign<=1;sign+=2)
    {
        lag_deg=5*sign;
        left_approach_pings=0U;
        worker_flags=OBS_EVT_SCAN_REQ;
        run_scan();
        assert(scan_plan_result(&stored_plan)==SCAN_PLAN_ROUTE);
        assert(left_approach_pings==0U);
        bool const b_left=scan_plan_side_is_left(&stored_plan);
        uint32_t const lateral=(uint32_t)scan_plan_lateral_cm(&stored_plan);
        worker_flags=OBS_EVT_SCAN_REQ;
        run_scan();
        assert(scan_plan_side_is_left(&stored_plan)==b_left &&
               (uint32_t)scan_plan_lateral_cm(&stored_plan)==lateral);
    }
    lag_deg=0;
    puts("PASS every scan bearing is reached moving right; a servo lag is "
         "one constant offset and repeat scans agree");

    return 0;
}
/*** end of file ***/

