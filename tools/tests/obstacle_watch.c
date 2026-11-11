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
uint32_t servo_set_angle(int32_t angle)
{
    unsigned movement=(unsigned)abs(angle-current_angle);
    current_angle=angle;
    if(angle<min_angle) min_angle=angle;
    if(angle>max_angle) max_angle=angle;
    return OBS_SERVO_SETTLE_BASE_MSEC + movement*OBS_SERVO_SETTLE_PER_DEG_MSEC;
}
int32_t obs_state_watch_angle_get(void) {return watch_angle;}
ultrasonic_result_t ultrasonic_ping(int32_t *range)
{
    clock_ms+=OBS_ECHO_TIMEOUT_MSEC;
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
    run_scan();
    assert(plan_count==1U && scan_plan_result(&stored_plan)==SCAN_PLAN_ROUTE);
    assert(min_angle==-70 && max_angle==70 && current_angle==0);
    assert((worker_flags&OBS_EVT_SCAN_DONE) &&
           !(worker_flags&OBS_EVT_SCAN_REQ));
    assert(clock_ms-scan_started<OBS_SCAN_TIMEOUT_MSEC);
    puts("PASS real scan worker covers both servo extremes within its timeout");
    return 0;
}
