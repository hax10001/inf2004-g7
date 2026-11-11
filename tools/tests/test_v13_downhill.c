/* V13 pure governor test: signed-pitch, slope and safety boundaries. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "app/app_config.h"
#include "modules/motion/downhill.h"


#if APP_DOWNHILL_NOSE_DOWN_SIGN != 0
static float difference(float a,float b)
{
    float d=a-b;
    return d<0.0f?-d:d;
}

#endif

static void slope(downhill_governor_t *s,float pitch,int n)
{
    for(int i=0;i<n;i++)downhill_sample(s,pitch,0.0f,true,true);
}

int main(void)
{
    downhill_governor_t s={0};
    downhill_reset(&s);
    assert(s.scale_percent==100U && !s.fault);
#if APP_DOWNHILL_NOSE_DOWN_SIGN == 0
    slope(&s,8.0f,10);
    assert(!s.engaged && s.scale_percent==100U);
    for(int i=0;i<8;i++)downhill_sample(&s,0,0,false,true);
    assert(!s.fault); /* Sign unverified -> feature explicitly disabled. */
    puts("V13: unverified IMU sign leaves governor disabled PASS");
#else
    float l=0.0f,r=0.0f;
    float const down=8.0f*(float)APP_DOWNHILL_NOSE_DOWN_SIGN;
    float const up=-8.0f*(float)APP_DOWNHILL_NOSE_DOWN_SIGN;
    slope(&s,up,8);
    assert(!s.engaged && s.scale_percent==100U); /* uphill */
    downhill_reset(&s);
    for(int i=0;i<8;i++)
        downhill_sample(&s,down,15.0f,true,true);
    assert(!s.engaged); /* one-wheel hump: high roll */
    downhill_reset(&s);
    slope(&s,down,2);
    assert(!s.engaged && s.scale_percent==100U);
    slope(&s,down,1);
    assert(s.engaged && s.scale_percent==95U);
    slope(&s,down,6); /* 5 points/cycle, no sudden STOP/ON. */
    assert(s.engaged && s.scale_percent==65U);
    downhill_scale_pair(&s,10.0f,10.0f,&l,&r);
    assert(difference(l,6.5f)<.001f && difference(r,6.5f)<.001f);
    downhill_scale_pair(&s,6.0f,12.0f,&l,&r);
    assert(difference(l,3.9f)<.001f && difference(r,7.8f)<.001f);
    assert(l<r && difference(r/l,2.0f)<.001f); /* left turn */
    downhill_scale_pair(&s,12.0f,6.0f,&l,&r);
    assert(l>r && difference(l/r,2.0f)<.001f); /* right turn */
    downhill_sample(&s,down,10.0f,true,true);
    assert(s.engaged); /* already downhill: a one-wheel hump cannot release */
    downhill_sample(&s,up,0,true,true);
    assert(s.engaged); /* one bad reading cannot release */
    slope(&s,up,24);
    assert(!s.engaged && s.scale_percent==100U); /* gradual recovery */
    downhill_reset(&s);
    slope(&s,down,3);
    assert(s.engaged);
    for(int i=0;i<3;i++)
        downhill_sample(&s,NAN,0.0f,true,true);
    assert(!s.fault && s.scale_percent==95U);
    for(int i=0;i<16;i++)
        downhill_sample(&s,down,0.0f,false,true);
    assert(!s.fault); /* 19 missing periods: hold reduced speed. */
    downhill_sample(&s,down,0.0f,false,true);
    assert(s.fault); /* 20 missing periods: fail closed. */
    slope(&s,up,20);
    assert(s.fault); /* cannot auto-clear */
    downhill_scale_pair(&s,10.0f,20.0f,&l,&r);
    assert(l==0.0f && r==0.0f);
    downhill_reset(&s);
    assert(!s.fault && s.scale_percent==100U);
    slope(&s,down,3);
    assert(!downhill_wheel_over_target(&s,2.0f,40.0f));
    assert(!downhill_wheel_over_target(&s,9.75f,14.0f));
    for(int i=0;i<12;i++)
        assert(!downhill_observe_speed(&s,9.75f,9.75f,20,0));
    assert(downhill_wheel_over_target(&s,9.75f,45.0f));
    for(int i=0;i<9;i++)
        assert(!downhill_observe_speed(&s,9.75f,9.75f,45,0));
    assert(downhill_observe_speed(&s,9.75f,9.75f,45,0));
    assert(s.fault);
    downhill_reset(&s);
    slope(&s,down,3);
    for(int i=0;i<7;i++)
        assert(!downhill_observe_speed(&s,2.0f,9.75f,40,0));
    assert(!s.fault); /* dragged stopped inner wheel ignored */
    downhill_reset(&s);
    for(int i=0;i<4;i++)
        downhill_sample(&s,0.0f,0.0f,false,false);
    assert(!s.fault); /* stationary calibration / remote HOLD */
    for(int i=0;i<20;i++)
        downhill_sample(&s,0.0f,0.0f,false,true);
    assert(s.fault); /* prolonged IMU loss, not a short dropout */
    printf("V13: pitch sign %d, slope, turn ratios, overspeed, stale, "
           "hump and reset PASS\n", APP_DOWNHILL_NOSE_DOWN_SIGN);
#endif
    /* Production terrain input is already confirmed by the IMU task; it
     * must reduce RPM even when pitch sign remains unverified. */
    downhill_reset(&s);
    for(int i=0;i<10;i++)
        downhill_sample_terrain(&s,0,0,true,true,false);
    assert(!s.engaged && s.scale_percent==100U); /* no descent */
    downhill_sample_terrain(&s,0,0,true,true,true);
    assert(s.engaged && s.scale_percent==95U);
    for(int i=0;i<6;i++)
        downhill_sample_terrain(&s,0,0,true,true,true);
    assert(s.scale_percent==65U);
    for(int i=0;i<19;i++)
        downhill_sample_terrain(&s,0,0,false,true,true);
    assert(!s.fault && s.scale_percent==65U);
    downhill_sample_terrain(&s,0,0,false,true,true);
    assert(s.fault);
    downhill_reset(&s);
    downhill_sample_terrain(&s,0,0,true,true,true);
    for(int i=0;i<9;i++)
        assert(!downhill_observe_speed(&s,9.75f,9.75f,45,0));
    assert(downhill_observe_speed(&s,9.75f,9.75f,45,0) && s.fault);
    puts("PASS confirmed terrain descent, gradual target, stale/overspeed latches");
    return 0;
}
