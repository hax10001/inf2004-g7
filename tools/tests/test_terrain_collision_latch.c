/* Verify the real mutex-owned IMU event mailbox, including rearming.
 * No real I2C, motion commands, PWM or unsafe cross-core bit writes.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifndef APP_COLLISION_ENABLED
#define APP_COLLISION_ENABLED 1 /* Also compiled with 0 to verify disabled mode. */
#endif
#include "modules/terrain/terrain.h"
#include "modules/terrain/task_imu.c"
#include "modules/terrain/tilt_math.c"

static uint32_t sample_time;
static int16_t sample_x;
static unsigned int impact_logs;
ER tk_get_otm(SYSTIM *out)
{*out=(SYSTIM){.lo=sample_time};return E_OK;}
bool lsm303_read_accel(ID device,int16_t *x,int16_t *y,int16_t *z)
{(void)device;*x=sample_x;*y=0;*z=1000;return true;}
bool lsm303_read_mag(ID device,int16_t *x,int16_t *y,int16_t *z)
{(void)device;(void)x;(void)y;(void)z;return false;}
INT tm_printf(UB const *format,...)
{if(strstr((char const *)format,"Collision detected"))++impact_logs;return 0;}

static void imu_sample(imu_history_t *history,int16_t x)
{sample_x=x;sample_time+=20U;process_sample(1,history);}

static unsigned int locks=0U;
ID tk_cre_mtx(T_CMTX const *config)
{
    assert(config != NULL);
    return (ID)42;
}
ER tk_del_mtx(ID id) {(void)id;return E_OK;}
ER tk_loc_mtx(ID id, TMO timeout)
{
    (void)timeout;
    assert(id==42);
    ++locks;
    return E_OK;
}
ER tk_unl_mtx(ID id)
{
    assert(id==42);
    assert(locks>0U);
    --locks;
    return E_OK;
}

int main(void)
{
    terrain_snapshot s={0};
    terrain_snapshot out={0};
    assert(terrain_init()==E_OK);
    s.b_valid=1U;
    s.b_collision=1U;
    assert(!terrain_collision_enabled());
    /* Picking up the stopped car cannot create a fault or increment count. */
    for (unsigned int i=0U; i<20U; ++i)
        assert(terrain_publish_snapshot(&s)==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK);
    assert(!out.b_collision && out.collision_sequence==0U);
    assert(terrain_set_collision_enabled(true)==E_OK);
    assert(terrain_collision_enabled());
    assert(terrain_publish_snapshot(&s)==E_OK);
    s.b_collision=0U;
    assert(terrain_publish_snapshot(&s)==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK);
    assert(out.b_collision==1U); /* Event remains pending until STOP. */
    assert(terrain_set_collision_enabled(false)==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK && out.b_collision);
    assert(terrain_acknowledge_collision()==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK);
    assert(out.b_collision==0U);
    assert(terrain_publish_snapshot(&s)==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK);
    assert(out.b_collision==0U); /* Old event never repeats. */
    s.b_collision=1U;
    assert(terrain_publish_snapshot(&s)==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK && !out.b_collision);
    assert(out.collision_sequence==1U); /* STOP-time impact not counted. */
    assert(terrain_set_collision_enabled(true)==E_OK);
    assert(terrain_publish_snapshot(&s)==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK);
    assert(out.b_collision==1U); /* New independent collision still works. */
    uint32_t observed=out.collision_sequence;
    assert(terrain_publish_snapshot(&s)==E_OK); /* New event before ACK. */
    assert(terrain_ack_collision(observed)==E_OBJ);
    assert(terrain_get_snapshot(&out)==E_OK && out.b_collision);
    assert(out.collision_sequence!=observed);
    assert(terrain_ack_collision(out.collision_sequence)==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK && !out.b_collision);
    /* A producer sampled before STOP but publishing after it is gated too. */
    assert(terrain_set_collision_enabled(false)==E_OK);
    assert(terrain_publish_snapshot(&s)==E_OK);
    assert(terrain_get_snapshot(&out)==E_OK && !out.b_collision);
    assert(out.collision_sequence==3U);
    imu_history_t history={0};
    for (unsigned int i=0U; i<20U; ++i)
        imu_sample(&history,(i&1U)?1000:-1000);
    assert(terrain_get_snapshot(&out)==E_OK && !out.b_collision);
    assert(out.collision_sequence==3U && impact_logs==0U);
    assert(terrain_set_collision_enabled(true)==E_OK);
    imu_sample(&history,-1000); /* Fresh baseline ignores the STOP-time jerk. */
    assert(terrain_get_snapshot(&out)==E_OK && !out.b_collision);
    imu_sample(&history,1000); /* A new run-time horizontal impact still works. */
#if APP_COLLISION_ENABLED
    assert(terrain_get_snapshot(&out)==E_OK && out.b_collision);
    assert(out.collision_sequence==4U && impact_logs==1U);
#else
    assert(terrain_get_snapshot(&out)==E_OK && !out.b_collision);
    assert(out.collision_sequence==3U && impact_logs==0U);
#endif
    assert(terrain_set_collision_enabled(false)==E_OK);
    assert(terrain_ack_collision(out.collision_sequence)==E_OK);
    imu_sample(&history,-1000);
    assert(terrain_get_snapshot(&out)==E_OK && !out.b_collision);
    assert(out.collision_sequence==(APP_COLLISION_ENABLED?4U:3U));
    assert(impact_logs==(APP_COLLISION_ENABLED?1U:0U));
    assert(locks==0U);
    puts("PASS terrain collision: stopped handling ignored, run impacts latch, "
         "STOP acknowledgement, fresh rearm and late producer gated");
    return 0;
}
