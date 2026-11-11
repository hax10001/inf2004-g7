/* Production PI, encoder estimator and PWM driver against a simple plant.
 * Plant parameters are assumptions, never physical qualification evidence. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "modules/motion/motion.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "board/board_config.h"
#include "app/app_config.h"
#include <bsp/libbsp.h>

static uint32_t now_us, pins[32];
static float rpm[2], positions[2];
static encoder_sample_t enc[2];
static FILE *recording;
ER pwm_set_pin(UINT p) {(void)p;return E_OK;}
ER pwm_set_wrap(UINT p,UW v) {(void)p;(void)v;return E_OK;}
ER pwm_set_cc(UINT p,UW v) {pins[p]=v;return E_OK;}
ER pwm_set_enabled(UINT p,BOOL v) {(void)p;(void)v;return E_OK;}
ID tk_cre_mtx(const T_CMTX *c) {(void)c;return 42;}
ER tk_loc_mtx(ID id,TMO t) {(void)id;(void)t;return E_OK;}
ER tk_unl_mtx(ID id) {(void)id;return E_OK;}
INT tm_printf(UB const *f,...) {(void)f;return 0;}
ER encoder_init(void) {return E_OK;}
uint32_t encoder_now_usec(void) {return now_us;}
void encoder_read(encoder_wheel_t w,encoder_sample_t *s) {*s=enc[w];}
uint8_t bc_wave_capture_active(void) {return 0U;}

static float effort(unsigned wheel)
{
    uint32_t a=pins[wheel?10:8], b=pins[wheel?11:9];
    if (a==b && a) return -(float)a*100.0f/MOTOR_PWM_WRAP;
    return (float)(wheel?a:b)*100.0f/MOTOR_PWM_WRAP;
}

static void advance(float gravity)
{
    now_us+=50000U;
    for (unsigned w=0;w<2;w++)
    {
        float u=effort(w);
        /* Forward effort above static deadband, linear drag and gravity.
         * Proportional short brake adds a bounded opposing effort. */
        float equilibrium=gravity + (u>8.0f?(u-8.0f)*3.0f:0.0f) +
                                    (u<0.0f?u*2.0f:0.0f);
        rpm[w]+=0.15f*(equilibrium-rpm[w]);
        if (rpm[w]<0) rpm[w]=0;
        positions[w]+=rpm[w]*1056.0f/1200.0f;
        int32_t count=(int32_t)positions[w];
        if (count!=enc[w].count)
        {
            enc[w].count=count;
            enc[w].edge_usec=now_us-(uint32_t)
                ((positions[w]-count)*60000000.0f/(rpm[w]*1056.0f));
        }
    }
}

static void run(float gravity,float pitch)
{
    pitch *= (float)APP_DOWNHILL_NOSE_DOWN_SIGN;
    motion_status_t status={0};
    motion_reset_run();motion_apply(LINE_UNCERTAIN);
    float error_sum=0, min=1000, max=0;
    unsigned observations=0, brake_samples=0;
    for (unsigned i=0;i<1600;i++)
    {
        advance(gravity);
        terrain_snapshot s={.b_valid=true,.height_valid=true,
            .pitch_deg=pitch,.timestamp_msec=now_us/1000U,
            .onHump=gravity>0,.hump_phase=gravity>0 ?
                TERRAIN_HUMP_DESCENDING:TERRAIN_HUMP_LEVEL};
        motion_observe_terrain(&s,now_us/1000U);
        motion_apply(LINE_FORWARD);
        assert(!motion_downhill_fault());
        assert(motion_get_status(&status)==E_OK);
        if (i%10U==0U)
            fprintf(recording,
                "%.1f,%.1f,%u,%.3f,%.3f,%.3f,%.3f,%u,%u,%.3f,%.3f,"
                "%u,%u,%u\n",gravity,pitch,now_us/1000U,
                status.left_target_rpm,status.right_target_rpm,
                status.left_rpm,status.right_rpm,
                status.left_pwm,status.right_pwm,effort(0),effort(1),
                status.downhill_percent,status.downhill_engaged,
                status.downhill_fault);
        assert(effort(0)>=-25.01f && effort(1)>=-25.01f);
        if (effort(0)<0) brake_samples++;
        if (i>=1400)
        {
            error_sum+=fabsf(rpm[0]-status.left_target_rpm);
            if (rpm[0]<min) min=rpm[0];
            if (rpm[0]>max) max=rpm[0];
            observations++;
        }
    }
    float mean=error_sum/observations;
    printf("PLANT gravity=%.1f pitch=%.1f target=%.2f RPM "
           "MAE=%.3f RPM ripple=%.3f RPM brake_samples=%u\n",
           gravity,pitch,status.left_target_rpm,mean,max-min,brake_samples);
    assert(mean<1.0f && max-min<1.5f);
    for (unsigned side=0;side<2;side++)
    {
        for (unsigned i=0;i<10;i++)
        {
            advance(gravity);
            terrain_snapshot s={.b_valid=true,.height_valid=true,
                .pitch_deg=pitch,.timestamp_msec=now_us/1000U,
                .onHump=gravity>0,.hump_phase=gravity>0 ?
                    TERRAIN_HUMP_DESCENDING:TERRAIN_HUMP_LEVEL};
            motion_observe_terrain(&s,now_us/1000U);
            motion_apply(side?LINE_CORRECT_RIGHT:LINE_CORRECT_LEFT);
        }
        assert(motion_get_status(&status)==E_OK);
        if (side) assert(status.right_target_rpm<status.left_target_rpm);
        else assert(status.left_target_rpm<status.right_target_rpm);
    }
    motion_apply(LINE_UNCERTAIN);
    assert(!pins[8] && !pins[9] && !pins[10] && !pins[11]);
    motor_brake();
    assert(pins[8] && pins[9] && pins[10] && pins[11]);
    motion_stop_now();
    assert(!pins[8] && !pins[9] && !pins[10] && !pins[11]);
    /* Negative effort means matched PWM on both inputs, never reverse. */
    motor_set_regulated(-1000,-1500);
    assert(pins[8]==1000 && pins[9]==1000);
    assert(pins[10]==1500 && pins[11]==1500);
    motor_stop();
}

static void transitions(void)
{
    motion_status_t status={0};
    motion_reset_run();motion_stop_now();
    unsigned int changes=0U;
    float prior_effort=0.0f, prior_target=0.0f;
    for(unsigned i=0;i<600;i++)
    {
        bool const descending=i>=100 && i<350;
        advance(descending?8.0f:0.0f);
        terrain_snapshot s={.b_valid=true,.height_valid=true,
            .onHump=descending,.hump_phase=descending ?
                TERRAIN_HUMP_DESCENDING:TERRAIN_HUMP_LEVEL,
            .timestamp_msec=now_us/1000U};
        motion_observe_terrain(&s,now_us/1000U);
        motion_apply(LINE_FORWARD);
        assert(motion_get_status(&status)==E_OK && !motion_downhill_fault());
        if(i>=100)
        {
            float const change=fabsf(effort(0)-prior_effort);
            if(change>0.01f) changes++;
            assert(change<=2.02f);
            assert(fabsf(status.left_target_rpm-prior_target)<=0.751f);
            assert(status.left_target_rpm>0.0f);
            assert(rpm[0]>1.0f); /* No repeated stop/start at slope edges. */
        }
        assert(pins[8]<=MOTOR_PWM_WRAP && pins[9]<=MOTOR_PWM_WRAP);
        prior_effort=effort(0);prior_target=status.left_target_rpm;
        if(i==200) assert(status.downhill_engaged &&
                          status.downhill_percent==65U);
    }
    assert(changes>10U && status.downhill_percent==100U);
    /* A stopped inner wheel must stay physically zero even with a learned
     * positive integral from the descent controller. */
    for(unsigned i=0;i<7;i++)
    {
        advance(0.0f);
        terrain_snapshot s={.b_valid=true,.height_valid=true,.onHump=true,
            .hump_phase=TERRAIN_HUMP_DESCENDING,
            .timestamp_msec=now_us/1000U};
        motion_observe_terrain(&s,now_us/1000U);
        motion_apply(LINE_FORWARD);
    }
    assert(motion_get_status(&status)==E_OK && status.downhill_engaged &&
           status.downhill_percent==65U);
    motion_drive(0.0f,15.0f);
    assert(!pins[8] && !pins[9]);
    motion_stop_now();
    assert(!pins[8] && !pins[9] && !pins[10] && !pins[11]);
    puts("PASS production terrain descent: bounded RPM/duty transitions, "
         "continuous travel, stopped inner wheel and immediate STOP");
}

int main(void)
{
    char path[80];
    snprintf(path,sizeof(path),"tmp/v15/plant-sign-%d.csv",
             APP_DOWNHILL_NOSE_DOWN_SIGN);
    recording=fopen(path,"w");assert(recording);
    fputs("gravity_rpm,pitch_deg,time_ms,target_left_rpm,target_right_rpm,"
          "measured_left_rpm,measured_right_rpm,pwm_left,pwm_right,"
          "effort_left_percent,effort_right_percent,governor_percent,"
          "governor_engaged,fault_reason\n",recording);
    assert(motion_init()==E_OK);
    run(0,0);run(3,6);run(8,10);run(12,14);
    transitions();
    fclose(recording);
    puts("V15 plant: production PI, encoder feedback, proportional brake, "
         "steering and zero-output STOP PASS (model only)");
    return 0;
}
