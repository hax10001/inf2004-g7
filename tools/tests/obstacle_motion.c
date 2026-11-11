/* Exercise the real speed controller with simulated motors and encoders. */
#include <assert.h>
#include <stdio.h>
#include "modules/motion/motion.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "app/app_config.h"

static uint32_t left_pwm,right_pwm;
static uint32_t clock_usec;

void motor_stop(void) {left_pwm=0U;right_pwm=0U;}
void motor_brake(void) {left_pwm=0U;right_pwm=0U;}
void motor_init(void) {motor_stop();}
void motor_set_forward(uint32_t l,uint32_t r) {left_pwm=l;right_pwm=r;}
void motor_set_regulated(int32_t l,int32_t r)
{left_pwm=(uint32_t)(l<0?-l:l);right_pwm=(uint32_t)(r<0?-r:r);}
void motor_set_directional(int32_t l,int32_t r)
{left_pwm=(uint32_t)(l<0?-l:l);right_pwm=(uint32_t)(r<0?-r:r);}
ER encoder_init(void) {return E_OK;}
void encoder_read(encoder_wheel_t wheel,encoder_sample_t *sample)
{(void)wheel;*sample=(encoder_sample_t){0};}
uint32_t encoder_now_usec(void) {clock_usec+=50000U;return clock_usec;}
uint8_t bc_wave_capture_active(void) {return 0U;}
ID tk_cre_mtx(const T_CMTX *cfg) {assert(cfg!=NULL);return 42;}
ER tk_loc_mtx(ID id,TMO timeout) {(void)timeout;assert(id==42);return E_OK;}
ER tk_unl_mtx(ID id) {assert(id==42);return E_OK;}
INT tm_printf(UB const *format,...) {(void)format;return 0;}

int main(void)
{
    motion_status_t status={0};
    assert(motion_init()==E_OK);
    assert(left_pwm==0U && right_pwm==0U);
    for(int i=0;i<3;i++)motion_apply(LINE_FORWARD);
    assert(motion_get_status(&status)==E_OK);
    assert(status.left_target_rpm==15.0f);
    assert(status.right_target_rpm==15.0f);
    motion_drive(0.0f,25.0f);
    assert(left_pwm==0U && right_pwm>0U);
    assert(motion_get_status(&status)==E_OK);
    assert(status.left_target_rpm==0.0f);
    assert(status.right_target_rpm==23.0f);
    motion_drive(0.0f,25.0f);
    assert(motion_get_status(&status)==E_OK);
    assert(status.right_target_rpm==25.0f);
    motion_drive(25.0f,0.0f);
    assert(left_pwm>0U && right_pwm==0U);
    motion_apply(LINE_UNCERTAIN);
    assert(left_pwm==0U && right_pwm==0U);
    assert(motion_get_status(&status)==E_OK);
    assert(status.left_pwm==0U && status.right_pwm==0U);
    motion_drive(25.0f,25.0f);
    motion_drive(0.0f,0.0f);
    assert(left_pwm==0U && right_pwm==0U);
#if APP_DOWNHILL_NOSE_DOWN_SIGN != 0
    /* Real production motion.c target path, not only a pure helper. */
    float const down = (float)(8 * APP_DOWNHILL_NOSE_DOWN_SIGN);
    for (int i=0;i<10;i++)
        motion_downhill_sample(down,0.0f,true,true);
    for (int i=0;i<3;i++)motion_apply(LINE_FORWARD);
    assert(motion_get_status(&status)==E_OK);
    assert(status.left_target_rpm>9.7f &&
           status.left_target_rpm<9.8f);
    assert(status.right_target_rpm>9.7f &&
           status.right_target_rpm<9.8f);
    for (int i=0;i<5;i++)motion_apply(LINE_CORRECT_LEFT);
    assert(motion_get_status(&status)==E_OK);
    assert(status.left_target_rpm<status.right_target_rpm);
    motion_apply(LINE_UNCERTAIN);
    for (int i=0;i<5;i++)motion_apply(LINE_CORRECT_RIGHT);
    assert(motion_get_status(&status)==E_OK);
    assert(status.right_target_rpm<status.left_target_rpm);
    motion_apply(LINE_UNCERTAIN);
    motion_drive(10.0f,20.0f);
    motion_drive(10.0f,20.0f);
    assert(motion_get_status(&status)==E_OK);
    assert(status.left_target_rpm>6.4f &&
           status.left_target_rpm<6.6f);
    assert(status.right_target_rpm>12.9f &&
           status.right_target_rpm<13.1f);
    for (int i=0;i<20;i++)
        motion_downhill_sample(0.0f,0.0f,false,true);
    assert(motion_downhill_faulted());
    motion_apply(LINE_FORWARD);
    assert(left_pwm==0U && right_pwm==0U);
    motion_drive(25.0f,25.0f);
    assert(left_pwm==0U && right_pwm==0U);
    motion_downhill_reset(); /* Controlled test of explicit STOP reset. */
    assert(!motion_downhill_faulted());
#endif
    puts("Obstacle motion: RPM ramp, stopped inner wheel and immediate STOP PASS");
    return 0;
}
