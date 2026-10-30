/* Real ultrasonic and servo drivers against simulated PWM/GPIO registers.
 * Time advances in ten-millisecond RTOS ticks, including across timer wrap.
 */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "modules/obstacle/ultrasonic.h"
#include "modules/obstacle/servo.h"
#include "modules/obstacle/obs_state.h"
#include "modules/obstacle/obs_config.h"
#include <bsp/libbsp.h>

static uint32_t usec, trigger_usec, pulse_usec, compare;
static uint32_t csr, divider, top, control;
static unsigned delays, output_writes, lock_depth;
static int triggered, stuck, pwm_failure;
static UINT flags;

INT tm_printf(const UB *format, ...) { (void)format; return 0; }
ID tk_cre_mtx(const T_CMTX *p) { (void)p; return 1; }
ER tk_loc_mtx(ID id, TMO t)
{ (void)id; (void)t; assert(lock_depth++ == 0); return E_OK; }
ER tk_unl_mtx(ID id) { (void)id; assert(lock_depth-- == 1); return E_OK; }
ID tk_cre_flg(const T_CFLG *p) { flags = p->iflgptn; return 2; }
ER tk_ref_flg(ID id, T_RFLG *p) { (void)id; p->flgptn = flags; return E_OK; }
ER tk_set_flg(ID id, UINT bits) { (void)id; flags |= bits; return E_OK; }
ER tk_clr_flg(ID id, UINT keep) { (void)id; flags &= keep; return E_OK; }
ER tk_dly_tsk(UINT delay)
{
    ++delays;
    usec += ((delay + 9) / 10) * 10000;
    return E_OK;
}
uint32_t encoder_now_usec(void) { return usec++; }
ER gpio_set_pin(UINT pin, UINT mode)
{ assert(pin == BOARD_US_TRIG_PIN && mode == GPIO_MODE_OUT); return E_OK; }
ER gpio_set_val(UINT pin, UINT value)
{
    assert(pin == BOARD_US_TRIG_PIN);
    if (value) { triggered = 1; trigger_usec = usec; }
    return E_OK;
}
UINT gpio_get_val(UINT pin)
{
    assert(pin == BOARD_US_ECHO_PIN);
    if (stuck) { return 1; }
    return triggered && (uint32_t)(usec - trigger_usec) < pulse_usec;
}
ER pwm_set_pin(UINT pin)
{ assert(pin == BOARD_US_ECHO_PIN || pin == BOARD_SERVO_PIN); return E_OK; }
ER pwm_set_wrap(UINT pin, UW value)
{ assert(pin == BOARD_SERVO_PIN && value == 19999); return E_OK; }
ER pwm_set_cc(UINT pin, UW value)
{
    assert(pin == BOARD_SERVO_PIN);
    if (pwm_failure) { return E_OBJ; }
    compare = value;
    return E_OK;
}
ER pwm_set_enabled(UINT pin, BOOL value)
{ assert(pin == BOARD_SERVO_PIN && value); return E_OK; }
UW in_w(UW address)
{
    if (address == 0x40050030U)
    {
        if (!triggered) { return 0; }
        uint32_t elapsed = usec - trigger_usec;
        return elapsed < pulse_usec ? elapsed : pulse_usec;
    }
    if (address == GPIO_CTRL(BOARD_US_ECHO_PIN)) { return control; }
    return 0;
}
void out_w(UW address, UW value)
{
    ++output_writes;
    if (address == 0x40050028U) { csr = value; }
    if (address == 0x4005002cU) { divider = value; }
    if (address == 0x40050038U) { top = value; }
    if (address == GPIO_CTRL(BOARD_US_ECHO_PIN)) { control = value; }
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *name = argv[1];
    assert(obs_state_init() == E_OK);
    assert(ultrasonic_init() == E_OK);
    assert(csr == 0x11 && divider == 2000 && top == 65535);
    assert((control & (3U << 12)) == (2U << 12));
    if (strcmp(name, "servo") == 0)
    {
        assert(servo_init() == E_OK && compare == 1500);
        assert(servo_set_angle(200) == 280 && compare == 2300);
        assert(servo_set_angle(-200) == 520 && compare == 700);
        pwm_failure = 1;
        servo_set_angle(0);
        assert(obs_state_status_faults(obs_state_status_get()) & OBS_FAULT_SERVO);
        assert(servo_init() == E_OBJ);
    }
    else
    {
        int32_t mm = -1;
        pulse_usec = 1000;
        if (strcmp(name, "echo_wrap") == 0) { usec = UINT32_MAX - 50; }
        if (strcmp(name, "no_echo") == 0) { pulse_usec = 0; }
        if (strcmp(name, "too_far") == 0) { pulse_usec = 25000; }
        if (strcmp(name, "long_echo") == 0) { pulse_usec = 50000; }
        if (strcmp(name, "stuck_echo") == 0) { stuck = 1; }
        ultrasonic_result_t result = ultrasonic_ping(&mm);
        if (strcmp(name, "no_echo") == 0 || stuck)
        { assert(result == ULTRASONIC_NO_ECHO && mm == 0); }
        else if (pulse_usec >= 25000)
        { assert(result == ULTRASONIC_OUT_OF_RANGE && mm == 0); }
        else { assert(result == ULTRASONIC_OK && mm == 171); }
        assert(delays <= 3); /* 30 ms deadline, not thirty 10 ms ticks. */
        assert(ultrasonic_ping(NULL) == ULTRASONIC_NO_ECHO);
        assert(ultrasonic_median_of_three(10, 3, 7) == 7);
        assert(ultrasonic_median_of_three(3, 10, 7) == 7);
    }
    printf("PASS %s (RTOS polls=%u, writes=%u)\n", name, delays, output_writes);
    return 0;
}
