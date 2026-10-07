#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "motor_controller.h"

#define RIGHT_OFFSET 5U

#define BASE_SPEED 20U
#define BASE_LEFT_SPEED 15U
#define BASE_RIGHT_SPEED 15U
#define TURN_SPEED 20U

#define MOTOR_M1A 8U
#define MOTOR_M1B 9U
#define MOTOR_M2A 10U
#define MOTOR_M2B 11U

// This part is the pwm slice, with wrap and all those things.
#include <bsp/libbsp.h>
#define PWM_WRAP 12499U
// ==================================================================
// Speed convert function
// ==================================================================
static UINT speed_to_pwm(UINT speed_percent) {
    if (speed_percent > 100U) {
        speed_percent = 100U;
    }
    return (PWM_WRAP * speed_percent) / 100U;
}

void motor_controller_init(void)
{
    /* Initialize motor pins as PWM and set them LOW */
    ER er;
    UW duty;
    /* GP8, GP9, GP10, GP11 -> PWM function */
    er = pwm_set_pin(MOTOR_M1A);
    tm_printf((UB *)"M1A pin: %d\n", er);

    er = pwm_set_pin(MOTOR_M1B);
    tm_printf((UB *)"M1B pin: %d\n", er);

    er = pwm_set_pin(MOTOR_M2A);
    tm_printf((UB *)"M2A pin: %d\n", er);

    er = pwm_set_pin(MOTOR_M2B);
    tm_printf((UB *)"M2B pin: %d\n", er);

    pwm_set_wrap(MOTOR_M1A, PWM_WRAP);
    pwm_set_wrap(MOTOR_M2A, PWM_WRAP);

    /* Start completely stopped */
    pwm_set_cc(MOTOR_M1A, 0U);
    pwm_set_cc(MOTOR_M1B, 0U);
    pwm_set_cc(MOTOR_M2A, 0U);
    pwm_set_cc(MOTOR_M2B, 0U);

    pwm_set_enabled(MOTOR_M1A, TRUE);
    pwm_set_enabled(MOTOR_M2A, TRUE);

    tm_printf((UB *)"[MOTOR INIT] Motor controller initialized PWM successfully.\n");
}

void motor_controller_stop(void)
{
    /* Stop the motors by setting all control pins LOW */
    const UINT pins[4] = {
        MOTOR_M1A,
        MOTOR_M1B,
        MOTOR_M2A,
        MOTOR_M2B
    };

    for (UINT i = 0; i < 4; i++) {
        pwm_set_cc(pins[i], 0U);
    }

    tm_printf((UB *)"[MOTOR STOP] Motors stopped successfully.\n");
}

void motor_controller_apply(LINE_ACTION action)
{
    UINT left_speed  = 0U;
    UINT right_speed = 0U;

    tm_printf((UB *)"[MOTOR REQUEST] Action=%s\n", line_follow_text(action));

    switch (action) {
        case LINE_FORWARD:
            left_speed  = BASE_LEFT_SPEED;
            right_speed = BASE_RIGHT_SPEED;
            break;

        case LINE_CORRECT_LEFT:
            left_speed  = 0U;
            right_speed = BASE_RIGHT_SPEED;
            break;

        case LINE_CORRECT_RIGHT:
            left_speed  = BASE_LEFT_SPEED;
            right_speed = 0U;
            break;

        case LINE_UNCERTAIN:
            left_speed  = 0U;
            right_speed = 0U;
            break;
        default:
            motor_controller_stop();
            tm_printf((UB *)"[MOTOR] Uncertain action, stopping motors.\n");
            return;
    }

    // After getting the speed values from IR_TASK
    // Convert speed percentages to PWM values
    UINT left_pwm = speed_to_pwm(left_speed);
    UINT right_pwm = speed_to_pwm(right_speed);

    // Assuiming M1A/M2A = forward direction, M1B/M2B = reverse direction
    // Currently this is hardcoded to only go forward, so M1B/M2B are always 0 with varying speed.
    pwm_set_cc(MOTOR_M1A, 0U); // Ensure reverse is off
    pwm_set_cc(MOTOR_M1B, left_pwm); 

    pwm_set_cc(MOTOR_M2A, right_pwm);
    pwm_set_cc(MOTOR_M2B, 0U); // Ensure reverse is off

    tm_printf(
        (UB *)"[MOTOR REQUEST] LEFT=%d RIGHT=%d\n",
        left_pwm,
        right_pwm
    );
}