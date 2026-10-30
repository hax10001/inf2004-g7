/** @file motor.c
 * @brief Preserve the tested motor wiring and BSP operation sequence.
 */
#include "motor.h"
#include "board/board_config.h"
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#define MOTOR_PIN_COUNT 4U

void motor_init (void)
{
    ER error = pwm_set_pin(BOARD_MOTOR_LEFT_A_PIN);
    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"M1A pin: %d\n", error);

    error = pwm_set_pin(BOARD_MOTOR_LEFT_B_PIN);
    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"M1B pin: %d\n", error);

    error = pwm_set_pin(BOARD_MOTOR_RIGHT_A_PIN);
    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"M2A pin: %d\n", error);

    error = pwm_set_pin(BOARD_MOTOR_RIGHT_B_PIN);
    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"M2B pin: %d\n", error);

    pwm_set_wrap(BOARD_MOTOR_LEFT_A_PIN, MOTOR_PWM_WRAP);
    pwm_set_wrap(BOARD_MOTOR_RIGHT_A_PIN, MOTOR_PWM_WRAP);

    pwm_set_cc(BOARD_MOTOR_LEFT_A_PIN, 0U);
    pwm_set_cc(BOARD_MOTOR_LEFT_B_PIN, 0U);
    pwm_set_cc(BOARD_MOTOR_RIGHT_A_PIN, 0U);
    pwm_set_cc(BOARD_MOTOR_RIGHT_B_PIN, 0U);

    pwm_set_enabled(BOARD_MOTOR_LEFT_A_PIN, TRUE);
    pwm_set_enabled(BOARD_MOTOR_RIGHT_A_PIN, TRUE);

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[MOTOR INIT] Motor controller initialized PWM "
                    "successfully.\n");
}

void motor_stop (void)
{
    uint32_t const pins[MOTOR_PIN_COUNT] = {
        BOARD_MOTOR_LEFT_A_PIN, BOARD_MOTOR_LEFT_B_PIN, BOARD_MOTOR_RIGHT_A_PIN,
        BOARD_MOTOR_RIGHT_B_PIN};

    for (uint32_t index = 0U; index < MOTOR_PIN_COUNT; index++)
    {
        pwm_set_cc(pins[index], 0U);
    }

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[MOTOR STOP] Motors stopped successfully.\n");
}

void motor_set_forward (uint32_t left_pwm, uint32_t right_pwm)
{
    /* WARNING: The tested robot drives the left wheel through its B pin. */
    pwm_set_cc(BOARD_MOTOR_LEFT_A_PIN, 0U);
    pwm_set_cc(BOARD_MOTOR_LEFT_B_PIN, left_pwm);
    pwm_set_cc(BOARD_MOTOR_RIGHT_A_PIN, right_pwm);
    pwm_set_cc(BOARD_MOTOR_RIGHT_B_PIN, 0U);
}

/*** end of file ***/

