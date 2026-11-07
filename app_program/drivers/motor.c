/** @file motor.c
 * @brief Preserve the tested motor wiring and BSP operation sequence.
 */
#include "motor.h"
#include "board/board_config.h"
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#define MOTOR_PIN_COUNT 4U
static volatile uint32_t g_last_left_pwm;
static volatile uint32_t g_last_right_pwm;
/* Log only the change to stopped: the controller re-applies stop every
 * 50 ms while stopped, and blocking USB prints disturb encoder timing. */
static bool g_b_stopped = false;

void motor_last_command (uint16_t * left, uint16_t * right)
{
    if (left != NULL) *left = (uint16_t)g_last_left_pwm;
    if (right != NULL) *right = (uint16_t)g_last_right_pwm;
}

void motor_init (void)
{
    g_last_left_pwm = 0U;
    g_last_right_pwm = 0U;
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
    g_last_left_pwm = 0U;
    g_last_right_pwm = 0U;
    uint32_t const pins[MOTOR_PIN_COUNT] = {
        BOARD_MOTOR_LEFT_A_PIN, BOARD_MOTOR_LEFT_B_PIN, BOARD_MOTOR_RIGHT_A_PIN,
        BOARD_MOTOR_RIGHT_B_PIN};

    for (uint32_t index = 0U; index < MOTOR_PIN_COUNT; index++)
    {
        pwm_set_cc(pins[index], 0U);
    }

    /* UB views the same string bytes; tm_printf does not modify them. */
#ifndef B1_BARCODE_TEST /* quiet barcode test log */
    if (!g_b_stopped)
    {
        tm_printf((UB *)"[MOTOR STOP] Motors stopped successfully.\n");
    }
#endif
    g_b_stopped = true;
}

void motor_brake (void)
{
    g_last_left_pwm = 0U;
    g_last_right_pwm = 0U;
    g_b_stopped = false; /* the release to coast is logged */
    uint32_t const pins[MOTOR_PIN_COUNT] = {
        BOARD_MOTOR_LEFT_A_PIN, BOARD_MOTOR_LEFT_B_PIN, BOARD_MOTOR_RIGHT_A_PIN,
        BOARD_MOTOR_RIGHT_B_PIN};

    /* A compare level above the wrap holds the output high all period. */
    for (uint32_t index = 0U; index < MOTOR_PIN_COUNT; index++)
    {
        pwm_set_cc(pins[index], MOTOR_PWM_WRAP + 1U);
    }
}

void motor_set_forward (uint32_t left_pwm, uint32_t right_pwm)
{
    g_b_stopped = false;
    g_last_left_pwm = left_pwm;
    g_last_right_pwm = right_pwm;
    /* WARNING: The tested robot drives the left wheel through its B pin. */
    pwm_set_cc(BOARD_MOTOR_LEFT_A_PIN, 0U);
    pwm_set_cc(BOARD_MOTOR_LEFT_B_PIN, left_pwm);
    pwm_set_cc(BOARD_MOTOR_RIGHT_A_PIN, right_pwm);
    pwm_set_cc(BOARD_MOTOR_RIGHT_B_PIN, 0U);
}

void motor_set_directional (int32_t left_pwm, int32_t right_pwm)
{
    uint32_t left = (uint32_t)((left_pwm < 0) ? -left_pwm : left_pwm);
    uint32_t right = (uint32_t)((right_pwm < 0) ? -right_pwm : right_pwm);
    if (left > MOTOR_PWM_WRAP) left = MOTOR_PWM_WRAP;
    if (right > MOTOR_PWM_WRAP) right = MOTOR_PWM_WRAP;
    g_b_stopped = false;
    g_last_left_pwm = left;
    g_last_right_pwm = right;
    /* Preserve original tested forward polarity: L on B, R on A.
     * Reverse uses the opposite H-bridge input on the same motor. */
    pwm_set_cc(BOARD_MOTOR_LEFT_A_PIN, left_pwm < 0 ? left : 0U);
    pwm_set_cc(BOARD_MOTOR_LEFT_B_PIN, left_pwm >= 0 ? left : 0U);
    pwm_set_cc(BOARD_MOTOR_RIGHT_A_PIN, right_pwm >= 0 ? right : 0U);
    pwm_set_cc(BOARD_MOTOR_RIGHT_B_PIN, right_pwm < 0 ? right : 0U);
}

/*** end of file ***/

