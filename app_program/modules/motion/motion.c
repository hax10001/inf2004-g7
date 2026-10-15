/** @file motion.c
 * @brief Preserve line-action speed policy independently of PWM hardware.
 */
#include "motion.h"
#include "drivers/motor.h"
#include <tm/tmonitor.h>

#define MOTION_LEFT_SPEED 15U
#define MOTION_RIGHT_SPEED 15U
#define MOTION_MAX_PERCENT 100U

/** @brief Convert a bounded percentage to the tested PWM compare count. */
static uint32_t speed_to_pwm (uint32_t speed_percent);

ER motion_init (void)
{
    motor_init();
    return E_OK;
}

ER motion_start (void)
{
    return E_OK;
}

void motion_apply (line_follow_action_t action)
{
    uint32_t left_speed  = 0U;
    uint32_t right_speed = 0U;

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[MOTOR REQUEST] Action=%s\n", line_follow_text(action));

    switch (action)
    {
        case LINE_FORWARD:
            left_speed  = MOTION_LEFT_SPEED;
            right_speed = MOTION_RIGHT_SPEED;
        break;

        case LINE_CORRECT_LEFT:
            left_speed  = 0U;
            right_speed = MOTION_RIGHT_SPEED;
        break;

        case LINE_CORRECT_RIGHT:
            left_speed  = MOTION_LEFT_SPEED;
            right_speed = 0U;
        break;

        case LINE_UNCERTAIN:
            left_speed  = 0U;
            right_speed = 0U;
        break;

        default:
            motor_stop();
            /* UB views the same bytes; the API does not modify them. */
            tm_printf((UB *)"[MOTOR] Uncertain action, stopping motors.\n");
            return;
    }

    uint32_t left_pwm  = speed_to_pwm(left_speed);
    uint32_t right_pwm = speed_to_pwm(right_speed);

    motor_set_forward(left_pwm, right_pwm);

    /* PWM counts are at most 12499, within INT; UB preserves string bytes. */
    tm_printf((UB *)"[MOTOR REQUEST] LEFT=%d RIGHT=%d\n", (INT)left_pwm,
              (INT)right_pwm);
}

static uint32_t speed_to_pwm (uint32_t speed_percent)
{
    if (speed_percent > MOTION_MAX_PERCENT)
    {
        speed_percent = MOTION_MAX_PERCENT;
    }

    return (MOTOR_PWM_WRAP * speed_percent) / MOTION_MAX_PERCENT;
}

/*** end of file ***/

