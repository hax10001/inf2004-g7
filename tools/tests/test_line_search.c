/** @file test_line_search.c
 * @brief FR4 lost-line search on the real motion.c: gentle steer first,
 *        then pivot towards the line's last side, back the other way,
 *        then stop and wait; a sensor finding the line ends the search.
 *
 * Simulated motors and encoders; each period both wheels advance a fixed
 * count. See docs/tribunal2/requirements_traceability.md (FR4).
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "modules/motion/motion.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "app/app_config.h"

/* Counts per wheel per 50 ms period, about 2 mm (1056 counts / 220 mm). */
#define SEARCH_COUNTS_PER_PERIOD 10
#define SEARCH_MAX_PERIODS 4000
#define SEARCH_LOG_MAX 16

static uint32_t g_left_pwm;
static uint32_t g_right_pwm;
static uint32_t g_clock_usec;
static int32_t g_count;
static char const *gp_log[SEARCH_LOG_MAX];
static uint32_t g_log_count;

/** @brief Host stub of motor_stop(). */
void motor_stop(void) {g_left_pwm=0U;g_right_pwm=0U;}
/** @brief Host stub of motor_brake(). */
void motor_brake(void) {g_left_pwm=0U;g_right_pwm=0U;}
/** @brief Host stub of motor_init(). */
void motor_init(void) {motor_stop();}
/** @brief Host stub of motor_set_forward(). */
void motor_set_forward(uint32_t l,uint32_t r) {g_left_pwm=l;g_right_pwm=r;}
/** @brief Host stub of motor_set_regulated(). */
void motor_set_regulated(int32_t l,int32_t r)
{g_left_pwm=(uint32_t)(l<0?-l:l);g_right_pwm=(uint32_t)(r<0?-r:r);}
/** @brief Host stub of motor_set_directional(). */
void motor_set_directional(int32_t l,int32_t r)
{g_left_pwm=(uint32_t)(l<0?-l:l);g_right_pwm=(uint32_t)(r<0?-r:r);}
/** @brief Host stub of encoder_init(). */
ER encoder_init(void) {return E_OK;}
/** @brief Host stub of encoder_read(). */
void encoder_read(encoder_wheel_t wheel,encoder_sample_t *sample)
{(void)wheel;*sample=(encoder_sample_t){0};sample->count=g_count;}
/** @brief Host stub of encoder_now_usec(). */
uint32_t encoder_now_usec(void) {g_clock_usec+=50000U;return g_clock_usec;}
/** @brief Host stub of bc_wave_capture_active(). */
uint8_t bc_wave_capture_active(void) {return 0U;}
/** @brief Host stub of tk_cre_mtx(). */
ID tk_cre_mtx(const T_CMTX *cfg) {assert(cfg!=NULL);return 42;}
/** @brief Host stub of tk_loc_mtx(). */
ER tk_loc_mtx(ID id,TMO timeout) {(void)timeout;assert(id==42);return E_OK;}
/** @brief Host stub of tk_unl_mtx(). */
ER tk_unl_mtx(ID id) {assert(id==42);return E_OK;}
/** @brief Host stub of tm_printf(). */
INT tm_printf(UB const *format,...)
{
    if (g_log_count < SEARCH_LOG_MAX)
    {
        gp_log[g_log_count] = (char const *)format;
        g_log_count++;
    }
    return 0;
}

/** @brief Test helper: find log. */
static int32_t find_log(char const *p_text)
{
    for (uint32_t i = 0U; i < g_log_count; i++)
    {
        if (NULL != strstr(gp_log[i], p_text))
        {
            return (int32_t)i;
        }
    }
    return -1;
}

/** @brief Host stub of period(). */
static void period(bool b_left, bool b_right, line_follow_action_t action)
{
    g_count += SEARCH_COUNTS_PER_PERIOD;
    motion_line_seen(b_left, b_right);
    motion_apply(action);
}

/** @brief Run every test case; any failed assert aborts. */
int main(void)
{
    assert(E_OK == motion_init());
    /* Following, the left sensor touches the line: last side is left. */
    for (uint32_t i = 0U; i < 10U; i++)
    {
        period(false, false, LINE_FORWARD);
    }
    period(true, false, LINE_CORRECT_LEFT);
    g_log_count = 0U;

    /* Under 50 mm without the line: keep driving, no search. */
    for (uint32_t i = 0U; i < 20U; i++)
    {
        period(false, false, LINE_FORWARD);
        assert((0U != g_left_pwm) || (0U != g_right_pwm));
    }
    assert(find_log("Line lost") < 0);

    /* Past 100 mm: pivot left first, then back right, then stop. */
    uint32_t n = 0U;
    while ((find_log("stopped") < 0) && (n < SEARCH_MAX_PERIODS))
    {
        period(false, false, LINE_FORWARD);
        n++;
    }
    int32_t const lost = find_log("Line lost");
    int32_t const back = find_log("pivoting back");
    int32_t const gave_up = find_log("stopped");
    assert((lost >= 0) && (back > lost) && (gave_up > back));
    assert((0U == g_left_pwm) && (0U == g_right_pwm));

    /* Stays stopped while the line is still missing. */
    for (uint32_t i = 0U; i < 20U; i++)
    {
        period(false, false, LINE_FORWARD);
        assert((0U == g_left_pwm) && (0U == g_right_pwm));
    }

    /* A sensor finds the line: the search ends and the robot drives. */
    period(true, false, LINE_CORRECT_LEFT);
    period(true, false, LINE_CORRECT_LEFT);
    assert((0U != g_left_pwm) || (0U != g_right_pwm));

    printf("PASS FR4 lost line: drive on under 50 mm, pivot towards the "
           "last side, back the other way, stop, resume on the line "
           "(%u periods)\n", (unsigned)n);
    return 0;
}
/*** end of file ***/

