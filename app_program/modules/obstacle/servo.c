/** @file servo.c
 *
 * @brief 1 MHz PWM tick, 20 msec period, pulse width set by compare value.
 *
 * The sweep is limited so that every angle maps to its own pulse. Clamping
 * only the pulse would send several angles to the same position while the
 * scan still labels them with different angles.
 */
#include "servo.h"
#include "obs_config.h"
#include <bsp/libbsp.h>

#ifndef PWM_BASE
#define PWM_BASE 0x40050000U
#endif

#if ((OBS_SERVO_FRONT_USEC < OBS_SERVO_MIN_USEC) ||                            \
     (OBS_SERVO_FRONT_USEC > OBS_SERVO_MAX_USEC))
#error "OBS_SERVO_FRONT_USEC must lie inside the servo pulse range."
#endif

#define SERVO_SLICE_MASK 7U
#define SERVO_SLICE_STRIDE 0x14U
#define SERVO_SLICE_BASE                                                       \
    (PWM_BASE +                                                                \
     (((BOARD_SERVO_PIN >> 1U) & SERVO_SLICE_MASK) * SERVO_SLICE_STRIDE))
#define SERVO_REG_CSR (SERVO_SLICE_BASE + 0x00U)
#define SERVO_REG_DIV (SERVO_SLICE_BASE + 0x04U)
#define SERVO_WRAP 19999U
#define SERVO_DIV_INT_SHIFT 4U

/* Pulse room on each side of FRONT, in usec. */
#define SERVO_LOW_ROOM_USEC (OBS_SERVO_FRONT_USEC - OBS_SERVO_MIN_USEC)
#define SERVO_HIGH_ROOM_USEC (OBS_SERVO_MAX_USEC - OBS_SERVO_FRONT_USEC)
#define SERVO_LEFT_ROOM_USEC                                                   \
    ((OBS_SERVO_LEFT_SIGN > 0) ? SERVO_HIGH_ROOM_USEC : SERVO_LOW_ROOM_USEC)
#define SERVO_RIGHT_ROOM_USEC                                                  \
    ((OBS_SERVO_LEFT_SIGN > 0) ? SERVO_LOW_ROOM_USEC : SERVO_HIGH_ROOM_USEC)
#define SERVO_MIN_OF(a, b) (((a) < (b)) ? (a) : (b))
#define SERVO_LEFT_DEG                                                         \
    SERVO_MIN_OF(OBS_SERVO_LEFT_LIMIT_DEG,                                     \
                 (SERVO_LEFT_ROOM_USEC / OBS_SERVO_USEC_PER_DEG))
#define SERVO_RIGHT_DEG                                                        \
    SERVO_MIN_OF(OBS_SERVO_RIGHT_LIMIT_DEG,                                    \
                 (SERVO_RIGHT_ROOM_USEC / OBS_SERVO_USEC_PER_DEG))
#define SERVO_SPAN_DEG (SERVO_LEFT_DEG + SERVO_RIGHT_DEG)

static int32_t g_angle_deg = 0;

/** @brief Bound a value between a low and a high limit. */
static int32_t clamp_int (int32_t value, int32_t low, int32_t high);
/** @brief Write the pulse for an angle already inside the usable sweep. */
static void write_angle (int32_t angle);

ER servo_init (void)
{
    ER error = pwm_set_pin(BOARD_SERVO_PIN);

    if (error >= E_OK)
    {
        out_w(SERVO_REG_CSR, 0U);
        out_w(SERVO_REG_DIV, OBS_PWM_CLK_MHZ << SERVO_DIV_INT_SHIFT);
        error = pwm_set_wrap(BOARD_SERVO_PIN, SERVO_WRAP);
    }

    if (error >= E_OK)
    {
        /* The FRONT pulse is loaded before the output starts. */
        (void)servo_home();
        error = pwm_set_enabled(BOARD_SERVO_PIN, TRUE);
    }

    return error;
}

uint32_t servo_home (void)
{
    write_angle(0);

    return OBS_SERVO_HOME_MSEC;
}

uint32_t servo_set_angle (int32_t angle_deg)
{
    int32_t const angle =
        clamp_int(angle_deg, -SERVO_RIGHT_DEG, SERVO_LEFT_DEG);
    int32_t const moved =
        clamp_int(angle - g_angle_deg, -SERVO_SPAN_DEG, SERVO_SPAN_DEG);
    int32_t const travel = (moved < 0) ? -moved : moved;
    int32_t const settle =
        OBS_SERVO_SETTLE_BASE_MSEC + (travel * OBS_SERVO_SETTLE_PER_DEG_MSEC);

    write_angle(angle);

    /* The settle time is a sum of non-negative terms, so the cast is exact. */
    return (uint32_t)settle;
}

int32_t servo_angle_get (void)
{
    return g_angle_deg;
}

int32_t servo_left_limit_deg (void)
{
    return SERVO_LEFT_DEG;
}

int32_t servo_right_limit_deg (void)
{
    return SERVO_RIGHT_DEG;
}

static void write_angle (int32_t angle)
{
    int32_t const raw_pulse =
        OBS_SERVO_FRONT_USEC +
        (OBS_SERVO_LEFT_SIGN * angle * OBS_SERVO_USEC_PER_DEG);
    /* Kept as a guard; the sweep limits already keep the pulse in range. */
    int32_t const pulse =
        clamp_int(raw_pulse, OBS_SERVO_MIN_USEC, OBS_SERVO_MAX_USEC);

    /* The pulse is clamped to a positive range, so the cast is exact. */
    pwm_set_cc(BOARD_SERVO_PIN, (UW)pulse);
    g_angle_deg = angle;
}

static int32_t clamp_int (int32_t value, int32_t low, int32_t high)
{
    int32_t result = value;

    if (value < low)
    {
        result = low;
    }
    else if (value > high)
    {
        result = high;
    }
    else
    {
        /* The value is already inside the limits. */
    }

    return result;
}

/*** end of file ***/

