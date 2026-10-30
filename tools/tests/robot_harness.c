/* Host-only regression harness. setjmp stops infinite robot tasks at a
 * deterministic mock scheduling boundary; none of this enters firmware.
 */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include <math.h>
#include <tk/tkernel.h>
#include <dev_i2c.h>

#if LEGACY
#include "controller_task.h"
#include "ir_task.h"
#include "imu_task.h"
#include "ir_shared.h"
#include "line_follow.h"
#include "motor_controller.h"
INT usermain(void);
#define RUN_CTRL controller_task
#define RUN_IR ir_task
#define RUN_IMU imu_task
#define RUN_MOTOR motor_controller_apply
#else
#include "app_main.h"
#include "app/task_controller.h"
#include "modules/line_barcode/task_ir.h"
#include "modules/terrain/task_imu.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/line_barcode/line_follow.h"
#include "modules/motion/motion.h"
#include "drivers/lsm303.h"
#include "modules/terrain/terrain.h"
#include "modules/motion/wheel_speed.h"
#include "modules/motion/steering.h"
#include "board/board_config.h"
#define RUN_CTRL task_controller_task
#define RUN_IR task_ir_task
#define RUN_IMU task_imu_task
#define RUN_MOTOR motion_apply
#endif

static jmp_buf escape_task;
static int in_task, delays, delay_limit = 8, mode, variant;
static int create_calls, start_calls, pin_calls, time_calls, write_calls;
static int fail_create, fail_start, fail_mutex, fail_lock, fail_time;
static int fail_pin, fail_write, short_write, fail_open;
static int create_zero;
static uint32_t now_msec = 1000U;
static int quiet, exhaustive;
static uint16_t raw_word;

INT tm_printf(const UB *format, ...)
{
    va_list args;
    va_start(args, format);
    int count = vprintf((const char *)format, args);
    va_end(args);
    return count;
}

ID tk_cre_tsk(const T_CTSK *config)
{
    ++create_calls;
    printf("CREATE %d priority=%d stack=%d attr=%u\n", create_calls,
           config->itskpri, config->stksz, config->tskatr);
    return create_calls == fail_create ? (create_zero ? 0 : -5) : create_calls;
}
ER tk_sta_tsk(ID task, INT code)
{
    ++start_calls;
    printf("START %d code=%d\n", task, code);
    return start_calls == fail_start ? -5 : E_OK;
}
ID tk_cre_mtx(const T_CMTX *config)
{
    printf("MUTEX attr=%u\n", config->mtxatr);
    return fail_mutex ? -5 : 10;
}
ER tk_loc_mtx(ID mutex, TMO timeout)
{
    return fail_lock ? -5 : E_OK;
}
ER tk_unl_mtx(ID mutex) { return E_OK; }
ER tk_del_mtx(ID mutex) { return E_OK; }
ER tk_get_otm(SYSTIM *time)
{
    if (++time_calls == fail_time) { return -5; }
    time->hi = 0;
    time->lo = now_msec;
    return E_OK;
}
ER tk_slp_tsk(TMO timeout)
{
    printf("SLEEP %d\n", timeout);
    if (in_task) { longjmp(escape_task, 1); }
    return E_OK;
}
ER tk_dly_tsk(UINT delay)
{
    printf("DELAY %u\n", delay);
    now_msec += delay;
    ++delays;
    if (in_task && delays >= delay_limit) { longjmp(escape_task, 1); }
    if (mode == 1 && in_task)
    {
        if (variant == 2 && delays >= 2) { fail_lock = 1; }
        if (variant == 3 && delays >= 2) { fail_time = time_calls + 1; }
        if (variant == 6 && delays == 2)
        {
            /* A junction shorter than one controller period. */
            ir_shared_publish(1U, 1U, 1U);
            ir_shared_publish(0U, 0U, 1U);
        }
        else if (variant == 7)
        {
            /* Stops on a junction, lifted (reads black), placed on a line. */
            unsigned black = delays >= 2 && delays < 20;
            ir_shared_publish(black, black, 1U);
        }
        else if (variant == 8)
        {
            /* Coasts off the junction at once and is never touched. */
            unsigned black = delays == 2;
            ir_shared_publish(black, black, 1U);
        }
        else if (variant == 9 && delays == 2)
        {
            /* A left edge touch shorter than one controller period. */
            ir_shared_publish(1U, 0U, 1U);
            ir_shared_publish(0U, 0U, 1U);
        }
        else if (variant != 1 && variant != 2 && variant != 3)
        {
            unsigned left = delays == 2 || delays == 4;
            unsigned right = delays == 3 || delays == 4;
            ir_shared_publish(left, right, 1U);
        }
    }
    return E_OK;
}
ER gpio_set_pin(UINT pin, UINT pin_mode)
{
    printf("GPIO_INIT %u mode=%u\n", pin, pin_mode);
    return ++pin_calls == fail_pin ? -5 : E_OK;
}
UINT gpio_get_val(UINT pin)
{
    UINT value = (pin + delays) % 2U;
    printf("GPIO_READ %u value=%u\n", pin, value);
    return value;
}
ER gpio_set_val(UINT pin, UINT value) { return E_OK; }
ER pwm_set_pin(UINT pin) { printf("PWM_PIN %u\n", pin); return E_OK; }
ER pwm_set_wrap(UINT pin, UW wrap)
{
    printf("PWM_WRAP %u %u\n", pin, wrap); return E_OK;
}
ER pwm_set_cc(UINT pin, UW compare)
{
    printf("PWM_CC %u %u\n", pin, compare); return E_OK;
}
ER pwm_set_enabled(UINT pin, BOOL enabled)
{
    printf("PWM_ENABLE %u %u\n", pin, enabled); return E_OK;
}
ID tk_opn_dev(UB *name, UINT access)
{
    printf("DEVICE_OPEN %s %u\n", name, access);
    return fail_open ? -5 : 20;
}
ER tk_cls_dev(ID device, UINT option)
{
    printf("DEVICE_CLOSE %d\n", device); return E_OK;
}
ER tk_swri_dev(ID device, W start, void *data, SZ size, SZ *actual)
{
    ++write_calls;
    *actual = short_write == write_calls ? size - 1 : size;
    if (start == TDN_I2C_EXEC)
    {
        T_I2C_EXEC *txn = data;
        if (!quiet)
        {
            printf("I2C_READ address=%u reg=%u bytes=%d\n",
                   txn->sadr, txn->snd_data[0], txn->rcv_size);
        }
        for (int index = 0; index < txn->rcv_size; ++index)
        {
            txn->rcv_data[index] = exhaustive
                ? (uint8_t)((index % 2 == 0) ? raw_word : raw_word >> 8)
                : (uint8_t)((index * 37 + write_calls * 13) & 255);
        }
    }
    else if (!quiet)
    {
        UB *bytes = data;
        printf("I2C_WRITE address=%d reg=%u value=%u\n",
               start, bytes[0], bytes[1]);
    }
    return fail_write == write_calls ? -5 : E_OK;
}

/* The timer follows simulated time; reset has completed; other reads are 0. */
UW in_w(UW address)
{
    if (address == TIMER_TIMERAWL) { return now_msec * 1000U; }
    if (address == RESETS_RESET_DONE) { return RESETS_RESET_TIMER; }
    return 0U;
}
void out_w(UW address, UW value) { (void)address; (void)value; }
ER tk_def_int(UINT intno, const T_DINT *config)
{
    (void)intno; (void)config; return E_OK;
}
void EnableInt(UINT intno, INT level) { (void)intno; (void)level; }
void ClearInt(UINT intno) { (void)intno; }

#if !LEGACY
static void run_wheel_speed_units(void)
{
    wheel_speed_t speed;
    float const counts = (float)BOARD_ENC_COUNTS_PER_REV;

    wheel_speed_reset(&speed, 0, 1000U);
    assert(0.0f == speed.rpm);
    /* 10 edges in 10 ms is 1000 edges per second. */
    float rpm = wheel_speed_update(&speed, 10, 11000U, 12000U);
    assert(fabsf(rpm - 60000.0f / counts) < 0.01f);
    /* Reverse direction reports the same magnitude. */
    rpm = wheel_speed_update(&speed, 0, 21000U, 21000U);
    assert(fabsf(rpm - 60000.0f / counts) < 0.01f);
    /* No edge for 50 ms: at most one edge per 50 ms. */
    rpm = wheel_speed_update(&speed, 0, 21000U, 71000U);
    assert(fabsf(rpm - 1200.0f / counts) < 0.01f);
    /* No edge beyond the stop interval reads as stopped. */
    rpm = wheel_speed_update(&speed, 0, 21000U,
                             21001U + WHEEL_SPEED_STOP_USEC);
    assert(0.0f == rpm);
    /* The first edge after rest has no full interval to measure. */
    rpm = wheel_speed_update(&speed, 1, 900000U, 900000U);
    assert(0.0f == rpm);
    /* Timer and count wraparound still give the true difference. */
    wheel_speed_reset(&speed, INT32_MAX, UINT32_MAX - 499U);
    rpm = wheel_speed_update(&speed, INT32_MIN, 500U, 500U);
    assert(fabsf(rpm - 60000.0f / counts) < 0.01f);
}

static void run_units(void)
{
    run_wheel_speed_units();
    motion_status_t status;
    assert(E_PAR == motion_get_status(NULL));
    assert(E_OBJ == motion_get_status(&status));
    quiet = 1;
    assert(E_OK == motion_init());
    assert(E_OK == motion_get_status(&status) && !status.b_valid);
    /* Base speed ramps 8 RPM per period; first period is feedforward only:
     * deadband 8 + (21 - 8) x 8/40 = 10.6 percent of 12499 on the left,
     * and 8 + (22 - 8) x 8/40 = 10.8 percent on the right.
     */
    motion_apply(LINE_FORWARD);
    assert(E_OK == motion_get_status(&status) && status.b_valid);
    assert(8.0f == status.left_target_rpm && 8.0f == status.right_target_rpm);
    assert(1324U == status.left_pwm && 1349U == status.right_pwm);
    for (int period = 0; period < 4; ++period) { motion_apply(LINE_FORWARD); }
    assert(E_OK == motion_get_status(&status));
    assert(15.0f == status.left_target_rpm && 0.0f == status.steer);
    /* Off-centre asks for 0.3 + 2.0 x 0.05; steer moves 0.3 per period,
     * and the speed drops by 0.6 x 0.3 to 12.3 at the current 15 RPM base.
     */
    motion_apply(LINE_CORRECT_LEFT);
    assert(E_OK == motion_get_status(&status));
    assert(fabsf(status.steer - 0.3f) < 1e-5f);
    assert(fabsf(status.left_target_rpm - 8.61f) < 1e-3f);
    assert(fabsf(status.right_target_rpm - 15.99f) < 1e-3f);
    assert(status.left_pwm > 0U && status.left_pwm < status.right_pwm);
    /* Back on the line after a short touch: the nudge ends at once. */
    motion_apply(LINE_FORWARD);
    assert(E_OK == motion_get_status(&status));
    assert(0.0f == status.steer);
    /* Crossing over: the old left turn is dropped at once, and steer heads
     * towards -0.4 by one 0.3 step.
     */
    motion_apply(LINE_CORRECT_RIGHT);
    assert(E_OK == motion_get_status(&status));
    assert(fabsf(status.steer + 0.3f) < 1e-5f);
    /* Staying off-centre saturates the steer. */
    for (int period = 0; period < 20; ++period)
    {
        motion_apply(LINE_CORRECT_RIGHT);
    }
    assert(E_OK == motion_get_status(&status));
    assert(-STEERING_MAX == status.steer);
    assert(fabsf(status.left_target_rpm - 14.04f) < 1e-3f);
    assert(fabsf(status.right_target_rpm - 1.56f) < 1e-3f);
    /* After a full-strength turn the line may have slipped round beside the
     * robot: the turn is held for 12 more periods, then decays.
     */
    motion_apply(LINE_FORWARD);
    assert(E_OK == motion_get_status(&status));
    assert(-STEERING_MAX == status.steer);
    for (int period = 0; period < 12; ++period)
    {
        motion_apply(LINE_FORWARD);
    }
    assert(E_OK == motion_get_status(&status));
    assert(-STEERING_MAX == status.steer);
    motion_apply(LINE_FORWARD);
    assert(E_OK == motion_get_status(&status));
    assert(fabsf(status.steer + 0.68f) < 1e-5f);
    /* Uncertain stops at once, unramped, and clears steering. */
    motion_apply(LINE_UNCERTAIN);
    assert(E_OK == motion_get_status(&status));
    assert(0U == status.left_pwm && 0U == status.right_pwm);
    assert(0.0f == status.left_target_rpm && 0.0f == status.steer);
    /* The bounded bypass extension keeps Buddy 2's stop and ramp behavior. */
    motion_drive(100.0f, -1.0f);
    assert(E_OK == motion_get_status(&status));
    assert(8.0f == status.left_target_rpm && 0.0f == status.right_target_rpm);
    for (int i = 0; i < 8; ++i) { motion_drive(100.0f, 0.0f); }
    assert(E_OK == motion_get_status(&status));
    assert(15.0f == status.left_target_rpm && 0U == status.right_pwm);
    motion_drive(NAN, NAN);
    assert(E_OK == motion_get_status(&status));
    assert(0U == status.left_pwm && 0U == status.right_pwm);
    motion_apply(LINE_UNCERTAIN);
    /* Starting off-centre turns at once, scaled to the ramping speed. */
    motion_apply(LINE_CORRECT_LEFT);
    assert(E_OK == motion_get_status(&status));
    assert(fabsf(status.left_target_rpm - 4.592f) < 1e-3f);
    assert(fabsf(status.right_target_rpm - 8.528f) < 1e-3f);
    motion_apply(LINE_UNCERTAIN);
    /* A lone short touch ends its turn, but another on the same side soon
     * after is a bend: that turn is kept, decaying 0.1 x 0.85.
     */
    motion_apply(LINE_CORRECT_LEFT);
    motion_apply(LINE_FORWARD);
    assert(E_OK == motion_get_status(&status) && 0.0f == status.steer);
    motion_apply(LINE_FORWARD);
    motion_apply(LINE_FORWARD);
    motion_apply(LINE_CORRECT_LEFT);
    motion_apply(LINE_FORWARD);
    assert(E_OK == motion_get_status(&status));
    assert(fabsf(status.steer - 0.085f) < 1e-5f);
    motion_apply(LINE_UNCERTAIN);
    /* After a full-strength left turn, a swing past the line onto the right
     * sensor must not make the next short left touch count as a nudge: the
     * full turn resumes, rising 0.3 per period from 0.3.
     */
    for (int period = 0; period < 10; ++period)
    {
        motion_apply(LINE_CORRECT_LEFT);
    }
    motion_apply(LINE_FORWARD);
    motion_apply(LINE_CORRECT_RIGHT);
    motion_apply(LINE_FORWARD);
    motion_apply(LINE_CORRECT_LEFT);
    motion_apply(LINE_FORWARD);
    assert(E_OK == motion_get_status(&status));
    assert(fabsf(status.steer - 0.6f) < 1e-5f);
    /* Short touches do not extend the bend: well after the last long full
     * turn, a short left touch is a straight-line nudge again.
     */
    for (int period = 0; period < 30; ++period)
    {
        motion_apply(LINE_CORRECT_RIGHT);
        motion_apply(LINE_FORWARD);
    }
    motion_apply(LINE_CORRECT_LEFT);
    motion_apply(LINE_FORWARD);
    assert(E_OK == motion_get_status(&status));
    assert(0.0f == status.steer);
    motion_apply(LINE_UNCERTAIN);
    /* A long touch continuing a resumed full turn is still the bend: it
     * restarts the window, so a swing past the line long after the first
     * full turn, but soon after the long touch, is survived.
     */
    for (int period = 0; period < 10; ++period)
    {
        motion_apply(LINE_CORRECT_LEFT);
    }
    motion_apply(LINE_FORWARD);
    motion_apply(LINE_CORRECT_RIGHT);
    motion_apply(LINE_FORWARD);
    motion_apply(LINE_CORRECT_LEFT);
    motion_apply(LINE_FORWARD);
    for (int period = 0; period < 10; ++period)
    {
        motion_apply(LINE_CORRECT_LEFT);
    }
    for (int period = 0; period < 12; ++period)
    {
        motion_apply(LINE_FORWARD);
    }
    motion_apply(LINE_CORRECT_RIGHT);
    motion_apply(LINE_FORWARD);
    motion_apply(LINE_CORRECT_LEFT);
    motion_apply(LINE_FORWARD);
    assert(E_OK == motion_get_status(&status));
    assert(fabsf(status.steer - 0.6f) < 1e-5f);
    motion_apply(LINE_UNCERTAIN);
    quiet = 0;

    assert(LINE_FORWARD == line_follow_decide(0U, 0U));
    assert(LINE_CORRECT_LEFT == line_follow_decide(1U, 0U));
    assert(LINE_CORRECT_RIGHT == line_follow_decide(0U, 1U));
    assert(LINE_UNCERTAIN == line_follow_decide(1U, 1U));
    assert(LINE_UNCERTAIN == line_follow_decide(2U, 0U));

    ir_shared_snapshot_t snapshot = {0};
    assert(E_OBJ == ir_shared_get_latest(&snapshot));
    assert(E_OBJ == ir_shared_publish(0U, 0U, 0U));
    assert(E_PAR == ir_shared_get_latest(NULL));
    assert(E_OK == ir_shared_init());
    assert(E_OK == ir_shared_get_latest(&snapshot));
    assert(!snapshot.b_valid);
    assert(E_OK == ir_shared_publish(1U, 0U, 1U));
    assert(E_OK == ir_shared_get_latest(&snapshot));
    assert(snapshot.b_valid && snapshot.left == 1U && snapshot.right == 0U);
    assert(snapshot.barcode == 1U && snapshot.timestamp_msec == now_msec);
    assert(sizeof(snapshot) == 8U);
    assert(E_PAR == ir_shared_publish(2U, 0U, 0U));
    assert(E_OK == ir_shared_get_latest(&snapshot) && snapshot.left == 1U);
    fail_lock = 1;
    assert(-5 == ir_shared_get_latest(&snapshot));
    fail_lock = 0;
    fail_time = time_calls + 1;
    assert(-5 == ir_shared_publish(0U, 1U, 0U));
    fail_time = 0;
    assert(E_OK == ir_shared_get_latest(&snapshot));
    assert(snapshot.left == 1U && snapshot.right == 0U);

    /* A both-black sample is held after newer samples, until taken once. */
    bool b_junction = true;
    assert(E_PAR == ir_shared_take_junction(NULL));
    assert(E_OK == ir_shared_take_junction(&b_junction) && !b_junction);
    assert(E_OK == ir_shared_publish(1U, 1U, 0U));
    assert(E_OK == ir_shared_publish(0U, 0U, 0U));
    assert(E_OK == ir_shared_publish(1U, 0U, 0U));
    assert(E_OK == ir_shared_take_junction(&b_junction) && b_junction);
    assert(E_OK == ir_shared_take_junction(&b_junction) && !b_junction);
    fail_lock = 1;
    assert(-5 == ir_shared_take_junction(&b_junction));
    fail_lock = 0;

    /* A one-sided touch is held the same way, per side. */
    bool b_left = true;
    bool b_right = true;
    assert(E_PAR == ir_shared_take_sides(NULL, &b_right));
    assert(E_PAR == ir_shared_take_sides(&b_left, NULL));
    assert(E_OK == ir_shared_take_sides(&b_left, &b_right));
    assert(E_OK == ir_shared_take_sides(&b_left, &b_right));
    assert(!b_left && !b_right);
    assert(E_OK == ir_shared_publish(0U, 1U, 0U));
    assert(E_OK == ir_shared_publish(0U, 0U, 0U));
    assert(E_OK == ir_shared_take_sides(&b_left, &b_right));
    assert(!b_left && b_right);
    assert(E_OK == ir_shared_take_sides(&b_left, &b_right));
    assert(!b_left && !b_right);
    fail_lock = 1;
    assert(-5 == ir_shared_take_sides(&b_left, &b_right));
    fail_lock = 0;

    quiet = exhaustive = 1;
    for (uint32_t word = 0; word <= UINT16_MAX; ++word)
    {
        int16_t x, y, z;
        raw_word = (uint16_t)word;
        int32_t signed_word = word >= 32768U ? (int32_t)word - 65536
                                             : (int32_t)word;
        int16_t accel_expected = (int16_t)(signed_word >> 4);
        assert(lsm303_read_accel(20, &x, &y, &z));
        assert(x == accel_expected && y == accel_expected && z == accel_expected);
        uint32_t swapped = ((word & 255U) << 8) | (word >> 8);
        int16_t mag_expected = (int16_t)(swapped >= 32768U
            ? (int32_t)swapped - 65536 : (int32_t)swapped);
        assert(lsm303_read_mag(20, &x, &y, &z));
        assert(x == mag_expected && y == mag_expected && z == mag_expected);
    }
    quiet = exhaustive = 0;
    for (int axis = -2048; axis <= 2047; ++axis)
    {
        int16_t x = (int16_t)axis;
        int16_t y = (int16_t)(2047 - (axis + 2048));
        int16_t z = -1024;
        int32_t square = x * x + y * y + z * z;
        assert((int32_t)sqrt((double)square) == terrain_magnitude(x, y, z));
    }
    assert(3547 == terrain_magnitude(-2048, -2048, -2048));
    assert(5000 == terrain_jerk(1100, 1000, 20U));
    assert(-5000 == terrain_jerk(1000, 1100, 20U));
    assert(0 == terrain_jerk(1100, 1000, 0U));
    assert(-100000 == terrain_jerk(1100, 1000, UINT32_MAX));
    puts("UNIT_CHECKS_PASS: IR, mutex failures, all 65536 register words, "
         "4096 magnitude vectors, jerk boundaries");
}
#endif

int main(int argc, char **argv)
{
    assert(argc >= 2);
    char const *scenario = argv[1];
    variant = argc >= 3 ? atoi(argv[2]) : 0;
    if (strcmp(scenario, "startup") == 0)
    {
        if (variant == 1) { fail_mutex = 1; }
        if (variant >= 2 && variant <= 4) { fail_create = variant - 1; }
        if (variant >= 5 && variant <= 6) { fail_start = variant - 4; }
        if (variant >= 7) { fail_create = variant - 6; create_zero = 1; }
        printf("MAIN_RETURN %d\n", usermain());
    }
    else if (strcmp(scenario, "motor") == 0)
    {
        for (int action = -1; action <= 5; ++action)
        {
#if LEGACY
            RUN_MOTOR((LINE_ACTION)action);
#else
            RUN_MOTOR((line_follow_action_t)action);
#endif
        }
    }
    else if (strcmp(scenario, "controller") == 0)
    {
        mode = 1;
        if (variant == 5) { now_msec = UINT32_MAX - 100U; }
        if (variant >= 7) { delay_limit = 40; }
        ir_shared_init();
        if (variant != 4) { ir_shared_publish(0U, 0U, 1U); }
#if !LEGACY
        /* The controller waits for the IMU level reference; give it one. */
        terrain_init();
        terrain_calibrate(0, 0, 1000);
#endif
        if (setjmp(escape_task) == 0) { in_task = 1; RUN_CTRL(0, NULL); }
        puts("TASK_END");
    }
    else if (strcmp(scenario, "ir") == 0)
    {
        fail_pin = variant;
        ir_shared_init();
        if (setjmp(escape_task) == 0) { in_task = 1; RUN_IR(0, NULL); }
        puts("TASK_END");
    }
    else if (strcmp(scenario, "imu") == 0)
    {
        if (variant == 1) { fail_open = 1; }
        if (variant >= 2 && variant <= 6) { fail_write = variant - 1; }
        if (variant == 7) { fail_time = 1; }
        if (variant == 8) { fail_time = 3; }
        if (variant == 9) { short_write = 4; }
        if (setjmp(escape_task) == 0) { in_task = 1; RUN_IMU(0, NULL); }
        puts("TASK_END");
    }
#if !LEGACY
    else if (strcmp(scenario, "units") == 0) { run_units(); }
#endif
    else { return 2; }
    return 0;
}
