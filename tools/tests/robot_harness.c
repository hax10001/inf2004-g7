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
        if (variant != 1 && variant != 2 && variant != 3)
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

#if !LEGACY
static void run_units(void)
{
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
    fail_lock = 1;
    assert(-5 == ir_shared_get_latest(&snapshot));
    fail_lock = 0;
    fail_time = time_calls + 1;
    assert(-5 == ir_shared_publish(0U, 1U, 0U));
    fail_time = 0;
    assert(E_OK == ir_shared_get_latest(&snapshot));
    assert(snapshot.left == 1U && snapshot.right == 0U);

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
        ir_shared_init();
        if (variant != 4) { ir_shared_publish(0U, 0U, 1U); }
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
