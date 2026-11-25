/** @file test_calibration.c
 * @brief Host test: IMU boot calibration on the real task_imu.c.
 */
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "app/tasks.h"
#include "drivers/lsm303.h"
#include "modules/terrain/task_imu.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/terrain/terrain.h"

static const char *scenario;
static jmp_buf escape;
static uint32_t reads;
static uint32_t retries;
static uint32_t closes;
static uint32_t mutexes;
static int32_t calibrated;
/** @brief Test helper: true if the current scenario is @p name. */
static int32_t is(const char *name) { return strcmp(scenario, name) == 0; }

/** @brief Host stub for tm_printf() (test double). */
INT tm_printf(const UB *format, ...) { (void)format; return 0; }
/** @brief Host stub for tk_cre_mtx() (test double). */
ID tk_cre_mtx(const T_CMTX *config) { (void)config; return ++mutexes; }
/** @brief Host stub for tk_loc_mtx() (test double). */
ER tk_loc_mtx(ID id, TMO timeout) { (void)id; (void)timeout; return E_OK; }
/** @brief Host stub for tk_unl_mtx() (test double). */
ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
/** @brief Host stub for tk_del_mtx() (test double). */
ER tk_del_mtx(ID id) { (void)id; return E_OK; }
/** @brief Host stub for tasks_create() (test double). */
ID tasks_create(tasks_entry_t entry, PRI priority)
{ (void)entry; (void)priority; return 3; }
/** @brief Host stub for tk_sta_tsk() (test double). */
ER tk_sta_tsk(ID id, INT code) { (void)id; (void)code; return E_OK; }
/** @brief Host stub for tk_opn_dev() (test double). */
ID tk_opn_dev(UB *name, UINT mode) { (void)name; (void)mode; return 1; }
/** @brief Host stub for tk_cls_dev() (test double). */
ER tk_cls_dev(ID id, UINT option)
{ (void)id; (void)option; ++closes; return E_OK; }
/* A task that gives up sleeps forever; end the scenario there instead. */
ER tk_slp_tsk(TMO timeout) { (void)timeout; longjmp(escape, 1); return E_OK; }
/** @brief Host stub for console_state() (test double). */
UW console_state(void) { return 3; }
/** @brief Host stub for ir_shared_get_latest(): no IR sample. */
ER ir_shared_get_latest(ir_shared_snapshot_t *out)
{ (void)out; return E_OBJ; }
/** @brief Host stub for lsm303_init() (test double). */
bool lsm303_init(ID id) { (void)id; return true; }
/** @brief Host stub for tk_dly_tsk() (test double). */
ER tk_dly_tsk(UINT delay)
{
    if (delay == 500 && reads > 0)
    {
        assert(!terrain_is_calibrated());
        if (++retries == 3) { longjmp(escape, 1); }
    }
    return E_OK;
}
/** @brief Host stub for tk_get_otm() (test double). */
ER tk_get_otm(SYSTIM *time)
{
    (void)time;
    calibrated = terrain_is_calibrated();
    /* Successful startup reached normal sampling; stop before hardware I/O. */
    longjmp(escape, 1);
    return E_OK;
}
/** @brief Host stub for lsm303_read_accel() (test double). */
bool lsm303_read_accel(ID id, int16_t *x, int16_t *y, int16_t *z)
{
    (void)id;
    assert(!terrain_is_calibrated());
    ++reads;
    *x = 10; *y = 20; *z = 1000;
    if (is("persistent")) { *z = 0; return false; }
    if (!is("steady") && reads <= 8)
    { *x = 0; *y = 0; }
    if (reads == 8)
    {
        if (is("read_failure")) { return false; }
        if (is("movement")) { *x = 100; }
        if (is("magnitude")) { *z = 500; }
    }
    return true;
}
/** @brief Host stub for lsm303_read_mag() (test double). */
bool lsm303_read_mag(ID id, int16_t *x, int16_t *y, int16_t *z)
{ (void)id; (void)x; (void)y; (void)z; assert(0); return false; }

/** @brief Test entry point: run the scenario named in argv[1]. */
int main(int argc, char **argv)
{
    assert(argc == 2);
    scenario = argv[1];
    assert(terrain_init() == E_OK);
    if (!setjmp(escape)) { task_imu_task(0, NULL); }
    assert(closes == 0);
    if (is("persistent"))
    {
        assert(!terrain_is_calibrated() && !calibrated);
        assert(retries == 3 && reads == 3);
    }
    else
    {
        assert(calibrated && terrain_is_calibrated());
        assert(retries == (is("steady") ? 0u : 1u));
        assert(reads == (is("steady") ? 50u : 58u));
        float roll = 1;
        float pitch = 1;
        assert(terrain_relative_tilt(10, 20, 1000, &roll, &pitch) == E_OK);
        assert(fabsf(roll) < 0.001f && fabsf(pitch) < 0.001f);
    }
    printf("PASS %s: reads=%u retries=%u\n", scenario, reads, retries);
    return 0;
}
/*** end of file ***/

