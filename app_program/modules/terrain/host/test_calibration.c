/* Host-only startup tests using the real IMU task and saved reference. */
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "app/tasks.h"
#include "drivers/lsm303.h"
#include "modules/terrain/task_imu.h"
#include "modules/terrain/terrain.h"

static const char *scenario;
static jmp_buf escape;
static unsigned reads, retries, closes, mutexes;
static int calibrated;
static int is(const char *name) { return strcmp(scenario, name) == 0; }

INT tm_printf(const UB *format, ...) { (void)format; return 0; }
ID tk_cre_mtx(const T_CMTX *config) { (void)config; return ++mutexes; }
ER tk_loc_mtx(ID id, TMO timeout) { (void)id; (void)timeout; return E_OK; }
ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
ER tk_del_mtx(ID id) { (void)id; return E_OK; }
ID tasks_create(tasks_entry_t entry, PRI priority)
{ (void)entry; (void)priority; return 3; }
ER tk_sta_tsk(ID id, INT code) { (void)id; (void)code; return E_OK; }
ID tk_opn_dev(UB *name, UINT mode) { (void)name; (void)mode; return 1; }
ER tk_cls_dev(ID id, UINT option)
{ (void)id; (void)option; ++closes; return E_OK; }
UW console_state(void) { return 3; }
bool lsm303_init(ID id) { (void)id; return true; }
ER tk_dly_tsk(UINT delay)
{
    if (delay == 500 && reads > 0)
    {
        assert(!terrain_is_calibrated());
        if (++retries == 3) { longjmp(escape, 1); }
    }
    return E_OK;
}
ER tk_get_otm(SYSTIM *time)
{
    (void)time;
    calibrated = terrain_is_calibrated();
    /* Successful startup reached normal sampling; stop before hardware I/O. */
    longjmp(escape, 1);
    return E_OK;
}
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
bool lsm303_read_mag(ID id, int16_t *x, int16_t *y, int16_t *z)
{ (void)id; (void)x; (void)y; (void)z; assert(0); return false; }

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
        float roll = 1, pitch = 1;
        assert(terrain_relative_tilt(10, 20, 1000, &roll, &pitch) == E_OK);
        assert(fabsf(roll) < 0.001f && fabsf(pitch) < 0.001f);
    }
    printf("PASS %s: reads=%u retries=%u\n", scenario, reads, retries);
    return 0;
}
