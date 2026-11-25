/** @file test_mt_imu.c
 * @brief Host test (FR8 hump detection, FR17 lift detect, FR18 descent
 *        gate) on the real task_imu.c, terrain.c, tilt_math.c and
 *        lift_math.c, driven sample by sample through task_imu_task().
 *
 * The accelerometer, magnetometer, IR snapshot, encoders and kernel clock
 * are simulated; every 20 ms sample the published terrain snapshot is
 * checked. Covers the reported hump height (raw peak less
 * HUMP_HEIGHT_BIAS_MM, saturating at 127), the 3 mm climb gate on the
 * descending phase, travel and lift discards, the 3 s settle after being
 * put down, the jolt (shove) filter, heading, read failures, and the lift
 * flag against APP_LIFT_STOP_HEIGHT_MM and floor contact (fresh IR only).
 */
#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app/app_config.h"
#include "app/tasks.h"
#include "drivers/encoder.h"
#include "drivers/lsm303.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/terrain/task_imu.h"
#include "modules/terrain/terrain.h"
#include "platform/console.h"

/* IMU sample period, ms (task_imu.c IMU_SAMPLE_PERIOD_MSEC). */
#define T_PERIOD_MSEC 20U
/* Hump bias and settle time under test (task_imu.c). */
#define T_BIAS_MM 2U
#define T_SETTLE_MSEC 3000U
/* Log line buffer size. */
#define T_LINE_MAX 160U

/** @brief Simulated IR line-sensor states. */
typedef enum
{
    IR_FLOOR,   /**< Fresh sample, one sensor white: floor contact. */
    IR_AIR,     /**< Fresh sample, both black: no floor evidence. */
    IR_STALE,   /**< One sensor white, 1 ms past APP_IR_MAX_AGE_MSEC. */
    IR_EDGE,    /**< One sensor white, exactly APP_IR_MAX_AGE_MSEC old. */
    IR_INVALID, /**< Sample marked invalid. */
    IR_ERROR    /**< ir_shared_get_latest() fails. */
} ir_mode_t;

/** @brief A run of identical samples. */
typedef struct
{
    uint32_t  n;      /**< Samples in this phase. */
    int16_t   ax;     /**< Sensor X (roll axis), mg. */
    int16_t   ay;     /**< Sensor Y (pitch axis), mg. */
    int16_t   az;     /**< Sensor Z, mg. */
    int16_t   alt;    /**< Added to az on odd samples (keeps it moving). */
    int16_t   counts; /**< Encoder counts per wheel per sample. */
    ir_mode_t ir;
} phase_t;

/** @brief What one batch saw. */
typedef struct
{
    terrain_snapshot_t last;
    uint32_t samples;
    uint32_t live_max_mm;     /**< Highest live hump height. */
    bool     b_on_hump;       /**< Saw onHump. */
    bool     b_descending;    /**< Saw the descending phase. */
    uint32_t first_on_msec;   /**< First onHump time (0 = never). */
    bool     b_was_lifted;
    uint32_t landed_msec;     /**< First floor sample after a lift. */
    uint32_t lifted_count;
    uint32_t lifted_run;      /**< Current run of lifted samples. */
    uint32_t ended_run;       /**< Length of the last finished run. */
    uint32_t air_max_unlifted; /**< Highest no-floor height not lifted. */
    uint32_t min_lifted;       /**< Lowest height reported lifted. */
} stats_t;

static jmp_buf         g_escape;
static phase_t const * gp_phases;
static uint32_t        g_phase_count;
static uint32_t        g_index;
static uint32_t        g_target;
static stats_t         g_stats;
static phase_t         g_now_phase;
static uint32_t        g_now_msec = 100000U;
static int32_t         g_counts;
static bool            g_accel_ok = true;
static bool            g_mag_ok   = true;
static ID              g_open_result = 1;
static bool            g_init_ok     = true;
static bool            g_otm_fail;
static bool            g_expect_sleep;
static uint32_t        g_usb_polls_closed;
static uint32_t        g_usb_waits;
static uint32_t        g_sleeps;
static uint32_t        g_closes;
static uint32_t        g_accel_reads;
static int16_t         g_mag_x    = -100;
static int16_t         g_mag_y;
static char            g_done_line[T_LINE_MAX];
static uint32_t        g_discard_travel;
static uint32_t        g_discard_lifted;

/** @brief Host stub: capture HUMP DONE and DISCARDED lines. */
INT tm_printf (UB const * p_format, ...)
{
    char    line[T_LINE_MAX];
    va_list args;

    va_start(args, p_format);
    (void)vsnprintf(line, sizeof(line), (char const *)p_format, args);
    va_end(args);

    if (NULL != strstr(line, "HUMP DONE"))
    {
        (void)memcpy(g_done_line, line, sizeof(g_done_line));
    }

    if (NULL != strstr(line, "did not drive"))
    {
        g_discard_travel++;
    }

    if (NULL != strstr(line, "was lifted"))
    {
        g_discard_lifted++;
    }

    return 0;
}

/** @brief Host stub for tk_cre_mtx(). */
ID tk_cre_mtx (T_CMTX const * p_config)
{
    static ID next = 0;

    (void)p_config;
    next++;
    return next;
}

/** @brief Host stub for tk_loc_mtx(). */
ER tk_loc_mtx (ID mutex, TMO timeout)
{
    (void)mutex;
    (void)timeout;
    return E_OK;
}

/** @brief Host stub for tk_unl_mtx(). */
ER tk_unl_mtx (ID mutex)
{
    (void)mutex;
    return E_OK;
}

/** @brief Host stub for tk_del_mtx(). */
ER tk_del_mtx (ID mutex)
{
    (void)mutex;
    return E_OK;
}

/** @brief Host stub for tasks_create(). */
ID tasks_create (tasks_entry_t p_entry, PRI priority)
{
    (void)p_entry;
    (void)priority;
    return 3;
}

/** @brief Host stub for tk_sta_tsk(). */
ER tk_sta_tsk (ID task, INT code)
{
    (void)task;
    (void)code;
    return E_OK;
}

/** @brief Host stub for tk_opn_dev(): scripted handle. */
ID tk_opn_dev (UB * p_name, UINT mode)
{
    (void)p_name;
    (void)mode;
    return g_open_result;
}

/** @brief Host stub for tk_cls_dev(): count closes. */
ER tk_cls_dev (ID device, UINT option)
{
    (void)device;
    (void)option;
    g_closes++;
    return E_OK;
}

/** @brief Host stub: a task that gives up sleeps forever; only allowed
 *  when a start-up failure is being tested. */
ER tk_slp_tsk (TMO timeout)
{
    assert(TMO_FEVR == timeout);
    assert(g_expect_sleep);
    g_sleeps++;
    longjmp(g_escape, 1);
    return E_OK;
}

/** @brief Host stub: USB not enumerated for the first few polls. */
UW console_state (void)
{
    if (g_usb_polls_closed > 0U)
    {
        g_usb_polls_closed--;
        return CONSOLE_STATE_ENUMERATED - 1U;
    }

    return CONSOLE_STATE_OPEN;
}

/** @brief Host stub for lsm303_init(): scripted result. */
bool lsm303_init (ID device)
{
    (void)device;
    return g_init_ok;
}

/** @brief Host stub: kernel time is the simulated clock. */
ER tk_get_otm (SYSTIM * p_time)
{
    if (g_otm_fail)
    {
        g_otm_fail = false;
        return E_OBJ;
    }

    p_time->hi = 0;
    p_time->lo = g_now_msec;
    return E_OK;
}

/** @brief Host stub: both wheels advance the same distance forward. */
void encoder_read (encoder_wheel_t wheel, encoder_sample_t * p_sample)
{
    /* The left encoder is mounted mirrored on the robot. */
    p_sample->count     = (ENCODER_LEFT == wheel) ? -g_counts : g_counts;
    p_sample->edge_usec = 0U;
}

/** @brief Host stub: level before calibration, then the phase's axes. */
bool lsm303_read_accel (ID device, int16_t * p_x, int16_t * p_y,
                        int16_t * p_z)
{
    (void)device;
    g_accel_reads++;
    *p_y = g_now_phase.ay;

    if (!terrain_is_calibrated())
    {
        *p_x = 0;
        *p_y = 0;
        *p_z = 1000;
        return true;
    }

    g_counts += g_now_phase.counts;
    *p_x = g_now_phase.ax;
    *p_z = (int16_t)(g_now_phase.az +
                     (((g_index % 2U) != 0U) ? g_now_phase.alt : 0));
    return g_accel_ok;
}

/** @brief Host stub: magnetometer reading or failure. */
bool lsm303_read_mag (ID device, int16_t * p_x, int16_t * p_y,
                      int16_t * p_z)
{
    (void)device;
    *p_x = g_mag_x;
    *p_y = g_mag_y;
    *p_z = 0;
    return g_mag_ok;
}

/** @brief Host stub: the IR snapshot for the current phase. */
ER ir_shared_get_latest (ir_shared_snapshot_t * p_output)
{
    uint32_t age = 0U;

    switch (g_now_phase.ir)
    {
        case IR_ERROR:
            return E_OBJ;
        case IR_AIR:
            p_output->left  = 1U;
            p_output->right = 1U;
        break;
        case IR_STALE:
            age = APP_IR_MAX_AGE_MSEC + 1U;
            p_output->left  = 1U;
            p_output->right = 0U;
        break;
        case IR_EDGE:
            age = APP_IR_MAX_AGE_MSEC;
            p_output->left  = 1U;
            p_output->right = 0U;
        break;
        case IR_FLOOR:
        case IR_INVALID:
        default:
            p_output->left  = 1U;
            p_output->right = 0U;
        break;
    }

    p_output->barcode        = 0U;
    p_output->b_valid        = (IR_INVALID == g_now_phase.ir) ? 0U : 1U;
    p_output->timestamp_msec = g_now_msec - age;
    return E_OK;
}

/** @brief Select the phase that covers sample @p index. */
static void select_phase (uint32_t index)
{
    uint32_t start = 0U;

    for (uint32_t p = 0U; p < g_phase_count; p++)
    {
        if (index < (start + gp_phases[p].n))
        {
            g_now_phase = gp_phases[p];
            return;
        }

        start += gp_phases[p].n;
    }

    assert(false);
}

/** @brief Record one published snapshot into the batch statistics. */
static void observe (terrain_snapshot_t const * p_snap)
{
    stats_t * const p_st = &g_stats;

    p_st->last = *p_snap;
    p_st->samples++;

    if (p_snap->hump_height_mm > p_st->live_max_mm)
    {
        p_st->live_max_mm = p_snap->hump_height_mm;
    }

    if (0U != p_snap->onHump)
    {
        p_st->b_on_hump = true;

        if (0U == p_st->first_on_msec)
        {
            p_st->first_on_msec = p_snap->timestamp_msec;
        }

        if (TERRAIN_HUMP_DESCENDING == p_snap->hump_phase)
        {
            p_st->b_descending = true;
        }
    }

    /* FR17: lifted only at or above the limit with no floor contact. */
    if (0U != p_snap->b_lifted)
    {
        assert(p_snap->lift_height_mm >= APP_LIFT_STOP_HEIGHT_MM);
        assert(0U == p_snap->b_floor_contact);
        p_st->lifted_count++;
        p_st->lifted_run++;
        p_st->b_was_lifted = true;

        if ((0U == p_st->min_lifted) ||
            (p_snap->lift_height_mm < p_st->min_lifted))
        {
            p_st->min_lifted = p_snap->lift_height_mm;
        }
    }
    else
    {
        if (0U != p_st->lifted_run)
        {
            p_st->ended_run  = p_st->lifted_run;
            p_st->lifted_run = 0U;
        }

        /* ...and always lifted there (no missed stop). */
        assert((0U != p_snap->b_floor_contact) ||
               (p_snap->lift_height_mm < APP_LIFT_STOP_HEIGHT_MM));

        if ((0U == p_snap->b_floor_contact) &&
            (p_snap->lift_height_mm > p_st->air_max_unlifted))
        {
            p_st->air_max_unlifted = p_snap->lift_height_mm;
        }

        if (p_st->b_was_lifted && (0U == p_st->landed_msec) &&
            (0U != p_snap->b_floor_contact))
        {
            p_st->landed_msec = p_snap->timestamp_msec;
        }
    }
}

/** @brief Host stub: one sample has been processed at each 20 ms delay. */
ER tk_dly_tsk (UINT delay)
{
    terrain_snapshot_t snap = {0};

    if (APP_USB_WAIT_PERIOD_MSEC == delay)
    {
        g_usb_waits++;
    }

    if ((T_PERIOD_MSEC != delay) || !terrain_is_calibrated())
    {
        return E_OK; /* start-up and calibration waits */
    }

    assert(E_OK == terrain_get_snapshot(&snap));
    assert(snap.timestamp_msec == g_now_msec);
    observe(&snap);
    g_index++;

    if (g_index >= g_target)
    {
        longjmp(g_escape, 1);
    }

    g_now_msec += T_PERIOD_MSEC;
    select_phase(g_index);
    return E_OK;
}

/** @brief Run a phase table through the real IMU task. */
static void run (phase_t const * p_phases, uint32_t count)
{
    gp_phases     = p_phases;
    g_phase_count = count;
    g_target      = 0U;

    for (uint32_t p = 0U; p < count; p++)
    {
        g_target += p_phases[p].n;
    }

    g_index = 0U;
    (void)memset(&g_stats, 0, sizeof(g_stats));
    g_done_line[0] = '\0';
    g_now_msec += T_PERIOD_MSEC;
    select_phase(0U);

    if (0 == setjmp(g_escape))
    {
        task_imu_task(0, NULL);
    }

    assert(g_stats.samples == g_target);
}

/** @brief Value of "@p key=<int>" in the last HUMP DONE line. */
static int32_t done_value (char const * p_key)
{
    char const * p_at = strstr(g_done_line, p_key);
    int32_t      value = -1;

    assert(NULL != p_at);
    assert(1 == sscanf(p_at + strlen(p_key), "%" SCNd32, &value));
    return value;
}

/** @brief Start the task expecting it to give up and sleep forever. */
static void run_until_sleep (void)
{
    static phase_t const level[] = {{1000U, 0, 0, 1000, 0, 0, IR_FLOOR}};

    gp_phases      = level;
    g_phase_count  = 1U;
    g_target       = 1000U;
    g_index        = 0U;
    g_expect_sleep = true;
    g_sleeps       = 0U;
    select_phase(0U);

    if (0 == setjmp(g_escape))
    {
        task_imu_task(0, NULL);
    }

    g_expect_sleep = false;
    assert(1U == g_sleeps); /* never returned from the task function */
}

/** @brief Start-up failures park the task; USB wait; clock failure. */
static void test_startup_failures (void)
{
    g_open_result = E_OBJ;
    g_closes      = 0U;
    run_until_sleep();
    assert((0U == g_accel_reads) && (0U == g_closes));
    g_open_result = 1;
    g_init_ok     = false;
    run_until_sleep();
    assert((0U == g_accel_reads) && (1U == g_closes));
    g_init_ok = true;
    puts("PASS imu start-up: I2C open or sensor init failure parks the "
         "task (device closed), no sensor reads");

    /* USB not up for 3 polls; calibrates; then the clock fails. */
    g_usb_polls_closed = 3U;
    g_usb_waits        = 0U;
    g_closes           = 0U;
    g_otm_fail         = true;
    run_until_sleep();
    assert(terrain_is_calibrated());
    assert(50U == g_accel_reads); /* one calibration window, no samples */
    assert((3U + 1U) == g_usb_waits); /* 3 USB polls + sensor settle */
    assert(1U == g_closes);
    puts("PASS imu start-up: waits for USB, calibrates on 50 samples, "
         "parks if the kernel clock cannot be read");
}

/** @brief Level start-up: valid tilt, heading, floor, no hump. */
static void test_level_and_heading (void)
{
    static phase_t const level[] = {{30U, 0, 0, 1000, 0, 0, IR_FLOOR}};

    run(level, 1U);
    assert(terrain_is_calibrated());
    assert(g_stats.last.b_valid && g_stats.last.height_valid);
    assert(fabsf(g_stats.last.roll_deg) < 0.01f);
    assert(0U == g_stats.last.hump_height_mm && !g_stats.b_on_hump);
    assert(0U != g_stats.last.b_floor_contact);
    assert(0U == g_stats.last.b_lifted && 0U == g_stats.lifted_count);
    /* Magnetic field along +X (left) is a heading of 270 degrees. */
    assert(g_stats.last.heading_valid &&
           (27000U == g_stats.last.heading_centideg));
    puts("PASS imu level: calibrated, roll 0, no hump, floor contact, "
         "heading 270.00 deg");

    g_accel_ok = false;
    g_mag_ok   = false;
    run(level, 1U);
    g_accel_ok = true;
    g_mag_ok   = true;
    assert(!g_stats.last.b_valid && !g_stats.last.height_valid);
    assert(!g_stats.last.heading_valid);
    assert(0U == g_stats.last.hump_height_mm);
    g_mag_x = 0;
    g_mag_y = 0;
    run(level, 1U);
    assert(g_stats.last.b_valid && !g_stats.last.heading_valid);
    g_mag_x = -100;
    puts("PASS imu read failures: accel error -> invalid snapshot, mag "
         "error or zero field -> no heading");
}

/** @brief Nose straight down: roll undefined, snapshot invalid. */
static void test_invalid_tilt (void)
{
    static phase_t const down[] = {{60U, 0, 1000, 0, 0, 0, IR_FLOOR}};
    static phase_t const level[] = {{60U, 0, 0, 1000, 0, 0, IR_FLOOR}};

    run(down, 1U);
    assert(0U == g_stats.last.b_valid && 0U == g_stats.last.height_valid);
    assert(0U == g_stats.last.hump_height_mm && !g_stats.last.onHump);
    run(level, 1U);
    assert(g_stats.last.b_valid && (fabsf(g_stats.last.roll_deg) < 0.1f));
    puts("PASS imu invalid tilt: nose straight down (Y=Z=0) publishes an "
         "invalid snapshot, height 0; recovers when level");
}

/** @brief A jolt (|a| far from gravity) never moves the tilt estimate. */
static void test_shove (void)
{
    static phase_t const shove[] = {
        {20U, 0, 0, 1000, 0, 0, IR_FLOOR},
        {1U, 600, 0, 1000, 0, 0, IR_FLOOR}, /* 1166 mg: shoved */
    };
    static phase_t const tilt[] = {{1U, 93, 0, 996, 0, 0, IR_FLOOR}};

    run(shove, 2U);
    assert(fabsf(g_stats.last.roll_deg) < 0.05f);
    run(tilt, 1U);
    assert(g_stats.last.roll_deg > 0.5f); /* a real tilt does move it */
    puts("PASS imu jolt filter: 1166 mg sideways shove leaves roll 0, a "
         "1000 mg 5.3 deg tilt moves it");
}

/** @brief A real 5.3 degree hump with 6 mm climb: reported raw - 2. */
static void test_real_hump (void)
{
    static phase_t const hump[] = {
        {30U, 0, 0, 1000, 0, 0, IR_FLOOR},
        {30U, 93, 0, 996, 0, 0, IR_FLOOR},
        {30U, 93, 0, 996, 0, 10, IR_FLOOR},  /* 30 x 2.08 mm while tilted */
        {40U, 0, 0, 1000, 0, 0, IR_FLOOR},
    };
    uint32_t const toggle = g_stats.last.hump_done_toggle;

    run(hump, 4U);
    assert(g_stats.b_on_hump && !g_stats.last.onHump);
    assert(toggle != g_stats.last.hump_done_toggle);
    assert(10U == g_stats.live_max_mm); /* 110 * sin(5.3 deg) */
    assert((g_stats.live_max_mm - T_BIAS_MM) ==
           g_stats.last.last_hump_height_mm);
    assert(8U == g_stats.last.max_hump_height_mm);
    assert((done_value("peak_x10mm=") >= 80) &&
           (done_value("peak_x10mm=") <= 83));
    /* Climb over 3 mm: the descending phase (downhill cap) engaged. */
    assert(done_value("climb_x10mm=") >= 30);
    assert(g_stats.b_descending);
    uint32_t const reported = g_stats.last.last_hump_height_mm;

    printf("PASS imu hump: live peak %" PRIu32 " mm reported %" PRIu32
           " mm (bias %" PRIu32 "), climb %" PRId32 ".%" PRId32
           " mm reached DESCENDING\n", g_stats.live_max_mm, reported,
           (uint32_t)T_BIAS_MM, done_value("climb_x10mm=") / 10,
           done_value("climb_x10mm=") % 10);
}

/** @brief Smallest detectable hump: positive, raw - 2, no descent. */
static void test_small_hump (void)
{
    static phase_t const hump[] = {
        {15U, 58, 0, 998, 0, 0, IR_FLOOR},   /* about 3.3 deg */
        {45U, 58, 0, 998, 0, 5, IR_FLOOR},   /* 45 x 1.04 mm */
        {40U, 0, 0, 1000, 0, 0, IR_FLOOR},
    };
    uint32_t const toggle = g_stats.last.hump_done_toggle;

    run(hump, 3U);
    assert(g_stats.b_on_hump && (toggle != g_stats.last.hump_done_toggle));
    assert(6U == g_stats.live_max_mm);
    assert(4U == g_stats.last.last_hump_height_mm); /* not wrapped */
    assert(8U == g_stats.last.max_hump_height_mm);  /* highest kept */
    assert(done_value("climb_x10mm=") < 30);
    assert(!g_stats.b_descending); /* climb < 3 mm: no downhill cap */
    puts("PASS imu small hump: 3.3 deg (6 mm) reports 4 mm, max stays 8, "
         "climb < 3 mm never DESCENDING");
}

/** @brief Tilted without driving: discarded, nothing reported. */
static void test_no_travel (void)
{
    static phase_t const hump[] = {
        {40U, 93, 0, 996, 0, 0, IR_FLOOR},
        {40U, 0, 0, 1000, 0, 0, IR_FLOOR},
    };
    uint32_t const toggle = g_stats.last.hump_done_toggle;
    uint32_t const before = g_discard_travel;

    run(hump, 2U);
    assert(toggle == g_stats.last.hump_done_toggle);
    assert(g_stats.b_on_hump && !g_stats.last.onHump);
    assert((before + 1U) == g_discard_travel);
    assert(4U == g_stats.last.last_hump_height_mm);
    puts("PASS imu hump discard: 5.3 deg tilt with wheels still is not "
         "recorded");
}

/** @brief FR17: lifted flag against height and floor contact. */
static void test_lift_threshold (void)
{
    /* Profile A rises through 39 mm; profile B lands exactly on 40 mm. */
    static phase_t const lift_a[] = {
        {30U, 0, 0, 1000, 0, 0, IR_FLOOR},
        {3U, 0, 0, 1080, 0, 0, IR_AIR},       /* push: 80 mg */
        {40U, 0, 0, 1015, 30, 0, IR_AIR},     /* 15/45 mg: still rising */
        {10U, 0, 0, 1000, 0, 0, IR_FLOOR},    /* put down */
    };
    static phase_t const lift_b[] = {
        {30U, 0, 0, 1000, 0, 0, IR_FLOOR},
        {3U, 0, 0, 1080, 0, 0, IR_AIR},
        {40U, 0, 0, 1012, 22, 0, IR_AIR},     /* 12/34 mg */
        {10U, 0, 0, 1000, 0, 0, IR_FLOOR},
    };

    run(lift_a, 4U);
    assert(g_stats.lifted_count > 0U);
    /* Just under the limit with no floor contact: not lifted. */
    assert((APP_LIFT_STOP_HEIGHT_MM - 1U) == g_stats.air_max_unlifted);
    /* Back on the floor: not lifted at once, height zero. */
    assert(0U == g_stats.last.b_lifted);
    assert(0U != g_stats.last.b_floor_contact);
    assert(0U == g_stats.last.lift_height_mm);
    run(lift_b, 4U);
    /* Exactly at the limit with no floor contact: lifted. */
    assert(APP_LIFT_STOP_HEIGHT_MM == g_stats.min_lifted);
    assert(0U == g_stats.last.b_lifted);
    printf("PASS imu lift: %" PRIu32 " mm off the floor not lifted, %"
           PRIu32 " mm lifted (limit %" PRIu32 ", at or above), floor "
           "contact clears it at once\n",
           (uint32_t)(APP_LIFT_STOP_HEIGHT_MM - 1U), g_stats.min_lifted,
           (uint32_t)APP_LIFT_STOP_HEIGHT_MM);
}

/** @brief Floor evidence must be fresh, valid and not both black; then
 *  a hump right after being put down waits exactly 3 s. */
static void test_floor_evidence_and_settle (void)
{
    static phase_t const lift[] = {
        {30U, 0, 0, 1000, 0, 0, IR_FLOOR},
        {3U, 0, 0, 1080, 0, 0, IR_AIR},
        {35U, 0, 0, 1015, 30, 0, IR_AIR},     /* lifted by now */
        {3U, 0, 0, 1015, 30, 0, IR_STALE},
        {2U, 0, 0, 1015, 30, 0, IR_INVALID},
        {2U, 0, 0, 1015, 30, 0, IR_ERROR},
        {1U, 0, 0, 1015, 30, 0, IR_EDGE},     /* 200 ms old: still fresh */
    };
    static phase_t const land[] = {
        {170U, 93, 0, 996, 0, 10, IR_FLOOR},  /* hump straight away */
        {40U, 0, 0, 1000, 0, 0, IR_FLOOR},
    };

    run(lift, 7U);
    /* Stale, invalid and failed IR never counted as the floor: the seven
     * samples before the 200 ms one stayed lifted; that one landed it. */
    assert(g_stats.ended_run >= 7U);
    assert(0U != g_stats.last.b_floor_contact);
    assert(0U == g_stats.last.b_lifted);
    assert(g_stats.landed_msec == g_stats.last.timestamp_msec);
    uint32_t const landed = g_stats.landed_msec;

    run(land, 2U);
    assert(0U == g_stats.lifted_count);
    puts("PASS imu floor evidence: IR 201 ms old, invalid or unreadable "
         "keeps lifted; 200 ms old is floor");
    assert(g_stats.b_on_hump);
    assert((g_stats.first_on_msec - landed) == T_SETTLE_MSEC);
    puts("PASS imu settle: hump after being put down starts exactly 3000 "
         "ms after landing, not before");
}

/** @brief A hump that overlaps a lift is discarded. */
static void test_spoiled_hump (void)
{
    static phase_t const spoil[] = {
        {160U, 0, 0, 1000, 0, 0, IR_FLOOR},   /* > 3 s since landing */
        {30U, 93, 0, 996, 0, 10, IR_FLOOR},
        {3U, 93, 0, 1076, 0, 0, IR_AIR},
        {40U, 93, 0, 1011, 30, 0, IR_AIR},
        {40U, 0, 0, 1000, 0, 0, IR_FLOOR},
    };
    uint32_t const before = g_discard_lifted;
    uint32_t const toggle = g_stats.last.hump_done_toggle;

    run(spoil, 5U);
    assert(g_stats.b_on_hump && (g_stats.lifted_count > 0U));
    assert((before + 1U) == g_discard_lifted);
    assert(toggle == g_stats.last.hump_done_toggle);
    puts("PASS imu spoiled hump: lifted mid-hump -> discarded, no HUMP "
         "DONE toggle");
}

/** @brief A climb past 127 mm saturates, never wraps the 7-bit field. */
static void test_saturation (void)
{
    static phase_t const hump[] = {
        {160U, 0, 0, 1000, 0, 0, IR_FLOOR},
        {60U, 500, 0, 866, 0, 48, IR_FLOOR},  /* 30 deg, 10 mm a sample */
        {40U, 0, 0, 1000, 0, 0, IR_FLOOR},
    };

    run(hump, 3U);
    assert(done_value("climb_x10mm=") > 1290);
    assert(127U == g_stats.last.last_hump_height_mm);
    assert(127U == g_stats.last.max_hump_height_mm);
    assert(55U == g_stats.live_max_mm);
    puts("PASS imu hump saturation: 300 mm climb reports 127 mm (7-bit "
         "field), not a wrapped value");
}

/** @brief Run every IMU case in order (state carries between them). */
int main (void)
{
    assert(E_OK == terrain_init());
    test_startup_failures();
    test_level_and_heading();
    test_invalid_tilt();
    test_shove();
    test_real_hump();
    test_small_hump();
    test_no_travel();
    test_lift_threshold();
    test_floor_evidence_and_settle();
    test_spoiled_hump();
    test_saturation();
    return 0;
}

/*** end of file ***/

