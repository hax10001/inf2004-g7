/** @file test_obs_state.c
 * @brief Host test of the real obs_state.c packed status word (FR9-FR11
 *        telemetry): field round trips, boundaries, saturation, field
 *        isolation, the lock discipline and use before init.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <tk/tkernel.h>

/* Kernel semaphore and event flag doubles that obs_state.c needs. */
#ifndef TA_TFIFO
#define TA_TFIFO 0x00U
#endif
#ifndef TA_FIRST
#define TA_FIRST 0x00U
#endif
#ifndef TA_WMUL
#define TA_WMUL 0x08U
#endif

/** @brief Semaphore creation parameters (T-Kernel layout). */
typedef struct
{
    UW sematr;
    W  isemcnt;
    W  maxsem;
} T_CSEM;

/** @brief Event flag creation parameters (T-Kernel layout). */
typedef struct
{
    UW   flgatr;
    UINT iflgptn;
} T_CFLG;

/** @brief Event flag state as returned by tk_ref_flg(). */
typedef struct
{
    UINT flgptn;
} T_RFLG;

/** @brief Create a semaphore (double). */
ID tk_cre_sem(T_CSEM const * p_config);
/** @brief Take a semaphore (double). */
ER tk_wai_sem(ID sem, W count, TMO timeout);
/** @brief Release a semaphore (double). */
ER tk_sig_sem(ID sem, W count);
/** @brief Create an event flag (double). */
ID tk_cre_flg(T_CFLG const * p_config);
/** @brief Read an event flag (double). */
ER tk_ref_flg(ID flag, T_RFLG * p_info);
/** @brief Set event flag bits (double). */
ER tk_set_flg(ID flag, UINT bits);
/** @brief Clear event flag bits: keep only the given pattern (double). */
ER tk_clr_flg(ID flag, UINT keep);

#include "modules/obstacle/obs_state.c"

#define SEM_ID 7
#define FLAG_ID 9

static ID       g_sem_ret  = SEM_ID;
static ID       g_flag_ret = FLAG_ID;
static ER       g_wai_ret  = E_OK;
static int32_t  g_held     = 0;
static uint32_t g_takes    = 0U;
static uint32_t g_gives    = 0U;
static UINT     g_pattern  = 0U;

/** @brief Create a semaphore: check the binary configuration. */
ID tk_cre_sem(T_CSEM const * p_config)
{
    assert((1 == p_config->isemcnt) && (1 == p_config->maxsem));
    return g_sem_ret;
}

/** @brief Take the semaphore: never re-entered, always wait forever. */
ER tk_wai_sem(ID sem, W count, TMO timeout)
{
    assert((SEM_ID == sem) && (1 == count) && (TMO_FEVR == timeout));
    g_takes++;

    if (g_wai_ret >= E_OK)
    {
        assert(0 == g_held);
        g_held = 1;
    }

    return g_wai_ret;
}

/** @brief Release the semaphore: only when it is held. */
ER tk_sig_sem(ID sem, W count)
{
    assert((SEM_ID == sem) && (1 == count) && (1 == g_held));
    g_held = 0;
    g_gives++;
    return E_OK;
}

/** @brief Create the event flag: multi-waiter attribute required. */
ID tk_cre_flg(T_CFLG const * p_config)
{
    assert(0U != (p_config->flgatr & TA_WMUL));
    return g_flag_ret;
}

/** @brief Read the event flag pattern. */
ER tk_ref_flg(ID flag, T_RFLG * p_info)
{
    ER error = E_OK;

    if (FLAG_ID == flag)
    {
        p_info->flgptn = g_pattern;
    }
    else
    {
        error = E_OBJ;
    }

    return error;
}

/** @brief Set event flag bits. */
ER tk_set_flg(ID flag, UINT bits)
{
    assert(FLAG_ID == flag);
    g_pattern |= bits;
    return E_OK;
}

/** @brief T-Kernel semantics: the pattern is ANDed with the argument. */
ER tk_clr_flg(ID flag, UINT keep)
{
    assert(FLAG_ID == flag);
    g_pattern &= keep;
    return E_OK;
}

/** @brief Read the current word and check the lock was released. */
static uint32_t word_now(void)
{
    uint32_t const word = obs_state_status_get();

    assert(0 == g_held);
    return word;
}

/** @brief Check every field of a word against expected values. */
static void expect(uint32_t word, uint32_t phase, uint32_t route,
                   uint32_t faults, uint32_t count, uint32_t dist,
                   uint32_t width)
{
    assert((uint32_t)obs_state_status_phase(word) == phase);
    assert((uint32_t)obs_state_status_route(word) == route);
    assert(obs_state_status_faults(word) == faults);
    assert(obs_state_status_count(word) == count);
    assert(obs_state_status_distance_cm(word) == dist);
    assert(obs_state_status_width_cm(word) == width);
}

/** @brief Use before init: no lock, so reads give 0 and writes are
 *  dropped without touching the kernel (a task that starts early must
 *  not corrupt the word or crash).
 */
static void test_before_init(void)
{
    obs_state_set_phase(OBS_PHASE_FAULT);
    obs_state_set_width_cm(100);
    obs_state_faults_update(OBS_FAULT_SERVO, 0U);
    obs_state_count_bypass();
    assert(0U == word_now());
    assert((0U == g_takes) && (0U == g_gives));
    puts("PASS status word before init: reads 0, writes dropped, no lock");
}

/** @brief Init failures are reported, never silently accepted. */
static void test_init_errors(void)
{
    g_sem_ret = 0;
    assert(E_OBJ == obs_state_init());
    g_sem_ret = E_PAR;
    assert(E_PAR == obs_state_init());
    g_sem_ret  = SEM_ID;
    g_flag_ret = 0;
    assert(E_OBJ == obs_state_init());
    g_flag_ret = E_OBJ;
    assert(E_OBJ == obs_state_init());
    g_flag_ret = FLAG_ID;
    puts("PASS init reports semaphore and event flag creation errors");
}

/** @brief After init: idle, no route, no faults, no count, invalid
 *  distance and width (nothing measured yet must not read as 0 cm).
 */
static void test_init_defaults(void)
{
    assert(E_OK == obs_state_init());
    expect(word_now(), OBS_PHASE_IDLE, OBS_ROUTE_NONE, 0U, 0U,
           OBS_DIST_INVALID_CM, OBS_WIDTH_INVALID_CM);
    puts("PASS init word: idle, no route, invalid distance and width");
}

/** @brief Every phase and route round-trips and leaves the rest alone. */
static void test_phase_route(void)
{
    uint32_t phase = 0U;
    uint32_t route = 0U;

    obs_state_set_width_cm(37);
    obs_state_set_distance_mm(1234);

    for (phase = OBS_PHASE_IDLE; phase <= OBS_PHASE_FAULT; phase++)
    {
        for (route = OBS_ROUTE_NONE; route <= OBS_ROUTE_RIGHT; route++)
        {
            obs_state_set_phase((obs_state_phase_t)phase);
            obs_state_set_route((obs_state_route_t)route);
            expect(word_now(), phase, route, 0U, 0U, 123U, 37U);
        }
    }

    puts("PASS all 14 phases x 3 routes round-trip without side effects");
}

/** @brief Distance: whole cm (truncated), 0..5109 mm valid, anything
 *  else (no echo -1, beyond the 9-bit field) reads invalid.
 */
static void test_distance(void)
{
    static int32_t const  mm[]   = {0, 9, 10, 19, 250, 4000, 5109,
                                    5110, 6000, -1, -4000};
    static uint32_t const cm[]   = {0U, 0U, 1U, 1U, 25U, 400U, 510U,
                                    OBS_DIST_INVALID_CM, OBS_DIST_INVALID_CM,
                                    OBS_DIST_INVALID_CM,
                                    OBS_DIST_INVALID_CM};
    uint32_t              index  = 0U;

    obs_state_set_phase(OBS_PHASE_VERIFY);
    obs_state_set_route(OBS_ROUTE_RIGHT);

    for (index = 0U; index < (sizeof(mm) / sizeof(mm[0])); index++)
    {
        obs_state_set_distance_mm(mm[index]);
        expect(word_now(), OBS_PHASE_VERIFY, OBS_ROUTE_RIGHT, 0U, 0U,
               cm[index], 37U);
    }

    puts("PASS distance 0..5109 mm to cm, -1/out of range -> invalid 511");
}

/** @brief Width: 0..254 cm valid; negative or >254 read invalid (255),
 *  never wrapped into a small plausible width.
 */
static void test_width(void)
{
    int32_t width = 0;

    for (width = -3; width <= 600; width++)
    {
        uint32_t const want = ((width >= 0) && (width <= 254))
                                  ? (uint32_t)width
                                  : OBS_WIDTH_INVALID_CM;

        obs_state_set_width_cm(width);
        expect(word_now(), OBS_PHASE_VERIFY, OBS_ROUTE_RIGHT, 0U, 0U,
               OBS_DIST_INVALID_CM, want);
    }

    obs_state_set_width_cm(INT32_MIN);
    assert(OBS_WIDTH_INVALID_CM ==
           obs_state_status_width_cm(word_now()));
    obs_state_set_width_cm(INT32_MAX);
    assert(OBS_WIDTH_INVALID_CM ==
           obs_state_status_width_cm(word_now()));
    puts("PASS width 0..254 cm kept, <0 and >254 -> invalid 255 (no wrap)");
}

/** @brief Bypass count saturates at 15 instead of wrapping to 0. */
static void test_count(void)
{
    uint32_t index = 0U;

    obs_state_set_width_cm(12);

    for (index = 1U; index <= 20U; index++)
    {
        uint32_t const want = (index < 15U) ? index : 15U;

        obs_state_count_bypass();
        expect(word_now(), OBS_PHASE_VERIFY, OBS_ROUTE_RIGHT, 0U, want,
               OBS_DIST_INVALID_CM, 12U);
    }

    puts("PASS bypass count 1..15 then saturates (no wrap to 0)");
}

/** @brief Fault bits set and clear independently; bits outside the
 *  5-bit field never leak into the count or route.
 */
static void test_faults(void)
{
    uint32_t bit = 0U;

    for (bit = OBS_FAULT_NO_ECHO; bit <= OBS_FAULT_TIMEOUT; bit <<= 1U)
    {
        obs_state_faults_update(bit, 0U);
        assert(0U != (obs_state_status_faults(word_now()) & bit));
    }

    expect(word_now(), OBS_PHASE_VERIFY, OBS_ROUTE_RIGHT, 0x1FU, 15U,
           OBS_DIST_INVALID_CM, 12U);
    obs_state_faults_update(0U, OBS_FAULT_NO_ROUTE | OBS_FAULT_SERVO);
    expect(word_now(), OBS_PHASE_VERIFY, OBS_ROUTE_RIGHT,
           OBS_FAULT_NO_ECHO | OBS_FAULT_LINE_LOST | OBS_FAULT_TIMEOUT, 15U,
           OBS_DIST_INVALID_CM, 12U);
    /* Set and clear the same bit in one call: clear wins. */
    obs_state_faults_update(OBS_FAULT_SERVO, OBS_FAULT_SERVO);
    assert(0U == (obs_state_status_faults(word_now()) & OBS_FAULT_SERVO));
    /* Out-of-field bits are masked off. */
    obs_state_faults_update(0xFFFFFFE0U, 0U);
    expect(word_now(), OBS_PHASE_VERIFY, OBS_ROUTE_RIGHT,
           OBS_FAULT_NO_ECHO | OBS_FAULT_LINE_LOST | OBS_FAULT_TIMEOUT, 15U,
           OBS_DIST_INVALID_CM, 12U);
    obs_state_faults_update(0U, 0xFFFFFFFFU);
    expect(word_now(), OBS_PHASE_VERIFY, OBS_ROUTE_RIGHT, 0U, 15U,
           OBS_DIST_INVALID_CM, 12U);
    puts("PASS fault bits set/clear independently, extra bits masked");
}

/** @brief pack(): every field at its maximum and at zero round-trips;
 *  oversized inputs are masked to their field only.
 */
static void test_pack(void)
{
    uint32_t const all = obs_state_status_pack(OBS_PHASE_FAULT,
                                               OBS_ROUTE_RIGHT, 0x1FU, 15U,
                                               511U, 255U);
    uint32_t const none = obs_state_status_pack(OBS_PHASE_IDLE,
                                                OBS_ROUTE_NONE, 0U, 0U, 0U,
                                                0U);
    uint32_t const big = obs_state_status_pack(OBS_PHASE_IDLE,
                                               OBS_ROUTE_NONE, 0xFFFFFFFFU,
                                               0U, 0U, 0U);

    expect(all, OBS_PHASE_FAULT, OBS_ROUTE_RIGHT, 0x1FU, 15U, 511U, 255U);
    expect(none, 0U, 0U, 0U, 0U, 0U, 0U);
    assert(0U == none);
    expect(big, 0U, 0U, 0x1FU, 0U, 0U, 0U);
    /* One field at a time at its maximum: nothing spills over. */
    expect(obs_state_status_pack(OBS_PHASE_IDLE, OBS_ROUTE_NONE, 0U, 0U,
                                 0xFFFFU, 0U),
           0U, 0U, 0U, 0U, 0x1FFU, 0U);
    expect(obs_state_status_pack(OBS_PHASE_IDLE, OBS_ROUTE_NONE, 0U, 0xFFU,
                                 0U, 0xFFFFU),
           0U, 0U, 0U, 15U, 0U, 255U);
    puts("PASS pack: fields at 0 and max round-trip, oversize masked");
}

/** @brief Each locked update takes and gives once; a failed take skips
 *  the write and the give (no unbalanced release).
 */
static void test_lock(void)
{
    uint32_t const before = word_now();
    uint32_t       takes  = g_takes;
    uint32_t       gives  = g_gives;

    obs_state_set_phase(OBS_PHASE_ALIGN);
    assert((g_takes == (takes + 1U)) && (g_gives == (gives + 1U)));
    g_wai_ret = E_TMOUT;
    takes     = g_takes;
    gives     = g_gives;
    obs_state_set_phase(OBS_PHASE_SCANNING);
    obs_state_faults_update(OBS_FAULT_NO_ECHO, 0U);
    assert(0U == obs_state_status_get());
    assert((g_takes == (takes + 3U)) && (g_gives == gives));
    g_wai_ret = E_OK;
    assert(OBS_PHASE_ALIGN == obs_state_status_phase(word_now()));
    assert((word_now() & ~0xFU) == (before & ~0xFU));
    puts("PASS lock: one take/give per update, failed take writes nothing");
}

/** @brief Event flags: clear removes only the named bits (the inverse
 *  pattern T-Kernel expects), set keeps the others.
 */
static void test_flags(void)
{
    g_pattern = 0U;
    obs_state_flags_set(OBS_EVT_BLOCKED | OBS_EVT_SCAN_REQ |
                        OBS_EVT_FRONT_WATCH);
    obs_state_flags_clear(OBS_EVT_SCAN_REQ);
    assert((OBS_EVT_BLOCKED | OBS_EVT_FRONT_WATCH) == obs_state_flags_read());
    obs_state_flags_clear(OBS_EVT_BLOCKED | OBS_EVT_FRONT_WATCH);
    assert(0U == obs_state_flags_read());
    puts("PASS flags: clear drops only the named bits");
}

/** @brief Plan and range hand-off: the plan round-trips, and every
 *  published ping bumps its sequence even when the range repeats (the
 *  sequencer counts pings by sequence, so a repeat must not be lost).
 */
static void test_handoff(void)
{
    scan_plan_bypass_t const in   = {0xA5A5A5A5U, 0x5A5A5A5AU};
    scan_plan_bypass_t       out  = {0U, 0U};
    uint32_t const           side = obs_state_side_seq_get();
    uint32_t const           front = obs_state_front_seq_get();

    obs_state_plan_store(&in);
    obs_state_plan_load(&out);
    assert((in.word_a == out.word_a) && (in.word_b == out.word_b));
    obs_state_watch_angle_set(-70);
    assert(-70 == obs_state_watch_angle_get());
    obs_state_side_range_set(300);
    obs_state_side_range_set(300);
    assert(((side + 2U) == obs_state_side_seq_get()) &&
           (300 == obs_state_side_range_get()));
    obs_state_front_range_set(-1);
    obs_state_front_range_set(-1);
    assert(((front + 2U) == obs_state_front_seq_get()) &&
           (-1 == obs_state_front_range_get()));
    puts("PASS plan/angle round-trip; repeated pings still bump the seq");
}

/** @brief Run every obs_state test. */
int main(void)
{
    test_before_init();
    test_init_errors();
    test_init_defaults();
    test_phase_route();
    test_distance();
    test_width();
    test_count();
    test_faults();
    test_pack();
    test_lock();
    test_flags();
    test_handoff();
    return 0;
}

/*** end of file ***/

