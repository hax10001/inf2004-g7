/** @file obs_state.c
 *
 * @brief Packed status word, event flags and plan hand-off.
 *
 * Every field of the status word has one writer at a time, and each update
 * is a read-modify-write protected by a binary semaphore, so packed fields
 * cannot race. A semaphore is used instead of a mutex because the kernel
 * configuration allows only a few mutexes.
 */
#include "obs_state.h"
#include <stdbool.h>

#define STATUS_PHASE_SHIFT 0U
#define STATUS_PHASE_MASK 0xFU
#define STATUS_ROUTE_SHIFT 4U
#define STATUS_ROUTE_MASK 0x3U
#define STATUS_FAULT_SHIFT 6U
#define STATUS_FAULT_MASK 0x1FU
#define STATUS_COUNT_SHIFT 11U
#define STATUS_COUNT_MASK 0xFU
#define STATUS_DIST_SHIFT 15U
#define STATUS_DIST_MASK 0x1FFU
#define STATUS_WIDTH_SHIFT 24U
#define STATUS_WIDTH_MASK 0xFFU
#define STATUS_DIST_LIMIT_MM 5109
#define STATUS_WIDTH_LIMIT_CM 254
#define MM_PER_CM 10

static ID gh_sem                       = 0;
static ID gh_flag                      = 0;
static uint32_t volatile g_status      = 0U;
static uint32_t volatile g_plan_word_a = 0U;
static uint32_t volatile g_plan_word_b = 0U;
static int32_t volatile g_watch_side   = 1;

/** @brief Take the lock; false if the lock does not exist yet. */
static bool lock_take (void);
/** @brief Release the lock taken by lock_take(). */
static void lock_give (void);
/** @brief Replace one field of the status word under the lock. */
static void status_update (uint32_t shift, uint32_t mask, uint32_t value);
/** @brief Read one field of the status word. */
static uint32_t field_get (uint32_t shift, uint32_t mask);

ER obs_state_init (void)
{
    T_CSEM sem_config  = {0};
    T_CFLG flag_config = {0};
    ER     error       = E_OK;

    sem_config.sematr  = TA_TFIFO | TA_FIRST;
    sem_config.isemcnt = 1;
    sem_config.maxsem  = 1;
    gh_sem             = tk_cre_sem(&sem_config);

    if (gh_sem <= E_OK)
    {
        error = (E_OK == gh_sem) ? E_OBJ : gh_sem;
    }
    else
    {
        flag_config.flgatr = TA_TFIFO | TA_WMUL;
        gh_flag            = tk_cre_flg(&flag_config);

        if (gh_flag <= E_OK)
        {
            error = (E_OK == gh_flag) ? E_OBJ : gh_flag;
        }
        else
        {
            g_status = obs_state_status_pack(OBS_PHASE_IDLE, OBS_ROUTE_NONE, 0U,
                                             0U, OBS_DIST_INVALID_CM,
                                             OBS_WIDTH_INVALID_CM);
        }
    }

    return error;
}

UINT obs_state_flags_read (void)
{
    T_RFLG info    = {0};
    UINT   pattern = 0U;

    if (tk_ref_flg(gh_flag, &info) >= E_OK)
    {
        pattern = info.flgptn;
    }

    return pattern;
}

void obs_state_flags_set (UINT bits)
{
    tk_set_flg(gh_flag, bits);
}

void obs_state_flags_clear (UINT bits)
{
    /* Clearing keeps the bits that are set in the argument. */
    tk_clr_flg(gh_flag, ~bits);
}

uint32_t obs_state_status_get (void)
{
    uint32_t word = 0U;

    if (lock_take())
    {
        word = g_status;
        lock_give();
    }

    return word;
}

uint32_t obs_state_status_pack (obs_state_phase_t phase,
                                obs_state_route_t route, uint32_t faults,
                                uint32_t count, uint32_t dist_cm,
                                uint32_t width_cm)
{
    uint32_t const phase_bits = phase;
    uint32_t const route_bits = route;

    return ((phase_bits & STATUS_PHASE_MASK) << STATUS_PHASE_SHIFT) |
           ((route_bits & STATUS_ROUTE_MASK) << STATUS_ROUTE_SHIFT) |
           ((faults & STATUS_FAULT_MASK) << STATUS_FAULT_SHIFT) |
           ((count & STATUS_COUNT_MASK) << STATUS_COUNT_SHIFT) |
           ((dist_cm & STATUS_DIST_MASK) << STATUS_DIST_SHIFT) |
           ((width_cm & STATUS_WIDTH_MASK) << STATUS_WIDTH_SHIFT);
}

obs_state_phase_t obs_state_status_phase (uint32_t word)
{
    return (word >> STATUS_PHASE_SHIFT) & STATUS_PHASE_MASK;
}

obs_state_route_t obs_state_status_route (uint32_t word)
{
    return (word >> STATUS_ROUTE_SHIFT) & STATUS_ROUTE_MASK;
}

uint32_t obs_state_status_faults (uint32_t word)
{
    return (word >> STATUS_FAULT_SHIFT) & STATUS_FAULT_MASK;
}

uint32_t obs_state_status_count (uint32_t word)
{
    return (word >> STATUS_COUNT_SHIFT) & STATUS_COUNT_MASK;
}

uint32_t obs_state_status_distance_cm (uint32_t word)
{
    return (word >> STATUS_DIST_SHIFT) & STATUS_DIST_MASK;
}

uint32_t obs_state_status_width_cm (uint32_t word)
{
    return (word >> STATUS_WIDTH_SHIFT) & STATUS_WIDTH_MASK;
}

void obs_state_set_phase (obs_state_phase_t phase)
{
    uint32_t const bits = phase;

    status_update(STATUS_PHASE_SHIFT, STATUS_PHASE_MASK, bits);
}

void obs_state_set_route (obs_state_route_t route)
{
    uint32_t const bits = route;

    status_update(STATUS_ROUTE_SHIFT, STATUS_ROUTE_MASK, bits);
}

void obs_state_set_distance_mm (int32_t range_mm)
{
    uint32_t centimetres = OBS_DIST_INVALID_CM;

    if ((range_mm >= 0) && (range_mm <= STATUS_DIST_LIMIT_MM))
    {
        /* The range is checked to be non-negative above. */
        centimetres = (uint32_t)(range_mm / MM_PER_CM);
    }

    status_update(STATUS_DIST_SHIFT, STATUS_DIST_MASK, centimetres);
}

void obs_state_set_width_cm (int32_t width_cm)
{
    uint32_t value = OBS_WIDTH_INVALID_CM;

    if ((width_cm >= 0) && (width_cm <= STATUS_WIDTH_LIMIT_CM))
    {
        /* The width is checked to be non-negative above. */
        value = (uint32_t)width_cm;
    }

    status_update(STATUS_WIDTH_SHIFT, STATUS_WIDTH_MASK, value);
}

void obs_state_count_bypass (void)
{
    uint32_t const count = field_get(STATUS_COUNT_SHIFT, STATUS_COUNT_MASK);

    if (count < STATUS_COUNT_MASK)
    {
        status_update(STATUS_COUNT_SHIFT, STATUS_COUNT_MASK, count + 1U);
    }
}

void obs_state_faults_update (uint32_t set_mask, uint32_t clear_mask)
{
    if (lock_take())
    {
        uint32_t faults = (g_status >> STATUS_FAULT_SHIFT) & STATUS_FAULT_MASK;

        faults   = (faults | set_mask) & ~clear_mask;
        g_status = (g_status & ~(STATUS_FAULT_MASK << STATUS_FAULT_SHIFT)) |
                   ((faults & STATUS_FAULT_MASK) << STATUS_FAULT_SHIFT);
        lock_give();
    }
}

void obs_state_plan_store (scan_plan_bypass_t const * p_plan)
{
    g_plan_word_a = p_plan->word_a;
    g_plan_word_b = p_plan->word_b;
}

void obs_state_plan_load (scan_plan_bypass_t * p_plan)
{
    p_plan->word_a = g_plan_word_a;
    p_plan->word_b = g_plan_word_b;
}

void obs_state_watch_side_set (int32_t side)
{
    g_watch_side = side;
}

int32_t obs_state_watch_side_get (void)
{
    return g_watch_side;
}

static bool lock_take (void)
{
    return (gh_sem > E_OK) && (tk_wai_sem(gh_sem, 1, TMO_FEVR) >= E_OK);
}

static void lock_give (void)
{
    tk_sig_sem(gh_sem, 1);
}

static void status_update (uint32_t shift, uint32_t mask, uint32_t value)
{
    if (lock_take())
    {
        g_status = (g_status & ~(mask << shift)) | ((value & mask) << shift);
        lock_give();
    }
}

static uint32_t field_get (uint32_t shift, uint32_t mask)
{
    return (obs_state_status_get() >> shift) & mask;
}

/*** end of file ***/

