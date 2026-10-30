/** @file obs_state.c
 *
 * @brief Packed status word, event flags and plan hand-off.
 *
 * Every field of the status word has one writer at a time, and each update
 * is a protected read-modify-write, so packed fields cannot race.
 */
#include "obs_state.h"

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

static ID gh_mutex                     = 0;
static ID gh_flag                      = 0;
static uint32_t volatile g_status      = 0U;
static uint32_t volatile g_plan_word_a = 0U;
static uint32_t volatile g_plan_word_b = 0U;
static obs_guard_snapshot_t volatile g_guard = {0U, 0U};
static uint32_t volatile g_front = 0U;
static struct
{
    uint32_t b_left : 1;
} volatile g_watch = {1U};

/** @brief Replace one field of the status word under the mutex. */
static void status_update (uint32_t shift, uint32_t mask, uint32_t value);

void obs_state_front_mark (bool b_valid, uint32_t now_msec)
{
    if ((gh_mutex > E_OK) && (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        g_front = (now_msec & 0xFFFFFFU) |
                  (b_valid ? (1U << 24U) : 0U);
        tk_unl_mtx(gh_mutex);
    }
}

bool obs_state_front_ready (uint32_t now_msec, uint32_t max_age_msec)
{
    bool b_ready = false;

    if ((gh_mutex > E_OK) && (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        uint32_t const age =
            ((now_msec & 0xFFFFFFU) - (g_front & 0xFFFFFFU)) & 0xFFFFFFU;
        b_ready = (0U != (g_front & (1U << 24U))) && (age <= max_age_msec);

        if ((0U != (g_front & (1U << 24U))) && (age > max_age_msec))
        {
            /* Latch a dead sampler before its compact timestamp can wrap. */
            g_front &= ~(1U << 24U);
            g_status |= OBS_FAULT_TIMEOUT << STATUS_FAULT_SHIFT;
        }

        tk_unl_mtx(gh_mutex);
    }

    return b_ready;
}

uint32_t obs_state_guard_request (uint32_t now_msec)
{
    uint32_t epoch = 0U;

    if ((gh_mutex > E_OK) && (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        epoch = ((g_guard.clock & 0xFFU) % 255U) + 1U;
        g_guard.ranges = 0U;
        g_guard.clock = ((now_msec & 0xFFFFFFU) << 8U) | epoch;

        if (tk_unl_mtx(gh_mutex) < E_OK)
        {
            epoch = 0U;
        }
    }

    return epoch;
}

bool obs_state_guard_load (obs_guard_snapshot_t * p_snapshot)
{
    bool b_ok = false;

    if ((NULL != p_snapshot) && (gh_mutex > E_OK) &&
        (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        *p_snapshot = g_guard;
        b_ok = (tk_unl_mtx(gh_mutex) >= E_OK);
    }

    return b_ok;
}

bool obs_state_guard_sample (uint32_t epoch, uint32_t view, int32_t range_mm,
                             uint32_t reason)
{
    bool b_ok = false;

    if ((0U == reason) && ((range_mm < 20) || (range_mm > 4000)))
    {
        reason = OBS_GUARD_REASON_SENSOR;
    }

    if ((view < 3U) && (reason <= 3U) && (gh_mutex > E_OK) &&
        (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        if ((epoch != 0U) && (epoch == (g_guard.clock & 0xFFU)))
        {
            if ((0U != reason) && (0U == (g_guard.ranges >> 30U)))
            {
                g_guard.ranges |= reason << 30U;
            }
            else if ((range_mm >= 20) && (range_mm <= 4000))
            {
                uint32_t const shift = view * 9U;
                uint32_t const cm = (uint32_t)range_mm / 10U;
                g_guard.ranges =
                    (g_guard.ranges & ~(0x1FFU << shift)) | (cm << shift);
                g_guard.ranges |= 1U << (27U + view);
            }

            b_ok = true;
        }

        b_ok = (tk_unl_mtx(gh_mutex) >= E_OK) && b_ok;
    }

    return b_ok;
}

ER obs_state_init (void)
{
    T_CMTX mutex_config = {0};
    T_CFLG flag_config  = {0};
    ER     error        = E_OK;

    mutex_config.mtxatr = TA_INHERIT;
    gh_mutex            = tk_cre_mtx(&mutex_config);

    if (gh_mutex <= E_OK)
    {
        error = (E_OK == gh_mutex) ? E_OBJ : gh_mutex;
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
    else
    {
        /* Unknown flags must never authorise autonomous movement. */
        pattern = OBS_EVT_ABORT | OBS_EVT_BLOCKED;
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
    uint32_t word = obs_state_status_pack(
        OBS_PHASE_FAULT, OBS_ROUTE_NONE, OBS_FAULT_TIMEOUT, 0U,
        OBS_DIST_INVALID_CM, OBS_WIDTH_INVALID_CM);

    if ((gh_mutex > E_OK) && (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        word = g_status;
        tk_unl_mtx(gh_mutex);
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
    if ((gh_mutex > E_OK) && (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        uint32_t const count =
            (g_status >> STATUS_COUNT_SHIFT) & STATUS_COUNT_MASK;

        if (count < STATUS_COUNT_MASK)
        {
            g_status = (g_status & ~(STATUS_COUNT_MASK << STATUS_COUNT_SHIFT)) |
                       ((count + 1U) << STATUS_COUNT_SHIFT);
        }

        tk_unl_mtx(gh_mutex);
    }
}

void obs_state_faults_update (uint32_t set_mask, uint32_t clear_mask)
{
    if ((gh_mutex > E_OK) && (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        uint32_t faults = (g_status >> STATUS_FAULT_SHIFT) & STATUS_FAULT_MASK;

        faults   = (faults | set_mask) & ~clear_mask;
        g_status = (g_status & ~(STATUS_FAULT_MASK << STATUS_FAULT_SHIFT)) |
                   ((faults & STATUS_FAULT_MASK) << STATUS_FAULT_SHIFT);
        tk_unl_mtx(gh_mutex);
    }
}

bool obs_state_plan_store (scan_plan_bypass_t const * p_plan)
{
    bool b_ok = false;

    if ((NULL != p_plan) && (gh_mutex > E_OK) &&
        (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        g_plan_word_a = p_plan->word_a;
        g_plan_word_b = p_plan->word_b;
        b_ok = (tk_unl_mtx(gh_mutex) >= E_OK);
    }

    return b_ok;
}

bool obs_state_plan_load (scan_plan_bypass_t * p_plan)
{
    bool b_ok = false;

    if ((NULL != p_plan) && (gh_mutex > E_OK) &&
        (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        p_plan->word_a = g_plan_word_a;
        p_plan->word_b = g_plan_word_b;
        b_ok = (tk_unl_mtx(gh_mutex) >= E_OK);
    }

    return b_ok;
}

void obs_state_watch_side_set (int32_t side)
{
    if ((gh_mutex > E_OK) && (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        g_watch.b_left = (side >= 0) ? 1U : 0U;
        tk_unl_mtx(gh_mutex);
    }
}

int32_t obs_state_watch_side_get (void)
{
    int32_t side = 1;

    if ((gh_mutex > E_OK) && (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        side = (0U != g_watch.b_left) ? 1 : -1;
        tk_unl_mtx(gh_mutex);
    }

    return side;
}

static void status_update (uint32_t shift, uint32_t mask, uint32_t value)
{
    if ((gh_mutex > E_OK) && (tk_loc_mtx(gh_mutex, TMO_FEVR) >= E_OK))
    {
        g_status = (g_status & ~(mask << shift)) | ((value & mask) << shift);
        tk_unl_mtx(gh_mutex);
    }
}

/*** end of file ***/

