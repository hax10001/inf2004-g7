/** @file terrain.c
 * @brief IMU lifecycle, saved level reference, and motion calculations.
 */
#include "terrain.h"
#include "task_imu.h"
#include "tilt_math.h"
#include "app/app_config.h"
#include "app/tasks.h"

#define TERRAIN_ROOT_START_BIT (1U << 30U)
#define TERRAIN_MSEC_PER_SECOND 1000
#define TERRAIN_SIGNED_TIME_MAX 0x7FFFFFFFU
#define TERRAIN_LEVEL_TOLERANCE_DEG 5.0F

/* Initialise before tasks start; protect all subsequent shared access. */
static ID volatile gh_calibration_mutex           = 0;
static ID volatile gh_snapshot_mutex              = 0;
static terrain_calibration volatile g_calibration = {0};
static terrain_snapshot volatile g_snapshot       = {0};

/** @brief Compute an integer square root without the floating-point library. */
static uint32_t integer_sqrt (uint32_t number);
/** @brief Wrap an angle difference into -180..180 degrees. */
static float angle_difference (float current_deg, float reference_deg);

ER terrain_init (void)
{
    T_CMTX mutex_config = {0};
    mutex_config.mtxatr = TA_INHERIT;

    if ((gh_calibration_mutex > E_OK) || (gh_snapshot_mutex > E_OK))
    {
        return E_OBJ;
    }

    // Create calibration mutex
    ID h_calibration = tk_cre_mtx(&mutex_config);

    if (h_calibration <= E_OK)
    {
        return (h_calibration == E_OK) ? E_OBJ : h_calibration;
    }

    // Create snapshot mutex
    ID h_snapshot = tk_cre_mtx(&mutex_config);

    if (h_snapshot <= E_OK)
    {
        (void)tk_del_mtx(h_calibration);

        return (h_snapshot == E_OK) ? E_OBJ : h_snapshot;
    }

    gh_calibration_mutex = h_calibration;
    gh_snapshot_mutex    = h_snapshot;

    g_calibration = (terrain_calibration){0.0F, 0.0F, false};

    g_snapshot = (terrain_snapshot){0};

    return E_OK;
}

ER terrain_start (void)
{
    ID h_task = tasks_create(task_imu_task, APP_IMU_PRIORITY);

    if (h_task <= E_OK)
    {
        /* Keep the original rejection of a zero task handle. */
        return (E_OK == h_task) ? E_OBJ : h_task;
    }

#if APP_IMU_ENABLED
    return tk_sta_tsk(h_task, 0);
#else
    return E_OK;
#endif
}

ER terrain_tilt (int16_t axis_x, int16_t axis_y, int16_t axis_z,
                 float * p_roll_deg, float * p_pitch_deg)
{
    // Remember that the x,y,z needs to be remap
    // robotX = -Y
    // robotY = X
    // robotZ = Z
    if (!tilt_math_angles(-(axis_y), axis_x, axis_z, p_roll_deg, p_pitch_deg))
    {
        return E_PAR;
    }

    return E_OK;
}

ER terrain_calibrate (int16_t axis_x, int16_t axis_y, int16_t axis_z)
{
    terrain_calibration reference = {0.0F, 0.0F, false};
    // Uses pointers to avoid unnecessary copies of the struct.
    ER error = terrain_tilt(axis_x, axis_y, axis_z, &reference.roll_zero_deg,
                            &reference.pitch_zero_deg);

    if (error < E_OK)
    {
        return error;
    }

    if (gh_calibration_mutex <= E_OK)
    {
        return E_OBJ;
    }

    error = tk_loc_mtx(gh_calibration_mutex, TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    if (g_calibration.b_valid)
    {
        (void)tk_unl_mtx(gh_calibration_mutex);
        return E_OBJ;
    }

    reference.b_valid = true;
    g_calibration     = reference;
    return tk_unl_mtx(gh_calibration_mutex);
}

bool terrain_is_calibrated (void)
{
    if (gh_calibration_mutex <= E_OK)
    {
        return false;
    }

    if (tk_loc_mtx(gh_calibration_mutex, TMO_FEVR) < E_OK)
    {
        return false;
    }

    bool b_valid = g_calibration.b_valid;
    ER   error   = tk_unl_mtx(gh_calibration_mutex);
    return (error >= E_OK) && b_valid;
}

float terrain_hump_height (float roll_deg)
{
    /* The hump is under one wheel, so the robot tips sideways about the
     * other: roll alone measures it, and pitch from speeding up or slowing
     * down does not leak in.
     */
    return tilt_math_hump_height(roll_deg, (float)WHEEL_SPACING_MM);
}

bool terrain_is_level (terrain_snapshot const * p_snapshot)
{
    if ((NULL == p_snapshot) || (!p_snapshot->b_valid))
    {
        return false;
    }

    return (p_snapshot->roll_deg >= -TERRAIN_LEVEL_TOLERANCE_DEG) &&
           (p_snapshot->roll_deg <= TERRAIN_LEVEL_TOLERANCE_DEG) &&
           (p_snapshot->pitch_deg >= -TERRAIN_LEVEL_TOLERANCE_DEG) &&
           (p_snapshot->pitch_deg <= TERRAIN_LEVEL_TOLERANCE_DEG);
}

ER terrain_relative_tilt (int16_t axis_x, int16_t axis_y, int16_t axis_z,
                          float * p_roll_deg, float * p_pitch_deg)
{
    float roll_deg  = 0.0F;
    float pitch_deg = 0.0F;

    if ((NULL == p_roll_deg) || (NULL == p_pitch_deg))
    {
        return E_PAR;
    }

    ER error = terrain_tilt(axis_x, axis_y, axis_z, &roll_deg, &pitch_deg);

    if (error < E_OK)
    {
        return error;
    }

    if (gh_calibration_mutex <= E_OK)
    {
        return E_OBJ;
    }

    error = tk_loc_mtx(gh_calibration_mutex, TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    terrain_calibration reference = g_calibration;
    error                         = tk_unl_mtx(gh_calibration_mutex);

    if (error < E_OK)
    {
        return error;
    }

    if (!reference.b_valid)
    {
        return E_OBJ;
    }

    *p_roll_deg  = angle_difference(roll_deg, reference.roll_zero_deg);
    *p_pitch_deg = angle_difference(pitch_deg, reference.pitch_zero_deg);
    return E_OK;
}

int32_t terrain_magnitude (int16_t axis_x, int16_t axis_y, int16_t axis_z)
{
    int32_t axis_wide_x = axis_x;
    int32_t axis_wide_y = axis_y;
    int32_t axis_wide_z = axis_z;

    /* Each 12-bit axis is -2048..2047; the sum is 0..12582912. */
    uint32_t sum_squared =
        (uint32_t)((axis_wide_x * axis_wide_x) + (axis_wide_y * axis_wide_y) +
                   (axis_wide_z * axis_wide_z));

    /* The square root is at most 3547 and fits int32_t. */
    return (int32_t)integer_sqrt(sum_squared);
}

int32_t terrain_jerk (int32_t magnitude, int32_t previous,
                      uint32_t elapsed_msec)
{
    int32_t elapsed_signed = 0;

    if (0U == elapsed_msec)
    {
        return 0;
    }

    /* Both branches use representable casts. Preserve the original ARM
     * two's-complement conversion even when the uint32_t high bit is set.
     */
    if (elapsed_msec <= TERRAIN_SIGNED_TIME_MAX)
    {
        elapsed_signed = (int32_t)elapsed_msec;
    }
    else
    {
        elapsed_signed = -(int32_t)(UINT32_MAX - elapsed_msec) - 1;
    }

    /* 12-bit magnitudes bound the signed numerator to +/-3547000. */
    return ((magnitude - previous) * TERRAIN_MSEC_PER_SECOND) / elapsed_signed;
}

static float angle_difference (float current_deg, float reference_deg)
{
    float difference = current_deg - reference_deg;

    if (difference > 180.0F)
    {
        difference -= 360.0F;
    }
    else if (difference < -180.0F)
    {
        difference += 360.0F;
    }
    else
    {
        /* The difference is already within one half-turn. */
    }

    return difference;
}

static uint32_t integer_sqrt (uint32_t number)
{
    uint32_t result   = 0U;
    uint32_t root_bit = TERRAIN_ROOT_START_BIT;

    while (root_bit > number)
    {
        root_bit >>= 2U;
    }

    while (0U != root_bit)
    {
        if (number >= (result + root_bit))
        {
            number -= result + root_bit;
            result = (result >> 1U) + root_bit;
        }
        else
        {
            result >>= 1U;
        }

        root_bit >>= 2U;
    }

    return result;
}

ER terrain_publish_snapshot (terrain_snapshot const * p_snapshot)
{
    if (NULL == p_snapshot)
    {
        return E_PAR;
    }

    if (gh_snapshot_mutex <= E_OK)
    {
        return E_OBJ;
    }

    ER error = tk_loc_mtx(gh_snapshot_mutex, TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    // Critical section: copy complete snapshot
    g_snapshot = *p_snapshot;

    return tk_unl_mtx(gh_snapshot_mutex);
}

ER terrain_get_snapshot (terrain_snapshot * p_snapshot)
{
    if (NULL == p_snapshot)
    {
        return E_PAR;
    }

    if (gh_snapshot_mutex <= E_OK)
    {
        return E_OBJ;
    }

    ER error = tk_loc_mtx(gh_snapshot_mutex, TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    // Critical section: copy shared snapshot
    *p_snapshot = g_snapshot;

    return tk_unl_mtx(gh_snapshot_mutex);
}

/*** end of file ***/

