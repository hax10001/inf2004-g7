/** @file terrain.c
 * @brief IMU lifecycle and integer arithmetic with the tested ARM results.
 */
#include "terrain.h"
#include "task_imu.h"
#include "app/app_config.h"
#include "app/tasks.h"

#define TERRAIN_ROOT_START_BIT (1U << 30U)
#define TERRAIN_MSEC_PER_SECOND 1000
#define TERRAIN_SIGNED_TIME_MAX 0x7FFFFFFFU

/** @brief Compute an integer square root without the floating-point library. */
static uint32_t integer_sqrt (uint32_t number);

ER terrain_init (void)
{
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

/*** end of file ***/

