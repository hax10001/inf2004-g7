/** @file encoder_read_stub.c
 * @brief Host stub: a stationary robot for tests that link task_imu.c
 *        (its hump climb check reads the wheel encoders).
 */
#include "drivers/encoder.h"

/** @brief Host stub of encoder_read(). */
void encoder_read(encoder_wheel_t wheel, encoder_sample_t *p_sample)
{
    (void)wheel;
    *p_sample = (encoder_sample_t){0};
}
/*** end of file ***/

