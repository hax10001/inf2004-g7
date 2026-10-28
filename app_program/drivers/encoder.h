/** @file encoder.h
 * @brief Quadrature wheel encoder counts and edge timestamps.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
    ENCODER_LEFT = 0,
    ENCODER_RIGHT,
    ENCODER_WHEEL_COUNT
} encoder_wheel_t;

typedef struct
{
    int32_t  count;     /**< Signed channel-A edges since initialisation. */
    uint32_t edge_usec; /**< Microsecond timestamp of the latest edge. */
} encoder_sample_t;

/** @brief Claim the encoder pins, start the microsecond timer and install
 * the edge interrupt on the calling core.
 * @return E_OK, or the kernel error from defining the interrupt.
 */
ER encoder_init (void);
/** @brief Copy one wheel's count and latest edge time consistently.
 * @param wheel Wheel to read.
 * @param p_sample Receives the sample; unchanged for an invalid wheel.
 */
void encoder_read (encoder_wheel_t wheel, encoder_sample_t * p_sample);
/** @brief Read the free-running microsecond timer used for edge stamps. */
uint32_t encoder_now_usec (void);

#endif /* ENCODER_H */

/*** end of file ***/

