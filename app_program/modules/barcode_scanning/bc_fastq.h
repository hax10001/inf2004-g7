/** @file bc_fastq.h
 * @brief SPSC queue for microsecond-timestamped barcode-only samples.
 */
#ifndef BC_FASTQ_H
#define BC_FASTQ_H

#include <stdint.h>

#define BC_FASTQ_CAPACITY 32U
#define BC_FAST_PERIOD_USEC 4000U
#define BC_FAST_MAX_LATE_USEC 1000U

typedef struct
{
    uint32_t time_usec;
    int32_t left_count;
    int32_t right_count;
    uint32_t left_edge_usec;
    uint32_t right_edge_usec;
    uint8_t gpio_level;
    uint8_t valid;
} bc_fastq_sample_t;

/** @brief Reset only before producer starts or after it stops. */
void bc_fastq_reset (void);

/** @brief Single ISR producer; never blocks, fails closed on overflow. */
uint8_t bc_fastq_push (bc_fastq_sample_t const * p_sample);

/** @brief Single task consumer; marks next sample invalid after overflow. */
uint8_t bc_fastq_pop (bc_fastq_sample_t * p_sample);

/** @brief Next deadline, preserving phase; late periods are invalidated. */
uint32_t bc_fastq_next (uint32_t deadline, uint32_t now_usec,
                        uint8_t * p_valid);

#endif /* BC_FASTQ_H */

/*** end of file ***/

