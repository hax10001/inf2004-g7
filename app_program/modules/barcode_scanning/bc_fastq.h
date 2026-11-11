/** @file bc_fastq.h
 * @brief SPSC queue for microsecond-timestamped barcode-only samples.
 */
#ifndef BC_FASTQ_H
#define BC_FASTQ_H

#include <stdint.h>

/* 500 Hz capture; retain about 128 ms of queue headroom. */
#define BC_FASTQ_CAPACITY 64U
#define BC_FAST_PERIOD_USEC 2000U
#define BC_FAST_MAX_LATE_USEC 1000U

typedef struct
{
    uint32_t time_usec;
    int32_t left_count;
    int32_t right_count;
    uint32_t left_edge_usec;
    uint32_t right_edge_usec;
    /* One 32-bit packed word: 12-bit ADC plus three 1-bit flags.
     * This SPSC queue writes an entire record before publishing its
     * release-ordered index; the other core reads only after acquire.
     * The packed word is never concurrently modified by both cores. */
    unsigned int adc_raw:12;    /**< 0..4095 raw ADC sample. */
    unsigned int gpio_level:1; /**< Digital pin level, 0 or 1. */
    unsigned int valid:1;      /**< Capture interval valid. */
    unsigned int adc_valid:1;  /**< ADC conversion available. */
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

