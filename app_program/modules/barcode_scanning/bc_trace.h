/** @file bc_trace.h
 * @brief Buffered raw barcode diagnostics owned by the barcode worker.
 */
#ifndef BC_TRACE_H
#define BC_TRACE_H

#include "bc_fastq.h"

/** @brief Record raw queued samples before filtering; never print here. */
void bc_trace_record (bc_fastq_sample_t const * p_sample);

/** @brief Print one frozen record per worker poll, never during recording. */
void bc_trace_poll (void);

#endif /* BC_TRACE_H */

/*** end of file ***/

