/** @file bc_wave.h
 * @brief Experimental buffered analog Code 39 decoder and comparison logs.
 */
#ifndef BC_WAVE_H
#define BC_WAVE_H

#include "bc_fastq.h"

/** @brief Consume raw samples in the worker, never in the interrupt. */
void bc_wave_sample (bc_fastq_sample_t const * p_sample);

/** @brief Process a frozen buffer incrementally and publish unique matches. */
void bc_wave_poll (void);

/** @brief Diagnostic motor hold; always clear in live-control builds. */
uint8_t bc_wave_motion_hold (void);
/** Reports whether a completed capture is safe to rearm. */
uint8_t bc_wave_can_rearm (void);
/** Armed with a learned white reference and no pending reset. */
uint8_t bc_wave_is_ready (void);
/** Live scan window, used to lower wheel target temporarily. */
uint8_t bc_wave_capture_active (void);
/** Short scan/junction suppression window; bounded in navigation core. */
uint8_t bc_wave_navigation_guard_active(void);
void bc_wave_request_rearm (void);
/* Worker-owned cancellation; preserves the learned white reference. */
void bc_wave_request_cancel(void);
/** @brief While nonzero, no new capture starts (e.g. obstacle bypass). A
 *  capture already running finishes normally.
 */
void bc_wave_set_suppressed (uint8_t b_suppressed);

/** Scanner states reported over MQTT (bc_wave_status_t.state). */
#define BC_WAVE_STATE_OFF 0U
#define BC_WAVE_STATE_LEARNING 1U   /**< Learning white; keep on white. */
#define BC_WAVE_STATE_READY 2U      /**< Armed: waiting for a barcode. */
#define BC_WAVE_STATE_CAPTURE 3U    /**< Recording a pass. */
#define BC_WAVE_STATE_DECODING 4U   /**< Processing a recorded pass. */
#define BC_WAVE_STATE_CLEARING 5U   /**< Waiting for white after a pass. */
#define BC_WAVE_STATE_NOT_WHITE 6U  /**< Learned "white" was dark. */

/** Last-result reason when not a layout decoder code: not barcode-like. */
#define BC_WAVE_REASON_IGNORED 7U

typedef struct
{
    /* One barcode task constructs this diagnostic copy. Seven scanner
     * states and eight reasons require 3 bits each; letters + invalid
     * sentinel 31 require 5 bits. Do not serialize this native layout. */
    unsigned int state:3;        /**< BC_WAVE_STATE_*: 0..6. */
    unsigned int last_valid:1;
    unsigned int last_letter:5;  /**< 0..25, or 31 when none. */
    unsigned int last_reason:3;  /**< BC_LAYOUT_* or IGNORED (7). */
    uint8_t white_rejects;
    /* Raw ADC readings have 12-bit range; the diagnostic unit is a copy. */
    unsigned int white_mean:12;
    unsigned int white_sigma:12;
    uint16_t sequence;          /**< Last finished pass number. */
    uint16_t last_process_ms;
} bc_wave_status_t;

/** @brief Snapshot for telemetry. Fields are copied without a lock; they
 *  are diagnostics and may mix values from adjacent passes. */
void bc_wave_status (bc_wave_status_t * p_status);

/** @brief Ask the scanner to drop any capture and learn white again. Safe
 *  from any task; acted on by the barcode task once no decode is running.
 */
void bc_wave_request_relearn (void);

/** @brief Nonzero while a recorded pass is being decoded (no result yet).
 *  Navigation holds at a junction for it instead of faulting. */
uint8_t bc_wave_decode_pending (void);

#endif /* BC_WAVE_H */

/*** end of file ***/

