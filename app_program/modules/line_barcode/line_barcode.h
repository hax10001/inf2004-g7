/** @file line_barcode.h
 * @brief Buddy 3 task lifecycle, perception, and mission-event adapter.
 */
#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "bc_capture.h"

#define LBAR_EVENT_BARCODE 0x01U
#define LBAR_EVENT_JUNCTION 0x02U
#define LBAR_EVENT_FAULT 0x04U
#define LBAR_EVENT_REVERSE 0x08U
#define LBAR_EVENT_PENDING 0x10U
#define LBAR_EVENT_OVERFLOW 0x20U
#define LBAR_EVENT_REACQUIRED 0x40U

typedef struct
{
    uint32_t timestamp_msec;
    uint32_t sequence;
    uint8_t event_flags;
    uint8_t pending_letter;
    uint8_t line_state;
    uint8_t sensor_flags;
} lbar_status_t;

/** @brief Initialise mutexes and pure state. */
ER line_barcode_init (void);
/** @brief Start the existing IR sampler (no second task created). */
ER line_barcode_start (void);
/** @brief Configure actual measured tick widths before START and task start.
 * @note Until called, barcode decoding is disabled, not time-simulated.
 */
ER line_barcode_configure (bc_capture_config_t const * p_config,
                           uint8_t polarity_bits);
/** @brief Mission Coordinator START/STOP gate; default is inactive.
 * @note Must be called by Coordinator task, never an ISR.
 */
ER line_barcode_set_active (bool b_active);
/** @brief Buddy 5 task-level bounded recovery mode (not an ISR). */
ER line_barcode_set_recovery (bool b_recovering);
/** @brief Called by Buddy 3 IR sampler (not by an interrupt). */
ER line_barcode_process (uint8_t raw_bits);
/** @brief Publish a sticky sampling/sensor fault before task exit. */
ER line_barcode_report_fault (void);
/** @brief Safe status copy; events remain latched until acknowledged. */
ER line_barcode_get_status (lbar_status_t * p_output);
/** @brief Acknowledge detected events, without clearing pending action. */
ER line_barcode_ack (uint8_t event_flags);
/** @brief Clear pending data after STOP, abort, fault or performed turn.
 * @note Mission Coordinator must call this; current controller does not.
 */
ER line_barcode_cancel (void);

#endif /* LINE_BARCODE_H */

/*** end of file ***/

