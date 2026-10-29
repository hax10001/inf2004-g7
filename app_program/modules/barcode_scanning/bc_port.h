/** @file bc_port.h
 * @brief RTOS-protected barcode scanner using existing read-only encoders.
 */
#ifndef BC_PORT_H
#define BC_PORT_H

#include "bc_scan.h"
#include <stdint.h>
#include <tk/tkernel.h>

#define BC_PORT_LETTER_MASK 0x001FU
#define BC_PORT_VALID_FLAG 0x0020U
#define BC_PORT_COMPLETE_FLAG 0x0040U
#define BC_PORT_REVERSE_FLAG 0x0080U
#define BC_PORT_QUALITY_SHIFT 8U
#define BC_PORT_ERROR_SHIFT 10U
#define BC_PORT_ERROR_MASK 0x3C00U

/** 8-byte compact status, with reserved bits 14 and 15 kept zero.
 * Only letter values 0..25 are valid; 26..30 reserved, 31 invalid.
 */
typedef struct
{
    uint32_t timestamp_msec;
    uint16_t sequence;
    uint16_t info;
} bc_port_result_t;

/** No hard-coded wheel geometry or encoder polarity.
 * One travel unit equals one sum of two normalized encoder increments.
 */
typedef struct
{
    bc_scan_config_t scan;
    int8_t  left_forward_sign;   /**< Exactly +1 or -1. */
    int8_t  right_forward_sign;  /**< Exactly +1 or -1. */
    uint16_t max_wheel_step_delta; /**< Max left/right step mismatch. */
    uint16_t max_sample_travel_units; /**< Based on minimum element / 4. */
    uint16_t max_sample_interval_msec; /**< Measured scheduler jitter. */
    uint8_t black_gpio_level;     /**< 0 or 1, measured on target. */
} bc_port_config_t;

/** @brief Create mutex and set actual measured scan parameters.
 * @note Call after motion_init/encoder_init, before sampling begins.
 * @return E_OK for success, E_PAR for invalid calibration.
 */
ER bc_port_init (bc_port_config_t const * p_config);

/** @brief Legacy direct-feed API retained for host tests/integration.
 * Production 4 ms ISR captures into bc_fastq, never calls this API.
 * @return E_OK; E_OBJ when not configured; lock error otherwise.
 */
ER bc_port_sample (uint8_t raw_gpio_level, uint32_t now_msec);

/** @brief Decode completed captures in an existing lower-priority task.
 * The successful letter is held until ack/clear; no motor commands.
 */
/** @brief Process an ISR-captured coherent timestamp/encoder sample.
 * Called by the low-priority barcode task, never interrupt context.
 */
ER bc_port_sample_counts (uint8_t raw_gpio, uint32_t now_usec,
                          int32_t left, int32_t right,
                          uint32_t left_edge_usec,
                          uint32_t right_edge_usec, uint8_t b_valid);

ER bc_port_process (void);

/** @brief Copy result safely without consuming it.
 * @return E_OK if a result was copied, E_OBJ if none is pending.
 */
ER bc_port_peek (bc_port_result_t * p_output);

/** @brief Take one queued diagnostic independently of result ACK.
 * @return E_OBJ if no unprinted event is available.
 */
ER bc_port_diag_pop (bc_port_result_t * p_output);

/** @brief Acknowledge only the exact sequence the reader handled. */
ER bc_port_ack (uint16_t sequence);

/** @brief Cancel scanning and clear any pending result at STOP/FAULT. */
ER bc_port_clear (void);

#endif /* BC_PORT_H */

/*** end of file ***/

