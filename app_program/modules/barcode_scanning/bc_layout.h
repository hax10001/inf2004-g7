/** @file bc_layout.h
 * @brief Wide-bar layout and blurred-model Code 39 decoder.
 *
 * Built for a sensor too high to resolve narrow bars on small prints. The
 * six wide bars of *X* give scale, offset and direction and narrow the
 * letter to the group sharing that wide-bar layout. Within the group a
 * blurred model of the whole symbol, fitted by least squares, ranks the
 * letters; the top two are then compared only where their models differ
 * (F/P, Y/R, Z backwards/G differ in one narrow detail) and a near tie is
 * refused. gap_permille is that local misfit gap.
 * Refuses (valid = 0) rather than guess when layouts fit about equally.
 *
 * Replay saved logs with tools/tests/barcode_layout.c (runs this file)
 * before changing thresholds here. tools/barcode_replay.py mirrors the
 * earlier whole-symbol decision only. Pure C: no kernel calls.
 */
#ifndef BC_LAYOUT_H
#define BC_LAYOUT_H

#include <stdint.h>

/** Distance measures tried: wheel centre, then at the barcode sensor. */
#define BC_LAYOUT_VARIANTS 2U

/** Refusal reasons (bc_layout_result_t.reason). */
#define BC_LAYOUT_OK 0U
#define BC_LAYOUT_FEW_BARS 1U       /**< Fewer than six candidate bars. */
#define BC_LAYOUT_NO_FIT 2U         /**< No wide-bar layout fits. */
#define BC_LAYOUT_AMBIGUOUS 3U      /**< Two layouts fit about equally. */
#define BC_LAYOUT_MODEL_FAIL 4U     /**< Blurred model could not be fitted. */
#define BC_LAYOUT_DISAGREE 5U       /**< Centre and sensor read differ. */
#define BC_LAYOUT_MANY_BARS 6U      /**< Too many dark segments. */

/** Read one frozen sample. units: L+R travel since sample 0; left_step:
 * left-wheel counts added since the previous sample (forward only);
 * signal: dark level above the local white floor, zero or more.
 */
typedef void (* bc_layout_read_t)(uint32_t index, uint32_t * p_units,
                                  uint32_t * p_left_step,
                                  int32_t * p_signal);

typedef struct
{
    uint8_t valid;
    uint8_t letter;        /**< 0..25 when valid. */
    uint8_t reverse;
    uint8_t variant;       /**< 0 centre, 1 sensor distance. */
    uint8_t reason;        /**< BC_LAYOUT_* refusal code. */
    uint8_t runner;        /**< Runner-up letter in the group, or 255. */
    uint8_t other;         /**< Disagree/ambiguous: other letter, or 255. */
    uint8_t bars;          /**< Candidate bars found (deciding variant). */
    uint16_t err_centi;    /**< Wide-bar fit error, modules x100. */
    uint16_t module_x10;   /**< Narrow module, L+R units x10. */
    uint16_t gap_permille; /**< Model misfit gap to the runner-up. */
} bc_layout_result_t;

/** @brief Start decoding count frozen samples (data must stay unchanged).
 * @param b_has_left Nonzero when left_step is meaningful; else only the
 *                   wheel-centre distance is tried.
 */
void bc_layout_begin (uint32_t count, bc_layout_read_t read_sample,
                       uint8_t b_has_left);

/** @brief Do one bounded step of work. @return Nonzero when finished. */
uint8_t bc_layout_poll (void);

/** @brief Result of the last finished decode. */
bc_layout_result_t bc_layout_result (void);

/** @brief Short text for a BC_LAYOUT_* reason code. */
char const * bc_layout_reason_text (uint8_t reason);

#endif /* BC_LAYOUT_H */

/*** end of file ***/

