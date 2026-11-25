/** @file line_barcode.h
 * @brief IR task lifecycle; barcode decoding lives in barcode_scanning.
 */
#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H
#include <tk/tkernel.h>
/** @brief Create the shared IR snapshot resources. */
ER line_barcode_init (void);
/** @brief Create and start the IR sampling task. */
ER line_barcode_start (void);
#endif /* LINE_BARCODE_H */

/*** end of file ***/

