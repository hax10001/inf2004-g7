/** @file line_barcode.h
 * @brief IR subsystem lifecycle. Barcode decoding is not implemented yet.
 */
#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include <tk/tkernel.h>

/** @brief Create protected snapshot storage before any task starts. */
ER line_barcode_init (void);
/** @brief Create and start the IR sampling task. */
ER line_barcode_start (void);

#endif /* LINE_BARCODE_H */

/*** end of file ***/

