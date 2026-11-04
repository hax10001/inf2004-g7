/** @file line_barcode.h
 * @brief Shared IR lifecycle; barcode scanning is owned by barcode_scanning.
 */
#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H
#include <tk/tkernel.h>
ER line_barcode_init (void);
ER line_barcode_start (void);
#endif /* LINE_BARCODE_H */

/*** end of file ***/

