/* Barcode-only additions to the existing host RTOS doubles.
 * Firmware uses the real kernel headers, never this file.
 */
#ifndef BARCODE_KERNEL_H
#define BARCODE_KERNEL_H
#include <tk/tkernel.h>
typedef struct {
    UW sematr;
    INT isemcnt;
    INT maxsem;
} T_CSEM;
#define TA_TPRI 1U
ID tk_cre_sem(T_CSEM const *config);
ER tk_wai_sem(ID semaphore, INT count, TMO timeout);
ER tk_sig_sem(ID semaphore, INT count);
#endif
