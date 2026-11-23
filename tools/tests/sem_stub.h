/* Test-only shim for task-owned barcode port semaphore APIs. */
#ifndef G7_V11_SEM_STUB_H
#define G7_V11_SEM_STUB_H
#include <tk/tkernel.h>
typedef struct { UW sematr; W isemcnt; W maxsem; } T_CSEM;
#ifndef TA_TPRI
#define TA_TPRI 1U
#endif
ID tk_cre_sem(T_CSEM const *cfg);
ER tk_wai_sem(ID sem, W count, TMO timeout);
ER tk_sig_sem(ID sem, W count);
#endif
