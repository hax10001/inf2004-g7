/* Host-only flag API supplement to the existing shared kernel stubs. */
#ifndef OBSTACLE_HOST_FLAGS_H
#define OBSTACLE_HOST_FLAGS_H
#include <tk/tkernel.h>
typedef struct { UW flgatr; UINT iflgptn; } T_CFLG;
typedef struct { UINT flgptn; } T_RFLG;
#define TA_TFIFO 0U
#define TA_WMUL 0U
ID tk_cre_flg(const T_CFLG *config);
ER tk_ref_flg(ID id, T_RFLG *out);
ER tk_set_flg(ID id, UINT bits);
ER tk_clr_flg(ID id, UINT keep);
#endif
