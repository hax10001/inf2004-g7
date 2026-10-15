/* Host-only micro T-Kernel test doubles. Never included by the robot build. */
#ifndef HOST_TKERNEL_H
#define HOST_TKERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef int32_t INT;
typedef int32_t ER;
typedef int32_t ID;
typedef int32_t PRI;
typedef int32_t W;
typedef int32_t SZ;
typedef int32_t TMO;
typedef uint32_t UINT;
typedef uint32_t UW;
typedef uint8_t UB;
typedef unsigned int BOOL;
typedef void (*FP)();
typedef struct { W hi; UW lo; } SYSTIM;
typedef struct {
    void (*task)(INT, void *);
    PRI itskpri;
    SZ stksz;
    UW tskatr;
} T_CTSK;
typedef struct { UW mtxatr; } T_CMTX;
#define EXPORT
#define IMPORT extern
#define LOCAL static
#define Inline static inline
#define TA_HLNG 1U
#define TA_RNG3 0x300U
#define TA_INHERIT 1U
#define TRUE 1
#define FALSE 0
#define E_OK 0
#define E_PAR (-17)
#define E_OBJ (-41)
#define TMO_FEVR (-1)
#define TD_UPDATE 3U
#define BOARD_LED_PIN 16U
ID tk_cre_tsk(const T_CTSK *config);
ER tk_sta_tsk(ID task, INT code);
ER tk_dly_tsk(UINT delay);
ER tk_slp_tsk(TMO timeout);
ID tk_cre_mtx(const T_CMTX *config);
ER tk_loc_mtx(ID mutex, TMO timeout);
ER tk_unl_mtx(ID mutex);
ER tk_get_otm(SYSTIM *time);
ID tk_opn_dev(UB *name, UINT mode);
ER tk_cls_dev(ID device, UINT option);
ER tk_swri_dev(ID device, W start, void *data, SZ size, SZ *actual);
#endif
