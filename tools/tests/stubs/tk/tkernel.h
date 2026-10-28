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

/* Encoder driver: interrupts and RP2040 registers, simulated by the harness. */
typedef struct { UW intatr; FP inthdr; } T_DINT;
ER tk_def_int(UINT intno, const T_DINT *config);
void EnableInt(UINT intno, INT level);
void ClearInt(UINT intno);
#define DI(intsts) ((intsts) = 0U)
#define EI(intsts) ((void)(intsts))
UW in_w(UW address);
void out_w(UW address, UW value);
#define set_w(address, bits) out_w((address), in_w(address) | (bits))
#define clr_w(address, bits) out_w((address), in_w(address) & ~(UW)(bits))
#define XOSC_MHz 12
#define IO_BANK0_BASE 0x40014000U
#define PADS_BANK0_BASE 0x4001c000U
#define GPIO(n) (PADS_BANK0_BASE + 0x4U + ((n) * 4U))
#define GPIO_PUE (1U << 3)
#define GPIO_PDE (1U << 2)
#define SIO_BASE 0xD0000000U
#define GPIO_IN (SIO_BASE + 0x004U)
#define TIMER_TIMERAWL 0x40054028U
#define RESETS_RESET 0x4000C000U
#define RESETS_RESET_DONE 0x4000C008U
#define RESETS_RESET_TIMER 0x00200000U
#endif
