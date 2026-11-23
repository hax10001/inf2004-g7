#ifndef HOST_DEV_I2C_H
#define HOST_DEV_I2C_H
#include <tk/tkernel.h>
#define TDN_I2C_EXEC (-100)
typedef struct {
    UW sadr;
    SZ snd_size;
    UB *snd_data;
    SZ rcv_size;
    UB *rcv_data;
} T_I2C_EXEC;
#endif
