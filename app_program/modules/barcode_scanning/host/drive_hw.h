/* Register doubles only: retain the actual production bc_cal.h settings. */
#ifndef DRIVE_HW_H
#define DRIVE_HW_H
/* Existing register doubles model digital GPIO, not the ADC peripheral. */
#define APP_BARCODE_ANALOG_ENABLED 0
#include "barcode_kernel.h"
#define TIMER_ALARM2 0x40054018U
#define TIMER_ARMED 0x40054020U
#define TIMER_INTR 0x40054034U
#define TIMER_INTE 0x40054038U
#endif
