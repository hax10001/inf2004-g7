#ifndef ROBOPICO_IMU_TASK_H
#define ROBOPICO_IMU_TASK_H

#include <tk/tkernel.h>

/* Task entry point: implemented in imu_task.c, started by app_main.c. */
EXPORT void imu_task(INT stacd, void *exinf);

#endif
