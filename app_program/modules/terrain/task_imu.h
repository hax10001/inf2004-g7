/** @file task_imu.h
 * @brief Internal IMU task entry point.
 */
#ifndef TASK_IMU_H
#define TASK_IMU_H

#include <tk/tkernel.h>

/** @brief Run the original periodic IMU diagnostic when explicitly enabled. */
void task_imu_task (INT start_code, void * p_context);

#endif /* TASK_IMU_H */

/*** end of file ***/

