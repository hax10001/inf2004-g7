/** @file app_config.h
 * @brief Preserve the tested scheduling and startup configuration.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_TASK_STACK_BYTES 4096
#define APP_IR_PRIORITY 5
#define APP_CONTROLLER_PRIORITY 6
#define APP_IMU_PRIORITY 7
#define APP_STARTUP_DELAY_MSEC 5000
#define APP_CONTROLLER_PERIOD_MSEC 50U
#define APP_IR_MAX_AGE_MSEC 200U
#define APP_IR_PERIOD_MSEC 100
#define APP_USB_WAIT_ATTEMPTS 200
#define APP_USB_WAIT_PERIOD_MSEC 100
#define APP_USB_SETTLE_MSEC 500

/* WARNING: The tested startup creates the IMU task but does not start it. */
#define APP_IMU_ENABLED 0

#endif /* APP_CONFIG_H */

/*** end of file ***/

