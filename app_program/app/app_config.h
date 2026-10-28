/** @file app_config.h
 * @brief Preserve the tested scheduling and startup configuration.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_TASK_STACK_BYTES 4096
#define APP_IR_PRIORITY 5
#define APP_CONTROLLER_PRIORITY 6
#define APP_IMU_PRIORITY 7
#define APP_STARTUP_DELAY_MSEC 2000
#define APP_CONTROLLER_PERIOD_MSEC 50U
#define APP_IR_MAX_AGE_MSEC 200U
/* Faster than the controller, so each decision sees a recent reading. */
#define APP_IR_PERIOD_MSEC 20
/* At a junction: readings ignored while coasting, then how long a line must
 * be seen after being placed back before following resumes. A lift during
 * the hold is not seen, so the hold is kept just long enough to stop.
 */
#define APP_JUNCTION_HOLD_MSEC 250U
#define APP_JUNCTION_SETTLE_MSEC 150U
#define APP_USB_WAIT_ATTEMPTS 200
#define APP_USB_WAIT_PERIOD_MSEC 100
#define APP_USB_SETTLE_MSEC 500

/* WARNING: The tested startup creates the IMU task but does not start it. */
#define APP_IMU_ENABLED 0

/* Zero drives open loop, still logging RPM; use it to calibrate the target. */
#define APP_SPEED_CONTROL_ENABLED 1

/* One CSV line per control period, for tuning: paste into a spreadsheet. */
#define APP_MOTION_DEBUG_CSV 1

#endif /* APP_CONFIG_H */

/*** end of file ***/

