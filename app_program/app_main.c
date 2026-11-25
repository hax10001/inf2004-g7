/** @file app_main.c
 * @brief Robot startup: initialise every subsystem, then start the tasks.
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "app_main.h"
#include "modules/barcode_scanning/bc_setup.h"
#include "app/app_config.h"
#include "app/controller.h"
#include "modules/line_barcode/line_barcode.h"
#include "modules/motion/motion.h"
#include "modules/obstacle/obstacle.h"
#include "modules/terrain/terrain.h"
#include <tm/tmonitor.h>
#ifdef B1_ROBOT_APP
#include "modules/telemetry/buddy1_rt.h"
#endif

#ifdef B1_ROBOT_APP
/* Boot stage and last init error, printed by the USB console diagnostics
 * (usb_console.c), so they keep external linkage. */
volatile uint32_t g_usermain_stage;
volatile int32_t g_usermain_error;
/* Boot stages written to g_usermain_stage, in order. */
#define APP_MAIN_STAGE_ENTERED       1U /* usermain entered */
#define APP_MAIN_STAGE_DELAY_DONE    2U /* startup delay complete */
#define APP_MAIN_STAGE_MOTOR_INIT    3U /* motor init finished */
#define APP_MAIN_STAGE_IR_INIT       4U /* IR init finished */
#define APP_MAIN_STAGE_IMU_INIT      5U /* IMU init finished */
#define APP_MAIN_STAGE_CTRL_INIT     6U /* controller init finished */
#define APP_MAIN_STAGE_MOTOR_TASK    7U /* motor task started */
#define APP_MAIN_STAGE_IR_TASK       8U /* IR task started */
#define APP_MAIN_STAGE_IMU_TASK      9U /* IMU task started */
#define APP_MAIN_STAGE_CTRL_TASK     10U /* controller task started */
#define APP_MAIN_STAGE_TELEMETRY     11U /* telemetry task start attempted */
#endif

#if APP_TEST
#include "modules/line_barcode/test_lbar.h"
#include "modules/terrain/test_ter.h"
#include "modules/obstacle/test_obs.h"
#endif

INT usermain (void)
{
#ifdef B1_ROBOT_APP
    g_usermain_stage = APP_MAIN_STAGE_ENTERED;
#endif
    tk_dly_tsk(APP_STARTUP_DELAY_MSEC);
#ifdef B1_ROBOT_APP
    g_usermain_stage = APP_MAIN_STAGE_DELAY_DONE;
#endif

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[MAIN] Before motor init\n");
    ER error = motion_init();

    if (error < E_OK)
    {
        return 1;
    }

#ifdef B1_ROBOT_APP
    g_usermain_stage = APP_MAIN_STAGE_MOTOR_INIT;
#endif
    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[MAIN] After motor init\n");
    error = line_barcode_init();

    if (error < E_OK)
    {
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[MAIN] IR shared init failed: %d\n", error);
        return 1;
    }

#ifdef B1_ROBOT_APP
    g_usermain_stage = APP_MAIN_STAGE_IR_INIT;
#endif
    /* The barcode engine needs the encoders initialised first. */
    error = bc_setup_init();
    if (error < E_OK)
    {
        return 1;
    }
    error = obstacle_init();
    if (error < E_OK)
    {
        return 1;
    }
    error = terrain_init();

    if (error < E_OK)
    {
        return 1;
    }

#ifdef B1_ROBOT_APP
    g_usermain_stage = APP_MAIN_STAGE_IMU_INIT;
#endif
    error = controller_init();

    if (error < E_OK)
    {
#ifdef B1_ROBOT_APP
        g_usermain_error = (int32_t)error;
#endif
        return 1;
    }

    /* All shared resources exist before any subsystem can run a task. */
#ifdef B1_ROBOT_APP
    g_usermain_stage = APP_MAIN_STAGE_CTRL_INIT;
#endif
    error = motion_start();

    if (error < E_OK)
    {
        return 1;
    }

#ifdef B1_ROBOT_APP
    g_usermain_stage = APP_MAIN_STAGE_MOTOR_TASK;
#endif
    error = line_barcode_start();

    if (error < E_OK)
    {
        return 1;
    }

    /* Sensors must be running before the controller can request movement. */
#ifdef B1_ROBOT_APP
    g_usermain_stage = APP_MAIN_STAGE_IR_TASK;
#endif
    error = terrain_start();

    if (error < E_OK)
    {
        return 1;
    }

#ifdef B1_ROBOT_APP
    g_usermain_stage = APP_MAIN_STAGE_IMU_TASK;
#endif
    error = obstacle_start();
    if (error < E_OK)
    {
        return 1;
    }
    error = controller_start();

    if (error < E_OK)
    {
        return 1;
    }

    /* The barcode worker must start before MQTT. */
    error = bc_setup_start();
    if (error < E_OK)
    {
        return 1;
    }
#ifdef B1_ROBOT_APP
    g_usermain_stage = APP_MAIN_STAGE_CTRL_TASK;
    /* Lower priority than motion/IR/IMU. No motor driver ownership. */
    if(b1_rt_start()!=0) {
        /* Telemetry is optional; the robot tasks are already running. */
        tm_printf((UB *)
            "[MAIN] MQTT offline; robot tasks remain running.\n");
    }
    g_usermain_stage = APP_MAIN_STAGE_TELEMETRY;
#endif

#if APP_TEST
    test_lbar_run();
    test_ter_run();
    test_obs_run();
#endif

    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/

