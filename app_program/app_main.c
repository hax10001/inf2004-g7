/** @file app_main.c
 * @brief Robot startup using the template's two-phase subsystem lifecycle.
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "app_main.h"
#include "modules/barcode_scanning/bc_setup.h"
#include "app/app_config.h"
#include "app/controller.h"
#include "modules/line_barcode/line_barcode.h"
#include "modules/motion/motion.h"
#include "modules/terrain/terrain.h"
#include <tm/tmonitor.h>
#ifdef B1_ROBOT_APP
#include "modules/telemetry/buddy1_rt.h"
#endif

#ifdef B1_ROBOT_APP
/* Read-only stage marker for USB boot diagnostics. */
volatile unsigned int g7_usermain_stage;
volatile int g7_usermain_error;
#endif

#if APP_TEST
#include "modules/line_barcode/test_lbar.h"
#include "modules/terrain/test_ter.h"
#endif

INT usermain (void)
{
#ifdef B1_ROBOT_APP
    g7_usermain_stage = 1U; /* usermain entered */
#endif
    tk_dly_tsk(APP_STARTUP_DELAY_MSEC);
#ifdef B1_ROBOT_APP
    g7_usermain_stage = 2U; /* startup delay complete */
#endif

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[MAIN] Before motor init\n");
    ER error = motion_init();

    if (error < E_OK)
    {
        return 1;
    }

#ifdef B1_ROBOT_APP
    g7_usermain_stage = 3U; /* motor init finished */
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
    g7_usermain_stage = 4U; /* IR init finished */
#endif
    /* Hardware barcode engine needs encoder ownership initialized first. */
    error = bc_setup_init();
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
    g7_usermain_stage = 5U; /* IMU init finished */
#endif
    error = controller_init();

    if (error < E_OK)
    {
#ifdef B1_ROBOT_APP
        g7_usermain_error = (int)error;
#endif
        return 1;
    }

    /* All shared resources exist before any subsystem can run a task. */
#ifdef B1_ROBOT_APP
    g7_usermain_stage = 6U; /* controller init finished */
#endif
    error = motion_start();

    if (error < E_OK)
    {
        return 1;
    }

#ifdef B1_ROBOT_APP
    g7_usermain_stage = 7U; /* motor task started */
#endif
    error = line_barcode_start();

    if (error < E_OK)
    {
        return 1;
    }

    /* Preserve task creation order: IR, dormant IMU, controller. */
#ifdef B1_ROBOT_APP
    g7_usermain_stage = 8U; /* IR task started */
#endif
    error = terrain_start();

    if (error < E_OK)
    {
        return 1;
    }

#ifdef B1_ROBOT_APP
    g7_usermain_stage = 9U; /* IMU task started */
#endif
    error = controller_start();

    if (error < E_OK)
    {
        return 1;
    }

    /* Worker is independent of line-following and must precede MQTT. */
    error = bc_setup_start();
    if (error < E_OK)
    {
        return 1;
    }
#ifdef B1_ROBOT_APP
    g7_usermain_stage = 10U; /* controller task started */
    /* Lower priority than motion/IR/IMU. No motor driver ownership. */
    if(b1_rt_start()!=0) {
        /* Network monitoring is optional. The motion/IR/IMU/controller
         * tasks were already started independently above. */
        tm_printf((UB *)
            "[MAIN] MQTT offline; robot tasks remain running.\n");
    }
    g7_usermain_stage = 11U; /* telemetry task start attempted */
#endif

#if APP_TEST
    test_lbar_run();
    test_ter_run();
#endif

    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/

