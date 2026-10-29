/** @file app_main.c
 * @brief Robot startup using the template's two-phase subsystem lifecycle.
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "app_main.h"
#include "app/app_config.h"
#include "app/controller.h"
#include "modules/line_barcode/line_barcode.h"
#include "modules/motion/motion.h"
#include "modules/terrain/terrain.h"
#include <tm/tmonitor.h>
#include "modules/obstacle/obstacle.h"

#if APP_TEST
#include "modules/line_barcode/test_lbar.h"
#include "modules/terrain/test_ter.h"
#include "modules/obstacle/test_obs.h"
#endif

INT usermain (void)
{
    tk_dly_tsk(APP_STARTUP_DELAY_MSEC);

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[MAIN] Before motor init\n");
    ER error = motion_init();

    if (error < E_OK)
    {
        return 1;
    }

    /* UB views the same string bytes; tm_printf does not modify them. */
    tm_printf((UB *)"[MAIN] After motor init\n");
    error = line_barcode_init();

    if (error < E_OK)
    {
        /* UB views the same string bytes; tm_printf does not modify them. */
        tm_printf((UB *)"[MAIN] IR shared init failed: %d\n", error);
        return 1;
    }

    error = terrain_init();

    if (error < E_OK)
    {
        return 1;
    }

    error = obstacle_init();

    if (error < E_OK)
    {
                tm_printf((UB *)"[MAIN] Obstacle init failed: %d\n", error);
        return 1;
    }

    error = controller_init();

    if (error < E_OK)
    {
        return 1;
    }

    /* All shared resources exist before any subsystem can run a task. */
    error = motion_start();

    if (error < E_OK)
    {
        return 1;
    }

    error = line_barcode_start();

    if (error < E_OK)
    {
        return 1;
    }

    /* Preserve task creation order: IR, dormant IMU, controller. */
    error = terrain_start();

    if (error < E_OK)
    {
        return 1;
    }

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



#if APP_TEST
    test_lbar_run();
    test_ter_run();
    test_obs_run();
#endif

    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/

