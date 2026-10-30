/** @file obs_guard.h
 * @brief Additional guard flags and conservative bypass test settings.
 */
#ifndef OBS_GUARD_H
#define OBS_GUARD_H

#include "obs_config.h"

#if (OBS_SCAN_LEFT_DEG > OBS_SERVO_LEFT_LIMIT_DEG)
#error "Scan left angle exceeds the servo limit."
#endif
#if (OBS_SCAN_RIGHT_DEG > OBS_SERVO_RIGHT_LIMIT_DEG)
#error "Scan right angle exceeds the servo limit."
#endif
#if (OBS_SCAN_POINTS != \
     ((OBS_SCAN_LEFT_DEG + OBS_SCAN_RIGHT_DEG) / OBS_SCAN_STEP_DEG + 1))
#error "OBS_SCAN_POINTS does not match the scan angles."
#endif
#if ((OBS_SCAN_LEFT_DEG % OBS_SCAN_STEP_DEG) != 0) || \
    ((OBS_SCAN_RIGHT_DEG % OBS_SCAN_STEP_DEG) != 0)
#error "Scan limits must be multiples of the scan step."
#endif

#define OBS_EVT_FRONT_BLOCKED 0x80U
#define OBS_EVT_FRONT_READY 0x100U
#define OBS_REPLAN_LIMIT 3U
#define OBS_GUARD_WAIT_MSEC 200U
#define OBS_TURN_LIMIT_MSEC 6000U

#endif /* OBS_GUARD_H */

/*** end of file ***/

