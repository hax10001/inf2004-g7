/** @file obs_state.h
 * @brief Obstacle state and route enumerations.
 */
#ifndef OBS_STATE_H
#define OBS_STATE_H

#include "scan_plan.h"
#include <stdint.h>
#include <tk/tkernel.h>

/* The phase field is 4 bits, so 16 values fit; new phases are appended so
 * the existing numbers do not change.
 */
typedef enum
{
    OBS_PHASE_IDLE = 0,
    OBS_PHASE_CONFIRM,
    OBS_PHASE_STOPPING,
    OBS_PHASE_SCANNING,
    OBS_PHASE_TURN_OUT,
    OBS_PHASE_DIAG_OUT,
    OBS_PHASE_TURN_PARALLEL,
    OBS_PHASE_PASS_SIDE,
    OBS_PHASE_PASS_END,
    OBS_PHASE_TURN_BACK,
    OBS_PHASE_SEARCH_LINE,
    OBS_PHASE_ALIGN,
    OBS_PHASE_VERIFY,
    OBS_PHASE_FAULT,
    OBS_PHASE_FWD_CHECK
} obs_state_phase_t;

typedef enum
{
    OBS_ROUTE_NONE = 0,
    OBS_ROUTE_LEFT,
    OBS_ROUTE_RIGHT
} obs_state_route_t;

/* Fault bits; one bit per independent fault in the shared fault field. */
#define OBS_FAULT_NO_ECHO 0x01U
#define OBS_FAULT_NO_ROUTE 0x02U
#define OBS_FAULT_LINE_LOST 0x04U
#define OBS_FAULT_SERVO 0x08U
#define OBS_FAULT_TIMEOUT 0x10U

#define OBS_DIST_INVALID_CM 511U
#define OBS_WIDTH_INVALID_CM 255U

/* Event flag bits shared by the task and the controller side. */
#define OBS_EVT_BLOCKED 0x01U
#define OBS_EVT_SCAN_REQ 0x02U
#define OBS_EVT_SCAN_DONE 0x04U
#define OBS_EVT_ABORT 0x08U
#define OBS_EVT_SIDE_WATCH 0x10U
#define OBS_EVT_SIDE_SEEN 0x20U
#define OBS_EVT_SIDE_CLEAR 0x40U
/* Stopped forward check requested, finished, and found clear. */
#define OBS_EVT_FWD_REQ 0x80U
#define OBS_EVT_FWD_DONE 0x100U
#define OBS_EVT_FWD_CLEAR 0x200U
/* Forward guard wanted while moving, and an object found inside its range. */
#define OBS_EVT_GUARD 0x400U
#define OBS_EVT_FRONT_HIT 0x800U

/** @brief Create the lock and event flags; call before any task starts.
 *  @return E_OK, or a kernel error code.
 */
ER obs_state_init (void);

/** @brief Read the event flag pattern. */
UINT obs_state_flags_read (void);
/** @brief Set event flag bits. */
void obs_state_flags_set (UINT bits);
/** @brief Clear event flag bits. */
void obs_state_flags_clear (UINT bits);

/** @brief Read the packed status word; one aligned 32-bit read, no lock.
 *  Bits 0-3 phase, 4-5 route, 6-10 faults, 11-14 bypass count,
 *  15-23 distance in cm and 24-31 object width in cm.
 */
uint32_t obs_state_status_get (void);
/** @brief Build a status word from its fields. */
uint32_t obs_state_status_pack (obs_state_phase_t phase,
                                obs_state_route_t route, uint32_t faults,
                                uint32_t count, uint32_t dist_cm,
                                uint32_t width_cm);
/** @brief Phase field of a status word. */
obs_state_phase_t obs_state_status_phase (uint32_t word);
/** @brief Route field of a status word. */
obs_state_route_t obs_state_status_route (uint32_t word);
/** @brief Fault bits of a status word. */
uint32_t obs_state_status_faults (uint32_t word);
/** @brief Bypass count of a status word. */
uint32_t obs_state_status_count (uint32_t word);
/** @brief Distance field of a status word, in cm. */
uint32_t obs_state_status_distance_cm (uint32_t word);
/** @brief Object width field of a status word, in cm. */
uint32_t obs_state_status_width_cm (uint32_t word);

/** @brief Publish the phase. */
void obs_state_set_phase (obs_state_phase_t phase);
/** @brief Publish the chosen route. */
void obs_state_set_route (obs_state_route_t route);
/** @brief Publish a distance in mm; out-of-range values become invalid. */
void obs_state_set_distance_mm (int32_t range_mm);
/** @brief Publish the object width in cm. */
void obs_state_set_width_cm (int32_t width_cm);
/** @brief Count one completed bypass, saturating at the field maximum. */
void obs_state_count_bypass (void);
/** @brief Set and clear fault bits in one protected update. */
void obs_state_faults_update (uint32_t set_mask, uint32_t clear_mask);

/** @brief Store a plan; the reader waits for OBS_EVT_SCAN_DONE. */
void obs_state_plan_store (scan_plan_bypass_t const * p_plan);
/** @brief Load the stored plan. */
void obs_state_plan_load (scan_plan_bypass_t * p_plan);
/** @brief Set the side the OBJECT is on, to look at: 1 left, -1 right. */
void obs_state_watch_side_set (int32_t side);
/** @brief Get the side to look at. */
int32_t obs_state_watch_side_get (void);
/** @brief Publish the latest side range that saw the object, in mm. */
void obs_state_side_range_set (int32_t range_mm);
/** @brief Latest side range that saw the object, in mm; 0 if none. */
int32_t obs_state_side_range_get (void);

#endif /* OBS_STATE_H */

/*** end of file ***/

