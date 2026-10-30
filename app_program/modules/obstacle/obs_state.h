/** @file obs_state.h
 * @brief Obstacle state and route enumerations.
 */
#ifndef OBS_STATE_H
#define OBS_STATE_H

#include "scan_plan.h"
#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

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
    OBS_PHASE_REASSESS
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

#define OBS_GUARD_REASON_NONE 0U
#define OBS_GUARD_REASON_CLOSE 1U
#define OBS_GUARD_REASON_SENSOR 2U
#define OBS_GUARD_REASON_STALE 3U

/** @brief Atomic eight-byte live coverage snapshot, not a wire format.
 * Ranges: three nine-bit centimetre fields; valid views: bits 27..29;
 * reason: bits 30..31. Clock: low eight bits epoch, upper 24 bits ms.
 * Clock ages use modulo 2^24; every transaction is bounded to 2500 ms.
 */
typedef struct
{
    uint32_t ranges;
    uint32_t clock;
} obs_guard_snapshot_t;

/** @brief Invalidate coverage and start a new nonzero epoch; zero on error. */
uint32_t obs_state_guard_request (uint32_t now_msec);
/** @brief Copy live coverage under the shared mutex. */
bool obs_state_guard_load (obs_guard_snapshot_t * p_snapshot);
/** @brief Publish one view only if its epoch still matches. */
bool obs_state_guard_sample (uint32_t epoch, uint32_t view, int32_t range_mm,
                             uint32_t reason);
/** @brief Publish front validity and a modulo-2^24 millisecond timestamp. */
void obs_state_front_mark (bool b_valid, uint32_t now_msec);
/** @brief Require a valid recent forward sample; errors return false. */
bool obs_state_front_ready (uint32_t now_msec, uint32_t max_age_msec);

/** @brief Create the mutex and event flags; call before any task starts.
 *  @return E_OK, or a kernel error code.
 */
ER obs_state_init (void);

/** @brief Read the event flag pattern. */
UINT obs_state_flags_read (void);
/** @brief Set event flag bits. */
void obs_state_flags_set (UINT bits);
/** @brief Clear event flag bits. */
void obs_state_flags_clear (UINT bits);

/** @brief Copy the packed status word under its mutex.
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
bool obs_state_plan_store (scan_plan_bypass_t const * p_plan);
/** @brief Load the stored plan. */
bool obs_state_plan_load (scan_plan_bypass_t * p_plan);
/** @brief Set the side to watch: 1 is left, -1 is right. */
void obs_state_watch_side_set (int32_t side);
/** @brief Get the side to watch. */
int32_t obs_state_watch_side_get (void);

#endif /* OBS_STATE_H */

/*** end of file ***/

