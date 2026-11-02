/** @file obs_state.h
 * @brief Obstacle state and route enumerations.
 */
#ifndef OBS_STATE_H
#define OBS_STATE_H

#include "scan_plan.h"
#include <stdint.h>
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
    OBS_PHASE_FAULT
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
#define OBS_EVT_FRONT_WATCH 0x20U

/** @brief What a single ping found. */
typedef enum
{
    OBS_ECHO_VALID = 0, /**< An echo, so the range is a real surface. */
    OBS_ECHO_BEYOND,    /**< Echo started but nothing came back in range. */
    OBS_ECHO_NONE       /**< No echo pulse at all: unknown, never clear. */
} obs_state_echo_t;

/** @brief Where the ultrasonic sensor points, as the obstacle task knows. */
typedef enum
{
    OBS_AIM_FORWARD = 0, /**< Settled at FRONT. */
    OBS_AIM_LEFT,        /**< Settled at a bearing to the left. */
    OBS_AIM_RIGHT,       /**< Settled at a bearing to the right. */
    OBS_AIM_RETURNING,   /**< Moving back to FRONT; not usable yet. */
    OBS_AIM_SETTLING,    /**< Moving to a side bearing; not usable yet. */
    OBS_AIM_UNAVAILABLE  /**< Not started or parked after an abort. */
} obs_state_aim_t;

/** @brief One published ping, copied out as a whole under the lock. */
typedef struct
{
    uint32_t time_msec;   /**< Kernel time when the ping ended. */
    int16_t  range_mm;    /**< Meaningful only for OBS_ECHO_VALID. */
    int8_t   bearing_deg; /**< Servo bearing during the ping; + is left. */
    uint8_t  echo;        /**< An obs_state_echo_t value. */
    uint8_t  seq;         /**< Changes with every new ping. */
} obs_state_reading_t;

/** @brief The sensor orientation state. */
typedef struct
{
    uint32_t time_msec;   /**< When this state was entered. */
    int8_t   bearing_deg; /**< Commanded bearing; + is left. */
    uint8_t  aim;         /**< An obs_state_aim_t value. */
} obs_state_aim_info_t;

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
void obs_state_plan_store (scan_plan_bypass_t const * p_plan);
/** @brief Load the stored plan. */
void obs_state_plan_load (scan_plan_bypass_t * p_plan);
/** @brief Set the servo angle for the side watch; positive looks left. */
void obs_state_watch_angle_set (int32_t angle_deg);
/** @brief Get the servo angle for the side watch. */
int32_t obs_state_watch_angle_get (void);

/** @brief Publish a side ping with the bearing the servo really had. */
void obs_state_side_reading_set (obs_state_echo_t echo, int32_t range_mm,
                                 int32_t bearing_deg, uint32_t time_msec);
/** @brief Copy the latest side ping; seq changes with each new ping. */
void obs_state_side_reading_get (obs_state_reading_t * p_reading);
/** @brief Publish a forward ping; only taken with the servo at FRONT. */
void obs_state_front_reading_set (obs_state_echo_t echo, int32_t range_mm,
                                  uint32_t time_msec);
/** @brief Copy the latest forward ping; seq changes with each new ping. */
void obs_state_front_reading_get (obs_state_reading_t * p_reading);

/** @brief Publish where the sensor points and whether it has settled.
 *  @param time_msec When the servo reached, or was sent to, the bearing.
 */
void obs_state_aim_set (obs_state_aim_t aim, int32_t bearing_deg,
                        uint32_t time_msec);
/** @brief Copy the sensor orientation state. */
void obs_state_aim_get (obs_state_aim_info_t * p_info);

#endif /* OBS_STATE_H */

/*** end of file ***/

