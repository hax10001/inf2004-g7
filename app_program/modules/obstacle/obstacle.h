/** @file obstacle.h
 * @brief Obstacle detection, scan, bypass and line reacquisition.
 *
 * The obstacle task owns the ultrasonic sensor and servo. The controller
 * stays the only caller of the motion module and drives the bypass one
 * non-blocking step per period.
 */
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
    OBSTACLE_S_IDLE = 0,
    OBSTACLE_S_CONFIRM,
    OBSTACLE_S_STOPPING,
    OBSTACLE_S_SCANNING,
    OBSTACLE_S_TURN_OUT,
    OBSTACLE_S_DIAG_OUT,
    OBSTACLE_S_TURN_PARALLEL,
    OBSTACLE_S_PASS_SIDE,
    OBSTACLE_S_PASS_END,
    OBSTACLE_S_TURN_BACK,
    OBSTACLE_S_SEARCH_LINE,
    OBSTACLE_S_ALIGN,
    OBSTACLE_S_VERIFY,
    OBSTACLE_S_FAULT
} obstacle_state_t;

typedef enum
{
    OBSTACLE_ROUTE_NONE = 0,
    OBSTACLE_ROUTE_LEFT,
    OBSTACLE_ROUTE_RIGHT
} obstacle_route_t;

/* Fault bits; one bit each in the shared fault field. */
#define OBS_FAULT_NO_ECHO (1U << 0)
#define OBS_FAULT_NO_ROUTE (1U << 1)
#define OBS_FAULT_LINE_LOST (1U << 2)
#define OBS_FAULT_SERVO (1U << 3)
#define OBS_FAULT_TIMEOUT (1U << 4)

#define OBS_DIST_INVALID_CM 511U
#define OBS_WIDTH_INVALID_CM 255U

/** @brief What the controller should do this period. */
typedef struct
{
    float left_rpm;
    float right_rpm;
    bool  b_stop;        /**< Stop both wheels. */
    bool  b_line_follow; /**< Use the normal line action this period. */
    bool  b_done;        /**< Bypass finished; resume line following. */
    bool  b_fault;       /**< Stop and enter the error state. */
} obstacle_drive_t;

/** @brief Create the mutex, event flags, sensor and servo. */
ER obstacle_init (void);
/** @brief Create and start the obstacle task. */
ER obstacle_start (void);
/** @brief True once an obstacle has been confirmed ahead. */
bool obstacle_is_blocked (void);
/** @brief True if a critical sensor fault should stop the robot. */
bool obstacle_has_fault (void);
/** @brief Cancel any scan or bypass and park the servo (STOP or fault path). */
void obstacle_abort (void);
/** @brief Advance the bypass one step; call once per controller period.
 * @param line_left Left IR, 1 = black.
 * @param line_right Right IR, 1 = black.
 * @param now_msec Kernel time in milliseconds.
 */
obstacle_drive_t obstacle_bypass_step (uint32_t line_left, uint32_t line_right,
                                       uint32_t now_msec);

/** @brief Packed status word, copied under its mutex.
 * Bits: 0-3 state, 4-5 route, 6-10 faults, 11-14 bypass count,
 * 15-23 distance cm (511 invalid), 24-31 width cm (255 invalid).
 */
uint32_t obstacle_get_status (void);
uint32_t obstacle_status_pack (obstacle_state_t state, obstacle_route_t route,
                               uint8_t faults, uint8_t count, uint16_t dist_cm,
                               uint8_t width_cm);
obstacle_state_t obstacle_status_state (uint32_t word);
obstacle_route_t obstacle_status_route (uint32_t word);
uint8_t          obstacle_status_faults (uint32_t word);
uint8_t          obstacle_status_count (uint32_t word);
uint16_t         obstacle_status_distance_cm (uint32_t word);
uint8_t          obstacle_status_width_cm (uint32_t word);

#endif /* OBSTACLE_H */

/*** end of file ***/

