/** @file nav_core.h
 * @brief Deterministic, motor-free barcode-to-junction navigation decisions.
 * The caller owns one instance, all timing inputs, sensors and motor outputs.
 */
#ifndef NAV_CORE_H
#define NAV_CORE_H
#include <stdint.h>

typedef enum {
    NAV_WAIT = 0, NAV_FOLLOW, NAV_APPROACH, NAV_CROSS,
    NAV_TURN, NAV_REACQUIRE, NAV_RESUME, NAV_FAULT
} nav_state_t;
typedef enum {
    NAV_STRAIGHT = 0, NAV_LEFT = 1, NAV_RIGHT = 2, NAV_UTURN = 3
} nav_action_t;
typedef enum {
    NAV_OUTPUT_STOP = 0, NAV_OUTPUT_FOLLOW, NAV_OUTPUT_FORWARD,
    NAV_OUTPUT_LEFT, NAV_OUTPUT_RIGHT, NAV_OUTPUT_UTURN
} nav_drive_t;
/* These numbers are reserved MQTT mission event kinds. */
typedef enum {
    NAV_EVENT_NONE = 0, NAV_EVENT_QUEUED = 12,
    NAV_EVENT_JUNCTION = 13, NAV_EVENT_TURN_STARTED = 14,
    NAV_EVENT_TURN_COMPLETED = 15, NAV_EVENT_LINE_REACQUIRED = 16,
    NAV_EVENT_FOLLOWING = 17, NAV_EVENT_FAULT = 18,
    NAV_EVENT_SCAN_REJECTED = 19
} nav_event_t;
typedef struct {
    uint32_t confirm_msec;
    uint32_t min_turn_msec;
    uint32_t max_turn_msec;
    uint32_t max_cross_msec;
    uint32_t min_after_barcode_counts;
    uint32_t scan_cross_max_msec; /**< Maximum barcode-cross clearance. */
    uint32_t scan_cross_max_counts; /**< Encoder bound; no blind motion. */
} nav_config_t;
typedef struct {
    uint32_t now_msec;
    uint32_t travel_counts; /* Forward approximation, not mm. */
    uint8_t left_black;
    uint8_t right_black;
    uint8_t barcode_capturing;
} nav_sample_t;
typedef struct {
    nav_state_t state;
    uint8_t pending;
    uint8_t letter;
    uint8_t action;
    uint8_t edge_seen;
    uint8_t turn_announced;
    uint8_t resume_announced;
    uint8_t fault_reason; /* 1 unmapped; 2 turn; 3 cross timeout. */
    uint32_t state_since;
    uint32_t black_since;
    uint32_t black_from_counts;
    uint32_t edge_since;
    uint32_t barcode_counts;
    uint32_t last_barcode_counts;
    uint32_t last_barcode_ms;
    uint16_t last_barcode_seq;
    nav_config_t cfg;
} nav_controller_t;
typedef struct {
    nav_drive_t drive;
    nav_event_t event;
    uint8_t value;
} nav_result_t;
void nav_init(nav_controller_t *nav, const nav_config_t *cfg);
void nav_stop(nav_controller_t *nav);
void nav_start(nav_controller_t *nav, uint32_t now_msec);
/* 1 queued, 0 duplicate, -1 already queued, -2 not running. */
int nav_barcode(nav_controller_t *nav, uint8_t letter, uint8_t action,
                uint16_t seq, uint32_t ms, uint32_t traveled);
nav_result_t nav_step(nav_controller_t *nav, const nav_sample_t *sample);
#endif

/*** end of file ***/

