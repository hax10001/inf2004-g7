/** @file nav_core.h
 * @brief Deterministic, motor-free barcode-to-junction navigation decisions.
 * The caller owns one instance, all timing inputs, sensors and motor outputs.
 */
#ifndef NAV_CORE_H
#define NAV_CORE_H
#include <stdint.h>

typedef enum {
    NAV_WAIT = 0, NAV_FOLLOW, NAV_APPROACH, NAV_CROSS,
    NAV_TURN, NAV_REACQUIRE, NAV_RESUME, NAV_FAULT,
    NAV_CENTRE /* turn queued: driving on until the axle is over the cross */
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
    NAV_EVENT_SCAN_REJECTED = 19,
    NAV_EVENT_DECODE_WAIT = 20, /* stopped at junction, letter decoding */
    NAV_EVENT_SAFE_STOP = 21 /* fault source; existing EVENT payload */
} nav_event_t;
typedef struct {
    uint32_t confirm_msec;
    uint32_t min_turn_msec;
    uint32_t max_turn_msec;
    uint32_t max_cross_msec;
    uint32_t min_after_barcode_counts;
    /** Both line sensors must stay black over this much forward travel
     *  (encoder counts) before it counts as a junction. Shorter both-black
     *  runs, such as a curve's line passing diagonally under both sensors
     *  or a barcode bar, are driven straight over. */
    uint32_t junction_confirm_counts;
    /** Bounds that confirmation in time if the robot is barely moving:
     *  this long continuously both-black also counts as a junction. */
    uint32_t junction_confirm_max_msec;
    /** Junction reached while a barcode is still decoding: stop and wait
     *  this long for the letter (0 disables: fault at once, as before). */
    uint32_t decode_wait_msec;
    /** After decoding ends, how long the letter may take to arrive. */
    uint32_t decode_grace_msec;
    /** Left/right/U-turn: forward travel from where the crossing bar was
     *  first seen (both black) until the wheel axle is over the junction
     *  centre, so a pivot there leaves the sensors on the new arm.
     *  0: turn at once where the junction was confirmed (old behaviour). */
    uint32_t centre_counts;
    /** Time bound on that drive to the centre. */
    uint32_t centre_max_msec;
    /** Both wheels' travel added together (turn_counts) for a 90 degree
     *  pivot. The turn ignores the line until 60 degrees (150 for a
     *  U-turn), ends on the first black reading after that, and faults
     *  past 135 degrees (225). 0: old timed, edge-confirmed turn. */
    uint32_t turn90_counts;
    /** 0: a letter's action is used at the next junction only, after
     *  which a junction with no new letter is a safety stop.
     *  1: the action repeats at every junction until another barcode
     *  replaces it (a new letter overrides the queued one). Survives an
     *  obstacle bypass; cleared by STOP (nav_forget). */
    unsigned char repeat_action:1;
} nav_config_t;
typedef struct {
    uint32_t now_msec;
    uint32_t travel_counts; /* Forward approximation, not mm. */
    /* All five sensor flags are boolean and the caller owns this sample.
     * No read-modify-write occurs from an ISR on this packed storage. */
    unsigned char left_black:1;
    unsigned char right_black:1;
    unsigned char barcode_capturing:1;
    unsigned char barcode_decoding:1; /**< Recorded pass decoding. */
    /** The 1 ms sampler saw both-black broken by white since the previous
     *  step, so a both-black run must restart its distance measurement. */
    unsigned char both_interrupted:1;
    /** Both wheels' movement added together, either direction, never
     *  decreasing. A pivot's angle is its growth divided by the track. */
    uint32_t turn_counts;
} nav_sample_t;
typedef struct {
    /* Nine navigation states require four bits, action values 0..3 two.
     * Controller owns the instance; this is not an ISR-shared structure.
     * In particular, do NOT assume this native layout is a wire format. */
    unsigned int state:4;
    unsigned int pending:1;
    unsigned int letter:5;
    unsigned int action:2;
    unsigned int edge_seen:1;
    unsigned int turn_announced:1;
    unsigned int resume_announced:1;
    unsigned int fault_reason:3; /* 0..5: unmapped, turn, cross, no letter */
    unsigned int decode_hold:1;
    unsigned int hold_letter:1;
    unsigned int held:1;
    unsigned int held_letter:5;
    unsigned int held_action:2;
    unsigned int have_barcode:1; /* zero is a valid wrapped sequence */
    uint32_t hold_since;
    uint32_t decode_idle_since; /* 0 while the scanner is still decoding */
    uint32_t state_since;
    uint32_t black_since;
    uint32_t black_from_counts;
    uint32_t edge_since;
    uint32_t barcode_counts;
    uint32_t last_barcode_counts;
    uint32_t last_barcode_ms;
    uint32_t centre_from; /* travel where the crossing bar was first seen */
    uint32_t turn_from;   /* turn_counts when the pivot started */
    uint16_t last_barcode_seq;
    nav_config_t cfg;
} nav_controller_t;
typedef struct {
    /* Drive 0..5, event 0..20, value 0..5 (action or fault reason). */
    unsigned char drive:3;
    unsigned char event:5;
    unsigned char value:3;
} nav_result_t;
void nav_init(nav_controller_t *nav, const nav_config_t *cfg);
void nav_stop(nav_controller_t *nav);
/* STOP: also drop the remembered repeat action (a new run starts clean). */
void nav_forget(nav_controller_t *nav);
void nav_start(nav_controller_t *nav, uint32_t now_msec);
/* 1 queued (repeat mode: also replaced), 0 duplicate, -1 already queued
 * (single mode only), -2 not running. */
int nav_barcode(nav_controller_t *nav, uint8_t letter, uint8_t action,
                uint16_t seq, uint32_t ms, uint32_t traveled);
nav_result_t nav_step(nav_controller_t *nav, const nav_sample_t *sample);
#endif

/*** end of file ***/

