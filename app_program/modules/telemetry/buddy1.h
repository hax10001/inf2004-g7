/** @file buddy1.h
 * @brief Telemetry store API and compact subsystem structures.
 */
#ifndef BUDDY1_H
#define BUDDY1_H
#include "buddy1_protocol.h"
#include <stdint.h>

#define B1_MOTION_DEPTH 32u
#define B1_EVENT_DEPTH 24u
#define B1_SRC_MOTION 0x01u
#define B1_SRC_LINE 0x02u
#define B1_SRC_TERRAIN 0x04u
#define B1_SRC_OBSTACLE 0x08u
#define B1_SRC_MISSION 0x10u
/* EVENT kinds (dashboard state.js eventNames). */
#define B1_EVT_BARCODE 1u         /* barcode decoded (value = letter) */
#define B1_EVT_OBSTACLE 4u        /* obstacle detected (value = cm) */
#define B1_EVT_BYPASS_CHOSEN 5u   /* bypass side chosen (value = side) */
#define B1_EVT_BYPASSED 6u        /* obstacle bypassed */
#define B1_EVT_SAFETY_FAULT 7u    /* safety fault (value = fault bits) */
/* Terrain EVENT kind: a hump the robot drove over (value = peak mm). */
#define B1_EVT_HUMP_CROSSED 9u
#define B1_EVT_MISSION_STATE 10u  /* mission state changed */
/* EVENT source numbers (motion is never an EVENT source). */
#define B1_EVSRC_LINE 2u
#define B1_EVSRC_TERRAIN 3u
#define B1_EVSRC_OBSTACLE 4u
#define B1_EVSRC_MISSION 5u
/* Mission states reported by the controller. */
#define B1_MISSION_RUNNING 2u     /* line following */
#define B1_MISSION_OBSTACLE 3u    /* handling an obstacle */
#define B1_MISSION_STOPPED 6u     /* stopped, may recalibrate */
/* STATUS time tokens kept as command challenges. */
#define B1_CHALLENGE_SLOTS 8u
/* ODOM checkpoint spacing along the driven path. */
#define B1_CHECKPOINT_MM 500u
/* All time stamps are monotonically increasing milliseconds since boot. */
/* Source records use minimum-width fields and packed booleans. They are
 * never copied onto the MQTT wire as C structs; b1_store_t serialises
 * access under its SMP-safe mutex. */
typedef struct {
    uint32_t ms;
    uint32_t cumulative_mm;
    uint16_t seq;
    int16_t left_delta_mm;
    int16_t right_delta_mm;
    int16_t left_speed_mm_s;
    int16_t right_speed_mm_s;
    uint8_t       checkpoint:1;
} b1_motion_t;
typedef struct {
    uint32_t ms;
    uint16_t seq;
    uint16_t event_id;
    /* IR 0..7, 8 line states, barcode A-Z or 31, four actions. */
    uint8_t       ir:3;
    uint8_t       line:3;
    uint8_t       barcode:5;
    uint8_t       action:2;
    uint8_t       action_valid:1;
    uint8_t       event_kind:5; /* EVENT kind, 0 = none */
} b1_line_t;
typedef struct {
    uint32_t ms;
    uint16_t seq;
    uint16_t hump_mm;
    uint16_t max_hump_mm;
    uint16_t event_id;
    /* Height-valid quality (0..1), hump phase (0..2), EVENT kind. */
    uint8_t       quality:1;
    uint8_t       mode:2;
    uint8_t       event_kind:5;
    /* EVENT value: crossed hump's peak height, 0..127 mm. */
    uint8_t       event_value:7;
} b1_terrain_t;
typedef struct {
    uint32_t ms;
    uint16_t seq;
    uint16_t pwm_left;
    uint16_t pwm_right;
    uint16_t event_id;
    uint16_t event_value;
    uint8_t faults;
    uint8_t ready_blockers;
    uint8_t       event_kind:5;
    uint8_t       mission:3;
    uint8_t       ready:1;
    uint8_t       running:1;
    uint8_t       remote_hold:1;
    uint8_t       barcode_hold:1;
} b1_mission_t;
typedef struct {
    uint32_t ms;
    uint16_t seq;
    uint16_t distance_cm;
    uint16_t event_id;
    uint16_t event_value;
    uint8_t bypass_count;
    uint8_t faults;
    uint8_t       event_kind:5;
    uint8_t       side:2;
} b1_obstacle_t;
/* ODOM ring record, 10 bytes: time32, left delta16, right delta16, seq16.
 * Speeds and checkpoint state stay in the store. A byte array avoids
 * unaligned word access on the Cortex-M0+ and compiler padding. */
#define B1_ODREC_LEN 10u          /* ODOM ring record bytes */
#define B1_ODREC_OFF_LEFT 4       /* 16-bit left delta (after time) */
#define B1_ODREC_OFF_RIGHT 6      /* 16-bit right delta */
#define B1_ODREC_OFF_SEQ 8        /* 16-bit sequence */
typedef struct {
    uint8_t data[B1_ODREC_LEN];
} b1_odom_record_t;

/* EVENT ring record, 9 bytes: source3 + kind5, ID16, value16, time32.
 * Accessed bytewise only and never sent as a C struct. The store mutex
 * owns the ring. */
#define B1_EVREC_LEN 9u           /* EVENT ring record bytes */
#define B1_EVREC_OFF_ID 1         /* 16-bit event ID */
#define B1_EVREC_OFF_VALUE 3      /* 16-bit event value */
#define B1_EVREC_OFF_TIME 5       /* 32-bit time stamp */
#define B1_EVREC_SRC_MASK 7u      /* byte 0: source in bits 0..2 */
#define B1_EVREC_KIND_MASK 31u    /* byte 0: kind in bits 3..7 */
#define B1_EVREC_KIND_SHIFT 3u
typedef struct {
    uint8_t data[B1_EVREC_LEN];
} b1_event_t;

/* Lock/unlock must be a real SMP-safe RTOS mutex and must be non-ISR. */
typedef void (*b1_lock_fn_t)(void *ctx);
typedef uint8_t (*b1_coordinator_fn_t)(uint8_t cmd, void *ctx);
typedef struct {
    b1_lock_fn_t lock,unlock; void *lock_context;
    b1_motion_t motion; b1_line_t line; b1_terrain_t terrain;
        b1_mission_t mission; b1_obstacle_t obstacle;
    /* Single store mutex protects all packed state and bounded rings.
     * seen is a 5-bit source mask; ring head/count ranges are explicit. */
    uint32_t     seen:5;
    uint32_t     mapping_valid:1;
    uint32_t     committed:1;
    uint32_t     uncertain:1;
    uint8_t mapping[B1_MAP_LETTERS];
    b1_odom_record_t motions[B1_MOTION_DEPTH];
    uint32_t     motion_head:5;
    uint32_t     motion_count:6;
    b1_event_t events[B1_EVENT_DEPTH];
    uint32_t     event_head:5;
    uint32_t     event_count:5;
    uint32_t dropped,invalid,dup,session_ms,last_snapshot_ms,
        last_heartbeat_ms,last_status_ms,checkpoint_mm,checkpoint_ms;
    uint32_t     checkpoint_pending:1;
    uint16_t session;
    uint16_t wire_seq;
    uint16_t last_cmd_transaction;
    uint8_t last_cmd_kind;
    uint32_t     cmd_seen:1;
    /* Recently transmitted STATUS uptime values act as per-boot challenges.
     * A host echoes one exactly, instead of estimating clock offset. */
    uint32_t status_challenges[B1_CHALLENGE_SLOTS];
    uint32_t     status_challenge_head:3;
    b1_coordinator_fn_t coordinator; void *coordinator_context;
} b1_store_t;

/** @brief Initialise a store; lock and unlock must not be NULL. */
int32_t b1_store_init(b1_store_t *p_store, b1_lock_fn_t lock,
    b1_lock_fn_t unlock,
    void *p_ctx,
    uint16_t session);
/** @brief Register the callback that applies START/STOP/recalibrate. */
void b1_set_coordinator(b1_store_t *p_store, b1_coordinator_fn_t fn,
    void *p_ctx);
/* Ingest: copy one record under the mutex; -1 on NULL or a duplicate
 * sequence. No network work is done while the mutex is held. */
int32_t b1_ingest_motion(b1_store_t *p_store,const b1_motion_t *p_rec);
int32_t b1_ingest_line(b1_store_t *p_store,const b1_line_t *p_rec);
int32_t b1_ingest_terrain(b1_store_t *p_store,const b1_terrain_t *p_rec);
int32_t b1_ingest_obstacle(b1_store_t *p_store,const b1_obstacle_t *p_rec);
int32_t b1_ingest_mission(b1_store_t *p_store,const b1_mission_t *p_rec);
/** @brief Build at most one frame: STATUS when due, then EVENT, ODOM,
 *  CHECKPOINT, HEARTBEAT, SNAPSHOT. Returns 1 if a frame was built. */
int32_t b1_next_frame(b1_store_t *p_store,uint32_t now_ms,b1_frame_t *p_out);
/** @brief Stamp a SENSOR or BARCODE_DIAG frame with session and sequence. */
int32_t b1_sensor_frame(b1_store_t *p_store,b1_frame_t *p_frame);
/** @brief 1 if token is a STATUS time sent recently in this session.
 *  Commands other than STOP must echo one; STOP still checks the session. */
int32_t b1_recent_status_challenge(b1_store_t *p_store,uint32_t token,
    uint32_t now_ms);
/** @brief Apply a command and build its ACK; returns the result code.
 *  An accepted START/STOP means the coordinator accepted it, not that the
 *  motors have changed. */
int32_t b1_handle_command(b1_store_t *p_store,const b1_frame_t *p_request,
    b1_frame_t *p_ack);
#endif
/*** end of file ***/

