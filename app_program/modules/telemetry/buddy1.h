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
/* All time stamps are monotonically increasing milliseconds since boot. */
/* V9: application-owned records use minimum-width discrete states and
 * packed booleans. Records are never memcpy'ed onto the MQTT wire.
 * B1_Store serializes accesses under its SMP-safe task mutex. */
typedef struct {
    uint32_t ms,cumulative_mm;
    uint16_t seq;
    int16_t left_delta_mm,right_delta_mm;
    int16_t left_speed_mm_s,right_speed_mm_s;
    unsigned char checkpoint:1;
} B1_Motion;
typedef struct {
    uint32_t ms;
    uint16_t seq,event_id;
    /* IR 0..7, 8 line states, barcode A-Z or 31, four actions. */
    unsigned char ir:3,line:3,barcode:5,action:2,action_valid:1;
    unsigned char event_kind:5; /* Current app events 0..20. */
} B1_Line;
typedef struct {
    uint32_t ms;
    uint16_t seq,hump_mm,max_hump_mm,event_id;
    /* Height-valid quality (0..1), hump phase (0..2), event 0..20. */
    unsigned char quality:1,mode:2,event_kind:5;
} B1_Terrain;
typedef struct {
    uint32_t ms;
    uint16_t seq,pwm_left,pwm_right,event_id,event_value;
    uint8_t faults,ready_blockers;
    unsigned char event_kind:5;
    unsigned char mission:3,ready:1,running:1,remote_hold:1,
        barcode_hold:1;
} B1_Mission;
typedef struct {
    uint32_t ms;
    uint16_t seq,distance_cm,event_id,event_value;
    uint8_t bypass_count,faults;
    unsigned char event_kind:5,side:2;
} B1_Obstacle;
/* Only timestamp32, wheel deltas16+16 and sequence16 are needed
 * after ingest. The latest full B1_Motion remains separate for snapshots
 * and checkpoint state remains in the store. The 10-byte array avoids
 * Cortex-M0+ unaligned word accesses and compiler-specific padding. */
typedef struct {
    uint8_t data[10];
} B1_OdomRecord;

/* A nine-byte ring record: timestamp32 + ID16 + value16 +
 * source3 + kind5. Never dereference unaligned word pointers or publish
 * this memory as a C struct on the wire. B1_Store's mutex owns the ring. */
typedef struct {
    uint8_t data[9];
} B1_Event;

/* Lock/unlock must be a real SMP-safe RTOS mutex and must be non-ISR. */
typedef void (*B1_Lock)(void *ctx);
typedef uint8_t (*B1_Coordinator)(uint8_t cmd, void *ctx);
typedef struct {
    B1_Lock lock,unlock; void *lock_context;
    B1_Motion motion; B1_Line line; B1_Terrain terrain;
        B1_Mission mission; B1_Obstacle obstacle;
    /* Single store mutex protects all packed state and bounded rings.
     * seen is a 5-bit source mask; ring head/count ranges are explicit. */
    unsigned int seen:5, mapping_valid:1, committed:1, uncertain:1;
    uint8_t mapping[4];
    B1_OdomRecord motions[B1_MOTION_DEPTH];
    unsigned int motion_head:5, motion_count:6;
    B1_Event events[B1_EVENT_DEPTH];
    unsigned int event_head:5, event_count:5;
    uint32_t dropped,invalid,dup,session_ms,last_snapshot_ms,
        last_heartbeat_ms,last_status_ms,checkpoint_mm,checkpoint_ms;
    unsigned int checkpoint_pending:1;
    uint16_t session,wire_seq,last_cmd_transaction;
    uint8_t last_cmd_kind;
    unsigned int cmd_seen:1;
    /* Recently transmitted STATUS uptime values act as per-boot challenges.
     * A host echoes one exactly, instead of estimating clock offset. */
    uint32_t status_challenges[8];
    unsigned int status_challenge_head:3;
    B1_Coordinator coordinator; void *coordinator_context;
} B1_Store;

/* Each ingest takes a bounded *copy*. No network work in the mutex. */
int b1_store_init(B1_Store *s, B1_Lock lock, B1_Lock unlock, void *ctx,
    uint16_t session);
void b1_set_coordinator(B1_Store *s, B1_Coordinator fn, void *ctx);
int b1_ingest_motion(B1_Store *s,const B1_Motion *v);
int b1_ingest_line(B1_Store *s,const B1_Line *v);
int b1_ingest_terrain(B1_Store *s,const B1_Terrain *v);
int b1_ingest_obstacle(B1_Store *s,const B1_Obstacle *v);
int b1_ingest_mission(B1_Store *s,const B1_Mission *v);
/* One bounded message per call; prioritize events, then motion, then state. */
int b1_next_frame(B1_Store *s,uint32_t now_ms,B1_Frame *out);
/* Reserve a unique sequence for a measured sensor-detail frame. */
int b1_sensor_frame(B1_Store *s,B1_Frame *frame);
/* A command must echo a recently issued STATUS timestamp from this session.
 * STOP is intentionally exempt; it still uses normal session validation. */
int b1_recent_status_challenge(B1_Store *s,uint32_t token,
    uint32_t now_ms);
/* Returns an acknowledgment. Accepted START/STOP means coordinator accepted,
    not PWM proof. */
int b1_handle_command(B1_Store *s,const B1_Frame *request,B1_Frame *ack);
#endif
/*** end of file ***/

