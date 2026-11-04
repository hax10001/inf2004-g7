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
#define B1_SRC_MISSION 0x10u
/* All time stamps are monotonically increasing milliseconds since boot. */
typedef struct { uint16_t seq; uint32_t ms;
    int16_t left_delta_mm,right_delta_mm;
    int16_t left_speed_mm_s,right_speed_mm_s; uint32_t cumulative_mm;
        uint8_t checkpoint; } B1_Motion;
typedef struct { uint16_t seq; uint32_t ms;
    uint8_t ir,line,barcode,action,action_valid;
    uint16_t event_id; uint8_t event_kind; } B1_Line;
typedef struct { uint16_t seq; uint32_t ms; uint16_t hump_mm,max_hump_mm;
    uint8_t quality,mode; uint16_t event_id; uint8_t event_kind; } B1_Terrain;
typedef struct { uint16_t seq; uint32_t ms;
    uint8_t mission,faults,ready,running;
    uint8_t ready_blockers,remote_hold,barcode_hold;
    uint16_t pwm_left,pwm_right;
    uint16_t event_id; uint8_t event_kind; uint16_t event_value; } B1_Mission;
typedef struct { uint8_t source,kind; uint16_t id,value; uint32_t ms; }
    B1_Event;

/* Lock/unlock must be a real SMP-safe RTOS mutex and must be non-ISR. */
typedef void (*B1_Lock)(void *ctx);
typedef uint8_t (*B1_Coordinator)(uint8_t cmd, void *ctx);
typedef struct {
    B1_Lock lock,unlock; void *lock_context;
    B1_Motion motion; B1_Line line; B1_Terrain terrain;
        B1_Mission mission;
    uint8_t seen, mapping[4], mapping_valid, committed, uncertain;
    B1_Motion motions[B1_MOTION_DEPTH]; uint8_t motion_head,motion_count;
    B1_Event events[B1_EVENT_DEPTH]; uint8_t event_head,event_count;
    uint32_t dropped,invalid,dup,session_ms,last_snapshot_ms,
        last_heartbeat_ms,last_status_ms,checkpoint_mm,checkpoint_ms;
    uint8_t checkpoint_pending;
    uint16_t session,wire_seq,last_cmd_transaction;
    uint8_t last_cmd_kind,cmd_seen;
    /* Recently transmitted STATUS uptime values act as per-boot challenges.
     * A host echoes one exactly, instead of estimating clock offset. */
    uint32_t status_challenges[8];
    uint8_t status_challenge_head;
    B1_Coordinator coordinator; void *coordinator_context;
} B1_Store;

/* Each ingest takes a bounded *copy*. No network work in the mutex. */
int b1_store_init(B1_Store *s, B1_Lock lock, B1_Lock unlock, void *ctx,
    uint16_t session);
void b1_set_coordinator(B1_Store *s, B1_Coordinator fn, void *ctx);
int b1_ingest_motion(B1_Store *s,const B1_Motion *v);
int b1_ingest_line(B1_Store *s,const B1_Line *v);
int b1_ingest_terrain(B1_Store *s,const B1_Terrain *v);
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

