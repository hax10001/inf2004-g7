/** @file buddy1_protocol.h
 * @brief Versioned binary protocol fields and helper types.
 */
/* Buddy 1 compact application wire protocol; no C bitfield layout on wire. */
#ifndef BUDDY1_PROTOCOL_H
#define BUDDY1_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>
#define B1_VERSION 1u
#define B1_MAGIC 0xb1u
#define B1_MAX_PAYLOAD 48u
#define B1_MAX_FRAME (13u + B1_MAX_PAYLOAD + 1u)
#define B1_HEADER 13u
#define B1_TYPE_SNAPSHOT 1u
#define B1_TYPE_ODOM 2u
#define B1_TYPE_EVENT 3u
#define B1_TYPE_CHECKPOINT 4u
#define B1_TYPE_HEARTBEAT 5u
#define B1_TYPE_SENSOR 6u
#define B1_TYPE_STATUS 7u
#define B1_TYPE_BARCODE_DIAG 8u
#define B1_TYPE_COMMAND 32u
#define B1_TYPE_ACK 33u
#define B1_CMD_MAPPING 1u
#define B1_CMD_START 2u
#define B1_CMD_STOP 3u
#define B1_CMD_SNAPSHOT 4u
#define B1_CMD_RECAL_IMU 5u
#define B1_OK 0u
#define B1_BAD_COMMAND 1u
#define B1_BAD_STATE 2u
#define B1_BAD_MAPPING 3u
#define B1_DUPLICATE 4u
#define B1_NOT_READY 5u
#define B1_PENDING 6u
#define B1_STALE_COMMAND 7u

typedef struct {
    uint8_t type, flags, payload[B1_MAX_PAYLOAD], payload_len;
    uint16_t session, seq;
    uint32_t time_ms;
} B1_Frame;

typedef struct {
    uint8_t source_mask, stale_mask, mission, line, ir;
    uint8_t barcode, action, action_valid, side, bypass_count, faults;
    uint16_t hump_mm, max_hump_mm, obstacle_cm;
    int16_t speed_left_mm_s, speed_right_mm_s;
} B1_SnapshotPayload;

typedef struct {
    uint8_t command, mapping[4];
    uint16_t transaction;
} B1_Command;

size_t b1_frame_encode(const B1_Frame *in, uint8_t *out, size_t capacity);
int b1_frame_decode(const uint8_t *bytes, size_t size, B1_Frame *out);
int b1_pack_mapping(const uint8_t letters[4], uint8_t out[3]);
int b1_unpack_mapping(const uint8_t in[3], uint8_t letters[4]);
size_t b1_encode_snapshot(const B1_SnapshotPayload *s, uint8_t dst[13]);
int b1_decode_snapshot(const uint8_t *src, size_t size, B1_SnapshotPayload *s);
size_t b1_encode_command(const B1_Command *cmd, uint8_t *dest, size_t capacity);
int b1_decode_command(const uint8_t *src, size_t size, B1_Command *cmd);
void b1_put16(uint8_t *p, uint16_t value);
uint16_t b1_get16(const uint8_t *p);
void b1_put32(uint8_t *p, uint32_t value);
uint32_t b1_get32(const uint8_t *p);
#endif
/*** end of file ***/

