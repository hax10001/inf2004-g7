/** @file buddy1_protocol.h
 * @brief Buddy 1 binary wire protocol: offsets, sizes and frame types.
 * Payloads are serialised explicitly; C bit-field layout never reaches
 * the wire.
 */
#ifndef BUDDY1_PROTOCOL_H
#define BUDDY1_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>
#define B1_VERSION 1u
#define B1_MAGIC 0xb1u
#define B1_MAX_PAYLOAD 48u
#define B1_MAX_FRAME (B1_HEADER + B1_MAX_PAYLOAD + 1u)
#define B1_HEADER 13u
/* Frame header byte offsets (magic 0, version 1). */
#define B1_HDR_OFF_TYPE 2       /* message type */
#define B1_HDR_OFF_FLAGS 3      /* flags (bit 0 uncertain) */
#define B1_HDR_OFF_SESSION 4    /* 16-bit session */
#define B1_HDR_OFF_SEQ 6        /* 16-bit sequence */
#define B1_HDR_OFF_TIME 8       /* 32-bit time stamp, ms */
#define B1_HDR_OFF_LEN 12       /* payload length */
/* Little-endian helpers: shift of one byte and of one 16-bit half. */
#define B1_BYTE_SHIFT 8
#define B1_HALF_SHIFT 16
#define B1_HALF_BYTES 2         /* byte offset of the upper 16-bit half */
#define B1_BITS_PER_BYTE 8u     /* bit writer/reader and CRC loop */
#define B1_BITS_ROUND_UP 7u     /* rounds a bit count up to whole bytes */
#define B1_CRC_POLY 0x07u       /* CRC-8 polynomial */
#define B1_CRC_TOP_BIT 0x80u    /* CRC-8 carry-out bit */
/* Code 39 letters A-Z are 0..25; 31 means no barcode. */
#define B1_LETTER_MAX 25u       /* highest letter code (Z) */
#define B1_LETTER_COUNT 26U     /* letters A-Z */
#define B1_LETTER_NONE 31u      /* no-barcode sentinel */
#define B1_LETTER_MASK 31u      /* five-bit letter mask */
#define B1_LETTER_BITS 5u       /* bits per letter */
/* Route mapping: four letters packed in 20 bits over three bytes. */
#define B1_MAP_LETTERS 4u       /* letters per mapping */
#define B1_MAP_BYTES 3u         /* packed mapping bytes */
#define B1_MAP_BYTE_HI 2        /* top byte (bits 16..19) */
#define B1_MAP_IDX_L2 2         /* third letter index */
#define B1_MAP_IDX_L3 3         /* fourth letter index */
#define B1_MAP_SHIFT_L2 10u     /* third letter bit position */
#define B1_MAP_SHIFT_L3 15u     /* fourth letter bit position */
#define B1_MAP_RESERVED 0xf0u   /* top byte bits that must be zero */
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
#define B1_CMD_RECAL_BARCODE 6u   /* stopped only: relearn barcode white */
#define B1_CMD_LAST B1_CMD_RECAL_BARCODE
/* Command payload: kind, 16-bit transaction, then a mapping if any. */
#define B1_CMD_OFF_TRANSACTION 1 /* 16-bit transaction */
#define B1_CMD_OFF_MAPPING 3     /* packed mapping (MAPPING only) */
#define B1_CMD_LEN 3u            /* command without a mapping */
#define B1_CMD_LEN_MAPPING 6u    /* MAPPING command */
/* ACK payload: command, 16-bit transaction, result. */
#define B1_ACK_LEN 4u
#define B1_ACK_OFF_TRANSACTION 1 /* 16-bit transaction */
#define B1_ACK_OFF_RESULT 3      /* B1_OK etc. */
/* STATUS payload (14 bytes): byte 0 mapping valid, 1 ready. */
#define B1_STATUS_LEN 14u
#define B1_STATUS_OFF_RUNNING 2      /* mission running */
#define B1_STATUS_OFF_SEEN 3         /* source mask */
#define B1_STATUS_OFF_MISSION 4      /* mission state */
#define B1_STATUS_OFF_BLOCKERS 5     /* ready blockers */
#define B1_STATUS_OFF_HOLD 6         /* remote hold */
#define B1_STATUS_OFF_TRANSACTION 7  /* 16-bit last command transaction */
#define B1_STATUS_OFF_BC_HOLD 9      /* barcode motion hold */
#define B1_STATUS_OFF_PWM_LEFT 10    /* 16-bit left PWM */
#define B1_STATUS_OFF_PWM_RIGHT 12   /* 16-bit right PWM */
/* EVENT payload: source, kind, 16-bit ID, 16-bit value. */
#define B1_EVENT_LEN 6u
#define B1_EVENT_OFF_ID 2
#define B1_EVENT_OFF_VALUE 4
/* ODOM payload: 16-bit left, right deltas and sequence. */
#define B1_ODOM_LEN 6u
/* CHECKPOINT (cumulative mm) and HEARTBEAT (dropped) are 32-bit. */
#define B1_CHECKPOINT_LEN 4u
#define B1_HEARTBEAT_LEN 4u
/* Sensor detail: 24 bytes, or 32 with obstacle diagnostics. Each may be
 * followed by one 16-bit lift word: height 0..1022 mm in bits 0..9 (1023
 * unavailable), lifted flag in bit 10, bits 11..15 zero. */
#define B1_SENSOR_V1 24u
#define B1_SENSOR_DIAG 32u
#define B1_SENSOR_LIFT (B1_SENSOR_V1 + 2u)
#define B1_SENSOR_DIAG_LIFT (B1_SENSOR_DIAG + 2u)
/* Sensor payload byte offsets (byte 0 valid mask, byte 1 IR bits). */
#define B1_SENSOR_OFF_ROLL 2         /* 16-bit roll, centidegrees */
#define B1_SENSOR_OFF_PITCH 4        /* 16-bit pitch, centidegrees */
#define B1_SENSOR_OFF_HEADING 6      /* 16-bit heading, centidegrees */
#define B1_SENSOR_OFF_HEADING_HI 7   /* heading high byte */
#define B1_SENSOR_OFF_DISTANCE 8     /* 16-bit obstacle distance, cm */
#define B1_SENSOR_OFF_HUMP_PHASE 10  /* hump phase */
#define B1_SENSOR_OFF_CALIBRATED 11  /* IMU calibrated */
#define B1_SENSOR_OFF_LEFT_COUNT 12  /* 32-bit left encoder count */
#define B1_SENSOR_OFF_RIGHT_COUNT 16 /* 32-bit right encoder count */
#define B1_SENSOR_OFF_OBS_PHASE 20   /* obstacle phase */
#define B1_SENSOR_OFF_ON_HUMP 21     /* on a hump */
#define B1_SENSOR_OFF_HEIGHT_OK 22   /* height valid */
#define B1_SENSOR_OFF_ODOM_CONF 23   /* odometry confidence */
#define B1_SENSOR_OFF_LIFT 24        /* 16-bit lift word */
#define B1_LIFT_FLAG 0x0400u         /* lift word: lifted flag, bit 10 */
/* Barcode diagnostics: 28 bytes (V1), 42 with scanner state/white (V2). */
#define B1_BARCODE_DIAG_V1 28u
#define B1_BARCODE_DIAG_V2 42u
/* Barcode diagnostics byte offsets (byte 0 calibration, 1 state). */
#define B1_BCD_OFF_ERROR 2         /* last error */
#define B1_BCD_OFF_RAW 3           /* raw IR, low byte */
#define B1_BCD_OFF_REJECTED 4      /* 32-bit rejected scans */
#define B1_BCD_OFF_COMPLETED 8     /* 32-bit completed scans */
#define B1_BCD_OFF_SCAN_UNITS 12   /* 16-bit last scan units */
#define B1_BCD_OFF_RUNS 14         /* run count */
#define B1_BCD_OFF_ERROR_COPY 15   /* last error (V1 layout) */
#define B1_BCD_OFF_LEFT 16         /* 32-bit left encoder count */
#define B1_BCD_OFF_RIGHT 20        /* 32-bit right encoder count */
#define B1_BCD_OFF_EDGES 24        /* 32-bit edges */
#define B1_BCD_OFF_WAVE_STATE 28   /* V2: scanner state */
#define B1_BCD_OFF_WHITE_REJ 29    /* V2: white rejects */
#define B1_BCD_OFF_WHITE_MEAN 30   /* V2: 16-bit white mean */
#define B1_BCD_OFF_WHITE_SIGMA 32  /* V2: 16-bit white sigma */
#define B1_BCD_OFF_WAVE_SEQ 34     /* V2: 16-bit scan sequence */
#define B1_BCD_OFF_LAST_VALID 36   /* V2: last pass valid */
#define B1_BCD_OFF_LAST_LETTER 37  /* V2: last pass letter */
#define B1_BCD_OFF_LAST_REASON 38  /* V2: last pass reason */
#define B1_BCD_OFF_RESERVED 39     /* V2: reserved, zero */
#define B1_BCD_OFF_PROCESS_MS 40   /* V2: 16-bit last process time */
#define B1_OK 0u
#define B1_BAD_COMMAND 1u
#define B1_BAD_STATE 2u
#define B1_BAD_MAPPING 3u
#define B1_DUPLICATE 4u
#define B1_NOT_READY 5u
#define B1_PENDING 6u
#define B1_STALE_COMMAND 7u

/* SNAPSHOT payload: 13 bytes of LSB-first bit fields. */
#define B1_SNAPSHOT_LEN 13u
#define B1_SNAPSHOT_LAST 12          /* last byte; top five bits reserved */
#define B1_SNAPSHOT_PAD 0xf8u        /* reserved bits of the last byte */
#define B1_SNAP_W_SOURCES 5u         /* source and stale masks */
#define B1_SNAP_W_MISSION 3u
#define B1_SNAP_W_LINE 3u
#define B1_SNAP_W_IR 3u
#define B1_SNAP_W_BARCODE 5u
#define B1_SNAP_W_ACTION 2u
#define B1_SNAP_W_RANGE 10u          /* hump, max hump, obstacle */
#define B1_SNAP_W_SIDE 2u
#define B1_SNAP_W_BYPASS 8u
#define B1_SNAP_W_FAULTS 8u
#define B1_SNAP_W_SPEED 12u          /* signed wheel speeds */
#define B1_SOURCES_MASK 31u          /* five-bit source mask */
#define B1_MISSION_MASK 7u           /* three-bit mission state */
#define B1_LINE_MASK 7u              /* three-bit line state */
#define B1_IR_MASK 7u                /* three IR bits */
#define B1_ACTION_MASK 3u            /* two-bit action */
/* 10-bit snapshot fields: 1023 means unavailable on the wire. */
#define B1_RANGE10_NONE 1023u
/* Signed 12-bit speed: -2048 is reserved on the wire. */
#define B1_SPEED12_MAX 2047
#define B1_SPEED12_MASK 0xfffu       /* 12-bit field */
#define B1_SPEED12_SIGN 0x800u       /* 12-bit sign bit */
#define B1_SPEED12_EXTEND 0xf000u    /* sign extension to 16 bits */

typedef struct {
    uint8_t type;
    uint8_t flags;
    uint8_t payload[B1_MAX_PAYLOAD];
    uint8_t payload_len;
    uint16_t session;
    uint16_t seq;
    uint32_t time_ms;
} b1_frame_t;

/* Snapshot fields have the same widths as the wire format, which
 * b1_encode_snapshot() packs into 13 bytes. 1023 means unavailable;
 * speeds are clamped to -2047..2047 (-2048 is reserved). */
typedef struct {
    uint32_t     hump_mm:10;
    uint32_t     max_hump_mm:10;
    uint32_t     obstacle_cm:10;
    int32_t    speed_left_mm_s:12;
    int32_t    speed_right_mm_s:12;
    uint8_t       source_mask:5;
    uint8_t       stale_mask:5;
    uint8_t       mission:3;
    uint8_t       line:3;
    uint8_t       ir:3;
    uint8_t       barcode:5;
    uint8_t       action:2;
    uint8_t       action_valid:1;
    uint8_t       side:2;
    uint8_t       bypass_count:8;
    uint8_t       faults:8;
} b1_snapshot_payload_t;

typedef struct {
    uint8_t command;
    uint8_t mapping[B1_MAP_LETTERS];
    uint16_t transaction;
} b1_command_t;

size_t b1_frame_encode(const b1_frame_t *p_in, uint8_t *p_out, size_t capacity);
int32_t b1_frame_decode(const uint8_t *p_bytes, size_t size, b1_frame_t *p_out);
int32_t b1_pack_mapping(const uint8_t letters[B1_MAP_LETTERS],
    uint8_t out[B1_MAP_BYTES]);
int32_t b1_unpack_mapping(const uint8_t in[B1_MAP_BYTES],
    uint8_t letters[B1_MAP_LETTERS]);
size_t b1_encode_snapshot(const b1_snapshot_payload_t *p_snap,
    uint8_t dst[B1_SNAPSHOT_LEN]);
int32_t b1_decode_snapshot(const uint8_t *p_src, size_t size,
    b1_snapshot_payload_t *p_snap);
size_t b1_encode_command(const b1_command_t *p_cmd, uint8_t *p_dest,
    size_t capacity);
int32_t b1_decode_command(const uint8_t *p_src, size_t size,
    b1_command_t *p_cmd);
void b1_put16(uint8_t *p_buf, uint16_t value);
uint16_t b1_get16(const uint8_t *p_buf);
void b1_put32(uint8_t *p_buf, uint32_t value);
uint32_t b1_get32(const uint8_t *p_buf);
#endif
/*** end of file ***/

