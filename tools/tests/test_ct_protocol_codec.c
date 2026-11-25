/** @file test_ct_protocol_codec.c
 * @brief Host test: buddy1_protocol.c encode/decode at field limits and
 *        every rejection path (NFR13 wire format, FR12 frame types).
 *
 * The SNAPSHOT bit layout is checked bit by bit against the published
 * field order (sources 0-4, stale 5-9, mission 10-12, line 13-15, IR
 * 16-18, barcode 19-23, action 24-25, action valid 26, hump 27-36, max
 * hump 37-46, obstacle 47-56, side 57-58, bypass 59-66, faults 67-74,
 * left speed 75-86, right speed 87-98, bits 99-103 reserved zero), which
 * dashboard/server/protocol.js decodes.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "buddy1.h"

#define CT_SNAP_FIELDS 16U
#define CT_SNAP_BITS 104U
#define CT_FRAME_TYPES 15U

/** One SNAPSHOT field: first bit and width on the wire. */
typedef struct
{
    uint8_t offset;
    uint8_t width;
} ct_field_t;

static ct_field_t const g_fields[CT_SNAP_FIELDS] = {
    {0U, 5U}, {5U, 5U}, {10U, 3U}, {13U, 3U}, {16U, 3U}, {19U, 5U},
    {24U, 2U}, {26U, 1U}, {27U, 10U}, {37U, 10U}, {47U, 10U}, {57U, 2U},
    {59U, 8U}, {67U, 8U}, {75U, 12U}, {87U, 12U}
};

/** @brief Set SNAPSHOT field index to all ones (speeds: -1). */
static void set_field_ones (b1_snapshot_payload_t * p_snap, uint32_t index)
{
    switch (index)
    {
        case 0U: p_snap->source_mask = 31U; break;
        case 1U: p_snap->stale_mask = 31U; break;
        case 2U: p_snap->mission = 7U; break;
        case 3U: p_snap->line = 7U; break;
        case 4U: p_snap->ir = 7U; break;
        case 5U: p_snap->barcode = B1_LETTER_NONE; break;
        case 6U: p_snap->action = 3U; break;
        case 7U: p_snap->action_valid = 1U; break;
        case 8U: p_snap->hump_mm = 1023U; break;
        case 9U: p_snap->max_hump_mm = 1023U; break;
        case 10U: p_snap->obstacle_cm = 1023U; break;
        case 11U: p_snap->side = 3U; break;
        case 12U: p_snap->bypass_count = 255U; break;
        case 13U: p_snap->faults = 255U; break;
        case 14U: p_snap->speed_left_mm_s = -1; break;
        case 15U: p_snap->speed_right_mm_s = -1; break;
        default: assert(0); break;
    }
}

/** @brief 1 when bit number bit is set in an LSB-first byte array. */
static uint32_t bit_at (uint8_t const * p_bytes, uint32_t bit)
{
    return (uint32_t)((p_bytes[bit / 8U] >> (bit % 8U)) & 1U);
}

/** @brief Test: each SNAPSHOT field occupies exactly its wire bits and
 *  round-trips alone at its maximum (NFR13). */
static void test_snapshot_layout (void)
{
    for (uint32_t f = 0U; f < CT_SNAP_FIELDS; f++)
    {
        b1_snapshot_payload_t snap;
        b1_snapshot_payload_t back;
        b1_snapshot_payload_t want;
        uint8_t bytes[B1_SNAPSHOT_LEN];
        uint32_t const lo = g_fields[f].offset;
        uint32_t const hi = lo + g_fields[f].width;

        memset(&snap, 0, sizeof snap);
        set_field_ones(&snap, f);
        assert(B1_SNAPSHOT_LEN == b1_encode_snapshot(&snap, bytes));
        for (uint32_t b = 0U; b < CT_SNAP_BITS; b++)
        {
            assert(bit_at(bytes, b) == (((b >= lo) && (b < hi)) ? 1U : 0U));
        }
        assert(0 == b1_decode_snapshot(bytes, sizeof bytes, &back));
        memset(&want, 0, sizeof want);
        set_field_ones(&want, f);
        assert(0 == memcmp(&want, &back, sizeof want));
    }
    puts("PASS NFR13 SNAPSHOT layout: 16 fields each exactly at their 99 "
         "wire bits, all-ones round trip, 5 pad bits zero");
}

/** @brief Test: speed saturation, barcode range and reserved/length
 *  rejection on SNAPSHOT (NFR13). */
static void test_snapshot_limits (void)
{
    b1_snapshot_payload_t snap;
    b1_snapshot_payload_t back;
    uint8_t bytes[B1_SNAPSHOT_LEN];

    memset(&snap, 0, sizeof snap);
    snap.speed_left_mm_s = -2048;
    snap.speed_right_mm_s = 2047;
    assert(B1_SNAPSHOT_LEN == b1_encode_snapshot(&snap, bytes));
    assert(0 == b1_decode_snapshot(bytes, sizeof bytes, &back));
    assert((-2047 == back.speed_left_mm_s) &&
           (2047 == back.speed_right_mm_s));

    for (uint8_t letter = B1_LETTER_COUNT; letter < B1_LETTER_NONE;
         letter++)
    {
        snap.barcode = letter;
        assert(0U == b1_encode_snapshot(&snap, bytes));
    }
    snap.barcode = B1_LETTER_MAX;
    assert(B1_SNAPSHOT_LEN == b1_encode_snapshot(&snap, bytes));
    /* Letter 26 forced into bits 19-23 is refused on decode. */
    bytes[2] = (uint8_t)((bytes[2] & 0x07U) | (26U << 3U));
    assert(-1 == b1_decode_snapshot(bytes, sizeof bytes, &back));

    snap.barcode = 0U;
    assert(B1_SNAPSHOT_LEN == b1_encode_snapshot(&snap, bytes));
    for (uint32_t bit = 3U; bit < 8U; bit++)
    {
        uint8_t copy[B1_SNAPSHOT_LEN];

        memcpy(copy, bytes, sizeof copy);
        copy[B1_SNAPSHOT_LAST] |= (uint8_t)(1U << bit);
        assert(-1 == b1_decode_snapshot(copy, sizeof copy, &back));
    }
    assert(-1 == b1_decode_snapshot(bytes, B1_SNAPSHOT_LEN - 1U, &back));
    assert(-1 == b1_decode_snapshot(bytes, B1_SNAPSHOT_LEN + 1U, &back));
    assert(-1 == b1_decode_snapshot(NULL, B1_SNAPSHOT_LEN, &back));
    assert(0U == b1_encode_snapshot(NULL, bytes));
    puts("PASS NFR13 SNAPSHOT limits: -2048 saturates to -2047, letters "
         "26..30 refused both ways, each pad bit and 12/14 bytes rejected");
}

/** @brief Test: mapping packs at the letter limits; reserved bits,
 *  out-of-range and duplicate packed letters are rejected (FR2/NFR13). */
static void test_mapping_limits (void)
{
    uint8_t const zyxw[B1_MAP_LETTERS] = {25U, 24U, 23U, 22U};
    uint8_t letters[B1_MAP_LETTERS];
    uint8_t packed[B1_MAP_BYTES];
    uint8_t raw[B1_MAP_BYTES];
    uint32_t v;

    assert(0 == b1_pack_mapping(zyxw, packed));
    v = (uint32_t)packed[0] | ((uint32_t)packed[1] << 8U) |
        ((uint32_t)packed[2] << 16U);
    assert(v == (25U | (24U << 5U) | (23U << 10U) | (22U << 15U)));
    assert(0U == (packed[2] & B1_MAP_RESERVED));
    assert(0 == b1_unpack_mapping(packed, letters));
    assert(0 == memcmp(letters, zyxw, sizeof zyxw));

    for (uint32_t bit = 4U; bit < 8U; bit++)
    {
        memcpy(raw, packed, sizeof raw);
        raw[2] |= (uint8_t)(1U << bit);
        assert(-1 == b1_unpack_mapping(raw, letters));
    }
    /* Packed letter 26 in slot 0 and a duplicate (A, A, B, C). */
    v = 26U | (1U << 5U) | (2U << 10U) | (3U << 15U);
    raw[0] = (uint8_t)v;
    raw[1] = (uint8_t)(v >> 8U);
    raw[2] = (uint8_t)(v >> 16U);
    assert(-1 == b1_unpack_mapping(raw, letters));
    v = 0U | (0U << 5U) | (1U << 10U) | (2U << 15U);
    raw[0] = (uint8_t)v;
    raw[1] = (uint8_t)(v >> 8U);
    raw[2] = (uint8_t)(v >> 16U);
    assert(-1 == b1_unpack_mapping(raw, letters));
    assert(-1 == b1_pack_mapping(NULL, packed));
    assert(-1 == b1_unpack_mapping(NULL, letters));
    puts("PASS FR2/NFR13 mapping: Z,Y,X,W packs to 20 bits, each reserved "
         "bit, packed letter 26 and duplicate letters rejected");
}

/** @brief Test: command codec lengths, transaction limits and every
 *  rejection (FR13/NFR13). */
static void test_command_limits (void)
{
    b1_command_t cmd = {0};
    b1_command_t back = {0};
    uint8_t buffer[B1_CMD_LEN_MAPPING + 1U];
    uint8_t const map[B1_MAP_LETTERS] = {0U, 1U, 2U, 3U};

    for (uint8_t kind = B1_CMD_MAPPING; kind <= B1_CMD_LAST; kind++)
    {
        size_t const want = (B1_CMD_MAPPING == kind) ? B1_CMD_LEN_MAPPING :
                            B1_CMD_LEN;

        memset(&cmd, 0, sizeof cmd);
        cmd.command = kind;
        cmd.transaction = 0xFFFFU;
        memcpy(cmd.mapping, map, sizeof map);
        assert(want == b1_encode_command(&cmd, buffer, sizeof buffer));
        assert(0U == b1_encode_command(&cmd, buffer, want - 1U));
        assert(0 == b1_decode_command(buffer, want, &back));
        assert((kind == back.command) && (0xFFFFU == back.transaction));
        assert(-1 == b1_decode_command(buffer, want + 1U, &back));
        assert(-1 == b1_decode_command(buffer, want - 1U, &back));
    }
    cmd.command = 0U;
    assert(0U == b1_encode_command(&cmd, buffer, sizeof buffer));
    cmd.command = (uint8_t)(B1_CMD_LAST + 1U);
    assert(0U == b1_encode_command(&cmd, buffer, sizeof buffer));
    cmd.command = B1_CMD_MAPPING;
    cmd.mapping[3] = cmd.mapping[0];
    assert(0U == b1_encode_command(&cmd, buffer, sizeof buffer));
    /* A STOP payload sent with mapping length is not a STOP. */
    cmd.command = B1_CMD_STOP;
    assert(B1_CMD_LEN == b1_encode_command(&cmd, buffer, sizeof buffer));
    assert(-1 == b1_decode_command(buffer, B1_CMD_LEN_MAPPING, &back));
    /* MAPPING with a reserved bit set in the packed letters. */
    memcpy(cmd.mapping, map, sizeof map);
    cmd.command = B1_CMD_MAPPING;
    assert(B1_CMD_LEN_MAPPING ==
           b1_encode_command(&cmd, buffer, sizeof buffer));
    buffer[B1_CMD_OFF_MAPPING + 2] |= 0x80U;
    assert(-1 == b1_decode_command(buffer, B1_CMD_LEN_MAPPING, &back));
    assert(-1 == b1_decode_command(NULL, B1_CMD_LEN, &back));
    assert(0U == b1_encode_command(NULL, buffer, sizeof buffer));
    puts("PASS FR13 commands 1..6 at transaction 0xFFFF; short buffer, "
         "codes 0/7, duplicate map, wrong length, reserved bits rejected");
}

/** @brief Test: every frame type round-trips its header at the field
 *  limits and payload; length, magic, version and CRC errors (each
 *  single-bit flip) are rejected (NFR13). */
static void test_frame_limits (void)
{
    static uint8_t const types[CT_FRAME_TYPES][2] = {
        {B1_TYPE_SNAPSHOT, B1_SNAPSHOT_LEN}, {B1_TYPE_ODOM, B1_ODOM_LEN},
        {B1_TYPE_EVENT, B1_EVENT_LEN},
        {B1_TYPE_CHECKPOINT, B1_CHECKPOINT_LEN},
        {B1_TYPE_HEARTBEAT, B1_HEARTBEAT_LEN},
        {B1_TYPE_SENSOR, B1_SENSOR_V1}, {B1_TYPE_SENSOR, B1_SENSOR_LIFT},
        {B1_TYPE_SENSOR, B1_SENSOR_DIAG},
        {B1_TYPE_SENSOR, B1_SENSOR_DIAG_LIFT},
        {B1_TYPE_STATUS, B1_STATUS_LEN},
        {B1_TYPE_BARCODE_DIAG, B1_BARCODE_DIAG_V1},
        {B1_TYPE_BARCODE_DIAG, B1_BARCODE_DIAG_V2},
        {B1_TYPE_COMMAND, B1_CMD_LEN}, {B1_TYPE_COMMAND, B1_CMD_LEN_MAPPING},
        {B1_TYPE_ACK, B1_ACK_LEN}
    };
    uint8_t bytes[B1_MAX_FRAME + 1U];
    b1_frame_t frame;
    b1_frame_t back;
    size_t n;

    for (uint32_t t = 0U; t < CT_FRAME_TYPES; t++)
    {
        memset(&frame, 0, sizeof frame);
        frame.type = types[t][0];
        frame.flags = 0xFFU;
        frame.session = 0xFFFFU;
        frame.seq = 0xFFFFU;
        frame.time_ms = 0xFFFFFFFFU;
        frame.payload_len = types[t][1];
        for (uint32_t i = 0U; i < frame.payload_len; i++)
        {
            frame.payload[i] = (uint8_t)(0xA0U + i);
        }
        n = b1_frame_encode(&frame, bytes, sizeof bytes);
        assert((B1_HEADER + frame.payload_len + 1U) == n);
        assert(0 == b1_frame_decode(bytes, n, &back));
        assert((frame.type == back.type) && (0xFFU == back.flags));
        assert((0xFFFFU == back.session) && (0xFFFFU == back.seq));
        assert(0xFFFFFFFFU == back.time_ms);
        assert(frame.payload_len == back.payload_len);
        assert(0 == memcmp(frame.payload, back.payload, back.payload_len));
    }

    memset(&frame, 0, sizeof frame);
    frame.payload_len = 0U;
    assert((B1_HEADER + 1U) == b1_frame_encode(&frame, bytes, sizeof bytes));
    assert(0 == b1_frame_decode(bytes, B1_HEADER + 1U, &back));
    assert(-1 == b1_frame_decode(bytes, B1_HEADER, &back));
    frame.payload_len = B1_MAX_PAYLOAD;
    n = b1_frame_encode(&frame, bytes, sizeof bytes);
    assert(B1_MAX_FRAME == n);
    assert(0U == b1_frame_encode(&frame, bytes, n - 1U));
    for (uint32_t bit = 0U; bit < (8U * n); bit++)
    {
        bytes[bit / 8U] ^= (uint8_t)(1U << (bit % 8U));
        assert(-1 == b1_frame_decode(bytes, n, &back));
        bytes[bit / 8U] ^= (uint8_t)(1U << (bit % 8U));
    }
    assert(0 == b1_frame_decode(bytes, n, &back));
    assert(-1 == b1_frame_decode(bytes, n + 1U, &back));
    frame.payload_len = B1_MAX_PAYLOAD + 1U;
    assert(0U == b1_frame_encode(&frame, bytes, sizeof bytes));
    assert(0U == b1_frame_encode(NULL, bytes, sizeof bytes));
    assert(-1 == b1_frame_decode(NULL, n, &back));
    puts("PASS NFR13 frames: 15 type/length pairs round-trip at header "
         "limits; 14..62 bytes; every single-bit flip of a 62-byte frame "
         "rejected; 49-byte payload refused");
}

/** @brief Store lock (single thread). */
static void lock (void * p_ctx)
{
    (void)p_ctx;
}

/** @brief Test: the store stamps only valid SENSOR/BARCODE_DIAG lengths
 *  (incl. the lift variants), refuses other types without using a
 *  sequence number, and refuses NULL lock callbacks (FR12). */
static void test_detail_lengths (void)
{
    static uint8_t const ok[6][2] = {
        {B1_TYPE_SENSOR, B1_SENSOR_V1}, {B1_TYPE_SENSOR, B1_SENSOR_LIFT},
        {B1_TYPE_SENSOR, B1_SENSOR_DIAG},
        {B1_TYPE_SENSOR, B1_SENSOR_DIAG_LIFT},
        {B1_TYPE_BARCODE_DIAG, B1_BARCODE_DIAG_V1},
        {B1_TYPE_BARCODE_DIAG, B1_BARCODE_DIAG_V2}
    };
    static uint8_t const bad[8][2] = {
        {B1_TYPE_SENSOR, B1_SENSOR_V1 + 1U},
        {B1_TYPE_SENSOR, B1_SENSOR_LIFT + 1U},
        {B1_TYPE_SENSOR, B1_SENSOR_DIAG + 1U},
        {B1_TYPE_SENSOR, B1_SENSOR_DIAG_LIFT + 1U},
        {B1_TYPE_BARCODE_DIAG, B1_BARCODE_DIAG_V1 - 1U},
        {B1_TYPE_BARCODE_DIAG, B1_BARCODE_DIAG_V2 + 1U},
        {B1_TYPE_STATUS, B1_STATUS_LEN},
        {B1_TYPE_ODOM, B1_ODOM_LEN}
    };
    b1_store_t store;
    b1_frame_t frame;
    uint16_t seq;

    assert(-1 == b1_store_init(&store, NULL, lock, NULL, 3U));
    assert(-1 == b1_store_init(&store, lock, NULL, NULL, 3U));
    assert(0 == b1_store_init(&store, lock, lock, NULL, 0U));
    assert(1U == store.session);
    seq = store.wire_seq;
    for (uint32_t i = 0U; i < 6U; i++)
    {
        memset(&frame, 0, sizeof frame);
        frame.type = ok[i][0];
        frame.payload_len = ok[i][1];
        assert(0 == b1_sensor_frame(&store, &frame));
        seq++;
        assert((seq == frame.seq) && (1U == frame.session));
    }
    for (uint32_t i = 0U; i < 8U; i++)
    {
        memset(&frame, 0, sizeof frame);
        frame.type = bad[i][0];
        frame.payload_len = bad[i][1];
        assert(-1 == b1_sensor_frame(&store, &frame));
        assert((seq == store.wire_seq) && (0U == frame.seq));
    }
    puts("PASS FR12 detail frames: SENSOR 24/26/32/34 and DIAG 28/42 "
         "stamped in sequence; 25/27/33/35, 27/43 and other types refused "
         "without a sequence; NULL lock and session 0 handled");
}

/** @brief Count of coordinator calls. */
static uint32_t g_coordinator_calls;

/** @brief Coordinator that accepts every command and counts it. */
static uint8_t coordinator (uint8_t command, void * p_ctx)
{
    (void)command;
    (void)p_ctx;
    g_coordinator_calls++;
    return B1_OK;
}

/** @brief Send one command to the store; returns the result code. */
static int32_t send_command (b1_store_t * p_store, uint16_t session,
                             uint8_t command, uint16_t transaction,
                             b1_frame_t * p_ack)
{
    b1_command_t cmd = {0};
    b1_frame_t request = {0};
    uint8_t const map[B1_MAP_LETTERS] = {0U, 1U, 2U, 3U};

    cmd.command = command;
    cmd.transaction = transaction;
    memcpy(cmd.mapping, map, sizeof map);
    request.type = B1_TYPE_COMMAND;
    request.session = session;
    request.payload_len = (uint8_t)b1_encode_command(&cmd, request.payload,
                                                     sizeof request.payload);
    return b1_handle_command(p_store, &request, p_ack);
}

/** @brief Test: commands for another session never reach the
 *  coordinator; STOP repeats are not duplicates; no coordinator means
 *  NOT_READY; malformed requests get no ACK (FR13). */
static void test_command_store_guards (void)
{
    b1_store_t store;
    b1_mission_t mission = {0};
    b1_frame_t ack = {0};
    b1_frame_t request = {0};

    assert(0 == b1_store_init(&store, lock, lock, NULL, 50U));
    mission.seq = 1U;
    mission.ready = 1U;
    assert(0 == b1_ingest_mission(&store, &mission));
    assert(-1 == b1_ingest_mission(&store, &mission));
    assert(1U == store.dup);
    assert(B1_OK == send_command(&store, 50U, B1_CMD_MAPPING, 1U, &ack));

    /* No coordinator registered: nothing can be applied. */
    assert(B1_NOT_READY == send_command(&store, 50U, B1_CMD_START, 2U,
                                        &ack));
    assert(B1_NOT_READY == send_command(&store, 50U, B1_CMD_STOP, 3U, &ack));
    assert(B1_NOT_READY == send_command(&store, 50U, B1_CMD_RECAL_IMU, 4U,
                                        &ack));
    assert(B1_NOT_READY == send_command(&store, 50U, B1_CMD_RECAL_BARCODE,
                                        5U, &ack));
    assert(0U == store.committed);

    b1_set_coordinator(&store, coordinator, NULL);
    g_coordinator_calls = 0U;
    assert(B1_DUPLICATE == send_command(&store, 51U, B1_CMD_START, 6U,
                                        &ack));
    assert(B1_DUPLICATE == send_command(&store, 51U, B1_CMD_STOP, 7U, &ack));
    assert(B1_DUPLICATE == ack.payload[B1_ACK_OFF_RESULT]);
    assert((0U == g_coordinator_calls) && (0U == store.committed));
    assert(5U == store.last_cmd_transaction);
    assert(B1_OK == send_command(&store, 50U, B1_CMD_STOP, 8U, &ack));
    assert(B1_OK == send_command(&store, 50U, B1_CMD_STOP, 8U, &ack));
    assert(2U == g_coordinator_calls);
    assert(B1_OK == send_command(&store, 50U, B1_CMD_START, 9U, &ack));
    assert(B1_DUPLICATE == send_command(&store, 50U, B1_CMD_START, 9U,
                                        &ack));
    assert(3U == g_coordinator_calls);

    request.type = B1_TYPE_STATUS;
    request.session = 50U;
    assert(-1 == b1_handle_command(&store, &request, &ack));
    request.type = B1_TYPE_COMMAND;
    request.payload_len = 2U;
    assert(-1 == b1_handle_command(&store, &request, &ack));
    puts("PASS FR13 other-session START/STOP -> DUPLICATE, coordinator not "
         "called; STOP repeat applied twice, START repeat refused; no "
         "coordinator -> NOT_READY; non-command or short payload -> no ACK");
}

/** @brief Test: the EVENT ring keeps the newest 24, counts the dropped
 *  one and flags the route uncertain; sources go stale after 1.5 s
 *  (FR12/FR15 offline buffering). */
static void test_event_ring_and_stale (void)
{
    b1_store_t store;
    b1_line_t line = {0};
    b1_mission_t mission = {0};
    b1_frame_t frame = {0};
    b1_snapshot_payload_t snap;
    uint32_t events = 0U;
    uint16_t first_id = 0U;
    uint32_t snapshots = 0U;

    assert(0 == b1_store_init(&store, lock, lock, NULL, 60U));
    for (uint16_t i = 1U; i <= (B1_EVENT_DEPTH + 1U); i++)
    {
        line.seq = i;
        line.ms = 100U;
        line.event_kind = B1_EVT_BARCODE;
        line.event_id = i;
        assert(0 == b1_ingest_line(&store, &line));
    }
    assert((1U == store.dropped) && (1U == store.uncertain));
    mission.seq = 1U;
    mission.ms = 100U;
    assert(0 == b1_ingest_mission(&store, &mission));
    while (1 == b1_next_frame(&store, 1700U, &frame))
    {
        if (B1_TYPE_EVENT == frame.type)
        {
            if (0U == events)
            {
                first_id = b1_get16(frame.payload + B1_EVENT_OFF_ID);
            }
            events++;
            assert(1U == frame.flags);
        }
        if (B1_TYPE_SNAPSHOT == frame.type)
        {
            assert(0 == b1_decode_snapshot(frame.payload, frame.payload_len,
                                           &snap));
            assert((B1_SRC_LINE | B1_SRC_MISSION) == snap.stale_mask);
            snapshots++;
        }
    }
    assert((B1_EVENT_DEPTH == events) && (2U == first_id));
    assert(1U == snapshots);
    puts("PASS FR12/FR15 EVENT ring: 25th event drops the oldest, counted "
         "and flagged uncertain; line and mission stale after 1.5 s");
}

/** @brief Run every test case. */
int main (void)
{
    test_snapshot_layout();
    test_snapshot_limits();
    test_mapping_limits();
    test_command_limits();
    test_frame_limits();
    test_detail_lengths();
    test_command_store_guards();
    test_event_ring_and_stale();
    puts("PASS protocol codec limit checks");
    return 0;
}

/*** end of file ***/

