/** @file test_requirements_trace.c
 * @brief Host checks that close requirement-traceability gaps (FR1, FR2,
 *        FR13, NFR13) on the real buddy1.c / buddy1_protocol.c code.
 *
 * Each block names the requirement it covers. The production sources are
 * linked unchanged; only the lock callbacks and coordinator are stubs.
 * See docs/tribunal2/requirements_traceability.md.
 */
#include <assert.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "buddy1.h"

#define TRACE_SESSION 41u
#define TRACE_LETTER_Z 25u
#define TRACE_CRC8_POLY 0x07u

static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint8_t g_coord_calls;
static uint8_t g_coord_last;

static void lock(void *p_ctx)
{
    assert(0 == pthread_mutex_lock((pthread_mutex_t *)p_ctx));
}

static void unlock(void *p_ctx)
{
    assert(0 == pthread_mutex_unlock((pthread_mutex_t *)p_ctx));
}

static uint8_t coordinator(uint8_t command, void *p_ctx)
{
    (void)p_ctx;
    g_coord_calls++;
    g_coord_last = command;
    return B1_OK;
}

/* Independent CRC-8, polynomial 0x07, initial 0, MSB first (NFR13). */
static uint8_t crc8_reference(uint8_t const *p_bytes, size_t count)
{
    uint8_t crc = 0u;
    for (size_t i = 0u; i < count; i++)
    {
        crc = (uint8_t)(crc ^ p_bytes[i]);
        for (uint32_t bit = 0u; bit < 8u; bit++)
        {
            if (0u != (crc & 0x80u))
            {
                crc = (uint8_t)((uint8_t)(crc << 1u) ^ TRACE_CRC8_POLY);
            }
            else
            {
                crc = (uint8_t)(crc << 1u);
            }
        }
    }
    return crc;
}

static void store_ready(B1_Store *p_store, uint8_t b_ready)
{
    B1_Mission mission = {0};
    assert(0 == b1_store_init(p_store, lock, unlock, &g_mutex,
                              TRACE_SESSION));
    b1_set_coordinator(p_store, coordinator, NULL);
    mission.seq = 1u;
    mission.ready = b_ready;
    mission.remote_hold = 1u;
    assert(0 == b1_ingest_mission(p_store, &mission));
}

static int send(B1_Store *p_store, uint8_t command, uint16_t transaction,
                uint8_t const *p_map, B1_Frame *p_ack)
{
    B1_Command cmd = {0};
    B1_Frame req = {0};
    cmd.command = command;
    cmd.transaction = transaction;
    if (NULL != p_map)
    {
        memcpy(cmd.mapping, p_map, 4u);
    }
    req.type = B1_TYPE_COMMAND;
    req.session = TRACE_SESSION;
    req.payload_len = (uint8_t)b1_encode_command(&cmd, req.payload,
                                                 sizeof(req.payload));
    assert(0u != req.payload_len);
    return b1_handle_command(p_store, &req, p_ack);
}

/* FR2: four distinct letters A..Z; order Straight, Left, Right, U-turn;
 * rejected while running or committed. */
static void fr2_mapping(void)
{
    B1_Store store;
    B1_Frame ack = {0};
    B1_Mission mission = {0};
    uint8_t packed[3] = {0};
    uint8_t const order[4] = {18u, 11u, 17u, 20u}; /* S, L, R, U */
    uint8_t bad[4] = {0u, 1u, 2u, 3u};

    bad[2] = TRACE_LETTER_Z + 1u;
    assert(-1 == b1_pack_mapping(bad, packed));
    bad[2] = 1u;
    assert(-1 == b1_pack_mapping(bad, packed));
    bad[2] = TRACE_LETTER_Z;
    assert(0 == b1_pack_mapping(bad, packed));

    store_ready(&store, 1u);
    assert(B1_OK == send(&store, B1_CMD_MAPPING, 1u, order, &ack));
    assert(1u == store.mapping_valid);
    assert(0 == memcmp(store.mapping, order, 4u));

    mission.seq = 2u;
    mission.running = 1u;
    assert(0 == b1_ingest_mission(&store, &mission));
    assert(B1_BAD_STATE == send(&store, B1_CMD_MAPPING, 2u, order, &ack));
    assert(B1_BAD_STATE == ack.payload[3]);
    puts("PASS FR2 mapping: distinct A-Z letters, S/L/R/U order, "
         "refused while running");
}

/* FR1 (protocol level) and FR13: only the six commands exist; each gets
 * an ACK carrying command, transaction and result; START without a ready
 * controller is NOT_READY; STATUS echoes the last transaction. */
static void fr13_commands(void)
{
    B1_Store store;
    B1_Frame ack = {0};
    B1_Frame status = {0};
    B1_Command decoded = {0};
    uint8_t const map[4] = {0u, 1u, 2u, 3u};
    uint8_t raw[3] = {0u, 9u, 0u};
    uint8_t const accepted[] = {
        B1_CMD_MAPPING, B1_CMD_START, B1_CMD_STOP, B1_CMD_SNAPSHOT,
        B1_CMD_RECAL_IMU, B1_CMD_RECAL_BARCODE
    };

    assert(6u == B1_CMD_LAST);
    raw[0] = 0u;
    assert(0 != b1_decode_command(raw, 3u, &decoded));
    raw[0] = (uint8_t)(B1_CMD_LAST + 1u);
    assert(0 != b1_decode_command(raw, 3u, &decoded));

    store_ready(&store, 0u);
    assert(B1_OK == send(&store, B1_CMD_MAPPING, 10u, map, &ack));
    assert(B1_NOT_READY == send(&store, B1_CMD_START, 11u, NULL, &ack));
    assert(0u == g_coord_calls);

    store_ready(&store, 1u);
    g_coord_calls = 0u;
    for (uint32_t i = 0u; i < sizeof(accepted); i++)
    {
        uint16_t const transaction = (uint16_t)(20u + i);
        int const result = send(&store, accepted[i], transaction,
                                (B1_CMD_MAPPING == accepted[i]) ? map : NULL,
                                &ack);
        assert(result >= 0);
        assert(B1_TYPE_ACK == ack.type);
        assert(4u == ack.payload_len);
        assert(accepted[i] == ack.payload[0]);
        assert(transaction == b1_get16(&ack.payload[1]));
        assert((uint8_t)result == ack.payload[3]);
        if (B1_CMD_STOP == accepted[i])
        {
            assert(B1_CMD_STOP == g_coord_last);
        }
    }
    assert(g_coord_calls >= 1u);
    assert(1 == b1_next_frame(&store, 5000u, &status));
    assert(B1_TYPE_STATUS == status.type);
    assert((uint16_t)(20u + sizeof(accepted) - 1u) ==
           b1_get16(&status.payload[7]));
    puts("PASS FR13 commands: six command IDs only, every one ACKed with "
         "result, START NOT_READY, STATUS echoes last transaction");
}

/* NFR13: 13-byte little-endian header, CRC-8 (0x07), bad CRC or length
 * rejected. */
static void nfr13_frame(void)
{
    B1_Frame frame = {0};
    B1_Frame back = {0};
    uint8_t bytes[B1_MAX_FRAME] = {0};

    assert(13u == B1_HEADER);
    frame.type = B1_TYPE_HEARTBEAT;
    frame.session = 0x1234u;
    frame.seq = 0xA55Au;
    frame.time_ms = 0x01020304u;
    frame.payload_len = 2u;
    frame.payload[0] = 0xC3u;
    frame.payload[1] = 0x3Cu;
    size_t const n = b1_frame_encode(&frame, bytes, sizeof(bytes));
    assert((B1_HEADER + 2u + 1u) == n);
    assert(B1_MAGIC == bytes[0] && B1_VERSION == bytes[1]);
    assert(0x34u == bytes[4] && 0x12u == bytes[5]);
    assert(0x5Au == bytes[6] && 0xA5u == bytes[7]);
    assert(0x04u == bytes[8] && 0x01u == bytes[11]);
    assert(2u == bytes[12]);
    assert(crc8_reference(bytes, n - 1u) == bytes[n - 1u]);
    assert(0 == b1_frame_decode(bytes, n, &back));
    assert(back.seq == frame.seq && back.time_ms == frame.time_ms);

    bytes[12] = 3u;
    assert(0 != b1_frame_decode(bytes, n, &back));
    bytes[12] = 2u;
    bytes[n - 1u] ^= 0x01u;
    assert(0 != b1_frame_decode(bytes, n, &back));
    puts("PASS NFR13 frame: 13-byte LE header, CRC-8 0x07 matches a "
         "reference, bad length/CRC rejected");
}

int main(void)
{
    fr2_mapping();
    fr13_commands();
    nfr13_frame();
    puts("Requirement trace (FR1/FR2/FR13/NFR13) PASS");
    return 0;
}
