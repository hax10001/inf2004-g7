/** @file test_ct_telemetry_poll.c
 * @brief Host test: the real telemetry_peers.c poll paths into the real
 *        buddy1.c store and codec (FR12 telemetry content, NFR13 wire
 *        fields).
 *
 * Sensors are stubs the test sets. Covered here (and not by
 * test_telemetry_map.c): exact ODOM deltas, sub-millimetre carry, speeds,
 * the 500 mm checkpoint boundary with reverse and pivot travel, IR bits,
 * barcode EVENT and ACK rules, terrain and obstacle sentinels (1023),
 * obstacle EVENT kinds, every SENSOR frame field including the lift word,
 * every BARCODE_DIAG field, and the "skip, do not reserve a sequence"
 * rule while MQTT cannot send.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "modules/telemetry/telemetry_peers.h"
#include "modules/telemetry/buddy1_mqtt.h"
#include "modules/motion/motion.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/barcode_scanning/bc_cal.h"
#include "modules/barcode_scanning/bc_port.h"
#include "modules/barcode_scanning/bc_wave.h"
#include "drivers/encoder.h"
#include "modules/terrain/terrain.h"
#include "modules/obstacle/obs_state.h"

#define CT_SESSION 9U
#define CT_FRAMES 256U
#define CT_SENT 16U
#define CT_PERIOD_US 50000U
#define CT_NONE10 1023U
#define CT_NONE16 65535U
/* SENSOR valid mask with IR, obstacle and motion but no IMU. */
#define CT_VALID_NO_IMU 13U
/* Test-only packing of the obstacle status word. */
#define CT_OBS_ROUTE_SHIFT 3U
#define CT_OBS_FAULT_SHIFT 5U
#define CT_OBS_COUNT_SHIFT 9U
#define CT_OBS_DIST_SHIFT 17U
#define CT_OBS_PHASE_MASK 7U
#define CT_OBS_ROUTE_MASK 3U
#define CT_OBS_FAULT_MASK 15U
#define CT_OBS_COUNT_MASK 255U
#define CT_OBS_DIST_MASK 511U

/* Stub sensor state. */
static int32_t g_left;
static int32_t g_right;
static uint32_t g_usec = 1000000U;
static uint8_t g_motion_valid = 1U;
static ir_shared_snapshot_t g_ir;
static ER g_ir_result = E_OK;
static bc_port_result_t g_bc;
static ER g_bc_result = -1;
static uint32_t g_bc_acks;
static uint16_t g_bc_ack_seq;
static bc_port_stats_t g_stats;
static ER g_stats_result = -1;
static bc_wave_status_t g_wave;
static terrain_snapshot_t g_imu;
static bool g_calibrated = true;
static uint32_t g_obs_word;
static uint8_t g_can_send;
static b1_frame_t g_sent[CT_SENT];
static uint32_t g_sent_count;
/* Store under test and drained frames. */
static b1_store_t g_store;
static g7_telemetry_peers_t g_peers;
static b1_frame_t g_frames[CT_FRAMES];
static uint32_t g_frame_count;

/** @brief Stub: motion status; the poll reads encoder counts itself. */
ER motion_get_status (motion_status_t * p_status)
{
    memset(p_status, 0, sizeof(*p_status));
    p_status->b_valid = g_motion_valid;
    return E_OK;
}

/** @brief Stub: wheel counts set by the test. */
void encoder_read (encoder_wheel_t wheel, encoder_sample_t * p_sample)
{
    p_sample->count = (ENCODER_LEFT == wheel) ? g_left : g_right;
    p_sample->edge_usec = g_usec;
}

/** @brief Stub: encoder timer. */
uint32_t encoder_now_usec (void)
{
    return g_usec;
}

/** @brief Stub: IR snapshot set by the test. */
ER ir_shared_get_latest (ir_shared_snapshot_t * p_out)
{
    *p_out = g_ir;
    return g_ir_result;
}

/** @brief Stub: decoded barcode set by the test. */
ER bc_port_peek (bc_port_result_t * p_out)
{
    *p_out = g_bc;
    return g_bc_result;
}

/** @brief Stub: count scanner ACKs. */
ER bc_port_ack (uint16_t sequence)
{
    g_bc_acks++;
    g_bc_ack_seq = sequence;
    return E_OK;
}

/** @brief Stub: scanner statistics set by the test. */
ER bc_port_stats (bc_port_stats_t * p_out)
{
    *p_out = g_stats;
    return g_stats_result;
}

/** @brief Stub: scanner state set by the test. */
void bc_wave_status (bc_wave_status_t * p_out)
{
    *p_out = g_wave;
}

/** @brief Stub: IMU snapshot set by the test. */
ER terrain_get_snapshot (terrain_snapshot_t * p_out)
{
    *p_out = g_imu;
    return E_OK;
}

/** @brief Stub: IMU calibration flag. */
bool terrain_is_calibrated (void)
{
    return g_calibrated;
}

/** @brief Stub: packed obstacle status word set by the test. */
uint32_t obs_state_status_get (void)
{
    return g_obs_word;
}

/** @brief Stub decoder of the test packing: phase. */
obs_state_phase_t obs_state_status_phase (uint32_t word)
{
    return (obs_state_phase_t)(word & CT_OBS_PHASE_MASK);
}

/** @brief Stub decoder of the test packing: route. */
obs_state_route_t obs_state_status_route (uint32_t word)
{
    return (obs_state_route_t)((word >> CT_OBS_ROUTE_SHIFT) &
                               CT_OBS_ROUTE_MASK);
}

/** @brief Stub decoder of the test packing: fault bits. */
uint32_t obs_state_status_faults (uint32_t word)
{
    return (word >> CT_OBS_FAULT_SHIFT) & CT_OBS_FAULT_MASK;
}

/** @brief Stub decoder of the test packing: bypass count. */
uint32_t obs_state_status_count (uint32_t word)
{
    return (word >> CT_OBS_COUNT_SHIFT) & CT_OBS_COUNT_MASK;
}

/** @brief Stub decoder of the test packing: distance, cm. */
uint32_t obs_state_status_distance_cm (uint32_t word)
{
    return (word >> CT_OBS_DIST_SHIFT) & CT_OBS_DIST_MASK;
}

/** @brief Stub: MQTT send room chosen by the test. */
uint8_t b1_mqtt_can_send (void)
{
    return g_can_send;
}

/** @brief Stub: record submitted detail frames. */
int32_t b1_mqtt_submit (b1_frame_t const * p_frame)
{
    assert(g_sent_count < CT_SENT);
    g_sent[g_sent_count] = *p_frame;
    g_sent_count++;
    return 0;
}

/** @brief Stub: console output discarded. */
INT tm_printf (UB const * p_format, ...)
{
    (void)p_format;
    return 0;
}

/** @brief Store lock (single thread). */
static void lock (void * p_ctx)
{
    (void)p_ctx;
}

/** @brief Pack an obstacle status word in the test layout. */
static uint32_t obs_word (uint32_t phase, uint32_t route, uint32_t faults,
                          uint32_t count, uint32_t distance_cm)
{
    return phase | (route << CT_OBS_ROUTE_SHIFT) |
           (faults << CT_OBS_FAULT_SHIFT) | (count << CT_OBS_COUNT_SHIFT) |
           (distance_cm << CT_OBS_DIST_SHIFT);
}

/** @brief Fresh store and peers; every source off; 1 mm per count. */
static void reset_world (void)
{
    assert(0 == b1_store_init(&g_store, lock, lock, NULL, CT_SESSION));
    memset(&g_peers, 0, sizeof g_peers);
    g_peers.left_mm_per_count = 1.0f;
    g_peers.right_mm_per_count = 1.0f;
    g_left = 0;
    g_right = 0;
    g_usec += 10U * CT_PERIOD_US;
    g_motion_valid = 1U;
    memset(&g_ir, 0, sizeof g_ir);
    g_ir_result = -1;
    memset(&g_bc, 0, sizeof g_bc);
    g_bc_result = -1;
    g_bc_acks = 0U;
    memset(&g_stats, 0, sizeof g_stats);
    g_stats_result = -1;
    memset(&g_wave, 0, sizeof g_wave);
    memset(&g_imu, 0, sizeof g_imu);
    g_calibrated = true;
    g_obs_word = obs_word(0U, 0U, 0U, 0U, 100U);
    g_can_send = 0U;
    g_sent_count = 0U;
}

/** @brief Poll once. */
static uint8_t poll (void)
{
    return g7_telemetry_poll(&g_peers, &g_store);
}

/** @brief Move forward-positive wheel distances (counts) over 50 ms. */
static uint8_t drive (int32_t left, int32_t right)
{
    g_left += left * BC_CAL_LEFT_FORWARD_SIGN;
    g_right += right * BC_CAL_RIGHT_FORWARD_SIGN;
    g_usec += CT_PERIOD_US;
    return poll();
}

/** @brief Take every queued frame at time now_ms into g_frames. */
static void drain (uint32_t now_ms)
{
    b1_frame_t frame;

    g_frame_count = 0U;
    while (1 == b1_next_frame(&g_store, now_ms, &frame))
    {
        assert(g_frame_count < CT_FRAMES);
        g_frames[g_frame_count] = frame;
        g_frame_count++;
    }
}

/** @brief Number of drained frames of a type. */
static uint32_t count_type (uint8_t type)
{
    uint32_t n = 0U;

    for (uint32_t i = 0U; i < g_frame_count; i++)
    {
        if (type == g_frames[i].type)
        {
            n++;
        }
    }
    return n;
}

/** @brief The index-th drained frame of a type (asserts it exists). */
static b1_frame_t const * frame_of (uint8_t type, uint32_t index)
{
    uint32_t seen = 0U;

    for (uint32_t i = 0U; i < g_frame_count; i++)
    {
        if (type == g_frames[i].type)
        {
            if (seen == index)
            {
                return &g_frames[i];
            }
            seen++;
        }
    }
    assert(0);
    return NULL;
}

/** @brief Decode the drained SNAPSHOT. */
static b1_snapshot_payload_t snapshot (void)
{
    b1_snapshot_payload_t snap;
    b1_frame_t const * p_frame = frame_of(B1_TYPE_SNAPSHOT, 0U);

    assert(0 == b1_decode_snapshot(p_frame->payload, p_frame->payload_len,
                                   &snap));
    return snap;
}

/** @brief Signed 16-bit field of a payload. */
static int16_t get_s16 (uint8_t const * p_bytes)
{
    return (int16_t)b1_get16(p_bytes);
}

/** @brief Test: ODOM carries exact signed deltas and speeds, keeps the
 *  sub-millimetre remainder, and counts a pivot as no travel (FR12). */
static void test_odom_deltas (void)
{
    int32_t sum_left = 0;
    b1_frame_t const * p_odom;
    b1_snapshot_payload_t snap;

    reset_world();
    g_imu.b_valid = 1U;
    g_imu.timestamp_msec = 1U;
    assert(0U == (drive(0, 0) & B1_SRC_MOTION));
    assert(B1_SRC_MOTION == (drive(10, 12) & B1_SRC_MOTION));
    drain(g_usec / 1000U);
    p_odom = frame_of(B1_TYPE_ODOM, 0U);
    assert(B1_ODOM_LEN == p_odom->payload_len);
    assert(10 == get_s16(p_odom->payload));
    assert(12 == get_s16(p_odom->payload + 2));
    assert(1U == b1_get16(p_odom->payload + 4));
    assert((g_usec / 1000U) == p_odom->time_ms);
    snap = snapshot();
    assert((200 == snap.speed_left_mm_s) && (240 == snap.speed_right_mm_s));

    /* Backwards: negative deltas and speeds. */
    (void)drive(-5, -5);
    drain(g_usec / 1000U + 300U);
    p_odom = frame_of(B1_TYPE_ODOM, 0U);
    assert((-5 == get_s16(p_odom->payload)) &&
           (-5 == get_s16(p_odom->payload + 2)));
    snap = snapshot();
    assert(-100 == snap.speed_left_mm_s);

    /* 0.25 mm per count: 1 count per poll gives 1 mm every 4th poll. */
    g_peers.left_mm_per_count = 0.25f;
    g_peers.right_mm_per_count = 0.25f;
    for (uint32_t i = 0U; i < 40U; i++)
    {
        (void)drive(1, 1);
    }
    drain(g_usec / 1000U + 600U);
    for (uint32_t i = 0U; i < count_type(B1_TYPE_ODOM); i++)
    {
        sum_left += get_s16(frame_of(B1_TYPE_ODOM, i)->payload);
    }
    assert(10 == sum_left);
    assert(10U == count_type(B1_TYPE_ODOM));
    puts("PASS FR12 ODOM: signed L/R deltas, 200/240 mm/s speeds, reverse "
         "negative, 0.25 mm counts carried (40 counts -> 10 mm)");
}

/** @brief Test: a checkpoint exactly when |L+R|/2 reaches each 500 mm;
 *  reverse travel counts, a pivot on the spot does not (FR12). */
static void test_checkpoint_boundary (void)
{
    b1_frame_t const * p_cp;

    reset_world();
    g_imu.b_valid = 1U;
    g_imu.timestamp_msec = 1U;
    (void)drive(0, 0);
    for (uint32_t i = 0U; i < 49U; i++)
    {
        (void)drive(10, 10);
    }
    drain(g_usec / 1000U);
    assert(0U == count_type(B1_TYPE_CHECKPOINT));
    assert(490U == (g_peers.odom_travel_x2 / 2U));
    (void)drive(10, 10);
    drain(g_usec / 1000U);
    p_cp = frame_of(B1_TYPE_CHECKPOINT, 0U);
    assert(B1_CHECKPOINT_LEN == p_cp->payload_len);
    assert(500U == b1_get32(p_cp->payload));
    assert((g_usec / 1000U) == p_cp->time_ms);

    for (uint32_t i = 0U; i < 100U; i++)
    {
        (void)drive(10, -10);
    }
    drain(g_usec / 1000U);
    assert(B1_MOTION_DEPTH == count_type(B1_TYPE_ODOM));
    assert(0U == count_type(B1_TYPE_CHECKPOINT));
    assert(1000U == g_peers.odom_travel_x2);

    for (uint32_t i = 0U; i < 50U; i++)
    {
        (void)drive(-10, -10);
    }
    drain(g_usec / 1000U);
    assert(1U == count_type(B1_TYPE_CHECKPOINT));
    assert(1000U == b1_get32(frame_of(B1_TYPE_CHECKPOINT, 0U)->payload));
    puts("PASS FR12 checkpoint at exactly 500 mm, again at 1000 mm driving "
         "backwards; 100 pivot periods add no distance");
}

/** @brief Test: IR bits, no-barcode letter 31, one barcode EVENT per scan
 *  sequence with ACK after storing, invalid scans ignored (FR12). */
static void test_ir_and_barcode (void)
{
    b1_frame_t const * p_event;
    b1_snapshot_payload_t snap;
    uint16_t line_seq;

    reset_world();
    g_ir_result = E_OK;
    g_ir.b_valid = 1U;
    g_ir.left = 1U;
    g_ir.barcode = 1U;
    g_ir.timestamp_msec = 5000U;
    assert(B1_SRC_LINE == (poll() & B1_SRC_LINE));
    line_seq = g_store.line.seq;
    assert(0U == (poll() & B1_SRC_LINE));
    assert(line_seq == g_store.line.seq);
    drain(5000U);
    snap = snapshot();
    assert(5U == snap.ir);
    assert(B1_LETTER_NONE == snap.barcode);

    /* Not complete, not valid, letter 26: no EVENT, no ACK. */
    g_bc_result = E_OK;
    g_bc.sequence = 3U;
    g_bc.timestamp_msec = 5010U;
    g_bc.info = (uint16_t)(BC_PORT_VALID_FLAG | 6U);
    (void)poll();
    g_bc.info = (uint16_t)(BC_PORT_COMPLETE_FLAG | 6U);
    (void)poll();
    g_bc.info = (uint16_t)(BC_PORT_VALID_FLAG | BC_PORT_COMPLETE_FLAG |
                           B1_LETTER_COUNT);
    (void)poll();
    drain(5100U);
    assert(0U == count_type(B1_TYPE_EVENT));
    assert(0U == g_bc_acks);

    g_bc.info = (uint16_t)(BC_PORT_VALID_FLAG | BC_PORT_COMPLETE_FLAG | 6U);
    assert(B1_SRC_LINE == (poll() & B1_SRC_LINE));
    assert((1U == g_bc_acks) && (3U == g_bc_ack_seq));
    (void)poll();
    assert(1U == g_bc_acks);
    g_ir.timestamp_msec = 5400U;
    (void)poll();
    drain(5400U);
    assert(1U == count_type(B1_TYPE_EVENT));
    p_event = frame_of(B1_TYPE_EVENT, 0U);
    assert(B1_EVSRC_LINE == p_event->payload[0]);
    assert(B1_EVT_BARCODE == p_event->payload[1]);
    assert(3U == b1_get16(p_event->payload + B1_EVENT_OFF_ID));
    assert(6U == b1_get16(p_event->payload + B1_EVENT_OFF_VALUE));
    assert(5010U == p_event->time_ms);
    snap = snapshot();
    assert(6U == snap.barcode);
    puts("PASS FR12 IR bits L|B=5, letter 31 before any barcode, one G "
         "EVENT per scan sequence with one ACK, invalid scans ignored");
}

/** @brief Test: terrain and obstacle sentinels reach the SNAPSHOT as
 *  1023 and the hump height when valid (FR12, NFR13). */
static void test_sentinels (void)
{
    b1_snapshot_payload_t snap;

    reset_world();
    g_imu.b_valid = 1U;
    g_imu.height_valid = 0U;
    g_imu.hump_height_mm = 40U;
    g_imu.max_hump_height_mm = 55U;
    g_imu.timestamp_msec = 7000U;
    g_obs_word = obs_word(0U, 0U, 0U, 0U, OBS_DIST_INVALID_CM);
    (void)poll();
    drain(7000U);
    snap = snapshot();
    assert(CT_NONE10 == snap.hump_mm);
    assert(55U == snap.max_hump_mm);
    assert(CT_NONE10 == snap.obstacle_cm);

    g_imu.height_valid = 1U;
    g_imu.timestamp_msec = 7300U;
    g_obs_word = obs_word(0U, 0U, OBS_FAULT_NO_ECHO, 0U, 80U);
    (void)poll();
    drain(7300U);
    snap = snapshot();
    assert(40U == snap.hump_mm);
    assert(CT_NONE10 == snap.obstacle_cm);
    assert(OBS_FAULT_NO_ECHO == (snap.faults & OBS_FAULT_NO_ECHO));

    g_imu.timestamp_msec = 7600U;
    g_obs_word = obs_word(0U, 0U, 0U, 0U, 80U);
    (void)poll();
    drain(7600U);
    snap = snapshot();
    assert(80U == snap.obstacle_cm);
    puts("PASS FR12 sentinels: hump 1023 when height invalid, obstacle "
         "1023 for 511 cm and for no-echo, real values otherwise");
}

/** @brief Test: obstacle EVENT kinds fire once per change: fault, confirm,
 *  bypass side, bypass count (FR12). */
static void test_obstacle_events (void)
{
    uint32_t const confirm = (uint32_t)OBS_PHASE_CONFIRM;
    b1_frame_t const * p_event;

    reset_world();
    g_imu.b_valid = 1U;
    g_imu.timestamp_msec = 9000U;
    g_obs_word = obs_word(confirm, 0U, 0U, 0U, 25U);
    (void)poll();
    g_imu.timestamp_msec = 9050U;
    (void)poll();
    g_imu.timestamp_msec = 9100U;
    g_obs_word = obs_word(confirm, OBS_ROUTE_LEFT, 0U, 0U, 25U);
    (void)poll();
    g_imu.timestamp_msec = 9150U;
    g_obs_word = obs_word(0U, OBS_ROUTE_LEFT, 0U, 1U, 25U);
    (void)poll();
    g_imu.timestamp_msec = 9200U;
    g_obs_word = obs_word(0U, OBS_ROUTE_LEFT, 2U, 1U, 25U);
    (void)poll();
    g_imu.timestamp_msec = 9250U;
    (void)poll();
    drain(9250U);
    assert(4U == count_type(B1_TYPE_EVENT));
    p_event = frame_of(B1_TYPE_EVENT, 0U);
    assert((B1_EVSRC_OBSTACLE == p_event->payload[0]) &&
           (B1_EVT_OBSTACLE == p_event->payload[1]) &&
           (25U == b1_get16(p_event->payload + B1_EVENT_OFF_VALUE)));
    p_event = frame_of(B1_TYPE_EVENT, 1U);
    assert((B1_EVT_BYPASS_CHOSEN == p_event->payload[1]) &&
           (OBS_ROUTE_LEFT == b1_get16(p_event->payload +
                                       B1_EVENT_OFF_VALUE)));
    p_event = frame_of(B1_TYPE_EVENT, 2U);
    assert(B1_EVT_BYPASSED == p_event->payload[1]);
    p_event = frame_of(B1_TYPE_EVENT, 3U);
    assert((B1_EVT_SAFETY_FAULT == p_event->payload[1]) &&
           (2U == b1_get16(p_event->payload + B1_EVENT_OFF_VALUE)) &&
           (4U == b1_get16(p_event->payload + B1_EVENT_OFF_ID)));
    puts("PASS FR12 obstacle EVENTs: confirm (25 cm), LEFT chosen, "
         "bypassed, fault 2 - each once, IDs 1..4");
}

/** @brief Test: every SENSOR field incl. the lift word, the 26-byte
 *  length, and clamped tilt (FR12, NFR13). */
static void test_sensor_frame (void)
{
    b1_frame_t const * p_sensor;
    uint8_t bytes[B1_MAX_FRAME];
    b1_frame_t back;
    size_t n;

    reset_world();
    g_can_send = 1U;
    g_left = -123456;
    g_right = 654321;
    g_ir_result = E_OK;
    g_ir.b_valid = 1U;
    g_ir.right = 1U;
    g_ir.timestamp_msec = 12000U;
    g_imu.b_valid = 1U;
    g_imu.heading_valid = 1U;
    g_imu.heading_centideg = 27000U;
    g_imu.roll_deg = -5.5f;
    g_imu.pitch_deg = 400.0f;
    g_imu.hump_phase = (uint8_t)TERRAIN_HUMP_CLIMBING;
    g_imu.onHump = 1U;
    g_imu.height_valid = 1U;
    g_imu.lift_height_mm = 300U;
    g_imu.b_lifted = 1U;
    g_imu.timestamp_msec = 11990U;
    g_peers.odom_confidence = 2U;
    g_obs_word = obs_word(2U, 0U, 0U, 0U, 37U);
    (void)poll();
    assert(1U == g_sent_count);
    p_sensor = &g_sent[0];
    assert(B1_TYPE_SENSOR == p_sensor->type);
    assert(B1_SENSOR_LIFT == p_sensor->payload_len);
    assert(CT_SESSION == p_sensor->session);
    assert(12000U == p_sensor->time_ms);
    assert(31U == p_sensor->payload[0]);
    assert(2U == p_sensor->payload[1]);
    assert(-550 == get_s16(p_sensor->payload + B1_SENSOR_OFF_ROLL));
    assert(INT16_MAX == get_s16(p_sensor->payload + B1_SENSOR_OFF_PITCH));
    assert(27000U == b1_get16(p_sensor->payload + B1_SENSOR_OFF_HEADING));
    assert(37U == b1_get16(p_sensor->payload + B1_SENSOR_OFF_DISTANCE));
    assert(1U == p_sensor->payload[B1_SENSOR_OFF_HUMP_PHASE]);
    assert(1U == p_sensor->payload[B1_SENSOR_OFF_CALIBRATED]);
    assert(-123456 ==
           (int32_t)b1_get32(p_sensor->payload + B1_SENSOR_OFF_LEFT_COUNT));
    assert(654321U ==
           b1_get32(p_sensor->payload + B1_SENSOR_OFF_RIGHT_COUNT));
    assert(2U == p_sensor->payload[B1_SENSOR_OFF_OBS_PHASE]);
    assert(1U == p_sensor->payload[B1_SENSOR_OFF_ON_HUMP]);
    assert(1U == p_sensor->payload[B1_SENSOR_OFF_HEIGHT_OK]);
    assert(2U == p_sensor->payload[B1_SENSOR_OFF_ODOM_CONF]);
    assert((300U | B1_LIFT_FLAG) ==
           b1_get16(p_sensor->payload + B1_SENSOR_OFF_LIFT));
    n = b1_frame_encode(p_sensor, bytes, sizeof bytes);
    assert((B1_HEADER + B1_SENSOR_LIFT + 1U) == n);
    assert(0 == b1_frame_decode(bytes, n, &back));
    assert(0 == memcmp(back.payload, p_sensor->payload, B1_SENSOR_LIFT));

    /* Within 1 s: no second frame. Then IMU invalid, no heading, no
     * echo: lift 1023 without the flag, heading and distance 65535. */
    g_ir.timestamp_msec = 12999U;
    (void)poll();
    assert(1U == g_sent_count);
    g_imu.b_valid = 0U;
    g_imu.heading_valid = 0U;
    g_obs_word = obs_word(2U, 0U, OBS_FAULT_NO_ECHO, 0U, 37U);
    g_ir.timestamp_msec = 13000U;
    (void)poll();
    assert(2U == g_sent_count);
    p_sensor = &g_sent[1];
    assert(CT_VALID_NO_IMU == p_sensor->payload[0]);
    assert(CT_NONE16 == b1_get16(p_sensor->payload + B1_SENSOR_OFF_HEADING));
    assert(CT_NONE16 ==
           b1_get16(p_sensor->payload + B1_SENSOR_OFF_DISTANCE));
    assert(CT_NONE10 == b1_get16(p_sensor->payload + B1_SENSOR_OFF_LIFT));
    assert(p_sensor->seq == (uint16_t)(g_sent[0].seq + 1U));
    puts("PASS FR12/NFR13 SENSOR: 26 bytes, valid mask, IR, tilt clamp, "
         "heading, distance, counts, phases, lift 300|lifted; 65535/1023 "
         "when unavailable; one per second");
}

/** @brief Test: BARCODE_DIAG fields and saturation; while MQTT cannot
 *  send, detail frames are skipped without using a wire sequence. */
static void test_barcode_diag (void)
{
    b1_frame_t const * p_diag = NULL;
    uint16_t seq_before;

    reset_world();
    g_left = -7;
    g_right = 9;
    g_imu.b_valid = 1U;
    g_imu.timestamp_msec = 20000U;
    g_stats_result = E_OK;
    g_stats.state = 5U;
    g_stats.last_error = 8U;
    g_stats.raw = 1U;
    g_stats.rejected = 0xDEADBEEFU;
    g_stats.completed = 77U;
    g_stats.last_scan_units = 70000U;
    g_stats.runs = 29U;
    g_stats.edges = 0x01020304U;
    g_wave.state = 6U;
    g_wave.white_rejects = 3U;
    g_wave.white_mean = 4095U;
    g_wave.white_sigma = 12U;
    g_wave.sequence = 0xBEEFU;
    g_wave.last_valid = 1U;
    g_wave.last_letter = B1_LETTER_NONE;
    g_wave.last_reason = 7U;
    g_wave.last_process_ms = 1234U;
    seq_before = g_store.wire_seq;
    (void)poll();
    assert(0U == g_sent_count);
    assert(seq_before == g_store.wire_seq);

    g_can_send = 1U;
    g_imu.timestamp_msec = 21000U;
    (void)poll();
    for (uint32_t i = 0U; i < g_sent_count; i++)
    {
        if (B1_TYPE_BARCODE_DIAG == g_sent[i].type)
        {
            p_diag = &g_sent[i];
        }
    }
    assert(NULL != p_diag);
    assert(B1_BARCODE_DIAG_V2 == p_diag->payload_len);
    assert(2U == p_diag->payload[0]);
    assert(5U == p_diag->payload[1]);
    assert(8U == p_diag->payload[B1_BCD_OFF_ERROR]);
    assert(1U == p_diag->payload[B1_BCD_OFF_RAW]);
    assert(0xDEADBEEFU == b1_get32(p_diag->payload + B1_BCD_OFF_REJECTED));
    assert(77U == b1_get32(p_diag->payload + B1_BCD_OFF_COMPLETED));
    assert(CT_NONE16 == b1_get16(p_diag->payload + B1_BCD_OFF_SCAN_UNITS));
    assert(29U == p_diag->payload[B1_BCD_OFF_RUNS]);
    assert(8U == p_diag->payload[B1_BCD_OFF_ERROR_COPY]);
    assert(-7 == (int32_t)b1_get32(p_diag->payload + B1_BCD_OFF_LEFT));
    assert(9U == b1_get32(p_diag->payload + B1_BCD_OFF_RIGHT));
    assert(0x01020304U == b1_get32(p_diag->payload + B1_BCD_OFF_EDGES));
    assert(6U == p_diag->payload[B1_BCD_OFF_WAVE_STATE]);
    assert(3U == p_diag->payload[B1_BCD_OFF_WHITE_REJ]);
    assert(4095U == b1_get16(p_diag->payload + B1_BCD_OFF_WHITE_MEAN));
    assert(12U == b1_get16(p_diag->payload + B1_BCD_OFF_WHITE_SIGMA));
    assert(0xBEEFU == b1_get16(p_diag->payload + B1_BCD_OFF_WAVE_SEQ));
    assert(1U == p_diag->payload[B1_BCD_OFF_LAST_VALID]);
    assert(B1_LETTER_NONE == p_diag->payload[B1_BCD_OFF_LAST_LETTER]);
    assert(7U == p_diag->payload[B1_BCD_OFF_LAST_REASON]);
    assert(0U == p_diag->payload[B1_BCD_OFF_RESERVED]);
    assert(1234U == b1_get16(p_diag->payload + B1_BCD_OFF_PROCESS_MS));

    /* Scanner statistics unavailable: no frame, retried a second later. */
    g_sent_count = 0U;
    g_stats_result = -1;
    g_imu.timestamp_msec = 22000U;
    (void)poll();
    for (uint32_t i = 0U; i < g_sent_count; i++)
    {
        assert(B1_TYPE_BARCODE_DIAG != g_sent[i].type);
    }
    puts("PASS FR12 BARCODE_DIAG: 42 bytes, every field, scan units "
         "saturate at 65535, skipped without a sequence while offline");
}

/** @brief Test: no motion source means no ODOM and no MOTION bit; a
 *  missing wheel scale is "odometry unavailable" (FR12). */
static void test_motion_unavailable (void)
{
    reset_world();
    g_motion_valid = 0U;
    g_imu.b_valid = 1U;
    g_imu.timestamp_msec = 30000U;
    g_can_send = 1U;
    assert(0U == (drive(0, 0) & B1_SRC_MOTION));
    assert(0U == (drive(100, 100) & B1_SRC_MOTION));
    assert(0U == (g_sent[0].payload[0] & 8U));
    g_motion_valid = 1U;
    g_peers.left_mm_per_count = 0.0f;
    (void)drive(0, 0);
    assert(0U == (drive(100, 100) & B1_SRC_MOTION));
    drain(g_usec / 1000U);
    assert(0U == count_type(B1_TYPE_ODOM));
    puts("PASS FR12 motion invalid or wheel scale 0: no ODOM, no MOTION "
         "valid bit");
}

/** @brief Run every test case. */
int main (void)
{
    test_odom_deltas();
    test_checkpoint_boundary();
    test_ir_and_barcode();
    test_sentinels();
    test_obstacle_events();
    test_sensor_frame();
    test_barcode_diag();
    test_motion_unavailable();
    puts("PASS telemetry poll path checks");
    return 0;
}

/*** end of file ***/

