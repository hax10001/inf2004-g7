/** @file test_telemetry_map.c
 * @brief Host test: telemetry for the dashboard route map (FR14/FR15). Runs
 *        the real telemetry_peers.c and buddy1.c against stubbed sensors:
 *        ODOM sequence stays contiguous, distance checkpoints every 500 mm
 *        carry the same distance the dashboard integrates, a hump the robot
 *        drove over becomes one terrain EVENT with its peak height, a tilt
 *        without travel or a lift does not, mission EVENT IDs no longer flag
 *        the store uncertain, and the rings keep offline telemetry bounded
 *        and counted.
 */
#include <assert.h>
#include <stdarg.h>
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

static int32_t g_left;
static int32_t g_right;
static uint32_t g_usec;
static terrain_snapshot_t g_imu;

/** @brief Host stub of motion_get_status(). */
ER motion_get_status(motion_status_t *p)
{ memset(p, 0, sizeof(*p)); p->b_valid = 1; return E_OK; }
/** @brief Host stub of encoder_read(). */
void encoder_read(encoder_wheel_t w, encoder_sample_t *p)
{ p->count = (w == ENCODER_LEFT) ? g_left : g_right; p->edge_usec = g_usec; }
/** @brief Host stub of encoder_now_usec(). */
uint32_t encoder_now_usec(void) { return g_usec; }
/** @brief Host stub of ir_shared_get_latest(). */
ER ir_shared_get_latest(ir_shared_snapshot_t *p) { (void)p; return -1; }
/** @brief Host stub of bc_port_peek(). */
ER bc_port_peek(bc_port_result_t *p) { (void)p; return -1; }
/** @brief Host stub of bc_port_ack(). */
ER bc_port_ack(uint16_t s) { (void)s; return E_OK; }
/** @brief Host stub of bc_port_stats(). */
ER bc_port_stats(bc_port_stats_t *p) { memset(p, 0, sizeof(*p)); return -1; }
/** @brief Host stub of bc_wave_status(). */
void bc_wave_status(bc_wave_status_t *p) { memset(p, 0, sizeof(*p)); }
/** @brief Host stub of terrain_get_snapshot(). */
ER terrain_get_snapshot(terrain_snapshot_t *p) { *p = g_imu; return E_OK; }
/** @brief Host stub of terrain_is_calibrated(). */
bool terrain_is_calibrated(void) { return true; }
/** @brief Host stub of obs_state_status_get(). */
uint32_t obs_state_status_get(void) { return 0u; }
/** @brief Host stub of obs_state_status_phase(). */
obs_state_phase_t obs_state_status_phase(uint32_t w)
{ (void)w; return (obs_state_phase_t)0; }
/** @brief Host stub of obs_state_status_route(). */
obs_state_route_t obs_state_status_route(uint32_t w)
{ (void)w; return OBS_ROUTE_NONE; }
/** @brief Host stub of obs_state_status_faults(). */
uint32_t obs_state_status_faults(uint32_t w) { (void)w; return 0u; }
/** @brief Host stub of obs_state_status_count(). */
uint32_t obs_state_status_count(uint32_t w) { (void)w; return 0u; }
/** @brief Host stub of obs_state_status_distance_cm(). */
uint32_t obs_state_status_distance_cm(uint32_t w) { (void)w; return 0u; }
/** @brief Host stub of b1_mqtt_can_send(). */
uint8_t b1_mqtt_can_send(void) { return 0u; }
/** @brief Host stub of b1_mqtt_submit(). */
int32_t b1_mqtt_submit(const b1_frame_t *f) { (void)f; return 0; }
/** @brief Host stub of tm_printf(). */
INT tm_printf(const UB *format, ...) { (void)format; return 0; }

/** @brief Test helper: lock. */
static void lock(void *c) { (void)c; }
static b1_store_t store;
static g7_telemetry_peers_t peers;

/* Drive both wheels forward by counts and advance 50 ms. */
static void drive(int32_t counts)
{
    g_left += counts * BC_CAL_LEFT_FORWARD_SIGN;
    g_right += counts * BC_CAL_RIGHT_FORWARD_SIGN;
    g_usec += 50000u;
    g_imu.timestamp_msec = g_usec / 1000u;
    (void)g7_telemetry_poll(&peers, &store);
}

typedef struct { int odom, checkpoints, humps,
    last_hump_mm; uint32_t cp; } seen_t;

/** @brief Test helper: drain. */
static seen_t drain(void)
{
    seen_t s = {0};
    b1_frame_t f;
    int last_seq = -1;
    while (b1_next_frame(&store, g_usec / 1000u + 2000u * 1000u, &f) > 0) {
        if (f.type == B1_TYPE_ODOM) {
            int seq = f.payload[4] | (f.payload[5] << 8);
            if (last_seq >= 0)
            {
                assert(((seq - last_seq) & 0xffff) == 1);
            }
            last_seq = seq; s.odom++;
        } else if (f.type == B1_TYPE_CHECKPOINT) {
            s.checkpoints++;
            s.cp = (uint32_t)f.payload[0] | ((uint32_t)f.payload[1] << 8) |
                   ((uint32_t)f.payload[2] << 16) |
                   ((uint32_t)f.payload[3] << 24);
        } else if (f.type == B1_TYPE_EVENT && f.payload[0] == 3u) {
            assert(f.payload[1] == B1_EVT_HUMP_CROSSED);
            s.humps++; s.last_hump_mm = f.payload[4];
        }
        if (f.type == B1_TYPE_HEARTBEAT || f.type == B1_TYPE_SNAPSHOT)
        {
            break;
        }
    }
    return s;
}

/** @brief Run every test case; any failed assert aborts. */
int main(void)
{
    int i;
    seen_t s;
    uint32_t driven_x2 = 0u;
    assert(b1_store_init(&store, lock, lock, NULL, 7) == 0);
    memset(&peers, 0, sizeof(peers));
    peers.left_mm_per_count = (3.14159265f * 70.0f) / 1056.0f;
    peers.right_mm_per_count = peers.left_mm_per_count;
    g_imu.b_valid = 1; g_imu.height_valid = 1;
    drive(0); /* first sample only latches the counts */

    /* 1. 600 mm straight: 24 mm per 50 ms. One checkpoint at 500 mm. */
    for (i = 0; i < 25; i++) {
        drive(115); /* 115 counts = 23.96 mm */
        s = drain();
        if (s.checkpoints) {
            assert(s.cp >= 500u && s.cp < 525u);
            driven_x2 = s.cp;
        }
    }
    assert(driven_x2 != 0u);
    assert(peers.odom_travel_x2 / 2u >= 590u);
    assert(store.uncertain == 0u);
    puts("PASS ODOM contiguous, checkpoint at each 500 mm of travel");

    /* 2. A hump the terrain task recorded (HUMP DONE flips the toggle)
     * gives exactly one EVENT with its peak height. */
    g_imu.onHump = 1;
    for (i = 0; i < 4; i++) { drive(115); (void)drain(); }
    g_imu.onHump = 0; g_imu.last_hump_height_mm = 14;
    g_imu.hump_done_toggle ^= 1u;
    drive(115); s = drain();
    assert(s.humps == 1 && s.last_hump_mm == 14);
    drive(115); s = drain();
    assert(s.humps == 0);
    puts("PASS recorded hump -> one terrain EVENT with its peak height");

    /* 3. Discarded humps (no travel, lifted) leave the toggle
     * alone: no event, whatever the height fields say. */
    for (int k = 0; k < 3; k++) {
        g_imu.onHump = 1;
        for (i = 0; i < 4; i++) { drive(k == 0 ? 0 : 115); (void)drain(); }
        g_imu.onHump = 0; g_imu.last_hump_height_mm = (uint8_t)(20 + k);
        drive(115); s = drain();
        assert(s.humps == 0);
    }
    puts("PASS discarded humps (no travel, lifted) give no EVENT");

    /* 5. Mission EVENT IDs on their own counter keep records contiguous. */
    {
        b1_mission_t m = {0};
        uint16_t seq = 0u;
        uint16_t ev = 0u;
        for (i = 0; i < 10; i++) {
            m.ms = g_usec / 1000u + (uint32_t)i;
            m.event_kind = (i % 3 == 0) ? 14u : 0u;
            if (m.event_kind)
            {
                m.event_id = ++ev;
            }
            m.seq = ++seq;
            assert(b1_ingest_mission(&store, &m) == 0);
        }
        assert(store.uncertain == 0u);
        /* The old shared counter skipped a record number per event. */
        m.event_kind = 13u; m.event_id = ++seq; m.seq = ++seq;
        assert(b1_ingest_mission(&store, &m) == 0);
        assert(store.uncertain == 1u);
        store.uncertain = 0u;
        (void)drain();
    }
    puts("PASS mission EVENT IDs no longer mark the route uncertain");

    /* 6. Offline: nothing drained, the ODOM ring keeps the latest 32
     * records and counts the rest as dropped. */
    {
        uint32_t before = store.dropped;
        for (i = 0; i < 40; i++)
        {
            drive(115);
        }
        assert(store.motion_count == B1_MOTION_DEPTH);
        assert(store.dropped - before == 8u);
        s = drain();
        assert(s.odom == (int)B1_MOTION_DEPTH);
    }
    puts("PASS offline ODOM held in 32-entry ring, overflow counted");
    puts("PASS telemetry route-map checks");
    return 0;
}
/*** end of file ***/
/*** end of file ***/

