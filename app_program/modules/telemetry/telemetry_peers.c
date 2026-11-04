/** @file telemetry_peers.c
 * @brief Read team-owned snapshots and optionally ACK bench barcode results.
 */
#include "telemetry_peers.h"
#include "modules/motion/motion.h"
#include "buddy1_mqtt.h"
#include "buddy1_rt.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/barcode_scanning/bc_cal.h"
#include "modules/barcode_scanning/bc_port.h"
#include "drivers/encoder.h"
#include <tm/tmonitor.h>
#include "modules/terrain/terrain.h"
#include <limits.h>
#include <stddef.h>

static int16_t clamp16(float value)
{
    if(value > (float)INT16_MAX) return INT16_MAX;
    if(value < (float)INT16_MIN) return INT16_MIN;
    return (int16_t)value;
}

uint8_t g7_telemetry_poll(G7_TelemetryPeers *peers,B1_Store *store)
{
    uint8_t updated=0u;
    motion_status_t motion={0};
    ir_shared_snapshot_t ir={0};
    terrain_snapshot imu={0};
    if(peers==NULL || store==NULL)return 0u;

    /* Read encoder owner's atomic count snapshots directly: the motion
     * controller stops updating its *status* while in remote HOLD. Wheel
     * counts must remain observable for stationary commissioning tests.
     * No GPIO, interrupt, I2C or PWM actions occur in this task. */
    if(motion_get_status(&motion)>=E_OK && motion.b_valid) {
        encoder_sample_t left={0},right={0};
        encoder_read(ENCODER_LEFT,&left);
        encoder_read(ENCODER_RIGHT,&right);
        motion.left_count=left.count;
        motion.right_count=right.count;
        motion.timestamp_usec=encoder_now_usec();
        if(!peers->have_motion) {
            peers->have_motion=1u;
            peers->previous_left_count=left.count;
            peers->previous_right_count=right.count;
            peers->motion_last_us=motion.timestamp_usec;
        } else if(left.count!=peers->previous_left_count ||
                  right.count!=peers->previous_right_count) {
            int64_t dl=((int64_t)left.count-(int64_t)peers
                ->previous_left_count)*(int64_t)BC_CAL_LEFT_FORWARD_SIGN;
            int64_t dr=((int64_t)right.count-(int64_t)peers
                ->previous_right_count)*(int64_t)BC_CAL_RIGHT_FORWARD_SIGN;
            peers->previous_left_count=left.count;
            peers->previous_right_count=right.count;
            if(peers->left_mm_per_count>0.0f &&
               peers->right_mm_per_count>0.0f) {
                B1_Motion p={0};
                peers->left_residual_mm+=(float)dl*
                                          peers->left_mm_per_count;
                peers->right_residual_mm+=(float)dr*
                                           peers->right_mm_per_count;
                p.left_delta_mm=clamp16(peers->left_residual_mm);
                p.right_delta_mm=clamp16(peers->right_residual_mm);
                peers->left_residual_mm-=(float)p.left_delta_mm;
                peers->right_residual_mm-=(float)p.right_delta_mm;
                if(p.left_delta_mm!=0 || p.right_delta_mm!=0) {
                    uint32_t dt=motion.timestamp_usec-peers->motion_last_us;
                    p.seq=++peers->motion_seq;
                    p.ms=motion.timestamp_usec/1000u;
                    if(dt>0u) {
                        p.left_speed_mm_s=clamp16((float)
                            p.left_delta_mm*1000000.0f/(float)dt);
                        p.right_speed_mm_s=clamp16((float)
                            p.right_delta_mm*1000000.0f/(float)dt);
                    }
                    if(b1_ingest_motion(store,&p)==0)
                        updated|=B1_SRC_MOTION;
                    peers->motion_last_us=motion.timestamp_usec;
                }
            }
        }
    }
    if(ir_shared_get_latest(&ir)>=E_OK && ir.b_valid &&
       (!peers->have_ir || ir.timestamp_msec!=peers->ir_last_ms)) {
        B1_Line line={0};
        line.seq=++peers->ir_seq;line.ms=ir.timestamp_msec;
        line.ir=(uint8_t)(((ir.left!=0u)?1u:0u)|((ir.right!=0u)?2u:0u)|
                         ((ir.barcode!=0u)?4u:0u));
        line.barcode=peers->barcode_valid ? peers->barcode_letter : 31u;
        peers->last_ir_bits=line.ir;
        line.line=0u; /* raw IR is not authoritative navigation state */
        if(b1_ingest_line(store,&line)==0)updated|=B1_SRC_LINE;
        peers->ir_last_ms=ir.timestamp_msec;peers->have_ir=1u;
    }
    /* Observe Buddy 3's validated Code 39 mailbox, not raw IR guesses.
     * A-Z is transmitted as 0..25. No mission/turn action is triggered.
     * The bench-only ACK switch allows repeated cards to be scanned when
     * no Mission Coordinator has been assigned mailbox ownership. */
    {
        bc_port_result_t decoded={0};
        if(bc_port_peek(&decoded)>=E_OK &&
           (decoded.info & BC_PORT_VALID_FLAG)!=0U &&
           (decoded.info & BC_PORT_COMPLETE_FLAG)!=0U &&
           (decoded.info & BC_PORT_LETTER_MASK)<26U &&
           decoded.sequence!=peers->barcode_last_seq) {
            B1_Line found={0};
            found.seq=++peers->ir_seq;
            found.ms=decoded.timestamp_msec;
            found.ir=peers->last_ir_bits;
            found.barcode=(uint8_t)(decoded.info & BC_PORT_LETTER_MASK);
            found.event_id=(uint16_t)decoded.sequence;
            found.event_kind=1u; /* Barcode decoded. */
            if(b1_ingest_line(store,&found)==0) {
                peers->barcode_last_seq=decoded.sequence;
                peers->barcode_letter=found.barcode;
                peers->barcode_valid=1u;
                updated|=B1_SRC_LINE;
                g7_nav_note_decoded(found.barcode, decoded.sequence);
                tm_printf((UB *)
                    "[BARCODE] Decoded: %c seq=%u (Code 39)\n",
                    (INT)('A'+found.barcode),(UINT)decoded.sequence);
                /* Clear only after successful telemetry handoff. The
                 * scanner never directly changes mission/motor state. */
                (void)bc_port_ack(decoded.sequence);
            }
        }
    }
    if(terrain_get_snapshot(&imu)>=E_OK && imu.b_valid &&
       (!peers->have_imu || imu.timestamp_msec!=peers->imu_last_ms)) {
        B1_Terrain terrain={0};
        terrain.seq=++peers->imu_seq;terrain.ms=imu.timestamp_msec;
        terrain.hump_mm=imu.height_valid ? imu.hump_height_mm : 1023u;
        terrain.max_hump_mm=imu.max_hump_height_mm;
        if(terrain.hump_mm>1022u)terrain.hump_mm=1023u;
        if(terrain.max_hump_mm>1022u)terrain.max_hump_mm=1023u;
        terrain.quality=imu.height_valid?1u:0u;
        terrain.mode=(uint8_t)imu.hump_phase;
        /* Event dedup requires a producer event ID; current published
         * terrain snapshot has none, so do not invent one here. */
        if(b1_ingest_terrain(store,&terrain)==0)updated|=B1_SRC_TERRAIN;
        peers->imu_last_ms=imu.timestamp_msec;peers->have_imu=1u;
    }
    /* A compact extended sensor frame carries actual copied measurements.
     * It is separate from the existing bit-packed 13-byte mission snapshot.
     * Never claim an uninitialised ultrasonic module is measuring distance.
     * This code performs no hardware reads, GPIO writes, or motor calls. */
    {
        uint32_t now=peers->have_ir ? peers->ir_last_ms :
                     peers->imu_last_ms;
        if(now!=0u && now-peers->last_sensor_report_ms>=1000u) {
            B1_Frame f={0};
            uint8_t valid=0u;
            uint16_t distance=65535u; /* No obstacle hardware in this build. */
            uint8_t obstacle_phase=255u; /* Not initialized. */
            f.type=B1_TYPE_SENSOR;
            f.time_ms=now;
            f.payload_len=24u;
            if(ir.b_valid)valid|=1u;
            if(imu.b_valid)valid|=2u;
            if(motion.b_valid)valid|=8u;
            if(imu.heading_valid)valid|=16u;
            f.payload[0]=valid;
            f.payload[1]=peers->last_ir_bits;
            b1_put16(&f.payload[2],(uint16_t)
                clamp16(imu.roll_deg*100.0f));
            b1_put16(&f.payload[4],(uint16_t)
                clamp16(imu.pitch_deg*100.0f));
            b1_put16(&f.payload[6],imu.heading_valid ?
                (uint16_t)(imu.heading_deg*100.0f) : 65535u);
            b1_put16(&f.payload[8],distance);
            f.payload[10]=(uint8_t)imu.hump_phase;
            f.payload[11]=(uint8_t)terrain_is_calibrated();
            b1_put32(&f.payload[12],(uint32_t)motion.left_count);
            b1_put32(&f.payload[16],(uint32_t)motion.right_count);
            f.payload[20]=obstacle_phase;
            f.payload[21]=(uint8_t)imu.onHump;
            f.payload[22]=(uint8_t)imu.height_valid;
            f.payload[23]=peers->odom_confidence;
            if(b1_sensor_frame(store,&f)==0) {
                (void)b1_mqtt_submit(&f);
            }
            tm_printf((UB *)
                "[SENS] IR L=%u R=%u B=%u "
                "roll_cd=%d pitch_cd=%d heading_cd=%u "
                "hump=%u sonar=%u phase=%u "
                "encL=%d encR=%d valid=%u\n",
                (UINT)(peers->last_ir_bits&1u),
                (UINT)((peers->last_ir_bits>>1)&1u),
                (UINT)((peers->last_ir_bits>>2)&1u),
                (INT)clamp16(imu.roll_deg*100.0f),
                (INT)clamp16(imu.pitch_deg*100.0f),
                (UINT)f.payload[6]+((UINT)f.payload[7]<<8),
                (UINT)imu.hump_height_mm,(UINT)distance,
                (UINT)obstacle_phase,
                (INT)motion.left_count,(INT)motion.right_count,
                (UINT)valid);
            peers->last_sensor_report_ms=now;
        }
    }
    /* Barcode acquisition diagnostics over MQTT, NOT USB logs. A decoded
     * result is already transmitted through a separate line EVENT. Reports
     * are diagnostics only; they never ACK or control the scanner. */
    {
        uint32_t now=peers->have_ir ? peers->ir_last_ms :
                     peers->imu_last_ms;
        if(now!=0u && now-peers->last_barcode_report_ms>=1000u) {
            bc_port_stats_t d={0};
            if(bc_port_stats(&d)>=E_OK) {
                B1_Frame f={0};
                encoder_sample_t left={0},right={0};
                encoder_read(ENCODER_LEFT,&left);
                encoder_read(ENCODER_RIGHT,&right);
                f.type=B1_TYPE_BARCODE_DIAG;
                f.time_ms=now;
                f.payload_len=28u;
                f.payload[0]=(uint8_t)(BC_CAL_READY ? 2u : 1u);
                f.payload[1]=(uint8_t)d.state;
                f.payload[2]=(uint8_t)d.last_error;
                f.payload[3]=(uint8_t)(d.raw&0xffu);
                b1_put32(&f.payload[4],d.rejected);
                b1_put32(&f.payload[8],d.completed);
                b1_put16(&f.payload[12],
                    (uint16_t)(d.last_scan_units>65535u ? 65535u :
                               d.last_scan_units));
                f.payload[14]=(uint8_t)(d.runs>255u ? 255u :d.runs);
                f.payload[15]=(uint8_t)d.last_error;
                b1_put32(&f.payload[16],(uint32_t)left.count);
                b1_put32(&f.payload[20],(uint32_t)right.count);
                b1_put32(&f.payload[24],d.edges);
                if(b1_sensor_frame(store,&f)==0) {
                    (void)b1_mqtt_submit(&f);
                }
            }
            peers->last_barcode_report_ms=now;
        }
    }
    return updated;
}
/*** end of file ***/

