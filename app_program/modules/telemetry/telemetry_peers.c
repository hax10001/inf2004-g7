/** @file telemetry_peers.c
 * @brief Copy subsystem snapshots and hand off validated barcode results.
 */
#include "telemetry_peers.h"
#include "modules/motion/motion.h"
#include "buddy1_mqtt.h"
#include "buddy1_rt.h"
#include "modules/line_barcode/ir_shared.h"
#include "modules/barcode_scanning/bc_cal.h"
#include "modules/barcode_scanning/bc_port.h"
#include "modules/barcode_scanning/bc_wave.h"
#include "drivers/encoder.h"
#include <tm/tmonitor.h>
#include "modules/terrain/terrain.h"
#include "modules/obstacle/obs_state.h"
#include "app/app_config.h"
#include <limits.h>
#include <stddef.h>

/* 10-bit telemetry fields (hump mm, obstacle cm, lift mm): 1023 means
 * "unavailable" to protocol.js, so real values stop at 1022. */
#define PEERS_FIELD10_NONE 1023u
#define PEERS_FIELD10_MAX  1022u
#define PEERS_US_PER_MS    1000u        /* microseconds per millisecond */
#define PEERS_US_PER_S     1000000.0f   /* microseconds per second */
#define PEERS_CENTI        100.0f       /* degrees to centidegrees */
#define PEERS_REPORT_MS    1000u        /* sensor/barcode detail period */
/* IR bits: left 0, right 1, barcode sensor 2. */
#define PEERS_IR_LEFT      1u
#define PEERS_IR_RIGHT     2u
#define PEERS_IR_BARCODE   4u
#define PEERS_IR_BC_SHIFT  2            /* barcode bit position */
/* Sensor frame byte 0: which measurements are valid. */
#define PEERS_VALID_IR       1u
#define PEERS_VALID_IMU      2u
#define PEERS_VALID_OBSTACLE 4u
#define PEERS_VALID_MOTION   8u
#define PEERS_VALID_HEADING  16u
#define PEERS_U16_NONE     65535u       /* 16-bit field unavailable */
#define PEERS_U16_MAX      65535u       /* 16-bit saturation */
#define PEERS_U8_MAX       255u         /* 8-bit saturation */
#define PEERS_PHASE_NONE   255u         /* obstacle phase unavailable */
#define PEERS_BYTE_MASK    0xffu        /* low byte */
#define PEERS_CAL_READY    2u           /* barcode calibration measured */

/** @brief Clamp a float to the int16_t range. */
static int16_t clamp16(float value)
{
    if(value > (float)INT16_MAX)
    {
        return INT16_MAX;
    }
    if(value < (float)INT16_MIN)
    {
        return INT16_MIN;
    }
    return (int16_t)value;
}

uint8_t g7_telemetry_poll(g7_telemetry_peers_t *p_peers,b1_store_t *p_store)
{
    uint8_t updated=0u;
    motion_status_t motion={0};
    ir_shared_snapshot_t ir={0};
    terrain_snapshot_t imu={0};
    if(p_peers==NULL || p_store==NULL)
    {
        return 0u;
    }

    /* Read encoder counts directly: motion status stops updating during
     * remote HOLD, but wheel counts must stay observable while stopped. */
    if(motion_get_status(&motion)>=E_OK && motion.b_valid)
    {
        encoder_sample_t left={0},right={0};
        encoder_read(ENCODER_LEFT,&left);
        encoder_read(ENCODER_RIGHT,&right);
        motion.left_count=left.count;
        motion.right_count=right.count;
        motion.timestamp_usec=encoder_now_usec();
        if(!p_peers->have_motion)
        {
            p_peers->have_motion=1u;
            p_peers->previous_left_count=left.count;
            p_peers->previous_right_count=right.count;
            p_peers->motion_last_us=motion.timestamp_usec;
        }
        else if(left.count!=p_peers->previous_left_count ||
            right.count!=p_peers->previous_right_count)
        {
            int64_t dl=((int64_t)left.count-(int64_t)p_peers
                ->previous_left_count)*(int64_t)BC_CAL_LEFT_FORWARD_SIGN;
            int64_t dr=((int64_t)right.count-(int64_t)p_peers
                ->previous_right_count)*(int64_t)BC_CAL_RIGHT_FORWARD_SIGN;
            p_peers->previous_left_count=left.count;
            p_peers->previous_right_count=right.count;
            if(p_peers->left_mm_per_count>0.0f &&
                p_peers->right_mm_per_count>0.0f)
            {
                b1_motion_t p={0};
                p_peers->left_residual_mm+=(float)dl*
                p_peers->left_mm_per_count;
                p_peers->right_residual_mm+=(float)dr*
                p_peers->right_mm_per_count;
                p.left_delta_mm=clamp16(p_peers->left_residual_mm);
                p.right_delta_mm=clamp16(p_peers->right_residual_mm);
                p_peers->left_residual_mm-=(float)p.left_delta_mm;
                p_peers->right_residual_mm-=(float)p.right_delta_mm;
                if(p.left_delta_mm!=0 || p.right_delta_mm!=0)
                {
                    uint32_t dt=motion.timestamp_usec-p_peers->motion_last_us;
                    uint32_t before=p_peers->odom_travel_x2;
                    int32_t sum=(int32_t)p.left_delta_mm+
                    (int32_t)p.right_delta_mm;
                    uint32_t step=(uint32_t)(sum<0 ? -sum : sum);
                    p.seq=++p_peers->motion_seq;
                    p.ms=motion.timestamp_usec/PEERS_US_PER_MS;
                    /* Same distance the dashboard integrates, so a
                     * checkpoint can bridge ODOM records lost in a gap. */
                    p_peers->odom_travel_x2=before+step;
                    p.cumulative_mm=p_peers->odom_travel_x2/2u;
                    p.checkpoint=(before/(2u*B1_CHECKPOINT_MM))!=
                    (p_peers->odom_travel_x2/(2u*B1_CHECKPOINT_MM));
                    if(dt>0u)
                    {
                        p.left_speed_mm_s=clamp16((float)
                            p.left_delta_mm*PEERS_US_PER_S/(float)dt);
                        p.right_speed_mm_s=clamp16((float)
                            p.right_delta_mm*PEERS_US_PER_S/(float)dt);
                    }
                    if(b1_ingest_motion(p_store,&p)==0)
                    {
                        updated|=B1_SRC_MOTION;
                    }
                    p_peers->motion_last_us=motion.timestamp_usec;
                }
            }
        }
    }
    if(ir_shared_get_latest(&ir)>=E_OK && ir.b_valid &&
        (!p_peers->have_ir || ir.timestamp_msec!=p_peers->ir_last_ms))
    {
        b1_line_t line={0};
        line.seq=++p_peers->ir_seq;
        line.ms=ir.timestamp_msec;
        line.ir=(uint8_t)(((ir.left!=0u)?PEERS_IR_LEFT:0u)|
            ((ir.right!=0u)?PEERS_IR_RIGHT:0u)|
            ((ir.barcode!=0u)?PEERS_IR_BARCODE:0u));
        line.barcode=p_peers->barcode_valid ? p_peers->barcode_letter :
            B1_LETTER_NONE;
        p_peers->last_ir_bits=line.ir;
        line.line=0u; /* raw IR is not navigation state */
        if(b1_ingest_line(p_store,&line)==0)
        {
            updated|=B1_SRC_LINE;
        }
        p_peers->ir_last_ms=ir.timestamp_msec;
        p_peers->have_ir=1u;
    }
    /* Report each validated Code 39 letter (A-Z as 0..25) as a line
     * EVENT, and ACK the scanner only after the store accepted it. */
    {
        bc_port_result_t decoded={0};
        if(bc_port_peek(&decoded)>=E_OK &&
            (decoded.info & BC_PORT_VALID_FLAG)!=0U &&
            (decoded.info & BC_PORT_COMPLETE_FLAG)!=0U &&
            (decoded.info & BC_PORT_LETTER_MASK)<B1_LETTER_COUNT &&
            decoded.sequence!=p_peers->barcode_last_seq)
        {
            b1_line_t found={0};
            found.seq=++p_peers->ir_seq;
            found.ms=decoded.timestamp_msec;
            found.ir=p_peers->last_ir_bits;
            found.barcode=(uint8_t)(decoded.info & BC_PORT_LETTER_MASK);
            found.event_id=(uint16_t)decoded.sequence;
            found.event_kind=B1_EVT_BARCODE;
            if(b1_ingest_line(p_store,&found)==0)
            {
                p_peers->barcode_last_seq=decoded.sequence;
                p_peers->barcode_letter=found.barcode;
                p_peers->barcode_valid=1u;
                updated|=B1_SRC_LINE;
                tm_printf((UB *)
                    "[BARCODE] Decoded: %c seq=%u (Code 39)\n",
                    (INT)('A'+found.barcode),(UINT)decoded.sequence);
                /* The scanner never changes mission or motor state. */
                (void)bc_port_ack(decoded.sequence);
            }
        }
    }
    if(terrain_get_snapshot(&imu)>=E_OK && imu.b_valid &&
        (!p_peers->have_imu || imu.timestamp_msec!=p_peers->imu_last_ms))
    {
        b1_terrain_t terrain={0};
        terrain.seq=++p_peers->imu_seq;
        terrain.ms=imu.timestamp_msec;
        terrain.hump_mm=imu.height_valid ? imu.hump_height_mm :
        PEERS_FIELD10_NONE;
        terrain.max_hump_mm=imu.max_hump_height_mm;
        if(terrain.hump_mm>PEERS_FIELD10_MAX)
        {
            terrain.hump_mm=PEERS_FIELD10_NONE;
        }
        if(terrain.max_hump_mm>PEERS_FIELD10_MAX)
        {
            terrain.max_hump_mm=PEERS_FIELD10_NONE;
        }
        terrain.quality=imu.height_valid?1u:0u;
        terrain.mode=(uint8_t)imu.hump_phase;
        /* One EVENT per hump the terrain task recorded (HUMP DONE): it
         * flips hump_done_toggle then, and never for a discarded one (no
         * travel, lifted, no climb). Terrain outputs are read only. */
        if(p_peers->have_imu &&
            imu.hump_done_toggle!=p_peers->hump_toggle)
        {
            ++p_peers->hump_event_seq;
            if(p_peers->hump_event_seq==0u)
            {
                p_peers->hump_event_seq=1u;
            }
            terrain.event_kind=B1_EVT_HUMP_CROSSED;
            terrain.event_value=imu.last_hump_height_mm;
        }
        p_peers->hump_toggle=imu.hump_done_toggle;
        terrain.event_id=p_peers->hump_event_seq;
        if(b1_ingest_terrain(p_store,&terrain)==0)
        {
            updated|=B1_SRC_TERRAIN;
        }
        p_peers->imu_last_ms=imu.timestamp_msec;
        p_peers->have_imu=1u;
    }
    /* Copy the obstacle task's packed status word; no hardware access. */
#if APP_OBSTACLE_ENABLED
    {
        uint32_t now=p_peers->have_ir ? p_peers->ir_last_ms
            : p_peers->imu_last_ms;
        uint32_t word=obs_state_status_get();
        uint32_t previous=p_peers->obstacle_last_status;
        b1_obstacle_t obstacle={0};
        obstacle.seq=++p_peers->obstacle_seq;
        obstacle.ms=now;
        obstacle.distance_cm=(uint16_t)obs_state_status_distance_cm(word);
        obstacle.faults=(uint8_t)obs_state_status_faults(word);
        if(obstacle.distance_cm==OBS_DIST_INVALID_CM ||
            (obstacle.faults & OBS_FAULT_NO_ECHO)!=0u)
        {
            obstacle.distance_cm=PEERS_FIELD10_NONE;
        }
        obstacle.side=(uint8_t)obs_state_status_route(word);
        obstacle.bypass_count=(uint8_t)obs_state_status_count(word);
        if(obs_state_status_faults(word)!=0u &&
            (!p_peers->have_obstacle || obs_state_status_faults(previous)!=
            obs_state_status_faults(word)))
        {
            obstacle.event_kind=B1_EVT_SAFETY_FAULT;
            obstacle.event_value=obstacle.faults;
        }
        else if(obs_state_status_phase(word)==OBS_PHASE_CONFIRM &&
            (!p_peers->have_obstacle ||
            obs_state_status_phase(previous)!=OBS_PHASE_CONFIRM))
        {
            obstacle.event_kind=B1_EVT_OBSTACLE;
            obstacle.event_value=obstacle.distance_cm;
        }
        else if(obstacle.side!=OBS_ROUTE_NONE &&
            obs_state_status_route(previous)!=obstacle.side)
        {
            obstacle.event_kind=B1_EVT_BYPASS_CHOSEN;
            obstacle.event_value=obstacle.side;
        }
        else if(p_peers->have_obstacle &&
            obs_state_status_count(previous)!=obstacle.bypass_count)
        {
            obstacle.event_kind=B1_EVT_BYPASSED;
        }
        if(0U != obstacle.event_kind)
        {
            ++p_peers->obstacle_event_seq;
        }
        obstacle.event_id=p_peers->obstacle_event_seq;
        if(now!=0u && b1_ingest_obstacle(p_store,&obstacle)==0)
        {
            updated|=B1_SRC_OBSTACLE;
            p_peers->have_obstacle=1u;
            p_peers->obstacle_last_status=word;
        }
    }
#endif
    /* SENSOR detail frame of copied measurements, separate from the
     * 13-byte SNAPSHOT. No hardware reads, GPIO writes or motor calls. */
    {
        uint32_t now=p_peers->have_ir ? p_peers->ir_last_ms :
        p_peers->imu_last_ms;
        if(now!=0u && now-p_peers->last_sensor_report_ms>=PEERS_REPORT_MS)
        {
            b1_frame_t f={0};
            uint8_t valid=0u;
            uint16_t distance=PEERS_U16_NONE;
            uint8_t obstacle_phase=PEERS_PHASE_NONE;
#if APP_OBSTACLE_ENABLED
            uint32_t obstacle_word=obs_state_status_get();
            uint32_t obstacle_cm=obs_state_status_distance_cm(obstacle_word);
            obstacle_phase=(uint8_t)obs_state_status_phase(obstacle_word);
            valid|=PEERS_VALID_OBSTACLE;
            if(obstacle_cm!=OBS_DIST_INVALID_CM &&
                (obs_state_status_faults(obstacle_word) &
                OBS_FAULT_NO_ECHO)==0u)
            {
                distance=(uint16_t)obstacle_cm;
            }
#endif
            f.type=B1_TYPE_SENSOR;
            f.time_ms=now;
            f.payload_len=B1_SENSOR_LIFT;
            if(ir.b_valid)
            {
                valid|=PEERS_VALID_IR;
            }
            if(imu.b_valid)
            {
                valid|=PEERS_VALID_IMU;
            }
            if(motion.b_valid)
            {
                valid|=PEERS_VALID_MOTION;
            }
            if(imu.heading_valid)
            {
                valid|=PEERS_VALID_HEADING;
            }
            f.payload[0]=valid;
            f.payload[1]=p_peers->last_ir_bits;
            b1_put16(&f.payload[B1_SENSOR_OFF_ROLL],(uint16_t)
                clamp16(imu.roll_deg*PEERS_CENTI));
            b1_put16(&f.payload[B1_SENSOR_OFF_PITCH],(uint16_t)
                clamp16(imu.pitch_deg*PEERS_CENTI));
            b1_put16(&f.payload[B1_SENSOR_OFF_HEADING],imu.heading_valid ?
                imu.heading_centideg : PEERS_U16_NONE);
            b1_put16(&f.payload[B1_SENSOR_OFF_DISTANCE],distance);
            f.payload[B1_SENSOR_OFF_HUMP_PHASE]=(uint8_t)imu.hump_phase;
            f.payload[B1_SENSOR_OFF_CALIBRATED]=
                (uint8_t)terrain_is_calibrated();
            b1_put32(&f.payload[B1_SENSOR_OFF_LEFT_COUNT],
                (uint32_t)motion.left_count);
            b1_put32(&f.payload[B1_SENSOR_OFF_RIGHT_COUNT],
                (uint32_t)motion.right_count);
            f.payload[B1_SENSOR_OFF_OBS_PHASE]=obstacle_phase;
            f.payload[B1_SENSOR_OFF_ON_HUMP]=(uint8_t)imu.onHump;
            f.payload[B1_SENSOR_OFF_HEIGHT_OK]=(uint8_t)imu.height_valid;
            f.payload[B1_SENSOR_OFF_ODOM_CONF]=p_peers->odom_confidence;
            /* Lifted height 0..1022 mm in ten bits (1023 unavailable),
             * then the lifted flag; the top five bits stay zero. */
            b1_put16(&f.payload[B1_SENSOR_OFF_LIFT],(uint16_t)
                ((imu.b_valid ? (uint16_t)imu.lift_height_mm :
                PEERS_FIELD10_NONE) |
                ((imu.b_valid && imu.b_lifted) ? B1_LIFT_FLAG : 0u)));
            /* A detail frame is a moment's reading: when it cannot be
             * sent now, skip it rather than reserve a sequence number
             * that never arrives (the host would see a gap). */
            if(b1_mqtt_can_send() && b1_sensor_frame(p_store,&f)==0)
            {
                (void)b1_mqtt_submit(&f);
            }
#ifndef B1_BARCODE_TEST /* quiet barcode test log */
            tm_printf((UB *)
                "[SENS] IR L=%u R=%u B=%u "
                "roll_cd=%d pitch_cd=%d heading_cd=%u "
                "hump=%u sonar=%u phase=%u "
                "encL=%d encR=%d valid=%u lift=%u\n",
                (UINT)(p_peers->last_ir_bits&1u),
                (UINT)((p_peers->last_ir_bits>>1)&1u),
                (UINT)((p_peers->last_ir_bits>>PEERS_IR_BC_SHIFT)&1u),
                (INT)clamp16(imu.roll_deg*PEERS_CENTI),
                (INT)clamp16(imu.pitch_deg*PEERS_CENTI),
                (UINT)f.payload[B1_SENSOR_OFF_HEADING]+
                ((UINT)f.payload[B1_SENSOR_OFF_HEADING_HI]<<B1_BYTE_SHIFT),
                (UINT)imu.hump_height_mm,(UINT)distance,
                (UINT)obstacle_phase,
                (INT)motion.left_count,(INT)motion.right_count,
                (UINT)valid,(UINT)imu.lift_height_mm);
#endif
            p_peers->last_sensor_report_ms=now;
        }
    }
    /* BARCODE_DIAG frame over MQTT. Decoded letters go out as line EVENTs;
     * this frame never ACKs or controls the scanner. */
    {
        uint32_t now=p_peers->have_ir ? p_peers->ir_last_ms :
        p_peers->imu_last_ms;
        if(now!=0u && now-p_peers->last_barcode_report_ms>=PEERS_REPORT_MS)
        {
            bc_port_stats_t d={0};
            if(bc_port_stats(&d)>=E_OK)
            {
                b1_frame_t f={0};
                encoder_sample_t left={0},right={0};
                encoder_read(ENCODER_LEFT,&left);
                encoder_read(ENCODER_RIGHT,&right);
                f.type=B1_TYPE_BARCODE_DIAG;
                f.time_ms=now;
                f.payload_len=B1_BARCODE_DIAG_V2;
                f.payload[0]=(uint8_t)(BC_CAL_READY ? PEERS_CAL_READY : 1u);
                f.payload[1]=(uint8_t)d.state;
                f.payload[B1_BCD_OFF_ERROR]=(uint8_t)d.last_error;
                f.payload[B1_BCD_OFF_RAW]=(uint8_t)(d.raw&PEERS_BYTE_MASK);
                b1_put32(&f.payload[B1_BCD_OFF_REJECTED],d.rejected);
                b1_put32(&f.payload[B1_BCD_OFF_COMPLETED],d.completed);
                b1_put16(&f.payload[B1_BCD_OFF_SCAN_UNITS],
                    (uint16_t)(d.last_scan_units>PEERS_U16_MAX ?
                    PEERS_U16_MAX : d.last_scan_units));
                f.payload[B1_BCD_OFF_RUNS]=(uint8_t)(d.runs>PEERS_U8_MAX ?
                    PEERS_U8_MAX :d.runs);
                f.payload[B1_BCD_OFF_ERROR_COPY]=(uint8_t)d.last_error;
                b1_put32(&f.payload[B1_BCD_OFF_LEFT],(uint32_t)left.count);
                b1_put32(&f.payload[B1_BCD_OFF_RIGHT],(uint32_t)right.count);
                b1_put32(&f.payload[B1_BCD_OFF_EDGES],d.edges);
                {
                    /* V2 fields: scanner state, learned white, last pass. */
                    bc_wave_status_t w={0};
                    bc_wave_status(&w);
                    f.payload[B1_BCD_OFF_WAVE_STATE]=w.state;
                    f.payload[B1_BCD_OFF_WHITE_REJ]=w.white_rejects;
                    b1_put16(&f.payload[B1_BCD_OFF_WHITE_MEAN],w.white_mean);
                    b1_put16(&f.payload[B1_BCD_OFF_WHITE_SIGMA],
                        w.white_sigma);
                    b1_put16(&f.payload[B1_BCD_OFF_WAVE_SEQ],w.sequence);
                    f.payload[B1_BCD_OFF_LAST_VALID]=w.last_valid;
                    f.payload[B1_BCD_OFF_LAST_LETTER]=w.last_letter;
                    f.payload[B1_BCD_OFF_LAST_REASON]=w.last_reason;
                    f.payload[B1_BCD_OFF_RESERVED]=0u;
                    b1_put16(&f.payload[B1_BCD_OFF_PROCESS_MS],
                        w.last_process_ms);
                }
                if(b1_mqtt_can_send() && b1_sensor_frame(p_store,&f)==0)
                {
                    (void)b1_mqtt_submit(&f);
                }
            }
            p_peers->last_barcode_report_ms=now;
        }
    }
    return updated;
}
/*** end of file ***/

