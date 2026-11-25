/** @file buddy1.c
 * @brief Mutex-protected telemetry store and bounded frame aggregation.
 */
#include "buddy1.h"
#include <string.h>

/* Frame periods (FR12). STATUS also carries readiness and mapping. */
#define B1_STATUS_PERIOD_MS    1000u
#define B1_HEARTBEAT_PERIOD_MS 2000u
#define B1_SNAPSHOT_PERIOD_MS  250u
/* A source not updated for this long is flagged stale in SNAPSHOT. */
#define B1_SOURCE_STALE_MS     1500u
/* A STATUS time token answers a command challenge for this long. */
#define B1_CHALLENGE_MS        8000u
/* Sequence numbers within half the 16-bit range are newer. */
#define B1_SEQ_HALF            0x8000u

/** @brief Saturate a value to the 10-bit snapshot range so the bit-field
 *  cannot wrap before encoding. */
static uint16_t snap_range10(uint16_t value)
{
    return (value > B1_RANGE10_NONE) ? B1_RANGE10_NONE : value;
}
/** @brief Clamp a speed to the signed 12-bit wire range. */
static int16_t snap_speed12(int16_t value)
{
    if(value < -B1_SPEED12_MAX)
    {
        return -B1_SPEED12_MAX;
    }
    if(value > B1_SPEED12_MAX)
    {
        return B1_SPEED12_MAX;
    }
    return value;
}

/** @brief Return nonzero if a sequence number is newer (wrap-safe). */
static int32_t fresh(uint16_t prev,uint16_t curr)
{
    return curr != prev && (uint16_t)(curr - prev) < B1_SEQ_HALF;
}
/** @brief Append an EVENT to the bounded ring; count overwrites. */
static void put_event(b1_store_t *p_store,uint8_t src,uint8_t kind,uint16_t id,
    uint16_t val,uint32_t ms)
{
    uint8_t idx;
    if(kind == 0u)
    {
        return;
    }
    if(p_store->event_count == B1_EVENT_DEPTH)
    {
        /* Overwrite the oldest event and flag the history as incomplete. */
        p_store->event_head=(uint8_t)((p_store->event_head+1u)%B1_EVENT_DEPTH);
        p_store->event_count--;
        p_store->dropped++;
        p_store->uncertain=1u;
    }
    idx=(uint8_t)((p_store->event_head+p_store->event_count)%B1_EVENT_DEPTH);
    /* Byte-addressed, so safe on the Cortex-M0+ (no unaligned access). */
    p_store->events[idx].data[0] = (uint8_t)((src & B1_EVREC_SRC_MASK) |
        ((uint8_t)(kind & B1_EVREC_KIND_MASK) << B1_EVREC_KIND_SHIFT));
    b1_put16(p_store->events[idx].data+B1_EVREC_OFF_ID,id);
    b1_put16(p_store->events[idx].data+B1_EVREC_OFF_VALUE,val);
    b1_put32(p_store->events[idx].data+B1_EVREC_OFF_TIME,ms);
    p_store->event_count++;
}
/** @brief Initialise a store; see buddy1.h. */
int32_t b1_store_init(b1_store_t *p_store,b1_lock_fn_t lock,b1_lock_fn_t unlock,
    void *p_ctx,
    uint16_t session)
{
    if(p_store==NULL || lock==NULL || unlock==NULL)
    {
        return -1;
    }
    memset(p_store,0,sizeof(*p_store));
    p_store->lock=lock;
    p_store->unlock=unlock;
    p_store->lock_context=p_ctx;
    p_store->session=session==0u?1u:session;
    p_store->line.barcode=B1_LETTER_NONE;
    return 0;
}
/** @brief Register the callback that applies commands. */
void b1_set_coordinator(b1_store_t *p_store,b1_coordinator_fn_t fn,void *p_ctx)
{
    if(p_store==NULL)
    {
        return;
    }
    p_store->lock(p_store->lock_context);
    p_store->coordinator=fn;
    p_store->coordinator_context=p_ctx;
    p_store->unlock(p_store->lock_context);
}
/** @brief Store a motion record and queue it as ODOM. */
int32_t b1_ingest_motion(b1_store_t *p_store,const b1_motion_t *p_rec)
{
    uint8_t idx;
    if(p_store==NULL || p_rec==NULL)
    {
        return -1;
    }
    p_store->lock(p_store->lock_context);
    if((p_store->seen&B1_SRC_MOTION)!=0u && !fresh(p_store->motion.seq,
        p_rec->seq))
    {
        p_store->dup++;
        p_store->unlock(p_store->lock_context);
        return -1;
    }
    if((p_store->seen&B1_SRC_MOTION)!=0u && (uint16_t)(
        p_rec->seq-p_store->motion.seq)>1u)
    {
        p_store->uncertain=1u;
    }
    p_store->motion=*p_rec;
    p_store->seen|=B1_SRC_MOTION;
    if(p_store->motion_count==B1_MOTION_DEPTH)
    {
        p_store->motion_head=(uint8_t)((
            p_store->motion_head+1u)%B1_MOTION_DEPTH);
        p_store->motion_count--;
        p_store->dropped++;
        p_store->uncertain=1u;
    }
    idx=(uint8_t)((p_store->motion_head+p_store->motion_count)%B1_MOTION_DEPTH);
    /* The ring keeps only the ODOM payload; speeds, cumulative distance
     * and the checkpoint stay in p_store->motion and checkpoint_*. */
    b1_put32(p_store->motions[idx].data,p_rec->ms);
    b1_put16(p_store->motions[idx].data+B1_ODREC_OFF_LEFT,
        (uint16_t)p_rec->left_delta_mm);
    b1_put16(p_store->motions[idx].data+B1_ODREC_OFF_RIGHT,
        (uint16_t)p_rec->right_delta_mm);
    b1_put16(p_store->motions[idx].data+B1_ODREC_OFF_SEQ,p_rec->seq);
    p_store->motion_count++;
    if(0U != p_rec->checkpoint)
    {
        p_store->checkpoint_pending=1u;
        p_store->checkpoint_mm=p_rec->cumulative_mm;
        p_store->checkpoint_ms=p_rec->ms;
    }
    p_store->unlock(p_store->lock_context);
    return 0;
}
/* Generates b1_ingest_<NAME>() for one telemetry source. NAME, TYPE and
 * FIELD are tokens (function name, record type, store member), so they
 * cannot be parenthesised; MASK, SOURCE and VAL are expressions and are. */
#define B1_SOURCE_INGEST(NAME,TYPE,FIELD,MASK,SOURCE,VAL) \
int32_t b1_ingest_##NAME(b1_store_t *p_store,const TYPE *p_rec) \
{ \
    if((NULL==p_store)||(NULL==p_rec)) \
    { \
        return -1; \
    } \
    p_store->lock(p_store->lock_context); \
    if(((p_store->seen & (MASK))!=0u) && \
       !fresh(p_store->FIELD.seq,p_rec->seq)) \
    { \
        p_store->dup++; \
        p_store->unlock(p_store->lock_context); \
        return -1; \
    } \
    if(((p_store->seen & (MASK))!=0u) && \
       ((uint16_t)(p_rec->seq-p_store->FIELD.seq)>1u)) \
    { \
        p_store->uncertain=1u; \
    } \
    if((p_rec->event_kind!=0u) && \
       (((p_store->seen&(MASK))==0u) || \
        (p_rec->event_id!=p_store->FIELD.event_id))) \
    { \
        put_event(p_store,(SOURCE),p_rec->event_kind,p_rec->event_id, \
                  (uint16_t)(VAL),p_rec->ms); \
    } \
    p_store->FIELD=*p_rec; \
    p_store->seen|=(MASK); \
    p_store->unlock(p_store->lock_context); \
    return 0; \
}
B1_SOURCE_INGEST(line,b1_line_t,line,B1_SRC_LINE,B1_EVSRC_LINE,
    p_rec->barcode)
B1_SOURCE_INGEST(terrain,b1_terrain_t,terrain,B1_SRC_TERRAIN,
    B1_EVSRC_TERRAIN,p_rec->event_value)
B1_SOURCE_INGEST(obstacle,b1_obstacle_t,obstacle,B1_SRC_OBSTACLE,
    B1_EVSRC_OBSTACLE,p_rec->event_value)
/** @brief Store a mission record. Unlike other sources, a confirmed
 *  controller hold ends the START commitment; an ACKed STOP alone does
 *  not prove the motors stopped. */
int32_t b1_ingest_mission(b1_store_t *p_store,const b1_mission_t *p_rec)
{
    if(p_store==NULL||p_rec==NULL)
    {
        return -1;
    }
    p_store->lock(p_store->lock_context);
    if((p_store->seen&B1_SRC_MISSION)!=0u &&
        !fresh(p_store->mission.seq,p_rec->seq))
    {
        p_store->dup++;
        p_store->unlock(p_store->lock_context);
        return -1;
    }
    if((p_store->seen&B1_SRC_MISSION)!=0u &&
        (uint16_t)(p_rec->seq-p_store->mission.seq)>1u)
    {
        p_store->uncertain=1u;
    }
    if(p_rec->event_kind!=0u &&
        ((p_store->seen&B1_SRC_MISSION)==0u ||
        p_rec->event_id!=p_store->mission.event_id))
    {
        put_event(p_store,B1_EVSRC_MISSION,p_rec->event_kind,p_rec->event_id,
            p_rec->event_value,p_rec->ms);
    }
    p_store->mission=*p_rec;
    p_store->seen|=B1_SRC_MISSION;
    if(p_rec->remote_hold!=0u)
    {
        p_store->committed=0u;
    }
    p_store->unlock(p_store->lock_context);
    return 0;
}

/** @brief Return the mask of sources not updated recently. */
static uint8_t stale(const b1_store_t *p_store,uint32_t now)
{
    uint8_t mask=0u;
    if((p_store->seen&B1_SRC_MOTION)!=0u
        && now-p_store->motion.ms>B1_SOURCE_STALE_MS)
    {
        mask|=B1_SRC_MOTION;
    }
    if((p_store->seen&B1_SRC_LINE)!=0u
        && now-p_store->line.ms>B1_SOURCE_STALE_MS)
    {
        mask|=B1_SRC_LINE;
    }
    if((p_store->seen&B1_SRC_TERRAIN)!=0u
        && now-p_store->terrain.ms>B1_SOURCE_STALE_MS)
    {
        mask|=B1_SRC_TERRAIN;
    }
    if((p_store->seen&B1_SRC_MISSION)!=0u
        && now-p_store->mission.ms>B1_SOURCE_STALE_MS)
    {
        mask|=B1_SRC_MISSION;
    }
    if((p_store->seen&B1_SRC_OBSTACLE)!=0u
        && now-p_store->obstacle.ms>B1_SOURCE_STALE_MS)
    {
        mask|=B1_SRC_OBSTACLE;
    }
    return mask;
}
int32_t b1_next_frame(b1_store_t *p_store,uint32_t now_ms,b1_frame_t *p_out)
{
    b1_snapshot_payload_t x;
    uint8_t pending=0u;
    if(p_store==NULL||p_out==NULL)
    {
        return 0;
    }
    p_store->lock(p_store->lock_context);
    memset(p_out,0,sizeof(*p_out));
    p_out->session=p_store->session;
    p_out->seq=++p_store->wire_seq;
    p_out->time_ms=now_ms;
    p_out->flags=p_store->uncertain ? 1u : 0u;
    if(now_ms-p_store->last_status_ms>=B1_STATUS_PERIOD_MS)
    {
        /* Robot-reported readiness and mapping state. */
        p_store->last_status_ms=now_ms;
        p_store->status_challenges[p_store->status_challenge_head]=now_ms;
        p_store->status_challenge_head=(uint8_t)
        ((p_store->status_challenge_head+1u)%B1_CHALLENGE_SLOTS);
        p_out->type=B1_TYPE_STATUS;
        p_out->payload_len=B1_STATUS_LEN;
        p_out->payload[0]=p_store->mapping_valid;
        p_out->payload[1]=p_store->mission.ready;
        p_out->payload[B1_STATUS_OFF_RUNNING]=p_store->mission.running;
        p_out->payload[B1_STATUS_OFF_SEEN]=p_store->seen;
        p_out->payload[B1_STATUS_OFF_MISSION]=p_store->mission.mission;
        p_out->payload[B1_STATUS_OFF_BLOCKERS]=
            p_store->mission.ready_blockers;
        p_out->payload[B1_STATUS_OFF_HOLD]=p_store->mission.remote_hold;
        b1_put16(p_out->payload+B1_STATUS_OFF_TRANSACTION,
            p_store->last_cmd_transaction);
        p_out->payload[B1_STATUS_OFF_BC_HOLD]=p_store->mission.barcode_hold;
        b1_put16(p_out->payload+B1_STATUS_OFF_PWM_LEFT,
            p_store->mission.pwm_left);
        b1_put16(p_out->payload+B1_STATUS_OFF_PWM_RIGHT,
            p_store->mission.pwm_right);
        pending=1u;
    }
    else if(p_store->event_count!=0u)
    {
        b1_event_t e=p_store->events[p_store->event_head];
        p_store->event_head=(uint8_t)((p_store->event_head+1u)%B1_EVENT_DEPTH);
        p_store->event_count--;
        p_out->type=B1_TYPE_EVENT;
        p_out->time_ms=b1_get32(e.data+B1_EVREC_OFF_TIME);
        p_out->payload_len=B1_EVENT_LEN;
        p_out->payload[0]=(uint8_t)(e.data[0] & B1_EVREC_SRC_MASK);
        p_out->payload[1]=(uint8_t)(e.data[0] >> B1_EVREC_KIND_SHIFT);
        b1_put16(p_out->payload+B1_EVENT_OFF_ID,
            b1_get16(e.data+B1_EVREC_OFF_ID));
        b1_put16(p_out->payload+B1_EVENT_OFF_VALUE,
            b1_get16(e.data+B1_EVREC_OFF_VALUE));
        pending=1u;
    }
    else if(p_store->motion_count !=0u)
    {
        b1_odom_record_t rec=p_store->motions[p_store->motion_head];
        p_store->motion_head=(uint8_t)((
            p_store->motion_head+1u)%B1_MOTION_DEPTH);
        p_store->motion_count--;
        p_out->type=B1_TYPE_ODOM;
        p_out->time_ms=b1_get32(rec.data);
        p_out->payload_len=B1_ODOM_LEN;
        memcpy(p_out->payload,rec.data+B1_ODREC_OFF_LEFT,B1_ODOM_LEN);
        pending=1u;
    }
    else if(p_store->checkpoint_pending!=0u)
    {
        p_store->checkpoint_pending=0u;
        p_out->type=B1_TYPE_CHECKPOINT;
        p_out->time_ms=p_store->checkpoint_ms;
        p_out->payload_len=B1_CHECKPOINT_LEN;
        b1_put32(p_out->payload,p_store->checkpoint_mm);
        pending=1u;
    }
    else if(now_ms-p_store->last_heartbeat_ms>=B1_HEARTBEAT_PERIOD_MS)
    {
        p_store->last_heartbeat_ms=now_ms;
        p_out->type=B1_TYPE_HEARTBEAT;
        p_out->payload_len=B1_HEARTBEAT_LEN;
        b1_put32(p_out->payload,p_store->dropped);
        pending=1u;
    }
    else if(now_ms-p_store->last_snapshot_ms>=B1_SNAPSHOT_PERIOD_MS)
    {
        p_store->last_snapshot_ms=now_ms;
        memset(&x,0,sizeof(x));
        x.source_mask=p_store->seen;
        x.stale_mask=stale(p_store,now_ms);
        x.mission=p_store->mission.mission&B1_MISSION_MASK;
        x.line=p_store->line.line&B1_LINE_MASK;
        x.ir=p_store->line.ir&B1_IR_MASK;
        x.barcode=(p_store->seen&B1_SRC_LINE)?p_store->line.barcode:
            B1_LETTER_NONE;
        x.action=p_store->line.action&B1_ACTION_MASK;
        x.action_valid=p_store->line.action_valid;
        x.hump_mm=(p_store->seen&B1_SRC_TERRAIN) ?
        snap_range10(p_store->terrain.hump_mm) : B1_RANGE10_NONE;
        x.max_hump_mm=(p_store->seen&B1_SRC_TERRAIN) ?
        snap_range10(p_store->terrain.max_hump_mm) : B1_RANGE10_NONE;
        x.obstacle_cm=(p_store->seen&B1_SRC_OBSTACLE) ?
        snap_range10(p_store->obstacle.distance_cm) : B1_RANGE10_NONE;
        x.side=p_store->obstacle.side;
        x.bypass_count=p_store->obstacle.bypass_count;
        x.faults=(uint8_t)(p_store->mission.faults | p_store->obstacle.faults);
        x.speed_left_mm_s=snap_speed12(p_store->motion.left_speed_mm_s);
        x.speed_right_mm_s=snap_speed12(p_store->motion.right_speed_mm_s);
        p_out->type=B1_TYPE_SNAPSHOT;
        p_out->payload_len=(uint8_t)b1_encode_snapshot(&x,p_out->payload);
        pending=p_out->payload_len!=0u;
    }
    if(0U == pending)
    {
        p_store->wire_seq--;
    }
    p_store->unlock(p_store->lock_context);
    return pending ? 1 : 0;
}
int32_t b1_sensor_frame(b1_store_t *p_store,b1_frame_t *p_frame)
{
    if(p_store==NULL || p_frame==NULL ||
        (p_frame->type==B1_TYPE_SENSOR &&
        p_frame->payload_len!=B1_SENSOR_V1 &&
        p_frame->payload_len!=B1_SENSOR_DIAG &&
        p_frame->payload_len!=B1_SENSOR_LIFT &&
        p_frame->payload_len!=B1_SENSOR_DIAG_LIFT) ||
        (p_frame->type==B1_TYPE_BARCODE_DIAG &&
        p_frame->payload_len!=B1_BARCODE_DIAG_V1 &&
        p_frame->payload_len!=B1_BARCODE_DIAG_V2) ||
        (p_frame->type!=B1_TYPE_SENSOR &&
        p_frame->type!=B1_TYPE_BARCODE_DIAG))
    {
        return -1;
    }
    p_store->lock(p_store->lock_context);
    p_frame->session=p_store->session;
    p_frame->seq=++p_store->wire_seq;
    p_store->unlock(p_store->lock_context);
    return 0;
}
int32_t b1_recent_status_challenge(b1_store_t *p_store,uint32_t token,
    uint32_t now_ms)
{
    int32_t found=0;
    if(p_store==NULL || token==0u)
    {
        return 0;
    }
    p_store->lock(p_store->lock_context);
    for(uint8_t i=0u;i<B1_CHALLENGE_SLOTS;i++)
    {
        if(p_store->status_challenges[i]==token &&
            (uint32_t)(now_ms-token)<=B1_CHALLENGE_MS)
        {
            found=1;
            break;
        }
    }
    p_store->unlock(p_store->lock_context);
    return found;
}
int32_t b1_handle_command(b1_store_t *p_store,const b1_frame_t *p_request,
    b1_frame_t *p_ack)
{
    b1_command_t command;
    uint8_t result=B1_BAD_COMMAND;
    uint8_t need_callback=0u;
    b1_coordinator_fn_t callback=NULL;
    void *p_context=NULL;
    if((p_store==NULL)||(p_request==NULL)||(p_ack==NULL)||
       (p_request->type!=B1_TYPE_COMMAND))
    {
        return -1;
    }
    if(b1_decode_command(p_request->payload,p_request->payload_len,
        &command)!=0)
    {
        return -1;
    }
    p_store->lock(p_store->lock_context);
    /* STOP is idempotent and skips the duplicate filter. Network STOP is
     * advisory; the local mission fault path is authoritative. */
    if(p_request->session!=p_store->session)
    {
        result=B1_DUPLICATE;
    }
    else if(command.command!=B1_CMD_STOP && p_store->cmd_seen &&
        !fresh(p_store->last_cmd_transaction,
        command.transaction))
    {
        result=B1_DUPLICATE;
    }
    else if(command.command==B1_CMD_MAPPING)
    {
        if(p_store->mission.running || p_store->committed)
        {
            result=B1_BAD_STATE;
        }
        else {memcpy(p_store->mapping,command.mapping,
            B1_MAP_LETTERS);
            p_store->mapping_valid=1u;
            result=B1_OK;
        }
    }
    else if(command.command==B1_CMD_START)
    {
        if(!p_store->mapping_valid || p_store->mission.running)
        {
            result=B1_BAD_STATE;
        }
        else if(!p_store->mission.ready || p_store->coordinator==NULL)
        {
            result=B1_NOT_READY;
        }
        else {need_callback=1u;
            callback=p_store->coordinator;
            p_context=p_store->coordinator_context;
        }
    }
    else if(command.command==B1_CMD_STOP)
    {
        if(p_store->coordinator==NULL)
        {
            result=B1_NOT_READY;
        }
        else {need_callback=1u;
            callback=p_store->coordinator;
            p_context=p_store->coordinator_context;
        }
    }
    else if(command.command==B1_CMD_SNAPSHOT)
    {
        p_store->last_snapshot_ms=0u;
        result=B1_OK;
    }
    else if(command.command==B1_CMD_RECAL_IMU)
    {
        /* The IMU task owns calibration and its I2C device; only allowed
         * while stopped. */
        if(p_store->mission.running || (p_store->mission.mission!=0u &&
            p_store->mission.mission!=B1_MISSION_STOPPED))
        {
            result=B1_BAD_STATE;
        }
        else if(p_store->coordinator==NULL)
        {
            result=B1_NOT_READY;
        }
        else {need_callback=1u;
            callback=p_store->coordinator;
            p_context=p_store->coordinator_context;
        }
    }
    else if(command.command==B1_CMD_RECAL_BARCODE)
    {
        /* Learning white needs the sensor still over white: stopped only.
         * The coordinator forwards it to the barcode task. */
        if(0U != p_store->mission.running)
        {
            result=B1_BAD_STATE;
        }
        else if(p_store->coordinator==NULL)
        {
            result=B1_NOT_READY;
        }
        else {need_callback=1u;
            callback=p_store->coordinator;
            p_context=p_store->coordinator_context;
        }
    }
    if(result!=B1_DUPLICATE)
    {
        p_store->last_cmd_transaction=command.transaction;
        p_store->last_cmd_kind=command.command;
        p_store->cmd_seen=1u;
    }
    p_store->unlock(p_store->lock_context);
    /* Call the coordinator outside the store mutex. */
    if(need_callback!=0u)
    {
        result=callback(command.command,p_context);
    }
    p_store->lock(p_store->lock_context);
    if(command.command==B1_CMD_START && result==B1_OK)
    {
        p_store->committed=1u;
    }
    memset(p_ack,0,sizeof(*p_ack));
    p_ack->type=B1_TYPE_ACK;
    p_ack->session=p_store->session;
    p_ack->seq=++p_store->wire_seq;
    p_ack->time_ms=p_request->time_ms;
    p_ack->payload_len=B1_ACK_LEN;
    p_ack->payload[0]=command.command;
    b1_put16(p_ack->payload+B1_ACK_OFF_TRANSACTION,command.transaction);
    p_ack->payload[B1_ACK_OFF_RESULT]=result;
    p_store->unlock(p_store->lock_context);
    return (int32_t)result;
}
/*** end of file ***/

