/** @file buddy1.c
 * @brief Mutex-protected telemetry store and bounded frame aggregation.
 */
#include "buddy1.h"
#include <string.h>

static int fresh(uint16_t prev,uint16_t curr) {
    return curr != prev && (uint16_t)(curr - prev) < 0x8000u;
}
static void put_event(B1_Store *s,uint8_t src,uint8_t kind,uint16_t id,
    uint16_t val,uint32_t ms) {
    uint8_t idx;
    if(kind == 0u) return;
    if(s->event_count == B1_EVENT_DEPTH) {
        /* Oldest event overwritten, never pretend event history is complete. */
        s->event_head=(uint8_t)((s->event_head+1u)%B1_EVENT_DEPTH);
        s->event_count--; s->dropped++; s->uncertain=1u;
    }
    idx=(uint8_t)((s->event_head+s->event_count)%B1_EVENT_DEPTH);
    s->events[idx].source=src; s->events[idx].kind=kind;
    s->events[idx].id=id; s->events[idx].value=val; s->events[idx].ms=ms;
    s->event_count++;
}
int b1_store_init(B1_Store *s,B1_Lock lock,B1_Lock unlock,void *ctx,
    uint16_t session) {
    if(s==NULL || lock==NULL || unlock==NULL) return -1;
    memset(s,0,sizeof(*s)); s->lock=lock; s->unlock=unlock;
    s->lock_context=ctx; s->session=session==0u?1u:session;
    s->line.barcode=31u;
    return 0;
}
void b1_set_coordinator(B1_Store *s,B1_Coordinator fn,void *ctx) {
    if(s==NULL)return;
    s->lock(s->lock_context); s->coordinator=fn; s->coordinator_context=ctx;
    s->unlock(s->lock_context);
}
int b1_ingest_motion(B1_Store *s,const B1_Motion *v) {
    uint8_t idx;
    if(s==NULL || v==NULL) return -1;
    s->lock(s->lock_context);
    if((s->seen&B1_SRC_MOTION)!=0u && !fresh(s->motion.seq,v->seq)) {
        s->dup++;s->unlock(s->lock_context);return -1;
    }
    if((s->seen&B1_SRC_MOTION)!=0u && (uint16_t)(v->seq-s->motion.seq)>1u) s
        ->uncertain=1u;
    s->motion=*v;s->seen|=B1_SRC_MOTION;
    if(s->motion_count==B1_MOTION_DEPTH) {
        s->motion_head=(uint8_t)((s->motion_head+1u)%B1_MOTION_DEPTH);
        s->motion_count--;s->dropped++;s->uncertain=1u;
    }
    idx=(uint8_t)((s->motion_head+s->motion_count)%B1_MOTION_DEPTH);
    s->motions[idx]=*v; s->motion_count++;
    if(v->checkpoint){s->checkpoint_pending=1u;
        s->checkpoint_mm=v->cumulative_mm;s->checkpoint_ms=v->ms;}
    s->unlock(s->lock_context);return 0;
}
#define B1_SOURCE_INGEST(NAME,TYPE,FIELD,MASK,SOURCE,VAL) \
int b1_ingest_##NAME(B1_Store *s,const TYPE *v) { \
    if(s==NULL||v==NULL)return -1; \
    s->lock(s->lock_context); \
    if((s->seen & MASK)!=0u && !fresh(s->FIELD.seq,v->seq)) { \
        s->dup++;s->unlock(s->lock_context);return -1; \
    } \
    if((s->seen & MASK)!=0u && (uint16_t)(v->seq-s->FIELD.seq)>1u) s \
        ->uncertain=1u; \
    if(v->event_kind!=0u && ((s->seen&MASK)==0u || v->event_id!=s \
        ->FIELD.event_id)) \
        put_event(s,SOURCE,v->event_kind,v->event_id,(uint16_t)(VAL),v->ms); \
    s->FIELD=*v;s->seen|=MASK; \
    s->unlock(s->lock_context);return 0; \
}
B1_SOURCE_INGEST(line,B1_Line,line,B1_SRC_LINE,2u,v->barcode)
B1_SOURCE_INGEST(terrain,B1_Terrain,terrain,B1_SRC_TERRAIN,3u,v->hump_mm)
/* Unlike the other sources, a confirmed controller hold ends a previous
 * START commitment. Merely ACKing STOP is NOT proof motors have stopped. */
int b1_ingest_mission(B1_Store *s,const B1_Mission *v) {
    if(s==NULL||v==NULL)return -1;
    s->lock(s->lock_context);
    if((s->seen&B1_SRC_MISSION)!=0u &&
       !fresh(s->mission.seq,v->seq)){
        s->dup++;s->unlock(s->lock_context);return -1;
    }
    if((s->seen&B1_SRC_MISSION)!=0u &&
       (uint16_t)(v->seq-s->mission.seq)>1u)s->uncertain=1u;
    if(v->event_kind!=0u &&
       ((s->seen&B1_SRC_MISSION)==0u ||
        v->event_id!=s->mission.event_id))
        put_event(s,5u,v->event_kind,v->event_id,
                  v->event_value,v->ms);
    s->mission=*v;s->seen|=B1_SRC_MISSION;
    if(v->remote_hold!=0u)s->committed=0u;
    s->unlock(s->lock_context);return 0;
}

static uint8_t stale(const B1_Store *s,uint32_t now) {
    uint8_t mask=0u;
    if((s->seen&B1_SRC_MOTION)!=0u && now-s->motion.ms>1500u)
        mask|=B1_SRC_MOTION;
    if((s->seen&B1_SRC_LINE)!=0u && now-s->line.ms>1500u)mask|=B1_SRC_LINE;
    if((s->seen&B1_SRC_TERRAIN)!=0u && now-s->terrain.ms>1500u)
        mask|=B1_SRC_TERRAIN;
    if((s->seen&B1_SRC_MISSION)!=0u && now-s->mission.ms>1500u)
        mask|=B1_SRC_MISSION;
    return mask;
}
int b1_next_frame(B1_Store *s,uint32_t now_ms,B1_Frame *out) {
    B1_SnapshotPayload x;
    uint8_t pending=0u;
    if(s==NULL||out==NULL)return 0;
    s->lock(s->lock_context);
    memset(out,0,sizeof(*out));
    out->session=s->session;out->seq=++s->wire_seq;out->time_ms=now_ms;
    out->flags=s->uncertain ? 1u : 0u;
    if(now_ms-s->last_status_ms>=1000u) {
        /* Actual robot readiness and mapping, not a dashboard guess. */
        s->last_status_ms=now_ms;
        s->status_challenges[s->status_challenge_head]=now_ms;
        s->status_challenge_head=(uint8_t)
            ((s->status_challenge_head+1u)%8u);
        out->type=B1_TYPE_STATUS;
        out->payload_len=14u;
        out->payload[0]=s->mapping_valid;
        out->payload[1]=s->mission.ready;
        out->payload[2]=s->mission.running;
        out->payload[3]=s->seen;
        out->payload[4]=s->mission.mission;
        out->payload[5]=s->mission.ready_blockers;
        out->payload[6]=s->mission.remote_hold;
        b1_put16(out->payload+7,s->last_cmd_transaction);
        out->payload[9]=s->mission.barcode_hold;
        b1_put16(out->payload+10,s->mission.pwm_left);
        b1_put16(out->payload+12,s->mission.pwm_right);
        pending=1u;
    } else if(s->event_count!=0u) {
        B1_Event e=s->events[s->event_head];
        s->event_head=(uint8_t)((s->event_head+1u)%B1_EVENT_DEPTH);
            s->event_count--;
        out->type=B1_TYPE_EVENT;out->time_ms=e.ms;out->payload_len=6u;
        out->payload[0]=e.source;out->payload[1]=e.kind;
        b1_put16(out->payload+2,e.id);b1_put16(out->payload+4,e.value);
            pending=1u;
    } else if(s->motion_count !=0u) {
        B1_Motion m=s->motions[s->motion_head];
        s->motion_head=(uint8_t)((s->motion_head+1u)%B1_MOTION_DEPTH);
            s->motion_count--;
        out->type=B1_TYPE_ODOM;out->time_ms=m.ms;out->payload_len=6u;
        b1_put16(out->payload,(uint16_t)m.left_delta_mm);
        b1_put16(out->payload+2,(uint16_t)m.right_delta_mm);
        b1_put16(out->payload+4,m.seq);pending=1u;
        /* Checkpoint is emitted in a subsequent host snapshot/replay sample;
         * producer must explicitly send B1_TYPE_CHECKPOINT as required. */
    } else if(s->checkpoint_pending!=0u) {
        s->checkpoint_pending=0u;out->type=B1_TYPE_CHECKPOINT;
        out->time_ms=s->checkpoint_ms;out->payload_len=4u;
        b1_put32(out->payload,s->checkpoint_mm);pending=1u;
    } else if(now_ms-s->last_heartbeat_ms>=2000u) {
        s->last_heartbeat_ms=now_ms;out->type=B1_TYPE_HEARTBEAT;
        out->payload_len=4u;b1_put32(out->payload,s->dropped);pending=1u;
    } else if(now_ms-s->last_snapshot_ms>=250u) {
        s->last_snapshot_ms=now_ms;
        memset(&x,0,sizeof(x));x.source_mask=s->seen;
            x.stale_mask=stale(s,now_ms);
        x.mission=s->mission.mission&7u;x.line=s->line.line&7u;
            x.ir=s->line.ir&7u;
        x.barcode=(s->seen&B1_SRC_LINE)?s->line.barcode:31u;
        x.action=s->line.action&3u;x.action_valid=s->line.action_valid;
        x.hump_mm=(s->seen&B1_SRC_TERRAIN)?s->terrain.hump_mm:1023u;
        x.max_hump_mm=(s->seen&B1_SRC_TERRAIN)?s->terrain.max_hump_mm:1023u;
        /* V1 wire fields retained for compatibility, but no obstacle owner. */
        x.obstacle_cm=1023u;
        x.side=0u;x.bypass_count=0u;
        x.faults=s->mission.faults;
        x.speed_left_mm_s=s->motion.left_speed_mm_s;
        x.speed_right_mm_s=s->motion.right_speed_mm_s;
        out->type=B1_TYPE_SNAPSHOT;
        out->payload_len=(uint8_t)b1_encode_snapshot(&x,out->payload);
        pending=out->payload_len!=0u;
    }
    if(!pending)s->wire_seq--;
    s->unlock(s->lock_context);
    return pending ? 1 : 0;
}
int b1_sensor_frame(B1_Store *s,B1_Frame *frame) {
    if(s==NULL || frame==NULL ||
       (frame->type==B1_TYPE_SENSOR && frame->payload_len!=24u) ||
       (frame->type==B1_TYPE_BARCODE_DIAG && frame->payload_len!=28u) ||
       (frame->type!=B1_TYPE_SENSOR &&
        frame->type!=B1_TYPE_BARCODE_DIAG))return -1;
    s->lock(s->lock_context);
    frame->session=s->session;
    frame->seq=++s->wire_seq;
    s->unlock(s->lock_context);
    return 0;
}
int b1_recent_status_challenge(B1_Store *s,uint32_t token,
    uint32_t now_ms) {
    int found=0;
    if(s==NULL || token==0u)return 0;
    s->lock(s->lock_context);
    for(uint8_t i=0u;i<8u;i++){
        if(s->status_challenges[i]==token &&
           (uint32_t)(now_ms-token)<=8000u){found=1;break;}
    }
    s->unlock(s->lock_context);
    return found;
}
int b1_handle_command(B1_Store *s,const B1_Frame *request,B1_Frame *ack) {
    B1_Command command;
    uint8_t result=B1_BAD_COMMAND,need_callback=0u;
    B1_Coordinator callback=NULL;void *context=NULL;
    if(s==NULL||request==NULL||ack==NULL||request->type!=B1_TYPE_COMMAND)
        return -1;
    if(b1_decode_command(request->payload,request->payload_len,
        &command)!=0)return -1;
    s->lock(s->lock_context);
    /* STOP remains idempotent and bypasses the duplicate/replay filter.
       Network STOP is advisory;
           the local mission fault path is authoritative. */
    if(request->session!=s->session)result=B1_DUPLICATE;
    else if(command.command!=B1_CMD_STOP && s->cmd_seen &&
            !fresh(s->last_cmd_transaction,
            command.transaction))result=B1_DUPLICATE;
    else if(command.command==B1_CMD_MAPPING) {
        if(s->mission.running || s->committed)result=B1_BAD_STATE;
        else {memcpy(s->mapping,command.mapping,4u);s->mapping_valid=1u;
            result=B1_OK;}
    } else if(command.command==B1_CMD_START) {
        if(!s->mapping_valid || s->mission.running)result=B1_BAD_STATE;
        else if(!s->mission.ready || s->coordinator==NULL)result=B1_NOT_READY;
        else {need_callback=1u;callback=s->coordinator;
            context=s->coordinator_context;}
    } else if(command.command==B1_CMD_STOP) {
        if(s->coordinator==NULL)result=B1_NOT_READY;
        else {need_callback=1u;callback=s->coordinator;
            context=s->coordinator_context;}
    } else if(command.command==B1_CMD_SNAPSHOT) {
        s->last_snapshot_ms=0u;result=B1_OK;
    } else if(command.command==B1_CMD_RECAL_IMU) {
        /* The IMU task owns calibration and its I2C device.  Never call
         * terrain_calibrate() here with imaginary axes or while moving. */
        if(s->mission.running || (s->mission.mission!=0u &&
                                   s->mission.mission!=6u)) result=B1_BAD_STATE;
        else if(s->coordinator==NULL)result=B1_NOT_READY;
        else {need_callback=1u;callback=s->coordinator;
            context=s->coordinator_context;}
    }
    if(result!=B1_DUPLICATE){
        s->last_cmd_transaction=command.transaction;
            s->last_cmd_kind=command.command;s->cmd_seen=1u;
    }
    s->unlock(s->lock_context);
    /* IMPORTANT: no user/coordinator callback under our snapshot mutex. */
    if(need_callback!=0u)result=callback(command.command,context);
    s->lock(s->lock_context);
    if(command.command==B1_CMD_START && result==B1_OK) s->committed=1u;
    memset(ack,0,sizeof(*ack));ack->type=B1_TYPE_ACK;ack->session=s->session;
    ack->seq=++s->wire_seq;ack->time_ms=request->time_ms;ack->payload_len=4u;
    ack->payload[0]=command.command;
        b1_put16(ack->payload+1,command.transaction);
    ack->payload[3]=result;
    s->unlock(s->lock_context);return (int)result;
}
/*** end of file ***/

