#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include "buddy1.h"
static pthread_mutex_t mutex=PTHREAD_MUTEX_INITIALIZER;
static void lock(void *ctx) { assert(pthread_mutex_lock((pthread_mutex_t*)ctx)==0); }
static void unlock(void *ctx) { assert(pthread_mutex_unlock((pthread_mutex_t*)ctx)==0); }
static uint8_t coord(uint8_t command,void *ctx){ (void)ctx;return command==B1_CMD_START?B1_OK:B1_OK; }
static void protocol(void) {
  uint8_t map[4]={0,1,2,3},bytes[3],restored[4],buffer[B1_MAX_FRAME];
  B1_Frame orig={0},back={0};B1_SnapshotPayload snap={0},read={0};
  assert(b1_pack_mapping(map,bytes)==0);
  assert(bytes[0]==0x20&&bytes[1]==0x88&&bytes[2]==0x01);
  assert(b1_unpack_mapping(bytes,restored)==0);
  assert(memcmp(map,restored,4)==0);
  map[1]=0;assert(b1_pack_mapping(map,bytes)==-1);
  bytes[2]=0x80;assert(b1_unpack_mapping(bytes,restored)==-1);
  orig.type=B1_TYPE_SNAPSHOT;orig.session=123;orig.seq=10;orig.time_ms=36000;
  snap.source_mask=31;snap.stale_mask=2;snap.mission=3;snap.line=2;snap.ir=5;
  snap.barcode=8;snap.action=3;snap.action_valid=1;
  snap.hump_mm=67;snap.max_hump_mm=96;snap.obstacle_cm=1023;
  snap.speed_left_mm_s=-100;snap.speed_right_mm_s=1234;
  orig.payload_len=(uint8_t)b1_encode_snapshot(&snap,orig.payload);
  assert(orig.payload_len==13u);
  assert(b1_decode_snapshot(orig.payload,13,&read)==0);
  assert(read.barcode==8&&read.hump_mm==67&&read.speed_left_mm_s==-100);
  const size_t n=b1_frame_encode(&orig,buffer,sizeof buffer);
  assert(n==27u);assert(b1_frame_decode(buffer,n,&back)==0);
  assert(back.seq==orig.seq&&back.payload_len==13);
  buffer[17]^=0x80u;assert(b1_frame_decode(buffer,n,&back)!=0);
  assert(b1_frame_decode(buffer,n-1,&back)!=0);
}
static void aggregation(void) {
  B1_Store s;B1_Frame f={0},req={0},ack={0};B1_Line line={0};B1_Motion motion={0};B1_Mission mission={0};
  B1_Command cmd={0};
  assert(b1_store_init(&s,lock,unlock,&mutex,77)==0);
  b1_set_coordinator(&s,coord,NULL);
  /* V7.5 barcode diagnostic is deliberately not a decoded event. */
  B1_Frame diag={0};
  diag.type=B1_TYPE_BARCODE_DIAG;
  diag.payload_len=28u;
  assert(b1_sensor_frame(&s,&diag)==0);
  assert(diag.seq!=0 && diag.session==77);
  diag.payload_len=24u;
  assert(b1_sensor_frame(&s,&diag)==-1);
  diag.type=B1_TYPE_SENSOR;
  assert(b1_sensor_frame(&s,&diag)==0);
  motion.seq=1;motion.ms=30;motion.left_delta_mm=12;motion.right_delta_mm=13;
  assert(b1_ingest_motion(&s,&motion)==0);
  assert(b1_ingest_motion(&s,&motion)==-1);
  motion.seq=4;assert(b1_ingest_motion(&s,&motion)==0);assert(s.uncertain);
  line.seq=1;line.ms=35;line.barcode=2;line.event_id=4;line.event_kind=1;
  assert(b1_ingest_line(&s,&line)==0);
  assert(b1_next_frame(&s,40,&f)==1&&f.type==B1_TYPE_EVENT);
  assert(b1_next_frame(&s,40,&f)==1&&f.type==B1_TYPE_ODOM);
  assert(b1_next_frame(&s,40,&f)==1&&f.type==B1_TYPE_ODOM);
  assert(b1_next_frame(&s,300,&f)==1&&f.type==B1_TYPE_SNAPSHOT);
  /* Status frame exposes mapping not loaded before START. */
  assert(b1_next_frame(&s,1050,&f)==1 && f.type==B1_TYPE_STATUS);
  assert(f.payload_len==14u && f.payload[0]==0u && f.payload[5]==0u);
  assert(b1_recent_status_challenge(&s,f.time_ms,1060u)==1);
  assert(b1_recent_status_challenge(&s,f.time_ms+1u,1060u)==0);
  assert(b1_recent_status_challenge(&s,f.time_ms,10000u)==0);
  assert(b1_recent_status_challenge(&s,0u,1060u)==0);
  mission.seq=1;mission.ready=1;mission.ms=250;
  assert(b1_ingest_mission(&s,&mission)==0);
  req.type=B1_TYPE_COMMAND;req.session=77;req.time_ms=500;
  cmd.command=B1_CMD_MAPPING;cmd.transaction=1;
  cmd.mapping[0]=0;cmd.mapping[1]=1;cmd.mapping[2]=2;cmd.mapping[3]=3;
  req.payload_len=(uint8_t)b1_encode_command(&cmd,req.payload,sizeof req.payload);
  assert(b1_handle_command(&s,&req,&ack)==B1_OK);
  assert(s.mapping_valid==1u);
  assert(b1_next_frame(&s,2100u,&f)==1 && f.type==B1_TYPE_STATUS);
  assert(f.payload_len==14u && b1_get16(f.payload+7)==1u);
  assert(b1_recent_status_challenge(&s,1050u,2100u)==1);
  assert(b1_handle_command(&s,&req,&ack)==B1_DUPLICATE);
  cmd.command=B1_CMD_START;cmd.transaction=2;
  req.payload_len=(uint8_t)b1_encode_command(&cmd,req.payload,sizeof req.payload);
  assert(b1_handle_command(&s,&req,&ack)==B1_OK);
  assert(s.committed==1u);
  cmd.command=B1_CMD_MAPPING;cmd.transaction=3;
  req.payload_len=(uint8_t)b1_encode_command(&cmd,req.payload,sizeof req.payload);
  assert(b1_handle_command(&s,&req,&ack)==B1_BAD_STATE);
  /* A request to stop alone must not unlock remapping until the physical
   * controller has published an actually applied remote hold. */
  mission.seq=2;mission.remote_hold=0u;mission.running=1u;
  assert(b1_ingest_mission(&s,&mission)==0);
  assert(s.committed==1u);
  mission.seq=3;mission.running=0u;mission.remote_hold=1u;
  assert(b1_ingest_mission(&s,&mission)==0);
  assert(s.committed==0u);
  cmd.command=B1_CMD_MAPPING;cmd.transaction=4;
  req.payload_len=(uint8_t)b1_encode_command(&cmd,req.payload,sizeof req.payload);
  assert(b1_handle_command(&s,&req,&ack)==B1_OK);
}
static void start_requires_mapping(void){
 B1_Store s;B1_Mission mission={0};B1_Command cmd={0};B1_Frame req={0},ack={0};
 assert(b1_store_init(&s,lock,unlock,&mutex,112)==0);
 b1_set_coordinator(&s,coord,NULL);mission.seq=1;mission.ready=1;
 assert(b1_ingest_mission(&s,&mission)==0);
 cmd.command=B1_CMD_START;cmd.transaction=1;
 req.type=B1_TYPE_COMMAND;req.session=112;
 req.payload_len=(uint8_t)b1_encode_command(&cmd,req.payload,sizeof(req.payload));
 assert(b1_handle_command(&s,&req,&ack)==B1_BAD_STATE);
 assert(ack.payload[3]==B1_BAD_STATE);
}
static void recalibrate_guard(void){
 B1_Store s;B1_Mission mission={0};B1_Command cmd={0};B1_Frame req={0},ack={0};
 assert(b1_store_init(&s,lock,unlock,&mutex,99)==0);
 b1_set_coordinator(&s,coord,NULL);mission.seq=1;mission.mission=2;mission.running=1;
 assert(b1_ingest_mission(&s,&mission)==0);
 cmd.command=B1_CMD_RECAL_IMU;cmd.transaction=1;
 req.type=B1_TYPE_COMMAND;req.session=99;
 req.payload_len=(uint8_t)b1_encode_command(&cmd,req.payload,sizeof(req.payload));
 assert(req.payload_len==3u);
 assert(b1_handle_command(&s,&req,&ack)==B1_BAD_STATE);
 mission.seq=2;mission.mission=6;mission.running=0;
 assert(b1_ingest_mission(&s,&mission)==0);
 cmd.transaction=2;req.payload_len=(uint8_t)b1_encode_command(&cmd,req.payload,sizeof(req.payload));
 assert(b1_handle_command(&s,&req,&ack)==B1_OK);
}
int main(void){protocol();aggregation();start_requires_mapping();recalibrate_guard();puts("C codec + aggregation + IMU guard tests PASS");return 0;}
