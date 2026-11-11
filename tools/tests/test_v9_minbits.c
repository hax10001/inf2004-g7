/* Native regression: compact C representation must preserve wire semantics. */
#include "buddy1.h"
#include "telemetry_peers.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
int main(void) {
    B1_SnapshotPayload input,back;
    uint8_t binary[13],framebuf[B1_MAX_FRAME],packed[3];
    B1_Frame frame={0},again;
    uint8_t map[4]={0,4,12,25},remap[4]={0};
    unsigned i;
    memset(&input,0,sizeof(input));
    input.source_mask=23;input.stale_mask=2;input.mission=7;input.line=5;
    input.ir=7;input.barcode=25;input.action=3;input.action_valid=1;
    input.hump_mm=1023;input.max_hump_mm=450;input.obstacle_cm=850;
    input.side=2;input.bypass_count=207;input.faults=0xa5;
    input.speed_left_mm_s=-2047;input.speed_right_mm_s=2047;
    assert(b1_encode_snapshot(&input,binary)==13);
    assert(b1_decode_snapshot(binary,sizeof(binary),&back)==0);
    assert(back.source_mask==23&&back.stale_mask==2&&back.mission==7);
    assert(back.line==5&&back.ir==7&&back.barcode==25);
    assert(back.action==3&&back.action_valid==1&&back.hump_mm==1023);
    assert(back.max_hump_mm==450&&back.obstacle_cm==850);
    assert(back.side==2&&back.bypass_count==207&&back.faults==0xa5);
    assert(back.speed_left_mm_s==-2047&&back.speed_right_mm_s==2047);
    assert((back.faults & 0x01)==1 && (back.faults & 0x80)==0x80);
    assert(b1_pack_mapping(map,packed)==0);
    assert(b1_unpack_mapping(packed,remap)==0);
    assert(memcmp(map,remap,sizeof(map))==0);
    frame.type=B1_TYPE_SNAPSHOT;frame.payload_len=13;frame.session=313;
    frame.seq=77;frame.time_ms=123456;memcpy(frame.payload,binary,13);
    i=(unsigned)b1_frame_encode(&frame,framebuf,sizeof(framebuf));
    assert(i==27u);
    assert(b1_frame_decode(framebuf,i,&again)==0);
    assert(again.type==frame.type&&again.session==313&&again.seq==77);
    assert(memcmp(again.payload,binary,13)==0);
    printf("snapshot_hex=");
    for(i=0;i<13u;i++)printf("%02x",binary[i]);
    printf("\nframe_hex=");
    for(i=0;i<27u;i++)printf("%02x",framebuf[i]);
    printf("\nstore_bytes=%zu\npeers_bytes=%zu\n",
        sizeof(B1_Store),sizeof(G7_TelemetryPeers));
    puts("PASS v9 bit-field/CRC/mapping/fault/native golden vector");
    return 0;
}
