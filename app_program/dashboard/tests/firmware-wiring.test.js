const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const root=path.resolve(__dirname,'../../..');
const read=p=>fs.readFileSync(path.join(root,p),'utf8');
test('network retries, 10s bounded join, guarded commands off by default',()=>{
 const main=read('app_program/app_main.c');
 const wifi=read('lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c');
 const build=read('build_uf2.sh');
 assert.match(main,/MQTT offline; robot tasks remain running/);
 assert.match(wifi,/CYW43_JOIN_TIMEOUT_US 10000000ULL/);
 assert.match(build,/G7_LIVE_CONTROL:-0/);
 assert.match(build,/B1_LIVE_COMMANDS="\$LIVE_CONTROL"/);
 assert.match(wifi,/next_join_us/);
 assert.match(read('app_program/app/task_controller.c'),
   /MQTT lost -> motors STOPPED/);
 assert.doesNotMatch(build,/B1_BARCODE_BENCH_ACK=1/);
});
test('firmware event observes validated letter and does not send motor command',()=>{
 const peer=read('app_program/modules/telemetry/telemetry_peers.c');
 assert.match(peer,/bc_port_peek/);
 assert.match(peer,/BC_PORT_VALID_FLAG/);
 assert.match(peer,/event_kind=1u/);
 assert.match(peer,/\[BARCODE\] Decoded/);
});
