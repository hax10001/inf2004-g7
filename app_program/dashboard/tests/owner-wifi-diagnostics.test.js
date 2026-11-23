'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const root=path.resolve(__dirname,'../../..');
const read=p=>fs.readFileSync(path.join(root,p),'utf8');
test('MQTT-enabled radio owner attempts a direct join without scan prerequisite',()=>{
 const text=read('lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c');
 assert.match(text,/#if defined\(B1_ENABLE_MQTT\) && TM_WIFI_JOIN[\s\S]*?result = 0;[\s\S]*?#else[\s\S]*?cyw43_wifi_scan/);
 assert.match(text,/Direct first join:[\s\S]*?cyw43_wifi_join/);
 assert.match(text,/CYW43_JOIN_TIMEOUT_US 10000000ULL/);
 assert.match(text,/next_join_us/);
});
test('owner handles binary MQTT target topic while app does not own lwIP calls',()=>{
 const owner=read('lib/libnet/lwip/mqtt_utk.c');
 const radio=read('lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c');
 const app=read('app_program/modules/telemetry/buddy1_rt.c');
 assert.match(owner,/#define B1_MQTT_TOPIC_ROOT "g7\/b1\/robot"/);
 assert.match(owner,/#define B1_TOPIC_TELEM B1_MQTT_TOPIC_ROOT "\/telemetry"/);
 assert.match(radio,/mqtt_utk_poll\(&cyw43_state.netif\[CYW43_ITF_STA\]\)/);
 assert.doesNotMatch(app,/\btcp_write\s*\(|\bmqtt_utk_poll\s*\(/);
 assert.match(app,/\[NET\] wifi=%u mqtt=%u init=%d scan=%u/);
});
