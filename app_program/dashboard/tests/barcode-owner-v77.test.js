const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');const path=require('node:path');
const root=path.resolve(__dirname,'../../..');
const read=p=>fs.readFileSync(path.join(root,p),'utf8');
test('one barcode owner with timer scanner and no old decoder',()=>{
 const ir=read('app_program/modules/line_barcode/task_ir.c');
 const telemetry=read('app_program/modules/telemetry/telemetry_peers.c');
 const cal=read('app_program/modules/barcode_scanning/bc_cal.h');
 assert.match(ir,/bc_setup_sample/);
 assert.doesNotMatch(ir,/line_barcode_sample/);
 assert.match(telemetry,/bc_port_peek/);
 assert.match(telemetry,/bc_port_ack/);
 assert.match(telemetry,/bc_port_stats/);
 assert.match(cal,/#define BC_CAL_AUTO_ACK 0/);
 assert.ok(!fs.existsSync(path.join(root,'app_program/modules/line_barcode/bc_decode.c')));
});
test('connection auto-reconnect does not need a separate Connect button',()=>{
 const ui=read('app_program/dashboard/public/index.html');
 const js=read('app_program/dashboard/public/app.js');
 assert.doesNotMatch(ui,/id="connectionButton"/);
 assert.match(ui,/id="networkDetails"/);
 assert.match(ui,/id="configure"/);
 assert.match(js,/el\('networkDetails'\)\.onclick/);
});
