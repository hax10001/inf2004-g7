'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const {RobotState}=require('../server/state');
const {frame,TYPES}=require('../server/protocol');
function diag(completed,rejected,code=0){
 const p=Buffer.alloc(28);p[0]=2;p.writeUInt32LE(rejected,4);p.writeUInt32LE(completed,8);p[15]=code;return p;
}
function send(s,type,payload,seq,session=12){return s.ingest(frame(type,payload,{session,seq,ms:seq*1000}),'LIVE');}
function barcode(letter){return Buffer.from([2,1,1,0,letter.charCodeAt(0)-65,0]);}
test('failed scan count produces a visible alert in both live telemetry and monitor, not a guessed decoded letter',()=>{
 const s=new RobotState();
 send(s,TYPES.BARCODE_DIAG,diag(0,0),1);
 assert.equal(s.view().scanOutcomes.length,0);
 send(s,TYPES.BARCODE_DIAG,diag(0,1,4),2);
 const v=s.view();assert.equal(v.scanOutcomes[0].status,'failed');
 assert.equal(v.scanOutcomes[0].failureStatus,4);
 assert.equal(v.scanOutcomes[0].letter,null);
 assert.ok(v.logs.some(l=>l.category==='scanner'&&l.level==='warn'&&/SCAN FAILED/.test(l.message)));
 assert.ok(v.logs.some(l=>l.category==='barcode'&&l.level==='warn'&&/SCAN FAILED/.test(l.message)));
});
test('decoded A-Z event creates success with the actual letter and clears on new session',()=>{
 const s=new RobotState();send(s,TYPES.EVENT,barcode('M'),1);
 assert.equal(s.view().scanOutcomes[0].letter,'M');
 assert.ok(s.view().logs.some(l=>l.category==='scanner'&&/decoded letter M/.test(l.message)));
 assert.ok(s.view().logs.some(l=>l.category==='barcode'&&/decoded letter M/.test(l.message)));
 send(s,TYPES.BARCODE_DIAG,diag(1,0),2);
 assert.equal(s.view().scanOutcomes.length,1,'completion count cannot duplicate a letter success');
 send(s,TYPES.BARCODE_DIAG,diag(0,0),1,99);
 assert.equal(s.view().scanOutcomes.length,0,'old-session scanner results must not survive reboot');
});
test('completed-count does not invent a successful letter; actual event shows once',()=>{
 const s=new RobotState();send(s,TYPES.BARCODE_DIAG,diag(0,0),1);
 send(s,TYPES.BARCODE_DIAG,diag(1,0),2);
 assert.equal(s.view().barcodeDiag.completed,1);
 assert.equal(s.view().scanOutcomes.length,0, 'counter alone is not a letter');
 send(s,TYPES.EVENT,barcode('Q'),3);
 assert.equal(s.view().scanOutcomes.length,1);
 assert.equal(s.view().scanOutcomes[0].letter,'Q');
 assert.equal(s.view().logs.filter(x=>x.category==='scanner'&&/SCAN SUCCESS/.test(x.message)).length,1);
});
test('odometry sequence is exposed to browser only if measured incremental ODOM packet arrived',()=>{
 const s=new RobotState();send(s,TYPES.BARCODE_DIAG,diag(0,0),1);
 assert.equal(s.view().lastMotionSeq,null);
 const p=Buffer.alloc(6);p.writeInt16LE(6,0);p.writeInt16LE(8,2);p.writeUInt16LE(1,4);
 send(s,TYPES.ODOM,p,2);
 assert.equal(s.view().lastMotionSeq,1);
});
test('dashboard has two result views, independent sensor status fields, and responsive spacing styles',()=>{
 const pub=path.join(__dirname,'../public');
 const html=fs.readFileSync(path.join(pub,'index.html'),'utf8');
 const css=fs.readFileSync(path.join(pub,'style.css'),'utf8');
 const js=fs.readFileSync(path.join(pub,'app.js'),'utf8');
 for(const id of ['telemetryScanSpotlight','telemetryScanHistory','monitorScanSpotlight','overviewLastScan','telemetryConnection'])
  assert.match(html,new RegExp(`id="${id}"`));
 assert.match(css,/#view-telemetry \.telemetryHealth\s*\{\s*display:grid/);
 assert.match(css,/@media\(max-width:720px\)/);
 assert.match(js,/STREAM PAUSED/);
 assert.match(js,/\['commands','mission','network','scanner'\]/);
});
