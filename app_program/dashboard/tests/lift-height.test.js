const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const {frame,TYPES,interpret,decodeFrame}=require('../server/protocol');
const {RobotState}=require('../server/state');
function packet(length,liftWord){
 const p=Buffer.alloc(length);p[0]=1|2|8|16;p[1]=5;p[10]=0;p[11]=1;p[20]=255;
 if(liftWord!==undefined)p.writeUInt16LE(liftWord,length-2);
 return p;
}
const decode=p=>interpret(decodeFrame(frame(TYPES.SENSOR,p,{session:7,seq:1,ms:100})));
test('sensor frame carries the lifted height in ten bits plus a lifted flag',()=>{
 const onFloor=decode(packet(26,0));
 assert.equal(onFloor.liftMm,0);assert.equal(onFloor.lifted,false);
 const lifted=decode(packet(26,85|0x400));
 assert.equal(lifted.liftMm,85);assert.equal(lifted.lifted,true);
 const max=decode(packet(26,1022|0x400));
 assert.equal(max.liftMm,1022);
});
test('lift word of 1023 is unavailable and older 24/32-byte frames still decode',()=>{
 const unknown=decode(packet(26,1023));
 assert.equal(unknown.liftMm,null);assert.equal(unknown.lifted,null);
 const legacy=decode(packet(24));
 assert.equal(legacy.liftMm,null);assert.equal(legacy.lifted,null);
 assert.equal(decode(packet(32)).liftMm,null);
});
test('lift word follows obstacle diagnostics and reserved bits are rejected',()=>{
 const p=packet(34,40|0x400);p[0]|=4;p.writeInt16LE(120,24);p[30]=1;
 const r=decode(p);
 assert.equal(r.liftMm,40);assert.equal(r.lifted,true);
 assert.equal(r.obstacleDiag.sideRangeMm,120);
 assert.throws(()=>decode(packet(26,0x800)),/lift word/);
 assert.throws(()=>decode(packet(25)),/sensor payload/);
});
test('dashboard state logs lift and landing once and names fault 14',()=>{
 const s=new RobotState();
 const send=(seq,word)=>s.ingest(frame(TYPES.SENSOR,packet(26,word),{session:9,seq,ms:100*seq}));
 send(1,0);send(2,60|0x400);send(3,70|0x400);send(4,0);
 assert.equal(s.logs.filter(l=>l.message.includes('ROBOT LIFTED')).length,1);
 assert.equal(s.logs.filter(l=>l.message.includes('back on the floor')).length,1);
 assert.equal(s.view().sensor.liftMm,0);
 const e=Buffer.alloc(6);e[0]=5;e[1]=21;e.writeUInt16LE(1,2);e.writeUInt16LE(14,4);
 s.ingest(frame(TYPES.EVENT,e,{session:9,seq:5,ms:600}));
 assert.equal(s.view().controllerFault.code,14);
 assert.match(s.view().controllerFault.reason,/lifted/);
});
function status(mission,running){
 const p=Buffer.alloc(14);p[0]=1;p[1]=0;p[2]=running?1:0;p[3]=31;p[4]=mission;
 return p;
}
test('lift pause is reported as mission 4 and clears once running again',()=>{
 const s=new RobotState();
 const e=Buffer.alloc(6);e[0]=5;e[1]=21;e.writeUInt16LE(1,2);e.writeUInt16LE(14,4);
 s.ingest(frame(TYPES.EVENT,e,{session:3,seq:1,ms:100}));
 assert.ok(s.logs.some(l=>l.message.startsWith('LIFTED: motors stopped')));
 assert.ok(!s.logs.some(l=>l.message.startsWith('SAFE STOP')));
 s.ingest(frame(TYPES.STATUS,status(4,false),{session:3,seq:2,ms:200}));
 assert.ok(s.logs.some(l=>l.message.includes('resumes line following')));
 assert.equal(s.view().controllerFault.code,14);
 s.ingest(frame(TYPES.STATUS,status(2,true),{session:3,seq:3,ms:300}));
 assert.equal(s.view().controllerFault,null);
});
test('page names mission 4 as the lift pause',()=>{
 const js=fs.readFileSync(path.join(__dirname,'..','public','app.js'),'utf8');
 assert.ok(js.includes("'LIFTED (PAUSED)'"));
 assert.ok(!js.includes('LEGACY/UNUSED'));
});
test('serial sensor line reports the lifted height when present',()=>{
 const s=new RobotState();
 const base='[SENS] IR L=1 R=1 B=1 roll_cd=0 pitch_cd=0 heading_cd=100 hump=0 sonar=65535 phase=255 encL=1 encR=2 valid=27';
 s.ingestSerial(base+' lift=42');
 assert.equal(s.view().serialSensor.liftMm,42);
 s.ingestSerial(base);
 assert.equal(s.view().serialSensor.liftMm,null);
});
test('dashboard page shows the lifted height in every sensor view',()=>{
 const pub=path.join(__dirname,'..','public');
 const html=fs.readFileSync(path.join(pub,'index.html'),'utf8');
 const js=fs.readFileSync(path.join(pub,'app.js'),'utf8');
 for(const id of ['liftHeight','liftNote','sensorLift','sensorLifted','threeLift']){
  assert.ok(html.includes(`id="${id}"`),id+' missing from page');
  assert.ok(js.includes(`el('${id}')`),id+' never updated');
 }
});
test('ignored barcode scans are explained as non-stopping',()=>{
 const s=new RobotState();
 for(const [seq,value] of [[1,2],[2,3]]){
  const e=Buffer.alloc(6);e[0]=5;e[1]=19;e.writeUInt16LE(seq,2);e.writeUInt16LE(value,4);
  s.ingest(frame(TYPES.EVENT,e,{session:4,seq,ms:100*seq}));
 }
 assert.ok(s.logs.some(l=>l.message.includes('not in current four-letter mapping')&&l.message.includes('robot continues')));
 assert.ok(s.logs.some(l=>l.message.includes('scan result lost')&&l.message.includes('robot continues')));
 assert.equal(s.view().controllerFault,null);
});
