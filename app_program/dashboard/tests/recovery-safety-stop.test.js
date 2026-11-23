'use strict';
const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {RobotState}=require('../server/state');
const {frame,TYPES}=require('../server/protocol');
const index=fs.readFileSync(path.join(__dirname,'../server/index.js'),'utf8');
const js=fs.readFileSync(path.join(__dirname,'../public/app.js'),'utf8');
test('recoverable navigation safety stop has a distinct mission code and explicit STOP instructions',()=>{
 const s=new RobotState();
 const p=Buffer.alloc(14);p[0]=1;p[1]=0;p[2]=0;p[3]=0;p[4]=5;p[5]=2;p[6]=0;
 s.ingest(frame(TYPES.STATUS,p,{session:30,seq:1,ms:1000}));
 assert.equal(s.robotStatus.mission,5);
 assert.ok(s.logs.some(l=>l.message.includes('NAVIGATION SAFETY STOP')&&l.message.includes('press STOP')));
 assert.match(index,/status\?\.mission===5/);
 assert.match(js,/mission===5/);
});
test('genuine controller fault retains separate blocking message',()=>{
 const s=new RobotState();
 const p=Buffer.alloc(14);p[4]=7;p[5]=2;
 s.ingest(frame(TYPES.STATUS,p,{session:31,seq:1,ms:1000}));
 assert.ok(s.logs.some(l=>l.message.includes('CONTROLLER FAULT')));
 assert.match(index,/status\?\.mission===7/);
});
test('zero scanner rejection error is described as unknown, not no error',()=>{
 const s=new RobotState();
 const f=(rejected,seq)=>{const p=Buffer.alloc(28);p.writeUInt32LE(rejected,4);return frame(TYPES.BARCODE_DIAG,p,{session:30,seq,ms:1000+seq*100});};
 s.ingest(f(0,1));s.ingest(f(1,2));
 assert.ok(s.logs.some(l=>/SCAN FAILED/.test(l.message)&&/Reason unavailable/.test(l.message)));
 assert.ok(!s.logs.some(l=>/SCAN FAILED.*No error/.test(l.message)));
});
