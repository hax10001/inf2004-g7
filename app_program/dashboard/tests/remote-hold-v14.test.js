'use strict';
const test=require('node:test'),assert=require('node:assert/strict');
const {RobotState}=require('../server/state');
const {frame,TYPES}=require('../server/protocol');
function status(s,seq,{ready=false,running=false,hold=false,mission=6,blockers=2}){
 const p=Buffer.alloc(14);
 p[0]=1;p[1]=ready?1:0;p[2]=running?1:0;
 p[4]=mission;p[5]=blockers;p[6]=hold?1:0;
 s.ingest(frame(TYPES.STATUS,p,{session:42,seq,ms:1000+seq*100}));
}
test('V14 STOPPED is not mislabelled remote hold, then explicit STOP makes ready',()=>{
 const s=new RobotState();
 status(s,1,{mission:6,hold:false,blockers:2});
 assert.equal(s.robotStatus.remoteHold,false);
 assert(s.logs.some(x=>x.message.includes('STOPPED but not in remote hold')));
 status(s,2,{mission:6,hold:true,ready:true,blockers:0});
 assert.equal(s.robotStatus.remoteHold,true);
 assert(s.logs.some(x=>x.message.includes('Ready for START')));
 status(s,3,{mission:2,hold:false,running:true,blockers:2});
 assert(s.logs.some(x=>x.message.includes('RUNNING')));
});
