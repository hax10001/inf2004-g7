'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const {RobotState,readinessReasons}=require('../server/state');
const {frame,TYPES,interpret,decodeFrame}=require('../server/protocol');
const session=817;
let seq=0;
function send(s,type,payload,time){
 const raw=frame(type,payload,{session,seq:++seq,ms:time});
 return s.ingest(raw,'LIVE');
}
test('V7.8 status reports exact startup blockers and backwards compatibility',()=>{
 const now=Buffer.from([1,0,0,0x1b,6,8|32,1]);
 const extended=interpret(decodeFrame(frame(TYPES.STATUS,now,{session,seq:1,ms:1000})));
 assert.equal(extended.readinessBlockers,40);
 assert.deepEqual(readinessReasons(extended.readinessBlockers),['IMU not calibrated','IR data stale']);
 assert.equal(extended.remoteHold,true);
 const withTx=Buffer.from([1,0,0,0x1b,6,8|32,1,0x2c,0x01]);
 const v9=interpret(decodeFrame(frame(TYPES.STATUS,withTx,{session,seq:3,ms:1050})));
 assert.equal(v9.lastCommandTransaction,300);
 const older=interpret(decodeFrame(frame(TYPES.STATUS,now.subarray(0,5),{session,seq:2,ms:1020})));
 assert.equal(older.readinessBlockers,null);
});
test('V7.8 MQTT command clock cannot be rolled backward by barcode or odometry history',()=>{
 seq=0;const s=new RobotState();
 send(s,TYPES.STATUS,Buffer.from([1,1,0,0x1b,6,0,1]),10000);
 const initial=s.lastRobotMs;
 assert.equal(initial,10000);
 send(s,TYPES.EVENT,Buffer.from([2,1,1,0,0,0]),150);
 assert.equal(s.lastRobotMs,initial);
 send(s,TYPES.ODOM,Buffer.from([1,0,2,0,1,0]),200);
 assert.equal(s.lastRobotMs,initial);
 send(s,TYPES.SNAPSHOT,Buffer.alloc(13),9500);
 assert.equal(s.lastRobotMs,initial);
 send(s,TYPES.SENSOR,Buffer.alloc(24),10100);
 assert.equal(s.lastRobotMs,10100);
 send(s,TYPES.ACK,Buffer.from([2,12,0,0]),8000);
 assert.equal(s.lastRobotMs,10100);
 assert.ok(s.lastRobotClockAt!=null);
});
