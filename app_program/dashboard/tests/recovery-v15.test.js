'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const {frame,TYPES,decodeFrame,interpret}=require('../server/protocol');
const {RobotState}=require('../server/state');

test('V15 fault source uses compatible EVENT payload and clears only on ready HOLD',()=>{
  const state=new RobotState();
  const event=Buffer.from([5,21,42,0,1,0]);
  const wire=frame(TYPES.EVENT,event,{session:10,seq:1,ms:500});
  assert.equal(wire.length,20);
  assert.deepEqual(interpret(decodeFrame(wire)),
    {kind:'event',source:5,eventKind:21,eventId:42,value:1});
  state.ingest(wire);
  assert.equal(state.view().controllerFault.reason,'collision detected');
  assert(state.logs.some(x=>x.message==='SAFE STOP: collision detected'));
  const status=Buffer.alloc(14);status[0]=1;status[4]=6;
  state.ingest(frame(TYPES.STATUS,status,{session:10,seq:2,ms:600}));
  assert(state.view().controllerFault);
  status[1]=1;status[6]=1;
  state.ingest(frame(TYPES.STATUS,status,{session:10,seq:3,ms:700}));
  assert.equal(state.view().controllerFault,null);
  assert.equal(state.badFrames,0);
});

test('V15 differentiates decode, stale IR and downhill failures without changing STATUS',()=>{
  for(const [code,reason] of [[2,'stale IR'],[6,'decode timeout'],
      [9,'downhill overspeed'],[12,'scan handoff overflow'],[13,'unmapped barcode']]){
    const state=new RobotState();
    state.ingest(frame(TYPES.EVENT,Buffer.from([5,21,1,0,code,0]),{seq:1}));
    assert.equal(state.view().controllerFault.reason,reason);
  }
  for(const size of [5,7,9,14]){
    assert.equal(interpret(decodeFrame(frame(TYPES.STATUS,Buffer.alloc(size)))).kind,'status');
  }
});
