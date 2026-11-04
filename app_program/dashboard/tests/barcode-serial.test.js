const test=require('node:test');
const assert=require('node:assert/strict');
const {RobotState}=require('../server/state');
const {frame,TYPES}=require('../server/protocol');
test('serial barcode letters are extracted but never claimed as MQTT',()=>{
 const state=new RobotState();
 state.ingestSerial('[BARCODE] Decoded: M seq=3 (Code 39)');
 assert.equal(state.view().serialBarcode.letter,'M');
 assert.equal(state.view().mode,'LIVE');
 assert.match(state.view().logs[0].message,/Barcode decoded: M/);
});
test('real MQTT barcode event logs decoded letter',()=>{
 const state=new RobotState();
 const p=Buffer.from([2,1,9,0,12,0]);
 const v=state.ingest(frame(TYPES.EVENT,p,{session:7,seq:1,ms:900}));
 assert.equal(v.msg.value,12);
 assert.match(state.view().logs[0].message,/letter=M/);
});
