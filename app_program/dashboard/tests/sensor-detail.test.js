const test=require('node:test');
const assert=require('node:assert/strict');
const {frame,TYPES,interpret,decodeFrame}=require('../server/protocol');
const {RobotState}=require('../server/state');
function packet(){
 const p=Buffer.alloc(24);p[0]=1|2|8|16;p[1]=5;
 p.writeInt16LE(-125,2);p.writeInt16LE(250,4);
 p.writeUInt16LE(12345,6);p.writeUInt16LE(65535,8);
 p[10]=1;p[11]=1;p.writeInt32LE(-456,12);
 p.writeInt32LE(789,16);p[20]=255;return p;
}
test('sensor packet reports valid IMU, three IR and encoders',()=>{
 const r=interpret(decodeFrame(frame(TYPES.SENSOR,packet(),{session:42,seq:1,ms:200})));
 assert.equal(r.kind,'sensor');assert.equal(r.ir,5);
 assert.equal(r.rollDeg,-1.25);assert.equal(r.pitchDeg,2.5);
 assert.equal(r.headingDeg,123.45);
 assert.equal(r.ultrasonicCm,null);assert.equal(r.obstaclePhase,null);
 assert.equal(r.leftEncoder,-456);assert.equal(r.rightEncoder,789);
});
test('serial sensor mirror populates dashboard without claiming MQTT link',()=>{
 const s=new RobotState();
 s.ingestSerial('[SENS] IR L=1 R=0 B=1 roll_cd=-125 pitch_cd=250 heading_cd=12345 hump=8 sonar=65535 phase=255 encL=-456 encR=789 valid=27');
 assert.equal(s.view().serialSensor.ir,5);
 assert.equal(s.view().serialSensor.rollDeg,-1.25);
 assert.equal(s.view().serialSensor.headingDeg,123.45);
 assert.equal(s.view().serialSensor.ultrasonicCm,null);
 assert.equal(s.view().mode,'LIVE');
 assert.equal(s.view().frames,0);
});
test('MQTT sensor frame retains live origin and session',()=>{
 const s=new RobotState();
 s.ingest(frame(TYPES.SENSOR,packet(),{session:42,seq:1,ms:200}));
 assert.equal(s.view().session,42);
 assert.equal(s.view().sensor.source,'LIVE');
});

test('priority ACK arriving ahead of queued telemetry does not drop them',()=>{
 const s=new RobotState();
 const enc=(type,p,seq)=>frame(type,p,{session:5,seq,ms:100+seq});
 const hb=Buffer.alloc(4);
 const ack=Buffer.from([3,7,0,0]);
 assert.equal(s.ingest(enc(TYPES.HEARTBEAT,hb,1)).error,undefined);
 assert.equal(s.ingest(enc(TYPES.ACK,ack,4)).error,undefined);
 assert.equal(s.ingest(enc(TYPES.HEARTBEAT,hb,2)).error,undefined);
 assert.equal(s.ingest(enc(TYPES.HEARTBEAT,hb,3)).error,undefined);
 assert.equal(s.ingest(enc(TYPES.HEARTBEAT,hb,5)).error,undefined);
 assert.equal(s.view().odomGaps,0);
 assert.equal(s.view().badFrames,0);
});
