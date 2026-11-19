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

test('obstacle owner publishes measured ultrasonic range and bypass phase',()=>{
 const state=new RobotState(),p=packet();
 p[0]|=4;p.writeUInt16LE(35,8);p[20]=7;
 state.ingest(frame(TYPES.SENSOR,p,{session:43,seq:1,ms:200}));
 assert.equal(state.view().sensor.ultrasonicCm,35);
 assert.equal(state.view().sensor.obstaclePhase,7);
 assert.ok(state.logs.some(l=>l.message.includes('sonar=35 cm obstaclePhase=7')));
 p.writeUInt16LE(65535,8);
 state.ingest(frame(TYPES.SENSOR,p,{session:43,seq:2,ms:300}));
 assert.equal(state.view().sensor.ultrasonicCm,null);
 assert.equal(state.view().sensor.obstaclePhase,7);
});

test('extended sensor packet explains why the bypass has not returned',()=>{
 const state=new RobotState(),p=Buffer.alloc(32);packet().copy(p);
 p[0]|=4;p[20]=7;p.writeInt16LE(-1,24);p[26]=2;p[27]=0;
 p.writeInt16LE(145,28);p[30]=1|2;p[31]=2;
 state.ingest(frame(TYPES.SENSOR,p,{session:44,seq:1,ms:200}));
 const d=state.view().sensor.obstacleDiag;
 assert.equal(d.sideRangeMm,-1);assert.equal(d.objectSeen,true);
 assert.equal(d.clearHits,0);assert.equal(d.travelSinceHitMm,145);
 assert.equal(d.waitReason,2);
 assert.ok(state.logs.some(l=>l.message.includes('side=NO ECHO')&&l.message.includes('clear=0/3')));
 assert.throws(()=>interpret(decodeFrame(frame(TYPES.SENSOR,Buffer.alloc(31)))),/sensor payload/);
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

test('latest hump peak comes from a finished hump, even when lower',()=>{
 const s=new RobotState();
 s.ingestMeasurementLine('HUMP STARTED: climbing','mqtt');
 s.ingestMeasurementLine('HUMP: descending','mqtt');
 assert.equal(s.view().latestHumpMm,null);
 s.ingestMeasurementLine('HUMP DONE: peak_x10mm=216 count=12','mqtt');
 assert.equal(s.view().latestHumpMm,21.6);
 s.ingestMeasurementLine('HUMP DONE: peak_x10mm=182 count=13','mqtt');
 s.ingestMeasurementLine('HUMP STARTED: climbing','mqtt');
 s.ingestMeasurementLine('HUMP DONE: peak_x10mm=300 count=','mqtt');
 assert.equal(s.view().latestHumpMm,18.2);
});

test('obstacle width and forward split use the matching scan summary',()=>{
 const s=new RobotState();
 s.ingestMeasurementLine('[OBS] Object -25..30 deg, face 194 mm ahead, width 141 mm (near < 600 mm)','mqtt');
 assert.deepEqual(s.view().latestObstacleScan,{widthMm:141,leftMm:null,rightMm:null});
 s.ingestMeasurementLine('[OBS] Left : edge 30 deg, 135 mm out; need 251 mm; [BC ADC] raw=184','mqtt');
 s.ingestMeasurementLine('[OBS] Right: edge -25 deg, 143 mm out; need 259 mm','mqtt');
 assert.deepEqual(s.view().latestObstacleScan,{widthMm:141,leftMm:null,rightMm:null});
 s.ingestMeasurementLine('[OBS] Object -25..30 deg, face 194 mm ahead, width 146 mm','mqtt');
 const split='[OBS] Front split: width=146 mm left=13 mm right=133 mm';
 s.ingestSerial(split);
 assert.deepEqual(s.view().latestObstacleScan,{widthMm:146,leftMm:null,rightMm:null});
 s.ingestMeasurementLine('[BC ADC] raw=184','mqtt');
 s.ingestMeasurementLine(split,'mqtt');
 assert.deepEqual(s.view().latestObstacleScan,{widthMm:146,leftMm:13,rightMm:133});
 s.ingestMeasurementLine('[OBS] Object 0..0 deg, face 220 mm ahead, width 0 mm','mqtt');
 s.ingestMeasurementLine('[OBS] Front split: width=0 mm left=0 mm right=0 mm','mqtt');
 assert.deepEqual(s.view().latestObstacleScan,{widthMm:null,leftMm:null,rightMm:null});
});

test('a lost, stale, or inconsistent split leaves the scan width only',()=>{
 const s=new RobotState();
 const summary='[OBS] Object -25..30 deg, face 194 mm ahead, width 141 mm';
 const split='[OBS] Front split: width=141 mm left=0 mm right=141 mm';
 for(const breakLine of ['[netlog] lost 217 bytes','[OBS] Phase 3 at x=0 y=0 heading=0 ddeg',
  '[OBS] Front split: width=141 mm left=12 mm right=134 mm']){
  s.ingestMeasurementLine(summary,'mqtt');
  s.ingestMeasurementLine(breakLine,'mqtt');
  s.ingestMeasurementLine(split,'mqtt');
  assert.deepEqual(s.view().latestObstacleScan,{widthMm:141,leftMm:null,rightMm:null});
 }
 s.ingestSerial(summary);
 s.ingestSerial('[OBS] Front split: width=140 mm left=0 mm right=140 mm');
 assert.deepEqual(s.view().latestObstacleScan,{widthMm:140,leftMm:0,rightMm:140});
});

test('coarse sonar split is marked only for the scan with an echo estimate',()=>{
 const s=new RobotState();
 const summary='[OBS] Object -20..35 deg, face 187 mm ahead, width 136 mm';
 const split='[OBS] Front split: width=136 mm left=97 mm right=39 mm';
 s.ingestMeasurementLine(summary,'mqtt');
 s.ingestMeasurementLine(split+' (echo estimate)','mqtt');
 assert.deepEqual(s.view().latestObstacleScan,{widthMm:136,leftMm:97,rightMm:39,echoEstimate:true});
 s.ingestMeasurementLine(summary,'mqtt');
 s.ingestMeasurementLine(split+' (echo esti','mqtt');
 assert.deepEqual(s.view().latestObstacleScan,{widthMm:136,leftMm:null,rightMm:null});
 s.ingestMeasurementLine(summary,'mqtt');
 s.ingestMeasurementLine(split,'mqtt');
 assert.deepEqual(s.view().latestObstacleScan,{widthMm:136,leftMm:97,rightMm:39});
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
