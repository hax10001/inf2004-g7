const test=require('node:test'),assert=require('node:assert/strict');
const {frame,decodeFrame,command,snapshot,packMapping,interpret,TYPES}=require('../server/protocol');
const {topicMatch}=require('../server/mqtt');
const {RobotState}=require('../server/state');
test('mapping A,B,C,D packs 20 bits in three bytes',()=>{
  assert.deepEqual([...packMapping(['A','B','C','D'])],[0x20,0x88,0x01]);
  assert.throws(()=>packMapping(['A','A','C','D']));assert.throws(()=>packMapping(['a','B','C','D']));
  assert.deepEqual([...command('SET_MAPPING',45,['A','B','C','D'])],[1,45,0,0x20,0x88,1]);
});
test('frame CRC detects modifications and refuses truncated frames',()=>{
  const bytes=frame(TYPES.ODOM,Buffer.from([1,2,3,4,5,6]),{session:7,seq:8,ms:222});
  assert.equal(decodeFrame(bytes).seq,8);
  assert.throws(()=>decodeFrame(bytes.subarray(0,-1)));
  const damaged=Buffer.from(bytes);damaged[7]^=1;assert.throws(()=>decodeFrame(damaged));
  damaged[7]^=1;damaged[1]=99;assert.throws(()=>decodeFrame(damaged));
});
test('snapshot has valid zero and invalid sentinels',()=>{
  assert.throws(()=>snapshot(Buffer.alloc(12)));
  const s=snapshot(Buffer.alloc(13));assert.equal(s.humpMm,0);assert.equal(s.speedLeft,0);
});
test('MQTT wildcard matching',()=>{
  assert(topicMatch('g7/b1/+/telemetry','g7/b1/sim/telemetry'));
  assert(!topicMatch('g7/b1/+/telemetry','g7/b1/robot/ack'));
  assert(topicMatch('g7/#','g7/b1/robot/ack'));
});
test('odometry integration and missing delta flag',()=>{
 const s=new RobotState(120);
 const od=(seq,src,left,right)=>{const p=Buffer.alloc(6);p.writeInt16LE(left,0);p.writeInt16LE(right,2);p.writeUInt16LE(src,4);return frame(2,p,{session:3,seq,ms:seq*100});};
 s.ingest(od(1,1,100,100));assert.equal(s.pose.x,100);assert.equal(s.pose.y,0);
 s.ingest(od(2,2,0,100));assert(s.pose.theta>0);
 s.ingest(od(4,4,50,50));assert.equal(s.uncertain,true);assert(s.odomGaps>=1);
});

test('MQTT STATUS reports mapping and readiness independently of USB serial',()=>{
 const src=new RobotState();const bytes=frame(TYPES.STATUS,Buffer.from([0,0,0,0x17,6]),{session:7,seq:1,ms:1000});
 const decoded=interpret(decodeFrame(bytes));
 assert.equal(decoded.kind,'status');assert.equal(decoded.mappingLoaded,false);
 src.ingest(bytes,'LIVE');assert.equal(src.view().robotStatus.mappingLoaded,false);
 assert(src.view().logs.some(x=>x.message.includes('MAPPING NOT LOADED')));
 const b=frame(TYPES.STATUS,Buffer.from([1,1,0,0x17,6]),{session:7,seq:2,ms:2000});
 src.ingest(b,'LIVE');assert.equal(src.view().robotStatus.ready,true);
});

test('status frame decoding rejects invalid sizes',()=>{
 const bad=frame(TYPES.STATUS,Buffer.alloc(3),{session:7,seq:1,ms:1});
 assert.throws(()=>interpret(decodeFrame(bad)),/status payload/);
});

test('a live binary sensor frame produces telemetry-console lines',()=>{
 const src=new RobotState();const payload=Buffer.alloc(24);payload[0]=0x1b;payload[1]=5;
 payload.writeInt16LE(-123,2);payload.writeInt16LE(210,4);payload.writeUInt16LE(6500,6);
 payload.writeInt32LE(1234,12);payload.writeInt32LE(5678,16);
 src.ingest(frame(TYPES.SENSOR,payload,{session:4,seq:1,ms:5000}),'LIVE');
 assert(src.view().logs.some(x=>x.category==='telemetry'&&x.message.includes('IR L=1 R=0 B=1')));
 assert(src.view().logs.some(x=>x.category==='telemetry'&&x.message.includes('encL=1234')));
});
