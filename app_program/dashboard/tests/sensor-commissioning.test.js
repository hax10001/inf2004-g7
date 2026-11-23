const test = require('node:test');
const assert = require('node:assert/strict');
const {TYPES,frame,decodeFrame,interpret} = require('../server/protocol');
const {RobotState} = require('../server/state');
function diagnostic(overrides={}) {
  const p=Buffer.alloc(28);
  p[0]=2;p[1]=3;p[2]=0;p[3]=1;
  p.writeUInt32LE(5,4);p.writeUInt32LE(3,8);
  p.writeUInt16LE(123,12);p[14]=12;p[15]=4;
  p.writeInt32LE(-345,16);p.writeInt32LE(350,20);
  p.writeUInt32LE(42,24);
  Object.entries(overrides).forEach(([at,value])=>p[Number(at)]=value);
  return p;
}
test('barcode diagnostics are actual robot counters, not inferred barcode success',()=>{
 const parsed=interpret(decodeFrame(frame(TYPES.BARCODE_DIAG,diagnostic(),{session:9,seq:1,ms:2000})));
 assert.equal(parsed.kind,'barcodeDiag');assert.equal(parsed.mode,2);
 assert.equal(parsed.rawTransitions,42);assert.equal(parsed.completed,3);
 assert.equal(parsed.rejected,5);assert.equal(parsed.leftCount,-345);
 const s=new RobotState();
 s.ingest(frame(TYPES.BARCODE_DIAG,diagnostic(),{session:9,seq:1,ms:2000}));
 assert.equal(s.view().barcodeDiag.rawTransitions,42);
 assert.equal(s.view().events.length,0, 'diagnostics must not fabricate decoded-letter events');
});
test('invalid diagnostic length is rejected rather than fabricated',()=>{
 assert.throws(()=>interpret(decodeFrame(frame(TYPES.BARCODE_DIAG,Buffer.alloc(24)))),/diagnostic payload/);
});
test('sensor odometry flag labels nominal scale versus measured and retains heading validity',()=>{
 const p=Buffer.alloc(24);
 p[0]=2|8|16;p.writeInt16LE(100,2);p.writeUInt16LE(12755,6);
 p.writeInt32LE(-1056,12);p.writeInt32LE(1056,16);p[23]=1;
 const d=interpret(decodeFrame(frame(TYPES.SENSOR,p)));
 assert.equal(d.headingDeg,127.55);assert.equal(d.odometryConfidence,1);
 assert.equal(d.leftEncoder,-1056);assert.equal(d.rightEncoder,1056);
 p[23]=2;assert.equal(interpret(decodeFrame(frame(TYPES.SENSOR,p))).odometryConfidence,2);
 p[0]&=~16;assert.equal(interpret(decodeFrame(frame(TYPES.SENSOR,p))).headingDeg,null);
});
