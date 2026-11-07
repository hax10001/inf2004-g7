const test=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path');
const {frame,command,interpret,decodeFrame,TYPES,COMMANDS}=require('../server/protocol');
const {RobotState}=require('../server/state');

function diag({state=2,white=195,sigma=17,rejects=0,pass=0,valid=0,letter=31,reason=0,ms=0}={}){
  const d=Buffer.alloc(42);d[0]=2;d[28]=state;d[29]=rejects;
  d.writeUInt16LE(white,30);d.writeUInt16LE(sigma,32);d.writeUInt16LE(pass,34);
  d[36]=valid;d[37]=letter;d[38]=reason;d.writeUInt16LE(ms,40);
  return d;
}

test('42-byte barcode diagnostic carries scanner state, white and last pass',()=>{
  const m=interpret(decodeFrame(frame(TYPES.BARCODE_DIAG,diag({pass:3,valid:1,letter:24,ms:1810}))));
  assert.equal(m.kind,'barcodeDiag');
  assert.equal(m.scanner.stateName,'ready');
  assert.equal(m.scanner.whiteMean,195);assert.equal(m.scanner.whiteSigma,17);
  assert.equal(m.scanner.lastPass,3);assert.equal(m.scanner.lastLetter,'Y');
  assert.equal(m.scanner.lastProcessMs,1810);
  const dark=interpret(decodeFrame(frame(TYPES.BARCODE_DIAG,diag({state:6,white:2472,rejects:2}))));
  assert.equal(dark.scanner.stateName,'NOT ON WHITE');assert.equal(dark.scanner.whiteRejects,2);
});

test('older 28-byte barcode diagnostic still decodes without scanner fields',()=>{
  const m=interpret(decodeFrame(frame(TYPES.BARCODE_DIAG,Buffer.alloc(28))));
  assert.equal(m.scanner,null);
  assert.throws(()=>interpret(decodeFrame(frame(TYPES.BARCODE_DIAG,Buffer.alloc(30)))));
});

test('RECALIBRATE_BARCODE is command 6 with no arguments',()=>{
  assert.equal(COMMANDS.RECALIBRATE_BARCODE,6);
  assert.deepEqual([...command('RECALIBRATE_BARCODE',9)],[6,9,0]);
});

test('dashboard logs scanner state changes and each pass result',()=>{
  const s=new RobotState();
  const send=(p,seq)=>s.ingest(frame(TYPES.BARCODE_DIAG,p,{session:5,seq,ms:seq*1000}));
  send(diag({state:6,white:2472}),1);
  assert.ok(s.logs.some(l=>l.level==='warn'&&/NOT ON WHITE/.test(l.message)&&/Recalibrate barcode/.test(l.message)));
  send(diag({state:2,white:198}),2);
  assert.ok(s.logs.some(l=>/Scanner: ready \(white 198 ±17\)/.test(l.message)));
  send(diag({state:2,pass:1,valid:1,letter:5,ms:1900}),3);
  assert.ok(s.logs.some(l=>/Pass 1: read F in 1\.9 s/.test(l.message)));
  send(diag({state:2,pass:2,valid:0,reason:3,ms:600}),4);
  assert.ok(s.logs.some(l=>l.level==='warn'&&/Pass 2: no letter \(ambiguous letters\)/.test(l.message)));
});

test('UI and server expose a barcode recalibrate control',()=>{
  const root=path.join(__dirname,'..');
  const html=fs.readFileSync(path.join(root,'public/index.html'),'utf8');
  const app=fs.readFileSync(path.join(root,'public/app.js'),'utf8');
  const server=fs.readFileSync(path.join(root,'server/index.js'),'utf8');
  assert.match(html,/id="recalibrateBarcode"/);
  assert.match(app,/command\('RECALIBRATE_BARCODE'\)/);
  /* Scan banner leads with live scanner state once a result is old. */
  assert.match(app,/SCANNER READY · waiting for next barcode/);
  assert.match(app,/RE-ARMING · waiting for white after the barcode/);
  assert.match(app,/NOT ARMED · sensor not on white/);
  assert.match(server,/'RECALIBRATE_BARCODE'\]\.includes\(kind\)/);
  assert.match(server,/STOP the robot and place the barcode sensor over white/);
});
