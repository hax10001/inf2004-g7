const test=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path');
const {frame,command,interpret,decodeFrame,TYPES,COMMANDS}=require('../server/protocol');
const {RobotState}=require('../server/state');

function diag({state=2,white=195,sigma=17,rejects=0,pass=0,valid=0,letter=31,reason=0,ms=0,rejected=0,failureStatus=0}={}){
  const d=Buffer.alloc(42);d[0]=2;d[28]=state;d[29]=rejects;
  d.writeUInt32LE(rejected,4);d[15]=failureStatus;
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

test('an unreadable completed capture remains visible through rearm even when firmware did not count a rejection',()=>{
 const s=new RobotState();let seq=0;
 const send=p=>s.ingest(frame(TYPES.BARCODE_DIAG,p,{session:5,seq:++seq}));
 send(diag({state:3}));send(diag({state:4}));
 send(diag({state:5,pass:1,reason:7,ms:600}));
 assert.equal(s.scanOutcomes.length,1);
 assert.equal(s.scanOutcomes[0].status,'failed');
 assert.match(s.scanOutcomes[0].detail,/No barcode pattern found/);
 assert.ok(s.logs.some(x=>x.category==='scanner'&&/SCAN FAILED/.test(x.message)));
 send(diag({state:2,pass:1,reason:7,ms:600}));
 send(diag({state:2,pass:1,reason:7,ms:600,rejected:1,failureStatus:7}));
 assert.equal(s.scanOutcomes.length,1,'READY and delayed counter do not repeat the failure');
});

test('first diagnostic recovers a failure and simultaneous pass/counter updates create one result',()=>{
 const s=new RobotState();let seq=0;
 const send=p=>s.ingest(frame(TYPES.BARCODE_DIAG,p,{session:5,seq:++seq}));
 send(diag({pass:1,reason:3,rejected:1,failureStatus:7}));
 assert.equal(s.scanOutcomes.length,1);
 assert.match(s.scanOutcomes[0].detail,/ambiguous letters/);
 send(diag({pass:2,reason:1,rejected:2,failureStatus:7}));
 assert.equal(s.scanOutcomes.length,2);
 send(diag({pass:2,reason:1,rejected:2,failureStatus:7}));
 assert.equal(s.scanOutcomes.length,2);
});

test('a real diagnostic letter recovers a dropped result event and pairs once in either arrival order',()=>{
 for(const eventFirst of [false,true]){
  const s=new RobotState();let seq=0;
  const send=(type,p)=>s.ingest(frame(type,p,{session:5,seq:++seq}));
  const event=id=>Buffer.from([2,1,id,0,5,0]);
  if(eventFirst)send(TYPES.EVENT,event(1));
  send(TYPES.BARCODE_DIAG,diag({pass:1,valid:1,letter:5}));
  if(!eventFirst)send(TYPES.EVENT,event(1));
  assert.equal(s.scanOutcomes.length,1);
  assert.equal(s.scanOutcomes[0].letter,'F');
  send(TYPES.EVENT,event(2));
  send(TYPES.BARCODE_DIAG,diag({pass:2,valid:1,letter:5}));
  assert.equal(s.scanOutcomes.length,2,'the same letter on a new pass is a separate scan');
 }
});

test('legacy diagnostic already containing rejected attempts shows the failure on connect',()=>{
 const s=new RobotState();const p=Buffer.alloc(28);
 p.writeUInt32LE(2,4);p[15]=4;
 s.ingest(frame(TYPES.BARCODE_DIAG,p,{session:5,seq:1}));
 assert.equal(s.scanOutcomes.length,1);
 assert.equal(s.scanOutcomes[0].count,2);
 assert.match(s.scanOutcomes[0].detail,/Capture timed out/);
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
  /* Compact live state keeps arming visible without a diagnostic banner. */
  assert.match(app,/2:'Ready'/);
  assert.match(app,/5:'Rearming'/);
  assert.match(app,/6:'Needs white'/);
  assert.match(server,/'RECALIBRATE_BARCODE'\]\.includes\(kind\)/);
  assert.match(server,/STOP the robot and place the barcode sensor over white/);
});
