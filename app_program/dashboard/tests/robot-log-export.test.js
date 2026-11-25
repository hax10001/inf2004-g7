'use strict';
/* Robot console log over MQTT and recording CSV export (FR12, NFR11). */
const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {RobotLog}=require('../server/robotlog');
const {streamCsv,columns}=require('../server/recording_export');

function tempDir(){return fs.mkdtempSync(path.join(os.tmpdir(),'g7-log-'));}

test('robot log joins chunks into whole lines and holds a partial line',async()=>{
 const dir=tempDir(),log=new RobotLog(dir,'t1');
 assert.deepEqual(log.add(Buffer.from('[CTRL] IDLE -> LINE_')).map(l=>l.text),[]);
 const lines=log.add(Buffer.from('FOLLOWING\r\n[NAV] event=20\npart'));
 assert.deepEqual(lines.map(l=>l.text),['[CTRL] IDLE -> LINE_FOLLOWING','[NAV] event=20']);
 assert.deepEqual(lines.map(l=>l.n),[1,2]);
 assert.equal(log.snapshot().lines.length,2);
 await log.close();
 assert.equal(fs.readFileSync(log.path,'latin1'),'[CTRL] IDLE -> LINE_FOLLOWING\n[NAV] event=20\n');
});

test('robot log flushes a runaway line and keeps only the newest lines',async()=>{
 const dir=tempDir(),log=new RobotLog(dir,'t2',3);
 const long=log.add(Buffer.from('x'.repeat(2001)));
 assert.equal(long.length,1);assert.equal(long[0].text.length,2001);
 log.add(Buffer.from('a\nb\nc\nd\n'));
 assert.deepEqual(log.snapshot().lines.map(l=>l.text),['b','c','d']);
 assert.equal(log.total,5);
 await log.close();
});

test('recording export writes a header, skips bad rows and neutralises formulas',async()=>{
 const dir=tempDir(),file=path.join(dir,'rec.jsonl');
 fs.writeFileSync(file,[
  JSON.stringify({receivedAt:'2026-11-25T00:00:00Z',kind:'mqtt',topic:'g7/b1/robot/telemetry',payloadBase64:'sQE='}),
  'not json','',
  JSON.stringify({kind:'ack',command:'=START()',transaction:7})
 ].join('\n'));
 let out='';
 const res={write(s){out+=s;return true;},end(){},once(){}};
 await streamCsv(file,res);
 const rows=out.trim().split('\n');
 assert.equal(rows[0],columns.join(','));
 assert.equal(rows.length,3);
 assert.match(rows[2],/,'=START\(\),7,/);
});
