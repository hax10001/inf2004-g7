#!/usr/bin/env node
/* MQTT message sizes: compact binary frames vs the same values as text.
 * Replays real robot traffic from dashboard recordings (*.jsonl), decodes
 * every frame with the dashboard's own decoder, and compares the binary
 * frame with the decoded values written as JSON text (header fields
 * included: type, session, sequence, time).
 * Usage: node tools/message_sizes.js [recordings-dir] > out.csv */
const fs=require('fs'),path=require('path');
const P=require('../app_program/dashboard/server/protocol.js');
const dir=process.argv[2]||path.join(__dirname,'../app_program/dashboard/recordings');
const names=Object.fromEntries(Object.entries(P.TYPES).map(([k,v])=>[v,k]));
const stats={};
for(const file of fs.readdirSync(dir).filter(f=>f.endsWith('.jsonl'))){
  for(const line of fs.readFileSync(path.join(dir,file),'utf8').split('\n')){
    if(!line.includes('payloadBase64'))continue;
    let rec;try{rec=JSON.parse(line);}catch{continue;}
    const raw=Buffer.from(rec.payloadBase64,'base64');
    let f,v;try{f=P.decodeFrame(raw);v=P.interpret(f);}catch{continue;}
    const text=JSON.stringify({type:names[f.type]||f.type,session:f.session,
      seq:f.seq,ms:f.ms,...v});
    const key=`${names[f.type]||f.type} (${f.payload.length} B payload)`;
    const s=stats[key]??={n:0,bin:0,payload:f.payload.length,text:0};
    s.n++;s.bin+=raw.length;s.text+=Buffer.byteLength(text);
  }
}
console.log('Message,Frames_replayed,Binary_payload_bytes,Binary_frame_bytes,Text_JSON_bytes_avg,Binary_vs_text_percent');
let tb=0,tt=0;
for(const [k,s] of Object.entries(stats).sort()){
  const b=s.bin/s.n,t=s.text/s.n;tb+=s.bin;tt+=s.text;
  console.log(`${k},${s.n},${s.payload},${b.toFixed(0)},${t.toFixed(0)},${(100*b/t).toFixed(0)}%`);
}
console.log(`ALL,${Object.values(stats).reduce((a,s)=>a+s.n,0)},,${tb},${tt},${(100*tb/tt).toFixed(0)}%`);
