#!/usr/bin/env node
/* Telemetry cadence from real robot recordings: interval between frames of
 * each type by the robot's own millisecond clock, and sequence gaps.
 * Usage: node tools/telemetry_timing.js [recordings-dir] */
const fs=require('fs'),path=require('path');
const P=require('../app_program/dashboard/server/protocol.js');
const dir=process.argv[2]||path.join(__dirname,'../app_program/dashboard/recordings');
const names=Object.fromEntries(Object.entries(P.TYPES).map(([k,v])=>[v,k]));
const gaps={};let frames=0,seqGaps=0,seqPairs=0;
for(const file of fs.readdirSync(dir).filter(f=>f.endsWith('.jsonl')).sort()){
  const last={};let lastSeq=null,lastSession=null;
  for(const line of fs.readFileSync(path.join(dir,file),'utf8').split('\n')){
    if(!line.includes('payloadBase64'))continue;
    let f;try{f=P.decodeFrame(Buffer.from(JSON.parse(line).payloadBase64,'base64'));}catch{continue;}
    frames++;
    if(lastSession===f.session&&lastSeq!==null){seqPairs++;if(((f.seq-lastSeq)&0xffff)!==1)seqGaps++;}
    lastSeq=f.seq;lastSession=f.session;
    const k=names[f.type];
    if(last[k]!==undefined&&f.ms>last[k]&&f.ms-last[k]<5000)(gaps[k]??=[]).push(f.ms-last[k]);
    last[k]=f.ms;
  }
}
const q=(a,p)=>a[Math.min(a.length-1,Math.floor(p*a.length))];
console.log('Type,Intervals,Median_ms,P95_ms,Max_ms');
for(const [k,a] of Object.entries(gaps).sort()){a.sort((x,y)=>x-y);
  console.log(`${k},${a.length},${q(a,0.5)},${q(a,0.95)},${a[a.length-1]}`);}
console.log(`Frames ${frames}; sequence continuity ${((1-seqGaps/seqPairs)*100).toFixed(2)}% (${seqGaps} gaps in ${seqPairs})`);
