'use strict';
const test=require('node:test'),assert=require('node:assert/strict');
const {spawn}=require('node:child_process');
const net=require('node:net');
const fs=require('node:fs'),path=require('node:path');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
function freePort(){return new Promise((resolve,reject)=>{const s=net.createServer();s.listen(0,'127.0.0.1',()=>{const port=s.address().port;s.close(()=>resolve(port));});s.on('error',reject);});}
async function poll(fn){for(let i=0;i<100;i++){try{if(await fn())return;}catch{}await delay(35);}throw Error('Server not ready');}
test('read-only LAN refuses mapping and Start with explicit launch instructions',async()=>{
 const [webPort,brokerPort]=await Promise.all([freePort(),freePort()]);
 const cwd=path.resolve(__dirname,'..');
 const env={...process.env,WEB_PORT:String(webPort),BROKER_PORT:String(brokerPort),WEB_BIND:'127.0.0.1',BROKER_BIND:'127.0.0.1',LIVE_COMMANDS_ENABLED:'0'};
 const server=spawn(process.execPath,['server/index.js'],{cwd,env,stdio:'ignore'});
 try{
  const base=`http://127.0.0.1:${webPort}`;
  await poll(async()=>{const r=await fetch(base+'/api/state');return r.ok;});
  const r=await fetch(base+'/api/map-then-start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({letters:['A','B','C','D']})});
  assert.equal(r.status,409);
  const body=await r.json();assert.equal(body.code,'LIVE_CONTROLS_DISABLED');
  assert.match(body.error,/run_live_dashboard\.sh/);
  const netdata=await(await fetch(base+'/api/network')).json();assert.equal(netdata.adminCommandsEnabled,false);
 }finally{server.kill('SIGTERM');}
});
test('telemetry and command monitor are intentionally separated',()=>{
 const app=fs.readFileSync(path.resolve(__dirname,'../public/app.js'),'utf8');
 const html=fs.readFileSync(path.resolve(__dirname,'../public/index.html'),'utf8');
 assert.match(app,/function monitorOnlyLogs\(/);assert.match(app,/\['commands','mission','network','scanner'\]/);
 assert.match(app,/function renderCommandLedger\(/);assert.match(app,/category==='telemetry'\|\|x.category==='barcode'/);
 assert.match(html,/id="commandLedgerOutput"/);assert.match(html,/id="telemetryOutput"/);
 assert.match(html,/Code 39 is the <strong>barcode symbology<\/strong>/);
});
