const test=require('node:test'),assert=require('node:assert/strict');
const {spawn}=require('node:child_process');
const net=require('node:net');
const delay=ms=>new Promise(r=>setTimeout(r,ms));
const freePort=()=>new Promise((resolve,reject)=>{const s=net.createServer();s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));});s.on('error',reject);});
async function poll(url,condition,timeout=6500){
 const start=Date.now();let last;
 while(Date.now()-start<timeout){try{last=await(await fetch(url)).json();if(condition(last))return last;}catch{}await delay(100);}
 throw Error('timed out waiting for '+url+' last='+JSON.stringify(last));
}
test('E2E: local broker + simulator binary packets + HTTP dashboard + admin ACK',async()=>{
 const [webPort,brokerPort]=await Promise.all([freePort(),freePort()]);
 const env={...process.env,WEB_PORT:String(webPort),BROKER_PORT:String(brokerPort),WEB_BIND:'127.0.0.1',BROKER_BIND:'127.0.0.1'};
 const server=spawn(process.execPath,['server/index.js'],{cwd:__dirname+'/..',env,stdio:'ignore'});
 let demo;
 try{
   const url=`http://127.0.0.1:${webPort}`;
   await poll(url+'/api/state',s=>s.brokerConnected);
   demo=spawn(process.execPath,['tools/demo.js'],{cwd:__dirname+'/..',env,stdio:'ignore'});
   const diagnostics=await(await fetch(url+'/api/network')).json();
   assert.equal(diagnostics.brokerReachableFromGateway,true);
   assert.equal(diagnostics.robotStatus,'NOT RECEIVING');
   const v=await poll(url+'/api/state',s=>s.frames>6&&s.mode==='SIMULATED'&&s.snapshot?.sourceMask===23);
   assert.equal(v.badFrames,0);assert.equal(v.odomGaps,0);
   const afterDemo=await(await fetch(url+'/api/network')).json();
   assert.equal(afterDemo.simulatorStatus,'RUNNING');
   assert.equal(afterDemo.robotStatus,'NOT RECEIVING');
   const start=await fetch(url+'/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({kind:'START'})});
   assert.equal(start.status,200);
   await poll(url+'/api/state',s=>s.snapshot?.mission===2);
   const recalMoving=await fetch(url+'/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({kind:'RECALIBRATE_IMU'})});
   assert.equal(recalMoving.status,409);
   const stop=await fetch(url+'/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({kind:'STOP'})});
   assert.equal(stop.status,200);
   await poll(url+'/api/state',s=>s.snapshot?.mission===0);
   const recal=await fetch(url+'/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({kind:'RECALIBRATE_IMU'})});
   assert.equal(recal.status,200);
   await poll(url+'/api/state',s=>s.acks.some(x=>x.command===5));
   const sent=await fetch(url+'/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({kind:'SNAPSHOT'})});
   assert.equal(sent.status,200);assert.equal((await sent.json()).queued,true);
   const ack=await poll(url+'/api/state',s=>s.acks.some(x=>x.command===4));
   assert.equal(ack.acks[0].result,0);
 }finally{demo?.kill('SIGTERM');server.kill('SIGTERM');}
});
