'use strict';
const test=require('node:test'),assert=require('node:assert/strict');
const {spawn}=require('node:child_process');
const net=require('node:net');
const {mqttClient}=require('../server/mqtt');
const {frame,decodeFrame,TYPES}=require('../server/protocol');
const delay=ms=>new Promise(r=>setTimeout(r,ms));
const freePort=()=>new Promise((resolve,reject)=>{
 const s=net.createServer();s.listen(0,'127.0.0.1',()=>{const port=s.address().port;s.close(()=>resolve(port));});s.on('error',reject);
});
async function poll(fn,timeout=7000){const start=Date.now();let result;
 while(Date.now()-start<timeout){try{result=await fn();if(result)return result;}catch{}
 await delay(35);}throw Error('v7.9 reconnect condition not reached: '+JSON.stringify(result));}
test('after host restart transaction resumes at firmware counter; Pico reboot restores map but NEVER auto-START',async()=>{
 const [port,brokerPort]=await Promise.all([freePort(),freePort()]);
 const env={...process.env,WEB_PORT:String(port),BROKER_PORT:String(brokerPort),WEB_BIND:'127.0.0.1',BROKER_BIND:'127.0.0.1',LIVE_COMMANDS_ENABLED:'1',DASHBOARD_SOURCE:'robot'};
 const server=spawn(process.execPath,['server/index.js'],{cwd:__dirname+'/..',env,stdio:'ignore'});
 const url=`http://127.0.0.1:${port}`;
 let robot;let session=8123,seq=0;
 const commands=[];
 const emit=(type,p,ms)=>robot.publish('g7/b1/robot/telemetry',frame(type,p,{session,seq:++seq,ms}));
 const status=(mapped,lastTx,time)=>emit(TYPES.STATUS,Buffer.from([mapped?1:0,1,0,0x1b,6,0,1,lastTx&255,lastTx>>8]),time);
 try{
  await poll(async()=>{const x=await (await fetch(url+'/api/state')).json();return x.brokerConnected;});
  robot=mqttClient({host:'127.0.0.1',port:brokerPort,clientId:'robot-reconnect-v79',
   onMessage(topic,raw){if(topic!=='g7/b1/robot/command')return;
    const f=decodeFrame(raw),command=f.payload[0],tx=f.payload.readUInt16LE(1);
    commands.push({session,command,tx,token:f.ms});
    const ack=frame(TYPES.ACK,Buffer.from([command,tx&255,tx>>8,0]),{session,seq:++seq,ms:12345});
    setTimeout(()=>robot.publish('g7/b1/robot/ack',ack),25);
   }});
  await poll(async()=>robot.ready);robot.subscribe('g7/b1/robot/command');
  status(false,410,15000); // dashboard restarted: Pico already processed transaction 410
  await poll(async()=>{const s=await(await fetch(url+'/api/state')).json();return s.robotStatus?.commandToken===15000;});
  const reply=await fetch(url+'/api/command',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({kind:'SET_MAPPING',letters:['A','B','C','D']})});
  assert.equal(reply.status,200,await reply.text());
  const first=await poll(async()=>commands[0]);
  assert.deepEqual({kind:first.command,tx:first.tx,token:first.token},{kind:1,tx:411,token:15000});
  await poll(async()=>{const v=await(await fetch(url+'/api/state')).json();return v.routeMapping?.state==='ACCEPTED';});
  // A physical reboot loses volatile mapping, and gets a different boot session.
  session=8267;seq=0;
  status(false,0,1300);
  const restored=await poll(async()=>commands.find(c=>c.session===8267));
  assert.deepEqual({kind:restored.command,tx:restored.tx,token:restored.token},{kind:1,tx:1,token:1300});
  await poll(async()=>{const v=await(await fetch(url+'/api/state')).json();return v.routeMapping?.state==='ACCEPTED';});
  status(true,1,2400);
  await delay(180);
  assert.equal(commands.filter(x=>x.command===2).length,0,'auto map restoration must not auto-start');
  const v=await(await fetch(url+'/api/state')).json();
  assert.equal(v.robotStatus.mappingLoaded,true);
  assert.equal(v.routeMapping.state,'ACCEPTED');
 }finally{robot?.close();server.kill('SIGTERM');}
});
test('STOP can be sent with fresh MQTT link even before first STATUS token arrives',async()=>{
 const [port,brokerPort]=await Promise.all([freePort(),freePort()]);
 const env={...process.env,WEB_PORT:String(port),BROKER_PORT:String(brokerPort),WEB_BIND:'127.0.0.1',BROKER_BIND:'127.0.0.1',LIVE_COMMANDS_ENABLED:'1',DASHBOARD_SOURCE:'robot'};
 const server=spawn(process.execPath,['server/index.js'],{cwd:__dirname+'/..',env,stdio:'ignore'});
 let robot,received;
 const url=`http://127.0.0.1:${port}`;
 try{
  await poll(async()=>{const x=await (await fetch(url+'/api/state')).json();return x.brokerConnected;});
  robot=mqttClient({host:'127.0.0.1',port:brokerPort,clientId:'stop-without-status-v79',
    onMessage(topic,data){if(topic==='g7/b1/robot/command')received=decodeFrame(data);}});
  await poll(async()=>robot.ready);robot.subscribe('g7/b1/robot/command');
  robot.publish('g7/b1/robot/telemetry',frame(TYPES.SNAPSHOT,Buffer.alloc(13),{session:55,seq:1,ms:54321}));
  await poll(async()=>{const x=await(await fetch(url+'/api/state')).json();return x.session===55&&x.frames>0;});
  const reply=await fetch(url+'/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({kind:'STOP'})});
  assert.equal(reply.status,200,await reply.text());
  const cmd=await poll(async()=>received);
  assert.equal(cmd.payload[0],3);
  assert.equal(cmd.ms,0,'STOP is clock-independent; firmware still checks session');
  assert.equal(cmd.session,55);
 }finally{robot?.close();server.kill('SIGTERM');}
});
