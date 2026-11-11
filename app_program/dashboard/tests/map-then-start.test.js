'use strict';
const test=require('node:test'),assert=require('node:assert/strict');
const {spawn}=require('node:child_process');
const net=require('node:net');
const {mqttClient}=require('../server/mqtt');
const {frame,decodeFrame,TYPES}=require('../server/protocol');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
function freePort(){return new Promise((resolve,reject)=>{const s=net.createServer();s.listen(0,'127.0.0.1',()=>{const n=s.address().port;s.close(()=>resolve(n));});s.on('error',reject);});}
async function poll(fn,timeout=5000){const start=Date.now();while(Date.now()-start<timeout){const r=await fn();if(r)return r;await delay(50);}throw Error('Timed out waiting for mapping ACK -> Start');}
test('live mapping ACK is required before dashboard sends START; no USB serial needed',async()=>{
 const [webPort,brokerPort]=await Promise.all([freePort(),freePort()]);
 const session=4567,commands=[],sentTokens=[];
 const env={...process.env,MQTT_USER:'',MQTT_PASSWORD:'',ADMIN_TOKEN:'',WEB_PORT:String(webPort),BROKER_PORT:String(brokerPort),WEB_BIND:'127.0.0.1',BROKER_BIND:'127.0.0.1',LIVE_COMMANDS_ENABLED:'1',DASHBOARD_SOURCE:'robot'};
 const server=spawn(process.execPath,['server/index.js'],{cwd:__dirname+'/..',env,stdio:'ignore'});
 let robot,seq=0,uptime=5000;
 try{
  const base=`http://127.0.0.1:${webPort}`;
  await poll(async()=>{try{const d=await(await fetch(base+'/api/state')).json();return d.brokerConnected;}catch{return false;}});
  robot=mqttClient({host:'127.0.0.1',port:brokerPort,clientId:'robot-integration-test',user:'',password:'',
    onMessage(topic,payload){
      if(topic!=='g7/b1/robot/command')return;
      const f=decodeFrame(payload),cmd=f.payload[0],transaction=f.payload.readUInt16LE(1);commands.push(cmd);sentTokens.push(f.ms);
      if(cmd===1){
        /* Simulate only a valid robot mapping ACK; START is not emitted by
         * firmware test code until this ACK is delivered. */
        setTimeout(()=>{const ack=Buffer.from([1,transaction&255,transaction>>8,0]);
          robot.publish('g7/b1/robot/ack',frame(TYPES.ACK,ack,{session,seq:++seq,ms:uptime}));
          setTimeout(()=>robot.publish('g7/b1/robot/telemetry',
            frame(TYPES.STATUS,Buffer.from([1,1,0,0x1b,6,0,1]),
              {session,seq:++seq,ms:uptime+200})),120);},100);
      }
    }
  });
  await poll(async()=>robot.ready);
  robot.subscribe('g7/b1/robot/command');
  for(let i=0;i<5;i++){
    const payload=Buffer.alloc(13);payload[0]=0x10; // presence not safety gate; only enables packet freshness
    uptime+=100;
    robot.publish('g7/b1/robot/telemetry',frame(TYPES.SNAPSHOT,payload,{session,seq:++seq,ms:uptime}));
    await delay(40);
  }
  uptime+=1000;
  robot.publish('g7/b1/robot/telemetry',frame(TYPES.STATUS,
    Buffer.from([0,1,0,0x1b,6,0,1,0,0]),{session,seq:++seq,ms:uptime}));
  await poll(async()=>{const x=await(await fetch(base+'/api/state')).json();return x.mode==='LIVE' && x.robotStatus?.commandToken===uptime;});
  const r=await fetch(base+'/api/map-then-start',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({letters:['A','B','C','D']})});
  assert.equal(r.status,200,await r.text());
  await poll(async()=>commands.includes(1));
  await poll(async()=>commands.includes(2));
  assert.deepEqual(commands,[1,2]);
  assert.equal(sentTokens[0],uptime,'mapping echoes the robot-issued STATUS challenge');
  assert.equal(sentTokens[1],uptime+200,'START echoes fresh STATUS, not an estimated clock');
  const mapped=await poll(async()=>{
    const data=await(await fetch(base+'/api/state')).json();
    return data.routeMapping?.state==='ACCEPTED'?data:null;
  });
  assert.deepEqual(mapped.routeMapping.letters,['A','B','C','D']);
  assert.equal(mapped.routeMapping.session,session);
  robot.publish('g7/b1/robot/telemetry',frame(TYPES.STATUS,Buffer.from([1,1,0,0x1b,2]),{session,seq:++seq,ms:uptime+200}));
  const gotStatus=await poll(async()=>{
    const data=await(await fetch(base+'/api/state')).json();
    return data.robotStatus?.mappingLoaded?data:null;
  });
  assert.equal(gotStatus.routeMapping.state,'ACCEPTED');
  robot.publish('g7/b1/robot/telemetry',frame(TYPES.EVENT,Buffer.from([2,1,1,0,0,0]),{session,seq:++seq,ms:uptime+300}));
  await poll(async()=>{
    const data=await(await fetch(base+'/api/state')).json();
    return data.events?.some(e=>e.kind===1&&e.value===0&&!e.positionValid);
  });
 }finally{robot?.close();server.kill('SIGTERM');}
});
