'use strict';
/* Deterministic binary MQTT replay. Every plotted value is a SIMULATION.
 * --auto-start/--fast are demo-only CLI flags and never control a physical car. */
const {mqttClient}=require('../server/mqtt');
/* LAN mode uses persistent local secrets; do not make the student paste them into the shell. */
const fs=require('node:fs'),path=require('node:path');
let lanSecrets={};try{lanSecrets=JSON.parse(fs.readFileSync(path.join(__dirname,'../.lan-secrets.json'),'utf8'));}catch{}
const {frame,TYPES,decodeFrame,COMMANDS,crc8}=require('../server/protocol');
const host=process.env.MQTT_HOST||'127.0.0.1',port=+(process.env.BROKER_PORT||1883);
const sim=mqttClient({host,port,clientId:'b1-sim',user:process.env.MQTT_USER||lanSecrets.mqttUser||'',password:process.env.MQTT_PASSWORD||lanSecrets.mqttPassword||'',onMessage(topic,b){
  try {const f=decodeFrame(b);if(f.type!==TYPES.COMMAND)return;
    const command=f.payload[0],id=f.payload.readUInt16LE(1);
    let outcome=0;
    if(command===COMMANDS.SET_MAPPING){if(running)outcome=2;else mappingReady=true;}
    if(command===COMMANDS.START){if(!mappingReady)outcome=2;else running=true;}
    if(command===COMMANDS.STOP)running=false;
    if(command===COMMANDS.RECALIBRATE_IMU){
      if(running)outcome=2;
      else {simCalibrated=true;event(3,11,0);}
    }
    if(command===COMMANDS.RECALIBRATE_BARCODE){
      if(running)outcome=2;
      else simScannerState=1; /* learning white, then ready */
    }
    const p=Buffer.alloc(4);p[0]=command;p.writeUInt16LE(id,1);p[3]=outcome;
    send(TYPES.ACK,p,'ack');
  }catch(e){console.warn('bad command:',e.message);}
}});
const autoStart=process.argv.includes('--auto-start');
const intervalMs=process.argv.includes('--fast')?80:150;
let session=42,seq=0,mseq=0,tick=0,cycle=0,subscribed=false,running=autoStart,mappingReady=true,simCalibrated=true,simScannerState=2;
/* npm run demo starts paused; ./demo.sh opts into simulated autoplay only. */
let left=0,right=0;
function send(type,p,topic='telemetry',flags=0){const payload=frame(type,p,{session,seq:++seq,ms:cycle*intervalMs,flags});
  sim.publish(`${process.env.MQTT_NAMESPACE||'g7/b1'}/sim/${topic}`,payload);
}
function putBits(items){const out=Buffer.alloc(13);let n=0;for(const [v,count] of items){for(let i=0;i<count;i++,n++)if((v>>>i)&1)out[n>>3]|=1<<(n&7);}return out;}
function snap(phase){
  const speed=running?120:0,turn=running&&phase===1;
  const sourceMask=23,staleMask=0,mission=running?2:0,line=phase===3?4:1,
    ir=phase===1?7:2,barcode=(phase===1?2:31),action=1,actionValid=phase===1?1:0;
  const hump=running&&phase===2?Math.round(45+22*Math.sin(tick/3)):0,maxHump=phase>=2?67:0;
  return putBits([[sourceMask,5],[staleMask,5],[mission,3],[line,3],[ir,3],[barcode,5],
    [action,2],[actionValid,1],[hump,10],[maxHump,10],[1023,10],[0,2],[0,8],[0,8],
    [((speed-(turn?40:0))&4095),12],[((speed+(turn?40:0))&4095),12]]);
}
function event(source,kind,value=0){const p=Buffer.alloc(6);p[0]=source;p[1]=kind;p.writeUInt16LE(tick&65535,2);p.writeUInt16LE(value&65535,4);send(TYPES.EVENT,p);}
function loop(){if(!sim.ready)return;
 if(!subscribed){subscribed=true;sim.subscribe(`${process.env.MQTT_NAMESPACE||'g7/b1'}/sim/command`);console.log(autoStart?'SIMULATION AUTO-DRIVE READY. Only synthetic readings; no hardware control.':'SIMULATED robot ready, PAUSED. Open http://localhost:3000 and click Start.');}
 const t=tick%160,phase=t<55?0:t<87?1:t<125?2:0;
 if(running){
   const dd=phase===1? [12,24]:[18,18];left+=dd[0];right+=dd[1];
   const p=Buffer.alloc(6);p.writeInt16LE(dd[0],0);p.writeInt16LE(dd[1],2);p.writeUInt16LE(++mseq,4);
   send(TYPES.ODOM,p);
 }
 if(running&&t===55)event(2,1,2);if(running&&t===75)event(2,2,1);
 if(running&&t===87)event(3,3,45);if(running&&t===125)event(3,9,67);
 if(cycle%3===0)send(TYPES.SNAPSHOT,snap(phase));
 if(cycle%10===0){
   const sensor=Buffer.alloc(24);sensor[0]=27;sensor[1]=phase===1?7:2;
   sensor.writeInt16LE(phase===2?150:0,2);sensor.writeInt16LE(phase===2?100:0,4);
   sensor.writeUInt16LE(12345,6);sensor[10]=phase===2?1:0;sensor[11]=1;
   sensor.writeInt32LE(left,12);sensor.writeInt32LE(right,16);
   sensor[20]=255;send(TYPES.SENSOR,sensor);
   send(TYPES.STATUS,Buffer.from([mappingReady?1:0,1,running?1:0,23,running?2:6]));
   /* Simulated scanner status (42-byte barcode diagnostic). */
   const d=Buffer.alloc(42);d[0]=2;d[28]=simScannerState;
   d.writeUInt16LE(195,30);d.writeUInt16LE(17,32);d.writeUInt16LE(1,34);
   d[36]=1;d[37]=2;d[38]=0;d.writeUInt16LE(1800,40);
   send(TYPES.BARCODE_DIAG,d);
   if(simScannerState===1)simScannerState=2;
 }
 if(cycle%20===0){const h=Buffer.alloc(4);h.writeUInt32LE(0);send(TYPES.HEARTBEAT,h);}
 if(cycle%65===0){const checkpoint=Buffer.alloc(4);checkpoint.writeUInt32LE(Math.round((left+right)/2),0);send(TYPES.CHECKPOINT,checkpoint);}
 cycle++;if(running)tick++;
}
const t=setInterval(loop,intervalMs);
process.on('SIGINT',()=>{clearInterval(t);sim.close();process.exit(0);});
