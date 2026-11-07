'use strict';
const http=require('node:http'),fs=require('node:fs'),path=require('node:path');
const crypto=require('node:crypto');
const {interfaces,primary}=require('./network');
const {createBroker,mqttClient}=require('./mqtt');
const {RobotState,readinessReasons}=require('./state');
const {frame,command,TYPES,packMapping}=require('./protocol');
const env=process.env;
const {Recorder}=require('./recorder');
const {CommandTracker}=require('./commands');
const {streamCsv}=require('./recording_export');
const {RobotLog}=require('./robotlog');
const recorder=new Recorder(env.RECORDING_DIR||path.resolve(__dirname,'../recordings'));
/* Robot serial console over MQTT, saved beside the recording. */
const robotLog=new RobotLog(env.RECORDING_DIR||path.resolve(__dirname,'../recordings'),recorder.id);
process.on('SIGINT',()=>Promise.all([recorder.close(),robotLog.close()]).then(()=>process.exit(0)));
process.on('SIGTERM',()=>Promise.all([recorder.close(),robotLog.close()]).then(()=>process.exit(0)));
/* Reuse local LAN secrets for both npm start and npm run lan, so the demo can
 * connect in either mode after one-time WiFi setup. Never serve this file. */
let localSecrets={};try{localSecrets=JSON.parse(fs.readFileSync(path.resolve(__dirname,'../.lan-secrets.json'),'utf8'));}catch{}
const WEB_BIND=env.WEB_BIND||'127.0.0.1',BROKER_BIND=env.BROKER_BIND||'127.0.0.1';
const WEB_PORT=+(env.WEB_PORT||3000),BROKER_PORT=+(env.BROKER_PORT||1883);
const BROKER_HOST=env.BROKER_HOST||'127.0.0.1',BROKER_EXTERNAL=env.BROKER_EXTERNAL==='1';
const DASHBOARD_SOURCE=env.DASHBOARD_SOURCE||'auto';
const MQTT_NAMESPACE=env.MQTT_NAMESPACE||'g7/b1';
const ROBOT_ID=env.ROBOT_ID||'robot';
if(!/^[a-zA-Z0-9_-]+(?:\/[a-zA-Z0-9_-]+)*$/.test(MQTT_NAMESPACE)||
   !/^[a-zA-Z0-9_-]+$/.test(ROBOT_ID)||ROBOT_ID==='sim'){
  console.error('Invalid MQTT_NAMESPACE or ROBOT_ID');process.exit(1);
}
/* Never send actuator/calibration requests to a physical robot by default. */
const LIVE_COMMANDS_ENABLED=env.LIVE_COMMANDS_ENABLED==='1';
const LIVE_IMU_RECAL_ENABLED=env.LIVE_IMU_RECAL_ENABLED==='1';
if(!['auto','sim','robot'].includes(DASHBOARD_SOURCE)){console.error('DASHBOARD_SOURCE must be auto, sim, or robot');process.exit(1);}
let lastLiveFrame=0,lastSimFrame=0;
const ROBOT_STATUS_TOKEN_MAX_AGE_MS=2500;
const MQTT_USER=env.MQTT_USER??localSecrets.mqttUser??'',MQTT_PASSWORD=env.MQTT_PASSWORD??localSecrets.mqttPassword??'';
const ADMIN_TOKEN=env.ADMIN_TOKEN??localSecrets.adminToken??'';
if((WEB_BIND!=='127.0.0.1'&&WEB_BIND!=='::1'&&!ADMIN_TOKEN)||
   (BROKER_BIND!=='127.0.0.1'&&BROKER_BIND!=='::1'&&(!MQTT_USER||!MQTT_PASSWORD))){
  console.error('LAN exposure requires ADMIN_TOKEN for web, MQTT_USER and MQTT_PASSWORD for MQTT.');process.exit(1);
}
const state=new RobotState(+(env.TRACK_WIDTH_MM||120));
const commandTracker=new CommandTracker();
const subscribers=new Set();let tx=1,txSession=null,streamReady=false;
/* Confirmed mapping survives a Pico reboot within this dashboard process.
 * Restoring it configures a route only; it NEVER starts the motors. */
let confirmedMapping=null;
const restoreAttempted=new Set();
const mappingStarts=new Map();
const pendingMappings=new Map();
function networkDiagnostics(){
  const now=Date.now();const addrs=interfaces();const main=primary();
  return {ip:main?.ip||null,networkInterface:main?.interface||null,addresses:addrs,
    browserPort:WEB_PORT,brokerPort:BROKER_PORT,brokerExternal:BROKER_EXTERNAL,
    webLanEnabled:WEB_BIND==='0.0.0.0'||WEB_BIND==='::',
    mqttLanEnabled:BROKER_EXTERNAL||(BROKER_BIND==='0.0.0.0'||BROKER_BIND==='::'),
    brokerTarget:BROKER_EXTERNAL?BROKER_HOST:(main?.ip||null),
    brokerReachableFromGateway:streamReady,
    robotLastPacket:lastLiveFrame||null,simulatorLastPacket:lastSimFrame||null,
    robotStatus:lastLiveFrame&&now-lastLiveFrame<5000?'RECEIVING LIVE PACKETS':
      lastLiveFrame&&now-lastLiveFrame<15000?'STALE':'NOT RECEIVING',
    simulatorStatus:lastSimFrame&&now-lastSimFrame<5000?'RUNNING':'NOT RECEIVING',
    sharedWifiVerified:false,adminCommandsEnabled:LIVE_COMMANDS_ENABLED,
    note:'Live packets prove robot-to-broker network communication, not physical proximity, router SSID equality or actual motor control.'};
}
function currentView(){return {...state.view(),commands:commandTracker.view(),recording:recorder.status(),brokerConnected:streamReady,connection:networkDiagnostics(),serverTime:Date.now()};}
const send=(res,event,data)=>{try{res.write(`event: ${event}\ndata: ${JSON.stringify(data)}\n\n`);}catch{}};
function push(){const v=currentView();for(const r of subscribers)send(r,'state',v);}
const broker=BROKER_EXTERNAL?null:createBroker({host:BROKER_BIND,port:BROKER_PORT,user:MQTT_USER,password:MQTT_PASSWORD});
let client;
function startPreconditions(kind){
 const status=state.robotStatus;
 if(kind==='START' || kind==='SET_MAPPING'){
   if(kind==='START'){
     if(!status?.received || Date.now()-status.received>2000)
       throw Error('Robot readiness status is stale; wait for a new robot STATUS packet');
     if(!status?.mappingLoaded)throw Error('Mapping not loaded; send and ACK route mapping first');
     if(status?.running)throw Error('Robot already running; STOP before requesting a new START');
     if(status?.ready!==true){
       if(status?.mission===5)throw Error('Robot in NAVIGATION SAFETY STOP: press STOP once, then wait for robot READY before START. Do not repeatedly press START.');
       if(status?.mission===7)throw Error('Robot in CONTROLLER FAULT: START locked for safety. Inspect and resolve the fault; STOP cannot clear a hardware error.');
       const why=readinessReasons(status?.readinessBlockers);
       throw Error('Robot NOT READY: '+(why.length?why.join(', '):
         'waiting for controller readiness and fresh IR/IMU')); 
     }
     if(commandTracker.pending.some(x=>x.kind==='START'))
       throw Error('A START request is still awaiting ACK; do not send another');
   }else if(status?.running)throw Error('Cannot change mapping while robot is running');
 }
}
function commandClock(kind){
 /* STOP must remain available even if robot STATUS stopped arriving.
  * The firmware still enforces the active boot session and safety hold. */
 if(kind==='STOP')return 0;
 const s=state.robotStatus;
 if(!s || s.source!=='LIVE' || s.commandSession!==state.session ||
    !s.commandToken || !s.received ||
    Date.now()-s.received>ROBOT_STATUS_TOKEN_MAX_AGE_MS)
   throw Error('Waiting for fresh Pico STATUS synchronization; MQTT telemetry alone is not sufficient. Reconnect the robot and retry once status updates.');
 /* Echo exactly the robot-issued status time as a freshness challenge.
  * Never extrapolate Pico uptime from browser or server wall-clock. */
 return s.commandToken>>>0;
}
function publishRobotCommand(kind, letters=[]){
 if(!streamReady || Date.now()-lastLiveFrame>5000 || state.mode!=='LIVE')throw Error('Live robot telemetry required');
 if(!LIVE_COMMANDS_ENABLED)throw Error('Live controls disabled');
 startPreconditions(kind);
 const robotClock=commandClock(kind);
 /* Re-anchor transaction counter after dashboard restart. The Pico's STATUS
  * reports its last accepted transaction, which survives MQTT reconnections. */
 const lastTx=state.robotStatus?.lastCommandTransaction;
 if(txSession!==state.session){
   txSession=state.session;
   tx=lastTx==null?tx:((lastTx+1)&65535);
 }else if(lastTx!=null){
   const next=(lastTx+1)&65535,forward=(next-tx)&65535;
   if(forward && forward<32768)tx=next;
 }
 const transaction=tx&65535;
 tx=(tx+1)&65535;
 const data=frame(TYPES.COMMAND,command(kind,transaction,letters),
    {session:state.session,seq:tx,ms:robotClock});
 const sent=client.publish(`${MQTT_NAMESPACE}/${ROBOT_ID}/command`,data);
 commandTracker.sent({transaction,kind,target:ROBOT_ID,session:state.session,published:sent});
 state.log('commands',`${kind} #${transaction}: ${sent?'sent via MQTT':'publish failed'}`,
    sent?'info':'error');push();
 if(!sent)throw Error('MQTT publish failed');
 return transaction;
}
function bridge(){
  client=mqttClient({host:BROKER_HOST,port:BROKER_PORT,clientId:'b1-dashboard',user:MQTT_USER,password:MQTT_PASSWORD,
    onMessage(topic,bytes){
      const segments=topic.split('/');
      const ns=MQTT_NAMESPACE.split('/');
      /* Serial console text: shown and saved, never parsed as telemetry. */
      if(segments.length===ns.length+2&&segments.slice(0,ns.length).every((s,i)=>s===ns[i])&&
         segments[ns.length]===ROBOT_ID&&segments[ns.length+1]==='log'){
        const lines=robotLog.add(bytes);
        if(lines.length)for(const r of subscribers)send(r,'log',{lines});
        return;
      }
      if(segments.length!==ns.length+2||segments.slice(0,ns.length).some((s,i)=>s!==ns[i])||
         ![ROBOT_ID,'sim'].includes(segments[ns.length])||!['telemetry','ack'].includes(segments[ns.length+1]))return;
      const source=segments[ns.length];
      if(DASHBOARD_SOURCE!=='auto'&&source!==DASHBOARD_SOURCE)return;
      if(DASHBOARD_SOURCE==='auto'&&source==='sim'&&Date.now()-lastLiveFrame<5000)return;
      recorder.write('mqtt',{topic,source,payloadBase64:bytes.toString('base64')});
      const beforeSession=state.session;
      const r=state.ingest(bytes, source==='sim'?'SIMULATED':'LIVE');
      if(!r.error && beforeSession!==null && state.session!==beforeSession){
        mappingStarts.clear();pendingMappings.clear();
        commandTracker.invalidateSession(state.session);
        txSession=null;
        state.log('commands','Robot reboot/new session detected: old pending commands cancelled; send mapping to this boot before START','warn');
      }
      if(!r.error){
        if(source===ROBOT_ID)lastLiveFrame=Date.now();if(source==='sim')lastSimFrame=Date.now();
        if(r.msg.kind==='ack'){
          const matched=commandTracker.acknowledge({...r.msg,session:r.frame.session},source);
          recorder.write('ack',{source,topic,transaction:r.msg.transaction,ackResult:r.msg.result,session:r.frame.session,matched:!!matched});
          const mapping=pendingMappings.get(r.msg.transaction);
          if(mapping && r.msg.command===1 && matched && source===ROBOT_ID &&
             r.frame.session===mapping.session){
            pendingMappings.delete(r.msg.transaction);
            state.mappingResult(r.msg.transaction,r.msg.result);
            if(r.msg.result===0 && Array.isArray(mapping.letters) &&
               mapping.letters.length===4){
              confirmedMapping={letters:[...mapping.letters],session:state.session};
            }
          }
          const awaiting=mappingStarts.get(r.msg.transaction);
          if(awaiting){
            if(r.msg.result===0 && matched && state.mode==='LIVE' &&
               state.session===awaiting.session){
              /* ACK only confirms storage; wait for a fresh device STATUS
               * proving mapping and readiness before issuing START. */
              awaiting.acked=true;
              state.log('commands',
                'Mapping accepted; waiting for robot READY status before START');
            }else{
              mappingStarts.delete(r.msg.transaction);
              state.log('commands','Mapping rejected or stale; START NOT sent','warn');
            }
          }
          if(!matched)state.log('commands','Unmatched or late ACK; not attributed to a pending command','warn');
        }
        if(r.msg.kind==='status' && source===ROBOT_ID){
          /* Reapply only the last robot-ACKed route after a different boot.
           * Never auto-START; the operator still must press START explicitly. */
          const rs=state.robotStatus;
          if(LIVE_COMMANDS_ENABLED && confirmedMapping &&
             confirmedMapping.session!==state.session &&
             rs && !rs.mappingLoaded && !rs.running &&
             !restoreAttempted.has(state.session) &&
             pendingMappings.size===0 && mappingStarts.size===0){
            restoreAttempted.add(state.session);
            if(restoreAttempted.size>32)restoreAttempted.delete(restoreAttempted.values().next().value);
            try{
              const id=publishRobotCommand('SET_MAPPING',confirmedMapping.letters);
              pendingMappings.set(id,{session:state.session,
                letters:[...confirmedMapping.letters],createdAt:Date.now()});
              state.mappingRequested(confirmedMapping.letters,id);
              state.log('commands','Restoring previously ACKed route after Pico reboot; START remains manual');
            }catch(e){state.log('commands','Auto-restore not sent: '+e.message,'warn');}
          }
          for(const [id,request] of mappingStarts){
            if(!request.acked || request.session!==state.session)continue;
            if(!state.robotStatus?.mappingLoaded || !state.robotStatus?.ready){
              const blockers=readinessReasons(state.robotStatus?.readinessBlockers);
              state.log('commands','Mapping ACK received, but START blocked: '+
                (blockers.join(', ')||'robot mapping/readiness not confirmed'),'warn');
              mappingStarts.delete(id);
              continue;
            }
            mappingStarts.delete(id);
            try{publishRobotCommand('START');}catch(e){state.log('commands',
              'Mapping applied but START NOT sent: '+e.message,'warn');}
          }
        }
      }
      if(r.error)console.warn('rejected telemetry:',r.error);
      push();
    },onClose(){streamReady=false;state.log('network','MQTT gateway disconnected; retrying','warn');push();setTimeout(bridge,2500);}});
  const timer=setInterval(()=>{if(client.socket.destroyed){clearInterval(timer);return;}
    if(client.ready&&!streamReady){streamReady=client.subscribe(`${MQTT_NAMESPACE}/+/telemetry`);client.subscribe(`${MQTT_NAMESPACE}/+/ack`);client.subscribe(`${MQTT_NAMESPACE}/${ROBOT_ID}/log`);state.log('network','MQTT gateway subscribed to binary telemetry');push();}
  },100);
}
const mime={'.html':'text/html; charset=utf-8','.js':'application/javascript; charset=utf-8','.css':'text/css; charset=utf-8','.svg':'image/svg+xml'};
const base=path.resolve(__dirname,'../public');
function authorized(req){
  const remote=req.socket.remoteAddress||'';
  if(remote==='127.0.0.1'||remote==='::1'||remote==='::ffff:127.0.0.1')return true;
  const got=req.headers['x-admin-token']||'';
  const a=Buffer.from(String(got)),b=Buffer.from(ADMIN_TOKEN);
  return ADMIN_TOKEN&&a.length===b.length&&crypto.timingSafeEqual(a,b);
}
const server=http.createServer((req,res)=>{
  const url=new URL(req.url,'http://localhost');
  res.setHeader('Cache-Control','no-store');res.setHeader('X-Content-Type-Options','nosniff');
  if(url.pathname==='/api/robotlog.txt'&&req.method==='GET'){
    if(!authorized(req)){res.writeHead(401);res.end('Unauthorized');return;}
    res.setHeader('Content-Type','text/plain; charset=utf-8');res.setHeader('Content-Disposition','attachment; filename="'+robotLog.snapshot().file+'"');
    const input=fs.createReadStream(robotLog.path);input.on('error',()=>res.destroy());input.pipe(res);return;
  }
  if(['/api/recording','/api/recording.csv'].includes(url.pathname)&&req.method==='GET'){
    if(!authorized(req)){res.writeHead(401);res.end('Unauthorized');return;}
    if(url.pathname.endsWith('.csv')){
      res.setHeader('Content-Type','text/csv; charset=utf-8');res.setHeader('Content-Disposition','attachment; filename="'+recorder.id+'.csv"');
      streamCsv(recorder.path,res).catch(()=>res.destroy());return;
    }
    res.setHeader('Content-Type','application/x-ndjson');res.setHeader('Content-Disposition','attachment; filename="'+recorder.id+'.jsonl"');
    const input=fs.createReadStream(recorder.path);input.on('error',()=>res.destroy());input.pipe(res);return;
  }
  if(url.pathname==='/api/state'&&req.method==='GET'){
    res.setHeader('Content-Type','application/json');res.end(JSON.stringify(currentView()));return;
  }
  if(url.pathname==='/api/network'&&req.method==='GET'){
    res.setHeader('Content-Type','application/json');res.end(JSON.stringify(networkDiagnostics()));return;
  }
  if(url.pathname==='/api/events'&&req.method==='GET'){
    res.writeHead(200,{'Content-Type':'text/event-stream','Connection':'keep-alive','Cache-Control':'no-store'});
    subscribers.add(res);send(res,'state',currentView());send(res,'log',{reset:true,...robotLog.snapshot()});
    const keep=setInterval(()=>{if(!res.destroyed)res.write(': keepalive\n\n');},12000);
    req.on('close',()=>{clearInterval(keep);subscribers.delete(res);});return;
  }
  if(url.pathname==='/api/serial'&&req.method==='POST'){
    // A local cable bridge only; NEVER accept unauthenticated LAN injection.
    const remote=req.socket.remoteAddress||'';
    if(!['127.0.0.1','::1','::ffff:127.0.0.1'].includes(remote)){
      res.writeHead(403);res.end('Local serial bridge only');return;
    }
    let body='';req.on('data',part=>{
      body+=part;if(body.length>1024)req.destroy();
    });
    req.on('end',()=>{
      const line=body.slice(0,512);
      if(state.ingestSerial(line)){
        recorder.write('serial',{source:'USB/UART',text:line.slice(0,280)});
        push();
      }
      res.writeHead(204);res.end();
    });return;
  }
  if(url.pathname==='/api/map-then-start'&&req.method==='POST'){
    if(!authorized(req)){res.writeHead(401,{'Content-Type':'application/json'});res.end(JSON.stringify({error:'Unauthorized'}));return;}
    if(!LIVE_COMMANDS_ENABLED){res.writeHead(409,{'Content-Type':'application/json'});
      res.end(JSON.stringify({code:'LIVE_CONTROLS_DISABLED',error:'Dashboard is running read-only (npm run lan). Stop that process and run bash run_live_dashboard.sh from the project root to enable physical mapping/Start/Stop.'}));return;}
    let body='';req.on('data',buf=>{body+=buf;if(body.length>2000)req.destroy();});
    req.on('end',()=>{
      try{
        const letters=JSON.parse(body).letters;
        packMapping(letters); /* Validate explicitly chosen distinct letters. */
        if(state.robotStatus?.running)throw Error('Robot already running; cannot change route mapping');
        const transaction=publishRobotCommand('SET_MAPPING',letters);
        mappingStarts.set(transaction,{session:state.session,createdAt:Date.now()});
        pendingMappings.set(transaction,{session:state.session,letters:[...letters],createdAt:Date.now()});
        state.mappingRequested(letters,transaction);
        push();
        res.writeHead(200,{'Content-Type':'application/json'});
        res.end(JSON.stringify({message:`Mapping #${transaction} sent. START will be sent only if the robot ACKs mapping; safety checks still apply.`}));
      }catch(error){res.writeHead(409,{'Content-Type':'application/json'});
        state.log('commands','Mapping NOT SENT: '+error.message,'warn');push();
        res.end(JSON.stringify({code:'MAPPING_REJECTED',error:error.message}));}
    });return;
  }
  if(url.pathname==='/api/command'&&req.method==='POST'){
    if(!authorized(req)){res.writeHead(401);res.end('Unauthorized: set dashboard token');return;}
    if(!streamReady){res.writeHead(503);res.end('MQTT bridge disconnected');return;}
    let body='';req.on('data',buf=>{body+=buf;if(body.length>2000)req.destroy();});
    req.on('end',()=>{
      try{
        const obj=JSON.parse(body);
        const kind=obj.kind;
        if(!['SET_MAPPING','START','STOP','SNAPSHOT','RECALIBRATE_IMU','RECALIBRATE_BARCODE'].includes(kind))throw Error('invalid command');
        const age=Date.now()-state.lastSeen;
        if(!state.frames||age>5000){res.writeHead(409);res.end('No recent telemetry. Start npm run demo for simulation, or connect the Pico W.');return;}
        const live=state.mode==='LIVE';
        if(live&&!LIVE_COMMANDS_ENABLED){res.writeHead(409);res.end('Read-only server (npm run lan). Stop it and use bash run_live_dashboard.sh from the project root to enable live mapping and Start/Stop.');return;}
        if(live&&kind==='RECALIBRATE_IMU'&&!LIVE_IMU_RECAL_ENABLED){res.writeHead(409);res.end('Live IMU recalibration not implemented by the current IMU-task owner; command not sent.');return;}
        if(kind==='RECALIBRATE_IMU'){
          if(!state.snapshot || ![0,6,7].includes(state.snapshot.mission)){
            res.writeHead(409);res.end('STOP the robot before requesting IMU recalibration.');return;
          }
        }
        if(kind==='RECALIBRATE_BARCODE'){
          /* The robot also refuses while running; this gives a clear reason. */
          if(!state.snapshot || ![0,6,7].includes(state.snapshot.mission)){
            res.writeHead(409);res.end('STOP the robot and place the barcode sensor over white before recalibrating the barcode.');return;
          }
        }
        if(live){
          const transaction=publishRobotCommand(kind,obj.letters||[]);
          if(kind==='SET_MAPPING'){
            pendingMappings.set(transaction,{session:state.session,letters:[...obj.letters],createdAt:Date.now()});
            state.mappingRequested(obj.letters,transaction);
          }
          res.setHeader('Content-Type','application/json');
          res.end(JSON.stringify({queued:true,transaction,command:kind,
            target:ROBOT_ID,acknowledged:false,
            note:'MQTT publish is not proof of physical actuation'}));
          return;
        }
        const target='sim',transaction=tx++;
        const bytes=frame(TYPES.COMMAND,
          command(kind,transaction,obj.letters||[]),
          {session:state.session??1,seq:tx,ms:state.lastRobotMs>>>0});
        const ok=client.publish(`${MQTT_NAMESPACE}/${target}/command`,bytes);
        commandTracker.sent({transaction,kind,target,session:state.session,published:ok});
        state.log('commands',`${kind} #${transaction}: ${ok?'sent via MQTT':'publish failed'}; awaiting ACK`,ok?'info':'error');push();
        res.setHeader('Content-Type','application/json');
        res.end(JSON.stringify({queued:ok,transaction,command:kind,target,acknowledged:false,note:'MQTT publish is not proof of physical actuation'}));
      }catch(e){res.writeHead(409,{'Content-Type':'application/json'});
        state.log('commands','Command NOT SENT: '+e.message,'warn');push();
        res.end(JSON.stringify({code:'COMMAND_BLOCKED',error:e.message}));}
    });return;
  }
  if(req.method!=='GET'){res.writeHead(405);res.end();return;}
  let rel=url.pathname==='/'?'/index.html':url.pathname;
  if(!['/index.html','/app.js','/style.css','/car3d.js','/car3d_fallback.js'].includes(rel)){res.writeHead(404);res.end();return;}
  const fp=path.join(base,rel);fs.readFile(fp,(err,buf)=>{if(err){res.writeHead(404);res.end();return;}
    res.setHeader('Content-Type',mime[path.extname(fp)]||'application/octet-stream');res.end(buf);});
});
setInterval(()=>{for(const [id,request] of pendingMappings){
 if(Date.now()-request.createdAt>6000){pendingMappings.delete(id);state.mappingTimeout(id);push();}
}for(const [id,request] of mappingStarts){
 if(Date.now()-request.createdAt>5000){mappingStarts.delete(id);
   state.log('commands',request.acked?
     'Mapping accepted but no confirmed READY status; START NOT sent':
     'Mapping ACK timeout; START NOT sent','warn');push();}
}},1000).unref();
setInterval(()=>{const expired=commandTracker.expire();for(const e of expired){
  recorder.write('command_timeout',{target:e.target,command:e.kind,transaction:e.transaction,session:e.session});
  state.log('commands',`${e.kind} #${e.transaction} timed out: no matching ACK`,'warn');
}if(expired.length)push();},1000).unref();
(broker?broker.start():Promise.resolve()).then(()=>{
 console.log('Control mode: '+(LIVE_COMMANDS_ENABLED?'LIVE CONTROL OPT-IN ENABLED — bench testing only':'READ-ONLY. For physical mapping/Start/Stop use bash run_live_dashboard.sh from project root.'));
 console.log(BROKER_EXTERNAL?`External MQTT broker: mqtt://${BROKER_HOST}:${BROKER_PORT}`:`Dev MQTT broker: mqtt://${BROKER_BIND}:${BROKER_PORT}`);
 bridge();server.listen(WEB_PORT,WEB_BIND,()=>console.log(`Dashboard: http://${WEB_BIND}:${WEB_PORT}`));
});
process.on('SIGINT',()=>{for(const r of subscribers)r.end();server.close();if(client)client.close();(broker?broker.close():Promise.resolve()).then(()=>process.exit(0));});
