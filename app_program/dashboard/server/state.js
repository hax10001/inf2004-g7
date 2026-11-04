'use strict';
const {decodeFrame,interpret}=require('./protocol');
const sourceNames=['motion','line','terrain','obstacle','mission'];
const eventNames={1:'Barcode decoded',2:'Junction detected',3:'Hump detected',4:'Obstacle detected',5:'Bypass chosen',6:'Line reacquired',7:'Safety fault',8:'Line lost',9:'Hump finished',10:'Mission state',11:'IMU calibration requested (simulation)',12:'Navigation action queued',13:'Junction confirmed',14:'Turn started',15:'Turn completed',16:'Line reacquired',17:'Line following resumed',18:'Navigation safety stop',19:'Scan not queued'};
const scanFailureReasons=['Reason unavailable (firmware reported 0)',
  'Invalid sensor input','Barcode element width mismatch','Capture buffer overflow',
  'Capture timed out','Invalid encoder travel','Scan interrupted',
  'Code 39 pattern did not validate','Scanner/result queue busy'];
function scanFailureReason(code){return scanFailureReasons[code]||'Unknown decoder error';}
const READINESS_REASONS=[
  [1,'MQTT offline'],[2,'controller not in remote hold'],
  [4,'Start/Stop pending'],[8,'IMU not calibrated'],
  [16,'IR reading unavailable'],[32,'IR data stale'],
  [64,'controller lock or clock unavailable'],
  [128,'barcode capture still processing']
];
function readinessReasons(mask){
  if(mask==null)return [];
  return READINESS_REASONS.filter(([bit])=>mask&bit).map(([,name])=>name);
}
class RobotState {
  constructor(trackWidthMm=120) {this.trackWidthMm=trackWidthMm;this.logs=[];this.logSequence=0;this.reset();}
  log(category,message,level='info') {
    const line={id:++this.logSequence,time:Date.now(),category,level,message:String(message).slice(0,280)};
    this.logs.unshift(line);if(this.logs.length>300)this.logs.length=300;
    return line;
  }
  reset(){this.session=null;this.lastSeq=null;this.ackSequences=new Set();this.lastMotionSeq=null;this.odomGaps=0;this.frames=0;this.badFrames=0;
    this.pose={x:0,y:0,theta:0,distanceMm:0};this.points=[{x:0,y:0,ms:0,uncertain:false}];this.events=[];
    this.snapshot=null;this.robotStatus=null;this.routeMapping=null;this.sensor=null;this.barcodeDiag=null;this.scanOutcomes=[];this.serialSensor=null;this.serialBarcode=null;this.dropped=0;this.lastSeen=0;this.lastRobotMs=0;this.lastRobotClockAt=null;this.acks=[];this.uncertain=false;this.mode='LIVE';}
  recordScanOutcome(status, detail, options={}){
    const now=Date.now();
    const item={id:++this.logSequence,at:now,status,detail,letter:options.letter||null,
      failureStatus:options.failureStatus??null,count:options.count||1,source:options.source||'LIVE'};
    // A completed-count diagnostic may precede the actual letter event. Replace
    // that provisional result if the decoded letter arrives shortly afterward.
    const last=this.scanOutcomes[0];
    if(status==='success' && item.letter && last?.status==='success' &&
       !last.letter && last.source===item.source && now-last.at<2500){
      this.scanOutcomes[0]=item;
    }else{
      this.scanOutcomes.unshift(item);
      if(this.scanOutcomes.length>30)this.scanOutcomes.length=30;
    }
    const level=status==='failed'?'warn':'info';
    this.log('barcode',detail,level); // continuous telemetry view
    this.log('scanner',detail,level); // important event in command monitor
  }
  ingestSerial(text){
    const line=String(text||'').replace(/[\r\n]+/g,' ').trim().slice(0,280);
    if(!line)return false;
    const decoded=line.match(/\[BARCODE\]\s+Decoded:\s*([A-Z])\b/i)||
      line.match(/\[BC RESULT\]\s+([A-Z])\b/i);
    const matchSensor=line.match(/^\[SENS\] IR L=(\d+) R=(\d+) B=(\d+) roll_cd=(-?\d+) pitch_cd=(-?\d+) heading_cd=(\d+) hump=(\d+) sonar=(\d+) phase=(\d+) encL=(-?\d+) encR=(-?\d+) valid=(\d+)/);
    if(matchSensor){
      const v=matchSensor.slice(1).map(Number),mask=v[11];
      this.serialSensor={valid:mask,ir:(mask&1)?v[0]|v[1]<<1|v[2]<<2:null,
        rollDeg:(mask&2)?v[3]/100:null,pitchDeg:(mask&2)?v[4]/100:null,
        headingDeg:(mask&16)?v[5]/100:null,ultrasonicCm:(mask&4)&&v[7]!==65535?v[7]:null,
        humpMm:(mask&2)?v[6]:null,obstaclePhase:(mask&4)?v[8]:null,
        leftEncoder:(mask&8)?v[9]:null,rightEncoder:(mask&8)?v[10]:null,
        source:'USB SERIAL',received:Date.now()};
      this.log('serial',line);
    } else if(decoded){
      const letter=decoded[1].toUpperCase();
      this.serialBarcode={letter,at:Date.now(),source:'USB/UART SERIAL'};
      this.log('barcode',`Barcode decoded: ${letter} (USB/UART serial)`, 'info');
    } else this.log(line.startsWith('[NET]')?'network':line.startsWith('[CTRL]')?'mission':line.startsWith('[BC]')?'barcode':'serial',line);
    return true;
  }
  ingest(buffer,origin='LIVE'){
    let f,msg;try{f=decodeFrame(buffer);msg=interpret(f);}catch(e){this.badFrames++;this.log('network','Rejected binary telemetry: '+e.message,'error');return {error:e.message};}
    if(this.session!==f.session){this.reset();this.session=f.session;this.log('mission',`Session ${f.session} opened (${origin})`);}
    /* Firmware sends ACK ahead of queued telemetry. Accept that priority
     * without moving the telemetry sequence watermark past older frames. */
    const ackFrame=f.type===33;
    if(ackFrame){
      this.ackSequences.add(f.seq);
      if(this.ackSequences.size>80)this.ackSequences.delete(this.ackSequences.values().next().value);
    } else if(this.lastSeq!==null){
      const delta=(f.seq-this.lastSeq)&65535;
      if(delta===0 || delta>=32768){this.badFrames++;this.log('network',`Ignored duplicate or out-of-order frame ${f.seq}`,'warn');return {error:'duplicate/out-of-order'};}
      let skippedAcks=0;
      for(const seq of this.ackSequences){
        const gap=(seq-this.lastSeq)&65535;
        if(gap>0 && gap<delta){skippedAcks++;this.ackSequences.delete(seq);}
      }
      if(delta-skippedAcks!==1){this.odomGaps++;this.uncertain=true;
        this.log('network',`Wire sequence gap: ${this.lastSeq} → ${f.seq}`,'warn');}
    }
    if(!ackFrame)this.lastSeq=f.seq;
    this.frames++;this.lastSeen=Date.now();this.mode=origin;
    /* EVENT and ODOM packets may be historical backlog. Their original
     * timestamp must never move the command clock backwards. An ACK echoes
     * the request clock, not the robot's current uptime either. */
    if([1,5,6,7,8].includes(f.type) &&
       (this.lastRobotClockAt==null ||
        (((f.ms-this.lastRobotMs)>>>0)<0x80000000))){
      this.lastRobotMs=f.ms;
      this.lastRobotClockAt=this.lastSeen;
    }
    if(f.flags&1)this.uncertain=true;
    if(msg.kind==='status'){
      const prev=this.robotStatus;
      this.robotStatus={...msg,source:origin,received:Date.now(),
        commandToken:f.ms>>>0,commandSession:f.session};
      if(!prev || prev.mappingLoaded!==msg.mappingLoaded ||
         prev.ready!==msg.ready || prev.running!==msg.running ||
         prev.mission!==msg.mission ||
         prev.barcodeMotorHold!==msg.barcodeMotorHold ||
         ((prev.pwmLeft||0)>0)!==((msg.pwmLeft||0)>0) ||
         ((prev.pwmRight||0)>0)!==((msg.pwmRight||0)>0)){
        const blockers=readinessReasons(msg.readinessBlockers);
        const note=msg.mission===5?'NAVIGATION SAFETY STOP — press STOP once to reset, then wait for READY before START':
          msg.mission===7?'CONTROLLER FAULT — physical inspection and reboot required; STOP cannot reset':
          !msg.mappingLoaded?'ROUTE MAPPING NOT LOADED; send mapping before Start':
          msg.running?'Robot reports RUNNING':
          msg.ready?'Ready for START':
          'START blocked: '+(blockers.length?blockers.join(', '):
            'controller not ready (older firmware)');
        const drive=(msg.pwmLeft==null)?'' :
          `; barcode_hold=${msg.barcodeMotorHold?'YES':'NO'}`+
          `; commanded_PWM_L=${msg.pwmLeft} R=${msg.pwmRight}`;
        this.log('mission',`ROBOT STATUS: ${note}${drive} (source=${origin})`,
          msg.ready?'info':'warn');
      }
      if(!msg.mappingLoaded && this.routeMapping?.state==='ACCEPTED' &&
         Date.now()-this.routeMapping.at>1500){
        this.routeMapping={...this.routeMapping,state:'INVALIDATED',note:'Robot reports mapping not loaded; send it again'};
      }
    } else if(msg.kind==='sensor'){
      this.sensor={...msg,source:origin,received:Date.now()};
      if(origin==='LIVE'){
        const n=v=>v==null?'n/a':String(v);
        const bits=msg.ir;
        this.log('telemetry',`IR L=${bits==null?'?':bits&1?1:0} R=${bits==null?'?':bits&2?1:0} B=${bits==null?'?':bits&4?1:0} | `+
          `IMU roll=${n(msg.rollDeg)}° pitch=${n(msg.pitchDeg)}° heading=${n(msg.headingDeg)}° `+
          `cal=${msg.imuCalibrated?1:0} hump=${n(msg.humpPhase)} | `+
          `encL=${n(msg.leftEncoder)} encR=${n(msg.rightEncoder)} | `+
          `sonar=NOT INSTALLED`);
      }
    } else if(msg.kind==='barcodeDiag'){
      const previous=this.barcodeDiag;
      this.barcodeDiag={...msg,source:origin,received:Date.now()};
      if(!previous || previous.completed!==msg.completed || previous.rejected!==msg.rejected){
        /* The successful-count packet and actual letter event can arrive
         * in either order. Never invent a success alert without the letter:
         * completed counters remain visible in diagnostic telemetry. */
        if(previous && msg.rejected>previous.rejected){
          const count=msg.rejected-previous.rejected;
          this.recordScanOutcome('failed',
            `SCAN FAILED ×${count} — ${scanFailureReason(msg.failureStatus)} (code ${msg.failureStatus})`,
            {count,failureStatus:msg.failureStatus,source:origin});
        }
      }
    } else if(msg.kind==='snapshot'){
      if(this.snapshot && this.snapshot.mission!==msg.mission)this.log('mission',`Mission state ${this.snapshot.mission} → ${msg.mission}`);
      if(this.snapshot && this.snapshot.line!==msg.line)this.log('movement',`Line detector state ${this.snapshot.line} → ${msg.line}`);
      if(this.snapshot && this.snapshot.maxHumpMm!==msg.maxHumpMm && msg.maxHumpMm!==null)this.log('imu',`Maximum hump ${msg.maxHumpMm} mm`);
      if(this.snapshot && this.snapshot.faults!==msg.faults && msg.faults)this.log('mission',`Fault mask 0x${msg.faults.toString(16)}`,'error');
      this.snapshot=msg;
    } else if(msg.kind==='odometry'){
      if(this.lastMotionSeq!==null && ((msg.sourceSeq-this.lastMotionSeq)&65535)!==1){this.odomGaps++;this.uncertain=true;this.log('movement','Missing incremental wheel update','warn');}
      this.lastMotionSeq=msg.sourceSeq;
      const mm=(msg.leftMm+msg.rightMm)/2,dth=(msg.rightMm-msg.leftMm)/this.trackWidthMm;
      this.pose.x+=mm*Math.cos(this.pose.theta+dth/2);
      this.pose.y+=mm*Math.sin(this.pose.theta+dth/2);
      this.pose.theta+=dth;this.pose.distanceMm+=Math.abs(mm);
      this.points.push({x:this.pose.x,y:this.pose.y,ms:f.ms,uncertain:this.uncertain});
      if(this.points.length>2000)this.points.shift();
    } else if(msg.kind==='event') {
      const sample=this.points.slice().reverse().find(p=>p.ms<=f.ms)||this.points[0];
      const name=eventNames[msg.eventKind]||`Event ${msg.eventKind}`;
      const source=sourceNames[msg.source-1]||'unknown';
      this.events.unshift({id:msg.eventId,source,name,kind:msg.eventKind,value:msg.value,ms:f.ms,x:sample.x,y:sample.y,
        locationMeaning:'robot detection position',positionValid:this.lastMotionSeq!==null&&!this.uncertain});
      if(this.events.length>200)this.events.pop();
      const navNames=['Straight','Left','Right','U-turn'];
      const detail=msg.eventKind===1 && msg.value<=25?`letter=${String.fromCharCode(65+msg.value)}`:
        msg.eventKind>=12&&msg.eventKind<=17&&msg.value<=3?`action=${navNames[msg.value]}`:
        `value=${msg.value}`;
      if(msg.eventKind===1 && msg.value<=25){
        const letter=String.fromCharCode(65+msg.value);
        this.recordScanOutcome('success',`SCAN SUCCESS — decoded letter ${letter} (letter=${letter})`,
          {letter,source:origin});
      }else{
        this.log(source==='terrain'?'imu':source==='motion'||source==='line'?'movement':
          source==='obstacle'?'obstacle':'mission',`${name} · ${source} · ${detail}`,
          msg.eventKind===7?'error':'info');
      }
    } else if(msg.kind==='checkpoint') {
      if(Math.abs(msg.cumulativeMm-this.pose.distanceMm)>100){this.uncertain=true;this.log('movement','Distance checkpoint differs from reconstructed route','warn');}
    } else if(msg.kind==='heartbeat'){
      if(msg.dropped>this.dropped)this.log('network',`Firmware dropped ${msg.dropped-this.dropped} telemetry records`,'warn');
      this.dropped=msg.dropped;
    } else if(msg.kind==='ack'){
      this.acks.unshift({...msg,received:Date.now()});this.acks=this.acks.slice(0,20);
      const label=['','SET_MAPPING','START','STOP','SNAPSHOT','RECALIBRATE_IMU'][msg.command]||`COMMAND ${msg.command}`;
      const reason={1:'Invalid command',2:'BAD STATE: mapping not loaded or robot already running',
        3:'Invalid mapping',4:'Duplicate command',5:'NOT READY: IR/IMU/controller safety gate',
        6:'Command pending',7:'STALE COMMAND: latest robot STATUS token missing or expired'}[msg.result];
      this.log('commands',`${label} #${msg.transaction}: ${msg.result===0?'accepted':
        'REJECTED: '+(reason||'code '+msg.result)}; actuator completion is not implied`,
        msg.result===0?'info':'warn');
    }
    return {frame:f,msg};
  }
  mappingRequested(letters,transaction){
    this.routeMapping={letters:[...letters],transaction,session:this.session,state:'PENDING',
      at:Date.now(),note:'Sent via MQTT; robot has not acknowledged'};
  }
  mappingResult(transaction,result){
    if(!this.routeMapping || this.routeMapping.transaction!==transaction ||
      this.routeMapping.session!==this.session)return false;
    this.routeMapping={...this.routeMapping,state:result===0?'ACCEPTED':'REJECTED',
      at:Date.now(),result,note:result===0?'Robot ACK accepted mapping':'Robot rejected mapping (code '+result+')'};
    return true;
  }
  mappingTimeout(transaction){
    if(!this.routeMapping || this.routeMapping.transaction!==transaction ||
      this.routeMapping.state!=='PENDING')return;
    this.routeMapping={...this.routeMapping,state:'TIMED_OUT',
      at:Date.now(),note:'No robot ACK received; mapping not confirmed'};
  }
  view(){return {session:this.session,frames:this.frames,badFrames:this.badFrames,odomGaps:this.odomGaps,
    pose:this.pose,points:this.points,events:this.events,lastMotionSeq:this.lastMotionSeq,snapshot:this.snapshot,dropped:this.dropped,
    lastSeen:this.lastSeen,lastRobotMs:this.lastRobotMs,
    lastRobotClockAt:this.lastRobotClockAt,acks:this.acks,uncertain:this.uncertain,mode:this.mode,
    logs:this.logs,robotStatus:this.robotStatus,routeMapping:this.routeMapping,serialBarcode:this.serialBarcode,sensor:this.sensor,barcodeDiag:this.barcodeDiag,scanOutcomes:this.scanOutcomes,
    serialSensor:this.serialSensor};}
}
module.exports={RobotState,readinessReasons};
