'use strict';
const MAGIC = 0xb1, VERSION = 1;
const TYPES = {SNAPSHOT:1,ODOM:2,EVENT:3,CHECKPOINT:4,HEARTBEAT:5,SENSOR:6,STATUS:7,BARCODE_DIAG:8,COMMAND:32,ACK:33};
const COMMANDS = {SET_MAPPING:1,START:2,STOP:3,SNAPSHOT:4,RECALIBRATE_IMU:5,RECALIBRATE_BARCODE:6};
/* Barcode scanner state and last-pass reason (42-byte diagnostic, V2). */
const SCANNER_STATES = ['off','learning white','ready','capturing','decoding','clearing','NOT ON WHITE'];
const SCAN_REASONS = ['read','too few bars','no letter layout fits','ambiguous letters',
  'model fit failed','centre/sensor distances disagree','too many dark marks','not barcode-like'];
function crc8(a) { let crc=0; for(const byte of a) { let b=byte; for(let i=0;i<8;i++) { const carry=(crc^b)&128; crc=(crc<<1)&255; if(carry)crc^=7;b=(b<<1)&255; } } return crc; }
function frame(type,payload,opts={}) {
  if(payload.length>48)throw Error('payload too large');
  const b=Buffer.alloc(14+payload.length);
  b[0]=MAGIC;b[1]=VERSION;b[2]=type;b[3]=opts.flags||0;
  b.writeUInt16LE(opts.session??1,4);b.writeUInt16LE(opts.seq??0,6);
  b.writeUInt32LE((opts.ms??0)>>>0,8);b[12]=payload.length;
  Buffer.from(payload).copy(b,13);b[b.length-1]=crc8(b.subarray(0,b.length-1));return b;
}
function decodeFrame(b) {
  b=Buffer.from(b);
  if(b.length<14||b.length>62||b[0]!==MAGIC||b[1]!==VERSION||
     b.length!==14+b[12]||crc8(b.subarray(0,-1))!==b[b.length-1])throw Error('invalid B1 frame/version/CRC');
  return {type:b[2],flags:b[3],session:b.readUInt16LE(4),seq:b.readUInt16LE(6),ms:b.readUInt32LE(8),payload:b.subarray(13,-1)};
}
function bits(buffer) { let pos=0; return n=>{let x=0;for(let i=0;i<n;i++,pos++)x|=((buffer[pos>>3]>>(pos&7))&1)<<i;return x>>>0;}; }
const signed=(n,w)=>n&(1<<(w-1))?n-(1<<w):n;
function snapshot(p) {
  if(p.length!==13||(p[12]&0xf8)!==0)throw Error('invalid snapshot length/unused bits');
  const b=bits(p), sourceMask=b(5), staleMask=b(5);
  const mission=b(3),line=b(3),ir=b(3),barcode=b(5),action=b(2),actionValid=!!b(1);
  const humpMm=b(10),maxHumpMm=b(10),obstacleCm=b(10),side=b(2),bypassCount=b(8),faults=b(8);
  return {sourceMask,staleMask,mission,line,ir,barcode:barcode===31?null:String.fromCharCode(65+barcode),
    action:actionValid?action:null,humpMm:humpMm===1023?null:humpMm,
    maxHumpMm:maxHumpMm===1023?null:maxHumpMm,obstacleCm:obstacleCm===1023?null:obstacleCm,
    side,bypassCount,faults,speedLeft:signed(b(12),12),speedRight:signed(b(12),12)};
}
function packMapping(letters) {
  if(letters.length!==4||new Set(letters).size!==4||letters.some(x=>!(/^[A-Z]$/).test(x)))throw Error('four distinct A–Z letters required');
  const nums=letters.map(x=>x.charCodeAt(0)-65);
  let v=(nums[0]|(nums[1]<<5)|(nums[2]<<10)|(nums[3]<<15))>>>0;
  return Buffer.from([v&255,(v>>8)&255,(v>>16)&15]);
}
function command(kind,transaction,letters=[]) {
  const id=COMMANDS[kind]; if(!id)throw Error('unknown command');
  const b=Buffer.alloc(id===1?6:3);b[0]=id;b.writeUInt16LE(transaction&65535,1);
  if(id===1)packMapping(letters).copy(b,3);return b;
}
function interpret(f) {
  const p=f.payload;
  if(f.type===TYPES.SNAPSHOT)return {kind:'snapshot',...snapshot(p)};
  if(f.type===TYPES.ODOM) { if(p.length!==6)throw Error('bad odom');return {kind:'odometry',leftMm:p.readInt16LE(0),rightMm:p.readInt16LE(2),sourceSeq:p.readUInt16LE(4)}; }
  if(f.type===TYPES.EVENT){if(p.length!==6)throw Error('bad event');return {kind:'event',source:p[0],eventKind:p[1],eventId:p.readUInt16LE(2),value:p.readUInt16LE(4)};}
  if(f.type===TYPES.CHECKPOINT){if(p.length!==4)throw Error('bad checkpoint');return {kind:'checkpoint',cumulativeMm:p.readUInt32LE(0)};}
  if(f.type===TYPES.HEARTBEAT){if(p.length!==4)throw Error('bad heartbeat');return {kind:'heartbeat',dropped:p.readUInt32LE(0)};}
  if(f.type===TYPES.SENSOR){
    if(p.length!==24 && p.length!==32)throw Error('bad sensor payload');
    const valid=p[0];
    return {kind:'sensor',valid,ir:(valid&1)?p[1]:null,
      rollDeg:(valid&2)?p.readInt16LE(2)/100:null,
      pitchDeg:(valid&2)?p.readInt16LE(4)/100:null,
      headingDeg:(valid&16)&&p.readUInt16LE(6)!==65535?p.readUInt16LE(6)/100:null,
      ultrasonicCm:(valid&4)&&p.readUInt16LE(8)!==65535?p.readUInt16LE(8):null,
      humpPhase:(valid&2)?p[10]:null,imuCalibrated:!!p[11],
      leftEncoder:(valid&8)?p.readInt32LE(12):null,
      rightEncoder:(valid&8)?p.readInt32LE(16):null,
      obstaclePhase:(valid&4)?p[20]:null,onHump:!!p[21],
      heightValid:!!p[22],odometryConfidence:p[23]<=2?p[23]:0,
      obstacleDiag:p.length===32?{sideRangeMm:p.readInt16LE(24),sideHits:p[26],
        clearHits:p[27],travelSinceHitMm:p.readInt16LE(28),active:!!(p[30]&1),
        objectSeen:!!(p[30]&2),returnProbeArmed:!!(p[30]&4),waitReason:p[31]}:null};
  }
  if(f.type===TYPES.BARCODE_DIAG){
    if(p.length!==28 && p.length!==42)throw Error('bad barcode diagnostic payload');
    const scanner=p.length===42?{state:p[28],
      stateName:SCANNER_STATES[p[28]]||`state ${p[28]}`,
      whiteRejects:p[29],whiteMean:p.readUInt16LE(30),whiteSigma:p.readUInt16LE(32),
      lastPass:p.readUInt16LE(34),lastValid:!!p[36],
      lastLetter:p[36]&&p[37]<26?String.fromCharCode(65+p[37]):null,
      lastReason:p[38],lastReasonName:SCAN_REASONS[p[38]]||`reason ${p[38]}`,
      lastProcessMs:p.readUInt16LE(40)}:null;
    return {kind:'barcodeDiag',mode:p[0],scanState:p[1],status:p[2],
      raw:p[3],rejected:p.readUInt32LE(4),completed:p.readUInt32LE(8),
      lastRunUnits:p.readUInt16LE(12),runCount:p[14],failureStatus:p[15],
      leftCount:p.readInt32LE(16),rightCount:p.readInt32LE(20),
      rawTransitions:p.readUInt32LE(24),scanner};
  }
  if(f.type===TYPES.STATUS){
    if(p.length!==5 && p.length!==7 && p.length!==9 && p.length!==14)throw Error('bad status payload');
    return {kind:'status',mappingLoaded:!!p[0],ready:!!p[1],running:!!p[2],
      sourceMask:p[3],mission:p[4],
      readinessBlockers:p.length>=7?p[5]:null,
      remoteHold:p.length>=7?!!p[6]:null,
      lastCommandTransaction:p.length>=9?p.readUInt16LE(7):null,
      barcodeMotorHold:p.length===14?!!p[9]:null,
      pwmLeft:p.length===14?p.readUInt16LE(10):null,
      pwmRight:p.length===14?p.readUInt16LE(12):null};
  }
  if(f.type===TYPES.ACK){if(p.length!==4)throw Error('bad ack');return {kind:'ack',command:p[0],transaction:p.readUInt16LE(1),result:p[3]};}
  throw Error('unsupported telemetry type '+f.type);
}
module.exports={TYPES,COMMANDS,SCANNER_STATES,SCAN_REASONS,crc8,frame,decodeFrame,snapshot,packMapping,command,interpret};
