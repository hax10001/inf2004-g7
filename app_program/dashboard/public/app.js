'use strict';
const el=id=>document.getElementById(id),num=(n,d=1)=>Number(n||0).toFixed(d);
const safe=x=>String(x??'').replace(/[&<>"']/g,ch=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));
window.lastAckTime=0;window.lastCommandIssue=0;
let telemetryUpdated=0,ledgerUpdated='',routeUpdated='';
let view=null,connected=false,zoom=1,drag=null,offset={x:0,y:0},activeView='overview',logPaused=false,pausedLogs=[],logUpdated=0,lastClearedLogId=0;
const missionNames=['IDLE','SELF-CHECK','RUNNING','SCANNING','LEGACY/UNUSED','NAV SAFETY STOP','STOPPED','FAULT'];
const actions=['STRAIGHT','LEFT','RIGHT','U-TURN'];
const scanFailureNames=['Reason unavailable (firmware reported 0)','Invalid sensor input','Barcode width mismatch',
  'Capture buffer overflow','Capture timed out','Invalid encoder travel',
  'Scan interrupted','Code 39 pattern mismatch','Scanner/result queue busy'];
const colors={1:'#7866BA',2:'#C49B3B',3:'#E7B750',4:'#B44C42',5:'#5A91B9',6:'#258971',7:'#B53F42',8:'#A9742C',9:'#D7A24A',10:'#536879'};
const lineNames=['UNKNOWN','FOLLOWING','LEFT BIAS','RIGHT BIAS','SEARCHING','JUNCTION','LOST','FAULT'];
const commandLabels=['','SET_MAPPING','START','STOP','SNAPSHOT','RECALIBRATE_IMU'];
const readyBits=[[1,'MQTT offline'],[2,'not in safe hold'],[4,'command pending'],
 [8,'IMU uncalibrated'],[16,'IR unavailable'],[32,'IR stale'],[64,'controller busy'],[128,'barcode capture processing']];
function readyDetail(status){
 if(status?.mission===5)return 'Navigation safety stop — press STOP, wait for HOLD, then START';
 if(status?.mission===7)return 'Controller fault — inspect robot; power-cycle only after resolving fault';
 const b=status?.readinessBlockers;
 if(b==null)return 'Unknown blocker (flash V7.8 firmware for details)';
 return readyBits.filter(([flag])=>b&flag).map(([,label])=>label).join(', ')||
   'Awaiting controller readiness';
}

function changeView(name){if(!['overview','three','telemetry','console'].includes(name))return;activeView=name;
 for(const v of ['overview','three','telemetry','console']){el('view-'+v).classList.toggle('hidden',v!==name);const b=document.querySelector(`[data-view="${v}"]`);b.classList.toggle('active',v===name);b.setAttribute('aria-pressed',String(v===name));}
 if(name==='three')requestAnimationFrame(()=>{updateRouteMap();draw();});if(name==='console')renderLogs(true);if(name==='telemetry')renderTelemetry(true);
}
for(const b of document.querySelectorAll('[data-view]'))b.addEventListener('click',()=>changeView(b.dataset.view));
for(const b of document.querySelectorAll('[data-goto]'))b.addEventListener('click',()=>changeView(b.dataset.goto));
function status(id,on){el(id+'Lamp').classList.toggle('on',on);}
function show(){if(!view)return;const s=view.snapshot||{},p=view.pose||{x:0,y:0,theta:0,distanceMm:0};const age=view.lastSeen?Date.now()-view.lastSeen:Infinity;
 const live=view.mode==='LIVE'&&view.frames>0&&age<5000;el('mode').textContent=live?'● LIVE ROBOT':view.mode==='SIMULATED'?'● SIMULATION / NOT HARDWARE':view.serialBarcode?'● USB SERIAL (NO MQTT)':'NO DATA';el('mode').classList.toggle('live',live);
 el('distance').innerHTML=view.lastMotionSeq==null?'— <em>m</em>':`${num(p.distanceMm/1000,2)} <em>m</em>`;
 el('heading').innerHTML=view.lastMotionSeq==null?'— <em>°</em>':`${num(((p.theta*180/Math.PI)%360+360)%360,1)} <em>°</em>`;
 el('humpmax').innerHTML=s.maxHumpMm==null?'— <em>mm</em>':`${s.maxHumpMm} <em>mm</em>`;
el('mission').textContent=(s.sourceMask&16)?missionNames[s.mission]||'UNKNOWN':'UNREPORTED';
 el('leftWheel').textContent=s.sourceMask&1?`${s.speedLeft} mm/s`:'— mm/s';el('rightWheel').textContent=s.sourceMask&1?`${s.speedRight} mm/s`:'— mm/s';
 el('lineState').textContent=live&&(s.sourceMask&2)?lineNames[s.line]||'UNKNOWN':'UNAVAILABLE';el('barcode').textContent=s.barcode|| (view.serialBarcode?.letter?`${view.serialBarcode.letter} (USB serial)`:'—');el('action').textContent=s.action==null?'—':actions[s.action];
 [...el('ir').children].forEach((e,i)=>e.classList.toggle('active',live&&!!(s.sourceMask&2)&&!!((s.ir||0)&(1<<i))));
 el('routeMapping').textContent=live?(view.robotStatus?.mappingLoaded?'LOADED':'NOT LOADED / UNCONFIRMED'):'UNKNOWN · ROBOT OFFLINE';el('startReadiness').textContent=live?(view.robotStatus?.ready?'READY':
   'NOT READY · '+readyDetail(view.robotStatus)):'UNKNOWN · ROBOT OFFLINE';
 el('startReadiness').title=live&&!view.robotStatus?.ready?readyDetail(view.robotStatus):'';
 el('hump').textContent=s.humpMm==null?'—':s.humpMm+' mm';el('fault').textContent=s.sourceMask&16?'0x'+s.faults.toString(16).padStart(2,'0'):'—';
 el('gaps').textContent=`${view.odomGaps} / ${view.uncertain?'YES':'NO'}`;el('drops').textContent=`${view.badFrames} / ${view.dropped}`;
 el('freshness').textContent=s.sourceMask===undefined?'—':s.staleMask?`${s.staleMask.toString(2)} (STALE)`:'RECEIVED SOURCES CURRENT';
 const lastScan=view.scanOutcomes?.[0];
 el('overviewLastScan').textContent=lastScan?
   (lastScan.status==='failed'?`Failed · ${scanFailureNames[lastScan.failureStatus]||'Unknown reason'} (code ${lastScan.failureStatus??'?'})`:
    lastScan.letter?`Decoded ${lastScan.letter}`:'Scan completed; letter unavailable'):'No scan result reported';
 el('overviewLastScan').className=lastScan?`scanResultText ${lastScan.status}`:'scanResultText';
 const robotLive=view.mode==='LIVE' && age<5000;
 const sensorLag=view.sensor?.received?Date.now()-view.sensor.received:Infinity;
 const mqttSensorLive=robotLive && sensorLag<4000;
 const overviewHealth=[
   ['overviewRobotState',robotLive?'Connected':age<10000&&view.lastSeen?'Stale':'Offline',robotLive?'good':'neutral'],
   ['overviewMappingState',!robotLive?'Unknown':view.robotStatus?.mappingLoaded?'Loaded':'Not loaded',robotLive&&view.robotStatus?.mappingLoaded?'good':robotLive?'warn':'neutral'],
   ['overviewReadyState',!robotLive?'Unknown':view.robotStatus?.ready?'Ready':readyDetail(view.robotStatus),robotLive&&view.robotStatus?.ready?'good':robotLive?'warn':'neutral'],
   ['overviewSensorState',mqttSensorLive?`Live · ${num(sensorLag/1000)}s ago`:robotLive?'No fresh measurements':'Offline',mqttSensorLive?'good':robotLive?'warn':'neutral']
 ];
 for(const [id,label,health] of overviewHealth){const node=el(id);node.textContent=label;node.className='healthValue '+health;}
 const netSensor=mqttSensorLive?
   view.sensor:null;
 const usbSensor=view.serialSensor &&
   Date.now()-view.serialSensor.received<4000?view.serialSensor:null;
 const detail=netSensor||usbSensor;
 const ageSensor=detail?Date.now()-detail.received:Infinity;
 el('sensorSource').textContent=netSensor?'MQTT LIVE':usbSensor?'USB SERIAL':'NO FRESH DATA';
 el('sensorIR').textContent=detail?.ir==null?'UNAVAILABLE':
   `L=${detail.ir&1?1:0} · R=${detail.ir&2?1:0} · B=${detail.ir&4?1:0}`;
 el('sensorTilt').textContent=detail?.rollDeg==null?'UNAVAILABLE':
   `${num(detail.rollDeg)}° / ${num(detail.pitchDeg)}°`;
 el('sensorHeading').textContent=detail?.headingDeg==null?'UNAVAILABLE':
   `${num(detail.headingDeg)}° (uncalibrated heading may drift)`;
 const phaseNames=['LEVEL','CLIMBING','DESCENDING'];
 el('sensorPhase').textContent=detail?.humpPhase==null?'UNAVAILABLE':
   phaseNames[detail.humpPhase]||'UNKNOWN';
 el('sensorSonar').textContent='NOT INSTALLED';
 el('sensorObstacle').textContent=view.robotStatus?.mappingLoaded?'LOADED':'NOT LOADED';
 el('sensorEncoders').textContent=detail?.leftEncoder==null?'UNAVAILABLE':
   `${detail.leftEncoder} / ${detail.rightEncoder}`;
 el('sensorAge').textContent=detail?`${netSensor?'MQTT':'USB'} · ${num(ageSensor/1000)} s`:
   'NO SENSOR UPDATES';
 renderTelemetry();
 el('frameCount').textContent=`${view.frames} FRAMES`;el('session').textContent=view.session??'—';el('lastSeen').textContent=Number.isFinite(age)?`${num(age/1000)} s`:'—';
 el('robot').textContent=age<3000?(live?'LIVE ROBOT':'SIMULATION'):age<10000?'STALE':'OFFLINE';status('robot',age<3000);
 el('broker').textContent=view.brokerConnected?'CONNECTED':'OFFLINE';status('broker',!!view.brokerConnected);
 el('stream').textContent=connected?'CONNECTED':'OFFLINE';status('stream',connected);
 const ack=view.acks?.[0];if(ack&&ack.received>window.lastAckTime){window.lastAckTime=ack.received;el('cmdStatus').textContent=`${commandLabels[ack.command]||'COMMAND'} · Transaction ${ack.transaction}: ${ack.result===0?'ACK ACCEPTED':'REJECTED: '+({2:'Mapping not loaded or robot already running; configure mapping first',5:'Safety readiness check failed (IR/IMU/controller)',
   7:'Command expired before execution; request never acted on motors'}[ack.result]||'code '+ack.result)} (ACK is not physical actuator confirmation).`;if(ack.command===1)mappingFeedback(ack.result===0?'Robot ACK: route mapping accepted.':'Robot rejected route mapping (code '+ack.result+').',ack.result!==0);}
 const failed=view.commands?.recent?.[0];if(failed&&['TIMED_OUT','PUBLISH_FAILED'].includes(failed.status)&&failed.sentAt>window.lastCommandIssue){window.lastCommandIssue=failed.sentAt;el('cmdStatus').textContent=`${failed.kind} #${failed.transaction}: ${failed.status}. No robot completion confirmed.`;}
 el('threeMotion').textContent=(s.sourceMask&1)&&!(s.staleMask&1)&&age<3000?'Wheel odometry live':'UNAVAILABLE / STALE';el('threeLeft').textContent=(s.sourceMask&1)?s.speedLeft+' mm/s':'—';el('threeRight').textContent=(s.sourceMask&1)?s.speedRight+' mm/s':'—';
 el('threeHump').textContent=(s.sourceMask&4)?(s.humpMm==null?'UNAVAILABLE':s.humpMm+' mm'):'—';el('threeHeading').textContent=view.lastMotionSeq==null?'UNAVAILABLE':num(p.theta*180/Math.PI,1)+'° (odometry)';el('threeLine').textContent=(s.sourceMask&2)?lineNames[s.line]:'UNAVAILABLE';
 el('threeTilt').textContent=detail?.rollDeg==null?'UNAVAILABLE':
   `${num(detail.rollDeg)}° / ${num(detail.pitchDeg)}°`;
 el('threeSonar').textContent=view.robotStatus?.mappingLoaded?'LOADED':'NOT LOADED';
 window.G7Car3D?.setTelemetry(age<3000?s:{...s,sourceMask:0},p,detail);
 updateRouteMap();
 if(!logPaused)renderLogs();el('consoleBadge').textContent=monitorOnlyLogs().length;renderCommandLedger();
}
function updateRouteMap(){
 if(!view)return;
 const age=Date.now()-(view.lastSeen||0),isLive=view.mode==='LIVE'&&age<5000;
 const hasOdom=view.lastMotionSeq!=null;
 const odomValid=hasOdom && (view.points||[]).length>1 && (view.pose?.distanceMm||0)>1;
 const p=view.pose||{x:0,y:0,theta:0,distanceMm:0};
 el('routeLiveStatus').textContent=!isLive?'WAITING FOR LIVE ROBOT':
   odomValid?(view.uncertain?'LIVE · ESTIMATED PATH (GAPS)':'LIVE · ODOMETRY PATH'):
   hasOdom?'LIVE ROBOT · WHEELS NOT MOVING':'LIVE ROBOT · NO WHEEL ODOMETRY';
 el('routeLivePose').textContent=odomValid?
   `Relative x ${num(p.x/1000,2)} m · y ${num(p.y/1000,2)} m · ${num(p.distanceMm/1000,2)} m travelled`:
   hasOdom?'No displacement yet — route begins when wheel movement is measured':'Position unknown — no incremental wheel-motion packets';
 el('routeEmpty').classList.toggle('hidden',odomValid);
 const report=view.routeMapping,loaded=!!view.robotStatus?.mappingLoaded;
 const confirmed=report?.state==='ACCEPTED'&&report.session===view.session&&loaded&&isLive;
 let mapStatus='NOT CONFIRMED';
 if(report?.state==='PENDING')mapStatus='SENT — AWAITING ACK';
 else if(report?.state==='REJECTED')mapStatus='REJECTED BY ROBOT';
 else if(report?.state==='TIMED_OUT')mapStatus='ACK TIMED OUT';
 else if(report?.state==='INVALIDATED')mapStatus='NO LONGER LOADED';
 else if(report?.state==='ACCEPTED'&&loaded)mapStatus='ROBOT ACK ACCEPTED';
 else if(loaded)mapStatus='ROBOT REPORTS LOADED · LETTERS UNKNOWN';
 el('robotMappingStatus').textContent=mapStatus;
 const pairs=['Straight','Left','Right','U-turn'];
 const chips=report?.letters?.length===4 && ['PENDING','ACCEPTED'].includes(report.state)?
   pairs.map((name,i)=>`<span class="routeChip"><b>${safe(report.letters[i])}</b><span>${name}</span></span>`).join(''):
   '<span class="routeNotKnown">No known four-letter assignment for this session. Open Route mapping to send it.</span>';
 el('robotMappingInMap').innerHTML=chips;
 el('robotMappingInDialog').textContent=confirmed?
   report.letters.map((letter,i)=>`${letter} = ${pairs[i]}`).join(' · '):
   report?.state==='PENDING'?'Sent; waiting for the robot ACK.':
   loaded?'Robot reports a mapping loaded, but has not sent the letters back. Send mapping from this dashboard to confirm them.':
   'No robot-confirmed mapping from this session.';
 const barcodes=(view.events||[]).filter(e=>e.kind===1 && Number.isInteger(e.value)&&e.value>=0&&e.value<26);
 el('routeBarcodeCount').textContent=`${barcodes.length} recorded${barcodes.length>=200?' (limited)':''}`;
 const detected=barcodes.slice(0,24);
 el('routeDetections').innerHTML=detected.length?detected.map(e=>{
   const letter=String.fromCharCode(65+e.value),index=confirmed?report.letters.indexOf(letter):-1;
   const action=index<0?'No matched action':pairs[index];
   const location=e.positionValid&&odomValid?`At ~(${num(e.x/1000,2)}, ${num(e.y/1000,2)}) m`:'Position unavailable';
   return `<div class="routeDetection"><b>${letter}</b><span>${safe(action)}</span><small>${safe(location)}</small></div>`;
 }).join(''):'<span class="routeNotKnown">No barcode detected yet. The robot only reports letters after it physically decodes their symbols.</span>';
 el('routeEstimateHelp').textContent=odomValid?
   (view.uncertain?'Path is approximate; missing or inconsistent odometry frames were detected.':'Estimated route from received wheel-motion deltas; not a camera map or SLAM.'):
   hasOdom?'Odometry is arriving but reports no travelled distance. Raise the wheels before physical movement tests.':'The dashboard cannot reconstruct a physical route from IR/IMU readings alone. Check encoder motion and ODOM MQTT packets.';
}
function draw(){if(activeView!=='three')return;const c=el('map'),r=c.getBoundingClientRect();if(r.width<1||r.height<1)return;
 const dpr=Math.min(devicePixelRatio||1,2),w=Math.round(r.width*dpr),h=Math.round(r.height*dpr);if(c.width!==w||c.height!==h){c.width=w;c.height=h;}
 const g=c.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,r.width,r.height);
 const points=view?.points||[{x:0,y:0}],p=view?.pose||{x:0,y:0,theta:0};
 const xs=points.map(v=>v.x),ys=points.map(v=>v.y),midX=(Math.min(...xs)+Math.max(...xs))/2,midY=(Math.min(...ys)+Math.max(...ys))/2;
 const range=Math.max(1200,Math.max(...xs)-Math.min(...xs),Math.max(...ys)-Math.min(...ys)),scale=(Math.min(r.width,r.height)-85)/range*zoom;
 const px=x=>r.width/2+(x-midX)*scale+offset.x,py=y=>r.height/2-(y-midY)*scale+offset.y;
 g.strokeStyle='#9C978755';g.lineWidth=1;
 for(let x=-70;x<=70;x++){const v=px(x*200);if(v>=0&&v<=r.width){g.beginPath();g.moveTo(v,0);g.lineTo(v,r.height);g.stroke();}}
 for(let y=-70;y<=70;y++){const v=py(y*200);if(v>=0&&v<=r.height){g.beginPath();g.moveTo(0,v);g.lineTo(r.width,v);g.stroke();}}
 g.lineCap='round';g.lineJoin='round';g.lineWidth=3;
 for(let i=1;i<points.length;i++){g.setLineDash(points[i].uncertain?[4,6]:[]);g.strokeStyle=points[i].uncertain?'#C49B3B':'#314660';g.beginPath();g.moveTo(px(points[i-1].x),py(points[i-1].y));g.lineTo(px(points[i].x),py(points[i].y));g.stroke();}g.setLineDash([]);
 for(const e of (view?.events||[]).filter(e=>e.positionValid && view?.lastMotionSeq!=null)){g.fillStyle=colors[e.kind]||'#314660';g.beginPath();g.arc(px(e.x),py(e.y),5,0,Math.PI*2);g.fill();g.strokeStyle='#FBF8F1';g.lineWidth=2;g.stroke();}
 if(view?.lastMotionSeq!=null){g.save();g.translate(px(p.x),py(p.y));g.rotate(-p.theta);g.fillStyle='#C49B3B';g.shadowColor='#314660';g.shadowBlur=14;g.beginPath();g.moveTo(15,0);g.lineTo(-11,-9);g.lineTo(-6,0);g.lineTo(-11,9);g.closePath();g.fill();g.restore();}
 g.fillStyle='#314660';g.beginPath();g.arc(px(0),py(0),3,0,Math.PI*2);g.fill();el('mapScale').textContent=`Grid: 0.2 m · ${num(zoom)}×`;
}
for(const [id,initial] of [['straight','A'],['left','B'],['right','C'],['uturn','D']]){
 const select=el(id);for(const letter of 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'){const option=document.createElement('option');option.textContent=letter;option.value=letter;select.appendChild(option);}select.value=initial;
}
let networkData=null;
async function refreshNetwork(){
 try{const res=await fetch('/api/network',{cache:'no-store'});if(!res.ok)throw Error('HTTP '+res.status);
   networkData=await res.json();
   el('netIp').textContent=networkData.ip?`${networkData.ip} (${networkData.networkInterface})`:'No private LAN interface detected';
   el('netUrl').textContent=networkData.ip&&networkData.webLanEnabled?`http://${networkData.ip}:${networkData.browserPort}`:'Not exposed; run npm run lan';
   el('netBroker').textContent=networkData.mqttLanEnabled?((networkData.brokerTarget||'—')+':'+networkData.brokerPort):'Not exposed; run npm run lan';
   el('netGateway').textContent=networkData.brokerReachableFromGateway?'CONNECTED':'DISCONNECTED';
   el('netReal').textContent=networkData.robotStatus;
   el('netSim').textContent=networkData.simulatorStatus;
   el('netAge').textContent=networkData.robotLastPacket?`${((Date.now()-networkData.robotLastPacket)/1000).toFixed(1)} seconds ago`:'No real packets observed';
 }catch(e){el('netReal').textContent='Diagnostics unavailable: '+e.message;}
}
el('networkDetails').onclick=()=>{el('connectionDialog').showModal();refreshNetwork();};
el('refreshNetwork').onclick=refreshNetwork;
setInterval(()=>{if(el('connectionDialog').open)refreshNetwork();},1500);
const dialog=el('mappingDialog');
function mappingFeedback(message='',bad=false){
 const line=el('mappingFeedback');line.textContent=message;line.classList.toggle('error',bad);
}
function updateMappingMode(){
 const enabled=!!view?.connection?.adminCommandsEnabled;
 el('mappingMode').textContent=enabled?
   'LIVE CONTROL ENABLED. Motors must be raised for the first Start/Stop test.':
   'READ-ONLY MODE: Stop npm run lan and launch bash run_live_dashboard.sh from the project root before sending mapping.';
 el('mappingMode').classList.toggle('error',!enabled);
}
el('configure').onclick=()=>{mappingFeedback();updateMappingMode();updateRouteMap();dialog.showModal();};
el('overviewMappingButton').onclick=()=>el('configure').click();
async function command(kind){const letters=['straight','left','right','uturn'].map(id=>el(id).value);
 if(kind==='RECALIBRATE_IMU'&&!confirm('IMU recalibration requires the robot to be completely stationary. Continue?'))return;
 el('cmdStatus').textContent=`${kind}: submitting request…`;
 try{const res=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json','X-Admin-Token':el('token').value},body:JSON.stringify({kind,letters:kind==='SET_MAPPING'?letters:[]})});
 const text=await res.text();let data;try{data=JSON.parse(text);}catch{data=null;}
 if(!res.ok)throw Error(data?.error||text||'HTTP '+res.status);
 if(!data)throw Error('Invalid command response');
 el('cmdStatus').textContent=`${kind} #${data.transaction} sent to ${data.target}; awaiting robot ACK. This is not physical confirmation.`;
 if(kind==='SET_MAPPING'){mappingFeedback('Mapping #'+data.transaction+' sent; waiting for robot ACK.');}
 }catch(e){el('cmdStatus').textContent=`${kind} NOT SENT: ${e.message}`;if(kind==='SET_MAPPING'){mappingFeedback('Mapping NOT SENT: '+e.message,true);updateMappingMode();}else changeView('console');}
}
el('start').onclick=()=>{
  if(view?.mode==='LIVE' && !view.robotStatus?.mappingLoaded){
    el('cmdStatus').textContent='START needs a route mapping first. Choose four real barcode letters and send mapping.';
    updateMappingMode();mappingFeedback('Choose the four physical track letters first.');dialog.showModal();return;
  }
  command('START');
};el('stop').onclick=()=>command('STOP');el('recalibrate').onclick=()=>command('RECALIBRATE_IMU');el('snapshot').onclick=()=>command('SNAPSHOT');
el('setMapping').onclick=()=>command('SET_MAPPING');
el('mapThenStart').onclick=async()=>{
  const letters=['straight','left','right','uturn'].map(id=>el(id).value);
  if(new Set(letters).size!==4){el('cmdStatus').textContent='Mapping needs four different letters.';return;}
  mappingFeedback('Sending mapping; waiting for broker confirmation…');
  el('cmdStatus').textContent='Applying route mapping. Waiting for robot ACK before Start…';
  try{
    const response=await fetch('/api/map-then-start',{method:'POST',
      headers:{'Content-Type':'application/json','X-Admin-Token':el('token').value},
      body:JSON.stringify({letters})});
    const result=await response.json();if(!response.ok)throw Error(result.error||response.status);
    el('cmdStatus').textContent=result.message;mappingFeedback(result.message);
  }catch(error){el('cmdStatus').textContent='Mapping/Start not completed: '+error.message;mappingFeedback(error.message,true);updateMappingMode();}
};
el('zoomIn').onclick=()=>{zoom=Math.min(12,zoom*1.3);draw();};el('zoomOut').onclick=()=>{zoom=Math.max(.15,zoom/1.3);draw();};el('recenter').onclick=()=>{zoom=1;offset={x:0,y:0};draw();};
const map=el('map');map.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];map.setPointerCapture(e.pointerId);});map.addEventListener('pointermove',e=>{if(!drag)return;offset.x+=e.clientX-drag[0];offset.y+=e.clientY-drag[1];drag=[e.clientX,e.clientY];draw();});map.addEventListener('pointerup',()=>drag=null);
map.addEventListener('wheel',e=>{e.preventDefault();zoom*=e.deltaY<0?1.12:1/1.12;draw();},{passive:false});
el('resetCamera').onclick=()=>window.G7Car3D?.resetCamera();
function updateScannerPanels(){
 if(!view)return;
 const scans=view.scanOutcomes||[],last=scans[0];
 const spot=el('telemetryScanSpotlight'),title=el('telemetryScanTitle'),detail=el('telemetryScanDetail');
 const monitor=el('monitorScanSpotlight');
 const age=last?Math.max(0,Math.round((Date.now()-last.at)/1000)):null;
 spot.className='scanSpotlight '+(last?`scanSpotlight-${last.status}`:'scanSpotlight-idle');
 monitor.className='monitorScanNote '+(last?`monitorScanNote-${last.status}`:'');
 const heading=!last?'No scan result received':last.status==='failed'?'SCAN FAILED':
   last.letter?`LETTER ${last.letter} DECODED`:'SCAN COMPLETED · LETTER NOT RECEIVED';
 const explanation=!last?'Waiting for a completed or rejected scan from the robot.':
   `${last.detail} · ${age}s ago · ${last.source==='LIVE'?'Live MQTT':'Simulation'}${age>10?' · historical result':''}`;
 title.textContent=heading;detail.textContent=explanation;
 el('monitorScanTitle').textContent=!last?'Scanner results: none yet':heading;
 el('monitorScanDetail').textContent=explanation;
 el('telemetryScanHistory').innerHTML=scans.length?scans.slice(0,8).map(x=>
   `<div class="scanHistoryRow ${safe(x.status)}"><span>${x.status==='failed'?'✕':'✓'}</span><strong>${safe(x.detail)}</strong><time>${new Date(x.at).toLocaleTimeString()}</time></div>`).join(''):
   '<span class="emptyScan">No scan results reported this session.</span>';
}
function renderTelemetry(force=false){
 if(activeView!=='telemetry'&&!force||!view)return;
 const status=view.robotStatus;const last=view.sensor;
 const packetAge=view.lastSeen?Math.max(0,Math.floor((Date.now()-view.lastSeen)/1000)):null;
 el('telemetryConnection').textContent=view.mode==='LIVE'&&packetAge!==null&&packetAge<5?
   '● LIVE MQTT STREAM':packetAge===null?'● NO MQTT PACKETS YET':
   `● STREAM PAUSED · ${packetAge}s since packet`;
 el('telemetryConnection').className=packetAge!==null&&packetAge<5?'streamFresh':'streamStale';
 updateScannerPanels();
 el('telemetryMapping').textContent='Mapping: '+(status?.mappingLoaded?'LOADED':'NOT LOADED / UNKNOWN');
 el('telemetryReadiness').textContent='Controller ready: '+(status?.ready?'YES':'NO / UNKNOWN');
 el('telemetryTimestamp').textContent='Last sensor: '+(last?.received?`${new Date(last.received).toLocaleTimeString()} (${Math.floor((Date.now()-last.received)/1000)}s ago)`:'never');
 const motor=(status?.pwmLeft==null)?'Awaiting V8 motor status':
   `Motor command L=${status.pwmLeft}, R=${status.pwmRight} PWM · Barcode hold: ${status.barcodeMotorHold?'YES':'NO'}`;
 el('telemetryMotor').textContent=motor;
 const bc=view.barcodeDiag,bcLive=bc?.received&&Date.now()-bc.received<5000;
 if(bcLive){
   el('verifyBarcode').textContent=['Disabled','Calibration required','Scanner armed'][bc.mode]||`Mode ${bc.mode}`;
   el('verifyBarcodeDetail').textContent=`Digital sensor=${bc.raw}, black/white edges=${bc.rawTransitions}, capture state=${bc.scanState}, runs=${bc.runCount}, last width=${bc.lastRunUnits} counts. The edge counter must rise during a scan.`;
   el('verifyBarcodeCounters').textContent=`Completed ${bc.completed} · Rejected ${bc.rejected} · Last failure code ${bc.failureStatus}`;
 }else{
   el('verifyBarcode').textContent=bc?.received?'Scanner diagnostics paused':'No scanner diagnostic packets yet';
   el('verifyBarcodeDetail').textContent=bc?.received?
     `Last scanner update ${Math.floor((Date.now()-bc.received)/1000)}s ago. Waiting for fresh robot packets; these are historical counters.`:
     'V7.5 firmware must publish barcode diagnostic packets over MQTT. No USB monitor is required.';
   el('verifyBarcodeCounters').textContent=bc?.received?`Last totals: completed ${bc.completed} · rejected ${bc.rejected}`:'Completed — · Rejected —';
 }
 const sensorLive=last?.received&&Date.now()-last.received<5000;
 const encLive=sensorLive&&last.leftEncoder!=null&&last.rightEncoder!=null;
 el('verifyOdom').textContent=encLive?`Encoders: L=${last.leftEncoder}, R=${last.rightEncoder}`:'No fresh encoder measurements';
 const odomType=last?.odometryConfidence===2?'MEASURED SCALE':last?.odometryConfidence===1?'PROVISIONAL SCALE':'NOT CONFIGURED';
 el('verifyOdomDetail').textContent=encLive?`Odometry: ${odomType}. Turn each wheel by hand and check its count changes. Nominal distance and map coordinates are NOT verified.`:'Wheel counts unavailable. Confirm motion task and sensor packet freshness.';
 el('verifyCompass').textContent=sensorLive&&Number.isFinite(last.headingDeg)?`Magnetic heading ${last.headingDeg.toFixed(1)}°`:'Magnetic heading unavailable';
 el('verifyCompassDetail').textContent=sensorLive&&Number.isFinite(last.headingDeg)?'Rotate robot slowly with motors OFF; heading should change. Level heading is not tilt-compensated or calibrated to true north.':'Check LSM303 magnetometer power/I²C and the heading-valid flag; do not confuse IMU roll/pitch with compass validity.';
 const rows=(view.logs||[]).filter(x=>x.category==='telemetry'||x.category==='barcode'||x.category==='imu'||x.category==='movement').slice(0,150);
 const fp=`${rows[0]?.id||0}:${rows.length}:${view.robotStatus?.mappingLoaded}`;
 if(!force && fp===telemetryUpdated)return;telemetryUpdated=fp;
 el('telemetryOutput').innerHTML=rows.length?rows.map(l=>
   `<div class="logline ${safe(l.level)} ${l.category==='barcode'?(l.level==='warn'?'scan-failure':'scan-success'):''}"><time>${new Date(l.time).toLocaleTimeString()}</time><span class="level">${safe(l.level).toUpperCase()}</span><span class="category">${safe(l.category)}</span><span class="message">${safe(l.message)}</span></div>`).join(''):
   '<div class="placeholder">No binary MQTT sensor frames yet. No VS Code Serial Monitor is needed for this view.</div>';
}
el('copyTelemetry').onclick=async()=>{
 const value=(view?.logs||[]).filter(x=>x.category==='telemetry'||x.category==='barcode'||x.category==='imu'||x.category==='movement')
  .map(x=>new Date(x.time).toISOString()+' ['+x.category+'] '+x.message).join('\n');
 try{await navigator.clipboard.writeText(value);}catch{}
};
function monitorOnlyLogs(){return (view?.logs||[]).filter(l=>
 ['commands','mission','network','scanner'].includes(l.category)&&
 !/^Ignored duplicate|^Wire sequence gap|^Firmware dropped|^Rejected binary telemetry|^Mission state · mission · value 0$/.test(l.message));}
function renderCommandLedger(){
 if(!view)return;
 const entries=[...(view.commands?.pending||[]),...(view.commands?.recent||[])]
  .sort((a,b)=>b.sentAt-a.sentAt).slice(0,20);
 const signature=entries.map(e=>e.transaction+':'+e.status+':'+e.result).join('|');
 if(signature===ledgerUpdated)return;ledgerUpdated=signature;
 const reasons={1:'Invalid command',2:'Bad state — mapping missing or running',
   3:'Invalid mapping',4:'Duplicate request',5:'Controller not ready',
   6:'Pending',7:'Command expired before robot could act'};
 el('commandLedgerOutput').innerHTML=entries.length?entries.map(e=>
  `<div class="ledgerRow"><time>${new Date(e.sentAt).toLocaleTimeString()}</time>`+
  `<b>${safe(e.kind)} #${e.transaction}</b>`+
  `<span class="ledgerState ${safe(e.status.toLowerCase())}">${safe(e.status)}</span>`+
  `<small>${safe(e.status==='REJECTED'?(reasons[e.result]||'Code '+e.result):
    e.status==='ACCEPTED'?'ACK received from robot (not physical movement)':
    e.status==='SENT'?'Awaiting robot ACK':e.status==='TIMED_OUT'?'No ACK received':'Publish failed')}</small></div>`).join(''):
  '<div class="placeholder">No commands sent yet. Send mapping to see the request and its ACK here.</div>';
}
function filteredLogs(){const logs=logPaused?pausedLogs:monitorOnlyLogs();const cat=el('logFilter').value,query=el('logSearch').value.trim().toLowerCase();
 return logs.filter(l=>l.id>lastClearedLogId&&(cat==='all'||l.category===cat)&&(el('severityFilter').value==='all'||l.level===el('severityFilter').value)&&(!query||[l.category,l.message,l.level].join(' ').toLowerCase().includes(query)));}
function renderLogs(force=false){if(activeView!=='console'&&!force)return;updateScannerPanels();const logs=filteredLogs();el('logCount').textContent=`${logs.length} / ${logPaused?pausedLogs.length:monitorOnlyLogs().length} command/system messages`;
 const fingerprint=logs.length+':'+(logs[0]?.id||0)+':'+el('logFilter').value+':'+el('logSearch').value+':'+el('severityFilter').value+':'+lastClearedLogId+':'+logPaused;
 if(!force&&fingerprint===logUpdated)return;logUpdated=fingerprint;
 el('consoleOutput').innerHTML=logs.length?logs.slice(0,220).map(l=>`<div class="logline ${safe(l.level)} ${l.category==='scanner'?(l.level==='warn'?'scan-failure':'scan-success'):''}"><time>${new Date(l.time).toLocaleTimeString()}</time><span class="level">${safe(l.level).toUpperCase()}</span><span class="category">${safe(l.category)}</span><span class="message">${safe(l.message)}</span></div>`).join(''):'<div class="placeholder">No messages match this filter.</div>';
}
el('logFilter').onchange=()=>renderLogs(true);el('severityFilter').onchange=()=>renderLogs(true);el('logSearch').oninput=()=>renderLogs(true);
el('clearConsole').onclick=()=>{lastClearedLogId=(monitorOnlyLogs()[0]?.id||lastClearedLogId);renderLogs(true);};
el('pauseConsole').onclick=()=>{logPaused=!logPaused;if(logPaused)pausedLogs=[...monitorOnlyLogs()];el('pauseConsole').textContent=logPaused?'Resume view':'Pause view';renderLogs(true);};
async function downloadRecording(ext){
 try{const res=await fetch('/api/recording'+(ext==='csv'?'.csv':''),{headers:{'X-Admin-Token':el('token').value}});
 if(!res.ok)throw Error('HTTP '+res.status);const body=await res.blob();const href=URL.createObjectURL(body),anchor=document.createElement('a');
 anchor.href=href;anchor.download='g7-session-'+(view?.recording?.id||'recording')+'.'+ext;anchor.click();setTimeout(()=>URL.revokeObjectURL(href),1000);
 }catch(e){el('cmdStatus').textContent='Export unavailable: '+e.message;changeView('console');}
}
el('downloadJsonl').onclick=()=>downloadRecording('jsonl');el('downloadCsv').onclick=()=>downloadRecording('csv');
el('copyConsole').onclick=async()=>{const text=filteredLogs().map(l=>`${new Date(l.time).toISOString()} [${l.level.toUpperCase()}] ${l.category}: ${l.message}`).join('\n');try{await navigator.clipboard.writeText(text);el('copyConsole').textContent='Copied ✓';setTimeout(()=>el('copyConsole').textContent='Copy log',1600);}catch{el('copyConsole').textContent='Copy unavailable';}};
const stream=new EventSource('/api/events');stream.addEventListener('open',()=>{connected=true;show();});stream.addEventListener('error',()=>{connected=false;show();});stream.addEventListener('state',e=>{try{view=JSON.parse(e.data);show();draw();}catch(err){console.error('bad dashboard stream',err);}});
setInterval(()=>{el('time').textContent=new Date().toLocaleString();show();if(activeView==='console')updateScannerPanels();},1000);window.addEventListener('resize',draw);draw();
