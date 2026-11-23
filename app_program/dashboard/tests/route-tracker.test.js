'use strict';
/* FR14: dead-reckoned route, corrected at junction turns, with a marker
 * at the pose of each event. Frames are built as the robot sends them:
 * EVENT frames go ahead of queued ODOM, so arrival order is not time
 * order. */
const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {RobotState}=require('../server/state');
const {frame,TYPES}=require('../server/protocol');

function robot(){
  const s=new RobotState();let seq=0,odom=0,ms=1000;
  const send=(type,payload,t)=>s.ingest(frame(type,Buffer.from(payload),{session:5,seq:++seq,ms:t}));
  const api={s,
    get ms(){return ms;},
    /* Wheel deltas in mm over one 50 ms period. */
    wheels(l,r,{skip=0}={}){odom+=1+skip;ms+=50;const b=Buffer.alloc(6);b.writeInt16LE(l,0);b.writeInt16LE(r,2);b.writeUInt16LE(odom&65535,4);send(TYPES.ODOM,b,ms);},
    straight(mm,step=20){for(let d=0;d<mm;d+=step)api.wheels(step,step);},
    /* Pivot about the axle; 130 mm track: angle = (r-l)/130 rad. */
    pivot(deg,steps=10){const arc=deg*Math.PI/180*130/2;let done=0;for(let i=1;i<=steps;i++){const to=Math.round(arc*i/steps);api.wheels(-(to-done),to-done);done=to;}},
    event(source,kind,value,t=ms){const b=Buffer.alloc(6);b[0]=source;b[1]=kind;b.writeUInt16LE(++seq&65535,2);b.writeUInt16LE(value,4);send(TYPES.EVENT,b,t);},
    checkpoint(mm){const b=Buffer.alloc(4);b.writeUInt32LE(mm,0);send(TYPES.CHECKPOINT,b,ms);}
  };
  send(TYPES.STATUS,[1,1,1,0,2],ms);
  return api;
}
const near=(a,b,tol,msg)=>assert(Math.abs(a-b)<=tol,`${msg}: ${a} vs ${b}`);
const deg=r=>r*180/Math.PI;

test('track width matches the firmware wheel track (APP_NAV_TRACK_MM 130)',()=>{
  const cfg=fs.readFileSync(path.join(__dirname,'../../app/app_config.h'),'utf8');
  const mm=Number(/#define APP_NAV_TRACK_MM (\d+)U/.exec(cfg)[1]);
  assert.equal(new RobotState().trackWidthMm,mm);
  const r=robot();r.pivot(90);
  near(deg(r.s.view().pose.theta),90,0.5,'90 degree pivot reads 90 degrees');
  const old=new RobotState(120);assert.notEqual(old.trackWidthMm,130);
});

test('every event gets a marker at the pose it happened, even after a lost record',()=>{
  const r=robot();
  r.straight(400);
  // Barcode decoded at 200 mm, but its EVENT arrives after 400 mm of ODOM.
  r.event(2,1,16,1000+50*10);
  r.wheels(20,20,{skip:1});              // one ODOM record lost
  r.straight(200);
  r.event(4,4,25);                        // obstacle now
  const v=r.s.view();
  const bar=v.events.find(e=>e.kind===1),obs=v.events.find(e=>e.kind===4);
  assert.equal(bar.positionValid,true);near(bar.x,200,1,'barcode x');
  assert.equal(obs.positionValid,true);near(obs.x,620,25,'obstacle x');
  assert.equal(v.odomGaps,1);
});

test('an event received before the motion that led up to it waits for that motion',()=>{
  const r=robot();r.straight(100);
  r.event(2,1,0,r.ms+100);               // happened 100 ms from now
  r.straight(100);
  const e=r.s.view().events[0];
  assert.equal(e.positionPending,false);near(e.x,140,1,'placed at its own time');
});

test('a lost wheel update dashes one segment; the next checkpoint bridges the distance',()=>{
  const r=robot();r.straight(500);r.checkpoint(500);   // aligns the counts
  r.straight(200);
  r.wheels(20,20,{skip:3});                            // 3 records (60 mm) lost
  assert.equal(r.s.view().uncertain,true);
  r.straight(260);r.checkpoint(1040);
  const v=r.s.view();
  assert.equal(v.uncertain,false,'a checkpoint reconciles the route');
  near(v.pose.distanceMm,1040,1,'distance bridged');near(v.pose.x,1040,1,'bridged along the heading');
  assert.equal(v.points.filter(p=>p.uncertain).length>=1,true);
  assert(v.points.filter(p=>p.uncertain).length<=2,'only the affected segment is approximate');
});

test('frames from another source going missing do not make the route approximate',()=>{
  const r=robot();r.straight(200);
  r.s.ingest(frame(TYPES.HEARTBEAT,Buffer.alloc(4),{session:5,seq:999,ms:r.ms}));
  r.straight(100);
  const v=r.s.view();
  assert.equal(v.uncertain,false);assert.equal(v.odomGaps,0);assert(v.wireGaps>=1);
});

for(const [action,turn,name] of [[1,90,'Left'],[2,-90,'Right']]){
  test(`${name} turn snaps heading to the pre-junction heading ${turn>0?'+':''}${turn}`,()=>{
    const r=robot();
    // Line following with a small wiggle about 0 degrees.
    for(let i=0;i<30;i++)r.wheels(20+(i%2?1:-1),20+(i%2?-1:1));
    r.event(5,13,action);r.straight(134);         // junction, centre the axle
    r.event(5,14,action);
    r.pivot(turn*1.12);                             // encoder pivot overshoots 12 %
    r.event(5,15,action);r.straight(200);
    const v=r.s.view();
    near(deg(v.pose.theta),turn,0.5,'heading snapped');
    const pivot=v.events.find(e=>e.kind===14);
    near(v.pose.x,pivot.x,3,'drives out of the pivot point');
    near(Math.abs(v.pose.y-pivot.y),200,1,'straight out at right angles');
    assert.equal(v.turnCorrections,1);
  });
}

test('U-turn retraces the outbound line back through the junction',()=>{
  const r=robot();
  r.straight(600);r.event(5,13,3);r.straight(134);r.event(5,14,3);
  r.pivot(165);                                     // under-rotated pivot
  r.event(5,15,3);r.straight(734);
  const v=r.s.view();
  near(Math.abs(deg(v.pose.theta)),180,0.5,'heading reversed');
  near(v.pose.y,0,1,'back on the outbound line');near(v.pose.x,0,2,'back at the start');
});

test('turn events that overtake their ODOM are applied at the right time',()=>{
  const r=robot();r.straight(300);
  const t0=r.ms;
  // All three events arrive first (EVENT frames have send priority).
  r.event(5,13,1,t0+10);r.event(5,14,1,t0+20+50*7);r.event(5,15,1,t0+20+50*17);
  r.straight(134);r.pivot(80);r.straight(100);
  const v=r.s.view();
  near(deg(v.pose.theta),90,0.5,'heading');near(v.pose.x,440,2,'pivot point x');
});

test('obstacle bypass stays purely dead-reckoned',()=>{
  const r=robot();r.straight(300);r.event(4,4,20);
  r.pivot(60);r.straight(150);r.pivot(-60);
  const v=r.s.view();
  near(deg(v.pose.theta),0,1,'no snapping without a junction turn');
  assert.equal(v.turnCorrections,0);
});

test('map draws a distinct marker per kind and labels kind 6 correctly',()=>{
  const script=fs.readFileSync(path.join(__dirname,'../public/app.js'),'utf8');
  const html=fs.readFileSync(path.join(__dirname,'../public/index.html'),'utf8');
  for(const k of ['e.kind===1','e.kind===14','e.kind===9','e.kind===4'])assert(script.includes(k),k);
  assert.match(html,/Junction turn/);assert.match(html,/Hump \+ height/);assert.match(html,/Obstacle/);
  const r=robot();r.event(4,6,1);
  assert.equal(r.s.view().events[0].name,'Obstacle bypassed');
  r.event(3,9,14);assert.equal(r.s.view().events[0].name,'Hump crossed');
});
