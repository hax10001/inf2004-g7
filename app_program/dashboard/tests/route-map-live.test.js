'use strict';
const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {RobotState}=require('../server/state');
const {frame,TYPES}=require('../server/protocol');
const f=(type,payload,session,seq,ms=1000)=>frame(type,Buffer.from(payload),{session,seq,ms});
test('live map only reports ACK-accepted route assignments, not speculative selections',()=>{
 const s=new RobotState();
 s.ingest(f(TYPES.STATUS,[0,0,0,0,0],9,1));
 s.mappingRequested(['Q','R','S','T'],22);
 assert.equal(s.view().routeMapping.state,'PENDING');
 assert.equal(s.mappingResult(23,0),false);
 assert.equal(s.mappingResult(22,3),true);
 assert.equal(s.view().routeMapping.state,'REJECTED');
 s.mappingRequested(['A','B','C','D'],23);
 assert.equal(s.mappingResult(23,0),true);
 assert.equal(s.view().routeMapping.state,'ACCEPTED');
 assert.deepEqual(s.view().routeMapping.letters,['A','B','C','D']);
 s.ingest(f(TYPES.STATUS,[1,1,0,0,1],9,2));
 assert.equal(s.view().robotStatus.mappingLoaded,true);
 assert.equal(s.view().routeMapping.state,'ACCEPTED');
});
test('robot reboot clears the old route confirmation and physical position',()=>{
 const s=new RobotState();s.ingest(f(TYPES.STATUS,[1,1,0,0,1],9,1));
 s.mappingRequested(['A','B','C','D'],23);s.mappingResult(23,0);
 const od=Buffer.alloc(6);od.writeInt16LE(50,0);od.writeInt16LE(50,2);od.writeUInt16LE(1,4);
 s.ingest(f(TYPES.ODOM,od,9,2));assert.equal(s.view().pose.distanceMm,50);
 s.ingest(f(TYPES.STATUS,[0,0,0,0,0],10,1));
 assert.equal(s.view().routeMapping,null);assert.equal(s.view().pose.distanceMm,0);
 assert.equal(s.view().events.length,0);
});
test('barcodes record detection order but remain position-invalid without odometry',()=>{
 const s=new RobotState();s.ingest(f(TYPES.STATUS,[1,1,0,0,0],11,1));
 const bar=(id,value)=>[2,1,id&255,id>>8,value,0];
 s.ingest(f(TYPES.EVENT,bar(1,0),11,2,2000));
 s.ingest(f(TYPES.EVENT,bar(2,25),11,3,3000));
 assert.equal(s.view().events[0].value,25);
 assert.equal(s.view().events[0].positionValid,false);
 assert.equal(s.view().points.length,1);
});
test('the map visibly separates robot mapping, observed barcodes, and estimated route',()=>{
 const html=fs.readFileSync(path.join(__dirname,'../public/index.html'),'utf8');
 const script=fs.readFileSync(path.join(__dirname,'../public/app.js'),'utf8');
 for(const id of ['robotMappingStatus','robotMappingInMap','robotMappingInDialog','routeDetections','routeEmpty','routeLiveStatus','routeLivePose']){
  assert.match(html,new RegExp(`id="${id}"`));
 }
 assert.match(script,/routeMapping\.state|report\?\.state/);
 assert.match(script,/e\.positionValid/);
 assert.match(script,/Position unknown|No displacement yet/);
});
