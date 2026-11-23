'use strict';
const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const pub=n=>fs.readFileSync(path.join(__dirname,'../public',n),'utf8');
const overview=()=>pub('index.html').split('<section id="view-overview"')[1].split('<section id="view-three"')[0];
const telemetry=()=>pub('index.html').split('<section id="view-telemetry"')[1].split('<section id="view-console"')[0];
test('Overview is focused, with no mission event feed or detailed sensor summary',()=>{
 const block=overview();
 for(const title of ['Mission events','Sensor summary','IR, IMU and encoders','eventList','sensorGrid'])assert.doesNotMatch(block,new RegExp(title));
 for(const id of ['overviewRobotState','overviewMappingState','overviewReadyState','overviewSensorState','distance','heading','humpmax','leftWheel','rightWheel','ir','barcode'])assert.match(block,new RegExp(`id="${id}"`));
});
test('Telemetry keeps advanced diagnostic values accessible but collapsed by default',()=>{
 const block=telemetry();assert.match(block,/<details class="advancedDiagnostics" id="advancedDiagnostics">/);
 assert.doesNotMatch(block,/<details[^>]*\bopen\b/);
 for(const id of ['sensorIR','sensorTilt','sensorHeading','sensorEncoders','sensorAge','routeMapping','startReadiness','fault','drops','freshness'])assert.match(block,new RegExp(`id="${id}"`));
});
test('Overview reports stale connection and missing sensor packets separately',()=>{
 const js=pub('app.js');assert.match(js,/const mqttSensorLive=robotLive && sensorLag<4000/);
 assert.match(js,/overviewSensorState/);assert.match(js,/No fresh measurements/);
 assert.match(js,/UNKNOWN · ROBOT OFFLINE/);
 assert.match(js,/overviewMappingButton/);
 assert.doesNotMatch(js,/el\('events'\)/);
});
test('responsive overview fills width and keeps low-value mission-event spam out of command monitor',()=>{
 const css=pub('style.css'),js=pub('app.js');
 assert.match(css,/#view-overview \.overviewgrid\{display:block/);
 assert.match(css,/#view-overview \.overviewgrid \.stack\{display:grid;grid-template-columns:repeat\(2/);
 assert.match(css,/@media\(max-width:760px\)/);
 assert.match(js,/\^Mission state · mission · value 0\$/);
});
