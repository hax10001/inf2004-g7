'use strict';
const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const root=path.join(__dirname,'../public');
const app=fs.readFileSync(path.join(root,'app.js'),'utf8'),css=fs.readFileSync(path.join(root,'style.css'),'utf8');
const server=fs.readFileSync(path.join(__dirname,'../server/index.js'),'utf8');
test('unified console has four named quadrants and reuses existing monitor panels',()=>{
 for(const id of ['pane-sensors','pane-barcode','pane-robotlog','pane-commands'])assert.match(app,new RegExp("'"+id+"'"));
 assert.match(app,/grid\.insertBefore\(grid\.querySelector/);
 assert.match(css,/\.unifiedGrid\{display:grid;grid-template-columns:repeat\(2/);
 assert.match(css,/@media\(max-width:800px\)/);
});
test('live controls and original MQTT streams remain single-owned',()=>{
 assert.match(app,/const commandPanel=el\('view-console'\)/);
 assert.match(app,/const logPanel=el\('view-robotlog'\)/);
 assert.equal((app.match(/new EventSource\(/g)||[]).length,1);
 assert.match(server,/segments\[ns\.length\+1\]==='log'/);
 assert.match(server,/\['telemetry','ack'\]/);
 assert.match(server,/if\(kind==='STOP'\)return 0/);
 assert.match(server,/const LIVE_COMMANDS_ENABLED=env\.LIVE_COMMANDS_ENABLED==='1'/);
});
test('stale readiness and historical scans cannot be presented as live success',()=>{
 assert.match(app,/view\.lastSeen&&Date\.now\(\)-view\.lastSeen<5000/);
 assert.match(app,/bcDiag\?\.received\?'Offline':'Waiting'/);
 assert.match(app,/freshRobot\?\(status\?\.ready\?'ready':'not ready'\):'unavailable'/);
 assert.match(app,/new Date\(x\.at\)\.toLocaleString\(\)/);
});
