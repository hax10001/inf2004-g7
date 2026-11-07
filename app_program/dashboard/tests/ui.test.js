'use strict';
const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const read=name=>fs.readFileSync(path.join(__dirname,'../public',name),'utf8');
test('single navigation bar with Overview, Map, Telemetry Console, Monitor and Robot Log without redundant rail',()=>{
 const html=read('index.html');assert.equal((html.match(/class="navtab(?: active)?"/g)||[]).length,5);
 assert.doesNotMatch(html,/class="rail"/);for(const id of ['telemetryOutput','mapThenStart','start','stop','recalibrate','networkDetails','connectionDialog','netReal','logFilter','consoleOutput','map','car3d'])assert.match(html,new RegExp(`id="${id}"`));
});
test('browser scripts parse (a syntax error leaves the dashboard stuck at CONNECTING)',()=>{
 for(const f of ['app.js','car3d.js','car3d_fallback.js'])assert.doesNotThrow(()=>new (require('node:vm').Script)(read(f),{filename:f}),f);
});
test('3D renderer provides software fallback and stays inside browser',()=>{
 assert.match(read('index.html'),/car3d_fallback\.js/);assert.match(read('car3d.js'),/G7CreateSoftware3D/);
 assert.match(read('car3d_fallback.js'),/function project/);
});
test('theme and controls use one spacing system across monitoring views',()=>{
 const css=read('style.css'),html=read('index.html');
 for(const token of ['--navy: #182B42','--offwhite: #FFFEFB','--mustard: #C49B3B','--beige: #F0EADF','--panel-pad: 22px','--control-h: 42px'])assert.ok(css.includes(token),`missing ${token}`);
 assert.match(css,/\.monitorControls\s*\.btn\s*\{[^}]*height:var\(--control-h\)/);
 assert.match(css,/\.monitorControls\s*\{[^}]*margin:0 var\(--panel-pad\)/);
 assert.match(css,/\.consoleFilters\s*\.checkLabel input\[type="checkbox"\]/);
 assert.match(html,/class="btn subtle" id="resetCamera"/);
});
test('single-command simulation is opt-in and scoped to synthetic telemetry',()=>{
 const tools=fs.readFileSync(path.join(__dirname,'../tools/demo.js'),'utf8');
 const rootScript=fs.readFileSync(path.join(__dirname,'../../../demo.sh'),'utf8');
 const pkg=JSON.parse(fs.readFileSync(path.join(__dirname,'../package.json'),'utf8'));
 assert.match(tools,/process\.argv\.includes\('--auto-start'\)/);
 assert.match(tools,/process\.argv\.includes\('--fast'\)/);
 assert.match(rootScript,/DASHBOARD_SOURCE=sim/);
 assert.match(rootScript,/LIVE_COMMANDS_ENABLED=0 LIVE_IMU_RECAL_ENABLED=0/);
 assert.match(rootScript,/node tools\/demo\.js --auto-start --fast/);
 assert.ok(pkg.scripts['demo:drive']);
});
