'use strict';
const test=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const root=path.join(__dirname,'..');
const app=fs.readFileSync(path.join(root,'public/app.js'),'utf8');
const state=fs.readFileSync(path.join(root,'server/state.js'),'utf8');
const config=fs.readFileSync(path.join(root,'../app/app_config.h'),'utf8');

function render(view,{filter='all',search=''}={}){
 const nodes={scannerLiveState:{textContent:'',dataset:{}},
  scannerLivePanel:{dataset:{}},scannerLiveTitle:{textContent:''},
  scannerLiveDetail:{textContent:''},scannerLastResult:{textContent:'',dataset:{}},
  scannerFilter:{value:filter},scannerSearch:{value:search}};
 let result='';
 const code=app.match(/^const safe=.*$/m)[0]+'\n'+
  app.slice(app.indexOf('function updateScannerPanels(){'),
            app.indexOf('function renderTelemetry('))+'\nupdateScannerPanels();';
 vm.runInNewContext(code,{view,Date,el:id=>nodes[id],
  paintLiveList:(id,html)=>{assert.equal(id,'telemetryScanHistory');result=html;}});
 return {badge:nodes.scannerLiveState,html:result,nodes};
}

test('scanner card removes duplicate summaries, diagnostics and mapping warnings',()=>{
 for(const phrase of ['Scanner diagnostics paused','Last scanner update',
  'Last totals: completed','Last navigation outcome:','Edit barcode action mapping'])
  assert.ok(!app.includes(phrase),phrase);
 assert.match(app,/sensorPanel\.querySelector\('\.scanSpotlight'\)\.remove\(\)/);
 assert.match(app,/commandPanel\.querySelector\('\.monitorScanNote'\)\.remove\(\)/);
 assert.match(app,/peripheral\.append\(cards\[1\],cards\[2\]\)/);
});
test('old scans retain timestamped results while live scanner state goes offline',()=>{
 const old=Date.now()-8128000;
 const result=render({mode:'LIVE',lastSeen:old,barcodeDiag:{received:old,
  completed:6,rejected:4,scanner:{state:2}},
  scanOutcomes:[{at:old,status:'success',letter:'T',detail:'decoded T',source:'LIVE'}]});
 assert.equal(result.badge.textContent,'Offline');
 assert.equal(result.badge.dataset.tone,'quiet');
 assert.match(result.html,/Letter T/);assert.match(result.html,/<time datetime=/);
 assert.doesNotMatch(result.html,/8128|historical counters|completed 6|rejected 4/);
});
test('fresh ready and simulated readiness remain distinct',()=>{
 const view={mode:'LIVE',lastSeen:Date.now(),barcodeDiag:{received:Date.now(),
  scanner:{state:2}},scanOutcomes:[]};
 assert.equal(render(view).badge.textContent,'Ready');
 assert.equal(render(view).badge.dataset.tone,'ready');
 view.mode='SIMULATED';
 assert.equal(render(view).badge.textContent,'Simulated · Ready');
 assert.equal(render(view).badge.dataset.tone,'simulated');
 view.mode='LIVE';view.barcodeDiag.scanner.state=6;
 assert.equal(render(view).badge.textContent,'Needs white');
 assert.equal(render(view).badge.dataset.tone,'warning');
});
test('scan filtering keeps real rejected reasons and handles no matches',()=>{
 const now=Date.now();const view={scanOutcomes:[
  {at:now,status:'success',letter:'M',detail:'decoded M',source:'LIVE'},
  {at:now-1000,status:'failed',detail:'Capture timed out',source:'LIVE'}]};
 const failed=render(view,{filter:'failed'});
 assert.match(failed.html,/Capture timed out/);assert.doesNotMatch(failed.html,/Letter M/);
 assert.match(render(view,{search:'missing'}).html,/No matching scans/);
 assert.equal(view.scanOutcomes[0].letter,'M','rendering does not reorder the source');
 assert.match(state,/msg\.eventKind===19 && msg\.value===2\?'letter not in current four-letter mapping/);
});
test('IMU stays enabled in normal firmware profile',()=>{
 assert.match(config,/#define APP_IMU_ENABLED 1 \/\* Normal MQTT/);
});

test('scanner progress explains ready, decoding and rearming while the last failure stays visible',()=>{
 const now=Date.now();
 const view={mode:'LIVE',lastSeen:now,robotStatus:{running:false},
  barcodeDiag:{received:now,scanner:{state:2}},
  scanOutcomes:[{at:now,status:'failed',detail:'No barcode pattern found',source:'LIVE'}]};
 let result=render(view);
 assert.equal(result.nodes.scannerLiveTitle.textContent,'Ready to scan');
 assert.match(result.nodes.scannerLiveDetail.textContent,/Press Start/);
 assert.match(result.nodes.scannerLastResult.textContent,/Last scan failed.*No barcode pattern found/);
 for(const [phase,title] of [[3,'Reading barcode'],[4,'Decoding scan'],[5,'Rearming scanner']]){
  view.barcodeDiag.scanner.state=phase;result=render(view);
  assert.equal(result.nodes.scannerLiveTitle.textContent,title);
  assert.equal(result.nodes.scannerLivePanel.dataset.phase,String(phase));
  assert.equal(result.nodes.scannerLastResult.dataset.result,'failed');
 }
 view.lastSeen=now-6000;result=render(view);
 assert.equal(result.nodes.scannerLiveTitle.textContent,'Scanner offline');
 assert.match(result.nodes.scannerLastResult.textContent,/Last scan failed/);
});
