'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const root=path.join(__dirname,'../public');
const app=fs.readFileSync(path.join(root,'app.js'),'utf8');
const css=fs.readFileSync(path.join(root,'style.css'),'utf8');
const html=fs.readFileSync(path.join(root,'index.html'),'utf8');

test('every monitoring list has independent scroll-follow detection and Latest button',()=>{
 for(const id of ['telemetryOutput','telemetryScanHistory','robotLogOutput',
                  'consoleOutput','commandLedgerOutput'])
  assert.match(app,new RegExp("\\['"+id+"'|\\['"+id+"',"));
 assert.match(app,/node\.addEventListener\('scroll'/);
 assert.match(app,/set\(isAtLatest\(node\)\)/);
 assert.match(app,/jump\.addEventListener\('click'/);
 assert.match(app,/state\?\.follow\)node\.scrollTop=node\.scrollHeight/);
 assert.match(app,/node\.scrollTop=Math\.min\(oldTop/);
 assert.match(css,/\.latestButton\.hidden/);
});
test('new entries append oldest-to-newest, and pause-view does not stop ingestion',()=>{
 assert.match(app,/selectedScans\.reverse\(\)\.map/);
 assert.match(app,/rows\.reverse\(\)\.map/);
 assert.match(app,/logs\.slice\(0,220\)\.reverse\(\)\.map/);
 assert.match(app,/entries\.reverse\(\)\.map/);
 assert.match(app,/robotLog\.push\(\.\.\.\(d\.lines\|\|\[\]\)\)/);
 assert.equal((app.match(/new EventSource\(/g)||[]).length,1);
});
test('four responsive quadrants share a full-height card shell and internal scroll bodies',()=>{
 for(const id of ['pane-sensors','pane-barcode','pane-robotlog','pane-commands'])
  assert.match(app,new RegExp("'"+id+"'"));
 assert.match(css,/grid-auto-rows:minmax\(0,590px\)/);
 assert.match(css,/\.unifiedPane>\.scanTimeline/);
 assert.match(css,/scrollbar-gutter:stable/);
 assert.match(css,/@media\(max-width:1060px\)/);
 assert.match(html,/class="scanTimeline"/);
});
