'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict');
const {CommandTracker}=require('../server/commands');
test('ACK must match command id, target and boot session, cannot confirm actuation',()=>{
 let now=100;const t=new CommandTracker(()=>now);t.sent({transaction:5,kind:'START',target:'robot',session:9,published:true});
 assert.equal(t.acknowledge({transaction:5,command:2,result:0,session:9},'sim'),null);
 assert.equal(t.acknowledge({transaction:5,command:3,result:0,session:9},'robot'),null);
 assert.equal(t.acknowledge({transaction:5,command:2,result:0,session:10},'robot'),null);
 assert.equal(t.acknowledge({transaction:5,command:2,result:0,session:9},'robot').status,'ACCEPTED');
 assert.equal(t.pending.length,0);
});
test('timeout and failed publishes have explicit statuses',()=>{
 let now=100;const t=new CommandTracker(()=>now);
 assert.equal(t.sent({transaction:1,kind:'STOP',target:'robot',session:2,published:false}).status,'PUBLISH_FAILED');
 t.sent({transaction:2,kind:'SNAPSHOT',target:'sim',session:2,published:true});now+=5001;
 assert.equal(t.expire().length,1);assert.equal(t.view().recent[0].status,'TIMED_OUT');
});
