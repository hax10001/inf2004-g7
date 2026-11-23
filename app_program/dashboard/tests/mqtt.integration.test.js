const test=require('node:test'),assert=require('node:assert/strict');
const {createBroker,mqttClient}=require('../server/mqtt');
const {frame,decodeFrame,TYPES}=require('../server/protocol');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function ready(c){for(let i=0;i<60;i++){if(c.ready)return;await delay(25);}throw Error('MQTT handshake did not complete');}
test('real TCP MQTT broker routes fragmented binary frame with authentication',async()=>{
 const b=createBroker({port:0,user:'demo',password:'not-a-real-secret'});await b.start();
 const port=b.server.address().port;
 let seen=null;const reader=mqttClient({port,user:'demo',password:'not-a-real-secret',clientId:'reader',onMessage:(topic,bytes)=>seen={topic,bytes}});
 const pub=mqttClient({port,user:'demo',password:'not-a-real-secret',clientId:'writer'});
 try{
  await ready(reader);await ready(pub);
  assert(reader.subscribe('g7/b1/+/telemetry'));
  await delay(70);
  const payload=frame(TYPES.ODOM,Buffer.from([9,0,7,0,1,0]),{seq:8,session:42,ms:55});
  assert(pub.publish('g7/b1/sim/telemetry',payload));
  for(let i=0;i<60&&!seen;i++)await delay(20);
  assert(seen);assert.equal(seen.topic,'g7/b1/sim/telemetry');
  assert.equal(decodeFrame(seen.bytes).seq,8);
 }finally{reader.socket.destroy();pub.socket.destroy();await b.close();}
});
