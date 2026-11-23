'use strict';
/* Tiny MQTT 3.1.1 QoS 0 broker/client for local deterministic development.
   Not a production-grade broker; use Mosquitto with ACLs for live assessment. */
const net=require('node:net');
function varLen(n) {const a=[];do{let b=n%128;n=Math.floor(n/128);if(n)b|=128;a.push(b);}while(n);return Buffer.from(a);}
function str(s) {const b=Buffer.from(s);if(b.length>65535)throw Error('string too long');const h=Buffer.alloc(2);h.writeUInt16BE(b.length);return Buffer.concat([h,b]);}
function packet(head,data) {if(data.length>8192)throw Error('oversized MQTT packet');return Buffer.concat([Buffer.from([head]),varLen(data.length),data]);}
function packets(socket,callback) {
  let input=Buffer.alloc(0);
  socket.on('data',chunk=>{
    input=Buffer.concat([input,chunk]);
    if(input.length>32768){socket.destroy();return;}
    while(input.length>=2){let mul=1,len=0,pos=1,byte;
      do {if(pos>=input.length)return; byte=input[pos++]; len+=(byte&127)*mul;mul*=128; if(pos>5||len>8192){socket.destroy();return;}}while(byte&128);
      if(input.length<pos+len)return;
      const head=input[0],payload=input.subarray(pos,pos+len);
      input=input.subarray(pos+len);
      try{callback(head,payload);}catch(e){socket.destroy();return;}
    }
  });
}
function readStr(b,offset=0) {if(offset+2>b.length)throw Error('string length');const len=b.readUInt16BE(offset);if(offset+2+len>b.length)throw Error('string truncated');return [b.toString('utf8',offset+2,offset+2+len),offset+2+len];}
function topicMatch(sub,topic){if(sub===topic)return true;const a=sub.split('/'),b=topic.split('/');for(let i=0;i<a.length;i++) {if(a[i]==='#')return i===a.length-1;if(a[i]!== '+'&&a[i]!==b[i])return false;}return a.length===b.length;}
function createBroker({host='127.0.0.1',port=1883,user='',password=''}={}) {
  const peers=new Set();
  const server=net.createServer(socket=>{
    socket.setNoDelay(true);
    const peer={socket,connected:false,subscriptions:new Set()};peers.add(peer);
    socket.on('close',()=>peers.delete(peer));socket.on('error',()=>{});
    packets(socket,(head,p)=>{
      const type=head>>4;
      if(type===1){
        let [proto,o]=readStr(p);if(proto!=='MQTT'||p[o++]!==4)throw Error('only MQTT 3.1.1');
        const flags=p[o++];o+=2;[,o]=readStr(p,o);
        if(flags&4){[,o]=readStr(p,o);[,o]=readStr(p,o);}
        let u='',pw='';if(flags&128)[u,o]=readStr(p,o);if(flags&64)[pw,o]=readStr(p,o);
        const valid=(!user&&!password)||(u===user&&pw===password);
        socket.write(packet(0x20,Buffer.from([0,valid?0:4])));peer.connected=valid;
        if(!valid)socket.end();return;
      }
      if(!peer.connected)throw Error('not connected');
      if(type===3){
        if((head&6)!==0)throw Error('QoS 0 only');
        const [topic,o]=readStr(p);if(topic.includes('+')||topic.includes('#'))throw Error('wildcard publish');
        for(const other of peers)if(other.connected&& [...other.subscriptions].some(x=>topicMatch(x,topic))) {
          if(!other.socket.destroyed)other.socket.write(packet(0x30,Buffer.concat([str(topic),p.subarray(o)])));
        }
      } else if(type===8){
        if((head&15)!==2||p.length<5)throw Error('invalid subscribe');
        let o=2;const replies=[];
        while(o<p.length){let topic;[topic,o]=readStr(p,o);if(o>=p.length)throw Error('missing qos');const qos=p[o++];peer.subscriptions.add(topic);replies.push(qos===0?0:0x80);}
        socket.write(packet(0x90,Buffer.concat([p.subarray(0,2),Buffer.from(replies)])));
      } else if(type===12){socket.write(packet(0xd0,Buffer.alloc(0)));}
      else if(type===14)socket.end();
      else throw Error('unsupported MQTT packet');
    });
  });
  return {server,start:()=>new Promise(r=>server.listen(port,host,r)),close:()=>new Promise(r=>{for(const p of peers)p.socket.destroy();server.close(r);})};
}
function mqttClient({host='127.0.0.1',port=1883,clientId='b1-host',user='',password='',onMessage=()=>{},onClose=()=>{}}={}) {
  const socket=net.connect({host,port});let ready=false,active=false;
  socket.on('connect',()=>{
    let flags=2,auth=[];if(user){flags|=128;auth.push(str(user));}if(password){flags|=64;auth.push(str(password));}
    const body=Buffer.concat([str('MQTT'),Buffer.from([4,flags,0,30]),str(clientId),...auth]);
    socket.write(packet(0x10,body));
  });
  packets(socket,(head,p)=>{
    if(head===0x20){ready=p.length===2&&p[1]===0;active=ready;if(!ready)socket.destroy();}
    if((head>>4)===3){const [topic,o]=readStr(p);onMessage(topic,p.subarray(o));}
  });
  socket.on('error',()=>{});socket.on('close',()=>{ready=false;onClose();});
  return {publish(topic,body){if(!ready)return false;socket.write(packet(0x30,Buffer.concat([str(topic),Buffer.from(body)])));return true;},
    subscribe(topic){if(!ready)return false;socket.write(packet(0x82,Buffer.concat([Buffer.from([0,1]),str(topic),Buffer.from([0])])));return true;},
    get ready(){return ready;}, close(){if(!socket.destroyed)socket.end();},socket};
}
module.exports={varLen,packet,packets,str,readStr,topicMatch,createBroker,mqttClient};
