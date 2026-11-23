'use strict';
const os=require('node:os');
const {execFileSync}=require('node:child_process');
function privateV4(ip){const n=ip.split('.').map(Number);if(n.length!==4||n.some(x=>!Number.isInteger(x)||x<0||x>255))return false;
  return n[0]===10||(n[0]===172&&n[1]>=16&&n[1]<=31)||(n[0]===192&&n[1]===168);
}
function interfaces(){let entries;try{entries=os.networkInterfaces();}catch{return [];}return Object.entries(entries).flatMap(([name,addresses])=>(addresses||[])
  .filter(a=>a.family==='IPv4'&&!a.internal&&privateV4(a.address))
  .map(a=>({interface:name,ip:a.address})));}
function primary(){const all=interfaces();let preferred='';
  if(process.platform==='darwin')try{const v=execFileSync('route',['-n','get','default'],{encoding:'utf8',timeout:1000});preferred=(v.match(/interface:\s*([^\s]+)/)||[])[1]||'';}catch{}
  return all.find(a=>a.interface===preferred)||all.find(a=>a.interface==='en0')||all.find(a=>a.interface==='wlan0')||all[0]||null;
}
module.exports={interfaces,primary,privateV4};
