'use strict';
const fs=require('node:fs'),readline=require('node:readline');
const columns=['receivedAt','kind','source','topic','target','command','transaction','published','payloadBase64','session','seq','robotMs','ackResult'];
function csvField(value){let s=value==null?'':String(value);if(/^[=+@\-\t\r]/.test(s))s="'"+s;return /[",\r\n]/.test(s)?'"'+s.replaceAll('"','""')+'"':s;}
async function streamCsv(file,res){
 res.write(columns.join(',')+'\n');
 const input=fs.createReadStream(file,{encoding:'utf8'});
 for await(const row of readline.createInterface({input,crlfDelay:Infinity})){
  if(!row.trim())continue;
  let data;try{data=JSON.parse(row);}catch{continue;}
  if(!res.write(columns.map(key=>csvField(data[key])).join(',')+'\n')){
   await new Promise(resolve=>res.once('drain',resolve));
  }
 }
 res.end();
}
module.exports={columns,csvField,streamCsv};
