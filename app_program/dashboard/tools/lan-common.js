'use strict';
const fs=require('node:fs'),crypto=require('node:crypto'),path=require('node:path');
const {primary}=require('../server/network');
const secretsFile=path.resolve(__dirname,'../.lan-secrets.json');
function secrets(){if(fs.existsSync(secretsFile)){
  const v=JSON.parse(fs.readFileSync(secretsFile,'utf8'));
  if(!v.mqttUser||!v.mqttPassword||!v.adminToken)throw Error('Incomplete .lan-secrets.json; do not proceed');
  return v;
}
 const v={mqttUser:'g7_robot',mqttPassword:crypto.randomBytes(20).toString('hex'),adminToken:crypto.randomBytes(20).toString('hex')};
 fs.writeFileSync(secretsFile,JSON.stringify(v,null,2)+'\n',{mode:0o600,flag:'wx'});
 return v;
}
module.exports={primary,secrets,secretsFile};
