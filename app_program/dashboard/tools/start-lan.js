'use strict';
const path=require('node:path'),{spawn}=require('node:child_process');
const {primary,secrets,secretsFile}=require('./lan-common');
const address=primary();
if(!address){console.error('No private LAN IPv4 address found. Connect your Mac to WiFi first. Try npm start for localhost-only mode.');process.exit(2);}
const s=secrets();
const env={...process.env,WEB_BIND:'0.0.0.0',BROKER_BIND:'0.0.0.0',BROKER_HOST:'127.0.0.1',
 MQTT_USER:s.mqttUser,MQTT_PASSWORD:s.mqttPassword,ADMIN_TOKEN:s.adminToken};
console.log('\nGROUP G7 — LAN dashboard');
console.log(`Mac network: ${address.interface} (${address.ip})`);
console.log('Computer: http://localhost:3000');
console.log(`Other devices on reachable LAN: http://${address.ip}:3000`);
console.log(`Pico MQTT broker: ${address.ip}:1883`);
console.log('MQTT username: '+s.mqttUser);console.log('MQTT password: '+s.mqttPassword);
console.log('Dashboard admin token: '+s.adminToken);
console.log('Credentials stored locally in: '+secretsFile+' (DO NOT COMMIT OR SHARE)');
console.log('Control mode: '+(process.env.LIVE_COMMANDS_ENABLED==='1'?'LIVE CONTROL ENABLED (bench only)':'READ-ONLY. For mapping and Start/Stop use bash run_live_dashboard.sh from project root.'));
console.log('Next: npm run setup:wifi (one-time interactive Pico WiFi configuration)');
console.log('The Pico will NOT appear as connected until real Pico MQTT firmware is built, flashed and sending packets.\n');
const child=spawn(process.execPath,['server/index.js'],{cwd:path.resolve(__dirname,'..'),env,stdio:'inherit'});
for(const sig of ['SIGINT','SIGTERM'])process.on(sig,()=>child.kill(sig));
child.on('exit',(code)=>process.exitCode=code??0);
