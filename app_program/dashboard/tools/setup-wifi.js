'use strict';
const path=require('node:path'),fs=require('node:fs');
const {primary,secrets}=require('./lan-common');
const project=path.resolve(__dirname,'../../..');
const wifi=path.join(project,'config/wifi_credentials.h');
const broker=path.join(project,'config/wifi_secrets.h');
const legacyBroker=path.join(project,'app_program/modules/telemetry/buddy1_broker_local.h');
function cQuote(s){return JSON.stringify(s).replace(/\\u([0-9a-f]{4})/gi,(_,hex)=>`\\x${Number.parseInt(hex,16).toString(16).padStart(2,'0')}`);}
// Keep generated BARR-C header lines within 80 columns, even for long keys.
function cMacroString(name,value){
 const parts=[];let chunk='';
 for(const character of Array.from(value)){
  if(chunk && cQuote(chunk+character).length>40){parts.push(cQuote(chunk));chunk='';}
  chunk+=character;
 }
 if(chunk)parts.push(cQuote(chunk));
 if(parts.length<=1)return `#define ${name} ${parts[0]||'""'}`;
 const continuation=' \\' + '\n    ';
 return `#define ${name}` + continuation + parts.join(continuation);
}
async function ttyQuestion(label,hidden=false){
 if(!process.stdin.isTTY)throw Error('Run the WiFi wizard from an interactive Mac terminal.');
 return new Promise(resolve=>{
  process.stdout.write(label);const input=process.stdin;let answer='';input.setRawMode(true);input.resume();
  const cb=key=>{for(const keyText of key.toString('utf8')){
   if(keyText==='\r'||keyText==='\n'){
     input.removeListener('data',cb);input.setRawMode(false);process.stdout.write('\n');resolve(answer);return;
   }
   if(keyText==='\u0003'){input.setRawMode(false);process.exit(130);}
   if(keyText==='\u007f'||keyText==='\b'){
     if(answer.length){answer=answer.slice(0,-1);if(!hidden)process.stdout.write('\b \b');}
     continue;
   }
   if(keyText>=' '){answer+=keyText;if(!hidden)process.stdout.write(keyText);}
  }};
  input.on('data',cb);
 });
}
async function main(){const addr=primary();if(!addr)throw Error('Mac has no private LAN IPv4. Connect it to the same router as the Pico first.');
 console.log('\nG7 Pico WiFi setup — writes ONLY gitignored local configuration files.');
 console.log(`Mac IPv4 detected: ${addr.ip} (${addr.interface})`);
 const suggestedSSID=process.env.G7_DEFAULT_WIFI_SSID||'';
 const ssid=(await ttyQuestion(`2.4 GHz WiFi network name (SSID)${suggestedSSID?' ['+suggestedSSID+']':''}: `)).trim()||suggestedSSID;
 const password=await ttyQuestion('WiFi password (hidden): ',true);
 const brokerIP=(await ttyQuestion(`Mac MQTT broker IPv4 [${addr.ip}]: `)).trim()||addr.ip;
 if(!ssid||Buffer.byteLength(ssid)>32)throw Error('WiFi SSID must be 1–32 bytes.');
 if(Buffer.byteLength(password)<8||Buffer.byteLength(password)>63)throw Error('WPA2 password must be 8–63 bytes.');
 if(!/^\d{1,3}(\.\d{1,3}){3}$/.test(brokerIP)||brokerIP.split('.').some(x=>+x>255))throw Error('Invalid broker IPv4.');
 const creds=secrets();
 for(const f of [wifi,broker,legacyBroker])if(fs.existsSync(f))throw Error(`${f} exists; leaving it unchanged. Back it up and remove it yourself to regenerate.`);
 const a=`#ifndef WIFI_CREDENTIALS_H\n#define WIFI_CREDENTIALS_H\n/* Generated locally; do not commit. Pico W needs a 2.4 GHz-capable network. */\n#define UTK_WIFI_CREDENTIALS_SET 1\n#define UTK_WIFI_SSID ${cQuote(ssid)}\n#define UTK_WIFI_PASSWORD ${cQuote(password)}\n#define UTK_WIFI_AUTH CYW43_AUTH_WPA2_AES_PSK\n#endif\n`;
 const b=`/* Private MQTT configuration: only mqtt_utk.c reads this. */\n#ifndef WIFI_SECRETS_H\n#define WIFI_SECRETS_H\n/* Private compile-time settings. Never commit this file. */\n#define B1_BROKER_IPV4 ${cQuote(brokerIP)}\n#define B1_BROKER_PORT 1883\n${cMacroString("B1_MQTT_USER",creds.mqttUser)}\n${cMacroString("B1_MQTT_PASSWORD",creds.mqttPassword)}\n#endif\n/*** end of file ***/\n\n`;
 fs.writeFileSync(wifi,a,{mode:0o600,flag:'wx'});fs.writeFileSync(broker,b,{mode:0o600,flag:'wx'});
 console.log('\nCreated: config/wifi_credentials.h');
 console.log('Created: config/wifi_secrets.h (network-owner MQTT)');
 console.log(`Network: ${ssid}; MQTT broker: ${brokerIP}:1883`);
 console.log('Credentials are firmware compile-time configuration. You still need to build/flash Pico W WiFi/MQTT firmware.');
 console.log('NOTICE: WiFi/MQTT is opt-in at firmware build time; configuration alone does not activate it.');
 process.stdin.pause();process.stdin.unref?.();
}
main().catch(e=>{process.stdin.pause();process.stdin.unref?.();console.error('WiFi setup failed:',e.message);process.exitCode=1;});
