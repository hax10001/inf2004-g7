'use strict';
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto');
// Append-only host recording, independent of browser connections.
class Recorder {
 constructor(directory){fs.mkdirSync(directory,{recursive:true});this.id=Date.now()+'-'+crypto.randomBytes(4).toString('hex');this.path=path.join(directory,this.id+'.jsonl');this.error=null;this.dropped=0;this.stream=fs.createWriteStream(this.path,{flags:'ax',mode:0o600});this.stream.on('error',e=>{this.error=e.message;});}
 write(kind,data){if(this.error||this.stream.writableLength>1024*1024){this.dropped++;return false;}return this.stream.write(JSON.stringify({recording:this.id,receivedAt:new Date().toISOString(),kind,...data})+'\n');}
 close(){return new Promise(resolve=>{if(this.stream.destroyed)return resolve();this.stream.end(resolve);});}
 status(){return {id:this.id,error:this.error,dropped:this.dropped};}
}
module.exports={Recorder};
