'use strict';
const fs=require('node:fs'),path=require('node:path');
/* Robot serial console received over MQTT (<namespace>/<robot>/log).
   The firmware sends raw console bytes in chunks that usually end on a line
   break; a partial line is held until its newline arrives. Each dashboard
   run appends to one plain-text file in the same format as a USB serial
   capture, so barcode replay tools can read it directly. */
class RobotLog {
  constructor(directory,id,max=3000){
    fs.mkdirSync(directory,{recursive:true});
    this.path=path.join(directory,'robot-'+id+'.log');
    this.max=max;this.lines=[];this.partial='';this.total=0;this.error=null;
    this.stream=fs.createWriteStream(this.path,{flags:'a',mode:0o600});
    this.stream.on('error',e=>{this.error=e.message;});
  }
  /* Returns the complete lines this chunk finished, each {n,t,text}. */
  add(bytes){
    const text=this.partial+Buffer.from(bytes).toString('latin1').replace(/\r/g,'');
    const parts=text.split('\n');this.partial=parts.pop();
    if(this.partial.length>2000){parts.push(this.partial);this.partial='';}
    const t=Date.now(),out=[];
    for(const p of parts){
      const line={n:++this.total,t,text:p};out.push(line);this.lines.push(line);
      if(!this.error&&this.stream.writableLength<1024*1024)this.stream.write(p+'\n');
    }
    if(this.lines.length>this.max)this.lines.splice(0,this.lines.length-this.max);
    return out;
  }
  snapshot(){return {lines:this.lines,file:path.basename(this.path),error:this.error};}
  close(){return new Promise(r=>{if(this.stream.destroyed)return r();this.stream.end(r);});}
}
module.exports={RobotLog};
