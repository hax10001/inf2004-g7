'use strict';
/* Host-side command lifecycle. MQTT QoS-0 publish is not an execution proof. */
const COMMAND_NAME=['','SET_MAPPING','START','STOP','SNAPSHOT','RECALIBRATE_IMU'];
class CommandTracker {
  constructor(now=()=>Date.now()) {this.now=now;this.pending=[];this.completed=[];}
  sent({transaction,kind,target,session,published}) {
    const item={transaction,kind,target,session,sentAt:this.now(),status:published?'SENT':'PUBLISH_FAILED',result:null};
    if(published)this.pending.push(item);else this.completed.unshift(item);
    this.pending=this.pending.slice(-100);this.completed=this.completed.slice(0,50);return item;
  }
  acknowledge(f,source) {
    const kind=COMMAND_NAME[f.command];
    const idx=this.pending.findIndex(c=>c.transaction===f.transaction&&c.kind===kind&&c.target===source&&c.session===f.session);
    if(idx<0)return null;
    const item=this.pending.splice(idx,1)[0];item.status=f.result===0?'ACCEPTED':'REJECTED';item.result=f.result;item.ackAt=this.now();
    this.completed.unshift(item);this.completed=this.completed.slice(0,50);return item;
  }
  invalidateSession(session){
    const closed=[];
    this.pending=this.pending.filter(item=>{
      if(item.session===session)return true;
      item.status='SESSION_ENDED';item.closedAt=this.now();closed.push(item);return false;
    });
    this.completed.unshift(...closed);this.completed=this.completed.slice(0,50);
    return closed;
  }
  expire(maxAgeMs=5000) {
    const at=this.now(),timed=[];
    this.pending=this.pending.filter(item=>{
      if(at-item.sentAt<maxAgeMs)return true;
      item.status='TIMED_OUT';item.timedOutAt=at;timed.push(item);return false;
    });
    this.completed.unshift(...timed);this.completed=this.completed.slice(0,50);return timed;
  }
  view(){return {pending:this.pending.slice(),recent:this.completed.slice(0,25)};}
}
module.exports={CommandTracker};
