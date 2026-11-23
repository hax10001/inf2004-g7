'use strict';
/* Dead-reckoned route from ODOM wheel deltas, corrected at junction turns.
 *
 * The robot sends EVENT frames ahead of queued ODOM frames, so an event can
 * arrive before the wheel motion that led up to it. Everything here is
 * keyed by the robot's own millisecond timestamps, not by arrival order:
 * turn corrections wait for the first ODOM record after the event, and an
 * event's marker is fixed at the pose the robot had at the event time.
 *
 * Junction turns are encoder-counted pivots on a right-angle track. When a
 * Left / Right / U-turn completes, the heading is set to the heading held
 * before the junction (averaged over the last straight stretch, so the
 * line-following wiggle does not bias it) plus +90 / -90 / 180 degrees, and
 * the position to the pivot point. A U-turn therefore retraces the way it
 * came. Obstacle bypasses and curves stay purely dead-reckoned.
 */
const TURN_DEG={1:90,2:-90,3:180};
const HEADING_WINDOW_MM=250;   // straight stretch averaged before a junction
const HISTORY_LIMIT=600;       // recent ODOM poses kept for time lookups
const POINT_SPACING_MM=15;     // drawn route decimation
const POINT_TURN_RAD=4*Math.PI/180;
const POINT_LIMIT=4000;
const BRIDGE_MIN_MM=20;        // checkpoint shortfall bridged as approximate

function wrap(a){while(a>Math.PI)a-=2*Math.PI;while(a<=-Math.PI)a+=2*Math.PI;return a;}

class RouteTracker {
  constructor(trackWidthMm=130){this.trackWidthMm=trackWidthMm;this.reset();}
  reset(){
    this.pose={x:0,y:0,theta:0,distanceMm:0};
    this.points=[{x:0,y:0,ms:0,uncertain:false}];
    this.history=[];          // {ms,x,y,theta,d}
    this.lastMotionSeq=null;this.lastOdomMs=null;
    this.odomGaps=0;this.uncertain=false;this.nextUncertain=false;
    this.checkpointOffset=null;this.bridgedMm=0;
    this.pendingTurns=[];     // TURN events waiting for later ODOM
    this.turn=null;           // turn in progress {theta0,x,y,action}
    this.junction=null;       // last junction confirm {ms}
    this.turnCorrections=0;
    this.waiting=[];          // events waiting for their pose
  }
  /* Pose at robot time ms from the recent ODOM history. */
  poseAt(ms){
    const h=this.history;
    if(!h.length)return {x:this.pose.x,y:this.pose.y,theta:this.pose.theta,d:this.pose.distanceMm};
    if(ms>=h[h.length-1].ms)return null;          // not driven that far yet
    for(let i=h.length-1;i>=0;i--)if(h[i].ms<=ms)return h[i];
    return h[0];
  }
  headingBefore(ms){
    const h=this.history;let end=h.length-1;
    while(end>=0&&h[end].ms>ms)end--;
    if(end<0)return this.pose.theta;
    let sx=0,sy=0;
    for(let i=end;i>0&&h[end].d-h[i].d<=HEADING_WINDOW_MM;i--){
      const w=Math.max(0,h[i].d-h[i-1].d);
      sx+=w*Math.cos(h[i].theta);sy+=w*Math.sin(h[i].theta);
    }
    return (sx||sy)?Math.atan2(sy,sx):h[end].theta;
  }
  addPoint(uncertain,ms,force=false){
    const last=this.points[this.points.length-1],p=this.pose;
    const moved=Math.hypot(p.x-last.x,p.y-last.y);
    if(force||uncertain||moved>=POINT_SPACING_MM||
       Math.abs(wrap(p.theta-(last.theta??p.theta)))>=POINT_TURN_RAD){
      this.points.push({x:p.x,y:p.y,theta:p.theta,ms,uncertain});
      if(this.points.length>POINT_LIMIT)this.points.shift();
    }
  }
  /* Apply turn events that happened before this ODOM record's time. */
  applyTurns(ms){
    this.pendingTurns.sort((a,b)=>a.ms-b.ms);
    while(this.pendingTurns.length&&this.pendingTurns[0].ms<ms){
      const t=this.pendingTurns.shift();
      if(t.kind===13){this.junction={ms:t.ms};continue;}
      if(t.kind===14){
        if(!TURN_DEG[t.action]){this.turn=null;continue;}
        const ref=this.junction&&t.ms-this.junction.ms<8000?this.junction.ms:t.ms;
        this.turn={theta0:this.headingBefore(ref),x:this.pose.x,y:this.pose.y,
          action:t.action,ms:t.ms,positioned:true};
        continue;
      }
      if(t.kind===15&&TURN_DEG[t.action]){
        let turn=this.turn;
        if(!turn||turn.action!==t.action){
          // TURN_STARTED was not received: heading from before the
          // junction, position left as dead-reckoned.
          const ref=this.junction&&t.ms-this.junction.ms<8000?this.junction.ms:t.ms;
          turn={theta0:this.headingBefore(ref),action:t.action,positioned:false};
        }
        this.pose.theta=wrap(turn.theta0+TURN_DEG[t.action]*Math.PI/180);
        if(turn.positioned){this.pose.x=turn.x;this.pose.y=turn.y;}
        this.turnCorrections++;this.turn=null;this.junction=null;
        this.addPoint(false,t.ms,true);
      }
    }
  }
  odometry(msg,ms){
    let gap=false;
    if(this.lastMotionSeq!==null&&((msg.sourceSeq-this.lastMotionSeq)&65535)!==1){
      gap=true;this.odomGaps++;this.uncertain=true;this.nextUncertain=true;
    }
    this.lastMotionSeq=msg.sourceSeq;
    this.applyTurns(ms);
    // Events at or before this record's time sit where the robot was
    // before this delta.
    this.resolveWaiting(ms,false);
    const mm=(msg.leftMm+msg.rightMm)/2,dth=(msg.rightMm-msg.leftMm)/this.trackWidthMm;
    this.pose.x+=mm*Math.cos(this.pose.theta+dth/2);
    this.pose.y+=mm*Math.sin(this.pose.theta+dth/2);
    this.pose.theta=wrap(this.pose.theta+dth);this.pose.distanceMm+=Math.abs(mm);
    this.history.push({ms,x:this.pose.x,y:this.pose.y,theta:this.pose.theta,d:this.pose.distanceMm});
    if(this.history.length>HISTORY_LIMIT)this.history.shift();
    this.lastOdomMs=ms;
    this.resolveWaiting(ms,true);
    this.addPoint(this.nextUncertain,ms);this.nextUncertain=false;
    return {gap};
  }
  /* Robot's cumulative driven distance: bridges ODOM records lost in a
   * gap. The first checkpoint after a reset only aligns the two counts. */
  checkpoint(cumulativeMm,ms){
    const at=this.poseAt(ms),d=at?at.d:this.pose.distanceMm;
    if(this.checkpointOffset===null){this.checkpointOffset=cumulativeMm-d;return {aligned:true,missingMm:0};}
    const missing=cumulativeMm-this.checkpointOffset-d;
    if(missing>=BRIDGE_MIN_MM){
      this.pose.x+=missing*Math.cos(this.pose.theta);
      this.pose.y+=missing*Math.sin(this.pose.theta);
      this.pose.distanceMm+=missing;this.bridgedMm+=missing;
      for(const h of this.history)if(h.ms>ms)h.d+=missing;
      this.addPoint(true,this.lastOdomMs??ms,true);
    }else if(missing<-100){this.checkpointOffset=cumulativeMm-d;}
    this.uncertain=false;
    return {aligned:false,missingMm:missing};
  }
  navEvent(kind,action,ms){
    if(kind===13||kind===14||kind===15)this.pendingTurns.push({kind,action,ms});
    if(this.lastOdomMs!==null&&ms<this.lastOdomMs)this.applyTurns(this.lastOdomMs);
  }
  /* Give an event its pose now if that time has been driven, else later. */
  place(event){
    if(this.lastMotionSeq===null){event.positionValid=false;return;}
    const at=this.poseAt(event.ms);
    event.positionValid=true;event.approximate=this.uncertain;
    if(at){event.x=at.x;event.y=at.y;event.positionPending=false;}
    else{event.x=this.pose.x;event.y=this.pose.y;event.positionPending=true;this.waiting.push(event);}
  }
  /* An ODOM record at time t covers motion up to t: an event before t sits
   * where the robot was before this delta, one at t where it is after. */
  resolveWaiting(ms,after){
    this.waiting=this.waiting.filter(e=>{
      if(after?e.ms<=ms:e.ms<ms){e.x=this.pose.x;e.y=this.pose.y;e.positionPending=false;e.positionValid=true;return false;}
      return true;
    });
  }
  /* Events still waiting are shown at the current pose. */
  refreshWaiting(){for(const e of this.waiting){e.x=this.pose.x;e.y=this.pose.y;}}
}
module.exports={RouteTracker,TURN_DEG};
