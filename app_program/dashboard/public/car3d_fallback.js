/* CPU canvas 3D mesh projection: fallback when WebGL is unavailable. */
'use strict';
window.G7CreateSoftware3D=function(canvas){
 const g=canvas.getContext('2d');let yaw=.65,elev=.62,scale=93,drag=null,last=0,spin=0;
 let data={speedLeft:0,speedRight:0,humpMm:0,sourceMask:0};
 const sat=(x,a,b)=>Math.min(b,Math.max(a,x));
 const rot=(p,roll,heading)=>{let [x,y,z]=p;let a=y*Math.cos(roll)-z*Math.sin(roll),b=y*Math.sin(roll)+z*Math.cos(roll);return [x*Math.cos(heading)-a*Math.sin(heading),x*Math.sin(heading)+a*Math.cos(heading),b];};
 function project(p,w,h){let [x,y,z]=p;let right=-x*Math.sin(yaw)+y*Math.cos(yaw),up=-x*Math.sin(elev)*Math.cos(yaw)-y*Math.sin(elev)*Math.sin(yaw)+z*Math.cos(elev);
  const depth=x*Math.cos(elev)*Math.cos(yaw)+y*Math.cos(elev)*Math.sin(yaw)+z*Math.sin(elev);
  const perspective=1+(depth*.075);return [w/2+right*scale*perspective,h*.59-up*scale*perspective,depth];}
 function run(time){const dt=Math.min(.1,(time-last)/1000||0);last=time;spin+=dt*((Math.abs(data.speedLeft)+Math.abs(data.speedRight))/200);
  const W=canvas.clientWidth,H=canvas.clientHeight,dpr=Math.min(devicePixelRatio||1,2);if(W<1||H<1){requestAnimationFrame(run);return;}if(canvas.width!==Math.floor(W*dpr)||canvas.height!==Math.floor(H*dpr)){canvas.width=Math.floor(W*dpr);canvas.height=Math.floor(H*dpr);}g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,W,H);
  const hump=sat((data.humpMm||0)/220,0,.28),
  roll=Number.isFinite(data.rollDeg)?sat(data.rollDeg*Math.PI/180,-.5,.5):0;
  const heading=0; /* do not invent orientation */
  const poly=[];
  function face(vertices,color,alpha=1){const v=vertices.map(p=>project(p,W,H));poly.push({v,color,alpha,z:v.reduce((s,p)=>s+p[2],0)/v.length});}
  function cube(cx,cy,cz,dx,dy,dz,colors,local=true){const x=dx/2,y=dy/2,z=dz/2;
   const pts=[[-x,-y,-z],[-x,-y,z],[-x,y,-z],[-x,y,z],[x,-y,-z],[x,-y,z],[x,y,-z],[x,y,z]].map(p=>{const q=local?rot(p.map((v,i)=>v+[cx,cy,cz][i]),roll,heading):p.map((v,i)=>v+[cx,cy,cz][i]);return [q[0],q[1],q[2]];});
   const faces=[[0,2,6,4],[1,5,7,3],[0,4,5,1],[2,3,7,6],[0,1,3,2],[4,6,7,5]];
   for(let i=0;i<faces.length;i++)face(faces[i].map(j=>pts[j]),colors[i%colors.length]);}
  cube(0,0,-.20,8,8,.13,['#122639','#1c3c4f','#102435'],false);
  for(let i=-4;i<=4;i++){const a=project([-4,i,-.125],W,H),b=project([4,i,-.125],W,H),c=project([i,-4,-.125],W,H),d=project([i,4,-.125],W,H);g.strokeStyle='#2b4d5d44';g.beginPath();g.moveTo(a[0],a[1]);g.lineTo(b[0],b[1]);g.moveTo(c[0],c[1]);g.lineTo(d[0],d[1]);g.stroke();}
  if(data.sourceMask&4&&hump>.04)cube(.2,-.77,hump/2,1.1,.65,hump,['#a16d35','#cca044','#92661e'],false);
  cube(0,0,.45,2.20,1.20,.25,['#554a31','#af8938','#283e53']);
  cube(-.08,0,.70,1.3,.86,.19,['#b48a36','#d6b35e','#92722e']);
  cube(-.20,0,.82,.80,.57,.12,['#495160','#efd9ab','#aa8a49']);
  cube(-.21,0,.92,.36,.29,.10,['#354456','#f2dfa7','#80652c']);
  cube(.93,0,.69,.20,.82,.18,['#9e7d37','#e7d1a0','#74634b']);
  for(let side of [-1,1]){
   let y=side*.71;
   const center=rot([-.12,y,.22],roll,heading);
   const verts=[],segments=22;
   for(let k=0;k<segments;k++){let a=k*2*Math.PI/segments;verts.push(rot([-.12+Math.cos(a)*.35,y+side*.15,.22+Math.sin(a)*.35],roll,heading));}
   face(verts,'#161e28');
   const inner=[];for(let k=0;k<segments;k++){let a=k*2*Math.PI/segments;inner.push(rot([-.12+Math.cos(a)*.22,y+side*.16,.22+Math.sin(a)*.22],roll,heading));}face(inner,'#90bdcc');
   const spokes=[];for(let k=0;k<4;k++){const a=spin+k*Math.PI/2;spokes.push([rot([-.12+Math.cos(a)*.18,y+side*.18,.22+Math.sin(a)*.18],roll,heading),rot([-.12-Math.cos(a)*.18,y+side*.18,.22-Math.sin(a)*.18],roll,heading)]);}
   for(const [a,b] of spokes){let pa=project(a,W,H),pb=project(b,W,H);g.strokeStyle='#102c3a';g.lineWidth=3;g.beginPath();g.moveTo(pa[0],pa[1]);g.lineTo(pb[0],pb[1]);g.stroke();}
  }
  cube(.92,-.29,.72,.10,.21,.21,['#204e5e','#f4d88b','#37788e']);cube(.92,.29,.72,.10,.21,.21,['#204e5e','#f4d88b','#37788e']);
  poly.sort((a,b)=>a.z-b.z);
  for(const p of poly){g.globalAlpha=p.alpha;g.fillStyle=p.color;g.strokeStyle='#163647';g.lineWidth=1;g.beginPath();p.v.forEach((v,i)=>i===0?g.moveTo(v[0],v[1]):g.lineTo(v[0],v[1]));g.closePath();g.fill();g.stroke();}g.globalAlpha=1;
  requestAnimationFrame(run);
 }
 canvas.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
 canvas.addEventListener('pointermove',e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.009;elev=sat(elev+(e.clientY-drag[1])*.007,.2,1.2);drag=[e.clientX,e.clientY];});
 canvas.addEventListener('pointerup',()=>drag=null);
 canvas.addEventListener('wheel',e=>{e.preventDefault();scale=sat(scale*(e.deltaY>0?.9:1.1),48,160);},{passive:false});
 requestAnimationFrame(run);
 return {setTelemetry(s,p,sensor){data={...(s||{}),
   rollDeg:Number.isFinite(sensor?.rollDeg)?sensor.rollDeg:null};},
   resetCamera(){yaw=.65;elev=.62;scale=93;}};
};
