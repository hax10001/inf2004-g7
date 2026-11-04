/* G7 robot: small dependency-free WebGL 3D visualizer. GPU work stays on host. */
'use strict';
(() => {
const canvas=document.getElementById('car3d');
let gl;
try{gl=canvas.getContext('webgl',{antialias:true,alpha:false,depth:true});}catch{}
if(!gl){window.G7Car3D=window.G7CreateSoftware3D(canvas);return;}
const vs=`attribute vec3 aPosition; attribute vec3 aNormal;
uniform mat4 uMVP,uModel; varying vec3 vNormal; varying float vLight;
void main(){gl_Position=uMVP*vec4(aPosition,1.0);vNormal=mat3(uModel)*aNormal;}`;
const fs=`precision mediump float;uniform vec3 uColor; varying vec3 vNormal;
void main(){float d=max(dot(normalize(vNormal),normalize(vec3(-.3,.7,1.3))),0.0);float light=.34+.66*d;gl_FragColor=vec4(uColor*light,1.0);}`;
const compile=(type,src)=>{const s=gl.createShader(type);gl.shaderSource(s,src);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;};
let program;
try{program=gl.createProgram();gl.attachShader(program,compile(gl.VERTEX_SHADER,vs));gl.attachShader(program,compile(gl.FRAGMENT_SHADER,fs));gl.linkProgram(program);if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));}catch(e){console.warn('WebGL unavailable, using Canvas 3D:',e);window.G7Car3D=window.G7CreateSoftware3D(canvas);return;}
gl.useProgram(program);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);
const aP=gl.getAttribLocation(program,'aPosition'),aN=gl.getAttribLocation(program,'aNormal');
const uMVP=gl.getUniformLocation(program,'uMVP'),uModel=gl.getUniformLocation(program,'uModel'),uColor=gl.getUniformLocation(program,'uColor');
const id=()=>[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
function mul(a,b){const c=new Array(16).fill(0);for(let col=0;col<4;col++)for(let row=0;row<4;row++)for(let k=0;k<4;k++)c[col*4+row]+=a[k*4+row]*b[col*4+k];return c;}
function tr(x=0,y=0,z=0){const m=id();m[12]=x;m[13]=y;m[14]=z;return m;}
function sc(x=1,y=1,z=1){const m=id();m[0]=x;m[5]=y;m[10]=z;return m;}
function rx(t){const c=Math.cos(t),s=Math.sin(t);return [1,0,0,0,0,c,s,0,0,-s,c,0,0,0,0,1];}
function ry(t){const c=Math.cos(t),s=Math.sin(t);return [c,0,-s,0,0,1,0,0,s,0,c,0,0,0,0,1];}
function rz(t){const c=Math.cos(t),s=Math.sin(t);return [c,s,0,0,-s,c,0,0,0,0,1,0,0,0,0,1];}
function persp(fovy,ratio,near,far){const f=1/Math.tan(fovy/2),a=1/(near-far);return [f/ratio,0,0,0,0,f,0,0,0,0,(far+near)*a,-1,0,0,2*far*near*a,0];}
const norm=a=>{const l=Math.hypot(...a)||1;return a.map(v=>v/l);};
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
function look(eye,center,up){const z=norm(eye.map((e,i)=>e-center[i])),x=norm(cross(up,z)),y=cross(z,x);return [x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,-x.reduce((a,v,i)=>a+v*eye[i],0),-y.reduce((a,v,i)=>a+v*eye[i],0),-z.reduce((a,v,i)=>a+v*eye[i],0),1];}
function mesh(positions,normals){const pb=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,pb);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(positions),gl.STATIC_DRAW);
 const nb=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,nb);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(normals),gl.STATIC_DRAW);
 return {pb,nb,n:positions.length/3};}
function cubemesh(){const p=[],n=[];const faces=[[[1,0,0],[[1,-1,-1],[1,1,-1],[1,1,1],[1,-1,1]]],[[-1,0,0],[[-1,1,-1],[-1,-1,-1],[-1,-1,1],[-1,1,1]]],[[0,1,0],[[1,1,-1],[-1,1,-1],[-1,1,1],[1,1,1]]],[[0,-1,0],[[-1,-1,-1],[1,-1,-1],[1,-1,1],[-1,-1,1]]],[[0,0,1],[[1,-1,1],[1,1,1],[-1,1,1],[-1,-1,1]]],[[0,0,-1],[[-1,-1,-1],[-1,1,-1],[1,1,-1],[1,-1,-1]]]];
 for(const [face,vs] of faces)for(const i of [0,1,2,0,2,3]){p.push(...vs[i]);n.push(...face);}return mesh(p,n);}
function cylindermesh(segments=26){const p=[],n=[],put=(v,w,z,nx,ny,nz)=>{p.push(...v,...w,...z);for(let k=0;k<3;k++)n.push(nx,ny,nz);};
 for(let i=0;i<segments;i++){const t=i*Math.PI*2/segments,u=(i+1)*Math.PI*2/segments,a=[Math.cos(t),-1,Math.sin(t)],b=[Math.cos(u),-1,Math.sin(u)],c=[Math.cos(u),1,Math.sin(u)],d=[Math.cos(t),1,Math.sin(t)];
  const avg=[Math.cos((t+u)/2),0,Math.sin((t+u)/2)];put(a,b,c,...avg);put(a,c,d,...avg);put([0,-1,0],b,a,0,-1,0);put([0,1,0],d,c,0,1,0);
 }return mesh(p,n);}
const cube=cubemesh(),cyl=cylindermesh();
const theme={base:[.095,.16,.23],body:[.75,.58,.24],edge:[.95,.82,.51],board:[.18,.27,.37],chip:[.89,.81,.61],wheel:[.09,.13,.19],tire:[.10,.14,.19],hub:[.68,.70,.70],sonar:[.78,.71,.55],sensor:[.86,.67,.24],ground:[.10,.16,.23],hump:[.72,.48,.16]};
let yawCamera=.74,elevation=.48,radius=6.2,dragging=null;
let source={speedLeft:0,speedRight:0,humpMm:0,theta:0,
 hasMotion:false,hasTerrain:false};
let wheelLeft=0,wheelRight=0,lastT=0;
let priorTicks=null,estimatedTicks={left:0,right:0};
function renderShape(meshRef,m,color,vp){const mvp=mul(vp,m);gl.uniformMatrix4fv(uModel,false,new Float32Array(m));gl.uniformMatrix4fv(uMVP,false,new Float32Array(mvp));gl.uniform3fv(uColor,color);gl.bindBuffer(gl.ARRAY_BUFFER,meshRef.pb);gl.vertexAttribPointer(aP,3,gl.FLOAT,false,0,0);gl.enableVertexAttribArray(aP);gl.bindBuffer(gl.ARRAY_BUFFER,meshRef.nb);gl.vertexAttribPointer(aN,3,gl.FLOAT,false,0,0);gl.enableVertexAttribArray(aN);gl.drawArrays(gl.TRIANGLES,0,meshRef.n);}
function box(base,x,y,z,w,d,h,color,vp){renderShape(cube,mul(mul(base,tr(x,y,z)),sc(w/2,d/2,h/2)),color,vp);}
function disc(base,x,y,z,width,radius,rotation,color,vp){renderShape(cyl,mul(mul(mul(base,tr(x,y,z)),ry(rotation)),sc(radius,width/2,radius)),color,vp);}
function render(t){const dt=Math.min(.1,Math.max(0,(t-lastT)/1000));lastT=t;
 const cw=canvas.clientWidth,ch=canvas.clientHeight;if(cw<=0||ch<=0){requestAnimationFrame(render);return;}
 const dpr=Math.min(devicePixelRatio||1,2),W=Math.max(1,Math.round(cw*dpr)),H=Math.max(1,Math.round(ch*dpr));if(canvas.width!==W||canvas.height!==H){canvas.width=W;canvas.height=H;}gl.viewport(0,0,W,H);gl.clearColor(.055,.11,.17,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
 const eye=[radius*Math.cos(elevation)*Math.cos(yawCamera),radius*Math.cos(elevation)*Math.sin(yawCamera),radius*Math.sin(elevation)+.5];
 const vp=mul(persp(.82,W/H,.1,70),look(eye,[0,0,.4],[0,0,1]));
 box(id(),0,0,-.10,9,9,.12,theme.ground,vp);
 const humpHeight=Math.max(0,Math.min(.30,(source.humpMm||0)/240));
 if(source.hasTerrain && humpHeight>.018){box(id(),.22,-.80,humpHeight/2,1.05,.70,Math.max(.04,humpHeight),theme.hump,vp);}
 const roll=source.hasOrientation?Math.max(-.5,Math.min(.5,
   source.rollDeg*Math.PI/180)):0;
 const pitch=source.hasOrientation?Math.max(-.5,Math.min(.5,
   source.pitchDeg*Math.PI/180)):0;
 const turn=0; /* Heading is not a steering actuator measurement. */
 const bob=(Math.abs(source.speedLeft)+Math.abs(source.speedRight)>5)?Math.sin(t/140)*.008:0;
 const root=mul(mul(mul(tr(0,0,.43+bob),rz(turn)),rx(roll)),ry(pitch));
 box(root,0,0,0,2.03,1.16,.19,theme.base,vp);
 box(root,-.12,0,.16,1.20,.88,.15,theme.body,vp);
 box(root,-.10,0,.255,.88,.58,.06,theme.board,vp);
 box(root,-.20,0,.3,.35,.3,.09,theme.chip,vp);
 box(root,.22,0,.315,.20,.20,.11,theme.sensor,vp);
 box(root,-.89,0,.11,.19,.50,.18,theme.edge,vp);
 box(root,.96,0,.12,.12,.62,.18,theme.edge,vp);
 disc(root,.99,-.31,.17,.12,.115,0,theme.sonar,vp);disc(root,.99,.31,.17,.12,.115,0,theme.sonar,vp);
 if(source.hasMotion){
  wheelLeft+=dt*source.speedLeft/170;
  wheelRight+=dt*source.speedRight/170;
 } else if(source.hasTicks){
  /* Encoder-driven animation has illustrative scale, not calibrated RPM. */
  wheelLeft+=dt*estimatedTicks.left/75;
  wheelRight+=dt*estimatedTicks.right/75;
 }
 for(const side of [-1,1]){
  const yy=side*.70,rr=side<0?wheelLeft:wheelRight;
  disc(root,-.15,yy,-.13,.22,.32,rr,theme.tire,vp);
  disc(root,-.15,yy+side*.12,-.13,.05,.20,rr,theme.hub,vp);
  disc(root,-.15,yy+side*.15,-.13,.055,.08,rr,theme.base,vp);
 }
 disc(root,.84,0,-.24,.10,.16,t/220,theme.wheel,vp);
 requestAnimationFrame(render);
}
canvas.addEventListener('pointerdown',e=>{dragging=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointermove',e=>{if(!dragging)return;yawCamera+=(e.clientX-dragging[0])*.009;elevation=Math.min(1.26,Math.max(.12,elevation+(e.clientY-dragging[1])*.008));dragging=[e.clientX,e.clientY];});
canvas.addEventListener('pointerup',()=>dragging=null);
canvas.addEventListener('wheel',e=>{e.preventDefault();radius=Math.min(11,Math.max(2.8,radius*(e.deltaY>0?1.09:.91)));},{passive:false});
window.G7Car3D={setTelemetry(snapshot,pose,sensor){const s=snapshot||{};
 if(Number.isFinite(sensor?.leftEncoder)&&
    Number.isFinite(sensor?.rightEncoder)) {
  if(priorTicks && priorTicks.at!==sensor.received) {
   const dt=Math.max(.1,(sensor.received-priorTicks.at)/1000);
   estimatedTicks.left=Math.max(-400,Math.min(400,
     (sensor.leftEncoder-priorTicks.left)/dt));
   estimatedTicks.right=Math.max(-400,Math.min(400,
     (sensor.rightEncoder-priorTicks.right)/dt));
  }
  priorTicks={left:sensor.leftEncoder,right:sensor.right,
    at:sensor.received};
 } else {estimatedTicks={left:0,right:0};priorTicks=null;}
 source={speedLeft:Number(s.speedLeft)||0,speedRight:Number(s.speedRight)||0,
 humpMm:Number(s.humpMm)||0,theta:pose?.theta||0,
 hasMotion:!!(s.sourceMask&1),hasTerrain:!!(s.sourceMask&4),
 hasTicks:!!sensor && Number.isFinite(sensor.leftEncoder),
 rollDeg:sensor?.rollDeg,pitchDeg:sensor?.pitchDeg,
 hasOrientation:Number.isFinite(sensor?.rollDeg)&&
 Number.isFinite(sensor?.pitchDeg)};},resetCamera(){yawCamera=.74;elevation=.48;radius=6.2;}};
requestAnimationFrame(render);
})();
