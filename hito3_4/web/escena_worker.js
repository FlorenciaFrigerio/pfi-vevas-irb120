import * as THREE from './vendor/three.module.min.js';
function matriz(a){return new THREE.Matrix4().set(...a);}
class Renderer{
 constructor(){this.canvas=new OffscreenCanvas(1,1);this.ctx=this.canvas.getContext('2d');}
 dibujar(){
  if(!this.w||!this.h)return;
  const ctx=this.ctx,w=this.w,h=this.h;ctx.clearRect(0,0,w,h);ctx.fillStyle='#f7f9fc';ctx.fillRect(0,0,w,h);
  const ct=Math.cos(this.theta),st=Math.sin(this.theta),cp=Math.cos(this.phi),sp=Math.sin(this.phi),scale=Math.min(w,h)*1.1*this.zoom;
  const proj=p=>{const x=p[0]-.16,y=p[1],z=p[2]-.32;return [w*.5+scale*(-st*x+ct*y),h*.53-scale*(-sp*ct*x-sp*st*y+cp*z),cp*ct*x+cp*st*y+sp*z];};
  const path=(ps,color,width=1,dash=[])=>{if(!ps.length)return;ctx.beginPath();ps.map(proj).forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);};
  for(let i=-6;i<=6;i++){path([[i/10,-.6,0],[i/10,.6,0]],'#e0e7ef');path([[-.6,i/10,0],[.6,i/10,0]],'#e0e7ef');}
  if(this.planoZmin.visible){const ps=[[-.55,-.55,this.planoZmin.z],[.55,-.55,this.planoZmin.z],[.55,.55,this.planoZmin.z],[-.55,.55,this.planoZmin.z]].map(proj);ctx.beginPath();ps.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.fillStyle='#e7bdc612';ctx.fill();}
  if(this.entorno?.visible){
   const caras = [[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]];
   const superficies = this.entorno.cajas.flatMap(({min,max,confirmado})=>{
    const puntos = [min[0],max[0]].flatMap(x=>[min[1],max[1]].flatMap(y=>[min[2],max[2]].map(z=>proj([x,y,z]))));
    return caras.map(cara=>({ps:cara.map(i=>puntos[i]),confirmado}));
   });
   superficies.sort((a,b)=>a.ps.reduce((s,p)=>s+p[2],0)-b.ps.reduce((s,p)=>s+p[2],0));
   for(const {ps,confirmado} of superficies){
    ctx.beginPath();ps.forEach(([x,y],i)=>i?ctx.lineTo(x,y):ctx.moveTo(x,y));ctx.closePath();
    ctx.fillStyle = confirmado?'#236b9c38':'#8796a118';ctx.fill();
    ctx.strokeStyle = confirmado?'#236b9caa':'#8796a177';ctx.lineWidth=1;ctx.stroke();
   }
  }
  const axes=(M,l=.07)=>{const o=[M[3],M[7],M[11]];[0,1,2].forEach((k)=>path([o,o.map((x,i)=>x+l*M[i*4+k])],['#c7505d','#388b6e','#3e83b4'][k],1.5));};
  axes([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],.20);
  const drawRobot=(robot,ghost)=>{if(!robot.visible||!robot.M||!this.geometrias)return[];const faces=[];
   const colorFantasma = robot.color ?? 0x198ba7;
   const rgbFantasma = [colorFantasma >> 16, (colorFantasma >> 8) & 255, colorFantasma & 255];
   this.geometrias.forEach((g,j)=>{const m=matriz(robot.M[j]).multiply(this.align[j]).elements;
    for(let i=0;i<g.p.length;i+=9){
     const nx=m[0]*g.n[i]+m[4]*g.n[i+1]+m[8]*g.n[i+2],ny=m[1]*g.n[i]+m[5]*g.n[i+1]+m[9]*g.n[i+2],nz=m[2]*g.n[i]+m[6]*g.n[i+1]+m[10]*g.n[i+2];
     if(nx*cp*ct+ny*cp*st+nz*sp<0)continue;
     const p=[];for(let k=0;k<3;k++){const x=g.p[i+k*3],y=g.p[i+k*3+1],z=g.p[i+k*3+2];p.push(proj([m[0]*x+m[4]*y+m[8]*z+m[12],m[1]*x+m[5]*y+m[9]*z+m[13],m[2]*x+m[6]*y+m[10]*z+m[14]]));}
     if(Math.abs((p[1][0]-p[0][0])*(p[2][1]-p[0][1])-(p[1][1]-p[0][1])*(p[2][0]-p[0][0]))<.015)continue;
     const light=.60+.30*Math.max(0,-.3*nx-.4*ny+.86*nz);const base=ghost?rgbFantasma:j===0||j===6?[84,103,117]:[237,150,45];
     faces.push({points:p,depth:(p[0][2]+p[1][2]+p[2][2])/3,color:`rgb(${base.map(v=>Math.round(v*Math.round(light*12)/12)).join(',')})`,j});
    }
   });faces.sort((a,b)=>a.depth-b.depth);ctx.globalAlpha=ghost?.22:1;
   const layers=new Map();
   for(const f of faces){const key=f.j+":"+f.color;let l=layers.get(key);if(!l){l={color:f.color,depth:0,faces:[]};layers.set(key,l);}l.faces.push(f);l.depth+=f.depth;}
   for(const l of [...layers.values()].sort((a,b)=>a.depth/a.faces.length-b.depth/b.faces.length)){ctx.beginPath();for(const f of l.faces){f.points.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();}ctx.fillStyle=l.color;ctx.fill();}ctx.globalAlpha=1;
   if(robot.marcos)robot.M.slice(0,7).forEach(m=>axes(m,.06));axes(robot.M[7]);return faces;
  };
  this.faces = drawRobot(this.medido, false);
  drawRobot(this.fantasma, true);
  path(this.rastro, "#7a3eb1", 2);
  path(this.camino, "#198ba7", 1.8, [5, 4]);
  const denso = this.puntos.length > 8;   // figuras: puntos macizos chicos, sin aro blanco
  for (const punto of this.puntos) {
    const [x, y] = proj(punto);
    ctx.beginPath();
    ctx.arc(x, y, denso ? 2.5 : 4, 0, Math.PI * 2);
    ctx.fillStyle = denso ? "#0f6f86" : "#198ba7";
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 2;
    ctx.fill();
    if (!denso) ctx.stroke();
  }
 }
}
const r=new Renderer();
self.onmessage=({data:d})=>{
 if(d.tipo==='mallas'){r.geometrias=d.geometrias;r.align=d.align.map(matriz);return;}
 if(d.tipo==='pick'){
  const {x,y}=d,inside=(a,b,c)=>{const s=(p,q)=>(x-q[0])*(p[1]-q[1])-(p[0]-q[0])*(y-q[1]);const v=[s(a,b),s(b,c),s(c,a)];return !(v.some(x=>x<0)&&v.some(x=>x>0));};
  const hit=[...(r.faces||[])].reverse().find(f=>inside(...f.points));if(hit)self.postMessage({junta:Math.max(1,hit.j)});return;
 }
 if(d.tipo==='estado'){
  if(r.canvas.width!==d.w||r.canvas.height!==d.h){r.canvas.width=d.w;r.canvas.height=d.h;}
  Object.assign(r,d);r.dibujar();const bitmap=r.canvas.transferToImageBitmap();self.postMessage({bitmap},[bitmap]);
 }
};
