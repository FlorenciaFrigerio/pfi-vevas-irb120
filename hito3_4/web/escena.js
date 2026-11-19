// Geometría ROS-Industrial; matrices cinemáticas calculadas por Python.
import * as THREE from "three";
import { OrbitControls } from "./vendor/OrbitControls.js";
const NARANJA=0xe89129, CIAN=0x198ba7;
function matriz(a){return new THREE.Matrix4().set(...a);}
function stl(buffer){
 const v=new DataView(buffer), n=v.getUint32(80,true);
 if(buffer.byteLength!==84+50*n) throw Error("STL binario inválido");
 const pos=new Float32Array(n*9), normals=new Float32Array(n*9);
 for(let i=0;i<n;i++){const b=84+i*50;for(let k=0;k<3;k++)for(let d=0;d<3;d++){
 pos[i*9+k*3+d]=v.getFloat32(b+12+k*12+d*4,true);
 normals[i*9+k*3+d]=v.getFloat32(b+d*4,true);}}
 const g=new THREE.BufferGeometry();g.setAttribute("position",new THREE.BufferAttribute(pos,3));g.setAttribute("normal",new THREE.BufferAttribute(normals,3));return g;
}
class RobotVisual{
 constructor(scene,{color,opacidad=1,conMarcos=false}){
 this.grupo=new THREE.Group();scene.add(this.grupo);this.mallas=[];this.marcos=[];this.color=color;this.opacidad=opacidad;
 this.ejesTCP=new THREE.AxesHelper(.075);this.grupo.add(this.ejesTCP);
 if(conMarcos)for(let i=0;i<7;i++){const a=new THREE.AxesHelper(.055);a.visible=false;this.grupo.add(a);this.marcos.push(a);}
 }
 cargar(geometrias,alineacion){this.alineacion=alineacion.map(matriz);geometrias.forEach((g,i)=>{
 const m=new THREE.Mesh(g,new THREE.MeshStandardMaterial({color:this.opacidad<1?this.color:(i===0||i===6?0x4c5e6a:this.color),roughness:.5,metalness:.25,transparent:this.opacidad<1,opacity:this.opacidad,depthWrite:this.opacidad===1}));
 m.matrixAutoUpdate=false;m.userData.junta=Math.max(1,i);this.grupo.add(m);this.mallas.push(m);
 });if(this.M)this.actualizar(this.M);}
 actualizar(M){if(!M||M.length<8)return;this.M=M;this.mallas.forEach((m,i)=>m.matrix.copy(matriz(M[i]).multiply(this.alineacion[i])));this.ejesTCP.matrixAutoUpdate=false;this.ejesTCP.matrix.copy(matriz(M[7]));this.marcos.forEach((m,i)=>{m.matrixAutoUpdate=false;m.matrix.copy(matriz(M[i]));});}
 verMarcos(v){this.marcos.forEach(m=>m.visible=v);}
 fijarColor(hex){if(this._hex===hex)return;this._hex=hex;this.mallas.forEach(m=>m.material.color.setHex(hex));this.color=hex;}
 set visible(v){this.grupo.visible=v;}get visible(){return this.grupo.visible;}
}

function etiqueta(texto) {
  const c = document.createElement("canvas");
  c.width = 64; c.height = 64;
  const g = c.getContext("2d");
  g.fillStyle = "#ffffffdd";
  g.beginPath(); g.arc(32, 32, 22, 0, Math.PI * 2); g.fill();
  g.fillStyle = "#48677e"; g.font = "bold 30px sans-serif"; g.textAlign = "center"; g.textBaseline = "middle";
  g.fillText(texto, 32, 34);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false }));
  s.scale.set(0.035, 0.035, 1);
  s.position.set(0.02, 0.02, 0.03);
  s.renderOrder = 7;
  return s;
}

class EscenaWebGL {
  constructor(contenedor) {
    this.cont = contenedor;
    this.render = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.render.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.render.setClearColor(0xf7f9fc);
    contenedor.appendChild(this.render.domElement);
    this.escena = new THREE.Scene();
    this.camara = new THREE.PerspectiveCamera(42, 1, 0.01, 20);
    this.camara.up.set(0, 0, 1);
    this.controles = new OrbitControls(this.camara, this.render.domElement);
    this.controles.enableDamping = true;
    this.vistaInicial();

    this.escena.add(new THREE.HemisphereLight(0xffffff, 0xc3d0dc, 2.2));
    const luz = new THREE.DirectionalLight(0xffffff, 1.6);
    luz.position.set(1.5, -1.2, 2.2);
    this.escena.add(luz);

    const grilla = new THREE.GridHelper(1.6, 16, 0xb5c4d3, 0xdce5ee);
    grilla.rotation.x = Math.PI / 2;
    this.escena.add(grilla);
    const ejes = new THREE.AxesHelper(0.25);
    this.escena.add(ejes);
    for (const [t, p] of [["X", [0.28, 0, 0]], ["Y", [0, 0.28, 0]], ["Z", [0, 0, 0.28]]]) {
      const e = etiqueta(t);
      e.position.set(...p);
      e.scale.set(0.05, 0.05, 1);
      this.escena.add(e);
    }
    // Plano de altura mínima
    this.planoZmin = new THREE.Mesh(
      new THREE.PlaneGeometry(1.4, 1.4),
      new THREE.MeshBasicMaterial({ color: 0xff5a5f, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false })
    );
    this.planoZmin.position.z = 0.08;
    this.escena.add(this.planoZmin);

    this.medido = new RobotVisual(this.escena, { color: NARANJA, conMarcos: true });
    this.fantasma = new RobotVisual(this.escena, { color: CIAN, opacidad: 0.32 });
    this.fantasma.visible = false;
    const nombres=["base_link",...Array.from({length:6},(_,i)=>`link_${i+1}`)];
    Promise.all([fetch("modelo/alineacion.json").then(r=>r.json()),...nombres.map(n=>fetch(`modelo/${n}.stl`).then(r=>{if(!r.ok)throw Error(n);return r.arrayBuffer();}).then(stl))]).then(([alineacion,...g])=>{
      this.medido.cargar(g,alineacion);this.fantasma.cargar(g,alineacion);
      document.querySelector("#modelo-estado").textContent="IRB 120 · ROS-Industrial";
    }).catch(e=>{document.querySelector("#modelo-estado").textContent="Error al cargar 3D";console.error(e);});
    this.tieneArrastre = true;
    this._iniciarGizmo();
    const ray=new THREE.Raycaster(), mouse=new THREE.Vector2();let inicio;
    this.render.domElement.addEventListener("pointerdown",e=>inicio=[e.clientX,e.clientY]);
    this.render.domElement.addEventListener("pointerup",e=>{
      if(this.gz.bloqueaClic||!inicio||Math.hypot(e.clientX-inicio[0],e.clientY-inicio[1])>5)return;
      const b=this.render.domElement.getBoundingClientRect();mouse.set(2*(e.clientX-b.left)/b.width-1,1-2*(e.clientY-b.top)/b.height);ray.setFromCamera(mouse,this.camara);
      const hit=ray.intersectObjects(this.medido.mallas)[0];if(hit)window.dispatchEvent(new CustomEvent("vevas:joint",{detail:{junta:hit.object.userData.junta}}));
    });

    // Camino planificado y rastro medido del TCP
    this.camino = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: CIAN, dashSize: 0.012, gapSize: 0.008 }));
    this.escena.add(this.camino);
    this.rastroMax = 600;
    this.rastroPos = new Float32Array(this.rastroMax * 3);
    this.rastroN = 0;
    const gr = new THREE.BufferGeometry();
    gr.setAttribute("position", new THREE.BufferAttribute(this.rastroPos, 3));
    gr.setDrawRange(0, 0);
    this.rastro = new THREE.Line(gr, new THREE.LineBasicMaterial({ color: 0x637eaf }));
    this.escena.add(this.rastro);
    this.objetivo = new THREE.Mesh(new THREE.SphereGeometry(0.012, 20, 14), new THREE.MeshBasicMaterial({ color: CIAN }));
    this.objetivo.visible = false;
    this.escena.add(this.objetivo);

    this._redimensionar();
    new ResizeObserver(() => this._redimensionar()).observe(contenedor);
    const bucle = () => {
      this.controles.update();
      this._cuadroGizmo();
      this.render.render(this.escena, this.camara);
      if (this.alCuadro) this.alCuadro();
      requestAnimationFrame(bucle);
    };
    bucle();
  }

  vistaInicial() {
    this.camara.position.set(1.15, -1.45, 0.9);
    this.controles.target.set(0.2, 0, 0.3);
    this.controles.update();
  }

  _redimensionar() {
    const w = this.cont.clientWidth || 600, h = this.cont.clientHeight || 400;
    this.render.setSize(w, h);
    this.camara.aspect = w / h;
    this.camara.updateProjectionMatrix();
  }


  // ---- Flechas de arrastre (X rojo, Y verde, Z azul) sobre la punta
  _iniciarGizmo() {
    const gz = (this.gz = { grupo: new THREE.Group(), flechas: [], pick: [], pos: new THREE.Vector3(), visible: false, arr: null, hover: -1, bloqueaClic: false, cb: {} });
    gz.grupo.visible = false;
    gz.grupo.renderOrder = 20;
    this.escena.add(gz.grupo);
    const defs = [[[1, 0, 0], 0xe5483f], [[0, 1, 0], 0x2fb36b], [[0, 0, 1], 0x2f6fe0]];
    const Y = new THREE.Vector3(0, 1, 0);
    defs.forEach(([d, col], i) => {
      const dir = new THREE.Vector3(...d);
      const g = new THREE.Group();
      g.quaternion.setFromUnitVectors(Y, dir);
      const mat = new THREE.MeshBasicMaterial({ color: col, depthTest: false, transparent: true, opacity: 0.95 });
      const eje = new THREE.Mesh(new THREE.CylinderGeometry(0.0035, 0.0035, 0.11, 10), mat);
      eje.position.y = 0.075;
      const punta = new THREE.Mesh(new THREE.ConeGeometry(0.012, 0.04, 16), mat);
      punta.position.y = 0.15;
      const pick = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.2, 8), new THREE.MeshBasicMaterial({ transparent: true, opacity: 0, depthTest: false, depthWrite: false }));
      pick.position.y = 0.1;
      pick.userData.eje = i;
      [eje, punta, pick].forEach((m) => (m.renderOrder = 20));
      g.add(eje, punta, pick);
      gz.grupo.add(g);
      gz.flechas.push({ g, mat, col, dir });
      gz.pick.push(pick);
    });
    const centro = new THREE.Mesh(new THREE.SphereGeometry(0.009, 14, 10), new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false }));
    centro.renderOrder = 21;
    gz.grupo.add(centro);

    const ray = new THREE.Raycaster(), m = new THREE.Vector2();
    const rayo = (e) => {
      const b = this.render.domElement.getBoundingClientRect();
      m.set((2 * (e.clientX - b.left)) / b.width - 1, 1 - (2 * (e.clientY - b.top)) / b.height);
      ray.setFromCamera(m, this.camara);
      return ray;
    };
    const tras = (r, P0, d) => { // parámetro t del punto de la recta P0 + t·d más cercano al rayo
      const w = P0.clone().sub(r.ray.origin), b = d.dot(r.ray.direction);
      const den = 1 - b * b;
      if (den < 1e-3) return null; // eje casi paralelo a la vista
      return (b * r.ray.direction.dot(w) - d.dot(w)) / den;
    };
    const pintar = (i) => {
      gz.hover = i;
      gz.flechas.forEach((a, k) => a.mat.color.setHex(k === i || (gz.arr && gz.arr.i === k) ? 0xffd23f : a.col));
    };
    this.cont.addEventListener("pointerdown", (e) => {
      if (!gz.visible || e.button !== 0) return;
      const hit = rayo(e).intersectObjects(gz.pick, false)[0];
      if (!hit) return;
      if (gz.cb.alIniciar && gz.cb.alIniciar() === false) { e.stopPropagation(); return; }
      const i = hit.object.userData.eje, P0 = gz.pos.clone(), d = gz.flechas[i].dir;
      const t0 = tras(rayo(e), P0, d);
      if (t0 == null) return;
      gz.arr = { i, P0, d, t0, id: e.pointerId };
      gz.bloqueaClic = true;
      this.controles.enabled = false;
      pintar(i);
      e.stopPropagation();
      e.preventDefault();
    }, true);
    window.addEventListener("pointermove", (e) => {
      if (gz.arr) {
        const t = tras(rayo(e), gz.arr.P0, gz.arr.d);
        if (t == null) return;
        const dt = Math.max(-1.5, Math.min(1.5, t - gz.arr.t0));
        gz.pos.copy(gz.arr.P0).addScaledVector(gz.arr.d, dt);
        if (gz.cb.alArrastrar) gz.cb.alArrastrar({ pos: [gz.pos.x, gz.pos.y, gz.pos.z], eje: gz.arr.i, delta: dt });
        return;
      }
      if (!gz.visible || e.target !== this.render.domElement) { if (gz.hover >= 0) pintar(-1); return; }
      const hit = rayo(e).intersectObjects(gz.pick, false)[0];
      const i = hit ? hit.object.userData.eje : -1;
      if (i !== gz.hover) { pintar(i); this.render.domElement.style.cursor = i >= 0 ? "grab" : ""; }
    });
    const soltar = () => {
      if (!gz.arr) return;
      gz.arr = null;
      this.controles.enabled = true;
      pintar(-1);
      setTimeout(() => (gz.bloqueaClic = false), 60);
      if (gz.cb.alSoltar) gz.cb.alSoltar();
    };
    window.addEventListener("pointerup", soltar);
    window.addEventListener("pointercancel", soltar);
  }

  _cuadroGizmo() {
    const gz = this.gz;
    if (!gz.visible) { gz.grupo.visible = false; return; }
    gz.grupo.visible = true;
    gz.grupo.position.copy(gz.pos);
    gz.grupo.scale.setScalar(Math.max(0.35, this.camara.position.distanceTo(gz.pos) / 2.1));
  }

  // pos en metros (base del robot) o null para ocultar. Mientras se arrastra no se mueve sola.
  fijarGizmo(pos) {
    const gz = this.gz;
    if (!pos) { if (!gz.arr) gz.visible = false; return; }
    gz.visible = true;
    if (!gz.arr) gz.pos.set(pos[0], pos[1], pos[2]);
  }
  gizmoCallbacks(cb) { Object.assign(this.gz.cb, cb); }
  arrastrando() { return !!this.gz.arr; }

  // Proyección a píxeles del contenedor (null si está detrás de la cámara)
  proyectar(p) {
    const v = new THREE.Vector3(...p).project(this.camara);
    if (v.z > 1) return null;
    return { x: ((v.x + 1) / 2) * this.cont.clientWidth, y: ((1 - v.y) / 2) * this.cont.clientHeight };
  }
  // Ayuda para pruebas: posición en pantalla (coordenadas de ventana) del centro y de la mitad de cada flecha
  gizmoPantalla() {
    const gz = this.gz, b = this.render.domElement.getBoundingClientRect();
    this.camara.updateMatrixWorld();
    const esc = gz.grupo.scale.x;
    const aPant = (v) => { const p = v.clone().project(this.camara); return { x: b.left + ((p.x + 1) / 2) * b.width, y: b.top + ((1 - p.y) / 2) * b.height }; };
    const out = { visible: gz.visible, centro: aPant(gz.pos) };
    ["x", "y", "z"].forEach((k, i) => (out[k] = aPant(gz.pos.clone().addScaledVector(gz.flechas[i].dir, 0.1 * esc))));
    return out;
  }

  fijarZmin(z) { this.planoZmin.position.z = z; }

  agregarRastro(p) {
    if (!p) return;
    const n = this.rastroN;
    if (n > 0) {
      const i = (n - 1) * 3;
      const d = Math.hypot(p[0] - this.rastroPos[i], p[1] - this.rastroPos[i + 1], p[2] - this.rastroPos[i + 2]);
      if (d < 0.0015) return;
    }
    if (n >= this.rastroMax) {
      this.rastroPos.copyWithin(0, 3);
      this.rastroN = this.rastroMax - 1;
    }
    this.rastroPos.set(p, this.rastroN * 3);
    this.rastroN++;
    this.rastro.geometry.setDrawRange(0, this.rastroN);
    this.rastro.geometry.attributes.position.needsUpdate = true;
  }

  fijarCamino(puntos) {
    const g = new THREE.BufferGeometry().setFromPoints((puntos || []).map((p) => new THREE.Vector3(...p)));
    this.camino.geometry.dispose();
    this.camino.geometry = g;
    this.camino.computeLineDistances();
    if (puntos && puntos.length) {
      this.objetivo.position.set(...puntos[puntos.length - 1]);
      this.objetivo.visible = true;
    } else {
      this.objetivo.visible = false;
    }
  }

  captura() { return this.render.domElement.toDataURL("image/png"); }
}

// Renderizado alternativo de las mismas mallas para equipos sin WebGL.
class EscenaCanvas {
 constructor(cont){
  this.cont=cont;this.canvas=document.createElement('canvas');cont.appendChild(this.canvas);this.ctx=this.canvas.getContext('2d');
  this.worker=new Worker('escena_worker.js',{type:'module'});
  this.worker.onmessage=({data:d})=>{if(d.bitmap){this.ctx.clearRect(0,0,this.w,this.h);this.ctx.drawImage(d.bitmap,0,0,this.w,this.h);d.bitmap.close();this.ocupado=false;}if(d.junta)window.dispatchEvent(new CustomEvent('vevas:joint',{detail:{junta:d.junta}}));};
  this.worker.onerror=e=>{this.ocupado=false;document.querySelector('#modelo-estado').textContent='Error de renderizado compatible';console.error(e);};
  this.theta=-1.0;this.phi=.40;this.zoom=1;this.camino=[];this.rastro=[];this.planoZmin={visible:true,z:.08};
  const robot=()=>({M:null,visible:true,marcos:false,actualizar(M){this.M=M;},verMarcos(v){this.marcos=v;},fijarColor(){}});
  this.medido=robot();this.fantasma=robot();this.fantasma.visible=false;
  Promise.all([fetch('modelo/alineacion.json').then(r=>r.json()),...['base_link',...Array.from({length:6},(_,i)=>`link_${i+1}`)].map(n=>fetch(`modelo/${n}.stl`).then(r=>r.arrayBuffer()).then(stl))]).then(([a,...g])=>{
   this.align=a.map(matriz);this.geometrias=g.map(g=>({p:g.attributes.position.array,n:g.attributes.normal.array}));this.worker.postMessage({tipo:'mallas',geometrias:this.geometrias,align:a});document.querySelector('#modelo-estado').textContent='IRB 120 · vista compatible';
  }).catch(e=>{document.querySelector('#modelo-estado').textContent='Error de mallas';console.error(e);});
  let down,last,moved;
  this.canvas.addEventListener('pointerdown',e=>{down=true;moved=false;last=[e.clientX,e.clientY];this.canvas.setPointerCapture(e.pointerId);});
  this.canvas.addEventListener('pointermove',e=>{if(!down)return;const dx=e.clientX-last[0],dy=e.clientY-last[1];if(Math.abs(dx)+Math.abs(dy)>1)moved=true;this.theta-=dx*.008;this.phi=Math.max(.04,Math.min(1.45,this.phi+dy*.008));last=[e.clientX,e.clientY];});
  this.canvas.addEventListener('pointerup',e=>{down=false;if(moved)return;const r=this.canvas.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top;
   this.worker.postMessage({tipo:'pick',x,y});
  });
  this.canvas.addEventListener('wheel',e=>{e.preventDefault();this.zoom=Math.max(.6,Math.min(2.8,this.zoom*Math.exp(-e.deltaY*.001)));},{passive:false});
  new ResizeObserver(()=>{this.w=cont.clientWidth;this.h=cont.clientHeight;const d=Math.min(devicePixelRatio,2);this.canvas.width=this.w*d;this.canvas.height=this.h*d;this.canvas.style.width=this.w+'px';this.canvas.style.height=this.h+'px';this.ctx.setTransform(d,0,0,d,0,0);}).observe(cont);
  this.timer=setInterval(()=>this.dibujar(),240);
 }
 tieneArrastre=false;fijarGizmo(){}gizmoCallbacks(){}arrastrando(){return false;}proyectar(){return null;}gizmoPantalla(){return null;}
 vistaInicial(){this.theta=-1.0;this.phi=.40;this.zoom=1;}
 fijarZmin(z){this.planoZmin.z=z;}
 fijarCamino(p){this.camino=p||[];}
 agregarRastro(p){if(!p)return;if(this.rastro.length&&Math.hypot(...p.map((x,i)=>x-this.rastro.at(-1)[i]))<.002)return;this.rastro.push(p);if(this.rastro.length>600)this.rastro.shift();}
 captura(){return this.canvas.toDataURL('image/png');}
 dibujar(){
  if(!this.w||!this.h||!this.geometrias||this.ocupado)return;
  this.ocupado=true;
  this.worker.postMessage({tipo:'estado',w:Math.round(this.w),h:Math.round(this.h),theta:this.theta,phi:this.phi,zoom:this.zoom,medido:{M:this.medido.M,visible:this.medido.visible,marcos:this.medido.marcos},fantasma:{M:this.fantasma.M,visible:this.fantasma.visible,marcos:false},planoZmin:this.planoZmin,camino:this.camino,rastro:this.rastro});
 }

}
export class Escena{
 constructor(cont){const probe=document.createElement('canvas');const gl=probe.getContext('webgl2');if(gl){gl.getExtension('WEBGL_lose_context')?.loseContext();return new EscenaWebGL(cont);}return new EscenaCanvas(cont);}
}
