// Geometría ROS-Industrial; matrices cinemáticas calculadas por Python.
import * as THREE from "three";
import { OrbitControls } from "./vendor/OrbitControls.js";
import { interpolarMarcos } from "./orientacion.js";
export { interpolarMarcos } from "./orientacion.js";
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

// Controles del TCP en un lienzo superpuesto, con las proyecciones del visor activo.
const EJES = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1)];
const COLORES = ["#cf443c", "#208251", "#2d68c6"];
const NOMBRES = ["X", "Y", "Z"];
const PASOS_ANILLO = 64;
const ASIDERO_LIBRE = 9;
const LIMITE_ARRASTRE_M = 1.5;
const COLOR_TRAZA = 0x7a3eb1;
const MAX_MUESTRAS_TRAZA = 2048;

class TrazaTCP {
  puntos = [];

  agregar(punto) {
    if (punto?.length !== 3 || !punto.every(Number.isFinite)) return false;
    const ultimo = this.puntos.at(-1);
    if (ultimo && Math.hypot(...punto.map((x, i) => x - ultimo[i])) < 0.00025) return false;
    if (this.puntos.length >= MAX_MUESTRAS_TRAZA) {
      // Reduce el historial denso y conserva ambos extremos de la traza.
      this.puntos = this.puntos.filter((_, i, ps) => i % 2 === 0 || i === ps.length - 1);
    }
    this.puntos.push([...punto]);
    return true;
  }

  limpiar() { this.puntos = []; }
}

class MedidoVisual {
  muestras = [];

  recibir(M, reiniciar) {
    if (reiniciar) this.muestras = [];
    if (!M || M.length < 8) return;
    const ultimo = this.muestras.at(-1)?.M;
    const igual = ultimo && ultimo.every((F, i) => F.every((v, j) => v === M[i][j]));
    this.muestras.push({ M: igual ? ultimo : M, t: performance.now() });
    if (this.muestras.length > 10) this.muestras.shift();
  }

  cuadro() {
    const muestras = this.muestras;
    const t = performance.now() - 85;
    while (muestras.length > 1 && muestras[1].t <= t) muestras.shift();
    const [a, b] = muestras;
    if (!a || !b || a.t >= t) return a?.M;
    return interpolarMarcos(a.M, b.M, (t - a.t) / (b.t - a.t));
  }
}

function cajasEntorno(entorno = {}) {
  const mesa = entorno.mesa;
  const manuales = (entorno.cajas || []).filter((c) => !mesa || c.origen !== "mesa");
  return [
    ...manuales.map((c) => ({ ...c, confirmado: c.confirmado ?? entorno.confirmado })),
    ...(mesa?.cajas || []).map((c) => ({ ...c, confirmado: mesa.confirmado })),
    ...(mesa?.geometria_visual || []),
  ].flatMap((c) => {
    const { min_mm: min, max_mm: max } = c;
    if (min?.length !== 3 || max?.length !== 3 || !min.every((x, i) => Number.isFinite(x) && max[i] > x && Number.isFinite(max[i]))) return [];
    return [{ min: min.map((x) => x / 1000), max: max.map((x) => x / 1000), confirmado: c.confirmado }];
  });
}

class GizmoTCP {
  constructor(visor, superficie) {
    this.visor = visor;
    this.superficie = superficie;
    this.canvas = document.createElement("canvas");
    this.canvas.className = "lienzo-gizmo";
    this.canvas.setAttribute("aria-hidden", "true");
    this.ctx = this.canvas.getContext("2d");
    visor.cont.append(this.canvas);
    this.modo = "traslacion";
    this.cb = {};
    this.hover = -1;
    visor.cont.addEventListener("pointerdown", (e) => this.iniciar(e), true);
    window.addEventListener("pointermove", (e) => this.mover(e));
    window.addEventListener("pointerup", (e) => this.soltar(e));
    window.addEventListener("pointercancel", (e) => this.soltar(e));
    superficie.addEventListener("lostpointercapture", (e) => this.soltar(e));
    window.addEventListener("blur", () => this.soltar());
  }

  fijar(pos, orientacion) {
    if (!this.arr) this.pos = pos && new THREE.Vector3(...pos);
    this.orientacion = orientacion;
  }

  coordenadas(e) {
    const b = this.superficie.getBoundingClientRect();
    return { x: e.clientX - b.left, y: e.clientY - b.top };
  }

  elegir(p) {
    if (!this.pos || !this.centroPantalla || !this.trazos) return null;
    const { x, y } = this.centroPantalla;
    if (this.modo === "traslacion" && Math.hypot(p.x - x, p.y - y) <= ASIDERO_LIBRE) return { libre: true };
    let mejor = null, distancia = 8;
    if (this.modo === "traslacion") {
      distancia = 14;
      this.trazos.forEach((trazo, eje) => {
        if (!trazo.length) return;
        const [a, b] = trazo;
        const d = Math.hypot(p.x - b.x, p.y - b.y);
        if (d < distancia) {
          distancia = d;
          mejor = { eje, tangente: new THREE.Vector2(b.x - a.x, b.y - a.y).normalize() };
        }
      });
      if (mejor) return mejor;
      distancia = 8;
    }
    this.trazos?.forEach((trazo, eje) => {
      for (let i = 1; i < trazo.length; i++) {
        const a = trazo[i - 1], b = trazo[i];
        const dx = b.x - a.x, dy = b.y - a.y, largo = Math.hypot(dx, dy);
        if (largo < 0.1) continue;
        const t = THREE.MathUtils.clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / (largo * largo), 0, 1);
        const d = Math.hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
        if (d < distancia) {
          distancia = d;
          mejor = { eje, tangente: { x: dx / largo, y: dy / largo } };
        }
      }
    });
    return mejor;
  }

  parametroTraslacion(rayo, centro, eje) {
    const w = centro.clone().sub(rayo.origin), b = eje.dot(rayo.direction);
    const den = 1 - b * b;
    return den < 1e-3 ? null : (b * rayo.direction.dot(w) - eje.dot(w)) / den;
  }

  interseccionVertical(rayo, centro, normal) {
    if (Math.abs(normal.dot(rayo.direction)) < 0.17) return null;
    return rayo.intersectPlane(new THREE.Plane().setFromNormalAndCoplanarPoint(normal, centro), new THREE.Vector3());
  }

  iniciarLibre(rayo, centro) {
    const vista = this.visor.rayoGizmo(this.centroPantalla).direction;
    const normal = new THREE.Vector3(vista.x, vista.y, 0).normalize();
    const derecha = normal.lengthSq() > 0 ? new THREE.Vector3(normal.y, -normal.x, 0) : EJES[0];
    return { normal, derecha, apoyo: this.interseccionVertical(rayo, centro, normal) };
  }

  anguloEnPlano(rayo, centro, eje, radio) {
    const den = EJES[eje].dot(rayo.direction);
    if (Math.abs(den) < 0.18) return null;
    const t = centro.clone().sub(rayo.origin).dot(EJES[eje]) / den;
    const p = rayo.at(t, new THREE.Vector3()).sub(centro);
    const u = p.dot(EJES[(eje + 1) % 3]), v = p.dot(EJES[(eje + 2) % 3]);
    return Math.hypot(u, v) < radio / 5 ? null : Math.atan2(v, u);
  }

  iniciar(e) {
    if (this.arr || e.target !== this.superficie || e.button !== 0) return;
    const punto = this.coordenadas(e), elegido = this.elegir(punto);
    if (!elegido) return;
    const { libre, eje, tangente } = elegido, centro = this.pos.clone(), modo = this.modo;
    const radio = this.radios[eje], rayo = this.visor.rayoGizmo(punto);
    const planoLibre = libre && this.iniciarLibre(rayo, centro);
    const ultimo = libre ? null : modo === "rotacion" ? this.anguloEnPlano(rayo, centro, eje, radio) :
      this.ejesDeCanto[eje] ? null : this.parametroTraslacion(rayo, centro, EJES[eje]);
    e.stopPropagation();
    e.preventDefault();
    if (this.cb.alIniciar?.() === false) return;
    this.arr = { id: e.pointerId, modo, eje, tangente, centro, punto, radio, ultimo, planoLibre,
      escala: this.visor.escalaGizmo(centro), usaPlano: ultimo != null, delta: 0 };
    this.bloqueaClic = true;
    this.visor.bloquearOrbita(true);
    this.superficie.setPointerCapture(e.pointerId);
    this.superficie.style.cursor = "grabbing";
  }

  mover(e) {
    const a = this.arr;
    if (!a) {
      const hit = e.target === this.superficie && this.elegir(this.coordenadas(e));
      this.hover = hit ? (hit.libre ? "libre" : hit.eje) : -1;
      this.superficie.style.cursor = hit ? "grab" : "";
      return;
    }
    if (e.pointerId !== a.id) return;
    const p = this.coordenadas(e), rayo = this.visor.rayoGizmo(p);
    if (a.modo === "traslacion") {
      let desplazamiento;
      if (a.planoLibre) {
        const { normal, derecha, apoyo } = a.planoLibre;
        const puntoPlano = apoyo && this.interseccionVertical(rayo, a.centro, normal);
        if (puntoPlano) desplazamiento = puntoPlano.sub(apoyo);
        else if (apoyo) return;
        else desplazamiento = derecha.clone().multiplyScalar((p.x - a.punto.x) * a.escala)
          .addScaledVector(EJES[2], (a.punto.y - p.y) * a.escala);
      } else {
        let delta;
        if (a.usaPlano) {
          const t = this.parametroTraslacion(rayo, a.centro, EJES[a.eje]);
          if (t == null) return;
          delta = t - a.ultimo;
        } else {
          delta = ((p.x - a.punto.x) * a.tangente.x + (p.y - a.punto.y) * a.tangente.y) * a.escala;
        }
        desplazamiento = EJES[a.eje].clone().multiplyScalar(delta);
      }
      this.pos.copy(a.centro).add(desplazamiento.clampLength(0, LIMITE_ARRASTRE_M));
      this.cb.alArrastrar?.({ pos: this.pos.toArray() });
      return;
    }
    if (a.usaPlano) {
      const angulo = this.anguloEnPlano(rayo, a.centro, a.eje, a.radio);
      if (angulo == null) return;
      const diferencia = angulo - a.ultimo;
      a.delta += Math.atan2(Math.sin(diferencia), Math.cos(diferencia));
      a.ultimo = angulo;
    } else {
      // Un anillo de canto se arrastra sobre su tangente visible.
      a.delta = ((p.x - a.punto.x) * a.tangente.x + (p.y - a.punto.y) * a.tangente.y) / this.radioPx;
    }
    this.cb.alArrastrar?.({ eje: a.eje, angulo: a.delta });
  }

  soltar(e) {
    if (!this.arr || (e && e.pointerId !== this.arr.id)) return;
    const id = this.arr.id;
    this.arr = null;
    if (this.superficie.hasPointerCapture(id)) this.superficie.releasePointerCapture(id);
    this.visor.bloquearOrbita(false);
    this.superficie.style.cursor = "";
    this.hover = -1;
    setTimeout(() => { if (!this.arr) this.bloqueaClic = false; }, 0);
    this.cb.alSoltar?.();
  }

  dibujar() {
    const { ctx, canvas, pos } = this, { clientWidth: w, clientHeight: h } = this.visor.cont;
    const d = Math.min(window.devicePixelRatio, 2);
    if (canvas.width !== Math.round(w * d) || canvas.height !== Math.round(h * d)) {
      canvas.width = Math.round(w * d);
      canvas.height = Math.round(h * d);
    }
    ctx.setTransform(d, 0, 0, d, 0, 0);
    ctx.clearRect(0, 0, w, h);
    this.trazos = null;
    const centro = this.centroPantalla = pos && this.visor.proyectar(pos.toArray());
    if (!centro || !w || !h) return;
    const proyectar = (v) => this.visor.proyectar(v.toArray());
    const rotacion = this.modo === "rotacion";
    const radioPx = this.radioPx = Math.min(46, w * 0.13, h * 0.15);
    const escala = this.visor.escalaGizmo(pos) * radioPx;
    this.radios = EJES.map((_, i) => (0.88 + i * 0.13) * escala);
    this.ejesDeCanto = [false, false, false];
    const trazos = this.trazos = EJES.map((eje, i) => {
      if (!rotacion) {
        const punta = proyectar(pos.clone().addScaledVector(eje, 1.25 * escala));
        if (!punta) return [];
        let dx = punta.x - centro.x, dy = punta.y - centro.y;
        let largo = Math.hypot(dx, dy);
        if (largo < 16) {
          this.ejesDeCanto[i] = true;
          if (largo < 2) {
            [dx, dy] = [[-1, 0.5], [1, 0.5], [0, -1]][i];
            largo = Math.hypot(dx, dy);
          }
          const longitud = Math.max(24, radioPx * 0.66);
          punta.x = centro.x + dx / largo * longitud;
          punta.y = centro.y + dy / largo * longitud;
        }
        return [{ x: centro.x + 10 * dx / largo, y: centro.y + 10 * dy / largo }, punta];
      }
      const r = this.radios[i], u = EJES[(i + 1) % 3], v = EJES[(i + 2) % 3];
      return Array.from({ length: PASOS_ANILLO + 1 }, (_, k) => {
        const a = k * Math.PI * 2 / PASOS_ANILLO;
        return proyectar(pos.clone().addScaledVector(u, r * Math.cos(a)).addScaledVector(v, r * Math.sin(a)));
      });
    }).map((trazo) => trazo.every(Boolean) ? trazo : []);
    const linea = (puntos, color, ancho) => {
      ctx.beginPath();
      puntos.forEach(({ x, y }, i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y));
      ctx.strokeStyle = color;
      ctx.lineWidth = ancho;
      ctx.stroke();
    };
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    if (rotacion && this.orientacion) {
      this.orientacion.forEach((_, i) => {
        const d = new THREE.Vector3(...this.orientacion.map((fila) => fila[i]));
        const punta = proyectar(pos.clone().addScaledVector(d, escala * 0.57));
        if (!punta) return;
        linea([centro, punta], "#ffffff", 4);
        linea([centro, punta], COLORES[i], 2);
      });
    }
    trazos.forEach((trazo, i) => {
      if (!trazo.length) return;
      const resaltado = (this.arr ? this.arr.eje : this.hover) === i;
      const color = COLORES[i];
      linea(trazo, "#fff", resaltado ? 7 : 5);
      linea(trazo, color, resaltado ? 4 : 2.5);
      if (!rotacion) {
        const [a, b] = trazo, angulo = Math.atan2(b.y - a.y, b.x - a.x);
        ctx.beginPath();
        ctx.moveTo(b.x, b.y);
        ctx.lineTo(b.x - 12 * Math.cos(angulo - 0.36), b.y - 12 * Math.sin(angulo - 0.36));
        ctx.lineTo(b.x - 12 * Math.cos(angulo + 0.36), b.y - 12 * Math.sin(angulo + 0.36));
        ctx.closePath();
        ctx.fillStyle = color;
        ctx.fill();
        ctx.font = "bold 11px sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        const x = b.x + 10 * Math.cos(angulo), y = b.y + 10 * Math.sin(angulo);
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 3;
        ctx.strokeText(NOMBRES[i], x, y);
        ctx.fillText(NOMBRES[i], x, y);
      }
    });
    ctx.beginPath();
    ctx.arc(centro.x, centro.y, rotacion ? 3 : ASIDERO_LIBRE - 1, 0, Math.PI * 2);
    ctx.fillStyle = this.arr?.planoLibre || this.hover === "libre" ? "#dff2f5" : "#fff";
    ctx.fill();
    ctx.strokeStyle = "#48677e";
    ctx.lineWidth = 1;
    ctx.stroke();
    if (!rotacion) {
      linea([{ x: centro.x - 4, y: centro.y }, { x: centro.x + 4, y: centro.y }], "#48677e", 1.4);
      linea([{ x: centro.x, y: centro.y - 4 }, { x: centro.x, y: centro.y + 4 }], "#48677e", 1.4);
    }
  }

  pantalla() {
    const b = this.superficie.getBoundingClientRect();
    const ventana = (p) => ({ x: b.left + p.x, y: b.top + p.y });
    const centro = this.pos && this.visor.proyectar(this.pos.toArray());
    if (!centro || !this.trazos) return { visible: false };
    const out = { visible: true, modo: this.modo, centro: ventana(centro) };
    if (this.modo === "traslacion") out.libre = out.centro;
    this.trazos.forEach((trazo, i) => {
      if (!trazo.length) return;
      const k = NOMBRES[i].toLowerCase();
      if (this.modo === "rotacion") out["r" + k] = trazo.map(ventana);
      else out[k] = ventana(trazo[1]);
    });
    return out;
  }
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
    this.entorno = new THREE.Group();
    this.escena.add(this.entorno);

    this.medido = new RobotVisual(this.escena, { color: NARANJA, conMarcos: true });
    this.medidoVisual = new MedidoVisual();
    this.fantasma = new RobotVisual(this.escena, { color: CIAN, opacidad: 0.32 });
    this.fantasma.visible = false;
    const nombres=["base_link",...Array.from({length:6},(_,i)=>`link_${i+1}`)];
    Promise.all([fetch("modelo/alineacion.json").then(r=>r.json()),...nombres.map(n=>fetch(`modelo/${n}.stl`).then(r=>{if(!r.ok)throw Error(n);return r.arrayBuffer();}).then(stl))]).then(([alineacion,...g])=>{
      this.medido.cargar(g,alineacion);this.fantasma.cargar(g,alineacion);
      document.querySelector("#modelo-estado").textContent="IRB 120 · ROS-Industrial";
    }).catch(e=>{document.querySelector("#modelo-estado").textContent="Error al cargar 3D";console.error(e);});
    this.gz = new GizmoTCP(this, this.render.domElement);
    const ray=new THREE.Raycaster(), mouse=new THREE.Vector2();let inicio;
    this.render.domElement.addEventListener("pointerdown",e=>inicio=[e.clientX,e.clientY]);
    this.render.domElement.addEventListener("pointerup",e=>{
      if(this.gz.bloqueaClic||!inicio||Math.hypot(e.clientX-inicio[0],e.clientY-inicio[1])>5)return;
      const b=this.render.domElement.getBoundingClientRect();mouse.set(2*(e.clientX-b.left)/b.width-1,1-2*(e.clientY-b.top)/b.height);ray.setFromCamera(mouse,this.camara);
      const hit=ray.intersectObjects(this.medido.mallas)[0];if(hit)window.dispatchEvent(new CustomEvent("vevas:joint",{detail:{junta:hit.object.userData.junta}}));
    });

    // Camino planificado y rastro medido del TCP
    this.camino = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({
      color: CIAN, dashSize: 0.012, gapSize: 0.008, transparent: true, depthTest: false, depthWrite: false,
    }));
    this.camino.renderOrder = 10;
    this.escena.add(this.camino);
    this.traza = new TrazaTCP();
    this.rastroPos = new Float32Array(MAX_MUESTRAS_TRAZA * 3);
    const gr = new THREE.BufferGeometry();
    gr.setAttribute("position", new THREE.BufferAttribute(this.rastroPos, 3).setUsage(THREE.DynamicDrawUsage));
    gr.setDrawRange(0, 0);
    this.rastro = new THREE.Line(gr, new THREE.LineBasicMaterial({
      color: COLOR_TRAZA, transparent: true, depthTest: false, depthWrite: false,
    }));
    this.rastro.frustumCulled = false;
    this.rastro.renderOrder = 9;
    this.escena.add(this.rastro);

    const marca = document.createElement("canvas");
    marca.width = marca.height = 32;
    const marcaCtx = marca.getContext("2d");
    marcaCtx.beginPath();
    marcaCtx.arc(16, 16, 13, 0, Math.PI * 2);
    marcaCtx.fillStyle = "#fff";
    marcaCtx.fill();
    marcaCtx.beginPath();
    marcaCtx.arc(16, 16, 9, 0, Math.PI * 2);
    marcaCtx.fillStyle = "#198ba7";
    marcaCtx.fill();
    const texturaPunto = new THREE.CanvasTexture(marca);
    texturaPunto.colorSpace = THREE.SRGBColorSpace;
    this.puntos = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({
      map: texturaPunto, size: 11, sizeAttenuation: false, transparent: true, depthTest: false, depthWrite: false,
    }));
    this.puntos.renderOrder = 11;
    this.escena.add(this.puntos);

    this._redimensionar();
    new ResizeObserver(() => this._redimensionar()).observe(contenedor);
    const bucle = () => {
      if (!this.gz.arr) this.controles.update();
      const medido = this.medidoVisual.cuadro();
      if (medido && medido !== this.medido.M) this.medido.actualizar(medido);
      this.render.render(this.escena, this.camara);
      this.alCuadro?.();
      this.gz.dibujar();
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


  fijarGizmo(pos, orientacion) { this.gz.fijar(pos, orientacion); }
  fijarModoGizmo(modo) { this.gz.modo = modo; }
  gizmoCallbacks(cb) { this.gz.cb = cb; }
  gizmoPantalla() { return this.gz.pantalla(); }
  bloquearOrbita(bloquear) { this.controles.enabled = !bloquear; }
  escalaGizmo(pos) {
    const profundidad = -pos.clone().applyMatrix4(this.camara.matrixWorldInverse).z;
    return (2 * profundidad * Math.tan(THREE.MathUtils.degToRad(this.camara.fov / 2))) / (this.cont.clientHeight || 1);
  }
  rayoGizmo({ x, y }) {
    const ndc = new THREE.Vector2(2 * x / this.cont.clientWidth - 1, 1 - 2 * y / this.cont.clientHeight);
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(ndc, this.camara);
    return raycaster.ray;
  }

  // Proyección a píxeles del contenedor (null si está detrás de la cámara)
  proyectar(p) {
    const v = new THREE.Vector3(...p).project(this.camara);
    if (Math.abs(v.z) > 1) return null;
    return { x: ((v.x + 1) / 2) * this.cont.clientWidth, y: ((1 - v.y) / 2) * this.cont.clientHeight };
  }
  fijarZmin(z) { this.planoZmin.position.z = z; }
  fijarEntorno(entorno) {
    const cajas = cajasEntorno(entorno);
    for (const { min, max, confirmado } of cajas) {
      const dimensiones = max.map((v, i) => v - min[i]);
      const g = new THREE.BoxGeometry(...dimensiones);
      const color = confirmado ? 0x236b9c : 0x8796a1;
      const caja = new THREE.Mesh(g, new THREE.MeshStandardMaterial({
        color, transparent: true, opacity: confirmado ? 0.44 : 0.18, depthWrite: false,
      }));
      caja.position.set(...max.map((v, i) => (min[i] + v) / 2));
      caja.add(new THREE.LineSegments(new THREE.EdgesGeometry(g), new THREE.LineBasicMaterial({
        color, transparent: true, opacity: confirmado ? 0.82 : 0.48,
      })));
      this.entorno.add(caja);
    }
    return cajas.length > 0;
  }
  verEntorno(visible) { this.entorno.visible = visible; }

  recibirMedido(marcos, reiniciar = false) { this.medidoVisual.recibir(marcos, reiniciar); }

  agregarTraza(p) {
    const anteriores = this.traza.puntos;
    if (!this.traza.agregar(p)) return;
    const { puntos } = this.traza;
    if (puntos === anteriores) this.rastroPos.set(p, (puntos.length - 1) * 3);
    else puntos.forEach((punto, i) => this.rastroPos.set(punto, i * 3));
    this.rastro.geometry.setDrawRange(0, puntos.length);
    this.rastro.geometry.attributes.position.needsUpdate = true;
  }

  limpiarTraza() {
    this.traza.limpiar();
    this.rastro.geometry.setDrawRange(0, 0);
  }

  verTraza(visible) { this.rastro.visible = visible; }
  verCamino(visible) { this.camino.visible = visible; }
  verPuntos(visible) { this.puntos.visible = visible; }

  fijarCamino(puntos) {
    const g = new THREE.BufferGeometry().setFromPoints((puntos || []).map((p) => new THREE.Vector3(...p)));
    this.camino.geometry.dispose();
    this.camino.geometry = g;
    this.camino.computeLineDistances();
  }

  fijarPuntos(puntos) {
    const g = new THREE.BufferGeometry().setFromPoints((puntos || []).map((p) => new THREE.Vector3(...p)));
    this.puntos.geometry.dispose();
    this.puntos.geometry = g;
  }

  captura() { return this.render.domElement.toDataURL("image/png"); }
}

// Renderizado alternativo de las mismas mallas para equipos sin WebGL.
class EscenaCanvas {
 constructor(cont){
  this.cont=cont;this.canvas=document.createElement('canvas');cont.appendChild(this.canvas);this.ctx=this.canvas.getContext('2d');
  this.worker=new Worker('escena_worker.js',{type:'module'});
  this.versionCapas = 0;
  this.worker.onmessage = ({ data }) => {
    if (data.bitmap) {
      const vista = this.vistaPendiente;
      if (vista.w === this.w && vista.h === this.h && vista.versionCapas === this.versionCapas) {
        this.ctx.clearRect(0, 0, this.w, this.h);
        this.ctx.drawImage(data.bitmap, 0, 0, this.w, this.h);
        this.vistaMostrada = vista;
      }
      data.bitmap.close();
      this.ocupado = false;
    }
    if (data.junta) window.dispatchEvent(new CustomEvent("vevas:joint", { detail: { junta: data.junta } }));
  };
  this.worker.onerror=e=>{this.ocupado=false;document.querySelector('#modelo-estado').textContent='Error de renderizado compatible';console.error(e);};
  this.theta=-1.0;this.phi=.40;this.zoom=1;this.camino=[];this.puntos=[];this.traza=new TrazaTCP();this.planoZmin={visible:true,z:.08};
  this.verCapas = { traza: true, camino: true, puntos: true };
  this.entorno = { cajas: [], visible: true };
  this.gz = new GizmoTCP(this, this.canvas);
  const robot=()=>({M:null,visible:true,marcos:false,color:CIAN,actualizar(M){this.M=M;},verMarcos(v){this.marcos=v;},fijarColor(color){this.color=color;}});
  this.medido=robot();this.fantasma=robot();this.fantasma.visible=false;this.medidoVisual=new MedidoVisual();
  Promise.all([fetch('modelo/alineacion.json').then(r=>r.json()),...['base_link',...Array.from({length:6},(_,i)=>`link_${i+1}`)].map(n=>fetch(`modelo/${n}.stl`).then(r=>r.arrayBuffer()).then(stl))]).then(([a,...g])=>{
   this.align=a.map(matriz);this.geometrias=g.map(g=>({p:g.attributes.position.array,n:g.attributes.normal.array}));this.worker.postMessage({tipo:'mallas',geometrias:this.geometrias,align:a});document.querySelector('#modelo-estado').textContent='IRB 120 · vista compatible';
  }).catch(e=>{document.querySelector('#modelo-estado').textContent='Error de mallas';console.error(e);});
  let down,last,moved;
  this.canvas.addEventListener('pointerdown',e=>{down=true;moved=false;last=[e.clientX,e.clientY];this.canvas.setPointerCapture(e.pointerId);});
  this.canvas.addEventListener('pointermove',e=>{if(!down)return;const dx=e.clientX-last[0],dy=e.clientY-last[1];if(Math.abs(dx)+Math.abs(dy)>1)moved=true;this.theta-=dx*.008;this.phi=Math.max(.04,Math.min(1.45,this.phi+dy*.008));last=[e.clientX,e.clientY];});
  this.canvas.addEventListener('pointerup',e=>{const eraOrbita=down;down=false;if(!eraOrbita||moved)return;const r=this.canvas.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top;
   this.worker.postMessage({tipo:'pick',x,y});
  });
  this.canvas.addEventListener('pointercancel',()=>{down=false;});
  this.canvas.addEventListener('wheel',e=>{e.preventDefault();if(this.gz.arr)return;this.zoom=Math.max(.6,Math.min(2.8,this.zoom*Math.exp(-e.deltaY*.001)));},{passive:false});
  new ResizeObserver(()=>{this.w=cont.clientWidth;this.h=cont.clientHeight;this.vistaMostrada=null;const d=Math.min(devicePixelRatio,2);this.canvas.width=this.w*d;this.canvas.height=this.h*d;this.canvas.style.width=this.w+'px';this.canvas.style.height=this.h+'px';this.ctx.setTransform(d,0,0,d,0,0);}).observe(cont);
  this.timer=setInterval(()=>this.dibujar(),33);
  const cuadro = () => { this.alCuadro?.(); this.gz.dibujar(); requestAnimationFrame(cuadro); };
  requestAnimationFrame(cuadro);
 }
 fijarGizmo(pos, orientacion) { this.gz.fijar(pos, orientacion); }
 fijarModoGizmo(modo) { this.gz.modo = modo; }
 gizmoCallbacks(cb) { this.gz.cb = cb; }
 gizmoPantalla() { return this.gz.pantalla(); }
 bloquearOrbita(bloquear) {
   if (!bloquear || !this.vistaMostrada) return;
   // Empieza el gesto con la misma cámara que el usuario tiene a la vista.
   this.theta = this.vistaMostrada.theta;
   this.phi = this.vistaMostrada.phi;
   this.zoom = this.vistaMostrada.zoom;
   this.versionCapas++;
 }
 escalaGizmo() { return 1 / (Math.min(this.w, this.h) * 1.1 * (this.vistaMostrada?.zoom ?? this.zoom)); }
 ejesVista() {
   const { theta, phi } = this.vistaMostrada || this;
   const ct = Math.cos(theta), st = Math.sin(theta);
   const cp = Math.cos(phi), sp = Math.sin(phi);
   return [new THREE.Vector3(-st, ct, 0), new THREE.Vector3(-sp * ct, -sp * st, cp), new THREE.Vector3(cp * ct, cp * st, sp)];
 }
 proyectar(p) {
   if (!this.vistaMostrada) return null;
   const [der, arriba] = this.ejesVista(), escala = this.escalaGizmo();
   const v = new THREE.Vector3(...p).sub(new THREE.Vector3(0.16, 0, 0.32));
   return { x: this.w * 0.5 + der.dot(v) / escala, y: this.h * 0.53 - arriba.dot(v) / escala };
 }
 rayoGizmo({ x, y }) {
   const [der, arriba, frente] = this.ejesVista(), escala = this.escalaGizmo();
   const origen = new THREE.Vector3(0.16, 0, 0.32).addScaledVector(der, (x - this.w * 0.5) * escala)
     .addScaledVector(arriba, (this.h * 0.53 - y) * escala).addScaledVector(frente, 3);
   return new THREE.Ray(origen, frente.negate());
 }
 vistaInicial(){this.theta=-1.0;this.phi=.40;this.zoom=1;}
 fijarZmin(z){this.planoZmin.z=z;}
 fijarEntorno(entorno) {
   this.entorno.cajas = cajasEntorno(entorno);
   return this.entorno.cajas.length > 0;
 }
 verEntorno(visible) { this.entorno.visible = visible; }
 recibirMedido(marcos, reiniciar = false) { this.medidoVisual.recibir(marcos, reiniciar); }
 fijarCamino(p) { this.camino = p || []; this.versionCapas++; }
 fijarPuntos(p) { this.puntos = p || []; this.versionCapas++; }
 agregarTraza(p) { this.traza.agregar(p); }
 limpiarTraza() { this.traza.limpiar(); this.versionCapas++; }
 verTraza(visible) { this.verCapas.traza = visible; this.versionCapas++; }
 verCamino(visible) { this.verCapas.camino = visible; this.versionCapas++; }
 verPuntos(visible) { this.verCapas.puntos = visible; this.versionCapas++; }
 captura(){return this.canvas.toDataURL('image/png');}
 dibujar(){
  if(!this.w||!this.h||!this.geometrias||this.ocupado)return;
  const medido = this.medidoVisual.cuadro();
  if (medido) this.medido.actualizar(medido);
  const { w, h, theta, phi, zoom, versionCapas } = this;
  const vista = { w, h, theta, phi, zoom, versionCapas };
  const firma = [w,h,theta,phi,zoom,versionCapas,this.medido.M,this.medido.visible,this.medido.marcos,
    this.fantasma.visible,this.fantasma.visible && this.fantasma.M,this.fantasma.color,this.planoZmin.visible,
    this.planoZmin.z,this.entorno.cajas,this.entorno.visible,this.traza.puntos.length];
  if (this.vistaMostrada && firma.every((v, i) => v === this.firma?.[i])) return;
  this.firma = firma;
  this.vistaPendiente = vista;
  this.ocupado = true;
  this.worker.postMessage({ tipo: "estado", ...vista,
    medido: { M: this.medido.M, visible: this.medido.visible, marcos: this.medido.marcos },
    fantasma: { M: this.fantasma.M, visible: this.fantasma.visible, marcos: false, color: this.fantasma.color },
    planoZmin: this.planoZmin, entorno: this.entorno,
    camino: this.verCapas.camino ? this.camino : [],
    puntos: this.verCapas.puntos ? this.puntos : [],
    rastro: this.verCapas.traza ? this.traza.puntos : [],
  });
 }

}
export class Escena{
 constructor(cont){const probe=document.createElement('canvas');const gl=probe.getContext('webgl2');if(gl){gl.getExtension('WEBGL_lose_context')?.loseContext();return new EscenaWebGL(cont);}return new EscenaCanvas(cont);}
}
