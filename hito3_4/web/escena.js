// Escena 3D del IRB 120 dibujada a partir de los marcos D-H que manda el servidor.
// El servidor hace TODA la cinemática (Python); aquí solo se dibujan matrices 4x4.
// Cada robot = 7 marcos D-H (T_0^0 ... T_0^6) + el marco del TCP.

import * as THREE from "three";
import { OrbitControls } from "./vendor/OrbitControls.js";

const NARANJA = 0xff9b3d;
const GRIS = 0x8a96a3;
const CIAN = 0x4cc2ff;

function matriz(arr) {
  const m = new THREE.Matrix4();
  m.set(...arr);
  return m;
}
function pos(arr) { return new THREE.Vector3(arr[3], arr[7], arr[11]); }
function ejeZ(arr) { return new THREE.Vector3(arr[2], arr[6], arr[10]).normalize(); }

class RobotVisual {
  constructor(escena, { color, opacidad = 1, conMarcos = false }) {
    this.grupo = new THREE.Group();
    escena.add(this.grupo);
    const transp = opacidad < 1;
    this.matCuerpo = new THREE.MeshStandardMaterial({ color, metalness: 0.25, roughness: 0.55, transparent: transp, opacity: opacidad, depthWrite: !transp });
    this.matJunta = new THREE.MeshStandardMaterial({ color: transp ? color : GRIS, metalness: 0.4, roughness: 0.4, transparent: transp, opacity: opacidad, depthWrite: !transp });
    const cil = new THREE.CylinderGeometry(1, 1, 1, 28);
    // Pedestal (base fija)
    this.base = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.09, 0.17, 32), this.matJunta);
    this.base.rotation.x = Math.PI / 2;
    this.base.position.set(0, 0, 0.085);
    this.grupo.add(this.base);
    // Eslabones (segmentos entre puntos)
    this.segmentos = [];
    const radios = [0.062, 0.05, 0.042, 0.045, 0.034, 0.03];
    for (const r of radios) {
      const m = new THREE.Mesh(cil, this.matCuerpo);
      m.userData.r = r;
      this.grupo.add(m);
      this.segmentos.push(m);
    }
    // Juntas (cilindros a lo largo de z_{i-1})
    this.juntas = [];
    const dims = [[0.07, 0.10], [0.065, 0.13], [0.055, 0.11], [0.045, 0.05], [0.04, 0.09], [0.035, 0.03]];
    for (const [r, l] of dims) {
      const m = new THREE.Mesh(cil, this.matJunta);
      m.scale.set(r, l, r);
      this.grupo.add(m);
      this.juntas.push(m);
    }
    // Brida y TCP
    this.brida = new THREE.Mesh(cil, this.matJunta);
    this.brida.scale.set(0.032, 0.012, 0.032);
    this.grupo.add(this.brida);
    this.ejesTCP = new THREE.AxesHelper(0.08);
    this.ejesTCP.material.depthTest = false;
    this.ejesTCP.renderOrder = 5;
    this.grupo.add(this.ejesTCP);
    // Marcos D-H (opcional)
    this.marcos = [];
    if (conMarcos) {
      for (let i = 0; i <= 6; i++) {
        const g = new THREE.Group();
        const ax = new THREE.AxesHelper(0.07);
        ax.material.depthTest = false;
        ax.renderOrder = 6;
        g.add(ax);
        g.add(etiqueta(String(i)));
        g.visible = false;
        this.grupo.add(g);
        this.marcos.push(g);
      }
    }
  }

  _segmento(m, a, b) {
    const d = new THREE.Vector3().subVectors(b, a);
    const L = d.length();
    if (L < 1e-6) { m.visible = false; return; }
    m.visible = true;
    m.position.copy(a).addScaledVector(d, 0.5);
    m.scale.set(m.userData.r, L, m.userData.r);
    m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
  }

  _junta(m, arr, desplazar = 0) {
    const p = pos(arr);
    const z = ejeZ(arr);
    m.position.copy(p).addScaledVector(z, desplazar);
    m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), z);
  }

  actualizar(M) {
    // M: 7 marcos D-H + TCP, cada uno como 16 números (fila por fila)
    if (!M || M.length < 8) return;
    const o = M.map(pos);
    // o0 base, o1 hombro (eje 2), o2 codo (eje 3), o3 inicio antebrazo, o4 = o5 centro de muñeca, o6 brida
    this._segmento(this.segmentos[0], new THREE.Vector3(0, 0, 0.17), o[1]);
    this._segmento(this.segmentos[1], o[1], o[2]);
    this._segmento(this.segmentos[2], o[2], o[3]);
    this._segmento(this.segmentos[3], o[3], o[4]);
    this._segmento(this.segmentos[4], o[4], o[6]);
    this._segmento(this.segmentos[5], o[6], o[7]);
    this._junta(this.juntas[0], M[0], 0.24);
    for (let i = 1; i < 6; i++) this._junta(this.juntas[i], M[i]);
    this._junta(this.brida, M[6]);
    const T = matriz(M[7]);
    this.ejesTCP.position.setFromMatrixPosition(T);
    this.ejesTCP.quaternion.setFromRotationMatrix(T);
    this.marcos.forEach((g, i) => {
      const Ti = matriz(M[i]);
      g.position.setFromMatrixPosition(Ti);
      g.quaternion.setFromRotationMatrix(Ti);
    });
  }

  verMarcos(v) { this.marcos.forEach((g) => (g.visible = v)); }
  set visible(v) { this.grupo.visible = v; }
  get visible() { return this.grupo.visible; }
}

function etiqueta(texto) {
  const c = document.createElement("canvas");
  c.width = 64; c.height = 64;
  const g = c.getContext("2d");
  g.fillStyle = "#0d1117cc";
  g.beginPath(); g.arc(32, 32, 22, 0, Math.PI * 2); g.fill();
  g.fillStyle = "#ffffff"; g.font = "bold 30px sans-serif"; g.textAlign = "center"; g.textBaseline = "middle";
  g.fillText(texto, 32, 34);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false }));
  s.scale.set(0.035, 0.035, 1);
  s.position.set(0.02, 0.02, 0.03);
  s.renderOrder = 7;
  return s;
}

export class Escena {
  constructor(contenedor) {
    this.cont = contenedor;
    this.render = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.render.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.render.setClearColor(0x0d1117);
    contenedor.appendChild(this.render.domElement);
    this.escena = new THREE.Scene();
    this.camara = new THREE.PerspectiveCamera(42, 1, 0.01, 20);
    this.camara.up.set(0, 0, 1);
    this.controles = new OrbitControls(this.camara, this.render.domElement);
    this.controles.enableDamping = true;
    this.vistaInicial();

    this.escena.add(new THREE.HemisphereLight(0xdfe9ff, 0x1a1f26, 1.1));
    const luz = new THREE.DirectionalLight(0xffffff, 1.6);
    luz.position.set(1.5, -1.2, 2.2);
    this.escena.add(luz);

    const grilla = new THREE.GridHelper(1.6, 16, 0x3a4552, 0x222b36);
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

    // Camino planificado y rastro medido del TCP
    this.camino = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: CIAN, dashSize: 0.012, gapSize: 0.008 }));
    this.escena.add(this.camino);
    this.rastroMax = 600;
    this.rastroPos = new Float32Array(this.rastroMax * 3);
    this.rastroN = 0;
    const gr = new THREE.BufferGeometry();
    gr.setAttribute("position", new THREE.BufferAttribute(this.rastroPos, 3));
    gr.setDrawRange(0, 0);
    this.rastro = new THREE.Line(gr, new THREE.LineBasicMaterial({ color: 0xffe066 }));
    this.escena.add(this.rastro);
    this.objetivo = new THREE.Mesh(new THREE.SphereGeometry(0.012, 20, 14), new THREE.MeshBasicMaterial({ color: CIAN }));
    this.objetivo.visible = false;
    this.escena.add(this.objetivo);

    this._redimensionar();
    new ResizeObserver(() => this._redimensionar()).observe(contenedor);
    const bucle = () => {
      this.controles.update();
      this.render.render(this.escena, this.camara);
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
