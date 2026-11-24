// Edición del destino TCP y consulta IK. El plan se solicita al pulsar «Calcular y verificar».
import { f, tablaRamas } from "./calculos.js";
import { matrizZYX, rotarEnBase } from "./orientacion.js";

const esc = (t) => String(t ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const CIAN = 0x198ba7, ROJO = 0xd6403a;
const INTERVALO_MS = 100;

export function crearArrastre({ est, $, aviso, calcularPlan, invalidarPlan, abrirPestana }) {
  let modo = "traslacion";
  let enVuelo = false, timer = null, ultimaConsulta = -Infinity;
  const D0 = () => est.drag;
  const ikVigente = (D) => D?.revisionIK === D?.revision ? D?.ik : null;

  const tcpActual = () => {
    const p = est.vivo?.tcp_med;
    return p ? { x: p.x_mm, y: p.y_mm, z: p.z_mm, rz: p.rz_deg, ry: p.ry_deg, rx: p.rx_deg } : null;
  };
  function destino() {
    if (est.vivo?.ejec || est.vivo?.jog || (est.gz?.abierto && est.gz.modo === "junta")) return null;
    return D0()?.obj || tcpActual();
  }
  function preparar() {
    const v = est.vivo;
    if (!v?.tcp_med || !v.marcos_med) return null;
    if (v.ejec || v.jog || ["corriendo", "esperando"].includes(v.secuencia?.estado)) {
      aviso("Esperá a que termine el movimiento para fijar otro destino.", "error");
      return null;
    }
    return est.drag ||= { obj: tcpActual(), marcos: v.marcos_med, revision: 0 };
  }

  function alIniciar() {
    const D = preparar();
    if (!D) return false;
    D.arrastrando = true;
    D.inicioGesto = D.obj;
    return true;
  }
  function alArrastrar({ pos, eje, angulo }) {
    const D = D0();
    if (!D?.inicioGesto) return;
    const inicio = D.inicioGesto;
    editar(D, pos ? { ...inicio, x: pos[0] * 1000, y: pos[1] * 1000, z: pos[2] * 1000 } : rotarEnBase(inicio, eje, angulo));
  }
  function alSoltar() {
    const D = D0();
    if (!D) return;
    D.arrastrando = false;
    D.inicioGesto = null;
    if (!D.revision) { est.drag = null; return; }
    dibujarBadge();
    bombear();
  }
  function trasladar(eje, pasoMm) {
    const D = preparar();
    if (!D || D.arrastrando) return;
    const k = ["x", "y", "z"][eje];
    editar(D, { ...D.obj, [k]: D.obj[k] + pasoMm });
  }
  function definirPose(pose) {
    const D = preparar();
    if (!D || D.arrastrando) return;
    const { x, y, z, rz, ry, rx } = pose;
    editar(D, { x, y, z, rz, ry, rx });
  }

  function editar(D, obj) {
    if (!Object.keys(obj).some((k) => Math.abs(obj[k] - D.obj[k]) > 1e-6)) return;
    if (est.gz) est.gz.acum = null;
    invalidarPlan();
    D.revision++;
    D.obj = obj;
    D.pend = true;
    D.mensaje = null;
    D.fantasmaOn = true;
    D.planId = null;
    dibujarPanel();
    dibujarBadge();
    bombear();
  }

  // Se conserva solo la pose más reciente mientras hay una consulta en vuelo.
  async function bombear() {
    const D = D0();
    if (!D?.pend || enVuelo || timer) return;
    const espera = INTERVALO_MS - (performance.now() - ultimaConsulta);
    if (espera > 0) {
      timer = setTimeout(() => { timer = null; bombear(); }, espera);
      return;
    }
    const { obj, revision } = D;
    D.pend = false;
    enVuelo = true;
    ultimaConsulta = performance.now();
    try {
      const q_ref_deg = D.ultimoOk?.ik.q_deg || est.vivo?.q_med_deg || null;
      const r = await fetch("/api/ik_vivo", {
        method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ ...obj, q_ref_deg }),
      });
      if (!r.ok) throw new Error("Consulta IK incompleta");
      const d = await r.json();
      if (!d.ok || !d.ik) throw new Error("Respuesta IK incompleta");
      if (D === D0() && D.revision === revision) {
        const ik = D.ik = d.ik;
        D.revisionIK = revision;
        D.pedido = obj;
        D.marcos = ik.marcos || D.ultimoOk?.marcos || est.vivo?.marcos_med;
        if (ik.admisible) D.ultimoOk = { obj, marcos: D.marcos, ik };
      }
    } catch {
      if (D === D0() && D.revision === revision) D.mensaje = "No se pudo consultar la cinemática inversa.";
    } finally {
      enVuelo = false;
      if (D === D0() && D.revision === revision) { dibujarPanel(); dibujarBadge(); }
      bombear();
    }
  }

  async function calcular() {
    const D = D0();
    if (!ikVigente(D)?.admisible || D.arrastrando || D.calculando || (D.planId && est.plan?.id === D.planId)) return;
    D.calculando = true;
    dibujarBadge();
    const revision = D.revision;
    const d = await calcularPlan("calcular_pose", D.obj, false, true);
    if (D !== D0()) return;
    D.calculando = false;
    if (d.ok && D.revision === revision) {
      D.planId = d.plan.id;
      D.fantasmaOn = false;
      abrirPestana("objetivo");
    }
    dibujarPanel();
    dibujarBadge();
  }
  function descartar(silencioso = false) {
    const D = D0();
    if (D?.calculando || (D?.planId && est.plan?.id === D.planId)) invalidarPlan();
    reiniciar();
    if (!silencioso) aviso("Destino descartado. El control vuelve al TCP medido.", "ok");
  }
  function reiniciar() {
    est.drag = null;
    dibujarPanel();
    dibujarBadge();
  }
  function irAlUltimoValido() {
    const D = D0();
    if (!D?.ultimoOk) return;
    invalidarPlan();
    D.revision++;
    Object.assign(D, D.ultimoOk, { pend: false, mensaje: null, revisionIK: D.revision, pedido: D.ultimoOk.obj });
    dibujarPanel();
    dibujarBadge();
  }
  function verRamas() {
    const ik = D0()?.ik; if (!ik?.ramas) return;
    const ramas = ik.ramas.map((r) => ({
      ...r, hombro: r.nombre, codo: null, muneca: null, e_p_m: null, e_R_rad: null, sigma_min: null, distancia: null,
      motivos: r.motivo ? [r.motivo] : [],
    }));
    const razon = "Vista en vivo del destino. Calculá y verificá para ver los residuos de cada rama.";
    $("#ramas-contenido").innerHTML = tablaRamas({ tipo: "pose", ik: { ramas, elegida: ik.elegida, razon } });
    abrirPestana("ramas");
  }

  function dibujarBadge() {
    const el = $("#gz-badge"), D = D0();
    if (!D?.revision) { el.className = "gz-badge oculto"; return; }
    const { obj: o } = D, ik = ikVigente(D), ok = ik?.admisible;
    const coords = `X ${f(o.x, 0)} · Y ${f(o.y, 0)} · Z ${f(o.z, 0)} mm`;
    const orientacion = `Rz ${f(o.rz, 1)}° · Ry ${f(o.ry, 1)}° · Rx ${f(o.rx, 1)}°`;
    const motivo = ik?.motivos?.[0] || D.mensaje;
    const plan = D.planId && est.plan?.id === D.planId ? est.plan : null;
    const codigo = plan ? "plan" : ok ? "ok" : motivo ? "mal" : "";
    const titulo = plan ? (plan.ejecutable ? "Plan calculado" : "Plan bloqueado")
      : D.calculando ? "Revisando trayectoria…" : ok ? "✓ Destino admisible" : motivo ? "✗ " + esc(motivo) : "Calculando IK…";
    el.className = "gz-badge " + codigo + (D.arrastrando ? " flota" : "");
    const acciones = D.arrastrando ? "" : `<div class="gb-b">${plan ? "" : `<button id="gz-b-calc" data-a="calcular" class="gb-cta" ${ok && !D.calculando ? "" : "disabled"}>Calcular y verificar</button>`}${motivo && D.ultimoOk ? '<button data-a="valido" class="btn-chico">Volver al último válido</button>' : ""}<button data-a="descartar" class="btn-chico">Descartar</button></div>`;
    el.innerHTML = `<div class="gb-t ${codigo}">${titulo}</div><div class="gb-s">${coords}<br>${orientacion}</div>${plan ? '<div class="gb-s">Mirá la revisión en «Control».</div>' : ""}${acciones}`;
  }
  $("#gz-badge").addEventListener("click", (e) => {
    const a = e.target.closest("[data-a]")?.dataset.a;
    if (a === "calcular") calcular(); else if (a === "descartar") descartar(); else if (a === "valido") irAlUltimoValido();
  });

  // ---------- panel de IK/FK en vivo (pestaña Control)
  function celda(nombre, val, extra = "") { return `<div class="ik-c"><small>${nombre}</small><b>${val}</b>${extra}</div>`; }
  function dibujarPanel() {
    const el = $("#ik-vivo"), D = D0();
    const sigU = est.cfg?.cfg?.ik?.sigma_min ?? 0.02;
    if (!D?.ik) {
      const estado = !D?.revision ? "Arrastrá una flecha, el centro o un anillo del TCP para ver las ramas de Pieper, el error de FK y la proximidad a una singularidad."
        : D.mensaje || "Calculando la cinemática inversa del destino…";
      el.className = "ik-vivo vacio-ik";
      el.innerHTML = `<span class="kicker">CINEMÁTICA INVERSA EN VIVO</span><p>${esc(estado)}</p>`;
      return;
    }
    const ik = D.ik, ok = ik.admisible, vigente = !!ikVigente(D), o = D.pedido, act = tcpActual();
    const nVal = (ik.ramas || []).filter((r) => r.valida).length;
    const rama = ik.elegida != null ? ik.ramas?.[ik.elegida] : null;
    const sg = ik.sigma_min, sing = ik.singular || {};
    const nivel = sg == null ? "ok" : sg < sigU ? "singular" : sg < 2 * sigU || sing.cerca ? "cerca" : "ok";
    const pct = sg == null ? 0 : Math.min(100, (100 * sg) / (6 * sigU));
    const dlt = (k) => (act && o ? ` <i>${o[k] - act[k] >= 0 ? "+" : ""}${f(o[k] - act[k], 0)}</i>` : "");
    const q0 = est.vivo?.q_med_deg, q1 = ik.q_deg;
    const dmax = ik.delta_max_deg;
    let h = `<div class="ik-cab"><span class="kicker">CINEMÁTICA INVERSA EN VIVO</span><span class="ik-estado ${vigente ? (ok ? "ok" : "mal") : "pendiente"}">${vigente ? (ok ? "✓ Admisible" : "✗ No admisible") : esc(D.mensaje || "Actualizando IK…")}</span></div>`;
    h += `<div class="ik-pose">${celda("X", f(o.x, 1), dlt("x"))}${celda("Y", f(o.y, 1), dlt("y"))}${celda("Z", f(o.z, 1), dlt("z"))}${celda("Rz", f(o.rz, 1) + "°")}${celda("Ry", f(o.ry, 1) + "°")}${celda("Rx", f(o.rx, 1) + "°")}</div>`;
    h += `<div class="ik-fila"><div class="ik-rama"><small>Rama de Pieper</small><b>${rama ? `Rama ${rama.indice} · ${esc(rama.nombre)}` : "ninguna sirve"}</b><span>${nVal} de 8 válidas <button class="btn-chico" data-a="ramas">Ver las 8 ramas</button></span></div>`;
    h += `<div class="ik-fk"><small>Verificación FK (backend)</small><b>e<sub>p</sub> ${ik.verificacion ? ik.verificacion.e_p_mm.toExponential(1) : "—"} mm</b><b>e<sub>R</sub> ${ik.verificacion ? ik.verificacion.e_R_deg.toExponential(1) : "—"}°</b></div></div>`;
    h += `<div class="ik-sig ${nivel}"><small>σ<sub>min</sub> ${sg == null ? "—" : f(sg, 3)}</small><div class="ik-barra"><i style="width:${pct}%"></i><u style="left:${100 / 6}%" title="umbral de singularidad"></u><u style="left:${200 / 6}%" title="cerca de singularidad"></u></div><span>${sg == null ? "Sin postura para evaluar" : nivel === "ok" ? "lejos de singularidades" : esc(sing.tipo ? `${nivel === "singular" ? "singular" : "cerca"}: ${sing.tipo}` : nivel)}</span></div>`;
    if (q1) {
      h += `<table class="ik-q"><tr><th></th>${[1, 2, 3, 4, 5, 6].map((i) => `<th>q${i}</th>`).join("")}</tr>`;
      if (q0) h += `<tr><td>ahora</td>${q0.map((x) => `<td>${f(x, 1)}</td>`).join("")}</tr>`;
      h += `<tr><td>IK</td>${q1.map((x) => `<td><b>${f(x, 1)}</b></td>`).join("")}</tr>`;
      if (q0) h += `<tr><td>Δ</td>${q1.map((x, i) => { const d = x - q0[i]; return `<td class="${dmax != null && Math.abs(Math.abs(d) - dmax) < 0.05 ? "dmax" : ""}">${d >= 0 ? "+" : ""}${f(d, 1)}</td>`; }).join("")}</tr>`;
      h += `</table>`;
    }
    if (dmax != null) h += `<div class="ik-nota">Mayor giro de una junta: <b>${f(dmax, 1)}°</b></div>`;
    if (!ok && ik.motivos?.length) h += `<ul class="ik-motivos">${ik.motivos.map((m) => `<li>${esc(m)}</li>`).join("")}</ul>`;
    el.className = "ik-vivo " + (vigente ? (ok ? "ok" : "mal") : "");
    el.innerHTML = h;
  }
  $("#ik-vivo").addEventListener("click", (e) => { if (e.target.closest("[data-a=ramas]")) verRamas(); });

  function cuadro() {
    const esc3 = est.escena, o = destino();
    const a = o && [o.x / 1000, o.y / 1000, o.z / 1000];
    esc3.fijarGizmo(a, o && modo === "rotacion" ? matrizZYX(o) : null);
    if (o && est.gz?.abierto && est.gz.modo === "tcp") {
      const lectura = $("#gz-tcp-valor");
      const valores = `${f(o.x, 0)} · ${f(o.y, 0)} · ${f(o.z, 0)} mm`;
      if (lectura.textContent !== valores) lectura.textContent = valores;
    }
    const el = $("#gz-badge");
    if (el.classList.contains("oculto")) return;
    const p = (a && esc3.proyectar(a)) || { x: 0, y: 0 };
    const W = esc3.cont.clientWidth, H = esc3.cont.clientHeight, w = el.offsetWidth, h = el.offsetHeight;
    let x = p.x + 65;
    if (x + w > W - 6) x = p.x - w - 65;
    x = Math.max(6, x);
    const arriba = $("#gz-edicion").offsetTop + $("#gz-edicion").offsetHeight + 8 - esc3.cont.offsetTop;
    const y = Math.max(arriba, Math.min(H - h - 10, p.y - h / 2));
    el.style.transform = `translate(${x}px,${y + esc3.cont.offsetTop}px)`;
  }

  function fantasma() {
    const D = D0();
    if (!D?.fantasmaOn || !D.marcos) return null;
    return { marcos: D.marcos, color: ikVigente(D)?.admisible === false ? ROJO : CIAN };
  }

  function iniciar() {
    const e = est.escena;
    e.gizmoCallbacks({ alIniciar, alArrastrar, alSoltar });
    e.alCuadro = cuadro;
    const selector = $("#gz-edicion");
    const botonesModo = Array.from(selector.querySelectorAll("[data-modo]"));
    botonesModo.forEach((b) => b.addEventListener("click", () => {
      modo = b.dataset.modo;
      e.fijarModoGizmo(modo);
      botonesModo.forEach((c) => c.setAttribute("aria-pressed", String(c === b)));
      $("#gz-giro-pasos").classList.toggle("oculto", modo !== "rotacion");
    }));
    selector.querySelectorAll("[data-giro]").forEach((b) => b.addEventListener("click", () => {
      const D = preparar();
      if (!D || D.arrastrando) return;
      editar(D, rotarEnBase(D.obj, +b.dataset.giro, +b.dataset.signo * Math.PI / 36));
    }));
    dibujarPanel();
    window.__vevas = {
      gizmoPantalla: () => e.gizmoPantalla(),
      drag: () => est.drag,
      calcular, descartar, reiniciar,
    };
  }

  return { iniciar, fantasma, reiniciar, descartar, calcular, trasladar, definirPose };
}
