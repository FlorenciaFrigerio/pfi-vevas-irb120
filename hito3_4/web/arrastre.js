// Arrastre de la punta (TCP) con el mouse sobre el modelo 3D + panel de cinemática inversa en vivo.
// Las flechas viven en escena.js (WebGL). Acá se decide qué se calcula: POST /api/ik_vivo mientras se arrastra
// (una sola consulta en vuelo, ~12 por segundo como máximo) y, al soltar, el botón «Calcular y verificar»
// pide un plan normal (calcular_pose). Nunca se envía nada al robot desde acá.
import { f, tablaRamas } from "./calculos.js";

const esc = (t) => String(t ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const CIAN = 0x198ba7, ROJO = 0xd6403a;
const INTERVALO_MS = 80;

export function crearArrastre({ est, $, orden, aviso, recibirPlan, invalidarPlan, abrirPestana }) {
  let intentoNota = false;
  const D0 = () => est.drag;

  // ---------- utilidades
  const tcpActual = () => {
    const p = est.vivo?.tcp_med;
    return p ? { x: p.x_mm, y: p.y_mm, z: p.z_mm, rz: p.rz_deg, ry: p.ry_deg, rx: p.rx_deg } : null;
  };
  const anclaje = () => {
    const D = D0();
    if (D?.obj) return [D.obj.x / 1000, D.obj.y / 1000, D.obj.z / 1000];
    const v = est.vivo;
    if (!v?.marcos_med || v.ejec) return null;
    const m = v.marcos_med[7];
    return [m[3], m[7], m[11]];
  };
  const nuevo = () => ({ obj: null, inicio: null, ik: null, marcos: null, admisible: null, ultimoOk: null, pend: null, enVuelo: false, arrastrando: false, soltado: false, fantasmaOn: false, calculado: false, planId: null, mensaje: null, ult: 0 });

  // ---------- callbacks de la escena
  function alIniciar() {
    const v = est.vivo;
    if (!v?.tcp_med || !v.marcos_med) return false;
    if (v.ejec || v.jog) { aviso("Esperá a que termine el movimiento para fijar otro destino.", "error"); return false; }
    const D = (est.drag = est.drag || nuevo());
    if (!D.obj) { D.obj = tcpActual(); D.inicio = { ...D.obj }; D.ultimoOk = null; D.ik = null; }
    D.arrastrando = true; D.soltado = false; D.mensaje = null; D.sinBackend = false;
    return true;
  }
  function alArrastrar({ pos }) {
    const D = D0(); if (!D) return;
    if (D.calculado) { D.calculado = false; if (est.plan && est.plan.id === D.planId) invalidarPlan(); }
    D.obj = { ...D.obj, x: pos[0] * 1000, y: pos[1] * 1000, z: pos[2] * 1000 };
    D.pend = { ...D.obj };
    D.fantasmaOn = true;
    bombear();
  }
  function alSoltar() {
    const D = D0(); if (!D) return;
    D.arrastrando = false; D.soltado = true;
    if (D.enVuelo || D.pend) { if (!D.enVuelo) bombear(true); } else terminar();
  }

  // ---------- consulta IK (una en vuelo, con ritmo)
  async function pedir(obj, qref) {
    try {
      const r = await fetch("/api/ik_vivo", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ ...obj, q_ref_deg: qref }) });
      if (r.status === 404) return { ok: false, falta: true };
      return await r.json();
    } catch (e) { return { ok: false }; }
  }
  async function bombear(inmediato = false) {
    const D = D0();
    if (!D || D.enVuelo || !D.pend) return;
    const espera = inmediato ? 0 : INTERVALO_MS - (performance.now() - D.ult);
    if (espera > 4) { if (!D.timer) D.timer = setTimeout(() => { D.timer = null; bombear(); }, espera); return; }
    const obj = D.pend; D.pend = null; D.enVuelo = true; D.ult = performance.now();
    const qref = D.ik?.q_deg || est.vivo?.q_med_deg || null;
    const d = await pedir(obj, qref);
    D.enVuelo = false;
    if (est.drag !== D) return;
    if (d.ok && d.ik) aplicar(obj, d.ik);
    else if (d.falta && !D.sinBackend) { D.sinBackend = true; aviso("El servidor todavía no tiene /api/ik_vivo.", "error"); }
    if (D.pend) bombear(D.soltado);
    else if (D.soltado) terminar();
  }
  function aplicar(obj, ik) {
    const D = D0();
    D.ik = ik; D.pedido = obj; D.admisible = !!ik.admisible;
    if (ik.marcos) D.marcos = ik.marcos;
    if (ik.admisible) { D.ultimoOk = { obj: { ...obj }, marcos: ik.marcos, ik }; if (ik.marcos) D.marcosOk = ik.marcos; }
    else if (!ik.marcos && D.marcosOk) D.marcos = D.marcosOk; // sin IK: el fantasma queda en el último punto válido
    dibujarPanel(); dibujarBadge();
  }
  function terminar() {
    const D = D0(); if (!D) return;
    D.soltado = false;
    if (!D.ik) { // no hubo respuesta (sin backend): se vuelve al TCP medido
      est.drag = null; dibujarPanel(); dibujarBadge(); return;
    }
    dibujarPanel(); dibujarBadge();
  }

  // ---------- acciones del badge
  async function calcular() {
    const D = D0(); if (!D || !D.admisible || !D.ultimoOk) return;
    const o = D.ultimoOk.obj;
    const btn = $("#gz-b-calc"); if (btn) btn.disabled = true;
    const d = await orden("calcular_pose", { x: o.x, y: o.y, z: o.z, rz: o.rz, ry: o.ry, rx: o.rx });
    if (!d.ok) { dibujarBadge(); return; }
    est.drag = D;
    recibirPlan(d.plan, false, true);
    D.calculado = true; D.planId = d.plan.id; D.fantasmaOn = false;
    if (["datos", "pasos", "fk", "jac", "sec", "bit"].includes(est.pestana)) abrirPestana("objetivo");
    dibujarPanel(); dibujarBadge();
  }
  function descartar(silencioso = false) {
    const D = D0();
    if (D?.calculado && est.plan && est.plan.id === D.planId) invalidarPlan();
    est.drag = null;
    dibujarPanel(); dibujarBadge();
    if (!silencioso) aviso("Destino descartado: la flecha vuelve a la punta medida.", "ok");
  }
  function irAlUltimoValido() {
    const D = D0(); if (!D?.ultimoOk) return;
    D.obj = { ...D.ultimoOk.obj }; D.ik = D.ultimoOk.ik; D.admisible = true; D.pedido = D.obj;
    D.marcos = D.ultimoOk.marcos || D.marcos; D.pend = null;
    dibujarPanel(); dibujarBadge();
  }
  function verRamas() {
    const ik = D0()?.ik; if (!ik?.ramas) return;
    const ramas = ik.ramas.map((r) => ({
      ...r, hombro: r.nombre, codo: null, muneca: null, e_p_m: null, e_R_rad: null, sigma_min: null, distancia: null,
      motivos: r.motivo ? [r.motivo] : [],
    }));
    const razon = "Vista en vivo del arrastre (todavía sin plan). Calculá y verificá para ver los residuos de cada rama.";
    $("#ramas-contenido").innerHTML = tablaRamas({ tipo: "pose", ik: { ramas, elegida: ik.elegida, razon } });
    abrirPestana("ramas");
  }

  // ---------- badge flotante junto a las flechas
  function dibujarBadge() {
    const el = $("#gz-badge"), D = D0();
    if (!D || (!D.ik && !D.mensaje && !D.arrastrando)) { el.classList.add("oculto"); el.dataset.k = ""; return; }
    const ik = D.ik, ok = D.admisible;
    let h = "", k;
    const o = D.obj;
    const coords = o ? `X ${f(o.x, 0)} · Y ${f(o.y, 0)} · Z ${f(o.z, 0)} mm` : "";
    if (D.calculado) {
      k = "plan";
      h = `<div class="gb-t ok">✓ Plan calculado</div><div class="gb-s">Revisalo en «Control» y enviá con el botón de abajo.</div><div class="gb-b"><button data-a="descartar" class="btn-chico">Descartar</button></div>`;
    } else if (D.arrastrando) {
      k = "arr:" + (ok == null ? "" : ok ? 1 : 0) + (ik?.motivos?.[0] || "");
      h = `<div class="gb-t ${ok ? "ok" : "mal"}">${ok == null ? "Calculando…" : ok ? "✓ Admisible" : "✗ " + esc(ik?.motivos?.[0] || "No admisible")}</div><div class="gb-s">${coords}</div>`;
    } else if (ok) {
      k = "listo";
      h = `<div class="gb-t ok">✓ Destino admisible</div><div class="gb-s">${coords}</div><div class="gb-b"><button id="gz-b-calc" data-a="calcular" class="gb-cta">Calcular y verificar</button><button data-a="descartar" class="btn-chico">Descartar</button></div>`;
    } else {
      k = "mal";
      h = `<div class="gb-t mal">✗ ${esc(ik?.motivos?.[0] || D.mensaje || "No admisible")}</div>${ik?.motivos?.length > 1 ? `<div class="gb-s">${ik.motivos.slice(1, 3).map(esc).join(" · ")}</div>` : ""}<div class="gb-b"><button id="gz-b-calc" data-a="calcular" class="gb-cta" disabled>Calcular y verificar</button>${D.ultimoOk ? '<button data-a="valido" class="btn-chico">Volver al último válido</button>' : ""}<button data-a="descartar" class="btn-chico">Descartar</button></div>`;
    }
    el.dataset.k = k;
    el.className = "gz-badge " + (D.calculado ? "plan" : ok == null ? "" : ok ? "ok" : "mal") + (D.arrastrando ? " flota" : "");
    el.innerHTML = h;
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
    if (!D || !D.ik) {
      el.className = "ik-vivo vacio-ik";
      el.innerHTML = est.escena?.tieneArrastre === false
        ? `<span class="kicker">CINEMÁTICA INVERSA EN VIVO</span><p>El arrastre con el mouse necesita WebGL. Podés fijar el destino en «Pose del TCP» o con «Mover con botones».</p>`
        : `<span class="kicker">CINEMÁTICA INVERSA EN VIVO</span><p><b>Arrastrá una flecha de color</b> (X rojo · Y verde · Z azul) en la punta del robot 3D. Acá vas a ver la solución de Pieper, el error de FK y la distancia a una singularidad mientras la movés.</p>`;
      return;
    }
    const ik = D.ik, ok = !!ik.admisible, o = D.pedido || D.obj, act = tcpActual();
    const nVal = (ik.ramas || []).filter((r) => r.valida).length;
    const rama = ik.elegida != null ? ik.ramas?.[ik.elegida] : null;
    const sg = ik.sigma_min, sing = ik.singular || {};
    const nivel = sg == null ? "ok" : sg < sigU ? "singular" : sg < 2 * sigU || sing.cerca ? "cerca" : "ok";
    const pct = sg == null ? 0 : Math.min(100, (100 * sg) / (6 * sigU));
    const dlt = (k) => (act && o ? ` <i>${o[k] - act[k] >= 0 ? "+" : ""}${f(o[k] - act[k], 0)}</i>` : "");
    const q0 = est.vivo?.q_med_deg, q1 = ik.q_deg;
    const dmax = ik.delta_max_deg;
    let h = `<div class="ik-cab"><span class="kicker">CINEMÁTICA INVERSA EN VIVO</span><span class="ik-estado ${ok ? "ok" : "mal"}">${ok ? "✓ Admisible" : "✗ No admisible"}</span></div>`;
    h += `<div class="ik-pose">${celda("X", f(o.x, 1), dlt("x"))}${celda("Y", f(o.y, 1), dlt("y"))}${celda("Z", f(o.z, 1), dlt("z"))}${celda("Rz", f(o.rz, 1) + "°")}${celda("Ry", f(o.ry, 1) + "°")}${celda("Rx", f(o.rx, 1) + "°")}</div>`;
    h += `<div class="ik-fila"><div class="ik-rama"><small>Rama de Pieper</small><b>${rama ? `Rama ${rama.indice} · ${esc(rama.nombre)}` : "ninguna sirve"}</b><span>${nVal} de 8 válidas <button class="btn-chico" data-a="ramas">Ver las 8 ramas</button></span></div>`;
    h += `<div class="ik-fk"><small>Verificación FK (backend)</small><b>e<sub>p</sub> ${ik.verificacion ? ik.verificacion.e_p_mm.toExponential(1) : "—"} mm</b><b>e<sub>R</sub> ${ik.verificacion ? ik.verificacion.e_R_deg.toExponential(1) : "—"}°</b></div></div>`;
    h += `<div class="ik-sig ${nivel}"><small>σ<sub>min</sub> ${sg == null ? "—" : f(sg, 3)}</small><div class="ik-barra"><i style="width:${pct}%"></i><u style="left:${100 / 6}%" title="umbral de singularidad"></u><u style="left:${200 / 6}%" title="cerca de singularidad"></u></div><span>${nivel === "ok" ? "lejos de singularidades" : esc(sing.tipo ? `${nivel === "singular" ? "singular" : "cerca"}: ${sing.tipo}` : nivel)}</span></div>`;
    if (q1) {
      h += `<table class="ik-q"><tr><th></th>${[1, 2, 3, 4, 5, 6].map((i) => `<th>q${i}</th>`).join("")}</tr>`;
      if (q0) h += `<tr><td>ahora</td>${q0.map((x) => `<td>${f(x, 1)}</td>`).join("")}</tr>`;
      h += `<tr><td>IK</td>${q1.map((x) => `<td><b>${f(x, 1)}</b></td>`).join("")}</tr>`;
      if (q0) h += `<tr><td>Δ</td>${q1.map((x, i) => { const d = x - q0[i]; return `<td class="${dmax != null && Math.abs(Math.abs(d) - dmax) < 0.05 ? "dmax" : ""}">${d >= 0 ? "+" : ""}${f(d, 1)}</td>`; }).join("")}</tr>`;
      h += `</table>`;
    }
    if (dmax != null) h += `<div class="ik-nota">Mayor giro de una junta: <b>${f(dmax, 1)}°</b>${!ok && ik.motivos?.length ? "" : ""}</div>`;
    if (!ok && ik.motivos?.length) h += `<ul class="ik-motivos">${ik.motivos.map((m) => `<li>${esc(m)}</li>`).join("")}</ul>`;
    el.className = "ik-vivo " + (ok ? "ok" : "mal");
    el.innerHTML = h;
  }
  $("#ik-vivo").addEventListener("click", (e) => { if (e.target.closest("[data-a=ramas]")) verRamas(); });

  // ---------- por cuadro de render (escena WebGL)
  function cuadro() {
    const esc3 = est.escena, D = D0();
    if (!esc3?.tieneArrastre) return;
    const a = anclaje();
    esc3.fijarGizmo(a);
    const el = $("#gz-badge");
    if (el.classList.contains("oculto")) return;
    const p = a && esc3.proyectar(a);
    if (!p) return;
    const W = esc3.cont.clientWidth, H = esc3.cont.clientHeight, w = el.offsetWidth, h = el.offsetHeight;
    let x = p.x + 46; if (x + w > W - 6) x = p.x - w - 46;
    x = Math.max(6, x);
    const y = Math.max(112, Math.min(H - h - 44, p.y - h / 2));
    el.style.transform = `translate(${x}px,${y}px)`;
  }

  // fantasma que se ve mientras hay un destino sin plan
  function fantasma() {
    const D = D0();
    if (!D || !D.fantasmaOn || !D.marcos) return null;
    return { marcos: D.marcos, color: D.admisible === false ? ROJO : CIAN };
  }

  function iniciar() {
    const e = est.escena;
    if (e.tieneArrastre) {
      e.gizmoCallbacks({ alIniciar, alArrastrar, alSoltar });
      e.alCuadro = cuadro;
    } else if (!intentoNota) {
      intentoNota = true;
      $("#gz-nota-webgl").classList.remove("oculto");
    }
    dibujarPanel();
    window.__vevas = {
      gizmoPantalla: () => e.gizmoPantalla?.(),
      drag: () => est.drag,
      calcular, descartar, reiniciar: () => descartar(true),
    };
  }

  return { iniciar, fantasma, reiniciar: () => { est.drag = null; dibujarPanel(); dibujarBadge(); }, descartar, dibujarPanel, dibujarBadge };
}
