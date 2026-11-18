// Consola VEVAS: interfaz. Recibe el estado en vivo por SSE y manda órdenes por POST.
import { Escena } from "./escena.js";
import { GraficaDeslizante, GraficaTrayectoria, COLORES_JUNTAS } from "./graficas.js";
import { f, pasosPlan, tablaRamas, panelLSPB, panelFK, panelJacobiano, panelSecuencia, panelBitacora } from "./calculos.js";

const $ = (s) => document.querySelector(s);
const $$ = (s) => Array.from(document.querySelectorAll(s));

const est = {
  cfg: null,
  vivo: null,
  plan: null,
  planEjecutado: null,
  jogModo: "junta",
  jogJunta: 1,
  pestana: "objetivo",
  ramaVista: null,
  preview: { activo: false, t0: 0 },
  ultimoCaja: "",
  ultimoSec: "",
  nVivo: 0,
};

// Comunicación
async function orden(nombre, cuerpo = {}) {
  try {
    const r = await fetch("/api/" + nombre, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(cuerpo) });
    const d = await r.json();
    if (!d.ok && nombre !== "jog") aviso(d.mensaje || "Error", "error");
    return d;
  } catch (e) {
    aviso("Sin conexión con la consola: " + e, "error");
    return { ok: false };
  }
}

let tToast = null;
function aviso(texto, tipo = "") {
  const t = $("#toast");
  t.textContent = texto;
  t.className = "toast " + tipo;
  clearTimeout(tToast);
  tToast = setTimeout(() => t.classList.add("oculto"), tipo === "error" ? 6000 : 3000);
}

// Construcción inicial
async function iniciar() {
  est.cfg = await (await fetch("/api/config")).json();
  const cfg = est.cfg;

  const sel = $("#presets");
  sel.innerHTML = '<option value="">Poses de ejemplo…</option>' + cfg.presets.map((p, i) => `<option value="${i}">${p.nombre}</option>`).join("");
  sel.onchange = () => {
    const p = cfg.presets[sel.value];
    if (!p) return;
    $("#px").value = p.x; $("#py").value = p.y; $("#pz").value = p.z;
    $("#prz").value = p.rz; $("#pry").value = p.ry; $("#prx").value = p.rx;
  };
  $("#seis-juntas").innerHTML = [1, 2, 3, 4, 5, 6].map((i) => `<label>q${i} <input id="q${i}" type="number" step="1" value="${cfg.postura_inicial_deg[i - 1]}"><em>°</em></label>`).join("");
  $("#jog-juntas").innerHTML = [1, 2, 3, 4, 5, 6].map((i) => `<button data-j="${i}" class="${i === 1 ? "activo" : ""}">J${i}</button>`).join("");
  $$("#jog-juntas button").forEach((b) => (b.onclick = () => elegirJunta(+b.dataset.j)));
  $("#cuerpo-juntas").innerHTML = [1, 2, 3, 4, 5, 6].map((i) => `<tr id="fila-j${i}"><td>J${i}</td><td class="med">—</td><td class="cmd">—</td><td class="err">—</td><td><div class="rango"><div class="margen izq"></div><div class="margen der"></div><div class="cmd"></div><div class="pos"></div></div></td></tr>`).join("");
  const margen = cfg.cfg.ik.margen_limite_deg;
  cfg.limites_deg.forEach(([lo, hi], i) => {
    const w = (100 * margen) / (hi - lo);
    const fila = $(`#fila-j${i + 1}`);
    fila.querySelector(".margen.izq").style.cssText = `left:0;width:${w}%`;
    fila.querySelector(".margen.der").style.cssText = `right:0;width:${w}%`;
  });

  // 3D y gráficas
  est.escena = new Escena($("#lienzo3d"));
  est.escena.fijarZmin(cfg.cfg.seguridad.z_min_m);
  const lim = cfg.cfg.seguridad.error_seguimiento_max_deg;
  est.gJuntas = new GraficaDeslizante($("#g-err-juntas"), {
    series: COLORES_JUNTAS.map((c, i) => ({ nombre: `e${i + 1}`, color: c })),
    unidad: "error junta [°]",
    lineas: [{ valor: lim, color: "#ff5a5f" }, { valor: -lim, color: "#ff5a5f" }],
  });
  est.gTCP = new GraficaDeslizante($("#g-err-tcp"), {
    series: [{ nombre: "e_p [mm]", color: "#ffe066" }, { nombre: "e_R [°]", color: "#c792ea" }],
    unidad: "error TCP",
  });
  est.gQ = new GraficaTrayectoria($("#lspb-graficas"), "Posición q(t) [°]", "°");
  const cont2 = document.createElement("div"); $("#lspb-graficas").appendChild(cont2);
  const cont3 = document.createElement("div"); $("#lspb-graficas").appendChild(cont3);
  est.gQD = new GraficaTrayectoria(cont2, "Velocidad q̇(t) [°/s]", "°/s");
  est.gQDD = new GraficaTrayectoria(cont3, "Aceleración q̈(t) [°/s²]", "°/s²");

  conectarControles();
  conectarTeclado();
  ayudaJog();
  setInterval(() => orden("pulso"), 250);
  setInterval(redibujarLento, 400);
  setInterval(() => { est.gJuntas.dibujar(); est.gTCP.dibujar(); }, 100);
  animarVistaPrevia();
  abrirStream();
}

function abrirStream() {
  const es = new EventSource("/api/stream");
  es.onmessage = (e) => actualizar(JSON.parse(e.data));
  es.onerror = () => {
    $("#banda-modo").textContent = "SIN CONEXIÓN CON LA CONSOLA (¿se cerró el programa?)";
    $("#banda-modo").className = "banda real";
  };
}

// Controles
function conectarControles() {
  $$(".pestanas button").forEach((b) => (b.onclick = () => abrirPestana(b.dataset.pest)));
  $("#modo-expo").onchange = (e) => document.body.classList.toggle("sin-expl", !e.target.checked);
  $("#btn-parar").onclick = () => parar();
  $("#btn-vista").onclick = () => est.escena.vistaInicial();
  $("#ver-marcos").onchange = (e) => est.escena.medido.verMarcos(e.target.checked);
  $("#ver-zmin").onchange = (e) => (est.escena.planoZmin.visible = e.target.checked);
  $("#vel").oninput = (e) => ($("#vel-txt").textContent = e.target.value + " %");
  $("#vel").onchange = async (e) => {
    const d = await orden("velocidad", { pct: +e.target.value });
    if (d.ok) { $("#vel").value = d.vel_pct; $("#vel-txt").textContent = Math.round(d.vel_pct) + " %"; }
  };
  $("#btn-pose-actual").onclick = () => {
    const p = est.vivo && est.vivo.tcp_med;
    if (!p) return;
    $("#px").value = p.x_mm.toFixed(1); $("#py").value = p.y_mm.toFixed(1); $("#pz").value = p.z_mm.toFixed(1);
    $("#prz").value = p.rz_deg.toFixed(2); $("#pry").value = p.ry_deg.toFixed(2); $("#prx").value = p.rx_deg.toFixed(2);
  };
  $("#btn-q-actual").onclick = () => est.vivo && est.vivo.q_med_deg && est.vivo.q_med_deg.forEach((v, i) => ($(`#q${i + 1}`).value = v.toFixed(2)));
  $("#btn-q-inicial").onclick = () => est.cfg.postura_inicial_deg.forEach((v, i) => ($(`#q${i + 1}`).value = v));
  $("#btn-calc-pose").onclick = async () => {
    const b = { x: +$("#px").value, y: +$("#py").value, z: +$("#pz").value, rz: +$("#prz").value, ry: +$("#pry").value, rx: +$("#prx").value };
    const d = await orden("calcular_pose", b);
    if (d.ok) recibirPlan(d.plan, true);
  };
  $("#btn-calc-junta").onclick = async () => {
    const d = await orden("calcular_junta", { junta: +$("#j-sola").value, valor_deg: +$("#j-valor").value, relativo: $("#j-relativo").checked });
    if (d.ok) recibirPlan(d.plan, false);
  };
  $("#btn-calc-q").onclick = async () => {
    const q = [1, 2, 3, 4, 5, 6].map((i) => +$(`#q${i}`).value);
    const d = await orden("calcular_articular", { q_deg: q });
    if (d.ok) recibirPlan(d.plan, false);
  };
  $("#btn-ejecutar").onclick = () => ejecutar();
  $$("[data-cerrar]").forEach((b) => (b.onclick = () => b.closest(".modal").classList.add("oculto")));
  $("#jog-junta").onclick = () => fijarModoJog("junta");
  $("#jog-cart").onclick = () => fijarModoJog("cartesiano");
  $$(".flecha").forEach((b) => {
    const tecla = b.dataset.tecla;
    b.addEventListener("pointerdown", (e) => { e.preventDefault(); b.classList.add("apretada"); iniciarJog(tecla); });
    for (const ev of ["pointerup", "pointerleave", "pointercancel"]) b.addEventListener(ev, () => { b.classList.remove("apretada"); soltarJog(tecla); });
  });
  document.addEventListener("click", (e) => {
    if (e.target.id === "btn-sec") orden("secuencia", { pausar: $("#sec-pausar").checked });
    if (e.target.id === "btn-continuar") orden("continuar");
    if (e.target.id === "btn-armar") armar();
    if (e.target.id === "btn-desarmar") orden("desarmar");
  });
  const ramas = $("#ramas-contenido");
  ramas.addEventListener("mouseover", (e) => {
    const tr = e.target.closest("tr[data-rama]");
    est.ramaVista = tr ? +tr.dataset.rama : null;
  });
  ramas.addEventListener("mouseleave", () => (est.ramaVista = null));
  ramas.addEventListener("click", async (e) => {
    const tr = e.target.closest("tr[data-rama]");
    if (!tr || !est.plan) return;
    const r = est.plan.ik.ramas[+tr.dataset.rama];
    if (!r.valida) { aviso("Esa rama está descartada: " + (r.motivos || []).join("; "), "error"); return; }
    if (r.indice === est.plan.ik.elegida) return;
    const d = await orden("elegir_rama", { indice: r.indice });
    if (d.ok) { recibirPlan(d.plan, false); aviso(`Ahora se usa la rama ${r.indice}`, "ok"); }
  });
}

function abrirPestana(n) {
  est.pestana = n;
  $$(".pestanas button").forEach((b) => b.classList.toggle("activa", b.dataset.pest === n));
  $$(".pestana").forEach((p) => p.classList.toggle("activa", p.id === "p-" + n));
  redibujarLento(true);
  if (n === "lspb" && est.plan) dibujarLSPB(est.plan);
}

function elegirJunta(j) {
  est.jogJunta = j;
  $$("#jog-juntas button").forEach((b) => b.classList.toggle("activo", +b.dataset.j === j));
  $$(".tabla-juntas tbody tr").forEach((tr, i) => tr.classList.toggle("sel", est.jogModo === "junta" && i + 1 === j));
  ayudaJog();
}

function fijarModoJog(m) {
  est.jogModo = m;
  $("#jog-junta").classList.toggle("activo", m === "junta");
  $("#jog-cart").classList.toggle("activo", m === "cartesiano");
  $("#jog-juntas").style.opacity = m === "junta" ? 1 : 0.35;
  elegirJunta(est.jogJunta);
}

function ayudaJog() {
  $("#jog-ayuda").innerHTML = est.jogModo === "junta"
    ? `Modo <b>juntas</b>: teclas <b>1–6</b> eligen la junta (ahora <b>J${est.jogJunta}</b>). <b>→ / ↑</b> la giran en sentido positivo y <b>← / ↓</b> en negativo. Solo se mueve <b>mientras mantienes</b> la tecla.`
    : `Modo <b>cartesiano</b>: la punta se mueve en línea recta en la base del robot. <b>↑/↓</b> = ±X (adelante/atrás), <b>←/→</b> = ±Y (izquierda/derecha), <b>Re Pág / Av Pág</b> o <b>W/S</b> = ±Z. Cada paso se resuelve con Pieper (8 ramas → la más cercana) en vivo. La orientación se mantiene.`;
}

// Teclado
const teclasJog = new Set(["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "PageUp", "PageDown", "w", "s", "W", "S"]);
const jogs = {};

function jogDe(tecla) {
  if (est.jogModo === "junta") {
    if (["ArrowRight", "ArrowUp"].includes(tecla)) return { modo: "junta", eje: est.jogJunta, dir: 1 };
    if (["ArrowLeft", "ArrowDown"].includes(tecla)) return { modo: "junta", eje: est.jogJunta, dir: -1 };
    return null;
  }
  const mapa = { ArrowUp: [1, 1], ArrowDown: [1, -1], ArrowLeft: [2, 1], ArrowRight: [2, -1], PageUp: [3, 1], w: [3, 1], W: [3, 1], PageDown: [3, -1], s: [3, -1], S: [3, -1] };
  const m = mapa[tecla];
  return m ? { modo: "cartesiano", eje: m[0], dir: m[1] } : null;
}

function iniciarJog(tecla) {
  if (jogs[tecla]) return;
  const j = jogDe(tecla);
  if (!j) return;
  const enviar = async () => {
    const d = await orden("jog", { accion: "mantener", ...j });
    if (!d.ok && jogs[tecla]) { aviso(d.mensaje, "error"); soltarJog(tecla); }
  };
  enviar();
  jogs[tecla] = setInterval(enviar, 100);
}

function soltarJog(tecla) {
  if (!jogs[tecla]) return;
  clearInterval(jogs[tecla]);
  delete jogs[tecla];
  orden("jog", { accion: "soltar" });
}

function conectarTeclado() {
  document.addEventListener("keydown", (e) => {
    const enCampo = ["INPUT", "SELECT", "TEXTAREA"].includes(document.activeElement.tagName);
    if (e.key === "Escape") { parar(); e.preventDefault(); return; }
    if (enCampo) return;
    if (e.key === " ") { parar(); e.preventDefault(); return; }
    if (teclasJog.has(e.key)) { e.preventDefault(); if (!e.repeat) iniciarJog(e.key); return; }
    if (e.repeat) return;
    if (/^[1-6]$/.test(e.key)) { fijarModoJog("junta"); elegirJunta(+e.key); return; }
    switch (e.key) {
      case "Enter": ejecutar(); break;
      case "m": case "M": fijarModoJog(est.jogModo === "junta" ? "cartesiano" : "junta"); break;
      case "c": case "C": orden("continuar"); break;
      case "f": case "F": { const c = $("#ver-marcos"); c.checked = !c.checked; est.escena.medido.verMarcos(c.checked); break; }
      case "h": case "H": case "?": $("#ayuda").classList.toggle("oculto"); break;
      case "+": cambiarVel(+1); break;
      case "-": cambiarVel(-1); break;
    }
  });
  document.addEventListener("keyup", (e) => soltarJog(e.key));
  window.addEventListener("blur", () => Object.keys(jogs).forEach(soltarJog));
}

async function cambiarVel(s) {
  const v = Math.max(1, (+$("#vel").value) + s * (+$("#vel").value >= 10 ? 5 : 1));
  const d = await orden("velocidad", { pct: v });
  if (d.ok) { $("#vel").value = d.vel_pct; $("#vel-txt").textContent = Math.round(d.vel_pct) + " %"; }
}

async function parar() {
  Object.keys(jogs).forEach(soltarJog);
  const d = await orden("parar", { motivo: "botón PARAR" });
  if (d.ok) aviso("PARAR enviado", "error");
}

async function armar() {
  const cuerpo = { checklist: {}, frase: "" };
  $$(".caja-armado input[type=checkbox]").forEach((c) => (cuerpo.checklist[c.dataset.k] = c.checked));
  const fr = $(".caja-armado input[type=text]");
  if (fr) cuerpo.frase = fr.value;
  const d = await orden("armar", cuerpo);
  if (d.ok) aviso("Movimiento armado", "ok");
}

async function ejecutar() {
  if (!est.plan || !est.plan.ejecutable) { aviso("No hay un plan ejecutable.", "error"); return; }
  const v = est.vivo;
  if (v && v.modo && v.modo.exige_go) {
    const p = est.plan;
    $("#confirmar-texto").innerHTML = `<p><b>${p.descripcion}</b></p><p>Duración ${f(p.tray.tf, 2)} s · velocidad ${f(p.vel_pct, 0)} % · junta que manda: J${p.tray.junta_limitante}</p>
      <p>Destino: q = [${p.q_obj_deg.map((x) => f(x, 1)).join(", ")}] °</p><p>¿El área está despejada y la persona del FlexPendant tiene la mano en el paro de emergencia?</p>`;
    $("#confirmar").classList.remove("oculto");
    $("#confirmar-si").onclick = async () => { $("#confirmar").classList.add("oculto"); await enviarPlan(); };
    return;
  }
  await enviarPlan();
}

async function enviarPlan() {
  const d = await orden("ejecutar");
  if (d.ok) { aviso("Enviado al robot: " + d.mensaje, "ok"); est.preview.activo = false; }
}

// Planes
function recibirPlan(plan, irAPasos) {
  est.plan = plan;
  est.planEjecutado = null;
  const r = $("#resumen-plan");
  let h = `<b>Plan ${plan.id}</b> · ${plan.descripcion} · <small>${plan.creado}</small><br>`;
  if (plan.tipo === "pose" && plan.ik) {
    const el = plan.ik.elegida;
    const nval = plan.ik.ramas.filter((x) => x.valida).length;
    const nex = plan.ik.ramas.filter((x) => x.existe).length;
    h += `Pieper: ${nex} ramas existen, ${nval} válidas. ${el != null ? `<b>Elegida: rama ${el}</b> (${plan.ik.ramas[el].nombre}).` : "<b>Ninguna rama sirve.</b>"}<br>`;
    if (plan.verificacion) h += `Verificación FK: e<sub>p</sub> = ${plan.verificacion.e_p_m.toExponential(1)} m, e<sub>R</sub> = ${plan.verificacion.e_R_rad.toExponential(1)} rad.<br>`;
  }
  if (plan.tray) h += `Trayectoria LSPB: t<sub>f</sub> = ${f(plan.tray.tf, 2)} s (${plan.tray.motivo_tf}). Destino q = [${plan.q_obj_deg.map((x) => f(x, 1)).join(", ")}] °.<br>`;
  if (plan.problemas && plan.problemas.length) h += plan.problemas.map((p) => `<span class="problema">✗ ${p}</span>`).join("<br>");
  else h += '<span style="color:#3fd17a">✓ Revisión de seguridad: límites, altura mínima y singularidad OK. Listo para enviar.</span>';
  r.innerHTML = h;
  r.className = "resumen-plan " + (plan.ejecutable ? "ok" : "mal");
  $("#pasos-contenido").innerHTML = pasosPlan(plan, est.cfg.cfg);
  $("#ramas-contenido").innerHTML = tablaRamas(plan);
  $("#lspb-contenido").innerHTML = panelLSPB(plan);
  if (est.pestana === "lspb") dibujarLSPB(plan);
  est.escena.fijarCamino(plan.preview ? plan.preview.camino : []);
  est.preview = { activo: !!plan.preview, t0: performance.now() };
  if (irAPasos && est.pestana === "objetivo") aviso("Cálculo listo: revisa «Cálculo paso a paso» y «Las 8 ramas».", "ok");
  actualizarTuberia();
}

function dibujarLSPB(plan) {
  if (!plan.tray) return;
  est.gQ.fijar(plan.tray.t, plan.tray.q_deg);
  est.gQD.fijar(plan.tray.t, plan.tray.qd_deg_s);
  est.gQDD.fijar(plan.tray.t, plan.tray.qdd_deg_s2);
}

function animarVistaPrevia() {
  const paso = () => {
    const v = est.vivo, p = est.plan, esc = est.escena;
    const verF = $("#ver-fantasma").checked;
    if (esc) {
      if (est.ramaVista != null && p && p.ik && p.ik.ramas[est.ramaVista] && p.ik.ramas[est.ramaVista].marcos) {
        esc.fantasma.visible = true;
        esc.fantasma.actualizar(p.ik.ramas[est.ramaVista].marcos);
      } else if (v && v.ejec) {
        esc.fantasma.visible = verF && !!v.marcos_ref;
        if (v.marcos_ref) esc.fantasma.actualizar(v.marcos_ref);
      } else if (est.preview.activo && p && p.preview && p.preview.frames.length && (!v || !v.ejec)) {
        const tf = p.tray.tf, ciclo = tf + 1.2;
        const t = ((performance.now() - est.preview.t0) / 1000) % ciclo;
        const ts = p.preview.t;
        let k = ts.findIndex((x) => x >= Math.min(t, tf));
        if (k < 0) k = ts.length - 1;
        esc.fantasma.visible = verF;
        esc.fantasma.actualizar(p.preview.frames[k]);
      } else if (v && v.marcos_ref) {
        esc.fantasma.visible = verF;
        esc.fantasma.actualizar(v.marcos_ref);
      } else {
        esc.fantasma.visible = false;
      }
    }
    requestAnimationFrame(paso);
  };
  paso();
}

// Estado en vivo
function actualizar(v) {
  est.vivo = v;
  est.nVivo++;
  const m = v.modo || {};
  const banda = $("#banda-modo");
  if (m.solo_lectura) { banda.className = "banda lectura"; banda.textContent = m.texto; }
  else if (m.tipo === "sim") { banda.className = "banda sim"; banda.textContent = "SIMULACIÓN INTERNA · NO ES EL ROBOT REAL"; }
  else if (m.real === false) { banda.className = "banda mock"; banda.textContent = "ROS 2 · HARDWARE SIMULADO (use_fake_hardware) · NO ES EL ROBOT REAL"; }
  else if (m.exige_go && !m.permitir_real) { banda.className = "banda lectura"; banda.textContent = `${m.real ? "ROBOT REAL" : "HARDWARE SIN IDENTIFICAR"} · SOLO LECTURA (abre con --real y el GO para mover)`; }
  else { banda.className = "banda real"; banda.textContent = "ROBOT REAL · MOVIMIENTO HABILITADO · PARO DE EMERGENCIA A MANO"; }

  const pa = $("#estado-armado");
  pa.textContent = v.armado ? "ARMADO" : "DESARMADO";
  pa.className = "pastilla " + (v.armado ? "armado" : "desarmado");
  pa.title = v.armado ? "El robot puede moverse" : v.motivo_desarmado;
  $("#vel").max = m.vel_max_pct || 100;
  if (document.activeElement !== $("#vel")) { $("#vel").value = v.vel_pct; $("#vel-txt").textContent = Math.round(v.vel_pct) + " %"; }

  if (v.q_med_deg) {
    est.escena.medido.actualizar(v.marcos_med);
    const tcp = v.marcos_med[7];
    est.escena.agregarRastro([tcp[3], tcp[7], tcp[11]]);
    const p = v.tcp_med;
    $("#lectura-tcp").innerHTML = `TCP medido  x ${f(p.x_mm, 1)}  y ${f(p.y_mm, 1)}  z ${f(p.z_mm, 1)} mm<br>Rz ${f(p.rz_deg, 1)}°  Ry ${f(p.ry_deg, 1)}°  Rx ${f(p.rx_deg, 1)}°`;
    const s = v.sing;
    const av = $("#aviso-singular");
    const cerca = [];
    if (s.muneca_sin_q5 < 0.09) cerca.push("muñeca (q5 ≈ 0)");
    if (s.codo_sin_delta < 0.09) cerca.push("codo (brazo estirado)");
    if (s.hombro_r_w_m < 0.03) cerca.push("hombro (muñeca sobre el eje 1)");
    av.classList.toggle("oculto", !cerca.length);
    av.textContent = "Cerca de singularidad: " + cerca.join(", ") + `. σmin = ${f(s.sigma_min, 3)}`;
    actualizarJuntas(v);
  }

  const lim = est.cfg.cfg.seguridad.error_seguimiento_max_deg;
  const sem = $("#semaforo");
  if (v.err_deg) {
    const emax = Math.max(...v.err_deg.map(Math.abs));
    $("#n-eq").textContent = f(emax, 3) + "°";
    $("#n-ep").textContent = f(v.e_p_mm, 2) + " mm";
    $("#n-er").textContent = f(v.e_R_deg, 3) + "°";
    sem.className = "semaforo " + v.semaforo;
    sem.querySelector("span").textContent = { verde: "error bajo", ambar: `error alto (> ${lim / 2}°)`, rojo: `ERROR > ${lim}°` }[v.semaforo];
    est.gJuntas.agregar(v.t, v.err_deg);
    est.gTCP.agregar(v.t, [v.e_p_mm, v.e_R_deg]);
  } else {
    sem.className = "semaforo";
    sem.querySelector("span").textContent = "todavía no se comandó nada";
  }
  if (v.sing) $("#n-sig").textContent = f(v.sing.sigma_min, 3);

  const barra = $("#barra-ejec");
  if (v.ejec) {
    barra.classList.remove("oculto");
    barra.querySelector("div").style.width = (100 * v.ejec.progreso).toFixed(1) + "%";
    barra.querySelector("span").textContent = `Plan ${v.ejec.plan_id}: ${f(v.ejec.t, 1)} / ${f(v.ejec.tf, 1)} s · error máx. ${f(v.ejec.err_max_deg, 3)}° · e_p máx. ${f(v.ejec.e_p_max_mm, 2)} mm`;
    if (est.plan && v.ejec.plan_id === est.plan.id) {
      est.planEjecutado = est.plan.id;
      est.gQ.marcar(v.ejec.t); est.gQD.marcar(v.ejec.t); est.gQDD.marcar(v.ejec.t);
    }
  } else {
    barra.classList.add("oculto");
  }
  $("#btn-ejecutar").disabled = !(est.plan && est.plan.ejecutable && v.armado && !v.ejec && !v.jog);
  $("#btn-ejecutar").textContent = v.armado ? "ENVIAR AL ROBOT (Enter)" : "ENVIAR AL ROBOT (primero ARMAR)";
  if (v.ik_vivo && est.plan == null) { /* jog cartesiano sin plan: nada extra */ }
  actualizarTuberia();
  cajaArmado(v);
}

function actualizarJuntas(v) {
  const lims = est.cfg.limites_deg;
  for (let i = 0; i < 6; i++) {
    const fila = $(`#fila-j${i + 1}`);
    const [lo, hi] = lims[i];
    fila.querySelector(".med").textContent = f(v.q_med_deg[i], 2) + "°";
    fila.querySelector("td.cmd").textContent = v.q_ref_deg ? f(v.q_ref_deg[i], 2) + "°" : "—";
    const e = v.err_deg ? v.err_deg[i] : null;
    const tdE = fila.querySelector(".err");
    tdE.textContent = e == null ? "—" : f(e, 3) + "°";
    const lim = est.cfg.cfg.seguridad.error_seguimiento_max_deg;
    tdE.style.color = e == null ? "" : Math.abs(e) > lim ? "#ff5a5f" : Math.abs(e) > lim / 2 ? "#f2c14e" : "#3fd17a";
    fila.querySelector(".rango .pos").style.left = `calc(${(100 * (v.q_med_deg[i] - lo)) / (hi - lo)}% - 2px)`;
    const c = fila.querySelector(".rango .cmd");
    c.style.display = v.q_ref_deg ? "" : "none";
    if (v.q_ref_deg) c.style.left = `calc(${(100 * (v.q_ref_deg[i] - lo)) / (hi - lo)}% - 1px)`;
  }
}

function actualizarTuberia() {
  const li = $$("#tuberia li");
  li.forEach((l) => (l.className = ""));
  const p = est.plan, v = est.vivo;
  if (!p) return;
  const marca = (n, c) => (li[n - 1].className = c);
  marca(1, "hecho");
  if (p.tipo === "pose") {
    const el = p.ik && p.ik.elegida;
    marca(2, p.ik && p.ik.ramas.some((r) => r.existe) ? "hecho" : "fallo");
    marca(3, el != null ? "hecho" : "fallo");
    marca(4, el != null ? "hecho" : "fallo");
  } else {
    marca(2, "hecho"); marca(3, "hecho"); marca(4, "hecho");
  }
  if (p.tray) marca(5, p.ejecutable ? "hecho" : "fallo");
  if (v && v.ejec && v.ejec.plan_id === p.id) { marca(6, "hecho"); marca(7, "activo"); }
  else if (est.planEjecutado === p.id) { marca(6, "hecho"); marca(7, "hecho"); }
  else if (p.ejecutable) marca(6, "activo");
}

function cajaArmado(v) {
  const m = v.modo || {};
  const clave = [v.armado, m.exige_go, m.permitir_real, m.solo_lectura, v.motivo_desarmado].join("|");
  if (clave === est.ultimoCaja) return;
  est.ultimoCaja = clave;
  const c = $("#caja-armado");
  if (m.solo_lectura) {
    c.textContent = "Solo lectura ROS. Se pueden consultar estados y calcular planes. El armado está deshabilitado.";
  } else if (v.armado) {
    c.innerHTML = `<b style="color:#ff9b3d">MOVIMIENTO ARMADO.</b> El botón ENVIAR y las flechas mueven el robot.<br><button id="btn-desarmar" class="btn-desarmar" style="margin-top:6px">Desarmar</button>`;
  } else if (m.exige_go && !m.permitir_real) {
    c.innerHTML = `<b style="color:#f2c14e">Solo lectura.</b> Se detectó un robot real (o un hardware que no se pudo identificar). Se ven todos los cálculos y la posición real, pero no se puede mover. Para mover: cerrar y abrir con <code>--real --go GO_VEVAS.json</code>.`;
  } else if (m.exige_go) {
    c.innerHTML = `<b style="color:#ff5a5f">Armar el ROBOT REAL</b><br>
      <label><input type="checkbox" data-k="supervisor"> Supervisor presente y GO del día firmado</label>
      <label><input type="checkbox" data-k="paro"> Persona con la mano en el paro de emergencia del FlexPendant</label>
      <label><input type="checkbox" data-k="area"> Área despejada (nadie a menos de 1 m del alcance del robot)</label>
      <label><input type="checkbox" data-k="velocidad"> Velocidad baja revisada (máx. ${m.vel_max_pct} %)</label>
      <input type="text" placeholder="Escribe: ${est.cfg.frase_real}">
      <button id="btn-armar" class="btn-armar">ARMAR ROBOT REAL</button>
      <small>${v.motivo_desarmado || ""}</small>`;
  } else {
    c.innerHTML = `Armar habilita el botón ENVIAR y las flechas. En simulación no se pide el GO.<br><small>${v.motivo_desarmado || ""}</small><br><button id="btn-armar" class="btn-armar" style="margin-top:6px">ARMAR (simulación)</button>`;
  }
}

function redibujarLento(forzar = false) {
  const v = est.vivo;
  if (!v) return;
  if (est.pestana === "fk") $("#fk-contenido").innerHTML = panelFK(v);
  if (est.pestana === "jac") $("#jac-contenido").innerHTML = panelJacobiano(v);
  if (est.pestana === "bit") $("#bit-contenido").innerHTML = panelBitacora(v);
  if (est.pestana === "sec") {
    const clave = JSON.stringify(v.secuencia);
    if (clave !== est.ultimoSec || forzar) {
      const pausar = $("#sec-pausar") ? $("#sec-pausar").checked : true;
      est.ultimoSec = clave;
      $("#sec-contenido").innerHTML = panelSecuencia(v.secuencia, est.cfg.pasos_demo);
      if ($("#sec-pausar")) $("#sec-pausar").checked = pausar;
    }
  }
  // Si la secuencia generó un plan nuevo, traerlo para mostrar sus cálculos
  if (v.plan_id && (!est.plan || v.plan_id !== est.plan.id)) {
    fetch("/api/plan").then((r) => r.json()).then((p) => p && (!est.plan || p.id !== est.plan.id) && recibirPlan(p, false));
  }
}

iniciar();
