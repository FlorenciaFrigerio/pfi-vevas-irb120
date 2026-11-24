// Consola VEVAS: interfaz. Recibe el estado en vivo por SSE y manda órdenes por POST.
import { Escena, interpolarMarcos } from "./escena.js";
import { GraficaDeslizante, GraficaTrayectoria, COLORES_JUNTAS } from "./graficas.js";
import { crearArrastre } from "./arrastre.js";
import { marcosGiroJunta } from "./orientacion.js";
import { f, nombrePerfil, pasosPlan, permisoMovimientosGuiados, tablaRamas, panelLSPB, panelFK, panelJacobiano, panelSecuencia, panelBitacora,
  permisoIrAPose, proyectarFigura, errorFigura, agregarMuestra } from "./calculos.js";
import { dibujarVistaFigura } from "./vista_figura.js";

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
  ultimoGuiados: "",
  ultimoSec: "",
  nVivo: 0,
  verFalla: null,
  gz: { modo: "junta", acum: null },
  drag: null,
  revisionObjetivo: 0,
  calculoPendiente: false,
  // Vista de puntos de la figura: vista previa (diálogo) y plan calculado con los puntos medidos.
  figura: { vista: null, plan: null, medidos: [], dibujo: 0, pedido: 0 },
  poses: { lista: [], pendiente: false, editando: null, borrando: null, clave: "" },
};
let arr = null;

const esVirtual = (m) => m.tipo === "sim" || m.real === false;
function notaFuente(m) {
  if (m.tipo === "sim") return "Simulación · no conectado al brazo físico";
  if (m.real === false) return "ROS 2 · hardware simulado · no es el brazo físico";
  const via = m.tipo === "rws" ? "RWS" : "ROS 2";
  if (m.solo_lectura || (m.exige_go && !m.permitir_real)) return via + " · robot real · solo lectura";
  return via + " · ROBOT REAL · movimiento habilitado";
}

// ---- Revisión previa
const ETQ = { aprobado: "OK", aviso: "AVISO", rechazado: "RECHAZADO", no_verificado: "NO VERIFICADO" };
function dibujarRevision(plan) {
  const el = $("#panel-revision");
  const r = plan.revision;
  const detalle = $("#flujo-detalle");
  if (!r) { el.classList.add("oculto"); detalle.textContent = "No hay una trayectoria para revisar."; return; }
  el.classList.remove("oculto");
  const cinta = (r.cinta || []).map((c, i) => `<i class="c${c}" title="${((i + 0.5) / r.cinta.length * r.t_total).toFixed(2)} s"></i>`).join("");
  let h = `<div class="rev-cab"><b>Revisión previa</b><span class="rev-sello ${r.estado}">${ETQ[r.estado] || r.estado}</span></div>`;
  const sing = r.chequeos.find((c) => c.id === "singularidad" && c.estado === "rechazado");
  if (sing) h += `<div class="rev-sing"><b>No se puede enviar: la trayectoria pasa por (o cerca de) una singularidad.</b><small>${sing.detalle}</small></div>`;
  h += `<div class="rev-cinta" title="Recorrido de la trayectoria · rojo = punto con falla">${cinta}</div>`;
  h += `<ul class="rev-lista">` + r.chequeos.map((c) => `<li class="${c.estado}"><span class="rev-punto"></span><div><b>${c.nombre}</b><small>${c.detalle}</small></div></li>`).join("") + `</ul>`;
  if (r.falla) {
    h += `<div class="rev-falla"><b>${r.falla.check === "singularidad" ? "Instante crítico" : "Falla"}: ${r.chequeos.find((c) => c.id === r.falla.check)?.nombre || r.falla.check}</b> a los ${f(r.falla.t, 2)} s de ${f(r.t_total, 2)} s.
      <div class="rev-botones"><button id="rev-ver-falla" class="btn-chico">Ver la pose que falla</button><button id="rev-animar" class="btn-chico">Animar la trayectoria</button></div></div>`;
  } else if (r.bloquea_envio) {
    h += `<div class="rev-falla">Consultá los chequeos no verificados antes de volver a calcular.</div>`;
  }
  el.innerHTML = h;
  const bv = $("#rev-ver-falla"), ba = $("#rev-animar");
  if (bv) bv.onclick = () => { est.verFalla = r.falla; est.preview.activo = false; };
  if (ba) ba.onclick = () => { est.verFalla = null; est.preview = { activo: true, t0: performance.now() }; };
  detalle.innerHTML = h.replaceAll('id="rev-', 'id="flujo-rev-');
  for (const boton of [bv, ba]) {
    if (boton) $(`#flujo-${boton.id}`).onclick = () => { boton.onclick(); verPlanEn3D(); };
  }
}

// ---- Flechas sobre el modelo 3D
const PASOS = { junta: [1, 2, 5, 10], tcp: [5, 10, 25, 50] };
function gizmoActualizarPasos() {
  const sel = $("#gz-paso");
  const m = est.gz.modo;
  sel.innerHTML = PASOS[m].map((x) => `<option value="${x}">${x}${m === "junta" ? "°" : " mm"}</option>`).join("");
  sel.value = PASOS[m][1];
}
function gizmoTitulo() {
  const j = est.jogJunta;
  const nombres = ["Base", "Hombro", "Codo", "Antebrazo", "Muñeca", "Brida"];
  $("#gz-titulo").textContent = `J${j} · ${nombres[j - 1]}`;
  const v = est.vivo?.q_med_deg;
  const a = est.gz.acum;
  if (v) $("#gz-valor").textContent = a && a.junta === j ? `${f(a.valor, 1)}° (ahora ${f(v[j - 1], 1)}°)` : `${f(v[j - 1], 1)}°`;
}
function gizmoJunta(signo) {
  const v = est.vivo;
  if (!v?.q_med_deg || !v.marcos_med || v.ejec || v.jog || secuenciaActiva()) return;
  const j = est.jogJunta, paso = +$("#gz-paso").value;
  const valor = (est.gz.acum?.junta === j ? est.gz.acum.valor : v.q_med_deg[j - 1]) + signo * paso;
  const marcos = marcosGiroJunta(v.marcos_med, j, (valor - v.q_med_deg[j - 1]) * Math.PI / 180);
  const [min, max] = est.cfg.limites_deg[j - 1];
  const margen = est.cfg.cfg.ik.margen_limite_deg;
  est.gz.acum = { junta: j, valor, marcos, admisible: valor >= min + margen && valor <= max - margen };
  arr.reiniciar();
  invalidarPlan();
  $("#j-sola").value = j; $("#j-valor").value = valor.toFixed(1); $("#j-relativo").checked = false; sincronizarSlider();
  gizmoTitulo();
}
async function calcularGizmoJunta() {
  const a = est.gz.acum;
  if (!a) { aviso("Ajustá el objetivo de la junta primero.", "error"); return; }
  await calcularPlan("calcular_junta", { junta: a.junta, valor_deg: a.valor, relativo: false });
}
function dibujarFlechasJunta() {
  const el = $("#gz-flechas-junta"), visor = est.escena, v = est.vivo;
  const m = est.gz.abierto && est.gz.modo === "junta" && !v?.ejec && !v?.jog && v?.marcos_med?.[est.jogJunta];
  const p = m && visor.proyectar([m[3], m[7], m[11]]);
  el.classList.toggle("oculto", !p);
  if (!p) return;
  const x = Math.max(8, Math.min(visor.cont.clientWidth - el.offsetWidth - 8, p.x - el.offsetWidth - 12));
  const y = Math.max(88, Math.min(visor.cont.clientHeight - 36, p.y - 18)) + visor.cont.offsetTop;
  const anterior = est.gz.flechas;
  if (anterior && Math.hypot(x - anterior.x, y - anterior.y) < 2) return;
  est.gz.flechas = { x, y };
  el.style.transform = `translate(${Math.round(x)}px,${Math.round(y)}px)`;
}
function conectarGizmo() {
  gizmoActualizarPasos();
  const modo = (m) => {
    est.gz.modo = m; est.gz.acum = null;
    $("#gz-m-junta").classList.toggle("activo", m === "junta"); $("#gz-m-tcp").classList.toggle("activo", m === "tcp");
    $("#gz-junta").classList.toggle("oculto", m !== "junta"); $("#gz-tcp").classList.toggle("oculto", m !== "tcp");
    gizmoActualizarPasos();
    if (m === "junta") arr.descartar(true);
    gizmoTitulo();
  };
  $("#gz-m-junta").onclick = () => modo("junta");
  $("#gz-m-tcp").onclick = () => modo("tcp");
  $("#gz-junta-selector").onchange = (e) => elegirJunta(+e.target.value);
  $("#gz-menos").onclick = () => gizmoJunta(-1);
  $("#gz-mas").onclick = () => gizmoJunta(1);
  $$("#gz-tcp [data-e]").forEach((b) => (b.onclick = () => arr.trasladar(+b.dataset.e, +b.dataset.d * +$("#gz-paso").value)));
  $("#gz-calcular").onclick = () => {
    if (est.gz.modo === "junta") calcularGizmoJunta(); else arr.calcular();
  };
  $("#gz-limpiar").onclick = () => { est.gz.acum = null; arr.reiniciar(); invalidarPlan(); gizmoTitulo(); aviso("Cambios descartados. El siguiente objetivo parte de la posición medida.", "ok"); };
  window.addEventListener("vevas:joint", gizmoTitulo);
}

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
  const local = $("dialog[open] [data-mensaje-dialogo]");
  if (local) local.textContent = tipo === "error" ? texto : "";
  const t = $("#toast");
  t.textContent = texto;
  t.className = "toast " + tipo;
  clearTimeout(tToast);
  tToast = setTimeout(() => t.classList.add("oculto"), tipo === "error" ? 6000 : 3000);
}

// El servidor guarda un único plan, así que se permite un cálculo local en curso.
async function calcularPlan(nombre, cuerpo, irAPasos = false, desdeArrastre = false) {
  if (est.calculoPendiente || est.perfilPendiente || est.envioPendiente || est.vivo?.ejec || est.vivo?.jog || secuenciaActiva()) {
    aviso("Esperá a que termine la operación en curso.", "error");
    return { ok: false };
  }
  if (!desdeArrastre) arr.reiniciar();
  if (nombre !== "elegir_rama") invalidarPlan();
  const revision = est.revisionObjetivo;
  est.calculoPendiente = true;
  $("#btn-ejecutar").disabled = true;
  try {
    const d = await orden(nombre, cuerpo);
    if (revision !== est.revisionObjetivo) return { ok: false };
    if (d.ok) {
      recibirPlan(d.plan, irAPasos, desdeArrastre);
      if (nombre !== "calcular_figura" || $("#poses-predeterminadas").open) mostrarRevision();
    }
    return d;
  } finally {
    est.calculoPendiente = false;
    actualizarBotonEnvio();
  }
}

function secuenciaActiva() {
  return ["corriendo", "esperando"].includes(est.vivo?.secuencia?.estado);
}

// Construcción inicial
async function iniciar() {
  est.cfg = await (await fetch("/api/config")).json();
  const cfg = est.cfg;
  $("#perfil-plan").value = cfg.cfg.trayectoria?.perfil ?? "suave";
  const postura = cfg.cfg.postura_inicial_deg ?? cfg.postura_inicial_deg;

  const sel = $("#presets");
  sel.innerHTML = '<option value="">Poses de ejemplo…</option>' + cfg.presets.map((p, i) => `<option value="${i}">${p.nombre}</option>`).join("");
  sel.onchange = () => {
    if (sel.value === "") return;
    const p = cfg.presets[sel.value];
    if (!p) return;
    tipoObjetivo("pose");
    $("#px").value = p.x; $("#py").value = p.y; $("#pz").value = p.z;
    $("#prz").value = p.rz; $("#pry").value = p.ry; $("#prx").value = p.rx;
    arr.definirPose(p);
  };
  $("#seis-juntas").innerHTML = [1, 2, 3, 4, 5, 6].map((i) => `<label>q${i} <input id="q${i}" type="number" step="1" value="${postura[i - 1]}"><em>°</em></label>`).join("");
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
  const hayEntorno = est.escena.fijarEntorno(cfg.cfg.entorno);
  $("#opcion-entorno").classList.toggle("oculto", !hayEntorno);
  if (cfg.cfg.entorno.mesa) {
    const m = cfg.cfg.entorno.mesa;
    $("#opcion-entorno").title = `Aro de la mesa ${m.arista_superior_z_mm} mm · referencia del cilindro ${m.cilindro_base_z_mm} mm · travesaños ilustrativos`;
  }
  actualizarFrecuencia(cfg.cfg.servidor.hz_estado, null);
  const lim = cfg.cfg.seguridad.error_seguimiento_max_deg;
  est.gJuntas = new GraficaDeslizante($("#g-err-juntas"), {
    series: COLORES_JUNTAS.map((c, i) => ({ nombre: `e${i + 1}`, color: c })),
    unidad: "error junta [°]",
    lineas: [{ valor: lim, color: "#ff5a5f" }, { valor: -lim, color: "#ff5a5f" }],
  });
  est.gTCP = new GraficaDeslizante($("#g-err-tcp"), {
    series: [{ nombre: "e_p [mm]", color: "#2187a4" }],
    unidad: "posición [mm]",
  });
  est.gRot = new GraficaDeslizante($("#g-err-rot"), {series:[{nombre:"e_R [°]",color:"#9366aa"}],unidad:"orientación [°]"});
  est.historial=[];
  configurarGrafica();
  est.gQ = new GraficaTrayectoria($("#lspb-graficas"), "Posición q(t) [°]", "°");
  const cont2 = document.createElement("div"); $("#lspb-graficas").appendChild(cont2);
  const cont3 = document.createElement("div"); $("#lspb-graficas").appendChild(cont3);
  est.gQD = new GraficaTrayectoria(cont2, "Velocidad q̇(t) [°/s]", "°/s");
  est.gQDD = new GraficaTrayectoria(cont3, "Aceleración q̈(t) [°/s²]", "°/s²");

  arr = crearArrastre({ est, $, aviso, calcularPlan, invalidarPlan, abrirPestana });
  conectarControles();
  arr.iniciar();
  conectarTeclado();
  ayudaJog();
  setInterval(() => orden("pulso"), 250);
  setInterval(redibujarLento, 400);
  setInterval(() => { est.gJuntas.dibujar(); est.gTCP.dibujar(); est.gRot.dibujar(); if(est.pestana==="graficas") actualizarGrafica(); }, 100);
  animarVistaPrevia();
  abrirStream();
  cargarPoses();
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
  $("#btn-ayuda").onclick=()=>$("#ayuda").classList.remove("oculto");
  $$("[data-objetivo]").forEach(b=>b.onclick=()=>tipoObjetivo(b.dataset.objetivo));
  $("#j-sola").onchange=()=>elegirJunta(+$("#j-sola").value);
  $("#j-preview").oninput=e=>{ $("#j-valor").value=e.target.value;sincronizarSlider(); };
  $("#j-valor").oninput=sincronizarSlider;
  $("#j-relativo").onchange=sincronizarSlider;
  $$(".objetivo-form input,.objetivo-form select").forEach((el) => el.addEventListener("input", () => {
    arr.reiniciar();
    est.gz.acum = null;
    invalidarPlan();
  }));
  window.addEventListener("vevas:joint",e=>{abrirPestana("objetivo");tipoObjetivo("junta");elegirJunta(e.detail.junta);});
  $("#tipo-grafica").onchange=configurarGrafica;$("#junta-grafica").onchange=configurarGrafica;
  $("#btn-exportar").onclick=exportarCSV;
  $("#ventana-grafica").onchange=()=>{est.graficaPausada=false;$("#btn-pausar-grafica").textContent="Pausar gráfica";actualizarGrafica();};
  $("#btn-pausar-grafica").onclick=()=>{est.graficaPausada=!est.graficaPausada;$("#btn-pausar-grafica").textContent=est.graficaPausada?"Reanudar en vivo":"Pausar gráfica";};
  $("#modo-expo").onchange = (e) => document.body.classList.toggle("sin-expl", !e.target.checked);
  $("#btn-parar").onclick = () => parar();
  $("#btn-vista").onclick = () => est.escena.vistaInicial();
  $("#btn-vista-frente").onclick = () => est.escena.vistaFrontal?.();
  $("#ver-marcos").onchange = (e) => est.escena.medido.verMarcos(e.target.checked);
  $("#ver-zmin").onchange = (e) => (est.escena.planoZmin.visible = e.target.checked);
  $("#ver-entorno").onchange = (e) => est.escena.verEntorno(e.target.checked);
  $("#ver-camino").onchange = (e) => est.escena.verCamino?.(e.target.checked);
  $("#ver-traza").onchange = (e) => est.escena.verTraza?.(e.target.checked);
  $("#ver-puntos").onchange = (e) => est.escena.verPuntos?.(e.target.checked);
  $("#btn-limpiar-traza").onclick = () => est.escena.limpiarTraza?.();
  $("#frecuencia-visor").onchange = async (e) => {
    const select = e.target;
    select.disabled = true;
    const d = await orden("frecuencia", { hz: +select.value });
    select.disabled = false;
    if (d.ok) est.cfg.cfg.servidor.hz_estado = d.hz_vista;
    actualizarFrecuencia(est.cfg.cfg.servidor.hz_estado, d.hz_fuente ?? frecuenciaFuente());
  };
  $("#btn-calibracion").onclick = () => {
    $("#asistente-calibracion").classList.remove("oculto");
    $("#btn-comparar-pendant").focus();
  };
  $("#btn-comparar-pendant").onclick = compararPendant;
  $("#vel").oninput = (e) => ($("#vel-txt").textContent = e.target.value + " %");
  $("#vel").onchange = async (e) => {
    await fijarVelocidad(+e.target.value);
  };
  $("#btn-pose-actual").onclick = () => {
    invalidarPlan();
    const p = est.vivo && est.vivo.tcp_med;
    if (!p) return;
    $("#px").value = p.x_mm.toFixed(1); $("#py").value = p.y_mm.toFixed(1); $("#pz").value = p.z_mm.toFixed(1);
    $("#prz").value = p.rz_deg.toFixed(2); $("#pry").value = p.ry_deg.toFixed(2); $("#prx").value = p.rx_deg.toFixed(2);
  };
  $("#btn-q-actual").onclick = () => {invalidarPlan();est.vivo?.q_med_deg?.forEach((v,i)=>$(`#q${i+1}`).value=v.toFixed(2));};
  $("#btn-q-inicial").onclick = () => {
    invalidarPlan();
    (est.cfg.cfg.postura_inicial_deg ?? est.cfg.postura_inicial_deg).forEach((v, i) => { $(`#q${i + 1}`).value = v; });
  };
  $("#btn-calc-pose").onclick = async () => {
    const b = { x: +$("#px").value, y: +$("#py").value, z: +$("#pz").value, rz: +$("#prz").value, ry: +$("#pry").value, rx: +$("#prx").value };
    await calcularPlan("calcular_pose", b, true);
  };
  $("#btn-calc-junta").onclick = async () => {
    await calcularPlan("calcular_junta", { junta: +$("#j-sola").value, valor_deg: +$("#j-valor").value, relativo: $("#j-relativo").checked });
  };
  $("#btn-calc-q").onclick = async () => {
    const q = [1, 2, 3, 4, 5, 6].map((i) => +$(`#q${i}`).value);
    await calcularPlan("calcular_articular", { q_deg: q });
  };
  $("#btn-ejecutar").onclick = confirmarEjecucion;
  $("#confirmar-si").onclick = (e) => {
    if (e.detail <= 0 && !esVirtual(est.vivo?.modo || {})) return;
    enviarPlan(est.confirmarPlanId);
  };
  $("#btn-ver-revision").onclick = mostrarRevision;
  $("#btn-ver-revision-3d").onclick = mostrarRevision;
  $("#flujo-volver").onclick = () => etapaConfirmacion(false);
  $("#flujo-ver-3d").onclick = verPlanEn3D;
  $("#flujo-habilitar").onclick = () => armar();
  $("#flujo-plan").addEventListener("close", () => { est.confirmarPlanId = null; });
  $("#perfil-plan").onchange = cambiarPerfil;
  conectarFiguras();
  conectarPosesGuardadas();
  $("#pf-limpiar").onclick = () => { est.figura.medidos = []; dibujarPanelFigura(); };
  window.addEventListener("resize", () => { dibujarPanelFigura(); if ($("#poses-predeterminadas").open) dibujarVistaDialogo(); });
  conectarGizmo();
  $("#gz-toggle").onclick = () => {
    const g = $("#gizmo"), cerrado = g.classList.toggle("colapsado");
    est.gz.abierto = !cerrado;
    $("#gz-toggle").setAttribute("aria-expanded", String(!cerrado));
    $("#gz-toggle").textContent = cerrado ? "Mover con botones ▸" : "Mover con botones ▾";
  };
  $("#btn-home-j5").onclick = () => volverHome("j5");
  $("#btn-home-cero").onclick = () => volverHome("cero");
  $("#btn-home-inicial").onclick = () => volverHome("inicial");
  $$("[data-cerrar]").forEach((b) => (b.onclick = () => b.closest(".modal").classList.add("oculto")));
  $("#jog-junta").onclick = () => fijarModoJog("junta");
  $("#jog-cart").onclick = () => fijarModoJog("cartesiano");
  $$(".flecha").forEach((b) => {
    const tecla = b.dataset.tecla;
    b.addEventListener("pointerdown", (e) => { e.preventDefault(); b.classList.add("apretada"); iniciarJog(tecla); });
    for (const ev of ["pointerup", "pointerleave", "pointercancel"]) b.addEventListener(ev, () => { b.classList.remove("apretada"); soltarJog(tecla); });
  });
  document.addEventListener("click", (e) => {
    const activarMovimiento = e.detail > 0 || esVirtual(est.vivo?.modo || {});
    if (activarMovimiento && e.target.id === "btn-sec" && !est.calculoPendiente && !est.envioPendiente) {
      arr.reiniciar();
      invalidarPlan();
      orden("secuencia", { pausar: $("#sec-pausar").checked });
    }
    if (activarMovimiento && e.target.id === "btn-continuar") orden("continuar");
    if (["btn-armar", "btn-armar-dialogo"].includes(e.target.id)) armar(e.target.closest(".caja-armado"));
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
    const d = await calcularPlan("elegir_rama", { indice: r.indice });
    if (d.ok) aviso(`Ahora se usa la rama ${r.indice}`, "ok");
  });
}

function abrirPestana(n) {
  est.pestana = n;
  $$(".pestanas button").forEach((b) => b.classList.toggle("activa", b.dataset.pest === n));
  $$(".pestana").forEach((p) => p.classList.toggle("activa", p.id === "p-" + n));
  redibujarLento(true);
  if(n==="graficas")configurarGrafica();
  if (n === "lspb" && est.plan) dibujarLSPB(est.plan);
  if (n === "objetivo" && est.plan) dibujarMini(est.plan);
}

function elegirJunta(j) {
  if (est.jogJunta !== j) {
    if (est.gz.abierto && est.gz.modo === "junta") {
      est.gz.acum = null;
      arr.reiniciar();
    }
    invalidarPlan();
  }
  est.jogJunta = j;
  $("#gz-junta-selector").value = j;
  $("#j-sola").value=j;$("#btn-calc-junta").innerHTML=`Calcular y verificar J${j} <span>→</span>`;
  sincronizarSlider();
  $$("#jog-juntas button").forEach((b) => b.classList.toggle("activo", +b.dataset.j === j));
  $$(".tabla-juntas tbody tr").forEach((tr, i) => tr.classList.toggle("sel", est.jogModo === "junta" && i + 1 === j));
  ayudaJog();
  if (typeof gizmoTitulo === "function" && est.vivo) gizmoTitulo();
}

function fijarModoJog(m) {
  est.jogModo = m;
  $("#jog-junta").classList.toggle("activo", m === "junta");
  $("#jog-cart").classList.toggle("activo", m === "cartesiano");
  $("#jog-juntas").style.opacity = m === "junta" ? 1 : 0.35;
  elegirJunta(est.jogJunta);
}

function ayudaJog() {
  const teclado = esVirtual(est.vivo?.modo || {});
  $("#jog-ayuda").innerHTML = est.jogModo === "junta"
    ? `Mantené pulsada una flecha para mover <b>J${est.jogJunta}</b>. ${teclado ? "En simulación también podés usar las teclas de flecha." : ""}`
    : `Usá las flechas para mover el TCP en los ejes fijos de la base. La orientación se mantiene. ${teclado ? "En simulación, ↑/↓ = ±X, ←/→ = ±Y y W/S = ±Z." : ""}`;
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
  if (est.vivo?.modo?.tipo === "rws") return;
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
    const foco = document.activeElement.tagName;
    const enCampo = ["INPUT", "SELECT", "TEXTAREA"].includes(foco);
    const simulacion = esVirtual(est.vivo?.modo || {});
    if ($("dialog[open]") && (e.key !== "Escape" || !(est.envioPendiente || est.vivo?.ejec || est.vivo?.jog))) return;
    if (e.key === "Escape") { parar(); e.preventDefault(); return; }
    if (enCampo) return;
    if (["BUTTON", "A"].includes(foco) && ["Enter", " "].includes(e.key)) return;
    if (e.key === " ") { parar(); e.preventDefault(); return; }
    if (teclasJog.has(e.key) && simulacion) { e.preventDefault(); if (!e.repeat) iniciarJog(e.key); return; }
    if (e.repeat) return;
    if (/^[1-6]$/.test(e.key)) { fijarModoJog("junta"); elegirJunta(+e.key); return; }
    switch (e.key) {
      case "Enter": if (simulacion) mostrarRevision(); break;
      case "m": case "M": fijarModoJog(est.jogModo === "junta" ? "cartesiano" : "junta"); break;
      case "c": case "C": if (simulacion) orden("continuar"); break;
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
  await fijarVelocidad(v);
}

async function fijarVelocidad(pct) {
  const d = await orden("velocidad", { pct });
  if (!d.ok) return;
  if (est.vivo?.modo?.tipo === "sim" && est.plan && est.plan.vel_pct !== d.vel_pct) {
    invalidarPlan();
    aviso("Velocidad del próximo plan actualizada. Recalculá y revisá la trayectoria.", "info");
  }
  $("#vel").value = d.vel_pct;
  $("#vel-txt").textContent = Math.round(d.vel_pct) + " %";
}

async function volverHome(destino) {
  if (est.vivo?.ejec) { aviso("Esperá a que termine el movimiento en curso.", "error"); return; }
  abrirPestana("objetivo");
  await calcularPlan("calcular_home", { destino });
}

async function parar() {
  Object.keys(jogs).forEach(soltarJog);
  const d = await orden("parar", { motivo: "botón PARAR" });
  if (d.ok) aviso(d.mensaje, "error");
}

function frecuenciaFuente() {
  return est.vivo?.modo?.tipo === "rws" ? est.cfg.cfg.rws.hz_lectura : null;
}
function actualizarFrecuencia(hz, fuente) {
  $("#frecuencia-visor").value = hz;
  $("#frecuencia-fuente").textContent = fuente == null ? "" : `Lectura RWS · ${fuente} Hz`;
}
async function compararPendant() {
  const boton = $("#btn-comparar-pendant"), resultado = $("#calibracion-resultado");
  boton.disabled = true;
  resultado.textContent = "Leyendo juntas y pose del controlador…";
  try {
    const r = await fetch("/api/verificacion_pendant");
    const d = await r.json();
    if (!r.ok || !d.ok) { resultado.textContent = d.mensaje || "No se pudo leer el controlador."; return; }
    const lista = (v, dec = 3) => v.map((x) => f(x, dec)).join(" · ");
    const filas = [
      ["J1–J6 (°)", lista(d.juntas_deg), lista(d.juntas_modelo_deg)],
      ["XYZ (mm)", lista(d.controlador.xyz_mm), lista(d.modelo.xyz_mm)],
      ["Cuaternión ABB (w, x, y, z)", lista(d.controlador.cuaternion_abb_wxyz, 5), lista(d.modelo.cuaternion_abb_wxyz, 5)],
      ["Euler ZYX (°)", lista(d.controlador.euler_zyx_deg) + (d.controlador.gimbal_zyx ? " *" : ""), lista(d.modelo.euler_zyx_deg) + (d.modelo.gimbal_zyx ? " *" : "")],
      ["Euler ZXY (°)", lista(d.controlador.euler_zxy_deg) + (d.controlador.gimbal_zxy ? " *" : ""), lista(d.modelo.euler_zxy_deg) + (d.modelo.gimbal_zxy ? " *" : "")],
    ];
    resultado.innerHTML = `<p class="marco-pendant"></p><table class="tabla-simple"><thead><tr><th>Lectura</th><th>Controlador (RWS)</th><th>Modelo FK</th></tr></thead><tbody>${filas.map((fila) => `<tr>${fila.map((v) => `<td>${v}</td>`).join("")}</tr>`).join("")}</tbody></table>
      <p><b>${d.coincide ? "Coincide dentro de las tolerancias" : "Diferencia fuera de tolerancia"}</b> · posición ${f(d.error_posicion_mm, 3)} mm (límite ${f(d.tolerancia_posicion_mm, 2)} mm) · orientación ${f(d.error_orientacion_deg, 3)}° (límite ${f(d.tolerancia_orientacion_deg, 2)}°).</p><p class="estado-calibracion"></p>`;
    const marco = d.frame;
    resultado.querySelector(".marco-pendant").textContent = `${marco.unidad_mecanica} · ${marco.herramienta} · ${marco.objeto} · ${marco.coordenadas} · variación máxima durante la captura ${f(d.variacion_juntas_deg, 3)}°`;
    resultado.querySelector(".estado-calibracion").textContent = `Calibración: ${d.calibracion.estado}. Consultá el estado en el ${d.calibracion.fuente}.`;
    if (filas.some((fila) => fila.some((v) => v.endsWith("*")))) {
      const nota = document.createElement("p");
      nota.textContent = "* En bloqueo de cardán los ángulos Euler no son únicos. Compará el cuaternión y el error de orientación.";
      resultado.append(nota);
    }
  } catch {
    resultado.textContent = "No se pudo consultar la verificación.";
  } finally {
    boton.disabled = false;
  }
}

async function armar(caja = null) {
  if (est.armadoPendiente) return;
  const cuerpo = { checklist: {}, frase: "" };
  caja?.querySelectorAll('input[type="checkbox"]').forEach((c) => (cuerpo.checklist[c.dataset.k] = c.checked));
  cuerpo.frase = caja?.querySelector('input[type="text"]')?.value || "";
  est.armadoPendiente = true;
  actualizarBotonEnvio();
  try {
    const d = await orden("armar", cuerpo);
    if (d.ok) aviso(esVirtual(est.vivo?.modo || {}) ? "Simulador habilitado" : "Movimiento armado", "ok");
  } finally {
    est.armadoPendiente = false;
    actualizarBotonEnvio();
  }
}

function mostrarRevision() {
  if (!est.plan) return;
  const dialogo = $("#flujo-plan");
  $("#poses-predeterminadas").close();
  etapaConfirmacion(false);
  if (!dialogo.open) dialogo.showModal();
  $("#flujo-titulo").focus();
}

function verPlanEn3D() {
  $("#flujo-plan").close();
  $(".escena").scrollIntoView({ block: "center" });
  $("#btn-ver-revision-3d").focus({ preventScroll: true });
}

function etapaConfirmacion(confirmar) {
  const p = est.plan, virtual = esVirtual(est.vivo?.modo || {});
  est.confirmarPlanId = confirmar ? p.id : null;
  $("#flujo-titulo").textContent = confirmar
    ? (virtual ? "Confirmar ejecución en simulación" : "Confirmar envío al robot real")
    : `Revisar plan ${p.id}`;
  $("#flujo-revision").classList.toggle("oculto", confirmar);
  $("#flujo-confirmacion").classList.toggle("oculto", !confirmar);
  $("#flujo-revision-acciones").classList.toggle("oculto", confirmar);
  $("#flujo-confirmar-acciones").classList.toggle("oculto", !confirmar);
  $("#flujo-plan .flujo-cuerpo").scrollTop = 0;
  $("#flujo-plan [data-mensaje-dialogo]").textContent = "";
  $("#confirmar-si").textContent = virtual ? "Confirmar ejecución" : "Confirmar envío";
  if (confirmar) {
    const texto = $("#confirmar-texto");
    texto.replaceChildren();
    for (const linea of [
      `Plan ${p.id} · ${p.descripcion}`,
      `${nombrePerfil(p.tray.perfil)} · ${f(p.tray.tf, 2)} s · velocidad del plan ${f(p.vel_pct, 0)} %`,
      `Destino q = [${p.q_obj_deg.map((x) => f(x, 1)).join(", ")}] °`,
      virtual ? "La ejecución comienza al pulsar «Confirmar ejecución»." : "Confirmá que el área esté despejada y que la persona del FlexPendant tenga la mano en el paro de emergencia.",
    ]) {
      const lineaTexto = document.createElement("p");
      lineaTexto.textContent = linea;
      texto.append(lineaTexto);
    }
  }
  actualizarBotonEnvio();
  if ($("#flujo-plan").open) $("#flujo-titulo").focus();
}

function confirmarEjecucion() {
  if (puedeEnviar()) etapaConfirmacion(true);
}

function estadoActual() {
  return !!est.ultimaRecepcion && performance.now() - est.ultimaRecepcion <= 2000;
}
function puedeEnviar() {
  const v = est.vivo;
  return !!(est.plan?.ejecutable && v?.armado && estadoActual() && !v.ejec && !v.jog && !secuenciaActiva() && !est.calculoPendiente && !est.perfilPendiente && !est.envioPendiente);
}
function actualizarBotonEnvio() {
  const v = est.vivo, p = est.plan, virtual = esVirtual(v?.modo || {}), disponible = puedeEnviar();
  $("#btn-ejecutar").disabled = !disponible;
  $("#confirmar-si").disabled = !disponible || p?.id !== est.confirmarPlanId;
  $("#btn-ver-revision").classList.toggle("oculto", !p);
  $("#btn-ver-revision-3d").classList.toggle("oculto", !p);
  const habilitar = $("#flujo-habilitar");
  habilitar.classList.toggle("oculto", !p?.ejecutable || !virtual || !!v?.armado);
  habilitar.disabled = est.armadoPendiente || !estadoActual() || !!v?.ejec || !!v?.jog;
  if (!$("#flujo-plan").open) return;
  const texto = bloqueoC02(v?.modo) ? avisoC02
    : !p?.ejecutable ? "La revisión bloquea la ejecución. Consultá el detalle del plan."
    : est.envioPendiente ? "Enviando el plan confirmado…"
    : est.calculoPendiente ? "Finalizando el cálculo…"
    : !estadoActual() ? "Esperando un estado reciente de la consola."
    : v.ejec || v.jog || secuenciaActiva() ? "Ya hay un movimiento en curso."
    : !v.armado ? (virtual ? "Habilitá el simulador para continuar." : "El armado del robot debe estar habilitado para continuar.")
    : est.confirmarPlanId ? "Confirmá para ejecutar el plan revisado."
    : virtual ? "Plan disponible en simulación. Revisá el resultado y continuá."
      : "Revisá el resultado antes de continuar al envío.";
  const estado = $("#flujo-estado");
  if (estado.textContent !== texto) estado.textContent = texto;
}
async function enviarPlan(id) {
  if (!id || est.plan?.id !== id || !puedeEnviar()) return;
  est.envioPendiente = true;
  actualizarBotonEnvio();
  try {
    const d = await orden("ejecutar", { plan_id: id });
    if (d.ok) {
      $("#flujo-plan").close();
      est.preview.activo = false;
      est.gz.acum = null;
      arr.reiniciar();
      if (esVirtual(est.vivo?.modo || {})) $(".escena").scrollIntoView({ block: "center" });
      aviso((esVirtual(est.vivo?.modo || {}) ? "Simulación: " : "Robot: ") + d.mensaje, "ok");
    }
  } finally {
    est.envioPendiente = false;
    actualizarBotonEnvio();
  }
}

async function cambiarPerfil(e) {
  const select = e.target;
  const trayectoria = est.cfg.cfg.trayectoria;
  est.perfilPendiente = true;
  select.disabled = true;
  invalidarPlan();
  try {
    const d = await orden("perfil", { perfil: select.value });
    if (d.ok) trayectoria.perfil = d.perfil;
  } finally {
    est.perfilPendiente = false;
    select.value = trayectoria.perfil ?? "suave";
    select.disabled = false;
    actualizarBotonEnvio();
  }
}

function conectarFiguras() {
  const modal = $("#poses-predeterminadas"), form = $("#form-figura");
  const descripciones = {
    circulo: "Indicá el diámetro del círculo.", corazon: "Indicá el ancho del corazón.",
    cuadrado: "Indicá el lado del cuadrado. Se detiene en cada esquina.",
    triangulo: "Indicá el lado del triángulo equilátero. Se detiene en cada esquina.",
    saludo: "La mano va y viene en arco tres veces; el tamaño es el ancho del vaivén.",
  };
  const actualizarDescripcion = () => {
    const figura = $("#figura-tipo").value;
    $("#figura-descripcion").textContent = descripciones[figura];
  };
  $("#btn-poses-predeterminadas").onclick = () => {
    const permiso = permisoMovimientosGuiados(est.vivo?.modo);
    if (!modal.open && permiso.permitido) {
      const tamano = $("#figura-tamano");
      tamano.max = String(permiso.maxMm);
      if (tamano.valueAsNumber > permiso.maxMm) tamano.value = String(permiso.maxMm);
      $("#figura-modo").textContent = permiso.textoDialogo;
      actualizarDescripcion();
      modal.querySelector("[data-mensaje-dialogo]").textContent = "";
      modal.showModal();
      pedirVistaFigura();
    }
  };
  let tVista = null;
  form.addEventListener("input", () => {
    invalidarPlan();
    actualizarDescripcion();
    clearTimeout(tVista);
    tVista = setTimeout(pedirVistaFigura, 120);
  });
  // Al cerrar sin calcular, los puntos de la vista previa no quedan en el 3D.
  modal.addEventListener("close", () => { if (!est.plan) est.escena.fijarPuntos([]); });
  form.onsubmit = async (e) => {
    e.preventDefault();
    if (!permisoMovimientosGuiados(est.vivo?.modo).permitido) {
      modal.querySelector("[data-mensaje-dialogo]").textContent = permisoMovimientosGuiados(est.vivo?.modo).tituloFiguras;
      return;
    }
    const boton = $("#btn-calcular-figura");
    boton.disabled = true;
    boton.textContent = "Calculando…";
    modal.querySelector("[data-mensaje-dialogo]").textContent = "";
    const cuerpo = { figura: $("#figura-tipo").value, tamano_mm: $("#figura-tamano").valueAsNumber, plano: $("#figura-plano").value };
    try {
      await calcularPlan("calcular_figura", cuerpo);
      if (cuerpo.plano === "YZ") est.escena.vistaFrontal?.();   // la figura de frente se mira de frente
    }
    finally { boton.disabled = false; boton.textContent = "Calcular trayectoria"; }
  };
}

// Vista previa de puntos: la geometría viene de vevas/figuras.py (endpoint que no mueve nada).
async function pedirVistaFigura() {
  const n = ++est.figura.pedido;
  const cuerpo = { figura: $("#figura-tipo").value, tamano_mm: $("#figura-tamano").valueAsNumber, plano: $("#figura-plano").value };
  let d;
  try {
    const r = await fetch("/api/vista_figura", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(cuerpo) });
    d = await r.json();
  } catch {
    d = { ok: false, mensaje: "Sin conexión con la consola." };
  }
  if (n !== est.figura.pedido) return;
  est.figura.vista = d.ok ? d.vista : null;
  const info = $("#figura-vista-info");
  if (!d.ok) {
    info.textContent = d.mensaje || "No se pudo generar la vista previa.";
  } else {
    const v = d.vista, np = v.paradas.length;
    info.textContent = `${v.n_puntos} puntos · plano ${v.plano_vista}`
      + ` · ${np ? `${np} ${np === 1 ? "parada" : "paradas"} (v = 0)` : "sin paradas intermedias"}`
      + (v.origen_m ? " · origen en el TCP actual" : "")
      + (v.excede_tope ? ` · supera el máximo de ${v.tope_mm} mm de este modo` : "");
    // También en el gemelo 3D (si todavía no hay un plan calculado).
    if (v.puntos_m && !est.plan) est.escena.fijarPuntos(v.puntos_m);
  }
  dibujarVistaDialogo();
}

function dibujarVistaDialogo() {
  const v = est.figura.vista;
  dibujarVistaFigura($("#figura-lienzo"), v
    ? { plan2d: v.puntos_2d_mm, paradas: v.paradas, ejes: v.ejes }
    : { vacio: $("#figura-vista-info").textContent || "Elegí figura y tamaño." });
}

function figuraDePlan(plan) {
  const fig = plan.figura, puntos = fig.puntos_objetivo_m || plan.preview?.puntos || [];
  const plano = fig.plano_vista || fig.plano || "XY";
  return { plan_id: plan.id, descripcion: plan.descripcion, puntos_m: puntos, plano, paradas: fig.paradas || [],
    ejes: [plano[0], plano[1]], origen_m: puntos[0] };
}

function dibujarPanelFigura() {
  const panel = $("#panel-figura"), fp = est.figura.plan;
  panel.classList.toggle("oculto", !fp);
  if (!fp) return;
  const med = est.figura.medidos;
  $("#pf-titulo").textContent = `Recorrido de la figura · plan ${fp.plan_id} · ${fp.descripcion}`;
  dibujarVistaFigura($("#pf-lienzo"), {
    plan2d: proyectarFigura(fp.puntos_m, fp.plano, fp.origen_m),
    medidos2d: proyectarFigura(med, fp.plano, fp.origen_m),
    paradas: fp.paradas, ejes: fp.ejes,
  });
  const e = errorFigura(med, fp.puntos_m);
  $("#pf-error").textContent = e.n
    ? `Medido vs planificado (distancia 3D al recorrido): máx ${f(e.max_mm, 2)} mm · medio ${f(e.medio_mm, 2)} mm · ${e.n} puntos`
    : "Al ejecutar el plan se superponen aquí los puntos medidos del TCP.";
}

function seguirFigura(v) {
  const fp = est.figura.plan;
  const siguiendo = !!(fp && v.ejec && v.ejec.plan_id === fp.plan_id);
  if (est.figura.siguiendo && !siguiendo) dibujarPanelFigura();   // terminó: dibujo final con el error completo
  est.figura.siguiendo = siguiendo;
  if (!siguiendo || !v.tcp_med) return;
  if (!(Number.isFinite(v.edad_estado_s) && v.edad_estado_s <= est.cfg.cfg.seguridad.watchdog_estado_s)) return;
  const p = v.tcp_med;
  if (agregarMuestra(est.figura.medidos, [p.x_mm / 1000, p.y_mm / 1000, p.z_mm / 1000])) {
    const ahora = performance.now();
    if (ahora - est.figura.dibujo > 150) { est.figura.dibujo = ahora; dibujarPanelFigura(); }
  }
}

// Poses guardadas: guardar la actual, listar, renombrar, borrar (con confirmación) e «Ir a pose».
function conectarPosesGuardadas() {
  $("#pg-form").onsubmit = async (e) => {
    e.preventDefault();
    const nombre = $("#pg-nombre").value.trim();
    if (!nombre) { mostrarAvisoPoses("Escribí un nombre para la pose."); $("#pg-nombre").focus(); return; }
    const d = await ordenPoses("pose_guardar", { nombre });
    if (d.ok) { $("#pg-nombre").value = ""; aviso(`Pose «${d.pose.nombre}» guardada.`, "ok"); }
  };
  $("#pg-lista").addEventListener("click", async (e) => {
    const b = e.target.closest("button[data-accion]");
    if (!b || b.disabled) return;
    const nombre = b.closest("li").dataset.nombre, accion = b.dataset.accion;
    if (accion === "ir") {
      if (est.vivo?.ejec) { aviso("Esperá a que termine el movimiento en curso.", "error"); return; }
      await calcularPlan("calcular_pose_guardada", { nombre });
    } else if (accion === "renombrar") {
      est.poses.editando = nombre; est.poses.borrando = null; dibujarPoses(true);
      $("#pg-lista .pg-renombrar input")?.focus();
    } else if (accion === "borrar") {
      est.poses.borrando = nombre; est.poses.editando = null; dibujarPoses(true);
    } else if (accion === "cancelar") {
      est.poses.borrando = est.poses.editando = null; dibujarPoses(true);
    } else if (accion === "si-borrar") {
      const d = await ordenPoses("pose_borrar", { nombre, confirmar: true });
      if (d.ok) aviso(`Pose «${nombre}» borrada.`, "info");
    } else if (accion === "aceptar") {
      await renombrarPose(b.closest("li"));
    }
  });
  $("#pg-lista").addEventListener("keydown", (e) => {
    if (e.key === "Enter" && e.target.matches(".pg-renombrar input")) { e.preventDefault(); renombrarPose(e.target.closest("li")); }
  });
}

async function renombrarPose(li) {
  const nuevo = li.querySelector("input").value.trim();
  if (!nuevo) { mostrarAvisoPoses("Escribí el nuevo nombre."); return; }
  const d = await ordenPoses("pose_renombrar", { nombre: li.dataset.nombre, nuevo });
  if (d.ok) aviso(`Pose renombrada a «${d.pose.nombre}».`, "ok");
}

async function ordenPoses(nombre, cuerpo) {
  if (est.poses.pendiente) return { ok: false };
  est.poses.pendiente = true;
  dibujarPoses(true);
  try {
    const d = await orden(nombre, cuerpo);
    if (d.ok) { est.poses.editando = est.poses.borrando = null; recibirPoses(d); }
    else mostrarAvisoPoses(d.mensaje || "No se pudo completar la operación.");
    return d;
  } finally {
    est.poses.pendiente = false;
    dibujarPoses(true);
  }
}

async function cargarPoses() {
  const d = await orden("poses_listar");
  if (d.ok) recibirPoses(d);
}

function recibirPoses(d) {
  est.poses.lista = d.poses || [];
  est.poses.disponible = d.disponible !== false;
  mostrarAvisoPoses(d.aviso || "");
  $("#pg-cuenta").textContent = est.poses.disponible
    ? (est.poses.lista.length ? `${est.poses.lista.length} guardada${est.poses.lista.length === 1 ? "" : "s"}` : "ninguna todavía")
    : "sin carpeta de datos";
  if (d.ruta) $("#pg-cuenta").title = d.ruta;
  dibujarPoses(true);
}

function mostrarAvisoPoses(texto) {
  const a = $("#pg-aviso");
  a.textContent = texto;
  a.classList.toggle("oculto", !texto);
}

function fechaCorta(iso) {
  const m = /^(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2})/.exec(iso || "");
  return m ? `${m[3]}/${m[2]} ${m[4]}:${m[5]}` : "";
}

// Se redibuja solo cuando cambia algo que afecta a los botones (no en cada muestra del estado).
function dibujarPoses(forzar = false) {
  const v = est.vivo, permiso = permisoIrAPose(v?.modo);
  const ocupado = est.poses.pendiente || !!v?.ejec || !!v?.jog || est.calculoPendiente || est.envioPendiente;
  const clave = [permiso.permitido, ocupado, est.poses.disponible].join("|");
  if (!forzar && clave === est.poses.clave) return;
  est.poses.clave = clave;
  $("#pg-guardar").disabled = ocupado || est.poses.disponible === false;
  $("#pg-guardar").title = est.poses.disponible === false ? "No hay carpeta de datos configurada (--datos)."
    : v?.ejec || v?.jog ? "Esperá a que el robot esté quieto." : "Guarda las 6 juntas recibidas y la pose del TCP por FK";
  const lista = $("#pg-lista");
  const boton = (texto, accion, clase = "", deshabilitado = false, titulo = "") => {
    const b = document.createElement("button");
    b.type = "button"; b.className = "btn-chico " + clase; b.dataset.accion = accion; b.textContent = texto;
    b.disabled = deshabilitado; if (titulo) b.title = titulo;
    return b;
  };
  lista.replaceChildren(...est.poses.lista.slice().reverse().map((p) => {
    const li = document.createElement("li");
    li.dataset.nombre = p.nombre;
    const acciones = document.createElement("div");
    acciones.className = "pg-acciones";
    if (est.poses.editando === p.nombre) {
      const caja = document.createElement("div");
      caja.className = "pg-renombrar";
      const input = document.createElement("input");
      input.type = "text"; input.maxLength = 40; input.value = p.nombre; input.setAttribute("aria-label", "Nuevo nombre de la pose");
      caja.append(input);
      acciones.append(boton("Aceptar", "aceptar", "pg-ir", est.poses.pendiente), boton("Cancelar", "cancelar"));
      li.append(caja, acciones);
      return li;
    }
    const info = document.createElement("div");
    info.className = "pg-info";
    const b = document.createElement("b");
    b.textContent = p.nombre;
    const small = document.createElement("small");
    const t = p.tcp || {};
    const tcp = Number.isFinite(t.x_mm) ? ` · TCP (${f(t.x_mm, 0)}, ${f(t.y_mm, 0)}, ${f(t.z_mm, 0)}) mm` : "";
    small.textContent = `${fechaCorta(p.fecha)} · q = [${p.q_deg.map((x) => f(x, 1)).join(", ")}]°${tcp}`;
    small.title = small.textContent;
    info.append(b, small);
    if (est.poses.borrando === p.nombre) {
      const txt = document.createElement("span");
      txt.className = "pg-confirmar"; txt.textContent = "¿Borrar esta pose?";
      acciones.append(txt, boton("Sí, borrar", "si-borrar", "pg-si", est.poses.pendiente), boton("Cancelar", "cancelar"));
    } else {
      acciones.append(
        boton("Ir", "ir", "pg-ir", ocupado || !permiso.permitido,
          permiso.permitido ? "Calcula un plan articular revisado hasta esta pose (después: revisión, armado y confirmación)" : permiso.motivo),
        boton("Renombrar", "renombrar", "", est.poses.pendiente),
        boton("Borrar", "borrar", "pg-borrar", est.poses.pendiente));
    }
    li.append(info, acciones);
    return li;
  }));
}

// Planes
function recibirPlan(plan, irAPasos, desdeArrastre = false) {
  est.planDescartado=null;
  if (!desdeArrastre && est.drag) arr.reiniciar();
  if (est.gz.acum) est.gz.acum.marcos = null;
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
  if (plan.tray) h += `${nombrePerfil(plan.tray.perfil)} · ${f(plan.tray.tf, 2)} s. Destino q = [${plan.q_obj_deg.map((x) => f(x, 1)).join(", ")}] °.<br>`;
  if (plan.notas && plan.notas.length) h += plan.notas.map((p) => `<span class="nota-plan">${p}</span>`).join("<br>") + "<br>";
  if (plan.problemas && plan.problemas.length) h += plan.problemas.map((p) => `<span class="problema">✗ ${p}</span>`).join("<br>");
  else h += '<span style="color:#13785c">✓ Revisión previa sin bloqueos (mirá el detalle abajo).</span>';
  r.innerHTML = h;
  r.className = "resumen-plan " + (plan.ejecutable ? "ok" : "mal");
  $("#flujo-resumen").innerHTML = h;
  $("#flujo-resumen").className = r.className;
  est.verFalla = null;
  dibujarRevision(plan);
  $("#pasos-contenido").innerHTML = pasosPlan(plan, est.cfg.cfg);
  $("#ramas-contenido").innerHTML = tablaRamas(plan);
  $("#lspb-contenido").innerHTML = panelLSPB(plan);
  if (est.pestana === "lspb") dibujarLSPB(plan);
  if (plan.figura) {
    est.figura.plan = figuraDePlan(plan);
    est.figura.medidos = [];
  } else {
    est.figura.plan = null;
  }
  dibujarPanelFigura();
  const camino = plan.preview?.camino || [];
  est.escena.fijarCamino(camino);
  est.escena.fijarPuntos(plan.preview?.puntos ?? (camino.length ? [camino[0], camino.at(-1)] : []));
  est.preview = { activo: !!plan.preview, t0: performance.now() };
  if (irAPasos && est.pestana === "objetivo") aviso("Cálculo listo: revisa «Cálculo paso a paso» y «Las 8 ramas».", "ok");
  actualizarTuberia();
  dibujarMini(plan);
  if(est.pestana==="graficas")configurarGrafica();
}

function dibujarMini(plan) {
  const m = $("#mini-tray");
  if (!plan?.tray) { m.classList.add("oculto"); return; }
  m.classList.remove("oculto");
  if (!est.gMiniQ) {
    est.gMiniQ = new GraficaTrayectoria($("#mini-q"), "Posición q(t) [°]", "°");
    est.gMiniQD = new GraficaTrayectoria($("#mini-qd"), "Velocidad q̇(t) [°/s]", "°/s");
    $("#mini-ver-lspb").onclick = () => abrirPestana("lspb");
    $("#mini-ver-graf").onclick = () => abrirPestana("graficas");
  }
  est.gMiniQ.fijar(plan.tray.t, plan.tray.q_deg);
  est.gMiniQD.fijar(plan.tray.t, plan.tray.qd_deg_s);
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
      dibujarFlechasJunta();
      const a = est.gz.acum;
      const gd = arr?.fantasma() || (a?.marcos && { marcos: a.marcos, color: a.admisible ? 0x198ba7 : 0xd6403a });
      esc.fantasma.fijarColor((est.verFalla && est.verFalla.marcos) || (gd && gd.color !== 0x198ba7) ? 0xd6403a : 0x198ba7);
      if (est.verFalla && est.verFalla.marcos) {
        esc.fantasma.visible = true;
        esc.fantasma.actualizar(est.verFalla.marcos);
      } else if (gd) {
        esc.fantasma.visible = true;
        esc.fantasma.actualizar(gd.marcos);
      } else if (est.ramaVista != null && p && p.ik && p.ik.ramas[est.ramaVista] && p.ik.ramas[est.ramaVista].marcos) {
        esc.fantasma.visible = true;
        esc.fantasma.actualizar(p.ik.ramas[est.ramaVista].marcos);
      } else if (v && v.ejec) {
        esc.fantasma.visible = verF && !!v.marcos_ref;
        if (v.marcos_ref) esc.fantasma.actualizar(v.marcos_ref);
      } else if (est.preview.activo && p && p.preview && p.preview.frames.length && (!v || !v.ejec)) {
        const tf = p.tray.tf, ciclo = tf + 1.2;
        const t = Math.min(tf, ((performance.now() - est.preview.t0) / 1000) % ciclo);
        const ts = p.preview.t;
        const siguiente = ts.findIndex((x) => x >= t);
        const k = siguiente < 0 ? ts.length - 1 : siguiente;
        const anterior = Math.max(0, k - 1);
        const duracion = ts[k] - ts[anterior];
        const fraccion = duracion > 0 ? Math.min(1, (t - ts[anterior]) / duracion) : 1;
        esc.fantasma.visible = verF;
        esc.fantasma.actualizar(interpolarMarcos(p.preview.frames[anterior], p.preview.frames[k], fraccion));
      } else if (v && v.marcos_ref) {
        esc.fantasma.visible = verF && (!v.err_deg || Math.max(...v.err_deg.map(Math.abs))>.05);
        esc.fantasma.actualizar(v.marcos_ref);
      } else {
        esc.fantasma.visible = false;
      }
    }
    requestAnimationFrame(paso);
  };
  paso();
}

// Preparar J5 y poses predeterminadas: SIM, o robot real por RWS con C02 verificado.
function botonesGuiados(modo) {
  const p = permisoMovimientosGuiados(modo);
  const clave = [p.permitido, p.sim, p.maxMm, p.tituloFiguras].join("|");
  if (clave === est.ultimoGuiados) return;
  est.ultimoGuiados = clave;
  const figuras = $("#btn-poses-predeterminadas"), j5 = $("#btn-home-j5");
  figuras.disabled = j5.disabled = !p.permitido;
  figuras.title = p.tituloFiguras;
  j5.title = p.tituloJ5;
  j5.querySelector("small").textContent = p.subtituloJ5;
  const modal = $("#poses-predeterminadas");
  if (!p.permitido && modal.open) modal.close();
}

// Estado en vivo
function actualizar(v) {
  const cambioFuente = est.vivo?.modo?.tipo !== v.modo?.tipo || est.vivo?.modo?.real !== v.modo?.real;
  est.vivo = v;
  if (cambioFuente) {
    ayudaJog();
    actualizarFrecuencia(est.cfg.cfg.servidor.hz_estado, frecuenciaFuente());
    $("#vel").title = v.modo?.tipo === "rws" ? "Velocidad nominal del plan. La velocidad física depende del módulo RAPID." : "Velocidad nominal de cálculo del plan.";
    $$(".flecha").forEach((b) => { b.disabled = v.modo?.tipo === "rws"; });
    est.escena.limpiarTraza?.();
    $("#traza-fuente").textContent = v.modo?.tipo === "sim" ? "Traza SIM" : "Traza por FK";
  }
  botonesGuiados(v.modo);
  dibujarPoses();
  seguirFigura(v);
  est.ultimaRecepcion=performance.now();
  $("#estado-fuente").textContent=v.modo?.tipo==="sim"?"Simulador interno":v.modo?.real===false?"ROS · hardware simulado":"Estado leído del robot";
  $("#nota-fuente").textContent=notaFuente(v.modo||{});
  $("#edad-estado").textContent=`${f(1000*v.edad_estado_s,0)} ms · edad de muestra`;
  if(v.q_med_deg){est.historial.push({t:v.t,q:v.q_med_deg,ref:v.q_ref_deg,e:v.err_deg,ep:v.e_p_mm,er:v.e_R_deg,sigma:v.sing?.sigma_min});while(est.historial.length&&est.historial[0].t<v.t-120)est.historial.shift();}

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

  if (v.q_med_deg && v.marcos_med && v.tcp_med) {
    est.escena.recibirMedido(v.marcos_med, cambioFuente);
    const p = v.tcp_med;
    if (Number.isFinite(v.edad_estado_s) && v.edad_estado_s <= est.cfg.cfg.seguridad.watchdog_estado_s) {
      est.escena.agregarTraza([p.x_mm / 1000, p.y_mm / 1000, p.z_mm / 1000]);
    }
    $("#lectura-tcp").innerHTML = `TCP por FK  x ${f(p.x_mm, 1)}  y ${f(p.y_mm, 1)}  z ${f(p.z_mm, 1)} mm<br>Rz ${f(p.rz_deg, 1)}°  Ry ${f(p.ry_deg, 1)}°  Rx ${f(p.rx_deg, 1)}°`;
    const s = v.sing;
    const av = $("#aviso-singular");
    const cerca = [];
    if (s.muneca_sin_q5 < 0.09) cerca.push("muñeca (q5 ≈ 0)");
    if (s.codo_sin_delta < 0.09) cerca.push("codo (brazo estirado)");
    if (s.hombro_r_w_m < 0.03) cerca.push("hombro (muñeca sobre el eje 1)");
    av.classList.toggle("oculto", !cerca.length);
    av.textContent = "Cerca de singularidad: " + cerca.join(", ") + `. σmin = ${f(s.sigma_min, 3)}`;
    actualizarJuntas(v);
    badgeSing(v);
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
    est.gTCP.agregar(v.t, [v.e_p_mm]);
    est.gRot.agregar(v.t,[v.e_R_deg]);
  } else {
    sem.className = "semaforo";
    sem.querySelector("span").textContent = "todavía no se comandó nada";
  }
  if (v.sing) $("#n-sig").textContent = f(v.sing.sigma_min, 3);

  const barra = $("#barra-ejec");
  if (v.ejec) {
    barra.classList.remove("oculto");
    barra.querySelector("div").style.width = v.ejec.progreso == null ? "0" : (100 * v.ejec.progreso).toFixed(1) + "%";
    barra.querySelector("span").textContent = v.ejec.progreso == null
      ? `Plan ${v.ejec.plan_id} · ${m.progreso || "Trayectoria en curso"} · ${f(v.ejec.t, 1)} s transcurridos`
      : `Plan ${v.ejec.plan_id} · ${f(v.ejec.t, 1)} / ${f(v.ejec.tf, 1)} s · error máx. ${f(v.ejec.err_max_deg, 3)}° · e_p máx. ${f(v.ejec.e_p_max_mm, 2)} mm`;
    if (est.plan && v.ejec.plan_id === est.plan.id) {
      est.planEjecutado = est.plan.id;
      est.gQ.marcar(v.ejec.t); est.gQD.marcar(v.ejec.t); est.gQDD.marcar(v.ejec.t);
      est.gMiniQ?.marcar(v.ejec.t); est.gMiniQD?.marcar(v.ejec.t);
    }
  } else {
    barra.classList.add("oculto");
  }
  actualizarBotonEnvio();
  if (v.ik_vivo && est.plan == null) { /* jog cartesiano sin plan: nada extra */ }
  actualizarTuberia();
  cajaArmado(v);
}

function badgeSing(v) {
  const s = v.sing, el = $("#badge-sing");
  if (!s) return;
  let nivel = s.nivel;
  if (!nivel) {
    const umb = est.cfg?.cfg?.ik?.sigma_min ?? 0.02;
    nivel = s.sigma_min < umb ? "singular" : s.sigma_min < 2 * umb || s.muneca_sin_q5 < 0.09 || s.codo_sin_delta < 0.09 ? "cerca" : "ok";
  }
  const txt = { ok: "Singularidad: lejos", cerca: "Singularidad: CERCA", singular: "SINGULAR" }[nivel] || nivel;
  const k = nivel + "|" + f(s.sigma_min, 3);
  if (el.dataset.k !== k) { el.dataset.k = k; el.className = "badge-sing " + nivel; el.innerHTML = `<i></i>${txt} <small>σmin ${f(s.sigma_min, 3)}</small>`; }
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
    marca(2, "no-aplica"); marca(3, "no-aplica"); marca(4, "no-aplica");
  }
  if (p.tray) marca(5, "hecho");
  const rev = p.revision;
  if (rev) marca(6, rev.bloquea_envio ? "fallo" : rev.estado === "aprobado" ? "hecho" : "aviso");
  if (v && v.ejec && v.ejec.plan_id === p.id) { marca(7, "hecho"); marca(8, "activo"); }
  else if (v?.ultimo_resumen?.plan_id === p.id) { marca(7, "hecho"); marca(8, v.ultimo_resumen.estado === "completado" ? "hecho" : "fallo"); li[7].title = v.ultimo_resumen.llegada_verificada ? "Llegada verificada en SIM" : v.ultimo_resumen.estado; }
  else if (p.ejecutable) marca(7, "activo");
}

const avisoC02 = "Movimiento real bloqueado (C02): límite de velocidad por junta sin verificar.";

function bloqueoC02(modo) {
  return modo?.tipo === "rws" && modo.velocidad_articular_verificada === false;
}

function cajaArmado(v) {
  const m = v.modo || {};
  const clave = [v.armado, m.exige_go, m.permitir_real, m.solo_lectura, bloqueoC02(m), v.motivo_desarmado].join("|");
  if (clave === est.ultimoCaja) return;
  est.ultimoCaja = clave;
  const c = $("#caja-armado");
  const armadoReal = $("#flujo-armado-real");
  armadoReal.replaceChildren();
  if (bloqueoC02(m)) {
    c.textContent = m.c02_motivo ? `${avisoC02} ${m.c02_motivo}` : avisoC02;
    armadoReal.textContent = c.textContent;
  } else if (m.solo_lectura) {
    c.textContent = "Modo de solo lectura. Se pueden consultar estados y calcular planes.";
  } else if (v.armado) {
    c.innerHTML = esVirtual(m) ? `<b style="color:#ff9b3d">SIMULADOR HABILITADO.</b> Ejecutar y las flechas mueven únicamente el robot virtual.<br><button id="btn-desarmar" class="btn-desarmar" style="margin-top:6px">Desarmar</button>` : `<b style="color:#ff5a5f">ROBOT REAL ARMADO.</b> Enviar mueve el brazo físico (velocidad máx. ${m.vel_max_pct} %). Mano en el paro de emergencia.<br><button id="btn-desarmar" class="btn-desarmar" style="margin-top:6px">Desarmar</button>`;
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
    armadoReal.innerHTML = c.innerHTML.replace('id="btn-armar"', 'id="btn-armar-dialogo"');
  } else {
    c.innerHTML = `Habilitá el simulador para ejecutar el plan o usar las flechas.<br><small>${v.motivo_desarmado || ""}</small><br><button id="btn-armar" class="btn-armar" style="margin-top:6px">Habilitar simulador</button>`;
  }
}

function redibujarLento(forzar = false) {
  const v = est.vivo;
  if (!v) return;
  if(est.pestana==="datos")dibujarDatos(v);
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
  if (secuenciaActiva() && !est.calculoPendiente && v.plan_id && v.plan_id !== est.planDescartado && v.plan_id !== est.plan?.id) {
    const revision = est.revisionObjetivo;
    fetch("/api/plan").then((r) => r.json()).then((p) => {
      if (p?.id === v.plan_id && secuenciaActiva() && est.vivo.plan_id === p.id && revision === est.revisionObjetivo) recibirPlan(p, false);
    }).catch(() => {});
  }
}

function tipoObjetivo(n){
 if((est.tipoObjetivo||"junta")!==n)invalidarPlan();est.tipoObjetivo=n;
 $$("[data-objetivo]").forEach(b=>b.classList.toggle("activo",b.dataset.objetivo===n));
 $$(".objetivo-form").forEach(p=>p.classList.toggle("oculto",p.id!=="objetivo-"+n));
}
function invalidarPlan(){
 est.revisionObjetivo++;
 est.planDescartado=est.plan?.id||est.vivo?.plan_id;est.plan=null;est.verFalla=null;$("#panel-revision").classList.add("oculto");$("#mini-tray").classList.add("oculto");est.preview.activo=false;est.ramaVista=null;
 $("#flujo-plan").close();
 est.confirmarPlanId = null;
 $("#ramas-contenido").textContent = "Calculá un plan o abrí las ramas de la cinemática inversa en vivo.";
 $("#btn-ejecutar").disabled=true;$("#resumen-plan").className="resumen-plan";$("#resumen-plan").textContent="Objetivo modificado. Volvé a calcular antes de ejecutar.";
 est.escena?.fijarCamino([]);est.escena?.fijarPuntos?.([]);actualizarTuberia();actualizarBotonEnvio();
}
function sincronizarSlider() {
  const relativo = $("#j-relativo").checked;
  const [min, max] = relativo ? [-15, 15] : est.cfg.limites_deg[est.jogJunta - 1];
  const slider = $("#j-preview"), v = +$("#j-valor").value;
  slider.min = min;
  slider.max = max;
  slider.step = relativo ? "0.5" : "0.1";
  slider.value = v;
  const grados = (a) => `${a > 0 ? "+" : ""}${a}°`;
  $("#j-preview-valor").textContent = grados(v);
  $("#j-valor-etiqueta").textContent = relativo ? "Desplazamiento" : "Ángulo objetivo";
  [min, (min + max) / 2, max].forEach((a, i) => { $("#j-preview-rango").children[i].textContent = grados(a); });
}
function configurarGrafica(){
 est.graficaPausada=false;$("#btn-pausar-grafica").textContent="Pausar gráfica";
 if(est.gPrincipal){est.gPrincipal.observador.disconnect();est.gPrincipal.u.destroy();}
 const tipo=$("#tipo-grafica").value, sel=$("#junta-grafica").value;
 const js=sel==="todas"?[0,1,2,3,4,5]:[+sel];
 const spec={posicion:["°","q"],error:["°","e"],velocidad:["°/s","qd_deg_s"],aceleracion:["°/s²","qdd_deg_s2"],tcp:["mm","ep"],orientacion:["°","er"],singularidad:["σ (J mixto)","sigma"]}[tipo];
 let series;
 if(["tcp","orientacion","singularidad"].includes(tipo))series=[{label:tipo,stroke:"#2187a4",width:2}];
 else series=js.flatMap(j=>tipo==="posicion"?[{label:`J${j+1} recibida`,stroke:COLORES_JUNTAS[j],width:2},{label:`J${j+1} ref.`,stroke:COLORES_JUNTAS[j],dash:[5,4],width:1.3}]:[{label:`J${j+1}`,stroke:COLORES_JUNTAS[j],width:2}]);
 const cont=$("#grafica-principal");
 const u=new uPlot({width:Math.max(260,cont.clientWidth),height:280,legend:{show:true,live:false},scales:{x:{time:false}},axes:[{stroke:"#748a9b",grid:{stroke:"#edf1f5"},label:"Tiempo [s]",labelSize:20,font:"10px sans-serif"},{stroke:"#748a9b",grid:{stroke:"#edf1f5"},label:spec[0],labelSize:18,size:55,font:"10px sans-serif"}],series:[{label:"t [s]"},...series.map(s=>({...s,points:{show:false}}))]},[[],...series.map(()=>[])],cont);
 const observador=new ResizeObserver(()=>{if(cont.clientWidth)u.setSize({width:cont.clientWidth,height:280});});observador.observe(cont);
 est.gPrincipal={u,observador,tipo,js,spec};actualizarGrafica();
}
function actualizarGrafica(){
 const g=est.gPrincipal;if(!g||est.graficaPausada)return;const {tipo,js,spec}=g;const planned=["velocidad","aceleracion"].includes(tipo);let data;
 if(planned){const tr=est.plan?.tray;data=tr?[tr.t,...js.map(j=>tr[spec[1]].map(q=>q[j]))]:[[],...js.map(()=>[])];}
 else{const all=est.historial||[],ventana=+$("#ventana-grafica").value;const h=all.filter(x=>x.t>=(all.at(-1)?.t||0)-ventana);data=[h.map(x=>x.t-(h[0]?.t||0))];if(["tcp","orientacion","singularidad"].includes(tipo))data.push(h.map(x=>x[spec[1]]??null));else for(const j of js){data.push(h.map(x=>x[spec[1]]?.[j]??null));if(tipo==="posicion")data.push(h.map(x=>x.ref?.[j]??null));}}
 g.u.setData(data);
 const vals=data.slice(1).flat().filter(v=>v!=null&&Number.isFinite(v));
 $("#grafica-max").textContent=vals.length?`${f(Math.max(...vals.map(Math.abs)),3)} ${spec[0]}`:"Sin datos";
 $("#grafica-fuente").textContent=planned?"Plan calculado":"Simulación interna";
 $("#grafica-edad").textContent=est.vivo?`${f(est.vivo.edad_estado_s*1000,0)} ms · edad de muestra`:"—";
 $("#grafica-nota").textContent=planned?(est.plan?"Curva planificada del último plan; no es una medición de velocidad ni aceleración.":"Calculá un plan en Control para ver estas curvas."):tipo==="posicion"?"Línea continua: recibida; punteada: referencia. Pausá la gráfica para ampliar un tramo; doble clic restablece la vista.":tipo==="singularidad"?"σ mínimo del Jacobiano calculado. Mezcla unidades lineales y angulares; no tiene un umbral universal.":"Errores derivados de referencia y estado simulado. Sin referencia, no se inventa un error cero.";
}
function dibujarDatos(v){
 const p=v.tcp_med,s=v.sing;
 const grupos=[ ["Estado y comunicaciones",[["Fuente","Simulador interno","Sin conexión física"],["Edad de muestra",`${f(v.edad_estado_s*1000,1)} ms`,"Edad reportada por backend"],["Plan",v.plan_id??"—","Calculado"],["Ejecución",v.ejec?`${f(v.ejec.progreso*100,1)} %`:"En espera","Simulador"]]],
 ["Cinemática",[["Ángulos q",v.q_med_deg?.map(x=>f(x,2)).join(" · ")+" °","Recibidos del simulador"],["TCP XYZ",p?`${f(p.x_mm,2)} · ${f(p.y_mm,2)} · ${f(p.z_mm,2)} mm`:"—","FK propia"],["Euler ZYX",p?`${f(p.rz_deg,2)} · ${f(p.ry_deg,2)} · ${f(p.rx_deg,2)} °`:"—","Calculado · puede tener gimbal lock"],["Cuaternión wxyz",p?.cuaternion?.map(x=>f(x,5)).join(" · ")||"—","Calculado"],["σ mínimo",s?f(s.sigma_min,6):"—","Jacobiano mixto"],["Número de condición",s?f(s.condicion,2):"—","Calculado"]]],
 ["Seguimiento",[["Error q (medida − referencia)",v.err_deg?.map(x=>f(x,3)).join(" · ")||"Sin referencia","Grados"],["Error TCP posición",v.e_p_mm==null?"Sin referencia":f(v.e_p_mm,3)+" mm","Comparación de dos FK"],["Error TCP orientación",v.e_R_deg==null?"Sin referencia":f(v.e_R_deg,4)+" °","Distancia angular"]]],
 ["Telemetría pendiente de una fuente",[["Velocidad / aceleración medidas","No disponible","Las curvas del plan no son mediciones"],["Torque / corriente / temperatura","No disponible","Sin sensores o telemetría"],["Pose medida externamente","No disponible","Requiere medición independiente"],["Estado IRC5 / seguridad física","No conectado","No se deduce de la simulación"]]] ];
 $("#datos-contenido").innerHTML=grupos.map(([titulo,filas])=>`<section class="dato-grupo"><h4>${titulo}</h4><div class="datos-tabla"><table class="tabla-simple"><tbody>${filas.map(([a,b,c])=>`<tr><td>${a}</td><td>${b}</td><td>${c}</td></tr>`).join("")}</tbody></table></div></section>`).join("");
}
function exportarCSV(){
 const all=est.historial||[],ventana=+$("#ventana-grafica").value;const h=all.filter(x=>x.t>=(all.at(-1)?.t||0)-ventana);if(!h.length)return;
 const cab=["fuente","t_s",...Array.from({length:6},(_,j)=>`q${j+1}_deg`),...Array.from({length:6},(_,j)=>`ref${j+1}_deg`),...Array.from({length:6},(_,j)=>`error${j+1}_deg`),"ep_mm","eR_deg","sigma_min"];
 const lines=[cab.join(","),...h.map(x=>["simulacion",x.t,...x.q,...(x.ref||Array(6).fill("")),...(x.e||Array(6).fill("")),x.ep??"",x.er??"",x.sigma??""].join(","))];
 const url=URL.createObjectURL(new Blob([lines.join("\n")],{type:"text/csv;charset=utf-8"}));const a=document.createElement("a");a.href=url;a.download="vevas_simulacion_ventana.csv";a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}
setInterval(()=>{if(est.ultimaRecepcion&&performance.now()-est.ultimaRecepcion>2000){$("#estado-fuente").textContent="Sin actualizaciones";$("#edad-estado").textContent=`${f((performance.now()-est.ultimaRecepcion)/1000,1)} s sin recibir`;actualizarBotonEnvio();}},500);
iniciar().catch(e=>{console.error(e);aviso("No se pudo iniciar la interfaz: "+e.message,"error");});
