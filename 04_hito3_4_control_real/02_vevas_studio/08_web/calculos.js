// Presentación de los cálculos con sus números (KaTeX). Todo viene calculado del servidor.

const RAD = 180 / Math.PI;
export const f = (x, d = 4) => {
  if (x == null || Number.isNaN(x)) return "—";
  const t = Number(x).toFixed(d);
  return /^-0\.?0*$/.test(t) ? t.slice(1) : t; // sin "-0.0000"
};
const fe = (x) => (x == null ? "—" : Number(x).toExponential(1).replace("e", "\\times10^{") + "}");
const feTxt = (x) => (x == null ? "—" : Number(x).toExponential(1));
const filas4 = (a) => [a.slice(0, 4), a.slice(4, 8), a.slice(8, 12), a.slice(12, 16)];

export function tex(s, display = true) {
  try {
    return katex.renderToString(s, { displayMode: display, throwOnError: false, strict: false });
  } catch (e) {
    return `<code>${s}</code>`;
  }
}
export const mat = (M, d = 4) => "\\begin{bmatrix}" + M.map((r) => r.map((v) => f(v, d)).join(" & ")).join("\\\\") + "\\end{bmatrix}";
const vec = (v, d = 4) => "\\begin{bmatrix}" + v.map((x) => f(x, d)).join("\\\\") + "\\end{bmatrix}";

function paso(n, titulo, nota, cuerpo) {
  return `<div class="paso"><h4><span>${n}</span>${titulo}</h4>${nota ? `<p class="nota">${nota}</p>` : ""}<div class="ecu">${cuerpo}</div></div>`;
}

export function pasosPlan(plan, cfg) {
  if (!plan) return '<p class="vacio">Sin plan.</p>';
  if (plan.tipo === "pose") return pasosPose(plan, cfg);
  if (plan.figura) {
    const fig = plan.figura;
    const plano = fig.plano ? ` · plano ${fig.plano}` : "";
    const metodo = "Se busca una rama Pieper para cada punto y se mantiene la orientación del TCP.";
    const nParadas = Array.isArray(fig.paradas) ? fig.paradas.length : 0;
    const paradas = nParadas ? ` Se detiene (v = 0) en ${nParadas} ${nParadas === 1 ? "punto intermedio" : "puntos intermedios"}.` : "";
    return paso(1, "Recorrido predeterminado", plan.descripcion,
      `<p>${fig.n_puntos} puntos de control · tamaño ${f(fig.tamano_mm, 1)} mm${plano}. ${metodo}${paradas}</p>`) + pasoTrayectoria(2, plan);
  }
  return pasosArticular(plan);
}

// Preparar J5 y poses predeterminadas: SIM, o robot real por RWS con C02 verificado. El servidor manda
// (modo.figuras_permitidas); si no lo informa, solo SIM. Con ROS real o RWS sin C02 quedan deshabilitadas.
export function permisoMovimientosGuiados(modo) {
  const m = modo || {};
  const permitido = typeof m.figuras_permitidas === "boolean" ? m.figuras_permitidas : m.tipo === "sim";
  const sim = m.tipo === "sim";
  const maxMm = Number.isFinite(m.figura_max_mm) && m.figura_max_mm > 0 ? m.figura_max_mm : (sim ? 100 : 80);
  let motivo = "";
  if (!permitido) {
    if (m.tipo === "ros") motivo = "Con ROS real no está habilitado: solo en SIM o con el robot por RWS y C02 verificado.";
    else if (m.tipo === "rws") motivo = "Bloqueado: C02 (velocidad por junta en el RAPID) sin verificar. Verificá C02 para habilitarlo.";
    else motivo = m.figuras_motivo || "Disponible en SIM o con el robot real por RWS y C02 verificado.";
  }
  return {
    permitido,
    sim,
    maxMm,
    subtituloJ5: sim ? "revisar en SIM" : permitido ? "robot real · RWS C02" : "solo SIM o RWS con C02",
    tituloJ5: permitido
      ? (sim ? "Calcula un plan articular que deja J5 en +30° (simulación)."
             : "Plan articular J5 → +30° para el robot real (RWS, C02). Pasa por revisión, GO, armado y confirmación.")
      : motivo,
    tituloFiguras: permitido
      ? (sim ? "Calcular una figura en simulación"
             : `Calcular una figura para el robot real (RWS, C02; máx. ${maxMm} mm). Pasa por revisión, GO, armado y confirmación.`)
      : motivo,
    textoDialogo: sim
      ? "Trayectorias de simulación a partir de la pose actual, con perfil suave por tramos."
      : `Trayectorias para el ROBOT REAL a partir de la pose actual (máx. ${maxMm} mm). Se revisan antes de enviar y exigen GO, armado y confirmación. Si J5 ≈ 0, el plan lleva primero J5 a +30° y después dibuja.`,
  };
}

export function nombrePerfil(perfil) {
  if (perfil === "camino_quintico") return "Suave C² por tramos";
  if (perfil === "quintico") return "Suave quíntico";
  return "LSPB académico";
}

function pasosPose(plan, cfg) {
  const ik = plan.ik, P = ik.pasos, pp = plan.pose_pedida;
  const el = ik.elegida != null ? ik.ramas[ik.elegida] : null;
  let h = "";
  h += paso(1, "Pose pedida → matriz homogénea T<sub>d</sub>",
    "La pose se escribe como posición (en metros) y rotación R = Rz·Ry·Rx (EulerZYX, igual que el FlexPendant).",
    tex(`T_d=\\begin{bmatrix}R_z(${f(pp.rz, 1)}^\\circ)R_y(${f(pp.ry, 1)}^\\circ)R_x(${f(pp.rx, 1)}^\\circ) & p\\\\ 0\\;0\\;0 & 1\\end{bmatrix}=${mat(filas4(plan.T_d))}`));
  h += paso(2, "Quitar la herramienta: pose del marco 6 de la tabla D-H",
    "Pieper trabaja con el marco 6 de la tabla D-H. tool0 de ABB es ese marco girado 180° en z<sub>6</sub>; si hay herramienta, también se descuenta.",
    tex(`T_0^6=T_d\\,(T_{tool0}^{TCP})^{-1}\\,(T_6^{tool0})^{-1},\\quad T_6^{tool0}=R_z(180^\\circ)`) +
    tex(`T_0^6=${mat(filas4(P.T06.flat()))}`));
  const p6 = P.p6, z6 = P.z6, pw = P.p_w;
  h += paso(3, "Centro de muñeca (desacople de Pieper)",
    "Los ejes 4, 5 y 6 se cruzan en un punto. Retrocediendo d<sub>6</sub> = 0.072 m por el eje z<sub>6</sub> se llega a ese punto, que solo depende de q<sub>1</sub>, q<sub>2</sub>, q<sub>3</sub>.",
    tex(`p_w=p_6-d_6\\,z_6=${vec(p6)}-0.072\\,${vec(z6)}=${vec(pw)}\\;\\text{m}`));
  const q1a = P.q1_opciones[0] * RAD, q1b = P.q1_opciones[1] * RAD;
  h += paso(4, "q<sub>1</sub>: girar la base hacia el centro de muñeca",
    "Dos soluciones: mirando al punto (hombro adelante) o de espaldas (hombro atrás, 180° más).",
    tex(`q_1=\\operatorname{atan2}(y_w,x_w)=\\operatorname{atan2}(${f(pw[1])},\\,${f(pw[0])})=${f(q1a, 3)}^\\circ\\quad\\text{o}\\quad ${f(q1b, 3)}^\\circ`));
  if (el) {
    const br = P.brazo.find((b) => b.hombro === el.hombro);
    const co = br.codos.find((c) => c.codo === el.codo);
    const L3 = Math.hypot(0.07, 0.302), gam = Math.atan2(0.07, 0.302) * RAD;
    h += paso(5, `q<sub>2</sub> y q<sub>3</sub>: triángulo hombro–codo–muñeca (rama: hombro ${el.hombro}, codo ${el.codo})`,
      "En el plano del brazo, r es la distancia horizontal y s la altura sobre el hombro. Con la ley de cosenos sale el ángulo δ entre el brazo (a<sub>2</sub>) y el antebrazo efectivo (L<sub>3</sub>); el signo de δ es codo arriba o codo abajo.",
      tex(`r=${el.hombro === "adelante" ? "+" : "-"}\\sqrt{x_w^2+y_w^2}=${f(br.r)}\\,\\text{m},\\qquad s=z_w-d_1=${f(br.s)}\\,\\text{m}`) +
      tex(`L_3=\\sqrt{a_3^2+d_4^2}=${f(L3, 5)}\\,\\text{m},\\qquad \\gamma=\\operatorname{atan2}(a_3,d_4)=${f(gam, 3)}^\\circ`) +
      tex(`\\cos\\delta=\\frac{r^2+s^2-a_2^2-L_3^2}{2a_2L_3}=\\frac{${f(br.r * br.r + br.s * br.s - 0.0729 - L3 * L3, 5)}}{${f(2 * 0.27 * L3, 5)}}=${f(br.cos_delta, 5)}\\;\\Rightarrow\\;\\delta=${f(co.delta * RAD, 3)}^\\circ`) +
      tex(`q_3=\\delta+\\gamma-90^\\circ=${f(co.q3 * RAD, 3)}^\\circ`) +
      tex(`q_2=\\operatorname{atan2}(r,s)-\\operatorname{atan2}(L_3\\sin\\delta,\\;a_2+L_3\\cos\\delta)=${f(co.q2 * RAD, 3)}^\\circ`));
    const R36 = P.R36_elegida;
    const s5 = el.q_rad[4];
    h += paso(6, `Muñeca: q<sub>4</sub>, q<sub>5</sub>, q<sub>6</sub> (muñeca ${el.muneca})`,
      "Con q<sub>1</sub>, q<sub>2</sub>, q<sub>3</sub> ya conocidos se calcula R<sub>0</sub><sup>3</sup>. Lo que falta girar es R<sub>3</sub><sup>6</sup>; de sus elementos salen los tres ángulos de la muñeca. El signo de sen q<sub>5</sub> da las dos soluciones de muñeca.",
      tex(`R_3^6=(R_0^3)^T R_0^6=${mat(R36)}`) +
      tex(`q_5=\\operatorname{atan2}\\left(\\pm\\sqrt{r_{13}^2+r_{23}^2},\\,r_{33}\\right)=${f(el.q_deg[4], 3)}^\\circ`) +
      tex(`q_4=\\operatorname{atan2}\\left(\\tfrac{-r_{23}}{s_5},\\tfrac{-r_{13}}{s_5}\\right)=${f(el.q_deg[3], 3)}^\\circ,\\qquad q_6=\\operatorname{atan2}\\left(\\tfrac{-r_{32}}{s_5},\\tfrac{r_{31}}{s_5}\\right)=${f(el.q_deg[5], 3)}^\\circ`) +
      (Math.abs(Math.sin(s5)) < 0.05 ? '<p class="nota">Atención: sen q<sub>5</sub> ≈ 0, muñeca casi singular (q<sub>4</sub> y q<sub>6</sub> se confunden).</p>' : ""));
    h += paso(7, "Verificación con cinemática directa (FK)",
      "La solución se vuelve a meter en la FK y se compara con la pose pedida. Si el error no es prácticamente cero, la rama se descarta.",
      tex(`q^*=[${el.q_deg.map((v) => f(v, 2)).join(",\\;")}]^\\circ`) +
      tex(`e_p=\\lVert p(q^*)-p_d\\rVert=${fe(plan.verificacion.e_p_m)}\\,\\text{m},\\qquad e_R=\\angle\\!\\left(R_d^T R(q^*)\\right)=${fe(plan.verificacion.e_R_rad)}\\,\\text{rad}`));
  }
  h += paso(8, "Selección de la mejor rama", ik.razon + " Ver la pestaña «Las 8 ramas».",
    tex(`\\text{válida}\\iff e_p\\le ${fe(cfg.ik.tol_e_p_m)}\\,\\text{m},\\;e_R\\le ${fe(cfg.ik.tol_e_R_rad)}\\,\\text{rad},\\;\\text{límites con }${cfg.ik.margen_limite_deg}^\\circ\\text{ de margen},\\;\\sigma_{\\min}\\ge ${cfg.ik.sigma_min}`) +
    tex(`\\text{puntaje}=\\sum_i w_i\\,|q_i-q_{i,\\text{actual}}|,\\quad w=[1,1,1,0.6,0.6,0.4]`));
  if (plan.tray) h += pasoTrayectoria(9, plan);
  return h;
}

function pasosArticular(plan) {
  let h = "";
  const A = plan.fk_obj.A.map(filas4);
  h += paso(1, "Objetivo en juntas", plan.descripcion,
    tex(`q_{obj}=[${plan.q_obj_deg.map((v) => f(v, 2)).join(",\\;")}]^\\circ`));
  h += paso(2, "Cinemática directa: matrices de cada eslabón",
    "A<sub>i</sub> = Rz(θ<sub>i</sub>)·Tz(d<sub>i</sub>)·Tx(a<sub>i</sub>)·Rx(α<sub>i</sub>) con θ<sub>2</sub> = q<sub>2</sub> − 90°.",
    '<div class="dos-col">' + A.map((M, i) => tex(`A_${i + 1}=${mat(M, 3)}`)).join("") + "</div>");
  h += paso(3, "Producto de la cadena", "La pose de la brida es el producto ordenado de las seis matrices.",
    tex(`T_0^6=A_1A_2A_3A_4A_5A_6=${mat(filas4(plan.fk_obj.T06), 4)}`));
  const p = plan.pose_obj;
  h += paso(4, "Pose del TCP que resulta", "",
    tex(`p=(${f(p.x_mm, 1)},\\;${f(p.y_mm, 1)},\\;${f(p.z_mm, 1)})\\,\\text{mm},\\quad (R_z,R_y,R_x)=(${f(p.rz_deg, 2)}^\\circ,\\;${f(p.ry_deg, 2)}^\\circ,\\;${f(p.rx_deg, 2)}^\\circ)`));
  h += pasoTrayectoria(5, plan);
  return h;
}

function pasoTrayectoria(n, plan) {
  const t = plan.tray;
  const j = t.junta_limitante;
  const multiple = t.perfil === "camino_quintico";
  const suave = multiple || t.perfil === "quintico";
  let filas = "";
  for (let i = 0; i < 6; i++) {
    const cambio = multiple ? "" : `<td>${f(t.D_deg[i], 2)}</td>`;
    filas += `<tr${i + 1 === j ? ' style="color:#ff9b3d;font-weight:700"' : ""}><td>J${i + 1}</td>${cambio}<td>${f(t.V_deg_s[i], 2)}</td><td>${f(t.A_deg_s2[i], 2)}</td></tr>`;
  }
  const d = multiple ? "" : "<th>D [°]</th>";
  const v = suave ? "Pico |q̇|" : "V";
  const a = suave ? "Pico |q̈|" : "A";
  const tabla = `<table class="tabla-simple"><tr><th>Junta</th>${d}<th>${v} [°/s]</th><th>${a} [°/s²]</th></tr>${filas}</table>`;
  if (suave) {
    const explicacion = multiple
      ? "Polinomios quínticos por tramos que conservan posición, velocidad y aceleración en las uniones."
      : "La posición sigue un polinomio de quinto grado con velocidad y aceleración nulas al empezar y al llegar.";
    const ley = multiple ? "" : tex("q_i=q_{0,i}+D_i(10u^3-15u^4+6u^5),\\quad u=t/t_f");
    return paso(n, nombrePerfil(t.perfil), explicacion, ley + `<p>Duración ${f(t.tf, 3)} s · ${t.motivo_tf}.</p>` + tabla);
  }
  return paso(n, "Trayectoria LSPB sincronizada",
    `Todas las juntas usan el mismo t<sub>f</sub> y el mismo t<sub>c</sub> = κ·t<sub>f</sub> (κ = 0.25). Manda la junta más exigida: ${t.motivo_tf}.`,
    tex(`t_f=\\max_i\\left\\{\\frac{|D_i|}{(1-\\kappa)V_{\\max,i}},\\;\\sqrt{\\frac{|D_i|}{\\kappa(1-\\kappa)A_{\\max,i}}}\\right\\}=${f(t.tf, 3)}\\,\\text{s},\\qquad t_c=${f(t.tc, 3)}\\,\\text{s}`) +
    tex(`V_i=\\frac{D_i}{t_f-t_c},\\qquad A_i=\\frac{V_i}{t_c}`) +
    tabla);
}

export function tablaRamas(plan) {
  if (!plan || plan.tipo !== "pose") return '<p class="vacio">Las 8 ramas aparecen cuando se pide una pose (cinemática inversa).</p>';
  const ik = plan.ik;
  const lim = [[-165, 165], [-110, 110], [-110, 70], [-160, 160], [-120, 120], [-400, 400]];
  let h = `<p class="nota">Cada pose tiene hasta 8 soluciones: hombro (adelante/atrás) × codo (arriba/abajo) × muñeca (sin giro/girada). Todas se comprueban con FK. <b>Pasa el mouse sobre una fila</b> para ver esa postura en 3D; <b>haz clic</b> en una rama válida para usarla. Cálculo: ${f(ik.ms, 1)} ms.</p>`;
  h += '<table class="ramas"><tr><th>#</th><th>Rama</th>' + [1, 2, 3, 4, 5, 6].map((i) => `<th>q${i} [°]</th>`).join("") + "<th>e<sub>p</sub> [m]</th><th>e<sub>R</sub> [rad]</th><th>σ<sub>min</sub></th><th>Distancia</th><th>Estado</th></tr>";
  for (const r of ik.ramas) {
    const gana = r.indice === ik.elegida;
    const cls = gana ? "elegida" : r.valida ? "" : "invalida";
    h += `<tr class="${cls}" data-rama="${r.indice}">`;
    h += `<td>${r.indice}</td><td style="text-align:left">${r.hombro}<br><small>codo ${r.codo} · muñeca ${r.muneca}</small></td>`;
    if (r.q_deg) {
      r.q_deg.forEach((v, i) => {
        const fuera = v < lim[i][0] || v > lim[i][1];
        h += `<td class="${fuera ? "fuera" : ""}">${f(v, 1)}</td>`;
      });
      h += `<td>${feTxt(r.e_p_m)}</td><td>${feTxt(r.e_R_rad)}</td><td>${f(r.sigma_min, 3)}</td><td>${f(r.distancia, 3)}</td>`;
    } else {
      h += '<td colspan="10" style="text-align:left">no existe</td>';
    }
    const estado = gana ? '<span class="etq gana">ELEGIDA</span>' : r.valida ? '<span class="etq ok">válida</span>' : '<span class="etq no">descartada</span>';
    h += `<td class="motivo">${estado}<br><small>${(r.motivos || []).filter((m) => m !== "válida").join("; ")}</small></td></tr>`;
  }
  h += "</table>";
  h += `<p class="nota" style="margin-top:8px"><b>Por qué gana:</b> ${ik.razon}</p>`;
  return h;
}

export function panelLSPB(plan) {
  if (!plan || !plan.tray) return '<p class="vacio">Sin trayectoria.</p>';
  const t = plan.tray, c = plan.continuidad;
  const perfil = nombrePerfil(t.perfil);
  const lspb = t.tc != null;
  const descripcion = lspb
    ? "Perfil trapezoidal con fases de aceleración, velocidad constante y frenado. La aceleración cambia en las transiciones."
    : "Perfil quíntico con posición, velocidad y aceleración continuas.";
  return `<p class="nota"><b>${perfil}</b>. ${descripcion}</p>
  <table class="tabla-simple"><tr><th>Dato</th><th>Valor</th></tr>
  <tr><td>Duración t<sub>f</sub></td><td>${f(t.tf, 3)} s</td></tr>
  ${lspb ? `<tr><td>Mezcla t<sub>c</sub></td><td>${f(t.tc, 3)} s</td></tr>` : ""}
  <tr><td>Junta que manda</td><td>${t.junta_limitante ? "J" + t.junta_limitante : "—"} (${t.motivo_tf})</td></tr>
  <tr><td>Velocidad usada</td><td>${f(plan.vel_pct, 0)} % de la hoja de datos${plan.vel_cap_deg_s ? `, tope ${plan.vel_cap_deg_s} °/s` : ""}</td></tr>
  <tr><td>Salto máx. de q entre muestras de 1 ms</td><td>${f(c.salto_q_max_deg, 5)} °</td></tr>
  <tr><td>Salto máx. de q̇ entre muestras de 1 ms</td><td>${f(c.salto_qd_max_deg_s, 5)} °/s</td></tr>
  <tr><td>Llega a q<sub>f</sub> (diferencia)</td><td>${f(c.llega_a_qf_deg, 8)} °</td></tr>
  </table>`;
}

export function panelFK(v) {
  if (!v || !v.fk_vivo) return '<p class="vacio">Esperando datos del robot.</p>';
  const A = v.fk_vivo.A.map(filas4);
  const q = v.q_med_deg;
  const p = v.tcp_med;
  let h = `<div class="paso"><h4><span>FK</span>Cinemática directa con la posición MEDIDA (se actualiza sola)</h4>
  <p class="nota">Cada vez que llega /joint_states se recalculan las 6 matrices D-H y su producto. Es el mismo cálculo de clase, pero con los ángulos reales del robot.</p>
  <div class="ecu">${tex(`q_{medida}=[${q.map((x) => f(x, 2)).join(",\\;")}]^\\circ`)}</div></div>`;
  h += '<div class="dos-col">' + A.map((M, i) => tex(`A_${i + 1}(q_${i + 1})=${mat(M, 3)}`)).join("") + "</div>";
  h += tex(`T_0^6=\\prod_{i=1}^{6}A_i=${mat(filas4(v.fk_vivo.T06), 4)}`);
  h += tex(`p_{TCP}=(${f(p.x_mm, 1)},\\;${f(p.y_mm, 1)},\\;${f(p.z_mm, 1)})\\,\\text{mm}\\qquad q_{ABB}=[${p.cuaternion.map((x) => f(x, 4)).join(",\\;")}]`);
  return h;
}

export function panelJacobiano(v) {
  if (!v || !v.sing) return '<p class="vacio">Esperando datos del robot.</p>';
  const s = v.sing;
  const smax = Math.max(...s.sigma, 1e-9);
  const barras = s.sigma.map((x, i) => `<div style="height:${Math.max(2, (100 * x) / smax)}%"><span>${f(x, 3)}</span></div>`).join("");
  const ind = (titulo, valor, umbral, texto, unidad = "") => {
    const alerta = valor < umbral;
    const pct = Math.min(100, (100 * valor) / (umbral * 6));
    return `<div class="indicador ${alerta ? "alerta" : ""}"><small>${titulo}</small><br><b>${f(valor, 3)}${unidad}</b><div class="barrita"><div style="width:${pct}%;background:${alerta ? "#f2c14e" : "#3fd17a"}"></div></div><small>${texto}</small></div>`;
  };
  let h = `<p class="nota">El Jacobiano relaciona velocidades de las juntas con la velocidad de la punta: ẋ = J(q) q̇. Si un valor singular σ se acerca a 0, hay una dirección en la que la punta no se puede mover (singularidad) y las juntas tendrían que girar muy rápido. Se calcula en vivo con la posición medida.</p>`;
  h += `<div class="barras-sigma">${barras}</div><p class="nota" style="text-align:center">σ<sub>1</sub> … σ<sub>6</sub> de J con la parte lineal dividida por L = 0.3 m (para comparar m/rad con rad/rad)</p>`;
  h += `<table class="tabla-simple"><tr><th>Dato</th><th>Valor</th></tr>
  <tr><td>σ<sub>min</sub></td><td>${f(s.sigma_min, 4)}</td></tr>
  <tr><td>Rango (tolerancia max(m,n)·ε·σ<sub>max</sub>)</td><td>${s.rango} de 6</td></tr>
  <tr><td>Número de condición σ<sub>max</sub>/σ<sub>min</sub></td><td>${f(s.condicion, 1)}</td></tr>
  <tr><td>det J</td><td>${f(s.det_J, 6)}</td></tr></table>`;
  h += '<div class="indicadores">' +
    ind("Muñeca: |sen q5|", s.muneca_sin_q5, 0.09, "0 = ejes 4 y 6 alineados") +
    ind("Codo: |sen δ|", s.codo_sin_delta, 0.09, "0 = brazo estirado o plegado") +
    ind("Hombro: r<sub>w</sub>", s.hombro_r_w_m * 1000, 30, "0 = muñeca sobre el eje 1", " mm") + "</div>";
  h += tex(`\\det J=-a_2\\,L_3\\,\\sin\\delta\\;r_w\\;\\sin q_5`);
  if (s.J) h += tex(`J(q)=${mat(s.J, 3)}`);
  return h;
}

export function panelSecuencia(sec, pasosDemo) {
  let h = `<p class="nota">Esta es la trayectoria de demostración generada para el video: postura segura → cada junta sola (+5° y vuelta, 3 °/s) → las 6 a la vez → tres poses por cinemática inversa → regreso. Antes de cada paso se recalcula todo desde la posición medida.</p>`;
  h += `<div class="fila-botones"><button id="btn-sec" class="btn-principal" style="width:auto;padding:10px 18px">Iniciar secuencia</button>
  <label><input type="checkbox" id="sec-pausar" checked> pausar entre pasos (pulsar CONTINUAR o la tecla C)</label></div>`;
  if (sec) {
    h += `<div class="sec-estado"><b>${sec.estado.toUpperCase()}</b> · paso ${sec.paso} de ${sec.total}: ${sec.titulo || ""}<br><span class="nota">${sec.explicacion || ""}</span><br>${sec.texto || ""}
    ${sec.estado === "esperando" ? '<br><button id="btn-continuar" class="btn-continuar">CONTINUAR (C)</button>' : ""}</div>`;
  }
  h += '<ol class="lista-sec">' + pasosDemo.map((p, i) => {
    let c = "";
    if (sec && sec.paso === i + 1 && sec.estado !== "terminada") c = "actual";
    else if (sec && (i + 1 < sec.paso || sec.estado === "terminada")) c = "hecho";
    return `<li class="${c}">${i + 1}. ${p.titulo}</li>`;
  }).join("") + "</ol>";
  if (sec && sec.resultados && sec.resultados.length) {
    h += '<h3>Resultados</h3><table class="tabla-simple"><tr><th>Paso</th><th>Error seguimiento máx [°]</th><th>e<sub>p</sub> máx [mm]</th><th>Error final [°]</th><th>e<sub>p</sub> vs pose pedida [mm]</th></tr>';
    for (const r of sec.resultados) {
      h += `<tr><td>${r.titulo}</td><td>${f(r.err_seguimiento_max_deg, 3)}</td><td>${f(r.e_p_seguimiento_max_mm, 2)}</td><td>${f(r.error_final_max_deg, 4)}</td><td>${r.e_p_vs_pose_pedida_mm != null ? f(r.e_p_vs_pose_pedida_mm, 3) : "—"}</td></tr>`;
    }
    h += "</table>";
  }
  return h;
}

export function panelBitacora(v) {
  if (!v) return "";
  let h = `<p class="nota">Todo queda guardado en <code>${v.sesion}</code> (eventos.jsonl, muestras.csv, planes/, resumen.json). Para el informe: <code>python3 02_herramientas/analizar_sesion.py ${v.sesion}</code></p>`;
  if (v.ultimo_resumen) {
    const r = v.ultimo_resumen;
    h += `<h3>Último movimiento</h3><table class="tabla-simple"><tr><th>Dato</th><th>Valor</th></tr>
    <tr><td>Plan</td><td>${r.plan_id} · ${r.descripcion || ""}</td></tr>
    <tr><td>Estado</td><td>${r.estado}${r.motivo ? " · " + r.motivo : ""}</td></tr>
    <tr><td>Error de seguimiento máx.</td><td>${f(r.err_seguimiento_max_deg, 3)} °</td></tr>
    <tr><td>e<sub>p</sub> de seguimiento máx.</td><td>${f(r.e_p_seguimiento_max_mm, 2)} mm</td></tr>
    <tr><td>Error final (juntas)</td><td>${f(r.error_final_max_deg, 4)} °</td></tr>
    <tr><td>e<sub>p</sub> final vs pose pedida</td><td>${r.e_p_vs_pose_pedida_mm != null ? f(r.e_p_vs_pose_pedida_mm, 3) + " mm" : "—"}</td></tr>
    <tr><td>Respuesta del controlador</td><td>${r.resultado_controlador || "—"}</td></tr></table>`;
  }
  h += '<h3>Eventos</h3><div class="eventos">' + (v.eventos || []).slice().reverse().map((e) => `<div class="${e.nivel}">${e.hora} · ${e.texto}</div>`).join("") + "</div>";
  return h;
}

// ---- Poses guardadas: «Ir a pose» sigue las reglas de «Volver a cero» (SIM y ROS como siempre; RWS solo con C02).
export function permisoIrAPose(modo) {
  const m = modo || {};
  const permitido = typeof m.ir_a_pose_permitido === "boolean" ? m.ir_a_pose_permitido : m.tipo === "sim";
  return {
    permitido,
    motivo: permitido ? "" : (m.ir_a_pose_motivo || "Ir a una pose guardada con el robot real por RWS exige C02 verificado."),
  };
}

// ---- Vista de puntos de una figura
export const PLANOS_VISTA = { XY: [0, 1], XZ: [0, 2], YZ: [1, 2] };

// Proyección en el plano de dibujo: [u, v] en mm relativos al origen (por defecto, la primera muestra).
export function proyectarFigura(puntos_m, plano = "XY", origen_m = null) {
  const ejes = PLANOS_VISTA[String(plano).toUpperCase()] || PLANOS_VISTA.XY;
  const lista = Array.isArray(puntos_m) ? puntos_m : [];
  const o = origen_m || lista[0] || [0, 0, 0];
  return lista.map((p) => [(p[ejes[0]] - o[ejes[0]]) * 1000, (p[ejes[1]] - o[ejes[1]]) * 1000]);
}

// Distancia de un punto al recorrido (polilínea) más cercano; sirve en 2D y en 3D.
export function distanciaARecorrido(p, recorrido) {
  if (!recorrido?.length) return NaN;
  const dim = p.length;
  let mejor = Infinity;
  const d2 = (a, b) => { let s = 0; for (let i = 0; i < dim; i++) s += (a[i] - b[i]) ** 2; return s; };
  if (recorrido.length === 1) return Math.sqrt(d2(p, recorrido[0]));
  for (let k = 0; k + 1 < recorrido.length; k++) {
    const a = recorrido[k], b = recorrido[k + 1];
    let ab2 = 0, ap_ab = 0;
    for (let i = 0; i < dim; i++) { const ab = b[i] - a[i]; ab2 += ab * ab; ap_ab += (p[i] - a[i]) * ab; }
    const t = ab2 > 0 ? Math.min(1, Math.max(0, ap_ab / ab2)) : 0;
    let s = 0;
    for (let i = 0; i < dim; i++) s += (p[i] - (a[i] + t * (b[i] - a[i]))) ** 2;
    if (s < mejor) mejor = s;
  }
  return Math.sqrt(mejor);
}

// Error entre los puntos medidos (FK de las juntas recibidas) y el recorrido planificado, en mm.
// Ambas listas en metros, en la base del robot. Se usa la distancia 3D al tramo más cercano.
export function errorFigura(medidos_m, planificados_m) {
  const med = Array.isArray(medidos_m) ? medidos_m : [];
  if (!med.length || !planificados_m?.length) return { n: 0, max_mm: null, medio_mm: null };
  let max = 0, suma = 0;
  for (const p of med) {
    const d = distanciaARecorrido(p, planificados_m) * 1000;
    if (d > max) max = d;
    suma += d;
  }
  return { n: med.length, max_mm: max, medio_mm: suma / med.length };
}

// Agrega una muestra medida solo si se separa de la anterior (evita miles de puntos quietos).
export function agregarMuestra(lista, p, minimo_m = 0.0002, maximo = 4000) {
  const ultimo = lista.at(-1);
  if (ultimo && Math.hypot(p[0] - ultimo[0], p[1] - ultimo[1], p[2] - ultimo[2]) < minimo_m) return false;
  lista.push([p[0], p[1], p[2]]);
  if (lista.length > maximo) lista.splice(0, lista.length - maximo);
  return true;
}

// Paso "redondo" para las marcas de los ejes (1, 2 o 5 × 10^k mm).
export function pasoEje(rango_mm, marcas = 5) {
  const crudo = Math.max(rango_mm, 1e-6) / Math.max(1, marcas);
  const base = 10 ** Math.floor(Math.log10(crudo));
  const n = crudo / base;
  return (n <= 1 ? 1 : n <= 2 ? 2 : n <= 5 ? 5 : 10) * base;
}

// Encuadre con la MISMA escala en ambos ejes: devuelve funciones a píxeles (v crece hacia arriba).
export function encuadreFigura(conjuntos, ancho, alto, margen = 28) {
  const todos = conjuntos.flat().filter((p) => p && Number.isFinite(p[0]) && Number.isFinite(p[1]));
  let uMin = Math.min(...todos.map((p) => p[0])), uMax = Math.max(...todos.map((p) => p[0]));
  let vMin = Math.min(...todos.map((p) => p[1])), vMax = Math.max(...todos.map((p) => p[1]));
  if (!todos.length) { uMin = vMin = -10; uMax = vMax = 10; }
  const lado = Math.max(uMax - uMin, vMax - vMin, 2);
  const holgura = 0.08 * lado;
  const cu = (uMin + uMax) / 2, cv = (vMin + vMax) / 2;
  const escala = Math.min((ancho - 2 * margen) / (uMax - uMin + 2 * holgura || lado), (alto - 2 * margen) / (vMax - vMin + 2 * holgura || lado));
  const x = (u) => ancho / 2 + (u - cu) * escala;
  const y = (v) => alto / 2 - (v - cv) * escala;
  const u = (px) => cu + (px - ancho / 2) / escala;
  const v = (py) => cv - (py - alto / 2) / escala;
  return { escala, x, y, u, v, paso: pasoEje(Math.max(uMax - uMin, vMax - vMin, 2)) };
}

// ---- Velocidad: slider del plan y panel «Velocidad de las juntas»
export const VEL_PCT_VERIFICADO = 10; // lo ensayado con el robot real; por encima: solo simulación / fuera de lo verificado

// Aspecto del slider «Velocidad del plan»: nivel (ok · alto · muy-alto), aviso y nota del modo.
export function estadoSliderVel(pct, modo = {}) {
  const v = Number(pct);
  const verificado = modo.vel_pct_verificado ?? VEL_PCT_VERIFICADO;
  const nivel = !(v > verificado) ? "ok" : v > 2.5 * verificado ? "muy-alto" : "alto";
  const c02 = modo.tipo === "rws" && (modo.tope_fisico_c02 || (modo.velocidad_articular_verificada && modo.vlim_c02_deg_s));
  const nota = c02 ? "Robot real: tope físico C02 = 5 % de la ficha por junta" : "";
  const aviso = nivel === "ok" ? "" : `Más del ${verificado} %: solo simulación / fuera de lo verificado`;
  const titulo = [c02 ? "El RAPID limita cada junta al 5 % de la ficha: un plan más rápido no acelera el robot, solo tarda lo que fija C02." : "Velocidad nominal de cálculo del plan (% de la ficha, máx. 50 %).", aviso].filter(Boolean).join(" ");
  return { nivel, aviso, nota, titulo, c02: !!c02 };
}

// Uso del tope [%] → color de la barra: verde < 80 %, naranja hasta 100 %, rojo por encima.
export function nivelUso(pct) {
  if (pct == null || !Number.isFinite(pct)) return "sin";
  return pct > 100 ? "rojo" : pct >= 80 ? "naranja" : "verde";
}

// Filas por junta con lo que manda el servidor (misma cifra que se guarda en el registro de sesión).
export function filasVelocidad(v) {
  const med = v?.vel_medida_deg_s, plan = v?.vel_plan_deg_s, tope = v?.vel_tope_deg_s, max = v?.vel_max_mov_deg_s;
  return [0, 1, 2, 3, 4, 5].map((j) => {
    const m = med ? med[j] : null;
    const t = tope ? tope[j] : null;
    const uso = m != null && t > 0 ? (100 * Math.abs(m)) / t : null;
    const usoMax = max && t > 0 ? (100 * max[j]) / t : null;
    return { junta: j + 1, medida: m, plan: plan ? plan[j] : null, tope: t, uso, nivel: nivelUso(uso), max: max ? max[j] : null, usoMax, nivelMax: nivelUso(usoMax) };
  });
}

export function panelVelocidades(v) {
  if (!v) return '<p class="vacio">Esperando el estado del robot…</p>';
  const filas = filasVelocidad(v);
  const fuenteTope = v.vel_tope_fuente || "—";
  const fuentePlan = v.vel_plan_fuente ? ` (${v.vel_plan_fuente})` : "";
  const cuerpo = filas.map((r) => {
    const ancho = r.uso == null ? 0 : Math.min(100, r.uso);
    return `<tr><td>J${r.junta}</td><td class="vj-med">${f(r.medida, 1)}</td><td>${f(r.plan, 1)}</td><td>${f(r.tope, 1)}</td>`
      + `<td class="vj-uso"><div class="vj-barra ${r.nivel}"><div style="width:${ancho.toFixed(1)}%"></div></div><span>${r.uso == null ? "—" : f(r.uso, 0) + " %"}</span></td>`
      + `<td class="vj-max ${r.nivelMax}">${f(r.max, 1)}</td></tr>`;
  }).join("");
  const sinMedida = v.vel_medida_deg_s ? "" : '<p class="nota">Sin velocidad medida: faltan muestras recientes de posición.</p>';
  return `<table class="tabla-simple tabla-vel"><thead><tr><th>Junta</th><th>Medida °/s</th><th>Plan °/s${fuentePlan}</th><th>Tope °/s</th><th>% del tope</th><th>Máx. último mov.</th></tr></thead><tbody>${cuerpo}</tbody></table>`
    + `${sinMedida}<p class="nota">Tope: ${fuenteTope}. Medida: pendiente por mínimos cuadrados de las posiciones recibidas en ~0,3 s (con su marca de tiempo). Barra: verde &lt; 80 %, naranja hasta 100 %, rojo por encima.</p>`;
}
