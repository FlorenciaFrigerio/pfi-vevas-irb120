// Gráficas con uPlot: error en vivo (ventana deslizante) y trayectorias LSPB.

export const COLORES_JUNTAS = ["#c98425", "#2187a4", "#25866b", "#898427", "#9366aa", "#c04b65"];

const ejesOscuros = (etiqueta) => [
  { stroke: "#748a9b", grid: { stroke: "#e9eef4" }, ticks: { stroke: "#dce5ed" }, font: "11px sans-serif" },
  { stroke: "#748a9b", grid: { stroke: "#e9eef4" }, ticks: { stroke: "#dce5ed" }, font: "11px sans-serif", label: etiqueta, labelSize: 14, labelFont: "11px sans-serif", size: 52 },
];

function medida(cont) {
  const w = Math.max(220, cont.clientWidth - 4);
  const h = Math.max(120, cont.clientHeight - 30);
  return { width: w, height: h };
}

export class GraficaDeslizante {
  // series: [{nombre, color}], ventana en segundos, lineas: valores horizontales de referencia
  constructor(cont, { series, unidad, ventana = 20, lineas = [] }) {
    this.cont = cont;
    this.ventana = ventana;
    this.lineas = lineas;
    this.datos = [[], ...series.map(() => [])];
    const opts = {
      ...medida(cont),
      legend: { show: true, live: false },
      cursor: { show: false },
      scales: { x: { time: false }, y: { range: (u, min, max) => this._rango(min, max) } },
      axes: ejesOscuros(unidad),
      series: [{ label: "t [s]" }, ...series.map((s) => ({ label: s.nombre, stroke: s.color, width: 1.6, dash: s.dash, points: { show: false } }))],
      hooks: { draw: [(u) => this._dibujarLineas(u)] },
    };
    this.u = new uPlot(opts, this.datos, cont);
    this.observador=new ResizeObserver(() => this.u.setSize(medida(cont)));this.observador.observe(cont);
  }

  _rango(min, max) {
    let m = Math.max(Math.abs(min || 0), Math.abs(max || 0));
    for (const l of this.lineas) m = Math.max(m, Math.abs(l.valor) * 1.15);
    m = Math.max(m * 1.15, 1e-3);
    return this.lineas.some((l) => l.valor < 0) || min < 0 ? [-m, m] : [0, m];
  }

  _dibujarLineas(u) {
    const ctx = u.ctx;
    for (const l of this.lineas) {
      const y = u.valToPos(l.valor, "y", true);
      ctx.save();
      ctx.strokeStyle = l.color;
      ctx.setLineDash([6, 4]);
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(u.bbox.left, y);
      ctx.lineTo(u.bbox.left + u.bbox.width, y);
      ctx.stroke();
      ctx.restore();
    }
  }

  agregar(t, valores) {
    this.datos[0].push(t);
    valores.forEach((v, i) => this.datos[i + 1].push(v == null ? null : v));
    while (this.datos[0].length && this.datos[0][0] < t - this.ventana) this.datos.forEach((s) => s.shift());
  }

  dibujar() { this.u.setData(this.datos); }
  destruir() { this.observador.disconnect(); this.u.destroy(); }
}

export class GraficaTrayectoria {
  constructor(cont, titulo, unidad) {
    this.cont = cont;
    this.titulo = titulo;
    this.unidad = unidad;
    this.u = null;
    this.tActual = null;
  }

  fijar(t, filas) {
    // filas: matriz [muestra][junta]
    const datos = [t];
    for (let j = 0; j < 6; j++) datos.push(filas.map((f) => f[j]));
    if (this.u) this.u.destroy();
    const opts = {
      width: Math.max(300, this.cont.clientWidth - 4),
      height: 170,
      title: this.titulo,
      legend: { show: true, live: false },
      cursor: { show: true, drag: { x: false, y: false } },
      scales: { x: { time: false } },
      axes: ejesOscuros(this.unidad),
      series: [{ label: "t [s]" }, ...COLORES_JUNTAS.map((c, i) => ({ label: `J${i + 1}`, stroke: c, width: 1.6, points: { show: false } }))],
      hooks: { draw: [(u) => this._cursor(u)] },
    };
    this.u = new uPlot(opts, datos, this.cont);
  }

  _cursor(u) {
    if (this.tActual == null) return;
    const x = u.valToPos(this.tActual, "x", true);
    const ctx = u.ctx;
    ctx.save();
    ctx.strokeStyle = "#13778a";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x, u.bbox.top);
    ctx.lineTo(x, u.bbox.top + u.bbox.height);
    ctx.stroke();
    ctx.restore();
  }

  marcar(t) {
    this.tActual = t;
    if (this.u) this.u.redraw(false, false);
  }
}
