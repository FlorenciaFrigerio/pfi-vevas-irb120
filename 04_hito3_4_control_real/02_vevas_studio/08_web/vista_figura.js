// Vista 2D de una figura: muestras del TCP como puntos en el plano de dibujo (mm, misma escala en ambos ejes).
// Planificado: degradado claro → oscuro según el orden, flechas de sentido y numeración cada pocos puntos.
// Inicio, fin y paradas (esquinas o cúspide) marcados. Medido (FK de las juntas recibidas) en violeta.
import { encuadreFigura, f } from "./calculos.js";

const C_INI = [143, 211, 224];   // primer punto (claro)
const C_FIN = [16, 86, 107];     // último punto (oscuro)
const C_MEDIDO = "#7a3eb1";
const C_PARADA = "#c87d27";
const C_INICIO = "#13785c";
const C_FIN_MARCA = "#b62d41";

const mezcla = (t) => `rgb(${C_INI.map((a, i) => Math.round(a + (C_FIN[i] - a) * t)).join(",")})`;

export function dibujarVistaFigura(canvas, { plan2d = [], paradas = [], medidos2d = [], ejes = ["X", "Y"], vacio = "" } = {}) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = canvas.clientWidth || 320, h = canvas.clientHeight || 220;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#fbfdfe";
  ctx.fillRect(0, 0, w, h);
  ctx.font = "10px Inter, 'Segoe UI', system-ui, sans-serif";
  if (!plan2d.length) {
    ctx.fillStyle = "#7d8f9e";
    ctx.textAlign = "center";
    ctx.fillText(vacio || "Sin puntos para mostrar.", w / 2, h / 2);
    return null;
  }
  const enc = encuadreFigura([plan2d, medidos2d], w, h, 30);
  const { x, y, paso } = enc;

  // Cuadrícula con marcas en mm
  ctx.lineWidth = 1;
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (let u = Math.ceil(enc.u(0) / paso) * paso; u <= enc.u(w); u += paso) {
    const px = Math.round(x(u)) + 0.5;
    ctx.strokeStyle = Math.abs(u) < paso / 1e6 ? "#c5d3de" : "#e8eef4";
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, h - 14); ctx.stroke();
    ctx.fillStyle = "#8a9cab";
    if (px > 40 && px < w - 70) ctx.fillText(f(u, paso < 1 ? 1 : 0), px, h - 12);
  }
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  for (let v = Math.ceil(enc.v(h) / paso) * paso; v <= enc.v(0); v += paso) {
    const py = Math.round(y(v)) + 0.5;
    ctx.strokeStyle = Math.abs(v) < paso / 1e6 ? "#c5d3de" : "#e8eef4";
    ctx.beginPath(); ctx.moveTo(26, py); ctx.lineTo(w, py); ctx.stroke();
    ctx.fillStyle = "#8a9cab";
    if (py > 18 && py < h - 22) ctx.fillText(f(v, paso < 1 ? 1 : 0), 2, py);
  }
  ctx.fillStyle = "#5f788b";
  ctx.font = "600 10px Inter, 'Segoe UI', system-ui, sans-serif";
  ctx.textAlign = "right";
  ctx.textBaseline = "bottom";
  ctx.fillText(`${ejes[0]} [mm] →`, w - 4, h - 14);
  ctx.textAlign = "left";
  ctx.textBaseline = "top";
  ctx.fillText(`↑ ${ejes[1]} [mm]`, 30, 3);
  ctx.font = "10px Inter, 'Segoe UI', system-ui, sans-serif";

  // Recorrido planificado: línea fina, flechas de sentido y puntos en degradado
  const n = plan2d.length;
  ctx.strokeStyle = "#bcd8e0";
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  plan2d.forEach((p, k) => (k ? ctx.lineTo(x(p[0]), y(p[1])) : ctx.moveTo(x(p[0]), y(p[1]))));
  ctx.stroke();
  const cadaFlecha = Math.max(1, Math.round(n / 10));
  for (let k = Math.floor(cadaFlecha / 2); k + 1 < n; k += cadaFlecha) {
    const ax = x(plan2d[k][0]), ay = y(plan2d[k][1]), bx = x(plan2d[k + 1][0]), by = y(plan2d[k + 1][1]);
    const largo = Math.hypot(bx - ax, by - ay);
    if (largo < 0.5) continue;
    const ang = Math.atan2(by - ay, bx - ax), mx = (ax + bx) / 2, my = (ay + by) / 2;
    ctx.save();
    ctx.translate(mx, my);
    ctx.rotate(ang);
    ctx.fillStyle = mezcla(k / Math.max(1, n - 1));
    ctx.beginPath(); ctx.moveTo(6, 0); ctx.lineTo(-4, -4); ctx.lineTo(-4, 4); ctx.closePath(); ctx.fill();
    ctx.restore();
  }
  plan2d.forEach((p, k) => {
    ctx.fillStyle = mezcla(k / Math.max(1, n - 1));
    ctx.beginPath(); ctx.arc(x(p[0]), y(p[1]), 2.6, 0, Math.PI * 2); ctx.fill();
  });
  const cadaNumero = Math.max(4, Math.round(n / 8));
  ctx.fillStyle = "#3b6276";
  ctx.textAlign = "left";
  ctx.textBaseline = "bottom";
  for (let k = cadaNumero; k < n - 1; k += cadaNumero) ctx.fillText(String(k + 1), x(plan2d[k][0]) + 4, y(plan2d[k][1]) - 3);

  // Medido (encima del planificado)
  if (medidos2d.length) {
    ctx.strokeStyle = C_MEDIDO + "88";
    ctx.lineWidth = 1;
    ctx.beginPath();
    medidos2d.forEach((p, k) => (k ? ctx.lineTo(x(p[0]), y(p[1])) : ctx.moveTo(x(p[0]), y(p[1]))));
    ctx.stroke();
    ctx.fillStyle = C_MEDIDO + "b3";
    for (const p of medidos2d) { ctx.beginPath(); ctx.arc(x(p[0]), y(p[1]), 1.5, 0, Math.PI * 2); ctx.fill(); }
  }

  // Paradas (v = 0), inicio y fin
  ctx.textBaseline = "bottom";
  paradas.forEach((k, i) => {
    const p = plan2d[k];
    if (!p) return;
    ctx.strokeStyle = C_PARADA;
    ctx.lineWidth = 2;
    ctx.strokeRect(x(p[0]) - 5, y(p[1]) - 5, 10, 10);
    ctx.fillStyle = C_PARADA;
    ctx.fillText(`P${i + 1}`, x(p[0]) + 7, y(p[1]) - 5);
  });
  const ini = plan2d[0], fin = plan2d[n - 1];
  const cerrada = Math.hypot(x(ini[0]) - x(fin[0]), y(ini[1]) - y(fin[1])) < 3;
  ctx.lineWidth = 2;
  ctx.strokeStyle = C_FIN_MARCA;
  ctx.beginPath(); ctx.arc(x(fin[0]), y(fin[1]), cerrada ? 9 : 6, 0, Math.PI * 2); ctx.stroke();
  ctx.strokeStyle = C_INICIO;
  ctx.fillStyle = "#fff";
  ctx.beginPath(); ctx.arc(x(ini[0]), y(ini[1]), 5.5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  ctx.font = "600 10px Inter, 'Segoe UI', system-ui, sans-serif";
  ctx.textAlign = "right";
  ctx.textBaseline = "top";
  ctx.fillStyle = C_INICIO;
  ctx.fillText(cerrada ? "Inicio y fin" : "Inicio", x(ini[0]) - 8, y(ini[1]) + 6);
  if (!cerrada) {
    ctx.fillStyle = C_FIN_MARCA;
    ctx.fillText("Fin", x(fin[0]) - 8, y(fin[1]) + 6);
  }
  return enc;
}
