import assert from "node:assert/strict";
import test from "node:test";
import { permisoMovimientosGuiados, pasosPlan } from "../web/calculos.js";

test("SIM habilita Preparar J5 y figuras con el texto de simulación", () => {
  const p = permisoMovimientosGuiados({ tipo: "sim", figuras_permitidas: true, figura_max_mm: 100 });
  assert.equal(p.permitido, true);
  assert.equal(p.subtituloJ5, "revisar en SIM");
  assert.equal(p.maxMm, 100);
});

test("RWS con C02 habilita con tope de tamaño y aviso de robot real", () => {
  const p = permisoMovimientosGuiados({ tipo: "rws", real: true, figuras_permitidas: true, figura_max_mm: 80 });
  assert.equal(p.permitido, true);
  assert.equal(p.subtituloJ5, "robot real · RWS C02");
  assert.equal(p.maxMm, 80);
  assert.match(p.tituloFiguras, /GO, armado y confirmación/);
  assert.match(p.textoDialogo, /Preparar J5 a \+30°/);
});

test("ROS real y RWS sin C02 quedan deshabilitados con motivo claro", () => {
  const ros = permisoMovimientosGuiados({ tipo: "ros", real: true, figuras_permitidas: false });
  assert.equal(ros.permitido, false);
  assert.match(ros.tituloFiguras, /ROS real/);
  assert.equal(ros.tituloJ5, ros.tituloFiguras);
  const rws = permisoMovimientosGuiados({ tipo: "rws", real: true, figuras_permitidas: false });
  assert.equal(rws.permitido, false);
  assert.match(rws.tituloJ5, /C02/);
  // un servidor viejo que no informa el permiso: solo SIM
  assert.equal(permisoMovimientosGuiados({ tipo: "rws", velocidad_articular_verificada: true }).permitido, false);
  assert.equal(permisoMovimientosGuiados({ tipo: "sim" }).permitido, true);
  assert.equal(permisoMovimientosGuiados(undefined).permitido, false);
});

test("los pasos de una figura con esquinas mencionan las paradas", () => {
  const ceros = [0, 0, 0, 0, 0, 0];
  const tray = { perfil: "camino_quintico", V_deg_s: ceros, A_deg_s2: ceros, tf: 6, motivo_tf: "prueba" };
  const plan = { tipo: "figura", descripcion: "Triángulo · 30 mm · XY", tray,
    figura: { nombre: "triangulo", plano: "XY", n_puntos: 65, tamano_mm: 30, paradas: [22, 43] } };
  assert.match(pasosPlan(plan, {}), /2 puntos intermedios/);
  const circulo = { ...plan, figura: { ...plan.figura, nombre: "circulo", paradas: [] } };
  assert.doesNotMatch(pasosPlan(circulo, {}), /intermedio/);
});
