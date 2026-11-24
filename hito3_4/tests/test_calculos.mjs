import assert from "node:assert/strict";
import test from "node:test";
import {
  agregarMuestra, distanciaARecorrido, encuadreFigura, errorFigura, pasoEje, permisoIrAPose, permisoMovimientosGuiados,
  pasosPlan, proyectarFigura,
} from "../web/calculos.js";

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
  assert.match(p.textoDialogo, /J5 a \+30°/);
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

const cerca = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) <= tol, `${a} ≈ ${b}`);

test("proyección 2D en el plano de la figura, en mm y relativa a la primera muestra", () => {
  const o = [0.4, -0.1, 0.3];
  const puntos = [o, [0.41, -0.1, 0.32], [0.4, -0.08, 0.29]];
  assert.deepEqual(proyectarFigura(puntos, "XZ").map((p) => p.map((x) => Math.round(x * 1e6) / 1e6)), [[0, 0], [10, 20], [0, -10]]);
  assert.deepEqual(proyectarFigura(puntos, "yz").map((p) => p.map((x) => Math.round(x * 1e6) / 1e6)), [[0, 0], [0, 20], [20, -10]]);
  // origen explícito y plano desconocido → XY
  assert.deepEqual(proyectarFigura([[0.002, 0.003, 9]], "QQ", [0, 0, 0]).map((p) => p.map((x) => Math.round(x * 1e6) / 1e6)), [[2, 3]]);
  assert.deepEqual(proyectarFigura(undefined, "XY"), []);
});

test("distancia al recorrido: al tramo más cercano, no solo a las muestras", () => {
  const cuadrado = [[0, 0], [10, 0], [10, 10], [0, 10], [0, 0]];
  cerca(distanciaARecorrido([5, 1], cuadrado), 1);          // sobre el lado inferior, entre muestras
  cerca(distanciaARecorrido([5, 5], cuadrado), 5);
  cerca(distanciaARecorrido([13, 14], cuadrado), 5);        // fuera de la esquina (10, 10)
  cerca(distanciaARecorrido([1, 2, 2], [[1, 0, 0]]), Math.hypot(2, 2));
  cerca(distanciaARecorrido([3, 4, 0], [[0, 0, 0], [0, 0, 0]]), 5);   // tramo degenerado
  assert.ok(Number.isNaN(distanciaARecorrido([0, 0], [])));
});

test("error medido vs planificado en mm (máximo y medio)", () => {
  const plan = [[0, 0, 0], [0.01, 0, 0], [0.01, 0.01, 0]];   // metros
  const medidos = [[0.005, 0.001, 0], [0.011, 0.005, 0], [0.01, 0.01, 0.003]];
  const e = errorFigura(medidos, plan);
  assert.equal(e.n, 3);
  cerca(e.max_mm, 3, 1e-9);
  cerca(e.medio_mm, (1 + 1 + 3) / 3, 1e-9);
  assert.deepEqual(errorFigura([], plan), { n: 0, max_mm: null, medio_mm: null });
  assert.deepEqual(errorFigura(medidos, []), { n: 0, max_mm: null, medio_mm: null });
});

test("encuadre con la misma escala en ambos ejes y v hacia arriba", () => {
  const puntos = [[0, 0], [40, 0], [40, 10], [0, 10]];
  const enc = encuadreFigura([puntos], 400, 300, 20);
  const ancho = enc.x(40) - enc.x(0), alto = enc.y(0) - enc.y(10);
  cerca(ancho / 40, alto / 10, 1e-12);                       // misma escala
  assert.ok(alto > 0);                                        // y de pantalla decrece al subir v
  for (const [u, v] of puntos) {
    assert.ok(enc.x(u) >= 20 && enc.x(u) <= 380 && enc.y(v) >= 20 && enc.y(v) <= 280);
    cerca(enc.u(enc.x(u)), u, 1e-9);
    cerca(enc.v(enc.y(v)), v, 1e-9);
  }
  // un recorrido alto y angosto queda limitado por la altura
  const alto2 = encuadreFigura([[[0, 0], [1, 50]]], 400, 300, 20);
  assert.ok(alto2.y(0) - alto2.y(50) <= 260 + 1e-9);
  assert.equal(enc.paso, 10);
  assert.deepEqual([pasoEje(25), pasoEje(100), pasoEje(3), pasoEje(0.4)], [5, 20, 1, 0.1]);
});

test("las muestras medidas quietas no se repiten", () => {
  const lista = [];
  assert.equal(agregarMuestra(lista, [0, 0, 0]), true);
  assert.equal(agregarMuestra(lista, [0.0001, 0, 0]), false);
  assert.equal(agregarMuestra(lista, [0.001, 0, 0]), true);
  assert.equal(lista.length, 2);
  for (let i = 0; i < 10; i++) agregarMuestra(lista, [i, 0, 0], 0.0002, 5);
  assert.equal(lista.length, 5);
  assert.deepEqual(lista.at(-1), [9, 0, 0]);
});

test("«Ir a pose»: el servidor manda; sin dato, solo SIM", () => {
  assert.equal(permisoIrAPose({ tipo: "rws", ir_a_pose_permitido: true }).permitido, true);
  const rws = permisoIrAPose({ tipo: "rws", ir_a_pose_permitido: false, ir_a_pose_motivo: "Exige C02 verificado." });
  assert.equal(rws.permitido, false);
  assert.match(rws.motivo, /C02/);
  assert.equal(permisoIrAPose({ tipo: "ros", ir_a_pose_permitido: true }).permitido, true);
  assert.equal(permisoIrAPose({ tipo: "sim" }).permitido, true);
  assert.equal(permisoIrAPose(undefined).permitido, false);
});
