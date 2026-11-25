import assert from "node:assert/strict";
import test from "node:test";
import { estadoSliderVel, filasVelocidad, nivelUso, panelVelocidades } from "../web/calculos.js";

test("slider: hasta 10 % normal; por encima naranja y luego rojo con aviso fuera de lo verificado", () => {
  assert.equal(estadoSliderVel(5, { tipo: "sim" }).nivel, "ok");
  assert.equal(estadoSliderVel(10, { tipo: "sim" }).aviso, "");
  const alto = estadoSliderVel(20, { tipo: "sim" });
  assert.equal(alto.nivel, "alto");
  assert.match(alto.aviso, /solo simulación \/ fuera de lo verificado/);
  assert.equal(estadoSliderVel(30, { tipo: "sim" }).nivel, "muy-alto");
  assert.equal(estadoSliderVel(50, { tipo: "sim" }).nivel, "muy-alto");
});

test("slider: con RWS y C02 muestra el tope físico; sin C02 o en SIM no", () => {
  const rws = estadoSliderVel(5, { tipo: "rws", real: true, tope_fisico_c02: true });
  assert.equal(rws.nota, "Robot real: tope físico C02 = 5 % de la ficha por junta");
  assert.equal(rws.c02, true);
  assert.equal(estadoSliderVel(5, { tipo: "rws", real: true, tope_fisico_c02: false }).nota, "");
  assert.equal(estadoSliderVel(5, { tipo: "sim" }).nota, "");
});

test("uso del tope: verde < 80 %, naranja hasta 100 %, rojo por encima", () => {
  assert.equal(nivelUso(null), "sin");
  assert.equal(nivelUso(79.9), "verde");
  assert.equal(nivelUso(80), "naranja");
  assert.equal(nivelUso(100), "naranja");
  assert.equal(nivelUso(100.1), "rojo");
});

test("filas y panel de velocidades con los datos del estado", () => {
  const v = {
    vel_medida_deg_s: [10, 0, 0, 0, 0, -21], vel_plan_deg_s: [12.5, 0, 0, 0, 0, 0],
    vel_tope_deg_s: [12.5, 12.5, 12.5, 16, 16, 21], vel_max_mov_deg_s: [11, 0, 0, 0, 0, 22],
    vel_tope_fuente: "C02 · RAPID (5 % de la ficha)", vel_plan_fuente: "plan",
  };
  const filas = filasVelocidad(v);
  assert.equal(filas[0].uso, 80);
  assert.equal(filas[0].nivel, "naranja");
  assert.equal(filas[5].uso, 100);
  assert.equal(filas[5].nivelMax, "rojo");
  assert.equal(filas[1].nivel, "verde");
  const html = panelVelocidades(v);
  assert.match(html, /C02 · RAPID/);
  assert.match(html, /vj-barra naranja/);
  assert.equal((html.match(/<tr>/g) || []).length, 7);
  const sin = panelVelocidades({ vel_tope_deg_s: v.vel_tope_deg_s, vel_max_mov_deg_s: [0, 0, 0, 0, 0, 0] });
  assert.match(sin, /Sin velocidad medida/);
  assert.equal(filasVelocidad({}).every((r) => r.nivel === "sin"), true);
});
