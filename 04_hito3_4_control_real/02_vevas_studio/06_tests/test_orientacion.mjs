import assert from "node:assert/strict";
import test from "node:test";
import { eulerZYX, interpolarMarcos, marcosGiroJunta, matrizZYX, rotarEnBase } from "../08_web/orientacion.js";

const RAD = Math.PI / 180;
const origen = { x: 410, y: -60, z: 284, rz: 0, ry: 0, rx: 0 };

function aproximadamente(real, esperado) {
  assert.deepEqual(real.map((fila) => fila.length), esperado.map((fila) => fila.length));
  real.forEach((fila, i) => fila.forEach((v, j) => assert.ok(Math.abs(v - esperado[i][j]) < 1e-9, `${v} ≈ ${esperado[i][j]}`)));
}

test("los giros de 90 grados siguen la mano derecha en X, Y y Z", () => {
  const esperado = [
    [[1, 0, 0], [0, 0, -1], [0, 1, 0]],
    [[0, 0, 1], [0, 1, 0], [-1, 0, 0]],
    [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
  ];
  esperado.forEach((R, eje) => aproximadamente(matrizZYX(rotarEnBase(origen, eje, 90 * RAD)), R));
});

test("girar en la base premultiplica la orientación y preserva el XYZ del TCP", () => {
  const pose = { ...origen, rz: 37, ry: -21, rx: 49 };
  const girada = rotarEnBase(pose, 0, Math.PI / 2);
  assert.deepEqual([girada.x, girada.y, girada.z], [pose.x, pose.y, pose.z]);
  const [a, b, c] = matrizZYX(pose);
  aproximadamente(matrizZYX(girada), [a, c.map((x) => -x), b]);
  aproximadamente(matrizZYX(rotarEnBase(girada, 0, -Math.PI / 2)), matrizZYX(pose));
});

test("la conversión a Euler conserva la rotación al pasar por ±90° y ±180°", () => {
  for (const ry of [-179, -90, -89.99999, 89.99999, 90, 179]) {
    const R = matrizZYX({ rz: 179.9, ry, rx: -177 });
    aproximadamente(matrizZYX(eulerZYX(R)), R);
  }
  const giro = rotarEnBase({ ...origen, rz: 179 }, 2, 7 * RAD);
  assert.ok(Math.abs(giro.rz + 174) < 1e-9);
});

test("los giros sucesivos se componen en orden", () => {
  const girada = rotarEnBase(rotarEnBase(origen, 0, 90 * RAD), 2, 90 * RAD);
  aproximadamente(matrizZYX(girada), [[0, 0, 1], [1, 0, 0], [0, 1, 0]]);
  const inversa = rotarEnBase(rotarEnBase(origen, 2, 90 * RAD), 0, 90 * RAD);
  aproximadamente(matrizZYX(inversa), [[0, -1, 0], [0, 0, -1], [1, 0, 0]]);
});

test("el objetivo articular gira los marcos siguientes alrededor del eje y origen del padre", () => {
  const padre = [1, 0, 0, 0.2, 0, 0, 1, 0.3, 0, -1, 0, 0.4, 0, 0, 0, 1];
  const hijo = [1, 0, 0, 0.5, 0, 1, 0, 0.3, 0, 0, 1, 0.4, 0, 0, 0, 1];
  const [anterior, giro] = marcosGiroJunta([padre, hijo], 1, 90 * RAD);
  assert.equal(anterior, padre);
  aproximadamente([giro], [[0, 0, 1, 0.2, 0, 1, 0, 0.3, -1, 0, 0, 0.1, 0, 0, 0, 1]]);
  aproximadamente(marcosGiroJunta([padre, giro], 1, -90 * RAD), [padre, hijo]);
});

test("la interpolación rígida conserva los marcos medidos y toma el giro corto", () => {
  const marco = (rz, xyz) => matrizZYX({ rz, ry: 0, rx: 0 })
    .flatMap((fila, i) => [...fila, xyz[i]]).concat([0, 0, 0, 1]);
  const a = [marco(170, [0.2, 0.3, 0.4])];
  const b = [marco(-170, [0.4, 0.1, 0.2])];
  const copia = structuredClone([a, b]);
  aproximadamente(interpolarMarcos(a, b, 0.5), [marco(180, [0.3, 0.2, 0.3])]);
  assert.deepEqual([a, b], copia);
  assert.equal(interpolarMarcos(a, b, 0), a);
  assert.equal(interpolarMarcos(a, b, 1), b);
});
