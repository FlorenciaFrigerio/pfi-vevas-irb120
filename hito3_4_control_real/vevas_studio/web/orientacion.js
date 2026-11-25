import { Matrix4, Quaternion, MathUtils } from "./vendor/three.module.min.js";

const RAD = Math.PI / 180;

export function matrizZYX({ rz, ry, rx }) {
  const cz = Math.cos(rz * RAD), sz = Math.sin(rz * RAD);
  const cy = Math.cos(ry * RAD), sy = Math.sin(ry * RAD);
  const cx = Math.cos(rx * RAD), sx = Math.sin(rx * RAD);
  return [
    [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
    [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
    [-sy, cy * sx, cy * cx],
  ];
}

export function eulerZYX(R) {
  const cy = Math.hypot(R[0][0], R[1][0]);
  const ry = Math.atan2(-R[2][0], cy) / RAD;
  if (cy < 1e-9) return { rz: Math.atan2(-R[0][1], R[1][1]) / RAD, ry, rx: 0 };
  return {
    rz: Math.atan2(R[1][0], R[0][0]) / RAD,
    ry,
    rx: Math.atan2(R[2][1], R[2][2]) / RAD,
  };
}

// Giro respecto de un eje fijo de la base. El TCP conserva su posición.
export function rotarEnBase(pose, eje, anguloRad) {
  if (Math.abs(anguloRad) < 1e-12) return { ...pose };
  const R = matrizZYX(pose);
  const a = (eje + 1) % 3, b = (eje + 2) % 3;
  const c = Math.cos(anguloRad), s = Math.sin(anguloRad);
  const filaA = R[a], filaB = R[b];
  R[a] = filaA.map((v, j) => c * v - s * filaB[j]);
  R[b] = filaB.map((v, j) => s * filaA[j] + c * v);
  return { ...pose, ...eulerZYX(R) };
}

// El cambio de una única junta gira sus marcos descendientes y el TCP
// alrededor del eje Z del marco padre de Denavit-Hartenberg.
export function marcosGiroJunta(marcos, junta, anguloRad) {
  const padre = marcos[junta - 1];
  const [ex, ey, ez] = [padre[2], padre[6], padre[10]];
  const origen = [padre[3], padre[7], padre[11]];
  const c = Math.cos(anguloRad), s = Math.sin(anguloRad);
  const rotar = ([x, y, z]) => {
    const axial = (1 - c) * (ex * x + ey * y + ez * z);
    return [
      c * x + s * (ey * z - ez * y) + axial * ex,
      c * y + s * (ez * x - ex * z) + axial * ey,
      c * z + s * (ex * y - ey * x) + axial * ez,
    ];
  };
  return marcos.map((M, i) => {
    if (i < junta) return M;
    const p = rotar(origen.map((v, fila) => M[fila * 4 + 3] - v));
    const columnas = [0, 1, 2].map((col) => rotar([M[col], M[4 + col], M[8 + col]]));
    return origen.flatMap((v, fila) => [...columnas.map((col) => col[fila]), v + p[fila]]).concat([0, 0, 0, 1]);
  });
}

// Interpolación rígida para la presentación. Conserva los cuadros de entrada.
export function interpolarMarcos(a, b, fraccion) {
  if (!a) return b;
  if (!b || a === b || a.length !== b.length || fraccion <= 0) return a;
  if (fraccion >= 1) return b;
  const ma = new Matrix4(), mb = new Matrix4();
  const qa = new Quaternion(), qb = new Quaternion();
  return a.map((A, i) => {
    const B = b[i];
    ma.set(...A);
    mb.set(...B);
    qa.setFromRotationMatrix(ma).normalize();
    qb.setFromRotationMatrix(mb).normalize();
    qa.slerp(qb, fraccion);
    return ma.makeRotationFromQuaternion(qa)
      .setPosition(MathUtils.lerp(A[3], B[3], fraccion),
        MathUtils.lerp(A[7], B[7], fraccion),
        MathUtils.lerp(A[11], B[11], fraccion))
      .transpose().toArray();
  });
}
