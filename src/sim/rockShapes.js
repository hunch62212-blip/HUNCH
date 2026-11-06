// Shared rock point clouds. Physics uses their convex hull; rendering uses the same hull,
// so what you see is exactly what the rover can hit.
import { mulberry32, makeNoise } from './terrain.js';

function icosphere(sub) {
  const t = (1 + Math.sqrt(5)) / 2;
  let v = [[-1, t, 0], [1, t, 0], [-1, -t, 0], [1, -t, 0], [0, -1, t], [0, 1, t], [0, -1, -t], [0, 1, -t], [t, 0, -1], [t, 0, 1], [-t, 0, -1], [-t, 0, 1]];
  let f = [[0, 11, 5], [0, 5, 1], [0, 1, 7], [0, 7, 10], [0, 10, 11], [1, 5, 9], [5, 11, 4], [11, 10, 2], [10, 7, 6], [7, 1, 8], [3, 9, 4], [3, 4, 2], [3, 2, 6], [3, 6, 8], [3, 8, 9], [4, 9, 5], [2, 4, 11], [6, 2, 10], [8, 6, 7], [9, 8, 1]];
  v = v.map((p) => { const L = Math.hypot(...p); return p.map((c) => c / L); });
  for (let s = 0; s < sub; s++) {
    const cache = new Map(); const nf = [];
    const mid = (a, b) => {
      const k = a < b ? a + '_' + b : b + '_' + a;
      if (cache.has(k)) return cache.get(k);
      const p = [(v[a][0] + v[b][0]) / 2, (v[a][1] + v[b][1]) / 2, (v[a][2] + v[b][2]) / 2];
      const L = Math.hypot(...p); v.push(p.map((c) => c / L)); cache.set(k, v.length - 1); return v.length - 1;
    };
    for (const [a, b, c] of f) { const ab = mid(a, b), bc = mid(b, c), ca = mid(c, a); nf.push([a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]); }
    f = nf;
  }
  return v;
}

let cached = null;
export function rockShapes() {
  if (cached) return cached;
  const shapes = [];
  for (let k = 0; k < 6; k++) {
    const rand = mulberry32(1000 + k * 77);
    const noise = makeNoise(rand);
    const base = icosphere(2);
    const pts = [];
    // a few cutting planes make blocky, fractured boulders
    const planes = [];
    for (let p = 0; p < 5; p++) {
      const a = rand() * Math.PI * 2, b = Math.acos(2 * rand() - 1);
      planes.push({ n: [Math.sin(b) * Math.cos(a), Math.cos(b), Math.sin(b) * Math.sin(a)], d: 0.62 + rand() * 0.3 });
    }
    for (const p of base) {
      let r = 1 + 0.22 * noise(p[0] * 1.7 + k, p[2] * 1.7 - k) + 0.1 * noise(p[1] * 3.1 + 2 * k, p[0] * 3.1);
      let q = p.map((c) => c * r);
      for (const pl of planes) {
        const dist = q[0] * pl.n[0] + q[1] * pl.n[1] + q[2] * pl.n[2];
        if (dist > pl.d) q = q.map((c, i) => c - pl.n[i] * (dist - pl.d));
      }
      pts.push(q[0] * 0.5, q[1] * 0.5, q[2] * 0.5); // unit diameter
    }
    shapes.push(new Float32Array(pts));
  }
  cached = shapes;
  return shapes;
}

// world-space transform for a rock instance (shared by physics + render)
export function rockTransform(r) {
  const hx = r.d * r.sx, hy = r.d * r.sy, hz = r.d * r.sz; // full extents
  const cy = r.y + hy * (0.5 - r.bury);
  return { scale: [hx, hy, hz], pos: [r.x, cy, r.z], yaw: r.yaw, tilt: r.tilt, height: hy * (1 - r.bury) };
}
