// Map-relative localization (MRL) against an orbital elevation model (v1.5.0).
//
// Why: on a one-way 2 km traverse the SLAM map is built from the drifting estimate itself, so scan-to-map matching
// keeps the estimate consistent with its own map but cannot see slow odometry drift (0.3-0.5 % of distance with the
// soil-dependent slip). Only the fiducials at the pad, habitat and power dock gave absolute fixes, and the error grew
// to 7-9 m between them. Real lunar rover concepts close that gap by matching the rover's local elevation map to an
// orbital digital elevation model (LRO NAC stereo DTMs: 2-5 m posting, ~0.5 m vertical precision).
//
// The orbital DEM [ASSUMED pre-mission product]: the site's elevation averaged over 4 x 4 m cells (2 x 2 samples per
// cell), plus Gaussian height noise sigma 0.3 m and a smooth +-0.3 m long-wavelength warp (stereo DTM errors are
// spatially correlated). It does not hold rocks (sub-cell) or anything built on site. Seeded, so repeatable.
//
// The fix: every ~40 m of driving, the SLAM height map within 45 m of the estimate is sampled on a 2 m grid, its
// best-fit plane is removed (likewise for the DEM samples at each candidate offset), and the residual heights are
// correlated (normalised cross-correlation) over a +-14 m search window (1 m steps, then a 0.25 m refinement). It is
// accepted only when the patch has relief (r.m.s. about its plane > 0.2 m, 300+ points), the peak is strong
// (NCC > 0.8) and unique (beats everything more than 4 m away by > 0.03), and the offset is significant (> 1.2 m)
// yet within the plausible drift since the last fix. A single match is good to ~1.2-1.5 m, so Autonomy.orbitalFix
// averages >= 3 agreeing matches before it applies their mean through the same pose-graph frame shift the fiducial
// fixes use (Autonomy.applyFrameShift -> Slam.shiftFrame).
import { WORLD } from './config.js';
import { heightAt, mulberry32 } from './terrain.js';

const POST = 4;          // m, DEM posting
const NOISE = 0.3;       // m, per-post height noise (1 sigma; NAC DTM vertical precision is typically 0.2-0.5 m)
const WARP = 0.3;        // m, long-wavelength error amplitude

export class OrbitalDEM {
  constructor(terrain, seed = 7331) {
    this.half = WORLD.size / 2;
    this.n = Math.floor(WORLD.size / POST);
    this.h = new Float32Array(this.n * this.n);
    const rand = mulberry32(seed);
    const gauss = () => { let u = 0; while (u === 0) u = rand(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rand()); };
    const p1 = rand() * 6.3, p2 = rand() * 6.3;
    for (let j = 0; j < this.n; j++) for (let i = 0; i < this.n; i++) {
      const x0 = -this.half + i * POST, z0 = -this.half + j * POST;
      let s = 0;
      for (const a of [0.25, 0.75]) for (const b of [0.25, 0.75]) s += heightAt(terrain, x0 + a * POST, z0 + b * POST);
      const warp = WARP * Math.sin(x0 / 310 + p1) * Math.cos(z0 / 270 + p2);
      this.h[j * this.n + i] = s / 4 + NOISE * gauss() + warp;
    }
  }
  // bilinear sample at world (x, z); the post value sits at its cell centre
  sample(x, z) {
    const fx = (x + this.half) / POST - 0.5, fz = (z + this.half) / POST - 0.5;
    let i = Math.floor(fx), j = Math.floor(fz);
    if (i < 0 || j < 0 || i >= this.n - 1 || j >= this.n - 1) return NaN;
    const ax = fx - i, az = fz - j, n = this.n, H = this.h;
    return (H[j * n + i] * (1 - ax) + H[j * n + i + 1] * ax) * (1 - az) + (H[(j + 1) * n + i] * (1 - ax) + H[(j + 1) * n + i + 1] * ax) * az;
  }
}

// Correlate the SLAM map around (cx, cz) with the DEM. Returns { ok, dx, dz, ncc, second, relief, n, why }:
// (dx, dz) is the shift to ADD to the estimate (est frame -> DEM / true frame).
export function matchToDEM(slam, dem, cx, cz, opts = {}) {
  // Heights with each side's best-fit plane removed: the patch's overall tilt (which correlates almost equally well a
  // few metres off: raw-height NCC 0.95 vs 0.94 four metres away) is taken out, the bumps and crater shapes at
  // 4-45 m scale stay. Slopes were tried and are too noisy against a 4 m DEM. The plane design matrix depends only on
  // the sample layout, so one projection serves every candidate offset.
  const R = opts.radius ?? 45, step = 2, W = opts.window ?? 14;
  const px = [], pz = [], ph = [];
  for (let dz = -R; dz <= R; dz += step) for (let dx = -R; dx <= R; dx += step) {
    if (dx * dx + dz * dz > R * R) continue;
    const h = slam.heightInterp(cx + dx, cz + dz, 1);
    if (h === h) { px.push(dx); pz.push(dz); ph.push(h); }
  }
  const n = ph.length;
  if (n < 300) return { ok: false, n, why: 'too little mapped ground' };
  // plane fit h = a + b u + c v by least squares: (X^T X)^-1 X^T, X = [1 u v]
  let su = 0, sv = 0, suu = 0, svv = 0, suv = 0;
  for (let i = 0; i < n; i++) { const u = px[i], v = pz[i]; su += u; sv += v; suu += u * u; svv += v * v; suv += u * v; }
  const A = [[n, su, sv], [su, suu, suv], [sv, suv, svv]];
  const det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (Math.abs(det) < 1e-9) return { ok: false, n, why: 'degenerate patch' };
  const inv = [
    [(A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det, (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det, (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det],
    [(A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det, (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det, (A[0][2] * A[1][0] - A[0][0] * A[1][2]) / det],
    [(A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det, (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / det, (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det]];
  const detrend = (q, out) => {
    let b0 = 0, b1 = 0, b2 = 0;
    for (let i = 0; i < n; i++) { b0 += q[i]; b1 += q[i] * px[i]; b2 += q[i] * pz[i]; }
    const a = inv[0][0] * b0 + inv[0][1] * b1 + inv[0][2] * b2, b = inv[1][0] * b0 + inv[1][1] * b1 + inv[1][2] * b2, c = inv[2][0] * b0 + inv[2][1] * b1 + inv[2][2] * b2;
    let ss = 0;
    for (let i = 0; i < n; i++) { const r = q[i] - a - b * px[i] - c * pz[i]; out[i] = r; ss += r * r; }
    return ss;
  };
  const rs = new Float64Array(n), ss = detrend(ph, rs);
  const relief = Math.sqrt(ss / n);          // r.m.s. height about the patch plane (m)
  if (relief < 0.2) return { ok: false, n, relief, why: 'flat ground (no relief to match)' };
  const q = new Float64Array(n), rq = new Float64Array(n);
  const ncc = (ox, oz) => {
    for (let i = 0; i < n; i++) { const v = dem.sample(cx + px[i] + ox, cz + pz[i] + oz); if (v !== v) return -1; q[i] = v; }
    const sbb = detrend(q, rq); let sab = 0;
    for (let i = 0; i < n; i++) sab += rs[i] * rq[i];
    return sbb > 0 ? sab / Math.sqrt(ss * sbb) : -1;
  };
  const grid = [];
  let best = { c: -2, x: 0, z: 0 };
  for (let oz = -W; oz <= W; oz += 1) for (let ox = -W; ox <= W; ox += 1) {
    const c = ncc(ox, oz); grid.push([ox, oz, c]);
    if (c > best.c) best = { c, x: ox, z: oz };
  }
  let second = -2;
  for (const [ox, oz, c] of grid) if ((ox - best.x) ** 2 + (oz - best.z) ** 2 > 16 && c > second) second = c;
  const cx0 = best.x, cz0 = best.z;
  for (let oz = -1; oz <= 1 + 1e-9; oz += 0.25) for (let ox = -1; ox <= 1 + 1e-9; ox += 0.25) {
    const c = ncc(cx0 + ox, cz0 + oz); if (c > best.c) best = { c, x: cx0 + ox, z: cz0 + oz };
  }
  const minC = opts.minNcc ?? 0.8, minGap = opts.minGap ?? 0.03;
  const gapReq = best.c > 0.88 ? Math.min(minGap, 0.015) : minGap;
  const ok = best.c > minC && best.c - second > gapReq;
  let sdy = 0, ndy = 0;
  if (ok) {
    for (let i = 0; i < n; i++) {
      const v = dem.sample(cx + px[i] + best.x, cz + pz[i] + best.z);
      if (v === v) { sdy += v - ph[i]; ndy++; }
    }
  }
  const dy = ndy ? sdy / ndy : 0;
  return { ok, dx: best.x, dz: best.z, dy, ncc: best.c, second, relief, n, why: ok ? '' : best.c <= minC ? 'weak match' : 'ambiguous match' };
}
