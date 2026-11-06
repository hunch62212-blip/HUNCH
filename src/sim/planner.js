// Navigation: global A* on a 2 m costmap derived from the SLAM elevation map, and a
// footprint-aware local planner that rolls out constant-curvature arcs over the 0.5 m map.
import { WORLD, ROVER } from './config.js';
import { RES, TILE, CLS } from './slam.js';

const PRES = 2;                                   // planning cell size (m)
const PN = WORLD.size / PRES;                     // 1024
const HALF = WORLD.size / 2;
export const LETHAL = 1e9;

export class Planner {
  constructor(sim, slam) {
    this.sim = sim; this.slam = slam;
    this.cost = new Float32Array(PN * PN);
    this.g = new Float32Array(PN * PN);
    this.parent = new Int32Array(PN * PN);
    this.stamp = new Uint32Array(PN * PN);
    this.closed = new Uint32Array(PN * PN);
    this.search = 0;
    this.unknownCost = 1.5;
    this.routeBonus = 0.5;
    this.inflate = 5.8;   // m, hard inflation (AEGIS-V1 half width 4.9 m + margin)
    this.soft = 15;       // m, soft cost falloff
    this.builtVersion = -1;
    this.lastPlanMs = 0; this.expanded = 0;
    this.blocked = [];    // operator/fault-reported blocked spots {x,z,r}
    this.useRestricted = true;
  }

  idx(x, z) { const i = Math.floor((x + HALF) / PRES), j = Math.floor((z + HALF) / PRES); if (i < 0 || j < 0 || i >= PN || j >= PN) return -1; return j * PN + i; }
  center(k) { return { x: (k % PN) * PRES + PRES / 2 - HALF, z: Math.floor(k / PN) * PRES + PRES / 2 - HALF }; }

  buildCostmap(routes = []) {
    const t0 = performance.now();
    const C = this.cost;
    C.fill(this.unknownCost);
    const lethal = [];
    const sub = PRES / RES; // 4
    const perTile = TILE / sub; // 16 planning cells per tile side
    for (const t of this.slam.tiles.values()) {
      if (t.dirty) this.slam.computeHazard(t);
      const ox = t.tx * TILE * RES, oz = t.tz * TILE * RES;
      for (let pj = 0; pj < perTile; pj++) for (let pi = 0; pi < perTile; pi++) {
        let known = 0, sum = 0, worst = 0;
        for (let a = 0; a < sub; a++) for (let b = 0; b < sub; b++) {
          const i = (pj * sub + a) * TILE + (pi * sub + b);
          if (!t.n[i]) continue;
          known++; const h = t.haz[i]; sum += h; if (h > worst) worst = h;
        }
        if (!known) continue;
        const k = this.idx(ox + (pi + 0.5) * PRES, oz + (pj + 0.5) * PRES);
        if (k < 0) continue;
        if (worst >= 255 && known >= 2) { C[k] = LETHAL; lethal.push(k); continue; }
        const frac = known / (sub * sub);
        const c = Math.max(1, (sum / known) / 20) + (worst > 120 ? 2 : 0);
        C[k] = frac * c + (1 - frac) * this.unknownCost;
      }
    }
    // known infrastructure (from the base database, not terrain): lander, building, restricted zones
    const stampDisk = (x, z, r, val) => {
      const n = Math.ceil(r / PRES);
      const c0 = this.idx(x, z); if (c0 < 0) return;
      const ci = c0 % PN, cj = Math.floor(c0 / PN);
      for (let dj = -n; dj <= n; dj++) for (let di = -n; di <= n; di++) {
        if (di * di + dj * dj > (r / PRES) ** 2) continue;
        const i = ci + di, j = cj + dj; if (i < 0 || j < 0 || i >= PN || j >= PN) continue;
        const k = j * PN + i;
        if (val >= LETHAL) { if (C[k] < LETHAL) { C[k] = LETHAL; lethal.push(k); } }
        else C[k] = Math.max(C[k], val);
      }
    };
    stampDisk(WORLD.pad.x, WORLD.pad.z, 8.5, LETHAL);
    const H = this.sim.terrain.hab;
    for (let s = -16; s <= 16; s += 2) for (let d = 0; d <= 16; d += 2) stampDisk(H.face.x - H.nx * d + H.tx * s, H.face.z - H.nz * d + H.tz * s, 2, LETHAL);
    if (this.useRestricted) for (const z of this.sim.terrain.restricted) stampDisk(z.x, z.z, z.r, LETHAL);
    const now = this.sim.time;
    this.blocked = this.blocked.filter((b) => b.fault || now - b.t < (b.ttl ?? 150));
    for (const b of this.blocked) stampDisk(b.x, b.z, b.r, LETHAL);
    // world boundary
    for (let i = 0; i < PN; i++) { C[i] = C[(PN - 1) * PN + i] = C[i * PN] = C[i * PN + PN - 1] = LETHAL; }
    // inflation: hard core + soft exponential skirt
    const R1 = this.inflate / PRES, R2 = this.soft / PRES, n2 = Math.ceil(R2);
    const kern = [];
    for (let dj = -n2; dj <= n2; dj++) for (let di = -n2; di <= n2; di++) {
      const d = Math.hypot(di, dj);
      if (d > R2) continue;
      kern.push(di, dj, d <= R1 ? LETHAL * 0.5 : 6 * Math.exp(-(d - R1) / 1.6));
    }
    const infl = new Float32Array(PN * PN);
    for (const k of lethal) {
      const ci = k % PN, cj = (k / PN) | 0;
      for (let q = 0; q < kern.length; q += 3) {
        const i = ci + kern[q], j = cj + kern[q + 1];
        if (i < 0 || j < 0 || i >= PN || j >= PN) continue;
        const kk = j * PN + i;
        if (kern[q + 2] > infl[kk]) infl[kk] = kern[q + 2];
      }
    }
    for (let k = 0; k < C.length; k++) {
      if (C[k] >= LETHAL) continue;
      if (infl[k] >= LETHAL * 0.5) C[k] = LETHAL * 0.5; else C[k] += infl[k];
    }
    // teach-and-repeat prior: cells on proven routes are cheaper
    for (const r of routes) {
      for (const p of r.pts) {
        const k = this.idx(p.x, p.z); if (k < 0) continue;
        for (const dk of [0, 1, -1, PN, -PN]) { const kk = k + dk; if (C[kk] < LETHAL * 0.5) C[kk] = Math.max(0.45, C[kk] * this.routeBonus); }
      }
    }
    this.lethalCount = lethal.length;
    this.buildMs = performance.now() - t0;
  }

  passable(k, startK, escapeR2) {
    const c = this.cost[k];
    if (c < LETHAL * 0.5) return c;
    // Around the start the rover is physically standing over whatever is mapped there
    // (it straddles its cargo and saddles), so obstacles are passable at a high cost.
    if (escapeR2) {
      const a = this.center(k), b = this.center(startK);
      if ((a.x - b.x) ** 2 + (a.z - b.z) ** 2 < escapeR2) return c >= LETHAL ? 60 : 25;
    }
    return -1;
  }

  plan(sx, sz, gx, gz, opts = {}) {
    const t0 = performance.now();
    let s = this.idx(sx, sz), goal = this.idx(gx, gz);
    if (s < 0 || goal < 0) return null;
    // goal inside an obstacle: pick the nearest acceptable cell
    if (this.cost[goal] >= LETHAL * 0.5 && !opts.exactGoal) {
      let best = -1, bd = Infinity;
      const gi = goal % PN, gj = (goal / PN) | 0;
      for (let r = 1; r < 15 && best < 0; r++) for (let dj = -r; dj <= r; dj++) for (let di = -r; di <= r; di++) {
        const k = (gj + dj) * PN + gi + di; if (k < 0 || k >= PN * PN) continue;
        if (this.cost[k] < LETHAL * 0.5) { const d = di * di + dj * dj; if (d < bd) { bd = d; best = k; } }
      }
      if (best < 0) return null; goal = best;
    }
    const escapeR2 = (opts.escape ?? 14) ** 2;
    const stamp = ++this.search;
    const g = this.g, P = this.parent, ST = this.stamp, CL = this.closed, C = this.cost;
    // binary heap
    let heapK = new Int32Array(1 << 16), heapF = new Float32Array(1 << 16), hn = 0;
    const push = (k, f) => {
      if (hn >= heapK.length) { const nk = new Int32Array(heapK.length * 2); nk.set(heapK); heapK = nk; const nf = new Float32Array(heapF.length * 2); nf.set(heapF); heapF = nf; }
      let i = hn++; heapK[i] = k; heapF[i] = f;
      while (i > 0) { const p = (i - 1) >> 1; if (heapF[p] <= heapF[i]) break; [heapK[p], heapK[i]] = [heapK[i], heapK[p]]; [heapF[p], heapF[i]] = [heapF[i], heapF[p]]; i = p; }
    };
    const pop = () => {
      const k = heapK[0]; hn--; heapK[0] = heapK[hn]; heapF[0] = heapF[hn];
      let i = 0;
      for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; if (l < hn && heapF[l] < heapF[m]) m = l; if (r < hn && heapF[r] < heapF[m]) m = r; if (m === i) break; [heapK[m], heapK[i]] = [heapK[i], heapK[m]]; [heapF[m], heapF[i]] = [heapF[i], heapF[m]]; i = m; }
      return k;
    };
    const gi = goal % PN, gj = (goal / PN) | 0;
    const hw = opts.heuristicWeight ?? 1.6; // weighted A*: unknown terrain costs 1.5/m, so this stays near-optimal but fast
    const H = (k) => { const di = Math.abs(k % PN - gi), dj = Math.abs(((k / PN) | 0) - gj); return hw * PRES * (Math.max(di, dj) + 0.4142 * Math.min(di, dj)); };
    g[s] = 0; ST[s] = stamp; P[s] = -1; push(s, H(s));
    const nb = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, 1.4142], [1, -1, 1.4142], [-1, 1, 1.4142], [-1, -1, 1.4142]];
    let expanded = 0, found = false;
    const maxExp = opts.maxExpand ?? 1100000;
    while (hn > 0) {
      const k = pop();
      if (CL[k] === stamp) continue;
      CL[k] = stamp; expanded++;
      if (k === goal) { found = true; break; }
      if (expanded > maxExp) break;
      const ki = k % PN, kj = (k / PN) | 0;
      const ck = this.passable(k, s, escapeR2);
      for (const [di, dj, w] of nb) {
        const i = ki + di, j = kj + dj; if (i < 0 || j < 0 || i >= PN || j >= PN) continue;
        const nk = j * PN + i;
        if (CL[nk] === stamp) continue;
        const cn = this.passable(nk, s, escapeR2);
        if (cn < 0) continue;
        const ng = g[k] + PRES * w * 0.5 * (Math.max(0.45, ck) + cn);
        if (ST[nk] !== stamp || ng < g[nk]) { ST[nk] = stamp; g[nk] = ng; P[nk] = k; push(nk, ng + H(nk)); }
      }
    }
    this.expanded = expanded;
    this.lastPlanMs = performance.now() - t0;
    if (!found) return null;
    const cells = [];
    for (let k = goal; k !== -1; k = P[k]) cells.push(k);
    cells.reverse();
    let pts = cells.map((k) => this.center(k));
    pts[0] = { x: sx, z: sz };
    pts[pts.length - 1] = opts.exactGoal || this.cost[this.idx(gx, gz)] < LETHAL * 0.5 ? { x: gx, z: gz } : pts[pts.length - 1];
    pts = this.smooth(pts);
    return { pts: this.resample(pts, 1.0), cost: g[goal], expanded, ms: this.lastPlanMs };
  }

  segmentCost(a, b) {
    const d = Math.hypot(b.x - a.x, b.z - a.z), n = Math.max(1, Math.ceil(d / 1));
    let sum = 0;
    for (let i = 0; i <= n; i++) {
      const k = this.idx(a.x + (b.x - a.x) * i / n, a.z + (b.z - a.z) * i / n);
      if (k < 0) return Infinity;
      const c = this.cost[k]; if (c >= LETHAL * 0.5) return Infinity; sum += c;
    }
    return sum / (n + 1) * d;
  }
  // string pulling, accepting a shortcut only if it is not costlier than the grid path
  smooth(pts) {
    if (pts.length < 3) return pts;
    const out = [pts[0]];
    let i = 0;
    while (i < pts.length - 1) {
      let j = Math.min(pts.length - 1, i + 40);
      for (; j > i + 1; j--) {
        let along = 0; for (let k = i; k < j; k++) along += this.segmentCost(pts[k], pts[k + 1]);
        const direct = this.segmentCost(pts[i], pts[j]);
        if (direct <= along * 1.02) break;
      }
      out.push(pts[j]); i = j;
    }
    return out;
  }
  resample(pts, step) {
    const out = [pts[0]];
    for (let i = 1; i < pts.length; i++) {
      const a = out[out.length - 1], b = pts[i];
      let d = Math.hypot(b.x - a.x, b.z - a.z);
      let ax = a.x, az = a.z;
      while (d > step) { ax += (b.x - ax) * step / d; az += (b.z - az) * step / d; out.push({ x: ax, z: az }); d = Math.hypot(b.x - ax, b.z - az); }
    }
    out.push(pts[pts.length - 1]);
    return out;
  }
  // cost of following pts[from..] on the current costmap (Infinity if it crosses an obstacle)
  pathCost(pts, from = 0) {
    let c = 0;
    for (let i = Math.max(0, from); i < pts.length - 1; i++) { const sc = this.segmentCost(pts[i], pts[i + 1]); if (!Number.isFinite(sc)) return Infinity; c += sc; }
    return c;
  }
  pathBlocked(pts, from = 0, horizon = 400) {
    for (let i = from; i < Math.min(pts.length, from + horizon); i++) {
      const k = this.idx(pts[i].x, pts[i].z);
      if (k < 0 || this.cost[k] >= LETHAL) return i;
    }
    return -1;
  }
}

// ---------------- local planner -------------------------------------------------
// wheel contact stations (each 1.22 m wide tyre sampled at its centre and both edges) and belly points
const WHEELS = ROVER.corners.flatMap(([x, z]) => [[x, z], [x, z - 0.5], [x, z + 0.5]]);
const BELLY = [[4.2, 0], [2.5, -1.8], [2.5, 1.8], [0, -1.8], [0, 1.8], [0, 0], [-2.5, -1.8], [-2.5, 1.8], [-4.2, 0]];

export class LocalPlanner {
  constructor(slam) { this.slam = slam; this.prevK = 0; this.last = null; }

  // Rolls out constant-curvature arcs (forward, or reverse when dir = -1) and checks every
  // wheel contact patch plus the straddle-bay belly clearance along each one.
  // Hazards already under the vehicle's current footprint are ignored (it is standing there).
  evaluate(pose, target, vDes, loaded, opts = {}) {
    const S = this.slam;
    const dir = opts.dir ?? 1;
    const belly = opts.belly ?? (loaded ? 1.5 : 0.75); // tub underside clearance (carry ride raises it 0.82 m)
    const kmax = opts.kmax ?? (vDes < 0.6 ? 0.16 : 0.11);
    const nK = 14;
    const ks = []; for (let i = -nK; i <= nK; i++) ks.push(kmax * i / nK);
    const horizon = opts.horizon ?? Math.max(9, Math.min(18, Math.abs(vDes) * 10));
    const c0 = Math.cos(pose.yaw), s0 = Math.sin(pose.yaw);
    const here = WHEELS.map(([wx, wz]) => [pose.x + wx * c0 + wz * s0, pose.z - wx * s0 + wz * c0]);
    const underUs = (px, pz) => { for (const [hx, hz] of here) if ((px - hx) ** 2 + (pz - hz) ** 2 < 1.4 * 1.4) return true; return false; };
    let best = null;
    const arcs = [];
    for (const k of ks) {
      let x = pose.x, z = pose.z, yaw = pose.yaw;
      let hz = 0, freeDist = horizon, unknown = 0, ok = true, hitPt = null, knownDist = horizon;
      const pts = [];
      for (let s = 0.5; s <= horizon + 1e-6; s += 0.5) {
        yaw += k * 0.5 * dir;
        x += Math.cos(yaw) * 0.5 * dir; z += -Math.sin(yaw) * 0.5 * dir;
        if ((s * 2) % 2 === 0) pts.push({ x, z });
        const c = Math.cos(yaw), sn = Math.sin(yaw);
        let ground = 0, gk = 0, hit = false;
        for (const [wx, wz] of WHEELS) {
          const px = x + wx * c + wz * sn, pz = z - wx * sn + wz * c;
          const wh = S.wheelHazard(px, pz, opts.relaxed);
          if (!wh) { unknown++; if (knownDist === horizon && !underUs(px, pz)) knownDist = s - 0.5; continue; }
          ground += wh.h; gk++;
          if (wh.lethal && !underUs(px, pz)) { hit = true; hitPt = { x: px, z: pz }; break; }
          hz += wh.cost;
        }
        if (!hit && gk >= 3 && Math.abs(s - Math.round(s)) < 1e-6) {
          ground /= gk;
          for (const [bx, bz] of BELLY) {
            const qx = x + bx * c + bz * sn, qz = z - bx * sn + bz * c;
            const h = S.heightAt(qx, qz);
            if (h === h && h - ground > belly && !underUs(qx, qz)) { hit = true; hitPt = { x: qx, z: qz }; break; }
          }
        }
        if (hit) { freeDist = s - 0.5; ok = freeDist >= Math.min(6, horizon * 0.5); break; }
      }
      const ex = x - target.x, ez = z - target.z;
      const fwdYaw = dir > 0 ? yaw : yaw + Math.PI;
      let dh = Math.atan2(-(target.z - z), target.x - x) - fwdYaw; dh = Math.atan2(Math.sin(dh), Math.cos(dh));
      const J = 1.6 * Math.hypot(ex, ez) + 2.0 * Math.abs(dh) + 0.03 * hz + 6 * Math.abs(k) + 18 * Math.abs(k - this.prevK)
        + 0.02 * unknown + (freeDist < horizon ? (horizon - freeDist) * 2.5 : 0);
      arcs.push({ k, ok, freeDist, pts, hitPt, knownDist });
      if (ok && (!best || J < best.J)) best = { J, k, freeDist, knownDist };
    }
    const hits = arcs.filter((a) => a.hitPt).map((a) => a.hitPt);
    if (!best) { this.last = { ok: false, arcs, hits, horizon }; return this.last; }
    if (dir > 0) this.prevK = best.k;
    this.last = { ok: true, k: best.k, freeDist: best.freeDist, knownDist: Math.min(best.knownDist, best.freeDist), arcs, hits, horizon };
    return this.last;
  }

  pivotSafe(pose) {
    // the wheels sweep circles of radius ~5.3 m and ~7.3 m about the centre
    const S = this.slam;
    for (let a = 0; a < Math.PI * 2; a += 0.1) for (const r of [3.84, 4.6, 5.2, 5.9]) {
      const wh = S.wheelHazard(pose.x + Math.cos(a) * r, pose.z + Math.sin(a) * r);
      if (wh && wh.lethal) return false;
    }
    return true;
  }
}
