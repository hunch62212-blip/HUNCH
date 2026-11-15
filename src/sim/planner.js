// Navigation: global A* on a 2 m costmap derived from the SLAM elevation map, and a
// footprint-aware local planner that rolls out constant-curvature arcs over the 0.5 m map.
import { WORLD, ROVER, NAV } from './config.js';
import { RES, TILE, CLS } from './slam.js';

const PRES = 2;                                   // planning cell size (m)
const PN = WORLD.size / PRES;                     // 1250 cells per side on the 2.5 km map
const HALF = WORLD.size / 2;
export const LETHAL = 1e9;

export class Planner {
  constructor(sim, slam) {
    this.sim = sim; this.slam = slam;
    this.cost = new Float32Array(PN * PN);
    this.hardLethal = new Uint8Array(PN * PN);
    this.g = new Float32Array(PN * PN);
    this.parent = new Int32Array(PN * PN);
    this.stamp = new Uint32Array(PN * PN);
    this.closed = new Uint32Array(PN * PN);
    this.search = 0;
    this.unknownCost = 1.5;          // floor of the price of unmapped ground (per m)
    // v1.5.0: unmapped ground is priced at what the mapped ground has turned out to cost (median of the known cells,
    // smoothed across builds, clamped 1.5..6). With a fixed 1.5 /m it was ~5x cheaper than mapped ground (median 6-8 /m:
    // most real ground sits in some rock's soft inflation skirt), so every scan made the route ahead dearer than the
    // still-unmapped alternatives and A* kept swinging the route sideways or back ("changing its mind").
    this.unknownAdaptive = null; this.unkEff = this.unknownCost;
    // CHANGES.md §17: unmapped ground within unknownHazardRadius of mapped SLOPE / CRATER ground is a likely rim or pocket
    // (A* "explore" at 1.5 /m routed the loaded rover straight into such pockets and it could not turn round)
    this.unknownNearHazardCost = 12.0;
    this.unknownHazardRadius = 15;   // m
    this.loadedMinRadius = 18;       // m, smallest turning radius the planned centre line may have while carrying a module
    this.knownMask = new Uint8Array(PN * PN);
    this.lastSmooth = { corners: 0, forced: 0, filleted: 0, unfilleted: 0 };
    this.routeBonus = 0.5;
    this.inflate = NAV.inflateEmpty;   // m, hard inflation (AEGIS-V1 half width 4.9 m + margin); navTo sets NAV.inflateLoaded (6.5) while loaded
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
    const C = this.cost, HL = this.hardLethal;
    HL.fill(0);
    const unk = this.unkEff = Math.max(this.unknownCost, Math.min(6, this.unknownAdaptive ?? this.unknownCost));
    C.fill(unk);
    const lethal = [];
    const sub = PRES / RES; // 4
    const perTile = TILE / sub; // 16 planning cells per tile side
    const KM = this.knownMask; KM.fill(0);
    const hazK = [];        // planning cells that hold mapped CLS.SLOPE / CLS.CRATER ground
    const partial = [];     // k, frac, c of partly mapped cells: finished below, once the near-hazard mask exists
    for (const t of this.slam.tiles.values()) {
      if (t.dirty) this.slam.computeHazard(t);
      const ox = t.tx * TILE * RES, oz = t.tz * TILE * RES;
      for (let pj = 0; pj < perTile; pj++) for (let pi = 0; pi < perTile; pi++) {
        let known = 0, sum = 0, worst = 0, hz = false;
        for (let a = 0; a < sub; a++) for (let b = 0; b < sub; b++) {
          const i = (pj * sub + a) * TILE + (pi * sub + b);
          if (!t.n[i]) continue;
          known++; const h = t.haz[i]; sum += h; if (h > worst) worst = h;
          const cl = t.cls[i]; if (cl === CLS.SLOPE || cl === CLS.CRATER) hz = true;
        }
        if (!known) continue;
        const k = this.idx(ox + (pi + 0.5) * PRES, oz + (pj + 0.5) * PRES);
        if (k < 0) continue;
        KM[k] = 1;
        if (hz) hazK.push(k);
        if (worst >= 255 && known >= 2) { C[k] = LETHAL; lethal.push(k); continue; }
        const frac = known / (sub * sub);
        const c = Math.max(1, (sum / known) / 20) + (worst > 120 ? 2 : 0);
        C[k] = frac * c + (1 - frac) * unk;
        if (frac < 1) partial.push(k, frac, c);
      }
    }
    // unmapped ground next to mapped slopes / craters costs 12 /m instead of 1.5 /m (CHANGES.md §17)
    this.unknownInflated = 0;
    if (hazK.length && this.unknownNearHazardCost > unk) {
      const near = this.nearMask(hazK, this.unknownHazardRadius), uh = Math.max(this.unknownNearHazardCost, unk * 1.5);
      for (let k = 0; k < C.length; k++) if (near[k] && !KM[k]) { C[k] = uh; this.unknownInflated++; }
      for (let q = 0; q < partial.length; q += 3) { const k = partial[q]; if (near[k]) C[k] = partial[q + 1] * partial[q + 2] + (1 - partial[q + 1]) * uh; }
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
        if (val >= LETHAL) { if (C[k] < LETHAL) { C[k] = LETHAL; lethal.push(k); } HL[k] = 1; }
        else C[k] = Math.max(C[k], val);
      }
    };
    const cg = this.sim.rover && this.sim.rover.lift.cargo;
    const cgHalf = cg ? Math.max(cg.T.adapterR || 0, cg.T.shape === 'cylinder' ? cg.T.radius : cg.T.width / 2) : 0;
    const tall = cg ? Math.max(0, Math.hypot(cg.T.length / 2 + Math.abs(cg.T.carryX || 0), cgHalf) + 0.5 - this.inflate) : 0;
    // (thin posts are also checked against the rover's actual rectangle by the local planner: LocalPlanner.posts, v1.7.0. A wider
    // disc here was tried and dropped: it made the pad-apron masts close the start of proven loaded routes.)
    const post = tall;
    stampDisk(WORLD.pad.x, WORLD.pad.z, Math.max(19.2, 12.0 + tall), LETHAL);
    const H = this.sim.terrain.hab;
    for (let s = -16; s <= 16; s += 2) for (let d = 0; d <= 16; d += 2) stampDisk(H.face.x - H.nx * d + H.tx * s, H.face.z - H.nz * d + H.tz * s, 2, LETHAL);
    // surveyed base structures (terrain.baseStructures, the same list that builds their colliders) and the
    // power dock. Thin poles (apron light masts, array posts: r 0.2 m) are easy for the LiDAR height map to miss
    // and an 8 m wide carried module clips them; mapped-in keep-outs stop the planner routing past them.
    // a carried module (6 m up, longer and wider than the deck) sweeps beyond the rover's own inflation radius;
    // only tall structures reach it, so they alone get the extra margin
    const line = (x, z, ax, az, len, r) => { const n = Math.max(1, Math.ceil(len / r)); for (let i = 0; i <= n; i++) { const s = -len / 2 + len * i / n; stampDisk(x + ax * s, z + az * s, r, LETHAL); } };
    for (const it of this.sim.terrain.base || []) {
      if (it.kind === 'dome') stampDisk(it.x, it.z, it.r, LETHAL);
      else if (it.kind === 'mast' || it.kind === 'light') stampDisk(it.x, it.z, it.r + post, LETHAL);
      else if (it.kind === 'array') { stampDisk(it.x, it.z, 0.2 + post, LETHAL); line(it.x, it.z, 1, 0, it.w, 0.5 + tall); } // post + panel (world-x aligned, as world.js)
      else if (it.len) line(it.x, it.z, Math.cos(it.yaw), -Math.sin(it.yaw), it.len, it.r); // tunnel / modules (horizontal cylinders)
    }
    const PD = this.sim.powerDock && this.sim.powerDock.layout;
    if (PD) {
      for (const a of PD.arrays) stampDisk(a.x, a.z, 0.45 + post, LETHAL);
      const b = PD.bank; // bank box: depth along the face normal, width along the face
      line(b.x, b.z, H.tx, H.tz, b.w, b.d / 2);
    }
    if (this.useRestricted) for (const z of this.sim.terrain.restricted) stampDisk(z.x, z.z, z.r, LETHAL);
    const now = this.sim.time;
    this.blocked = this.blocked.filter((b) => b.fault || now - b.t < (b.ttl ?? 150));
    for (const b of this.blocked) stampDisk(b.x, b.z, b.r, LETHAL);
    // world boundary
    for (let i = 0; i < PN; i++) {
      C[i] = C[(PN - 1) * PN + i] = C[i * PN] = C[i * PN + PN - 1] = LETHAL;
      HL[i] = HL[(PN - 1) * PN + i] = HL[i * PN] = HL[i * PN + PN - 1] = 1;
    }
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
    // what mapped, passable ground costs (sampled median), for the next build's unknown-ground price
    { const KMk = this.knownMask, smp = [];
      for (let k = 0; k < C.length; k += 3) if (KMk[k] && C[k] < LETHAL * 0.5) smp.push(C[k]);
      if (smp.length > 200) {
        smp.sort((a, b) => a - b);
        const med = smp[smp.length >> 1];
        this.unknownAdaptive = this.unknownAdaptive == null ? med : this.unknownAdaptive + (med - this.unknownAdaptive) * 0.25;
      } }
    // teach-and-repeat prior: cells on proven routes are cheaper
    for (const r of routes) {
      for (const p of r.pts) {
        const k = this.idx(p.x, p.z); if (k < 0) continue;
        for (const dk of [0, 1, -1, PN, -PN]) { const kk = k + dk; if (kk >= 0 && kk < C.length && C[kk] < LETHAL * 0.5) C[kk] = Math.max(0.45, C[kk] * this.routeBonus); }
      }
    }
    this.lethalCount = lethal.length;
    this.buildMs = performance.now() - t0;
  }

  // 1 where a planning cell lies within radiusM of any listed cell (circular dilation, one run per row)
  nearMask(cells, radiusM) {
    const rc = radiusM / PRES, n = Math.floor(rc), W = PN + 1;
    const diff = new Int32Array(PN * W);
    for (const k of cells) {
      const ci = k % PN, cj = (k / PN) | 0;
      for (let dj = -n; dj <= n; dj++) {
        const j = cj + dj; if (j < 0 || j >= PN) continue;
        const w = Math.floor(Math.sqrt(rc * rc - dj * dj));
        diff[j * W + Math.max(0, ci - w)]++; diff[j * W + Math.min(PN - 1, ci + w) + 1]--;
      }
    }
    const mask = new Uint8Array(PN * PN);
    for (let j = 0; j < PN; j++) { let run = 0; for (let i = 0; i < PN; i++) { run += diff[j * W + i]; if (run > 0) mask[j * PN + i] = 1; } }
    return mask;
  }

  passable(k, startK, escapeR2) {
    const c = this.cost[k];
    if (c < LETHAL * 0.5) return c;
    // Around the start the rover is physically standing over whatever is mapped there
    // (it straddles its cargo and saddles), so obstacles are passable at a high cost.
    if (escapeR2) {
      const a = this.center(k), b = this.center(startK);
      // Only traverse LETHAL cells when actually starting inside LETHAL terrain; inflated 0.5*LETHAL margins must step out, not shortcut through 25-deg craters.
      if ((a.x - b.x) ** 2 + (a.z - b.z) ** 2 < escapeR2) return c >= LETHAL ? (this.cost[startK] >= LETHAL ? 60 : 500) : 25;
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
        const i = gi + di, j = gj + dj; if (i < 0 || j < 0 || i >= PN || j >= PN) continue; // (was an index-only check: a goal near the map edge wrapped onto the opposite side of the next row)
        const k = j * PN + i;
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
    const hw = opts.heuristicWeight ?? 1.07 * this.unkEff; // weighted A*, scaled to the price of unmapped ground (most of a long route)
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
  // Can a corner at p1 (path p0 -> p1 -> p2) be rounded with an arc of radius R? The arc's tangent length is
  // R tan(theta/2) and may use at most half of each adjacent segment (the neighbouring corners share the other half);
  // the first and last segment give all of theirs.
  cornerOK(p0, p1, p2, R, first = false, last = false) {
    const ax = p1.x - p0.x, az = p1.z - p0.z, bx = p2.x - p1.x, bz = p2.z - p1.z;
    const la = Math.hypot(ax, az), lb = Math.hypot(bx, bz);
    if (la < 1e-6 || lb < 1e-6) return true;
    const th = Math.acos(Math.max(-1, Math.min(1, (ax * bx + az * bz) / (la * lb))));
    if (th < 0.02) return true;
    const T = R * Math.tan(th / 2);
    return T <= (first ? la : la / 2) && T <= (last ? lb : lb / 2);
  }
  // string pulling, accepting a shortcut only if it is not costlier than the grid path. Loaded (CHANGES.md §17): the
  // carried module's CoM is 6.8 m up and the skid-steer cannot point-turn, so a shortcut is also refused when the
  // corner it makes with the previous segment cannot be rounded at loadedMinRadius; after pulling, the corners are
  // replaced by arcs (fillet).
  smooth(pts) {
    if (pts.length < 3) return pts;
    const R = this.sim.rover && this.sim.rover.loaded ? this.loadedMinRadius : 0;
    this.lastSmooth = { corners: 0, forced: 0, filleted: 0, unfilleted: 0 };
    const out = [pts[0]];
    let i = 0;
    while (i < pts.length - 1) {
      let j = Math.min(pts.length - 1, i + 40);
      for (; j > i + 1; j--) {
        let along = 0; for (let k = i; k < j; k++) along += this.segmentCost(pts[k], pts[k + 1]);
        const direct = this.segmentCost(pts[i], pts[j]);
        if (direct > along * 1.02) continue;
        if (R && out.length >= 2 && !this.cornerOK(out[out.length - 2], pts[i], pts[j], R, out.length === 2, j === pts.length - 1)) continue;
        break;
      }
      if (R && j === i + 1 && out.length >= 2 && !this.cornerOK(out[out.length - 2], pts[i], pts[j], R, out.length === 2, j === pts.length - 1)) this.lastSmooth.forced++;
      out.push(pts[j]); i = j;
    }
    return R ? this.fillet(out, R) : out;
  }
  // Replace each corner by a circular arc of radius R where the tangent length fits; a corner that does not fit (or
  // whose arc would leave the mapped free space) stays a sharp vertex and is counted in lastSmooth.unfilleted
  fillet(pts, R) {
    if (pts.length < 3) return pts;
    const out = [pts[0]], L = this.lastSmooth;
    for (let v = 1; v < pts.length - 1; v++) {
      const p0 = pts[v - 1], p1 = pts[v], p2 = pts[v + 1];
      const ax = p1.x - p0.x, az = p1.z - p0.z, bx = p2.x - p1.x, bz = p2.z - p1.z;
      const la = Math.hypot(ax, az), lb = Math.hypot(bx, bz);
      const th = la > 1e-6 && lb > 1e-6 ? Math.acos(Math.max(-1, Math.min(1, (ax * bx + az * bz) / (la * lb)))) : 0;
      if (th < 0.02) { out.push(p1); continue; }
      L.corners++;
      const T = R * Math.tan(th / 2);
      const prev = out[out.length - 1];
      const room = Math.min(v === 1 ? la : la / 2, v === pts.length - 2 ? lb : lb / 2, Math.hypot(p1.x - prev.x, p1.z - prev.z));
      if (T > room + 1e-6) { out.push(p1); L.unfilleted++; continue; }
      const ua = { x: ax / la, z: az / la }, ub = { x: bx / lb, z: bz / lb };
      const t1 = { x: p1.x - ua.x * T, z: p1.z - ua.z * T }, t2 = { x: p1.x + ub.x * T, z: p1.z + ub.z * T };
      const sgn = Math.sign(ua.x * ub.z - ua.z * ub.x) || 1;
      const cx = t1.x - sgn * R * ua.z, cz = t1.z + sgn * R * ua.x; // centre: t1 + R x the normal of ua that points into the turn
      const a1 = Math.atan2(t1.z - cz, t1.x - cx), nArc = Math.max(2, Math.ceil(R * th / 0.25));
      const arc = [t1];
      for (let q = 1; q < nArc; q++) { const a = a1 + sgn * th * q / nArc; arc.push({ x: cx + R * Math.cos(a), z: cz + R * Math.sin(a) }); }
      arc.push(t2);
      // the arc cuts the inside of the corner, which the cost check of the chords did not see
      const clear = arc.every((q) => { const k = this.idx(q.x, q.z); return k >= 0 && this.cost[k] < LETHAL * 0.5; });
      if (!clear) { out.push(p1); L.unfilleted++; continue; }
      for (const q of arc) out.push(q);
      L.filleted++;
    }
    out.push(pts[pts.length - 1]);
    return out;
  }
  // Route straightening (v1.7.0). A route is recorded as driven, so a loaded leg that explored unmapped crater field keeps
  // every probe and detour of that first drive (R1 measured 2.9 km for a 2.0 km straight line) and each later repeat drove
  // them again. Once a route is stored, every stretch whose chord runs over ground that is already mapped, outside the hard
  // obstacle margin, no costlier than the stretch itself, and passes the full wheel + belly footprint check (with this
  // route's load) at every metre is replaced by the chord; corners are then rounded as A* paths are (loaded: R 18 m).
  // Unmapped ground is never used: the result only runs where the rover has seen the ground.
  straighten(pts, loaded) {
    if (!pts || pts.length < 30) return null;
    const R = loaded ? this.loadedMinRadius : 0;
    const mapped = (x, z) => { const k = this.idx(x, z); return k >= 0 && this.knownMask[k] && this.cost[k] < LETHAL * 0.5; };
    const chordOK = (a, b) => {
      const L = Math.hypot(b.x - a.x, b.z - a.z), n = Math.max(1, Math.ceil(L)), yaw = Math.atan2(-(b.z - a.z), b.x - a.x);
      for (let q = 0; q <= n; q++) if (!mapped(a.x + (b.x - a.x) * q / n, a.z + (b.z - a.z) * q / n)) return false;
      for (let q = 0; q <= n; q++) {
        const fp = checkFootprint(this.slam, a.x + (b.x - a.x) * q / n, a.z + (b.z - a.z) * q / n, yaw, loaded, { wheels: WHEEL_CENTERS, convexBelly: true });
        if (fp.hit || fp.unmapped) return false;
      }
      return true;
    };
    const out = [pts[0]];
    let i = 0, cuts = 0;
    while (i < pts.length - 1) {
      let j = Math.min(pts.length - 1, i + 160);
      for (; j > i + 1; j -= j - i > 40 ? 8 : j - i > 12 ? 3 : 1) {
        let along = 0; for (let k = i; k < j; k++) along += this.segmentCost(pts[k], pts[k + 1]);
        const direct = this.segmentCost(pts[i], pts[j]);
        if (!Number.isFinite(direct) || direct > along * 1.02) continue;
        if (R && out.length >= 2 && !this.cornerOK(out[out.length - 2], pts[i], pts[j], R, out.length === 2, j === pts.length - 1)) continue;
        if (!chordOK(pts[i], pts[j])) continue;
        break;
      }
      if (j > i + 1) cuts++;
      out.push(pts[Math.max(j, i + 1)]); i = Math.max(j, i + 1);
    }
    const sm = R ? this.fillet(out, R) : out;
    const res = this.resample(sm, 1.0);
    let len = 0; for (let k = 1; k < res.length; k++) len += Math.hypot(res[k].x - res[k - 1].x, res[k].z - res[k - 1].z);
    return { pts: res, length: len, cuts };
  }
  // smallest circumradius of the path, sampled every `step` m (for the loaded turning-radius check)
  static pathMinRadius(pts, step = 3) {
    let best = Infinity;
    for (let i = 0; i + 2 * step < pts.length; i++) {
      const a = pts[i], b = pts[i + step], c = pts[i + 2 * step];
      const ab = Math.hypot(b.x - a.x, b.z - a.z), bc = Math.hypot(c.x - b.x, c.z - b.z), ca = Math.hypot(a.x - c.x, a.z - c.z);
      const area2 = Math.abs((b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x));
      if (area2 < 1e-9) continue;
      best = Math.min(best, ab * bc * ca / (2 * area2));
    }
    return best;
  }
  resample(pts, step) {
    const cp = (p, x = p.x, z = p.z) => (p.t != null ? { x, z, t: p.t } : { x, z });
    const out = [cp(pts[0])];
    for (let i = 1; i < pts.length; i++) {
      const a = out[out.length - 1], b = pts[i];
      let d = Math.hypot(b.x - a.x, b.z - a.z);
      let ax = a.x, az = a.z;
      while (d > step) { ax += (b.x - ax) * step / d; az += (b.z - az) * step / d; out.push(cp(b, ax, az)); d = Math.hypot(b.x - ax, b.z - az); }
    }
    out.push(cp(pts[pts.length - 1]));
    return out;
  }
  // cost of following pts[from..] on the current costmap (Infinity if it crosses an obstacle)
  pathCost(pts, from = 0) {
    let c = 0;
    for (let i = Math.max(0, from); i < pts.length - 1; i++) { const sc = this.segmentCost(pts[i], pts[i + 1]); if (!Number.isFinite(sc)) return Infinity; c += sc; }
    return c;
  }
  pathBlocked(pts, from = 0, horizon = 400) {
    const S = this.slam, loaded = !!S._payload;
    const end = Math.min(pts.length, from + horizon);
    for (let i = from; i < end; i++) {
      const p = pts[i], k = this.idx(p.x, p.z);
      if (k < 0 || this.hardLethal[k]) return i;
      if (this.cost[k] < LETHAL) continue;
      // Centerline fell inside a terrain-lethal cell (e.g., straddled crater): test the rover's actual wheel tracks and belly.
      const p0 = pts[Math.max(0, i - 1)], p1 = pts[Math.min(pts.length - 1, i + 1)];
      const dx = p1.x - p0.x, dz = p1.z - p0.z;
      const yaw = dx * dx + dz * dz > 1e-6 ? Math.atan2(-dz, dx) : 0;
      const fp = checkFootprint(S, p.x, p.z, yaw, loaded, { wheels: WHEEL_CENTERS, convexBelly: true });
      if (fp.hit || fp.unmapped || fp.gk < 4) return i;
    }
    return -1;
  }
}

// ---------------- local planner -------------------------------------------------
// wheel contact stations (each 1.22 m wide tyre sampled at its centre and both edges) and belly points
const WHEEL_CENTERS = ROVER.corners.map(([x, z]) => [x, z]);
const WHEELS = ROVER.corners.flatMap(([x, z]) => [[x, z], [x, z - 0.5], [x, z + 0.5]]);
const BELLY = [[4.2, 0], [2.5, -1.8], [2.5, 1.8], [0, -1.8], [0, 1.8], [0, 0], [-2.5, -1.8], [-2.5, 1.8], [-4.2, 0]];

/**
 * Checks wheel contact hazards and fitted-plane straddle belly clearance for a pose (x, z, yaw).
 */
function checkFootprint(S, x, z, yaw, loaded, opts = {}) {
  const c = Math.cos(yaw), sn = Math.sin(yaw);
  const belly = opts.belly ?? (loaded ? 1.5 : 0.75);
  const underUs = opts.underUs;
  const wheels = opts.wheels || WHEELS;
  let gk = 0, sx = 0, sz = 0, sh = 0, sxx = 0, szz = 0, sxz = 0, sxh = 0, szh = 0;
  let sf = 0, nf = 0, sm = 0, nm = 0, sr = 0, nr = 0;
  let hit = false, hitPt = null, unmapped = false, unknown = 0, hz = 0;
  for (const [wx, wz] of wheels) {
    const px = x + wx * c + wz * sn, pz = z - wx * sn + wz * c;
    const wh = S.wheelHazard(px, pz, opts.relaxed);
    // (a pinhole between LiDAR rings does not end the mapped distance: it made the rover creep at 0.3 m/s on mapped ground)
    if (!wh) { unknown++; if ((!underUs || !underUs(px, pz)) && !S.pinhole(px, pz)) unmapped = true; continue; }
    gk++; sx += wx; sz += wz; sh += wh.h; sxx += wx * wx; szz += wz * wz; sxz += wx * wz; sxh += wx * wh.h; szh += wz * wh.h;
    if (wx > 2) { sf += wh.h; nf++; } else if (wx < -2) { sr += wh.h; nr++; } else { sm += wh.h; nm++; }
    if (wh.lethal && (!underUs || !underUs(px, pz))) { hit = true; hitPt = { x: px, z: pz }; break; }
    hz += wh.cost;
  }
  if (!hit && gk >= 3 && opts.checkBelly !== false) {
    const mx = sx / gk, mz = sz / gk, mh = sh / gk;
    const cxx = sxx / gk - mx * mx, czz = szz / gk - mz * mz, cxz = sxz / gk - mx * mz;
    const cxh = sxh / gk - mx * mh, czh = szh / gk - mz * mh, det = cxx * czz - cxz * cxz;
    const plane = det > 0.5 ? { gx: (cxh * czz - czh * cxz) / det, gz: (czh * cxx - cxh * cxz) / det } : { gx: 0, gz: 0 };
    const axleOk = nf > 0 && nm > 0 && nr > 0;
    const hf = axleOk ? sf / nf : 0, hm = axleOk ? sm / nm : 0, hr = axleOk ? sr / nr : 0;
    const xSpan = Math.abs(ROVER.corners[0][0]) || 4.2;
    for (const [bx, bz] of BELLY) {
      const qx = x + bx * c + bz * sn, qz = z - bx * sn + bz * c;
      const h = S.heightAt(qx, qz);
      let ground = mh + plane.gx * (bx - mx) + plane.gz * (bz - mz);
      // When validating a proven path over a convex rim crest, middle wheels keep the chassis from sinking below the wheel chord.
      if (opts.convexBelly && axleOk) {
        const t = Math.min(1, Math.abs(bx) / xSpan);
        const chord = bx >= 0 ? hm + (hf - hm) * t : hm + (hr - hm) * t;
        ground = Math.max(ground, chord + plane.gz * (bz - mz));
      }
      if (h === h && h - ground > belly && (!underUs || !underUs(qx, qz))) { hit = true; hitPt = { x: qx, z: qz }; break; }
    }
  }
  return { hit, hitPt, unmapped, unknown, gk, hz };
}

export class LocalPlanner {
  // posts(): surveyed thin structures [{x, z, r}] (light masts, array posts) that the LiDAR height map does not hold;
  // extent(): the rover's half length / half width, grown to the carried module's when loaded. Both set by Autonomy.
  constructor(slam) { this.slam = slam; this.prevK = 0; this.last = null; this.posts = null; this.extent = null; }

  // Rolls out constant-curvature arcs (forward, or reverse when dir = -1) and checks every
  // wheel contact patch plus the straddle-bay belly clearance along each one.
  // Hazards already under the vehicle's current footprint are ignored (it is standing there).
  evaluate(pose, target, vDes, loaded, opts = {}) {
    const S = this.slam;
    const dir = opts.dir ?? 1;
    const belly = opts.belly ?? (loaded ? 1.5 : 0.75); // tub underside clearance (carry ride raises it 0.82 m)
    // gentle skid steer (v1.5.0): the inner side runs at (1 - |k| b) of the centre speed (b = 3.84 m half track). Keep
    // |k| b <= 0.5 slow and <= 0.35 above 3 m/s (inner side >= 50 / 65 % of the outer), so turns scrub less and the
    // rover arcs round a bend instead of skidding into it
    const half = Math.abs(ROVER.corners[0][1]), sideMax = Math.abs(vDes) < 1 ? 0.5 : Math.abs(vDes) > 3 ? 0.35 : 0.5 - 0.075 * (Math.abs(vDes) - 1);
    const kmax = opts.kmax ?? Math.min(Math.abs(vDes) < 0.6 ? 0.16 : 0.11, sideMax / half);
    const nK = 14;
    const ks = []; for (let i = -nK; i <= nK; i++) ks.push(kmax * i / nK);
    const horizon = opts.horizon ?? Math.max(9, Math.min(30, Math.abs(vDes) * 6 + 6)); // (max 18 m capped speed at ~4.2 m/s via the stopping-distance governor)
    const c0 = Math.cos(pose.yaw), s0 = Math.sin(pose.yaw);
    const here = WHEELS.map(([wx, wz]) => [pose.x + wx * c0 + wz * s0, pose.z - wx * s0 + wz * c0]);
    const underUs = (px, pz) => { for (const [hx, hz] of here) if ((px - hx) ** 2 + (pz - hz) ** 2 < 1.4 * 1.4) return true; return false; };
    let best = null;
    const arcs = [];
    // surveyed posts near the arcs (v1.7.0): tested against the rover's rectangle, not the centre disc the global costmap uses. The
    // arcs cut inside the planned line on a bend, and a frame corner (6.95 m from the centre) caught an apron light mast. An arc is
    // refused where a post comes within 0.6 m of the rectangle AND closer than it is now, so a rover already touching one can move off.
    const ext = this.extent ? this.extent() : { hl: ROVER.frame.halfLen, hw: ROVER.frame.halfWid };
    const posts = (this.posts ? this.posts() : []).filter((q) => (q.x - pose.x) ** 2 + (q.z - pose.z) ** 2 < (horizon + ext.hl + 8) ** 2);
    const postGap = (q, x, z, yaw) => { const c = Math.cos(yaw), sn = Math.sin(yaw), dx = q.x - x, dz = q.z - z; return Math.max(Math.abs(dx * c - dz * sn) - ext.hl, Math.abs(dx * sn + dz * c) - ext.hw) - q.r; };
    const gap0 = posts.map((q) => postGap(q, pose.x, pose.z, pose.yaw));
    for (const k of ks) {
      let x = pose.x, z = pose.z, yaw = pose.yaw;
      let hz = 0, freeDist = horizon, unknown = 0, ok = true, hitPt = null, knownDist = horizon;
      const pts = [];
      for (let s = 0.5; s <= horizon + 1e-6; s += 0.5) {
        yaw += k * 0.5 * dir;
        x += Math.cos(yaw) * 0.5 * dir; z += -Math.sin(yaw) * 0.5 * dir;
        if ((s * 2) % 2 === 0) pts.push({ x, z });
        const fp = checkFootprint(S, x, z, yaw, loaded, { belly, relaxed: opts.relaxed, underUs, checkBelly: Math.abs(s - Math.round(s)) < 1e-6 });
        if (!fp.hit) for (let q = 0; q < posts.length; q++) { const g = postGap(posts[q], x, z, yaw); if (g < 0.6 && g < gap0[q] - 0.02) { fp.hit = true; fp.hitPt = { x: posts[q].x, z: posts[q].z }; break; } }
        unknown += fp.unknown; hz += fp.hz;
        if (fp.unmapped && knownDist === horizon) knownDist = s - 0.5;
        if (fp.hit) { hitPt = fp.hitPt; freeDist = s - 0.5; ok = freeDist >= Math.min(6, horizon * 0.5); break; }
      }
      const ex = x - target.x, ez = z - target.z;
      const fwdYaw = dir > 0 ? yaw : yaw + Math.PI;
      let dh = Math.atan2(-(target.z - z), target.x - x) - fwdYaw; dh = Math.atan2(Math.sin(dh), Math.cos(dh));
      const prevW = (this.last && this.last.freeDist != null && this.last.freeDist < 12) ? 4 : 18;
      const obsPen = freeDist < horizon ? (horizon - freeDist) * 2.5 + (freeDist < 12 ? (12 - freeDist) * 5.0 : 0) : 0;
      const J = 1.6 * Math.hypot(ex, ez) + 2.0 * Math.abs(dh) + 0.03 * hz + 6 * Math.abs(k) + prevW * Math.abs(k - this.prevK)
        + 0.02 * unknown + obsPen;
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
