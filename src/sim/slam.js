// LiDAR SLAM: tiled 2.5D elevation map + odometry/IMU dead reckoning + correlative
// scan-to-map matching (x, z, yaw search; vertical offset by robust median), hazard
// classification for traversability, and a teach-and-repeat route memory.
// Nothing here reads ground truth except the initial surveyed landing-site pose.
import * as THREE from 'three';
import { WORLD, ROVER } from './config.js';

export const RES = 0.5;
export const TILE = 64;                 // cells per tile side (32 m)
const TS = TILE * RES;
export const CLS = { UNKNOWN: 0, FREE: 1, ROUGH: 2, ROCK: 3, SLOPE: 4, CRATER: 5, LETHAL: 6 };

export class Slam {
  constructor(sim, sensors) {
    this.sim = sim; this.sensors = sensors;
    this.rate = 20;
    this.tiles = new Map();
    const p = sim.rover.pose();
    this.est = { x: p.x, y: p.y, z: p.z, yaw: p.yaw };
    this.lastMatch = { ok: false, score: 0, dx: 0, dz: 0, dyaw: 0, used: 0, t: 0 };
    this.matching = true; this.mapping = true; this.useSunSensor = true;
    this.matchMaxR = 50; this.matchMinInc = 0.05; this.matchMinN = 3; this.matchMaxVar = Infinity;
    this.trajectory = [];
    this.routes = [];
    this.recording = null;
    this.version = 0;          // bumps when the map changes (for renderers / planner)
    this.cellsKnown = 0;
    this.limits = { slopeLethal: 20, slopeCaution: 9, stepLethal: 0.85, stepCaution: 0.4 }; // 2.1 m wheels roll up 0.9 m edges
    sim.on('scan', (s) => this.onScan(s));
  }

  // ---------- map storage ----------
  tileKey(tx, tz) { return tx * 100000 + tz; }
  getTile(tx, tz, create) {
    const k = this.tileKey(tx, tz);
    let t = this.tiles.get(k);
    if (!t && create) {
      t = { tx, tz, born: this.sim.time, h: new Float32Array(TILE * TILE), h0: new Float32Array(TILE * TILE), t0: new Float32Array(TILE * TILE), v: new Float32Array(TILE * TILE), n: new Uint8Array(TILE * TILE), haz: new Uint8Array(TILE * TILE), cls: new Uint8Array(TILE * TILE), dirty: true, ver: 0 };
      this.tiles.set(k, t);
    }
    return t;
  }
  cellOf(x, z) { return [Math.floor(x / RES), Math.floor(z / RES)]; }
  heightCell(ix, iz) {
    const tx = Math.floor(ix / TILE), tz = Math.floor(iz / TILE);
    const t = this.tiles.get(this.tileKey(tx, tz));
    if (!t) return NaN;
    const i = (iz - tz * TILE) * TILE + (ix - tx * TILE);
    return t.n[i] ? t.h[i] : NaN;
  }
  heightAt(x, z) { return this.heightCell(Math.floor(x / RES), Math.floor(z / RES)); }
  // bilinear height between cell centres; NaN unless all 4 cells have >= minN samples
  heightInterp(x, z, minN = 1) {
    const fx = x / RES - 0.5, fz = z / RES - 0.5;
    const ix = Math.floor(fx), iz = Math.floor(fz), ax = fx - ix, az = fz - iz;
    const g = (i, j) => {
      const tx = Math.floor(i / TILE), tz = Math.floor(j / TILE);
      const t = this.tiles.get(tx * 100000 + tz);
      if (!t) return NaN;
      const k = (j - tz * TILE) * TILE + (i - tx * TILE);
      return t.n[k] >= minN && t.v[k] < this.matchMaxVar ? t.h[k] : NaN;
    };
    const h00 = g(ix, iz), h10 = g(ix + 1, iz), h01 = g(ix, iz + 1), h11 = g(ix + 1, iz + 1);
    return (h00 * (1 - ax) + h10 * ax) * (1 - az) + (h01 * (1 - ax) + h11 * ax) * az;
  }
  infoAt(x, z) {
    const ix = Math.floor(x / RES), iz = Math.floor(z / RES);
    const tx = Math.floor(ix / TILE), tz = Math.floor(iz / TILE);
    const t = this.tiles.get(this.tileKey(tx, tz));
    if (!t) return null;
    if (t.dirty) this.computeHazard(t);
    const i = (iz - tz * TILE) * TILE + (ix - tx * TILE);
    return { h: t.h[i], n: t.n[i], haz: t.haz[i], cls: t.cls[i] };
  }

  insert(x, y, z) {
    const ix = Math.floor(x / RES), iz = Math.floor(z / RES);
    const tx = Math.floor(ix / TILE), tz = Math.floor(iz / TILE);
    const t = this.getTile(tx, tz, true);
    const i = (iz - tz * TILE) * TILE + (ix - tx * TILE);
    const n = t.n[i];
    const now = this.sim.time;
    if (n === 0) { t.h[i] = y; t.h0[i] = y; t.t0[i] = now || 1e-3; t.v[i] = 0; t.n[i] = 1; this.cellsKnown++; }
    else {
      // running mean/variance capped at 8 samples: settles noise but adapts if the world changes.
      // High variance marks vertical faces (rocks, structures) whose heights are view-dependent.
      const w = Math.min(n + 1, 8);
      const d = y - t.h[i];
      t.h[i] += d / w;
      t.v[i] += (d * d - t.v[i]) / w;
      // reference height: averaged over the first 20 s of observation, then frozen (loop-closure anchor)
      if (now - t.t0[i] < 20) t.h0[i] = t.h[i];
      if (n < 255) t.n[i] = n + 1;
    }
    t.dirty = true;
  }

  // ---------- traversability ----------
  computeHazard(t) {
    const B = 3, W = TILE + 2 * B;
    const hp = new Float32Array(W * W).fill(NaN);
    for (let dz = -1; dz <= 1; dz++) for (let dx = -1; dx <= 1; dx++) {
      const nt = dx === 0 && dz === 0 ? t : this.tiles.get(this.tileKey(t.tx + dx, t.tz + dz));
      if (!nt) continue;
      for (let lz = 0; lz < W; lz++) {
        const gz = lz - B - dz * TILE; if (gz < 0 || gz >= TILE) continue;
        for (let lx = 0; lx < W; lx++) {
          const gx = lx - B - dx * TILE; if (gx < 0 || gx >= TILE) continue;
          const i = gz * TILE + gx;
          if (nt.n[i]) hp[lz * W + lx] = nt.h[i];
        }
      }
    }
    const L = this.limits;
    const tanLethal = Math.tan(L.slopeLethal * Math.PI / 180), tanCaution = Math.tan(L.slopeCaution * Math.PI / 180);
    for (let z = 0; z < TILE; z++) for (let x = 0; x < TILE; x++) {
      const i = z * TILE + x;
      if (!t.n[i]) { t.haz[i] = 0; t.cls[i] = CLS.UNKNOWN; continue; }
      const c = (z + B) * W + (x + B);
      const hc = hp[c];
      // least-squares plane over 5x5 window (2.5 m) -> slope; residual max -> step/rock height
      let sx = 0, sz = 0, sxx = 0, szz = 0, sxh = 0, szh = 0, sh = 0, k = 0;
      for (let dz = -2; dz <= 2; dz++) for (let dx = -2; dx <= 2; dx++) {
        const v = hp[c + dz * W + dx]; if (v !== v) continue;
        sx += dx; sz += dz; sxx += dx * dx; szz += dz * dz; sxh += dx * v; szh += dz * v; sh += v; k++;
      }
      if (k < 7) { t.haz[i] = 40; t.cls[i] = CLS.ROUGH; continue; }
      const mx = sx / k, mz = sz / k, mh = sh / k;
      const vx = sxx / k - mx * mx, vz = szz / k - mz * mz;
      const gx = vx > 1e-6 ? (sxh / k - mx * mh) / vx / RES : 0;
      const gz = vz > 1e-6 ? (szh / k - mz * mh) / vz / RES : 0;
      const grad = Math.hypot(gx, gz);
      let step = 0, below = 0;
      for (let dz = -2; dz <= 2; dz++) for (let dx = -2; dx <= 2; dx++) {
        const v = hp[c + dz * W + dx]; if (v !== v) continue;
        const pl = mh + (gx * (dx - mx) + gz * (dz - mz)) * RES;
        const r = v - pl;
        if (r > step) step = r; if (-r > below) below = -r;
      }
      step = Math.max(step, below) * 1.6; // peak-to-plane -> obstacle height (half the window sees the base)
      // crater cue: cell well below the wider neighbourhood mean
      let wsum = 0, wk = 0;
      for (let dz = -3; dz <= 3; dz += 3) for (let dx = -3; dx <= 3; dx += 3) { const v = hp[c + dz * W + dx]; if (v === v) { wsum += v; wk++; } }
      const depress = wk > 3 ? wsum / wk - hc : 0;
      let cost = 1 + 5 * Math.min(1, grad / tanLethal) ** 2 + 7 * Math.min(1, step / L.stepLethal) ** 2;
      let cls = CLS.FREE;
      if (grad > tanCaution) cls = CLS.SLOPE;
      if (step > L.stepCaution) cls = CLS.ROCK;
      if (depress > 0.35 && grad > tanCaution * 0.8) cls = CLS.CRATER;
      if (grad > tanLethal || step > L.stepLethal) { cost = 255; cls = depress > 0.2 ? CLS.CRATER : (step > L.stepLethal ? CLS.LETHAL : CLS.SLOPE); }
      else if (cls === CLS.FREE && step > 0.1) cls = CLS.ROUGH;
      t.haz[i] = cost >= 255 ? 255 : Math.min(254, Math.round(cost * 20));
      t.cls[i] = cost >= 255 ? (cls === CLS.CRATER ? CLS.CRATER : CLS.LETHAL) : cls;
    }
    t.dirty = false; t.ver++;
  }

  // Wheel-contact hazard from raw heights in a tight 5-point stencil (+/-0.5 m): what a wheel
  // actually rolls over, without the 2.5 m smoothing window used for the global costmap.
  // A carried module raises the CoM to ~6.8 m: the traversability limits tighten (tip margin 23 deg static)
  setPayload(loaded) {
    if (!!this._payload === !!loaded) return;
    this._payload = !!loaded;
    this.limits.slopeLethal = loaded ? 8.5 : 20; this.limits.slopeCaution = loaded ? 4.5 : 9;
    this.limits.stepLethal = loaded ? 0.6 : 0.85; this.limits.stepCaution = loaded ? 0.3 : 0.4;
    for (const t of this.tiles.values()) t.dirty = true;
  }

  wheelHazard(x, z, relaxed = false) {
    const h0 = this.heightAt(x, z);
    if (h0 !== h0) return null;
    let mx = h0, mn = h0, k = 1;
    const hx1 = this.heightAt(x + 0.5, z), hx0 = this.heightAt(x - 0.5, z), hz1 = this.heightAt(x, z + 0.5), hz0 = this.heightAt(x, z - 0.5);
    for (const v of [hx1, hx0, hz1, hz0]) if (v === v) { mx = Math.max(mx, v); mn = Math.min(mn, v); k++; }
    const gx = hx1 === hx1 && hx0 === hx0 ? (hx1 - hx0) : 0, gz = hz1 === hz1 && hz0 === hz0 ? (hz1 - hz0) : 0;
    const slope = Math.hypot(gx, gz);
    const step = mx - mn - slope * 0.5;
    const lim = relaxed ? { stepLethal: 0.85, slopeLethal: 20 } : this.limits; // relaxed: backing out along the track we came in on
    const lethal = step > lim.stepLethal || slope > Math.tan(lim.slopeLethal * Math.PI / 180) * 1.05;
    return { h: h0, lethal, cost: 1 + 6 * Math.min(1, step / this.limits.stepLethal) ** 2 + 4 * Math.min(1, slope / 0.4) ** 2 };
  }

  // object-level map update: forget the footprint of an object the robot just picked up
  clearFootprint(cx, cz, yaw, halfL, halfW) {
    const c = Math.cos(yaw), s = Math.sin(yaw);
    for (let a = -halfL; a <= halfL; a += RES * 0.5) for (let b = -halfW; b <= halfW; b += RES * 0.5) {
      const x = cx + a * c + b * s, z = cz - a * s + b * c;
      const ix = Math.floor(x / RES), iz = Math.floor(z / RES);
      const t = this.tiles.get(this.tileKey(Math.floor(ix / TILE), Math.floor(iz / TILE)));
      if (!t) continue;
      const i = (iz - Math.floor(iz / TILE) * TILE) * TILE + (ix - Math.floor(ix / TILE) * TILE);
      if (t.n[i]) { t.n[i] = 0; this.cellsKnown--; t.dirty = true; }
    }
    this.version++;
  }

  // ---------- localisation ----------
  update(dt) {
    const o = this.sensors.odom;
    const e = this.est;
    e.yaw += o.dyaw;
    const c = Math.cos(e.yaw), s = Math.sin(e.yaw);
    e.x += o.dx * c + o.dz * s;
    e.z += -o.dx * s + o.dz * c;
    e.y += o.dy;
    o.dx = o.dz = o.dy = o.dyaw = 0;
    // sun-sensor heading update (sun azimuth known from ephemeris)
    const ss = this.sensors.sunSensor;
    if (ss && ss.t !== this._sunUsed && this.useSunSensor) {
      this._sunUsed = ss.t;
      const sunAz = Math.atan2(-this.sim.sunDir.z, this.sim.sunDir.x);
      const yawMeas = sunAz - ss.azBody;
      let d = yawMeas - e.yaw; d = Math.atan2(Math.sin(d), Math.cos(d));
      e.yaw += d * 0.05;
    }
    // breadcrumbs for route memory
    const last = this.trajectory[this.trajectory.length - 1];
    if (!last || Math.hypot(last.x - e.x, last.z - e.z) > 2) {
      this.trajectory.push({ x: e.x, z: e.z, t: this.sim.time });
      if (this.trajectory.length > 20000) this.trajectory.shift();
      if (this.recording) this.recording.pts.push({ x: e.x, z: e.z });
    }
  }

  attitudeQuat(yaw, roll, pitch) {
    const pr = Math.asin(Math.max(-1, Math.min(1, Math.sin(roll) / Math.max(0.2, Math.cos(pitch)))));
    return new THREE.Quaternion().setFromEuler(new THREE.Euler(-pr, yaw, pitch, 'YZX'));
  }

  onScan(scan) {
    const e = this.est;
    const n = scan.n;
    // level the body-frame points with IMU attitude (yaw applied per candidate below)
    const q0 = this.attitudeQuat(0, scan.roll, scan.pitch);
    const lev = new Float32Array(n * 3);
    const v = new THREE.Vector3();
    let m = 0;
    for (let i = 0; i < n; i++) {
      const bx = scan.body[i * 3], by = scan.body[i * 3 + 1], bz = scan.body[i * 3 + 2];
      if (Math.abs(bx) < 6.8 && Math.abs(bz) < 6.0 && by > -0.4) continue; // self/cargo filter (body frame at ground level)
      v.set(bx, by, bz).applyQuaternion(q0);
      lev[m * 3] = v.x; lev[m * 3 + 1] = v.y; lev[m * 3 + 2] = v.z; m++;
    }
    // ---- loop closure on revisit, then local correlative scan-to-map tracking
    if (this.matching && this.cellsKnown > 4000) { this._loopClosure(lev, m); this._match(lev, m); }
    // ---- map update with the corrected pose
    if (this.mapping) {
      const c = Math.cos(e.yaw), s = Math.sin(e.yaw);
      for (let i = 0; i < m; i++) {
        const lx = lev[i * 3], ly = lev[i * 3 + 1], lz = lev[i * 3 + 2];
        const r2 = lx * lx + lz * lz;
        if (r2 > 75 * 75) continue;
        this.insert(e.x + lx * c + lz * s, e.y + ly, e.z - lx * s + lz * c);
      }
      this.version++;
    }
  }

  _match(lev, m) {
    const e = this.est;
    // subsample: favour points with structure (well above/below the sensor plane spread)
    const idx = [];
    const stride = Math.max(1, Math.floor(m / 450));
    for (let i = 0; i < m; i += stride) {
      const r = Math.hypot(lev[i * 3], lev[i * 3 + 2]);
      // reject grazing-incidence returns: their cell means are strongly view-dependent
      const inc = (ROVER.lidar.mountLocal.y - lev[i * 3 + 1]) / Math.max(1, r);
      if (r > 5 && r < this.matchMaxR && inc > this.matchMinInc) idx.push(i);
    }
    if (idx.length < 60) return;
    // Each candidate solves its own vertical offset (trimmed mean of residuals), so a height
    // bias can never be "explained" by sliding the pose along a slope.
    const res = new Float32Array(idx.length);
    const restoreVar = () => { this.matchMaxVar = Infinity; };
    const evalPose = (x, z, yaw) => {
      const c = Math.cos(yaw), s = Math.sin(yaw);
      let used = 0, sum = 0;
      for (const i of idx) {
        const lx = lev[i * 3], ly = lev[i * 3 + 1], lz = lev[i * 3 + 2];
        this.matchMaxVar = 0.012;
        const hq = this.heightInterp(x + lx * c + lz * s, z - lx * s + lz * c, this.matchMinN);
        if (hq !== hq) continue;
        const r = (e.y + ly) - hq;
        res[used++] = r; sum += r;
      }
      if (used < 50) return { score: 0, used, off: 0 };
      let m = sum / used, s2 = 0, k = 0;
      for (let i = 0; i < used; i++) if (Math.abs(res[i] - m) < 0.6) { s2 += res[i]; k++; }
      if (k > 10) m = s2 / k;
      let score = 0;
      for (let i = 0; i < used; i++) { const r = res[i] - m; score += Math.exp(-(r * r) / (2 * 0.1 * 0.1)); }
      // normalise per matched point: otherwise poses that overlap more already-mapped cells
      // (i.e. shifted backwards along the track) win, a classic scan-to-map bias
      return { score: score / used * idx.length, used, off: m };
    };
    const base = evalPose(e.x, e.z, e.yaw);
    if (base.used < 50) { restoreVar(); return; }
    let best = { score: -1 };
    const search = (cx, cz, cyaw, sxy, syaw, nxy, nyaw) => {
      for (let a = -nyaw; a <= nyaw; a++) for (let i = -nxy; i <= nxy; i++) for (let j = -nxy; j <= nxy; j++) {
        const x = cx + i * sxy, z = cz + j * sxy, yaw = cyaw + a * syaw;
        const r = evalPose(x, z, yaw);
        // odometry prior as a penalty in score units (shifting 0.3 m costs ~2 % of a perfect match)
        const sc = r.score - 0.02 * idx.length * (((x - e.x) ** 2 + (z - e.z) ** 2) / 0.09 + ((yaw - e.yaw) / 0.012) ** 2) * 0.5;
        if (sc > best.score) best = { score: sc, x, z, yaw, used: r.used, off: r.off };
      }
    };
    search(e.x, e.z, e.yaw, 0.2, 0.005, 2, 2);
    search(best.x, best.z, best.yaw, 0.06, 0.0017, 2, 1);
    const baseScore = base.score;
    const ok = best.used > 50 && best.score > baseScore * 1.01;
    const gain = 0.5;
    if (ok) {
      let dx = best.x - e.x, dz = best.z - e.z, dyaw = best.yaw - e.yaw;
      // degeneracy check: only correct along directions where the match is sharply peaked
      const raw = evalPose(best.x, best.z, best.yaw).score;
      const sharp = (ox, oz, oy) => { const a = evalPose(best.x + ox, best.z + oz, best.yaw + oy).score, b = evalPose(best.x - ox, best.z - oz, best.yaw - oy).score; return (raw - (a + b) / 2) / Math.max(1, raw); };
      const sx = sharp(0.35, 0, 0), sz = sharp(0, 0.35, 0), sy = sharp(0, 0, 0.012);
      if (sx < 0.04) dx = 0; if (sz < 0.04) dz = 0; if (sy < 0.03) dyaw = 0;
      this.degenerate = { x: sx < 0.04, z: sz < 0.04, yaw: sy < 0.03 };
      e.x += dx * gain; e.z += dz * gain; e.yaw += dyaw * gain;
      this.lastMatch = { ok: true, score: best.score / best.used, dx, dz, dyaw, used: best.used, t: this.sim.time };
    } else this.lastMatch = { ...this.lastMatch, ok: false, t: this.sim.time };
    restoreVar();
    const off = ok ? best.off : base.off;          // vertical: rover height relative to the map
    e.y -= Math.max(-0.3, Math.min(0.3, off)) * 0.5;
  }

  // Loop closure: when most of a scan lands in map tiles built long ago (a revisit), search a wide
  // window (+/-12 m, +/-3 deg) against that OLD map only, structures included, and snap the pose
  // back into agreement with it if one candidate clearly wins. Keeps the map frame consistent.
  _loopClosure(lev, m) {
    const now = this.sim.time;
    if (this.lcSuspended || now - (this._lcT || 0) < 10) return;
    const e = this.est;
    const idx = [];
    const stride = Math.max(1, Math.floor(m / 260));
    for (let i = 0; i < m; i += stride) { const r = Math.hypot(lev[i * 3], lev[i * 3 + 2]); if (r > 6 && r < 70) idx.push(i); }
    if (idx.length < 80) return;
    const OLD = 240;
    const oldH = (x, z) => {
      const ix = Math.floor(x / RES), iz = Math.floor(z / RES), tx = Math.floor(ix / TILE), tz = Math.floor(iz / TILE);
      const t = this.tiles.get(tx * 100000 + tz);
      if (!t) return NaN;
      const k = (iz - tz * TILE) * TILE + (ix - tx * TILE);
      return t.n[k] >= 2 && t.t0[k] !== 0 && now - t.t0[k] > OLD ? t.h0[k] : NaN;
    };
    // is this a revisit? (most points fall into old tiles at the current estimate)
    let oldHits = 0;
    { const c = Math.cos(e.yaw), s = Math.sin(e.yaw);
      for (const i of idx) { const x = e.x + lev[i * 3] * c + lev[i * 3 + 2] * s, z = e.z - lev[i * 3] * s + lev[i * 3 + 2] * c; if (oldH(x, z) === oldH(x, z)) oldHits++; } }
    if (oldHits < idx.length * 0.45) return;
    this._lcT = now;
    const res = new Float32Array(idx.length);
    const score = (x, z, yaw) => {
      const c = Math.cos(yaw), s = Math.sin(yaw); let u = 0, sum = 0;
      for (const i of idx) { const h = oldH(x + lev[i * 3] * c + lev[i * 3 + 2] * s, z - lev[i * 3] * s + lev[i * 3 + 2] * c); if (h !== h) continue; const r = e.y + lev[i * 3 + 1] - h; res[u++] = r; sum += r; }
      if (u < 60) return { sc: 0, u };
      const mu = sum / u; let sc = 0;
      for (let k = 0; k < u; k++) { const r = res[k] - mu; sc += Math.exp(-(r * r) / (2 * 0.25 * 0.25)); }
      return { sc: sc / u * Math.min(1, u / (idx.length * 0.6)), u, mu };
    };
    const cur = score(e.x, e.z, e.yaw);
    let best = { sc: -1 };
    for (let a = -3; a <= 3; a++) for (let i = -8; i <= 8; i++) for (let j = -8; j <= 8; j++) {
      const r = score(e.x + i * 1.5, e.z + j * 1.5, e.yaw + a * 0.0175);
      if (r.sc > best.sc) best = { ...r, x: e.x + i * 1.5, z: e.z + j * 1.5, yaw: e.yaw + a * 0.0175 };
    }
    // refine around the winner
    for (let a = -2; a <= 2; a++) for (let i = -4; i <= 4; i++) for (let j = -4; j <= 4; j++) {
      const r = score(best.x + i * 0.35, best.z + j * 0.35, best.yaw + a * 0.005);
      if (r.sc > best.sc) best = { ...r, x: best.x + i * 0.35, z: best.z + j * 0.35, yaw: best.yaw + a * 0.005 };
    }
    const shift = Math.hypot(best.x - e.x, best.z - e.z);
    // plausibility: odometry + IMU drift is ~1-3 % of distance travelled since the last anchor
    const travelled = this.sensors.odom.dist - (this._anchorDist || 0);
    const maxShift = 2 + 0.03 * travelled;
    if (best.sc > 0.75 && best.sc > cur.sc * 1.25 && shift > 0.8 && shift < maxShift && cur.sc > 0.3) {
      this.lastLoop = { t: now, shift, dyaw: (best.yaw - e.yaw) * 57.3, score: best.sc, before: cur.sc };
      e.x = best.x; e.z = best.z; e.yaw = best.yaw; e.y -= Math.max(-1, Math.min(1, best.mu)) * 0.8;
      this._repairMap(e.x, e.z, now);
      this._anchorDist = this.sensors.odom.dist;
      this.sim.log(`SLAM loop closure: revisited mapped area, pose corrected by ${shift.toFixed(1)} m / ${this.lastLoop.dyaw.toFixed(2)} deg (match ${cur.sc.toFixed(2)} -> ${best.sc.toFixed(2)})`, 'ok');
    }
  }

  // after a loop closure: trusted (old) cells near the robot revert to their reference heights and
  // cells first seen during the drifted interval are cleared, so drift "ghosts" disappear
  _repairMap(cx, cz, now) {
    const R = 90, since = now - 300;
    const tx0 = Math.floor((cx - R) / (TILE * RES)), tx1 = Math.floor((cx + R) / (TILE * RES));
    const tz0 = Math.floor((cz - R) / (TILE * RES)), tz1 = Math.floor((cz + R) / (TILE * RES));
    let reverted = 0, cleared = 0;
    for (let a = tx0; a <= tx1; a++) for (let b = tz0; b <= tz1; b++) {
      const t = this.tiles.get(this.tileKey(a, b)); if (!t) continue;
      for (let i = 0; i < t.n.length; i++) {
        if (!t.n[i]) continue;
        if (t.t0[i] < since) { if (t.h[i] !== t.h0[i]) { t.h[i] = t.h0[i]; t.n[i] = Math.min(t.n[i], 3); t.v[i] = 0; reverted++; } }
        else { t.n[i] = 0; this.cellsKnown--; cleared++; }
      }
      t.dirty = true;
    }
    this.version++;
    this.lastRepair = { t: now, reverted, cleared };
  }

  // ---------- route memory (teach & repeat) ----------
  beginRecord(from, to) { this.recording = { from, to, pts: [{ x: this.est.x, z: this.est.z }], t0: this.sim.time, e0: this.sim.rover.power.energyUsedWh }; }
  endRecord(success) {
    const r = this.recording; this.recording = null;
    if (!r || !success || r.pts.length < 3) return null;
    let len = 0; for (let i = 1; i < r.pts.length; i++) len += Math.hypot(r.pts[i].x - r.pts[i - 1].x, r.pts[i].z - r.pts[i - 1].z);
    const route = { id: `R${this.routes.length + 1}`, from: r.from, to: r.to, pts: r.pts, length: len, time: this.sim.time - r.t0, energyWh: this.sim.rover.power.energyUsedWh - r.e0, uses: 0, created: Date.now(), ok: true };
    // keep the best (shortest/fastest) route per endpoint pair
    const prev = this.routes.find((x) => x.from === r.from && x.to === r.to);
    if (prev && prev.time <= route.time) { prev.uses++; return prev; }
    if (prev) this.routes.splice(this.routes.indexOf(prev), 1);
    this.routes.push(route);
    this.sim.emit('routes');
    return route;
  }
  findRoute(from, to) {
    const a = this.routes.find((r) => r.ok && r.from === from && r.to === to);
    if (a) return { route: a, pts: a.pts };
    const b = this.routes.find((r) => r.ok && r.from === to && r.to === from);
    if (b) return { route: b, pts: [...b.pts].reverse() };
    return null;
  }

  // operator-supplied relocalisation (e.g. after a manual teleport / kidnapped robot)
  relocalize(p) { this.est.x = p.x; this.est.y = p.y; this.est.z = p.z; this.est.yaw = p.yaw; }

  clear() {
    this.tiles.clear(); this.cellsKnown = 0; this.routes = []; this.trajectory = []; this.version++;
    this.sim.emit('routes');
  }

  // ---------- persistence ----------
  serialize() {
    const tiles = [];
    for (const t of this.tiles.values()) tiles.push({ tx: t.tx, tz: t.tz, h: t.h, h0: t.h0, n: t.n, v: t.v });
    return { seed: this.sim.terrain.seed, savedAt: Date.now(), tiles, routes: this.routes };
  }
  deserialize(d) {
    if (!d || d.seed !== this.sim.terrain.seed) return false;
    this.tiles.clear(); this.cellsKnown = 0;
    for (const s of d.tiles) {
      const t = this.getTile(s.tx, s.tz, true);
      t.h.set(s.h); t.n.set(s.n); if (s.v) t.v.set(s.v); t.h0.set(s.h0 || s.h); t.t0.fill(-1e9); t.born = -1e9; t.dirty = true; // loaded tiles count as old
      for (let i = 0; i < t.n.length; i++) if (t.n[i]) this.cellsKnown++;
    }
    this.routes = d.routes || [];
    this.version++;
    this.sim.emit('routes');
    return true;
  }
}

export { TS as TILE_SIZE_M };
