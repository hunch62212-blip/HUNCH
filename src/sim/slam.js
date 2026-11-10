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
    this.kf = [];              // keyframe graph (CHANGES.md §17): {t, d, x, z, yaw}, a node every 10 m or 15 deg
    this.kfSpacing = 10; this.kfTurn = 15 * Math.PI / 180;
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
  // An unmapped cell inside mapped ground: a pinhole between LiDAR rings (16 channels leave 1-5 m between rings
  // 10-25 m out), not unexplored terrain. True when >= 5 of the 8 cells 0.5 m / 1.0 m away along the axes are mapped.
  // Any real negative obstacle (crater rim, hole) is metres wide and fails this.
  pinhole(x, z) {
    let k = 0;
    for (const d of [-1, -0.5, 0.5, 1]) { if (this.heightAt(x + d, z) === this.heightAt(x + d, z)) k++; if (this.heightAt(x, z + d) === this.heightAt(x, z + d)) k++; }
    return k >= 5;
  }
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
      let sx = 0, sz = 0, sxx = 0, szz = 0, sxz = 0, sxh = 0, szh = 0, sh = 0, k = 0;
      for (let dz = -2; dz <= 2; dz++) for (let dx = -2; dx <= 2; dx++) {
        const v = hp[c + dz * W + dx]; if (v !== v) continue;
        sx += dx; sz += dz; sxx += dx * dx; szz += dz * dz; sxz += dx * dz; sxh += dx * v; szh += dz * v; sh += v; k++;
      }
      if (k < 7) { t.haz[i] = 40; t.cls[i] = CLS.ROUGH; continue; }
      const mx = sx / k, mz = sz / k, mh = sh / k;
      // Least-squares plane through the window. The x and z slopes are solved jointly (2x2 normal equations): solving
      // each axis on its own ignores their correlation and was wrong whenever the samples are not a full grid (a
      // diagonal LiDAR ring). When the samples are nearly collinear (one ring far from the rover) the slope across the
      // ring is undetermined: use the minimum-norm fit (slope along the ring only). Before this, ring noise read as
      // 10-20 deg of slope and the whole edge of the mapped area turned lethal under the loaded limits.
      const cxx = sxx / k - mx * mx, czz = szz / k - mz * mz, cxz = sxz / k - mx * mz;
      const cxh = sxh / k - mx * mh, czh = szh / k - mz * mh, det = cxx * czz - cxz * cxz, tr = cxx + czz;
      let gx, gz, thin = false;
      if (det > 0.08 * tr * tr) { gx = (cxh * czz - czh * cxz) / det / RES; gz = (czh * cxx - cxh * cxz) / det / RES; }
      else {
        thin = true;
        const th = 0.5 * Math.atan2(2 * cxz, cxx - czz), uc = Math.cos(th), us = Math.sin(th);
        const vu = cxx * uc * uc + 2 * cxz * uc * us + czz * us * us, gu = vu > 1e-6 ? (cxh * uc + czh * us) / vu / RES : 0;
        gx = gu * uc; gz = gu * us;
      }
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
      else if (cls === CLS.FREE && (step > 0.1 || thin)) cls = CLS.ROUGH; // a single ring: slope only half known
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
    // loaded static tip angle ~29 deg (half-track 3.83 m / CoM 6.8 m); 10.5 deg keeps >= 18 deg of margin and stops
    // the map's slope over-read (5-6 deg true reads ~9, CHANGES.md §9) from walling the loaded rover in (was 8.5 / 4.5)
    this.limits.slopeLethal = loaded ? 10.5 : 20; this.limits.slopeCaution = loaded ? 6 : 9;
    this.limits.stepLethal = loaded ? 0.6 : 0.85; this.limits.stepCaution = loaded ? 0.3 : 0.4;
    for (const t of this.tiles.values()) t.dirty = true;
  }

  wheelHazard(x, z, relaxed = false) {
    const h0 = this.heightAt(x, z);
    if (h0 !== h0) return null;
    // Slope: least-squares plane through a 9-point cross (+-0.5 m and +-1.0 m on each axis, 2 m baseline).
    // The old 2-point difference over 1 m read 3-5 cm of map noise as 4-7 deg of slope and walled the loaded
    // rover in on flat ground; this fit's worst case for +-6 cm noise is ~4 deg (sum|d| * 0.06 / sum d^2).
    // per-axis sums include the centre point (d = 0); each axis is centred on the points it has, so a
    // half-mapped stencil at the map frontier does not bias the slope
    let sx = 0, sxh = 0, sxx = 0, shx = h0, nx = 1, sz = 0, szh = 0, szz = 0, shz = h0, nz = 1;
    const ring = [];
    for (const d of [-1, -0.5, 0.5, 1]) {
      const hx = this.heightAt(x + d, z), hz = this.heightAt(x, z + d);
      if (hx === hx) { sx += d; sxh += d * hx; sxx += d * d; shx += hx; nx++; if (Math.abs(d) === 0.5) ring.push([d, 0, hx]); }
      if (hz === hz) { sz += d; szh += d * hz; szz += d * d; shz += hz; nz++; if (Math.abs(d) === 0.5) ring.push([0, d, hz]); }
    }
    const vx = sxx - sx * sx / nx, vz = szz - sz * sz / nz;
    const gx = vx > 0.4 ? (sxh - sx * shx / nx) / vx : 0, gz = vz > 0.4 ? (szh - sz * shz / nz) / vz : 0;
    const mh = h0; // plane anchored at the wheel point itself
    const slope = Math.hypot(gx, gz);
    // step: spread of the inner 5 points about that plane (a rock / edge under the tyre, not the slope)
    let rMax = h0 - mh, rMin = h0 - mh;
    for (const [dx, dz, h] of ring) { const r = h - (mh + gx * dx + gz * dz); if (r > rMax) rMax = r; if (r < rMin) rMin = r; }
    const step = rMax - rMin;
    const lim = relaxed ? { stepLethal: 0.85, slopeLethal: 20 } : this.limits; // relaxed: backing out along the track we came in on
    const lethal = step > lim.stepLethal || slope > Math.tan(lim.slopeLethal * Math.PI / 180) * 1.05;
    return { h: h0, lethal, slope, step, cost: 1 + 6 * Math.min(1, step / this.limits.stepLethal) ** 2 + 4 * Math.min(1, slope / 0.4) ** 2 };
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
    this._keyframe(e);
    // breadcrumbs for route memory
    const last = this.trajectory[this.trajectory.length - 1];
    if (!last || Math.hypot(last.x - e.x, last.z - e.z) > 2) {
      this.trajectory.push({ x: e.x, z: e.z, t: this.sim.time, d: this.sensors.odom.dist });
      if (this.trajectory.length > 20000) this.trajectory.shift();
      if (this.recording) this.recording.pts.push({ x: e.x, z: e.z, t: this.sim.time });
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
  // The jump is a fresh start for drift, so a later fiducial fix must not bend data recorded before it.
  relocalize(p) {
    this.est.x = p.x; this.est.y = p.y; this.est.z = p.z; this.est.yaw = p.yaw;
    this._fixAnchor = { t: this.sim.time, d: this.sensors.odom.dist };
    this.kf = []; this._keyframe(this.est, true);
  }

  // ---------- keyframe pose graph (CHANGES.md §17) ----------
  // A sparse chain of poses (a node every kfSpacing m of odometer or kfTurn of heading change) with the relative-pose
  // constraints that the dead-reckoned estimate made between consecutive nodes. A surveyed fiducial pins the node at the
  // sighting; relaxing the graph spreads the translation AND heading error back along the nodes, and every map cell,
  // breadcrumb and route point then moves rigidly with its two neighbouring nodes. The old scheme translated each cell by
  // (offset x odometer fraction): no rotation, so a heading drift was never undone and local elevation tiles were sheared.
  _keyframe(e, force = false) {
    const d = this.sensors.odom.dist, K = this.kf, last = K[K.length - 1];
    let dyaw = 0; if (last) { dyaw = e.yaw - last.yaw; dyaw = Math.atan2(Math.sin(dyaw), Math.cos(dyaw)); }
    if (force || !last || d - last.d >= this.kfSpacing || Math.abs(dyaw) >= this.kfTurn) {
      if (last && last.t === this.sim.time) { last.x = e.x; last.z = e.z; last.yaw = e.yaw; last.d = d; return; }
      K.push({ t: this.sim.time, d, x: e.x, z: e.z, yaw: e.yaw });
      if (K.length > 3000) K.shift();
    }
  }

  // Gauss-Seidel (with successive over-relaxation) on the SE(2) pose graph: nodes[0] is held, nodes[n-1] is pinned to its
  // corrected pose, the free nodes in between settle so that each relative pose matches the odometry edge as closely as the
  // edge weights allow. Edge i -> i+1 measures (f, r, dpsi) in node i's frame; variance grows with the edge length
  // (sigma_t^2 = (0.02 m)^2 L, sigma_psi^2 = (0.003 rad)^2 L). Returns corrected {x, z, yaw} per node and the iterations used.
  static relaxChain(nodes, goal) {
    const n = nodes.length, wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
    const P = nodes.map((q) => ({ x: q.x, z: q.z, yaw: q.yaw }));
    const toLocal = (yaw, vx, vz) => { const c = Math.cos(yaw), s = Math.sin(yaw); return [vx * c - vz * s, vx * s + vz * c]; };
    const toWorld = (yaw, f, r) => { const c = Math.cos(yaw), s = Math.sin(yaw); return [f * c + r * s, -f * s + r * c]; };
    const E = [];
    for (let i = 0; i < n - 1; i++) {
      const [f, r] = toLocal(nodes[i].yaw, nodes[i + 1].x - nodes[i].x, nodes[i + 1].z - nodes[i].z);
      const L = Math.max(1, nodes[i + 1].d - nodes[i].d);
      E.push({ f, r, dpsi: wrap(nodes[i + 1].yaw - nodes[i].yaw), wt: 1 / (0.0004 * L), wr: 1 / (9e-6 * L) });
    }
    // start from the odometer-fraction (old scheme) solution: LM then only has the heading coupling left to resolve
    const span = Math.max(1, nodes[n - 1].d - nodes[0].d);
    for (let i = 1; i < n; i++) {
      const f = i === n - 1 ? 1 : (nodes[i].d - nodes[0].d) / span;
      P[i].x += f * goal.dx; P[i].z += f * goal.dz; P[i].yaw = wrap(P[i].yaw + f * goal.dyaw);
    }
    if (n < 3) return { P, iters: 0 };
    // Levenberg-Marquardt on the chain (nodes 0 and n-1 pinned). Each step linearises the edge residuals
    // (local displacement, heading increment) and solves the block-tridiagonal normal equations exactly (block
    // Thomas), so it converges in a handful of steps however long the chain is. The plain Gauss-Seidel sweep this
    // replaced did not converge on a 260-node, 2 km chain (nodes ended 150 m / 90 deg off; CHANGES.md §17).
    const cost = (Q) => {
      let c = 0;
      for (let i = 0; i < n - 1; i++) {
        const e = E[i], q = Q[i], nx = Q[i + 1];
        const [fl, rl] = toLocal(q.yaw, nx.x - q.x, nx.z - q.z);
        const w = wrap(nx.yaw - q.yaw - e.dpsi);
        c += e.wt * ((fl - e.f) ** 2 + (rl - e.r) ** 2) + e.wr * w * w;
      }
      return c;
    };
    const m = n - 2, D = new Array(m), U = new Array(m), G = new Array(m); // diagonal, upper blocks, gradient
    const inv3 = (M) => {
      const [a, b, c, d, e, f, g, h, k] = M, A = e * k - f * h, B = f * g - d * k, C = d * h - e * g, det = a * A + b * B + c * C;
      return [A, c * h - b * k, b * f - c * e, B, a * k - c * g, c * d - a * f, C, b * g - a * h, a * e - b * d].map((v) => v / det);
    };
    const mul = (X, Y) => { const Z = new Array(9); for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) Z[r * 3 + c] = X[r * 3] * Y[c] + X[r * 3 + 1] * Y[3 + c] + X[r * 3 + 2] * Y[6 + c]; return Z; };
    let lambda = 1e-3, c0 = cost(P), it = 0;
    for (; it < 25; it++) {
      for (let i = 0; i < m; i++) { D[i] = [0, 0, 0, 0, 0, 0, 0, 0, 0]; U[i] = [0, 0, 0, 0, 0, 0, 0, 0, 0]; G[i] = [0, 0, 0]; }
      for (let i = 0; i < n - 1; i++) {
        const e = E[i], q = P[i], nx = P[i + 1], c = Math.cos(q.yaw), s = Math.sin(q.yaw);
        const [fl, rl] = toLocal(q.yaw, nx.x - q.x, nx.z - q.z);
        const rf = fl - e.f, rr = rl - e.r, rp = wrap(nx.yaw - q.yaw - e.dpsi);
        // rows: f, r, psi; columns: (x, z, yaw) of node i (Ja) and node i+1 (Jb)
        const Ja = [-c, s, -rl, -s, -c, fl, 0, 0, -1], Jb = [c, -s, 0, s, c, 0, 0, 0, 1], w = [e.wt, e.wt, e.wr], r3 = [rf, rr, rp];
        const add = (blk, J1, J2) => { for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) { let v = 0; for (let k = 0; k < 3; k++) v += w[k] * J1[k * 3 + a] * J2[k * 3 + b]; blk[a * 3 + b] += v; } };
        const grad = (g, J) => { for (let a = 0; a < 3; a++) { let v = 0; for (let k = 0; k < 3; k++) v += w[k] * J[k * 3 + a] * r3[k]; g[a] += v; } };
        const ia = i - 1, ib = i; // free-variable slots of nodes i and i+1 (node k -> slot k-1)
        if (ia >= 0) { add(D[ia], Ja, Ja); grad(G[ia], Ja); }
        if (ib < m) { add(D[ib], Jb, Jb); grad(G[ib], Jb); }
        if (ia >= 0 && ib < m) add(U[ia], Ja, Jb);   // block (slot ia, slot ib); the lower block is its transpose
      }
      let ok = false;
      for (let tries = 0; tries < 12 && !ok; tries++) {
        // solve (H + lambda diag(H)) d = -g by block Thomas elimination (forward)
        const Dd = D.map((B) => { const X = B.slice(); for (let k = 0; k < 3; k++) X[k * 4] = B[k * 4] * (1 + lambda) + 1e-12; return X; });
        const rhs = G.map((g) => [-g[0], -g[1], -g[2]]), Dinv = new Array(m), Ds = new Array(m);
        for (let i = 0; i < m; i++) {
          if (i > 0) { // eliminate with the previous block: Ds[i] = Dd[i] - L Dinv[i-1] U[i-1], L = U[i-1]^T
            const Up = U[i - 1], L = [Up[0], Up[3], Up[6], Up[1], Up[4], Up[7], Up[2], Up[5], Up[8]], LD = mul(L, Dinv[i - 1]);
            const LDU = mul(LD, Up); Ds[i] = Dd[i].map((v, k) => v - LDU[k]);
            const r0 = rhs[i - 1]; for (let a = 0; a < 3; a++) rhs[i][a] -= LD[a * 3] * r0[0] + LD[a * 3 + 1] * r0[1] + LD[a * 3 + 2] * r0[2];
          } else Ds[i] = Dd[i];
          Dinv[i] = inv3(Ds[i]);
        }
        const dl = new Array(m);
        for (let i = m - 1; i >= 0; i--) {
          const r = rhs[i].slice();
          if (i < m - 1) for (let a = 0; a < 3; a++) r[a] -= U[i][a * 3] * dl[i + 1][0] + U[i][a * 3 + 1] * dl[i + 1][1] + U[i][a * 3 + 2] * dl[i + 1][2];
          dl[i] = [0, 1, 2].map((a) => Dinv[i][a * 3] * r[0] + Dinv[i][a * 3 + 1] * r[1] + Dinv[i][a * 3 + 2] * r[2]);
        }
        const T = P.map((q) => ({ x: q.x, z: q.z, yaw: q.yaw }));
        for (let i = 0; i < m; i++) { T[i + 1].x += dl[i][0]; T[i + 1].z += dl[i][1]; T[i + 1].yaw = wrap(T[i + 1].yaw + dl[i][2]); }
        const c1 = cost(T);
        if (Number.isFinite(c1) && c1 <= c0) { for (let i = 0; i < n; i++) P[i] = T[i]; ok = c0 - c1 > 1e-9 * (1 + c0) || true; const gain = c0 - c1; c0 = c1; lambda = Math.max(1e-9, lambda / 5); if (gain < 1e-7 * (1 + c0)) it = 999; }
        else lambda *= 8;
      }
      if (!ok || it >= 999) break;
    }
    return { P, iters: Math.min(it, 25) };
  }

  // Absolute fix (surveyed fiducial) on a built map: (dx, dz, dyaw) is the pose error at the sighting node (the current
  // pose). The graph since the last fix is relaxed with that node pinned (see relaxChain), then each map cell first seen
  // since the last fix, each breadcrumb and each route point since then is moved rigidly with the blend of its two
  // neighbouring nodes' corrections. Data older than the previous fix stays put.
  shiftFrame(dx, dz, dyaw = 0) {
    const now = this.sim.time, dNow = this.sensors.odom.dist, e = this.est;
    const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
    const A = this._fixAnchor || { t: 0, d: 0 };
    this._keyframe(e, true);                               // the sighting node
    const K = this.kf;
    let i0 = 0; while (i0 + 1 < K.length && K[i0 + 1].t <= A.t) i0++;   // newest node at or before the previous fix
    const chain = K.slice(i0);
    const rel = Slam.relaxChain(chain, { dx, dz, dyaw });
    const corr = chain.map((q, i) => ({ cx: q.x, cz: q.z, dx: rel.P[i].x - q.x, dz: rel.P[i].z - q.z, dyaw: wrap(rel.P[i].yaw - q.yaw) }));
    const rot = (vx, vz, a) => { const c = Math.cos(a), s = Math.sin(a); return [vx * c + vz * s, -vx * s + vz * c]; };
    const warpBy = (c, x, z) => { const [rx, rz] = rot(x - c.cx, z - c.cz, c.dyaw); return [c.cx + c.dx + rx, c.cz + c.dz + rz]; };
    const tAt = chain.map((q) => q.t), tFirst = chain[0].t;
    const seg = (t) => { // chain segment holding time t (binary search)
      let lo = 0, hi = chain.length - 1;
      while (lo < hi - 1) { const mid = (lo + hi) >> 1; if (tAt[mid] <= t) lo = mid; else hi = mid; }
      return lo;
    };
    const warp = (x, z, t) => {
      if (!(t > tFirst)) return [x, z];
      if (t >= tAt[tAt.length - 1]) return warpBy(corr[corr.length - 1], x, z);
      const j = seg(t), a = (t - tAt[j]) / Math.max(1e-6, tAt[j + 1] - tAt[j]);
      const [x0, z0] = warpBy(corr[j], x, z), [x1, z1] = warpBy(corr[j + 1], x, z);
      return [x0 + (x1 - x0) * a, z0 + (z1 - z0) * a];
    };
    // map: lift every cell first seen since the anchor, then drop it at its warped position
    const moved = [];
    for (const t of this.tiles.values()) {
      for (let i = 0; i < t.n.length; i++) {
        if (!t.n[i] || !(t.t0[i] > A.t)) continue;
        const ix = t.tx * TILE + (i % TILE), iz = t.tz * TILE + Math.floor(i / TILE);
        const [wx, wz] = warp((ix + 0.5) * RES, (iz + 0.5) * RES, t.t0[i]);
        const nx = Math.floor(wx / RES), nz = Math.floor(wz / RES);
        if (nx === ix && nz === iz) continue;
        moved.push(nx, nz, t.h[i], t.h0[i], t.t0[i], t.v[i], t.n[i]);
        t.n[i] = 0; this.cellsKnown--;
      }
    }
    for (let k = 0; k < moved.length; k += 7) {
      const ix = moved[k], iz = moved[k + 1], tx = Math.floor(ix / TILE), tz = Math.floor(iz / TILE);
      const t = this.getTile(tx, tz, true), i = (iz - tz * TILE) * TILE + (ix - tx * TILE);
      if (t.n[i] && !(t.t0[i] > A.t)) continue; // a trusted older cell already holds this spot
      if (!t.n[i]) this.cellsKnown++;
      t.h[i] = moved[k + 2]; t.h0[i] = moved[k + 3]; t.t0[i] = moved[k + 4]; t.v[i] = moved[k + 5]; t.n[i] = moved[k + 6];
    }
    for (const t of this.tiles.values()) t.dirty = true;
    // breadcrumbs and route points recorded since the anchor
    const shiftPts = (pts) => { for (const p of pts) if (p.t > A.t) { const [wx, wz] = warp(p.x, p.z, p.t); p.x = wx; p.z = wz; } };
    shiftPts(this.trajectory);
    for (const r of this.routes) shiftPts(r.pts);
    if (this.recording) shiftPts(this.recording.pts);
    // keyframes themselves take their relaxed poses (the sighting node ends on the surveyed pose)
    for (let i = 0; i < chain.length; i++) { chain[i].x = rel.P[i].x; chain[i].z = rel.P[i].z; chain[i].yaw = rel.P[i].yaw; }
    const c0 = corr[corr.length - 1], pre = { x: c0.cx, z: c0.cz };
    e.x = rel.P[chain.length - 1].x; e.z = rel.P[chain.length - 1].z; e.yaw = wrap(e.yaw + c0.dyaw);
    this._fixAnchor = { t: this.kf[this.kf.length - 1].t, d: dNow };
    // the same rigid correction for state that lives at the current pose (planned path, port estimates)
    const apply = (p) => { const [wx, wz] = warpBy(c0, p.x, p.z); p.x = wx; p.z = wz; };
    this.lastFix = { t: now, dx, dz, dyaw, cells: moved.length / 7, nodes: chain.length, iters: rel.iters, apply, dyawEnd: c0.dyaw, pre };
    this.version++;
    this.sim.emit('routes');
    return this.lastFix;
  }

  clear() {
    this.tiles.clear(); this.cellsKnown = 0; this.routes = []; this.trajectory = []; this.kf = []; this.version++;
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
    // stored routes are in the saved map's frame: drop point times so a fix in this session never moves them
    this.routes = (d.routes || []).map((r) => ({ ...r, pts: r.pts.map((p) => ({ x: p.x, z: p.z })) }));
    this.version++;
    this.sim.emit('routes');
    return true;
  }
}

export { TS as TILE_SIZE_M };
