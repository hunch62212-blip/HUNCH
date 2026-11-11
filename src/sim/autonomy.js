// Mission executive. Behaviours are generators: each tick the executive advances the active
// generator one step, so mission scripts read like the procedure they implement.
// All decisions use estimated (SLAM) state and sensor measurements, never ground truth.
import * as THREE from 'three';
import { ROVER, WORLD, DOCK, MOON, POWER_DOCK } from './config.js';
import { Planner, LocalPlanner, LETHAL } from './planner.js';
import { OrbitalDEM, matchToDEM } from './orbitalMap.js';
const LETHAL_HALF = LETHAL * 0.5; // planner hard-inflation cost (obstacle margin)

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const DEG = Math.PI / 180;

export const PRIORITY = { 'life-support': 5, 'emergency-repair': 5, 'power': 4, 'habitat-supplies': 3, 'science': 2 };

export class Autonomy {
  constructor(sim, sensors, slam) {
    this.sim = sim; this.sensors = sensors; this.slam = slam;
    this.rate = 10;
    this.planner = new Planner(sim, slam);
    this.local = new LocalPlanner(slam);
    this.dem = sim.terrain ? new OrbitalDEM(sim.terrain) : null; // orbital elevation model for map-relative fixes (orbitalMap.js)
    this.mrl = { on: true, dLast: 0, tLast: -1e9, fixes: 0, last: null };
    this.mode = 'MANUAL';      // MANUAL | AUTO
    this.gen = null; this.missionName = null;
    this.phase = 'Idle'; this.detail = '';
    this.path = null; this.pathIdx = 0; this.pathSource = '';
    this.goal = null;
    this.speedSetting = 99;    // autonomy cruise ceiling (m/s); 99 = automatic (smart-rpm tier decides)
    this.paused = false;
    this.faults = { comms: false, port: {} };
    this.reserveSoc = 0.2;
    this.metrics = this._newMetrics();
    this.history = [];
    this.cargoEst = new Map();     // cargo id -> {x,z,y,yaw,t,src}
    this.portEst = new Map();
    this.manifest = sim.cargo.map((c, i) => ({ id: c.id, cargo: c, port: c.dest, category: i === 0 ? 'life-support' : 'habitat-supplies', priority: i === 0 ? 5 : 3, status: 'waiting' }));
    this.docking = null;
    this.lastTick = 0;
  }

  _newMetrics() { return { start: this.sim.time, dist0: this.sim.rover.odo.dist, e0: this.sim.rover.power.energyUsedWh, deliveries: 0, accuracy: [], legs: [] }; }
  log(m, l) { this.sim.log(m, l); }

  // ---------------- state helpers ----------------
  est() { const e = this.slam.est; return { x: e.x, z: e.z, y: e.y, yaw: e.yaw }; }
  cmd(vx, vz = 0, wz = 0) { const c = this.sim.rover.cmd; c.vx = vx; c.vz = vz; c.wz = wz; }
  stop() { this.cmd(0, 0, 0); }
  // Yaw-rate loop for precision legs: a loaded skid-steer rover scrubs, so a small open-loop yaw-rate command
  // gives almost no turn (docking trace: kap -0.02 commanded, -0.002 achieved) and large ones lag by metres.
  // PI on the gyro yaw-rate error makes the achieved curvature follow the commanded one.
  cmdCurv(v, kap, reset = false) {
    if (reset || this._wzI == null) this._wzI = 0;
    // authority scales with speed (<= 0.04 /m of extra curvature): at a creep the module's sway dominates the
    // gyro and an unscaled integrator wound up and steered the nose 4 cm off in the last metre
    const wDes = v * kap, err = wDes - this.sensors.imu.yawRate, lim = 0.04 * Math.abs(v) + 0.001;
    this._wzI = clamp(this._wzI + 1.2 * err * (this.dt || 0.1), -lim, lim);
    this.cmd(v, 0, wDes + clamp(2.0 * err + this._wzI, -lim, lim));
  }
  relBody(p, e = this.est()) { // world(est) point -> body (fwd, right)
    const dx = p.x - e.x, dz = p.z - e.z, c = Math.cos(e.yaw), s = Math.sin(e.yaw);
    return { f: dx * c - dz * s, r: dx * s + dz * c };
  }
  site(x, z) {
    const P = WORLD.pad, H = WORLD.habitat, S = this.sim.terrain.pad.staging;
    if (Math.hypot(x - P.x, z - P.z) < 160) return 'PAD';
    if (Math.hypot(x - H.x, z - H.z) < 160) return 'HAB';
    if (Math.hypot(x - S.x, z - S.z) < 60) return 'STAGING';
    return `WP(${Math.round(x)},${Math.round(z)})`;
  }
  // convert a camera detection (true-frame measurement) into the rover's estimated map frame
  toEst(worldPt) {
    const R = this.sim.rover.pose(), e = this.est();
    const rel = new THREE.Vector3(worldPt.x - R.x, worldPt.y - R.y, worldPt.z - R.z).applyQuaternion(R.q.clone().invert());
    const q = this.slam.attitudeQuat(e.yaw, this.sensors.imu.roll, this.sensors.imu.pitch);
    rel.applyQuaternion(q);
    return { x: e.x + rel.x, y: e.y + rel.y, z: e.z + rel.z };
  }
  dirToEst(worldDir) {
    const R = this.sim.rover.pose(), e = this.est();
    const yawT = Math.atan2(-worldDir.z, worldDir.x);
    return wrap(yawT - R.yaw + e.yaw);
  }

  // surveyed fiducials (lander, habitat ports) have known positions in the base frame:
  // each sighting pulls the SLAM estimate toward the surveyed truth (absolute localisation)
  // Surveyed beacons initialise the map frame. Once a map exists, snapping the estimate to the
  // survey would make it inconsistent with everything already mapped (and with remembered routes),
  // so afterwards the rover stays in its self-consistent SLAM frame (no pose-graph back-end here).
  beaconFix(d) {
    if (!d.known) return;
    if (this.slam.cellsKnown > 30000) { this.mapFix(d); return; }
    if (d.range > 45) return;
    const m = this.toEst(d.pos);
    const e = this.slam.est;
    const w = 0.12 * d.confidence;
    e.x += (d.known.x - m.x) * w; e.z += (d.known.z - m.z) * w;
    const nYaw = this.dirToEst(d.normal) + d.yawNoise;
    const dy = wrap(Math.atan2(-d.known.nz, d.known.nx) - nYaw);
    if (d.range < 30) e.yaw += dy * w * 0.5;
    this.lastBeacon = { id: d.id, t: this.sim.time, range: d.range };
  }

  // Once a map exists, fiducial sightings are averaged (weight 1/sigma^2, sigma from the camera's
  // published fiducial accuracy) and applied as one correction that moves the map, routes and plan with
  // the estimate (Slam.shiftFrame), so everything stays in one consistent frame. CHANGES.md §8.
  mapFix(d) {
    const now = this.sim.time;
    this.lastBeacon = { id: d.id, t: now, range: d.range };
    if (d.range > (d.size >= 1.0 ? 95 : 60)) return;
    // never move the frame under a precision manoeuvre: the pickup / docking scene is changing (lcSuspended),
    // and inside 25 m of a port the final approach is closing a 6 cm cone on the port estimate. A frame shift
    // there moved the target by 0.5-1.7 m mid-approach and every soft capture attempt failed.
    if (this.slam.lcSuspended || /Final approach|Dock /.test(this.phase)) { this._fixes = null; return; }
    if (/pre-dock/.test(this.phase)) { const e = this.est(); for (const p of this.sim.ports) { const g = this.portGuess(p); if (Math.hypot(g.x - e.x, g.z - e.z) < 25) { this._fixes = null; return; } } }
    const m = this.toEst(d.pos);
    const sigma = 0.004 + 0.0009 * d.range * d.range / d.size;
    const w = 1 / (sigma * sigma);
    // one accumulator per fiducial: a mean over different tags mixes their survey / mounting errors
    const FX = this._fixes || (this._fixes = new Map());
    let F = FX.get(d.id);
    if (!F || now - F.tLast > 3) { F = { tLast: now, sx: 0, sz: 0, w: 0, n: 0, sy: 0, syy: 0, ny: 0 }; FX.set(d.id, F); }
    F.sx += (d.known.x - m.x) * w; F.sz += (d.known.z - m.z) * w; F.w += w; F.n++; F.tLast = now;
    // heading error from the tag's surveyed normal (CHANGES.md §17: the pose graph spreads it back along the drive)
    if (d.range < (d.size >= 1.0 ? 80 : 45) && d.known.nx != null) { const dy = wrap(Math.atan2(-d.known.nz, d.known.nx) - (this.dirToEst(d.normal) + d.yawNoise)); F.sy += dy; F.syy += dy * dy; F.ny++; }
    const dx = F.sx / F.w, dz = F.sz / F.w, off = Math.hypot(dx, dz);
    // accept only a well-averaged mean: >= 15 sightings of this tag and a standard error under
    // max(0.3 m, 3 % of the offset). A large drift is fixed coarsely first and later sightings refine it.
    // The offset must also be significant: more than 3 standard errors (and 0.5 m). The old test applied any mean over
    // 0.5 m once the standard error reached 0.3 m; with a true offset of zero that happens by chance in ~25 % of the
    // averages (2-D Gaussian, exp(-0.5^2 / (2 x 0.3^2))), so on the habitat apron the map and the estimate were shifted
    // 0.5-1.6 m by camera noise every few seconds (tests/out/dock_cyl40.log). At 3 standard errors it is ~1 %.
    const se = 1 / Math.sqrt(F.w);
    if (F.n < 15 || se > Math.max(0.3, 0.03 * off)) return;
    if (off < Math.max(0.5, 3 * se)) {
      if (se <= 0.5 / 3) FX.delete(d.id);   // averaged as far as it is useful and still no significant offset: no drift, start over
      return;                               // otherwise keep averaging this tag
    }
    this._fixes = null;
    // heading part of the fix: only from a well-averaged mean (standard error < 0.3 deg over >= 15 sightings), at most 4 deg
    let dyaw = 0;
    if (F.ny >= 15) { const mean = F.sy / F.ny, sd = Math.sqrt(Math.max(0, F.syy / F.ny - mean * mean)), se = sd / Math.sqrt(F.ny); if (se < 0.3 * DEG) dyaw = clamp(mean, -4 * DEG, 4 * DEG); }
    const A = this.slam._fixAnchor || { d: 0 };
    const limit = 5 + 0.08 * (this.sensors.odom.dist - A.d);
    if (off > limit) { this.log(`Fiducial ${d.id}: offset ${off.toFixed(1)} m exceeds plausible drift (${limit.toFixed(1)} m), ignored`, 'info'); return; }
    const r = this.applyFrameShift(dx, dz, dyaw);
    this.log(`SLAM fiducial fix (${d.id}): position corrected by ${off.toFixed(1)} m${dyaw ? ` / heading ${(dyaw / DEG).toFixed(2)} deg` : ''}, ${r.cells} map cells re-placed (${r.nodes}-node graph, ${r.iters} sweeps)`, 'ok');
  }

  // Move the estimate, the map, routes and all plan state by one correction (pose graph: Slam.shiftFrame), then replan
  applyFrameShift(dx, dz, dyaw = 0) {
    const r = this.slam.shiftFrame(dx, dz, dyaw);
    // plan state lives in the same frame: move it with the correction at the current pose and replan
    for (const p of this.path || []) r.apply(p);
    for (const b of this.planner.blocked) if (!b.fault) r.apply(b);
    for (const p of this.portEst.values()) { r.apply(p); p.nYaw = wrap(p.nYaw + r.dyawEnd); }
    this._replanNow = true;
    return r;
  }

  // Map-relative localization (orbitalMap.js): every ~40 m, correlate the local SLAM height map with the orbital DEM
  // and correct the drift it finds. Same precision-phase guards as the fiducial fixes.
  orbitalFix() {
    const M = this.mrl, dist = this.sensors.odom.dist;
    if (!M.on || !this.dem || dist - M.dLast < 40) return;
    M.dLast = dist;
    if (this.slam.lcSuspended || /Final approach|Dock |pre-dock|Pick|Clamp|Lift/.test(this.phase)) return;
    if (this.sim.time - (this.slam.lastFix?.t ?? -1e9) < 30) return;   // a fiducial fix just did this job
    const e = this.est();
    const m = matchToDEM(this.slam, this.dem, e.x, e.z);
    M.last = { t: this.sim.time, ...m };
    if (!m.ok) return;
    // One match is good to ~1.2-1.5 m (DEM noise and 4 m posting), and drift grows ~0.2 m per 40 m, so single matches
    // are averaged like the fiducial sightings: the 3 most recent agreeing matches (spread < 1.8 m) since the last frame
    // shift and within 400 m, applied only when the mean is clearly non-zero (> 1.5 m and > 2.5 standard errors).
    const fixT = this.slam.lastFix?.t ?? -1;
    M.buf = (M.buf || []).filter((q) => q.fixT === fixT && dist - q.d < 400);
    M.buf.push({ dx: m.dx, dz: m.dz, d: dist, fixT });
    // the 3 most recent only: drift keeps growing, and older matches describe an older, smaller error
    if (M.buf.length > 3) M.buf = M.buf.slice(-3);
    const B = M.buf, k = B.length;
    if (k < 3) return;
    const mx = B.reduce((a, q) => a + q.dx, 0) / k, mz = B.reduce((a, q) => a + q.dz, 0) / k;
    const sd = Math.sqrt(B.reduce((a, q) => a + (q.dx - mx) ** 2 + (q.dz - mz) ** 2, 0) / (k - 1));
    const off = Math.hypot(mx, mz), se = sd / Math.sqrt(k);
    if (sd > 1.8 || off < Math.max(1.5, 2.5 * se)) return;
    const A = this.slam._fixAnchor || { d: 0 };
    if (off > 5 + 0.08 * (dist - A.d)) { M.buf = []; return; }
    const r = this.applyFrameShift(mx, mz, 0);
    M.fixes++; M.tLast = this.sim.time; M.buf = [];
    this.log(`SLAM orbital-map fix: local terrain matched the orbital elevation map (${k} agreeing matches, spread ${sd.toFixed(1)} m): position corrected by ${off.toFixed(1)} m, ${r.cells} map cells re-placed`, 'ok');
  }

  updateEstimates() {
    for (const d of this.sensors.detections) {
      if (d.kind === 'cargo') {
        const c = d.obj, T = c.T;
        const p = this.toEst(d.pos);
        const nYaw = this.dirToEst(d.normal) + d.yawNoise;
        let axisYaw, center;
        const ax = (y) => ({ x: Math.cos(y), z: -Math.sin(y) });
        if (d.id.endsWith('/AFT')) { axisYaw = wrap(nYaw + Math.PI); const a = ax(axisYaw); center = { x: p.x + a.x * T.length / 2, z: p.z + a.z * T.length / 2 }; }
        else if (d.id.endsWith('/FWD')) { axisYaw = nYaw; const a = ax(axisYaw); center = { x: p.x - a.x * T.length / 2, z: p.z - a.z * T.length / 2 }; }
        else {
          const half = T.shape === 'cylinder' ? T.radius : T.width / 2;
          const n = ax(nYaw); center = { x: p.x - n.x * half, z: p.z - n.z * half };
          axisYaw = wrap(nYaw + (d.id.endsWith('/L') ? -Math.PI / 2 : Math.PI / 2));
          const a = ax(axisYaw); center.x += a.x * 1.2; center.z += a.z * 1.2; // side tags sit 1.2 m aft of centre
        }
        // inverse-variance fusion (a static object: many sightings average the per-frame noise down)
        const sp = 0.004 + 0.0009 * d.range * d.range / d.size, sy = (0.2 + 0.05 * d.range) * DEG;
        const lever = d.id.endsWith('/AFT') || d.id.endsWith('/FWD') ? T.length / 2 : 1.2;
        const varP = sp * sp + (sy * lever) ** 2, varY = sy * sy;
        const prev = this.cargoEst.get(c.id);
        // innovation gate: a sighting far outside the combined uncertainty is not fused. If the sightings keep
        // disagreeing (~1 s at 10 Hz) the old estimate is the stale one (relocalisation, mis-association): restart.
        let fresh = !prev || prev.src === 'manifest';
        if (!fresh) {
          const innov = Math.hypot(center.x - prev.x, center.z - prev.z), gate = 4 * Math.sqrt((prev.varP ?? 1) + varP) + 0.5;
          if (innov > gate) {
            prev.miss = (prev.miss || 0) + 1;
            if (prev.miss < 10) continue;
            this.log(`${c.id}: cargo estimate disagreed with ${prev.miss} sightings by ${innov.toFixed(1)} m, restarting it`, 'warn');
            fresh = true;
          } else prev.miss = 0;
        }
        if (fresh) this.cargoEst.set(c.id, { ...center, yaw: axisYaw, t: this.sim.time, src: d.id, range: d.range, varP, varY, miss: 0 });
        else {
          prev.varP = (prev.varP ?? 1) + 1e-5; prev.varY = (prev.varY ?? 1) + 1e-8; // process noise: est-frame drift
          const wp = prev.varP / (prev.varP + varP), wy = prev.varY / (prev.varY + varY);
          prev.x += (center.x - prev.x) * wp; prev.z += (center.z - prev.z) * wp;
          prev.yaw = wrap(prev.yaw + wrap(axisYaw - prev.yaw) * wy);
          prev.varP *= 1 - wp; prev.varY *= 1 - wy;
          prev.t = this.sim.time; prev.src = d.id; prev.range = d.range;
        }
      } else if (d.kind === 'beacon') {
        this.beaconFix(d);
      } else if (d.kind === 'port') {
        this.beaconFix(d);
        const p = this.toEst(d.pos);
        const nYaw = this.dirToEst(d.normal) + d.yawNoise;
        const prev = this.portEst.get(d.obj.id);
        const est = { x: p.x, z: p.z, y: p.y + 1.5, nYaw, t: this.sim.time, range: d.range };
        if (!prev) this.portEst.set(d.obj.id, est);
        else { const w = 0.35; prev.x += (est.x - prev.x) * w; prev.z += (est.z - prev.z) * w; prev.y += (est.y - prev.y) * w; prev.nYaw = wrap(prev.nYaw + wrap(nYaw - prev.nYaw) * w); prev.t = est.t; prev.range = est.range; }
      }
    }
  }

  // ---------------- executive ----------------
  start(name, gen) {
    if (this.sim.rover.estop) { this.log('Cannot start: E-STOP latched', 'warn'); return false; }
    if (!this.sim.rover.powered) { this.log('Cannot start: rover not powered (run START)', 'warn'); return false; }
    this.gen = gen; this.missionName = name; this.mode = 'AUTO'; this.paused = false;
    this.metrics = this._newMetrics();
    this.sim.rover.brake = false;
    this.log(`AUTO: ${name} started`, 'ok');
    this.sim.emit('mission', { name, state: 'start' });
    return true;
  }
  abort(reason = 'operator abort') {
    if (this.gen) this.log(`Mission "${this.missionName}" aborted: ${reason}`, 'warn');
    if (this.slam.recording) this.slam.endRecord(false);
    this.gen = null; this.missionName = null; this.mode = 'MANUAL'; this.phase = 'Idle'; this.detail = '';
    this.path = null; this.stop();
    this.sim.emit('mission', { state: 'abort' });
  }
  override() { // operator touched the controls
    if (this.mode === 'AUTO' && this.gen) { this.paused = true; this.mode = 'MANUAL'; this.log('Operator override: autonomy paused (type "resume" to continue)', 'warn'); }
  }
  resume() {
    if (!this.gen) { this.log('No mission to resume', 'warn'); return; }
    if (this.sim.rover.estop || !this.sim.rover.powered) { this.log('Cannot resume: rover not ready', 'warn'); return; }
    this.paused = false; this.mode = 'AUTO'; this.sim.rover.brake = false; this.log('Autonomy resumed', 'ok');
  }

  update(dt) {
    this.dt = dt;
    this.railsOk = false; // set again by navTo's cruise branch when the transit-warp check passes (Sim.stepAuto)
    this.updateEstimates();
    this.orbitalFix();
    const R = this.sim.rover;
    if (R.estop || !R.powered) { if (this.gen && !this.paused) { this.paused = true; this.mode = 'MANUAL'; } return; }
    if (this.mode !== 'AUTO' || !this.gen || this.paused) return;
    // FDIR hold (health.js): computer switch-over, SEU reset, declutch, battery isolation, repair, array stow.
    // The executive is frozen (motor controllers see a zero command) and resumes where it was.
    if (this.holdUntil > this.sim.time) {
      R.cmd.vx = R.cmd.vz = R.cmd.wz = 0;
      if (this._holdLog !== this.holdWhy) { this._holdLog = this.holdWhy; this.detail = `FDIR hold: ${this.holdWhy}`; }
      return;
    }
    this._holdLog = null;
    try {
      const r = this.gen.next(dt);
      if (r.done) {
        const res = r.value || {};
        this.stop();
        this.log(`Mission "${this.missionName}" ${res.ok === false ? 'FAILED: ' + res.why : 'complete'}`, res.ok === false ? 'err' : 'ok');
        this.history.push({ name: this.missionName, ok: res.ok !== false, ...this.summary() });
        this.sim.emit('mission', { name: this.missionName, state: 'done', ok: res.ok !== false });
        this.gen = null; this.mode = 'MANUAL'; this.phase = 'Idle'; this.path = null;
        R.brake = true;
      }
    } catch (e) {
      console.error(e);
      this.abort('internal error: ' + e.message);
    }
  }

  summary() {
    const R = this.sim.rover, m = this.metrics;
    const acc = m.accuracy.length ? m.accuracy[m.accuracy.length - 1] : null;
    return { time: this.sim.time - m.start, dist: R.odo.dist - m.dist0, energyWh: R.power.energyUsedWh - m.e0, deliveries: m.deliveries, accuracy: acc };
  }

  // ---------------- primitive behaviours ----------------
  *wait(sec, label) { const t0 = this.sim.time; if (label) this.detail = label; while (this.sim.time - t0 < sec) { this.stop(); yield; } }

  *checkPower(label) {
    const R = this.sim.rover;
    if (R.power.soc >= this.reserveSoc) return;
    if (R.loaded) {
      // the dock can't charge a loaded rover (the coupler is under the module bay) and the rover has no arrays:
      // finish the delivery if the battery covers it with margin, else pause for the operator
      const need = this.deliveryEnergyWh(), spare = R.power.wh - need, floor = 0.05 * ROVER.battery.capacityWh;
      if (spare >= floor) {
        if (!this._lowLoadedT || this.sim.time - this._lowLoadedT > 120) {
          this._lowLoadedT = this.sim.time;
          this.log(`Low battery (${(R.power.soc * 100).toFixed(0)}%) while loaded: delivery needs ~${(need / 1000).toFixed(0)} kWh, ${(R.power.wh / 1000).toFixed(0)} kWh left - continuing to ${label}, charging after drop-off`, 'warn');
        }
        return;
      }
      yield* this.pauseForOperator(`Low battery while loaded: mission paused for operator (needs ~${(need / 1000).toFixed(0)} kWh, ${(R.power.wh / 1000).toFixed(0)} kWh left; "resume" to continue anyway)`);
      return;
    }
    this.log(`Low battery (${(R.power.soc * 100).toFixed(0)}%): going to the power dock before ${label}`, 'warn');
    const r = yield* this.chargeAtDock(0.9);
    if (r && r.ok === false) yield* this.pauseForOperator(`SoC ${(R.power.soc * 100).toFixed(1)}%: power dock unreachable (${r.why}) - mission paused for operator ("resume" to continue anyway)`);
  }

  // Energy (Wh) to finish carrying the held module to its port: drive at the loaded rolling resistance plus
  // hotel load, from the current estimate (same model as the logistics manager) and the berthing / back-out.
  deliveryEnergyWh() {
    const R = this.sim.rover, c = R.lift.cargo, e = this.est();
    if (!c) return 0;
    const port = this.sim.ports.find((p) => p.id === c.dest) || this.sim.ports[0], g = this.portGuess(port);
    const dist = Math.hypot(g.x - e.x, g.z - e.z) * 1.3 + 60;
    const driveWh = dist * (R.mass + c.T.mass) * MOON.g * 0.09 / 0.86 / 3600;
    const hotelWh = 1100 * (dist / 1.0 + 300) / 3600;
    return (driveWh + hotelWh) * 1.25;
  }

  // Stop, set the brakes and pause the mission with an explicit status; the operator's "resume" continues it.
  *pauseForOperator(msg) {
    const R = this.sim.rover;
    this.stop(); R.brake = true;
    this.phase = 'PAUSED: operator decision required'; this.detail = msg;
    this.log(msg, 'err');
    this.paused = true; this.mode = 'MANUAL';
    yield; // resumes here after "resume"
    R.brake = false;
  }

  // Drive onto the power dock pad, kneel onto the underbody coupler and charge (CC-CV) to `target` SoC.
  *chargeAtDock(target = 0.95) {
    const R = this.sim.rover, PD = this.sim.powerDock, L = PD.layout;
    if (R.loaded) return { ok: false, why: 'cannot charge while carrying cargo' };
    // goals are the surveyed layout: the map frame is the survey frame (initialised and drift-corrected by the
    // lander / port / power-dock beacons). Was the true-minus-estimate pose offset: a ground-truth leak.
    const off = () => ({ x: 0, z: 0 });
    this.phase = 'Power dock: approach';
    let o = off();
    let r = yield* this.navTo({ x: L.entry.x + o.x, z: L.entry.z + o.z, name: 'power dock entry', tol: 2.0, yaw: L.yaw, yawTol: 3 * DEG });
    if (r && r.ok === false) return r;
    this.phase = 'Power dock: line up';
    yield* this.alignYaw(Math.atan2(-((L.pad.z + o.z) - this.est().z), (L.pad.x + o.x) - this.est().x), 1.5 * DEG);
    this.phase = 'Power dock: drive onto pad';
    let t0 = this.sim.time;
    while (this.sim.time - t0 < 90) {
      o = off();
      const rel = this.relBody({ x: L.pad.x + o.x, z: L.pad.z + o.z });
      if (rel.f < 0.2) break;
      const v = clamp(rel.f * 0.3, 0.12, 1.2);
      this.cmd(v, 0, v * clamp(-0.4 * Math.atan2(rel.r, Math.max(2, rel.f)), -1 / 8, 1 / 8));
      this.detail = `onto charge pad: ${rel.f.toFixed(1)} m, lateral ${rel.r.toFixed(2)} m`;
      yield;
    }
    this.stop(); R.brake = true;
    for (let i = 0; i < 30; i++) yield;
    this.phase = 'Power dock: couple';
    R.lift.target = ROVER.ride.kneel;
    t0 = this.sim.time;
    while (!PD.connected && this.sim.time - t0 < 20) { this.stop(); this.detail = `kneeling onto coupler: ride ${R.lift.pos.toFixed(2)} m`; yield; }
    if (!PD.connected) { R.lift.target = 0; R.brake = false; this.log('Power dock: coupler did not engage (not centred on the pad)', 'warn'); return { ok: false, why: 'coupler did not engage' }; }
    this.phase = 'Power dock: charging';
    const s0 = R.power.soc, ts = this.sim.time;
    while (R.power.soc < target && PD.connected) {
      this.stop();
      const rem = (target - R.power.soc) * ROVER.battery.capacityWh / Math.max(1, PD.chargeW);
      this.detail = `SoC ${(R.power.soc * 100).toFixed(1)}% -> ${(target * 100).toFixed(0)}% @ ${(PD.chargeW / 1000).toFixed(0)} kW | bank ${(PD.soc * 100).toFixed(1)}% | solar ${(PD.solarW / 1000).toFixed(0)} kW | ~${(rem * 60).toFixed(0)} min left`;
      if (!(PD.chargeW > 0) && PD.soc <= POWER_DOCK.bankFloor) this.detail += ' (bank at floor: waiting for sunrise)';
      yield;
    }
    this.log(`Charged ${(s0 * 100).toFixed(0)}% -> ${(R.power.soc * 100).toFixed(0)}% in ${((this.sim.time - ts) / 60).toFixed(1)} min (bank ${(PD.soc * 100).toFixed(0)}%)`, 'ok');
    this.phase = 'Power dock: undock';
    R.lift.target = 0;
    while (Math.abs(R.lift.pos) > 0.02) { this.stop(); this.detail = 'rising off the coupler'; yield; }
    R.brake = false;
    yield* this.backUp(18);
    return { ok: true };
  }
  *missionCharge(target = 0.95) { return yield* this.chargeAtDock(target); }

  *alignYaw(yaw, tol = 1.2 * DEG, opts = {}) {
    const R = this.sim.rover;
    let t0 = this.sim.time, noRate = 0, ok = true;
    while (true) {
      const e = this.est();
      const err = wrap(yaw - e.yaw);
      if (Math.abs(err) < tol && Math.abs(this.sensors.imu.yawRate) < 0.004) break;
      const wmax = R.loaded ? 0.07 : 0.11;
      // transit warp: a navigation pivot (sweep already checked clear by pivotSafe, rover empty) turns on rails at the
      // same rate the physics pivot runs; precision line-ups (pickup, docking, charging) never do
      if (opts.rails && !R.loaded && this.sim.warpWanted()) {
        this.railsOk = true;
        if (this.sim.warping) {
          const step = clamp(err, -wmax * this.dt, wmax * this.dt), T = R.pose();
          R.railsPlace(T.x, T.z, T.yaw + step, this.dt, 0);
          const o = this.sensors.odom; o.dyaw += step; o.dy += R.pose().y - T.y; o.vx = 0; o.vz = 0;
          this.sensors.imu.yawRate = Math.abs(err) - Math.abs(step) < tol ? 0 : step / this.dt; this.sensors.imu.roll = R.tilt.roll; this.sensors.imu.pitch = R.tilt.pitch;
          this._railsMoved = true;
          this.detail = `pivot turn (warp): heading error ${(err / DEG).toFixed(1)} deg`;
          yield; continue;
        }
      }
      const w = clamp(err * 0.9, -wmax, wmax);
      this.cmd(0, 0, Math.abs(w) < 0.012 ? Math.sign(w) * 0.012 : w);
      this.detail = `pivot turn: heading error ${(err / DEG).toFixed(1)} deg`;
      // stall: steering is set but the gyro sees no rotation -> something is blocking the sweep
      const steerDone = R.bogies.every((b) => Math.abs(b.steerTarget - b.steer) < 0.02);
      if (steerDone && Math.abs(w) > 0.02 && Math.abs(this.sensors.imu.yawRate) < 0.002) noRate += this.dt; else noRate = 0;
      if (noRate > 6) { this.log('PIVOT STALL: commanded yaw rate but gyro reads ~0 (sweep blocked)', 'err'); ok = false; break; }
      if (this.sim.time - t0 > 150) { this.log('Pivot timeout', 'warn'); ok = false; break; }
      yield;
    }
    this.stop();
    for (let i = 0; i < 10; i++) yield;
    return ok;
  }

  nearestOnPath(p) {
    const P = this.path; if (!P) return 0;
    let best = this.pathIdx, bd = Infinity;
    for (let i = Math.max(0, this.pathIdx - 5); i < Math.min(P.length, this.pathIdx + 60); i++) {
      const d = (P[i].x - p.x) ** 2 + (P[i].z - p.z) ** 2;
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  }

  routeIsClear(pts) {
    this.planner.buildCostmap();
    return this.planner.pathBlocked(pts, 0, pts.length) < 0;
  }

  // Global planning with graceful fallbacks: normal -> drop temporary blocks -> tighter inflation.
  replan(goal, reason, quiet = false) {
    const e = this.est();
    const P = this.planner;
    P.unknownNearHazardCost = (this.sim.rover.loaded || this.surveyLimits) ? 12.0 : 2.5;
    const routes = this.slam.routes.filter((r) => r.ok);
    P.buildCostmap(routes);
    // straight-in legs (final lines onto a pose goal): drive the segment itself when it is clear of the hard
    // obstacle margin, so the arrival heading is the segment's and the grid search can't bend it
    if (goal.straight) {
      const d = Math.hypot(goal.x - e.x, goal.z - e.z), n = Math.max(2, Math.ceil(d));
      const pts = []; for (let i = 0; i <= n; i++) pts.push({ x: e.x + (goal.x - e.x) * i / n, z: e.z + (goal.z - e.z) * i / n });
      if (pts.every((p, i) => { const k = P.idx(p.x, p.z); return k >= 0 && (i < 3 || P.cost[k] < LETHAL_HALF); })) {
        this.path = pts; this.pathIdx = 0; this.pathTight = false; this.pathSource = 'direct line';
        this.sim.emit('path');
        return true;
      }
    }
    let res = P.plan(e.x, e.z, goal.x, goal.z);
    if (!res) {
      const n = P.blocked.length; P.blocked = P.blocked.filter((b) => b.fault);
      if (n !== P.blocked.length) { P.buildCostmap(routes); res = P.plan(e.x, e.z, goal.x, goal.z); }
    }
    if (!res) {
      const inf = P.inflate; P.inflate = 5.0; P.buildCostmap(routes);
      res = P.plan(e.x, e.z, goal.x, goal.z, { escape: 20 });
      P.inflate = inf;
      if (res && !quiet) this.log('Planner: path found only with reduced obstacle margin (5.0 m) - proceeding slowly', 'warn');
      if (res) res.tight = true;
    }
    if (!res) { if (!quiet) this.log(`Planner: no path to ${goal.name || 'goal'} (${reason})`, 'warn'); return false; }
    // plan hysteresis: on routine replans keep the current path unless it is blocked or the new
    // one is clearly (>15 %) cheaper - otherwise near-equal detours make the rover thrash
    if (reason === 'periodic' && this.path && this.path.length > 2 && this.goal === goal) {
      const e2 = this.est(), i0 = this.nearestOnPath(e2);
      const cur = P.pathCost([{ x: e2.x, z: e2.z }, ...this.path.slice(i0)]);
      const neu = P.pathCost(res.pts);
      // commitment (v1.5.0): for 20 s after a switch, a routine replan only replaces a path that has become blocked;
      // after that the new one must be >15 % AND > 12 m of typical ground cheaper (tiny relative gains on a 2 km route
      // used to flip the plan every few seconds)
      if (Number.isFinite(cur) && this.sim.time - (this._pathSwitchT ?? -1e9) < 20) return true;
      if (Number.isFinite(cur) && (neu > cur * 0.85 || cur - neu < 12 * P.unkEff)) return true;
      // reversal guard: a routine replan that turns the rover back the way it came needs to be clearly (40 %)
      // cheaper. Newly mapped ground keeps raising the cost of the route ahead while the unmapped alternative
      // behind still looks cheap, and the loaded rover (no point turns) thrashed east-west for minutes (CHANGES.md §14)
      const dir = (pts, i) => { const a = pts[i] || pts[0], b = pts[Math.min(pts.length - 1, i + 15)]; return Math.atan2(b.z - a.z, b.x - a.x); };
      const dTurn = Math.abs(wrap(dir(res.pts, 0) - dir(this.path, i0)));
      if (Number.isFinite(cur) && dTurn > 100 * DEG && neu > cur * 0.6) return true;
      // a plan that swings the rover > 60 deg off its present line costs a long arc (or a stop and pivot): 30 % cheaper
      if (Number.isFinite(cur) && dTurn > 60 * DEG && neu > cur * 0.7) return true;
    }
    if (this.path !== res.pts) this._pathSwitchT = this.sim.time;
    this.path = res.pts; this.pathIdx = 0;
    this.pathTight = !!res.tight;
    this.pathSource = 'A* (explore)';
    this.lastPlan = { t: this.sim.time, ms: res.ms, expanded: res.expanded, len: res.pts.length, buildMs: P.buildMs };
    this.sim.emit('path');
    return true;
  }

  // Teach & repeat: take the portion of any proven route that runs between near-start and
  // near-goal (either direction), joined to the exact start/goal with short A* stubs.
  routeFromMemory(goal) {
    const e = this.est();
    let best = null;
    for (const r of this.slam.routes) {
      if (!r.ok || r.pts.length < 20) continue;
      let i0 = -1, d0 = Infinity, i1 = -1, d1 = Infinity;
      r.pts.forEach((p, i) => {
        const a = (p.x - e.x) ** 2 + (p.z - e.z) ** 2, b = (p.x - goal.x) ** 2 + (p.z - goal.z) ** 2;
        if (a < d0) { d0 = a; i0 = i; } if (b < d1) { d1 = b; i1 = i; }
      });
      d0 = Math.sqrt(d0); d1 = Math.sqrt(d1);
      if (d0 > 150 || d1 > 150 || Math.abs(i1 - i0) < 25) continue;
      const seg = i0 < i1 ? r.pts.slice(i0, i1 + 1) : r.pts.slice(i1, i0 + 1).reverse();
      const score = d0 + d1;
      if (!best || score < best.score) best = { r, seg, d0, d1, score };
    }
    if (!best) return null;
    const P = this.planner;
    P.buildCostmap(this.slam.routes.filter((x) => x.ok));
    if (P.pathBlocked(best.seg, 0, best.seg.length) >= 0) return { rejected: best.r };
    const stubA = best.d0 > 6 ? P.plan(e.x, e.z, best.seg[0].x, best.seg[0].z) : { pts: [{ x: e.x, z: e.z }] };
    const last = best.seg[best.seg.length - 1];
    const stubB = best.d1 > 6 ? P.plan(last.x, last.z, goal.x, goal.z) : { pts: [last, { x: goal.x, z: goal.z }] };
    if (!stubA || !stubB) return { rejected: best.r };
    return { route: best.r, pts: P.resample([...stubA.pts, ...best.seg, ...stubB.pts], 1.0), d0: best.d0, d1: best.d1 };
  }

  // navigate to a goal using the global/local planners and route memory
  *navTo(goal, opts = {}) {
    const R = this.sim.rover;
    const tol = goal.tol ?? 2.0;
    const from = this.site(this.est().x, this.est().z), to = goal.site || this.site(goal.x, goal.z);
    this.goal = goal;
    yield* this.checkPower(goal.name);
    this.phase = `Navigate to ${goal.name || to}`;
    this.slam.setPayload(R.loaded || !!this.surveyLimits); // the first plan already needs this leg's limits
    let usingRoute = false;
    const mem = Math.hypot(goal.x - this.est().x, goal.z - this.est().z) > 150 ? this.routeFromMemory(goal) : null;
    if (mem && mem.pts) {
      this.path = mem.pts; this.pathIdx = 0; usingRoute = true;
      mem.route.uses++;
      this.pathSource = `REPEAT ${mem.route.id} (${mem.route.from}->${mem.route.to}, used ${mem.route.uses}x)`;
      this.log(`Route memory: reusing proven route ${mem.route.id} (${mem.route.from}->${mem.route.to}); joins ${mem.d0.toFixed(0)} m from here, leaves ${mem.d1.toFixed(0)} m from goal`, 'ok');
      this.sim.emit('path');
    } else if (mem && mem.rejected) this.log(`Route memory: ${mem.rejected.id} is now blocked - exploring a new route`, 'warn');
    if (!usingRoute && !this.replan(goal, 'initial')) {
      yield* this.wait(2, 'waiting for map data');
      if (!this.replan(goal, 'retry')) return { ok: false, why: 'no path to goal (check geofences / goal location)' };
    }
    if (from !== to) this.slam.beginRecord(from, to);
    let lastReplan = this.sim.time, stuckT = this.sim.time, stuckPos = this.est();
    let blockedT = 0, episodes = 0, unwinds = 0, pivotFails = 0, backoffs = 0, bestD = Infinity, bestT = this.sim.time, lastPivotPos = null, bestTiles = this.slam.tiles.size, t00 = this.sim.time;
    const fail = (why) => { if (this.slam.recording) this.slam.endRecord(false); this.stop(); return { ok: false, why }; };
    while (true) {
      const e = this.est();
      const dGoal = Math.hypot(goal.x - e.x, goal.z - e.z);
      if (dGoal < tol) break;
      // overall progress watchdog: 4 minutes without getting 5 m closer => ask for help
      if (dGoal < bestD - 5) { bestD = dGoal; bestT = this.sim.time; episodes = 0; }
      // a detour around a keep-out zone / crater field moves away from the goal while it maps new ground: that is progress,
      // but only while it stays a detour. (v1.5.0: unbounded, this let a rover that could no longer steer drive 1.3 km
      // away from PORT-1 "mapping new ground" without the watchdog ever firing)
      else if (this.slam.tiles.size > bestTiles + 6 && this.sim.time - t00 < 3600 && dGoal < bestD + Math.max(60, 0.1 * bestD)) { bestTiles = this.slam.tiles.size; bestT = this.sim.time; }
      if (this.sim.time - bestT > 240) return fail(`no progress toward ${goal.name || 'goal'} for 4 min: operator assistance required`);
      if (Number.isFinite(bestD) && dGoal > bestD + Math.max(150, 0.25 * bestD)) return fail(`moving away from ${goal.name || 'goal'} (${dGoal.toFixed(0)} m, was ${bestD.toFixed(0)} m): stopped, operator assistance required`);
      // steering-authority check: moving, asked to turn, and the gyro says the body is not turning
      { const f = R._cmdF, yr = Math.abs(this.sensors.imu.yawRate || 0);
        if (f && !(this.sim.warping && this.railsOk) && Math.abs(f.vx) > 0.5 && Math.abs(f.wz) > 0.04 && yr < 0.25 * Math.abs(f.wz)) this._noSteerT = (this._noSteerT || 0) + this.dt;
        else this._noSteerT = Math.max(0, (this._noSteerT || 0) - 2 * this.dt);
        if (this._noSteerT > 15) { this._noSteerT = 0; return fail('vehicle is not responding to steering (yaw rate < 25 % of command for 15 s): stopped, operator assistance required'); } }
      this.pathIdx = this.nearestOnPath(e);
      if (this._replanNow) { this._replanNow = false; lastReplan = -1e9; bestD = Infinity; stuckPos = e; }
      // (in transit warp the map changes per metre, not per second: replanning every 5 sim-s would cost ~10 plans per wall second)
      if (this.sim.time - lastReplan > (this.sim.warp.active ? 20 : usingRoute ? 8 : 5)) {
        lastReplan = this.sim.time;
        if (usingRoute) {
          this.planner.buildCostmap(this.slam.routes.filter((r) => r.ok));
          if (this.planner.pathBlocked(this.path, this.pathIdx, 150) >= 0) {
            this.log('Remembered route blocked by a new obstacle: replanning around it', 'warn');
            usingRoute = false;
            if (!this.replan(goal, 'route blocked')) { this.stop(); yield* this.wait(2, 'waiting for map / path'); }
          }
        } else this.replan(goal, 'periodic', true);
      }
      // lookahead target on the path
      let li = this.pathIdx;
      const look = clamp(7 + R.telemetry().speed * 4, 8, 25);
      let acc = 0;
      while (li < this.path.length - 1 && acc < look) { acc += Math.hypot(this.path[li + 1].x - this.path[li].x, this.path[li + 1].z - this.path[li].z); li++; }
      const tgt = this.path[li];
      const rel = this.relBody(tgt, e);
      const headErr = Math.atan2(-rel.r, rel.f);
      // Turning (v1.5.0): heading errors up to 115 deg are taken on a moving arc (the local planner's curvature, slowed
      // for the turn below), so the rover no longer stops and spins for every bend. A point turn is kept only for a
      // target essentially behind it (> 115 deg; > 150 deg within 6 m of the last one), or when no forward arc is free.
      const sinceTurn = Math.hypot(e.x - (lastPivotPos?.x ?? 1e9), e.z - (lastPivotPos?.z ?? 1e9));
      if (!R.loaded && Math.abs(headErr) > (sinceTurn < 6 ? 150 : 115) * DEG && dGoal > tol + 3 && Math.hypot(rel.f, rel.r) > 5) { // a loaded rover never point-turns: wheels would dig ruts and the CoM is 6.8 m up
        if (this.local.pivotSafe(e)) {
          lastPivotPos = { x: e.x, z: e.z };
          this.stop(); for (let i = 0; i < 5; i++) yield;
          const ok = yield* this.alignYaw(Math.atan2(-(tgt.z - e.z), tgt.x - e.x), 8 * DEG, { rails: true });
          if (!ok) { pivotFails++; if (pivotFails >= 3) return fail('vehicle stalled during pivot: operator assistance required'); yield* this.recover(goal, 'pivot stall'); }
          stuckT = this.sim.time; stuckPos = this.est();
          continue;
        }
      }
      // A loaded rover can't point-turn, so a goal inside its turning circle is unreachable going forward: it
      // orbits the goal (the arc tracker holds R ~ 9 m). Back straight up until the goal needs a gentler arc.
      if (R.loaded && dGoal < 15 && backoffs < 3) {
        const g = this.relBody(goal, e), kReq = 2 * Math.abs(g.r) / Math.max(1e-6, g.f * g.f + g.r * g.r), K_OK = 0.08; // 1 / 12.5 m
        // (no backoff when going straight on already passes within the arrival tolerance)
        if (kReq > 1.25 * K_OK && Math.abs(g.r) < 1 / K_OK && !(g.f > 0 && Math.abs(g.r) < 0.9 * tol)) {
          const b = clamp(Math.sqrt(Math.max(0, 2 * Math.abs(g.r) / K_OK - g.r * g.r)) - g.f, 3, 20);
          backoffs++;
          this.log(`${goal.name || 'goal'} is inside the loaded turning circle (${g.f.toFixed(1)} m ahead, ${g.r.toFixed(1)} m across): reversing ${b.toFixed(0)} m to re-approach`, 'info');
          yield* this.backUp(b);
          this.replan(goal, 'turning-circle backoff', true);
          stuckT = this.sim.time; stuckPos = this.est(); bestD = Infinity;
          continue;
        }
      }
      // speed governor
      // a survey (teach) drive plans and checks the ground with the loaded limits, so the route it records is one a
      // loaded rover can repeat (point turns and empty speeds stay available: see surveyRoute)
      const loaded = R.loaded || !!this.surveyLimits;
      this.slam.setPayload(loaded);
      // tilt reflex: the ground ahead is steeper than the map said. Stop, mark it as a no-go, back out and replan.
      { const tiltDeg = Math.max(Math.abs(R.tilt.pitch), Math.abs(R.tilt.roll)) * 180 / Math.PI;
        if (tiltDeg > (loaded ? 14 : 17) && this.sim.time - (this._tiltT || -99) > 20) {
          this._tiltT = this.sim.time;
          this.log(`Tilt reflex: ${tiltDeg.toFixed(0)} deg (limit ${loaded ? 14 : 17}): steeper than mapped, marking slope, backing out`, 'warn');
          for (const f of [4, 9, 14]) this.planner.blocked.push({ x: e.x + Math.cos(e.yaw) * f, z: e.z - Math.sin(e.yaw) * f, r: 5, t: this.sim.time, ttl: 900 });
          this.stop(); yield* this.recover(goal, 'steep slope'); lastReplan = 0; continue;
        } }
      let v = Math.min(this.speedSetting, R.speedCap());
      if (this.sensors.faults.lidar) v = Math.min(v, usingRoute ? 0.3 : 0);
      { const mg = R.tilt.marginF ?? R.tilt.margin; if (mg < 22) v *= clamp((mg - 8) / 14, 0.3, 1); if (loaded) v = Math.min(v, 0.6 + Math.max(0, mg - 10) * 0.12); }
      if (R.wheels.some((w) => w.fault)) v *= 0.75;
      v *= this.sim.health ? this.sim.health.fx.speedK : 1; // FDIR derates (hub cooling, strut leak, battery string)
      // cargo shifted in the cradle (load-cell asymmetry): half speed until the cradle has re-centred it
      if (R.lift.shifted) {
        v *= 0.5;
        if (!this._shiftWarned) { this._shiftWarned = true; this.log('Cargo shift detected: speed derated 50% until the cradle re-centres the module', 'warn'); }
      } else if (this._shiftWarned) { this._shiftWarned = false; this.log('Cargo re-centred: speed derate lifted', 'ok'); }
      if (this.pathTight) v = Math.min(v, 0.6);
      v = Math.min(v, Math.max(0.3, Math.sqrt(2 * 0.12 * Math.max(0, dGoal - tol))));
      if (v <= 0) { this.stop(); this.detail = 'LiDAR failed off-route: holding for operator'; yield; continue; }
      // transit warp (>= 50x): a long cruise leg whose next 30 m are mapped and clear runs on rails (Sim.warpStep)
      { const vR = this._railsCheck(e, dGoal, v, loaded);
        this.railsOk = vR > 0;
        if (this.railsOk && this.sim.warping) {
          this._railsAdvance(vR, this.dt);
          this.detail = `TRANSIT WARP ${vR.toFixed(1)} m/s | ${this.pathSource} | ${dGoal.toFixed(0)} m to go`;
          stuckT = this.sim.time; stuckPos = this.est(); blockedT = 0;
          yield; continue;
        } }
      let lp = this.local.evaluate(e, tgt, v, loaded);
      // the gentle-turn arc set (|k| b <= 0.35-0.5) is the normal choice; in a tight spot try the full skid-steer arc
      // set at walking pace before declaring a block (v1.5.0: without this, the gentler arcs turned squeezes into
      // 25 m reverse "unwinds")
      if (!lp.ok) { const lp2 = this.local.evaluate(e, tgt, Math.min(v, 0.5), loaded, { kmax: loaded ? 0.11 : 0.2 }); if (lp2.ok) { lp = lp2; v = Math.min(v, 0.5); } }
      if (!lp.ok) {
        this.stop();
        const firstBlock = !blockedT;
        if (!blockedT) blockedT = this.sim.time;
        this.detail = 'local planner: no safe arc - learning obstacle and replanning';
        // put the actual obstacle points the arcs ran into into the global costmap
        if (!loaded) for (const h of lp.hits.slice(0, 30)) this.planner.blocked.push({ x: h.x, z: h.z, r: 1.6, t: this.sim.time, ttl: 180 });
        else if (firstBlock) {
          // loaded: the arcs sample only a narrow fan (~+-3 m at 7 m) and the 2.5 m costmap window reads a short
          // 11-12 deg face under the 10.5 deg limit, so A* side-stepped a few metres into the same face; and with a
          // 180 s memory it came back to it once the marks expired (east-west shuttling for minutes). Learn the hits
          // once per block for 15 min and sweep the wheel-scale hazard over the ground ahead (CHANGES.md §14)
          for (const h of lp.hits.slice(0, 30)) this.planner.blocked.push({ x: h.x, z: h.z, r: 1.6, t: this.sim.time, ttl: 900 });
          const c = Math.cos(e.yaw), s = Math.sin(e.yaw);
          for (let f = 3; f <= 24; f += 1) for (let l = -15; l <= 15; l += 1) {
            const px = e.x + c * f + s * l, pz = e.z - s * f + c * l, wh = this.slam.wheelHazard(px, pz);
            if (wh && wh.lethal) this.planner.blocked.push({ x: px, z: pz, r: 0.9, t: this.sim.time, ttl: 900 });
          }
        }
        if (this.sim.time - blockedT > 1.0) {
          episodes++; blockedT = 0;
          if (episodes > 8) return fail('boxed in by obstacles: operator assistance required');
          if (episodes === 1) this.log('Local planner blocked: obstacle learned into the map, replanning', 'warn');
          if (episodes >= 2) {
            // boxed in: a replan alone cannot get out (or reverse arc is blocked), so back out along the way in
            if (++unwinds > 4) return fail('boxed in again after 4 reverse recoveries: operator assistance required');
            yield* this.unwindPath(25);
            episodes = 0; bestD = Infinity; bestT = this.sim.time; stuckT = this.sim.time; stuckPos = this.est(); usingRoute = false;
          } else if (!this.replan(goal, 'local block', true)) yield* this.recover(goal, 'blocked');
          lastReplan = this.sim.time;
        }
        yield; continue;
      }
      blockedT = 0;
      // Two stopping-distance limits (braking at `dec` below, the same value the dynamics instruments use): stop 3 m
      // short of any known hazard, and never outrun the mapped ground - unseen terrain can hide a crater rim
      const dec = loaded ? 0.2 : 0.6; // loaded: under the ~0.4 m/s^2 nose-over limit of the passive pitch (CHANGES.md §14)
      let vCmd = Math.min(v, Math.max(0.3, Math.sqrt(2 * dec * Math.max(0, lp.freeDist - 3))));
      vCmd = Math.min(vCmd, Math.max(0.3, Math.sqrt(2 * dec * Math.max(0, lp.knownDist - 1.5))));
      { // tilt-aware speed: slow on tilted ground and so that the curve's lateral acceleration fits the remaining tip margin
        const mgn = R.tilt.marginF ?? R.tilt.margin, tiltDeg = Math.max(Math.abs(R.tilt.pitch), Math.abs(R.tilt.roll)) * 180 / Math.PI;
        const aOK = MOON.g * Math.tan(Math.min(60, Math.max(0.5, mgn - 9)) * DEG);
        vCmd = Math.min(vCmd, Math.max(0.35, Math.sqrt(aOK / Math.max(Math.abs(lp.k), 0.004))));
        // ride comfort / no skid in curves (v1.5.0): lateral acceleration <= 0.3 m/s^2 empty, 0.15 loaded (a skid-steer
        // turn scrubs the tyres sideways; slower = less scrub), and slow into a bend in proportion to the heading change
        vCmd = Math.min(vCmd, Math.max(0.5, Math.sqrt((loaded ? 0.15 : 0.3) / Math.max(Math.abs(lp.k), 0.002))));
        vCmd = Math.min(vCmd, Math.max(0.6, vCmd * (0.35 + 0.65 * Math.max(0, Math.cos(headErr)))));
        if (loaded) {
          // ground slope under the footprint from the map, not the body attitude: braking pitches the loaded body
          // forward (passive pitch), and slowing on that pitch braked it further into a nose-over (CHANGES.md §14)
          const c = Math.cos(e.yaw), s = Math.sin(e.yaw), H = (f, l) => this.slam.heightInterp(e.x + c * f + s * l, e.z - s * f + c * l);
          const gp = (H(4, 0) - H(-4, 0)) / 8, gr = (H(0, 3.5) - H(0, -3.5)) / 7;
          const gDeg = Number.isFinite(gp + gr) ? Math.atan(Math.hypot(gp, gr)) * 180 / Math.PI : tiltDeg;
          vCmd = Math.min(vCmd, Math.max(0.4, 3 - 0.22 * gDeg));
        }
      }
      // turbo (300 rpm) is only released on a straight, long, mapped, obstacle-free stretch
      R.turboOk = !loaded && !this.pathTight && Math.abs(lp.k) < 0.012 && lp.freeDist >= lp.horizon - 1 && lp.knownDist > 25 && dGoal > 50;
      this.cmd(vCmd, 0, vCmd * lp.k);
      this.detail = `${R.rpm.proto.toFixed(0)} rpm ${R.rpm.tier} | ${this.pathSource} | ${dGoal.toFixed(0)} m to go | v ${vCmd.toFixed(2)} m/s${lp.knownDist < 6 ? ' (creeping: unmapped ground ahead)' : ''} | ${Math.abs(lp.k) < 0.004 ? "straight" : `turn ${lp.k > 0 ? "L" : "R"} R=${Math.abs(1 / lp.k).toFixed(0)} m`}`;
      // stuck detection (embedding / slip / hidden obstacle)
      if (this.sim.time - stuckT > 20) {
        const moved = Math.hypot(e.x - stuckPos.x, e.z - stuckPos.z);
        if (moved < 1.0) {
          this.log('No progress for 20 s (slip or unseen obstacle): marking area and recovering', 'warn');
          this.planner.blocked.push({ x: e.x + Math.cos(e.yaw) * 9, z: e.z - Math.sin(e.yaw) * 9, r: 2.5, t: this.sim.time, ttl: 180 });
          yield* this.recover(goal, 'stuck');
        }
        stuckT = this.sim.time; stuckPos = this.est();
      }
      yield;
    }
    this.stop();
    if (this.slam.recording) {
      const r = this.slam.endRecord(true);
      if (r) this.log(`Route memory: stored ${r.id} ${r.from}->${r.to} (${r.length.toFixed(0)} m, ${(r.time / 60).toFixed(1)} min)`, 'ok');
    }
    this.planner.blocked = this.planner.blocked.filter((b) => b.fault);
    if (goal.yaw != null) {
      // a loaded rover avoids point turns (CoM ~7 m up, wheels dig in); small heading errors are left to the
      // arc / line-following controller of the next leg
      const yErr = Math.abs(wrap(goal.yaw - this.est().yaw));
      if (!R.loaded || yErr > 6 * DEG) yield* this.alignYaw(goal.yaw, goal.yawTol ?? 1.5 * DEG);
    }
    return { ok: true };
  }

  // ---------------- transit warp (Sim.warpStep) ----------------
  // Cruise speed the rails may use, or 0 when this stretch must run on full physics: short of the goal, a tight path,
  // a shifted load, a low tip margin, scout out, or anything lethal / unmapped under the six wheel tracks in the
  // next 30 m of path. Cheap enough to run every tick (it is skipped below the warp time scale).
  _railsCheck(e, dGoal, v, loaded) {
    const S = this.sim, R = S.rover;
    if (!S.warpWanted()) return 0;
    if (dGoal < 45 || this.pathTight || !this.path || R.lift.shifted || R.estop || !R.powered || S.scout.enabled || this.sensors.faults.lidar) return 0;
    if ((R.tilt.marginF ?? R.tilt.margin) < 20) return 0;
    const P = this.path;
    let acc = Math.hypot(P[this.pathIdx].x - e.x, P[this.pathIdx].z - e.z);
    if (acc > 6) return 0; // off the path (recovering, or the plan just changed): let the tracker rejoin it
    const half = Math.abs(ROVER.corners[0][1]);
    for (let i = this.pathIdx; i < P.length - 1 && acc < 30; i++) {
      const a = P[i], b = P[i + 1], L = Math.hypot(b.x - a.x, b.z - a.z);
      if (L < 1e-6) continue;
      const s = -(b.z - a.z) / L, c = (b.x - a.x) / L; // right vector of the segment = (sin yaw, cos yaw)
      for (const off of [0, -half, half]) {
        const wh = this.slam.wheelHazard(a.x + s * off, a.z + c * off);
        if (!wh) { if (acc < 12 && !this.slam.pinhole(a.x + s * off, a.z + c * off)) return 0; continue; }
        if (wh.lethal) return 0;
      }
      acc += L;
    }
    return clamp(v, 0, loaded ? 2.4 : 5.0); // (empty was 3.4; v is already capped by speedCap and the governors)
  }
  // Move the estimate along the path by v*h and the true body by the same body-frame motion (so the SLAM frame
  // offset is preserved), feeding the odometry the SLAM filter integrates in Sim.warpStep.
  _railsAdvance(v, h) {
    const R = this.sim.rover, P = this.path, e = this.est();
    // projection of the estimate on the segment at pathIdx (or the one before it)
    let i = Math.min(this.pathIdx, P.length - 2), t = 0;
    for (const j of [i - 1, i]) {
      if (j < 0) continue;
      const a = P[j], b = P[j + 1], dx = b.x - a.x, dz = b.z - a.z, L2 = dx * dx + dz * dz;
      const tt = L2 > 0 ? clamp(((e.x - a.x) * dx + (e.z - a.z) * dz) / L2, 0, 1) : 0;
      if (j === i - 1 && tt < 1) { i = j; t = tt; break; }
      if (j === i) t = tt;
    }
    const walk = (i0, t0, d) => {
      let k = i0, x = P[k].x + (P[k + 1].x - P[k].x) * t0, z = P[k].z + (P[k + 1].z - P[k].z) * t0;
      while (d > 0 && k < P.length - 1) {
        const bx = P[k + 1].x, bz = P[k + 1].z, L = Math.hypot(bx - x, bz - z);
        if (L >= d) { x += (bx - x) * d / L; z += (bz - z) * d / L; d = 0; break; }
        d -= L; x = bx; z = bz; k++;
      }
      return { x, z, k: Math.min(k, P.length - 2) };
    };
    const n = walk(i, t, v * h), ahead = walk(n.k, 0, Math.hypot(P[n.k].x - n.x, P[n.k].z - n.z) + 4);
    const want = Math.atan2(-(ahead.z - n.z), ahead.x - n.x);
    // heading follows the path tangent, limited to the turning radius (loaded R >= 12 m, empty R >= 5 m)
    const maxTurn = v * h / (R.loaded ? 12 : 5);
    const yawN = e.yaw + clamp(Number.isFinite(want) ? wrap(want - e.yaw) : 0, -maxTurn, maxTurn);
    const dyaw = wrap(yawN - e.yaw), c = Math.cos(yawN), s = Math.sin(yawN);
    const Dx = n.x - e.x, Dz = n.z - e.z;
    const fx = Dx * c - Dz * s, rz = Dx * s + Dz * c;      // body-frame step (forward, right) in the new heading
    const T = R.pose(), yT = T.yaw + dyaw, cT = Math.cos(yT), sT = Math.sin(yT);
    R.railsPlace(T.x + fx * cT + rz * sT, T.z - fx * sT + rz * cT, yT, h, v);
    const o = this.sensors.odom, y1 = R.pose().y;
    o.dx += fx; o.dz += rz; o.dyaw += dyaw; o.dy += y1 - T.y; o.dist += Math.hypot(Dx, Dz); o.vx = v; o.vz = 0;
    const im = this.sensors.imu; im.yawRate = dyaw / h; im.roll = R.tilt.roll; im.pitch = R.tilt.pitch;
    this._railsMoved = true;
  }

  // Recovery: back out along the best collision-free reverse arc, then replan.
  *recover(goal, why) {
    const e = this.est();
    const target = { x: e.x - Math.cos(e.yaw) * 10, z: e.z + Math.sin(e.yaw) * 10 };
    let lp = this.local.evaluate(e, target, 0.3, this.sim.rover.loaded, { dir: -1, horizon: 8, kmax: 0.12, relaxed: true });
    if (!lp.ok) {
      for (const ang of [-0.35, 0.35, -0.7, 0.7]) {
        const y = e.yaw + ang;
        const tgt = { x: e.x - Math.cos(y) * 10, z: e.z + Math.sin(y) * 10 };
        const alt = this.local.evaluate(e, tgt, 0.3, this.sim.rover.loaded, { dir: -1, horizon: 8, kmax: 0.12, relaxed: true });
        if (alt.ok) { lp = alt; break; }
      }
    }
    const phase0 = this.phase.replace(/ \(recovery\)$/, '');
    this.phase = phase0 + ' (recovery)';
    if (lp.ok) {
      const s0 = this.est(), t0 = this.sim.time;
      while (Math.hypot(this.est().x - s0.x, this.est().z - s0.z) < Math.min(5, lp.freeDist - 1) && this.sim.time - t0 < 40) {
        this.cmd(-0.3, 0, -0.3 * lp.k); this.detail = `recovery (${why}): reversing on clear arc`; yield;
      }
      this.stop();
    } else this.log(`Recovery (${why}): no clear reverse arc either`, 'warn');
    for (let i = 0; i < 8; i++) yield;
    this.replan(goal, 'recovery', true);
    this.phase = phase0;
  }

  // Reverse-recovery for a boxed-in rover (CHANGES.md §17). When the local planner cannot find a forward path
  // (e.g. boxed in by boulders or crater rims), the vehicle backs out along the breadcrumbs it drove in on (the proven
  // entry trajectory, which is clear because it just drove it), at 0.15 m/s. The rear arcs are checked every tick by
  // the local planner in reverse (the hazard-camera view astern); if one is blocked the unwind stops there. The corridor
  // ahead that boxed it in is marked lethal for 1200 s and the global planner is forced to replan from the new, open position.
  *unwindPath(distance = 25) {
    const R = this.sim.rover, P = this.planner, goal = this.goal;
    const phase0 = this.phase.replace(/ \((recovery|unwind)\)$/, '');
    this.phase = phase0 + ' (unwind)';
    this.unwindCount = (this.unwindCount || 0) + 1;
    const e0 = this.est(), TTL = 1200, now0 = this.sim.time;
    // 1. the blocking corridor: the hit points the arcs ran into, and a 4.5 m wide strip along the heading
    for (const h of (this.local.last?.hits || []).slice(0, 30)) P.blocked.push({ x: h.x, z: h.z, r: 1.6, t: now0, ttl: TTL });
    for (let f = 3; f <= 30; f += 3) P.blocked.push({ x: e0.x + Math.cos(e0.yaw) * f, z: e0.z - Math.sin(e0.yaw) * f, r: 4.5, t: now0, ttl: TTL });
    // 2. the way in: breadcrumbs from the newest backwards until `distance` m of path (only recent ones: older ones
    // may belong to another leg)
    const trace = [{ x: e0.x, z: e0.z }];
    let len = 0;
    for (let i = this.slam.trajectory.length - 1; i >= 0 && len < distance; i--) {
      const b = this.slam.trajectory[i];
      if (now0 - b.t > 900) break;
      const a = trace[trace.length - 1], d = Math.hypot(b.x - a.x, b.z - a.z);
      if (d < 1.0) continue;
      trace.push({ x: b.x, z: b.z }); len += d;
    }
    this.log(`Boxed in: unwinding ${Math.min(len, distance).toFixed(0)} m back along the entry trajectory at ${R.loaded ? 0.15 : 0.4} m/s, corridor ahead marked lethal for ${TTL} s`, 'warn');
    let moved = 0, ok = len >= 6, proven = 0;
    if (!ok) this.log('Unwind: no usable entry trajectory (breadcrumbs too short or too old)', 'warn');
    // empty: 0.4 m/s along its own fresh track (was 0.15: a 25 m unwind took ~3 min and dominated crater-field
    // exploration); loaded stays at 0.15 m/s (CoM 6.8 m up)
    const vU = R.loaded ? 0.15 : 0.4;
    const t0 = this.sim.time, tMax = Math.min(distance, len) / vU * 1.6 + 20;
    let ti = 1;
    while (ok && moved < Math.min(distance, len) - 1 && this.sim.time - t0 < tMax) {
      const e = this.est();
      moved = Math.hypot(e.x - e0.x, e.z - e0.z);
      // trace point to steer to: nearest one ahead of the progress so far, 5 m further back along the trace
      while (ti < trace.length - 1 && Math.hypot(trace[ti].x - e.x, trace[ti].z - e.z) < 5) ti++;
      const lp = this.local.evaluate(e, trace[ti], vU, R.loaded, { dir: -1, horizon: 8, kmax: 0.12, relaxed: true });
      let k = lp.k;
      if (!lp.ok) {
        // the rear arcs read blocked, but the trace is ground the wheels have just crossed: a crater rim mapped from
        // inside the pocket looks steeper than it drove (map 22-27 deg vs 14-17 deg true in tests/faults.mjs), so
        // follow the trace itself (pure pursuit, 0.15 m/s) while the tilt margin holds, and stop if it does not
        const c = Math.cos(e.yaw), s = Math.sin(e.yaw), tx = trace[ti].x - e.x, tz = trace[ti].z - e.z;
        const rT = tx * s + tz * c, d2 = Math.max(4, tx * tx + tz * tz);
        const mgn = R.tilt.marginF ?? R.tilt.margin;
        if (mgn < 10) { this.log('Unwind: tilt margin too small to keep reversing, stopping here', 'warn'); break; }
        k = Math.max(-0.12, Math.min(0.12, -2 * rT / d2));
        proven++;
      }
      this.cmd(-vU, 0, -vU * k);
      this.detail = `unwind: reversing ${moved.toFixed(1)} / ${Math.min(distance, len).toFixed(0)} m on the entry trajectory`;
      yield;
    }
    this.stop();
    for (let i = 0; i < 8; i++) yield;
    this._replanNow = true;
    if (goal) this.replan(goal, 'unwind', true);
    this.phase = phase0;
  }

  rearClear(e, ahead = 2.5) {
    // check the strip swept by the rear of the vehicle before reversing
    // only the wheel tracks matter: the straddle bay passes over low objects (cargo, saddles)
    const c = Math.cos(e.yaw), s = Math.sin(e.yaw);
    const hl = ROVER.frame.halfLen;
    for (let x = -hl; x >= -hl - ahead; x -= 0.5) for (const zz of [-4.4, -3.84, -3.3, 3.3, 3.84, 4.4]) {
      const wh = this.slam.wheelHazard(e.x + x * c + zz * s, e.z - x * s + zz * c);
      if (wh && wh.lethal) return false;
    }
    return true;
  }
  *backUp(dist) {
    const s0 = this.est(); const phase0 = this.phase.replace(/ \(recovery\)$/, ''); this.phase = phase0 + ' (recovery)';
    const t0 = this.sim.time;
    while (Math.hypot(this.est().x - s0.x, this.est().z - s0.z) < dist && this.sim.time - t0 < 60) {
      if (!this.rearClear(this.est())) { this.stop(); this.log('Reverse stopped: obstacle behind the vehicle', 'warn'); break; }
      this.cmd(-0.6, 0, 0); this.detail = 'recovery: reversing (rear strip clear)'; yield;
    }
    this.stop();
    this.phase = phase0;
  }

  // ---------------- cargo acquisition ----------------
  cargoGuess(c) {
    let g = this.cargoEst.get(c.id);
    if (!g) { // offload report from the lander (accurate to ~1.5 m / 3 deg)
      g = { x: c.initial.x + 0.9, z: c.initial.z - 0.7, yaw: c.initial.yaw + 0.03, t: -1, src: 'manifest' };
      this.cargoEst.set(c.id, g);
    }
    return g;
  }

  *acquire(c, opts = {}) {
    const R = this.sim.rover, T = c.T;
    try { return yield* this._acquire(c, opts); } finally { this.slam.lcSuspended = false; }
  }
  // AEGIS-V1 pickup (module_carry.js): kneel, drive along the module axis under it until the clamp bands are
  // over the rings, RISE so the V-saddles lift it off the stand's lug notches, then close the ring jaws.
  *_acquire(c, opts = {}) {
    const R = this.sim.rover, T = c.T, RD = ROVER.ride;
    if (R.loaded) return { ok: false, why: 'already carrying cargo' };
    const aft = Math.min(-T.length / 2, (T.adapterX || 0) - (T.adapterLen || 0) / 2);
    const carry = T.carryX || 0;
    const waitRide = function* (self, target, label) {
      R.lift.target = target;
      while (Math.abs(R.lift.pos - target) > 0.01) { self.stop(); self.detail = `${label}: ride ${R.lift.pos.toFixed(2)} -> ${target.toFixed(2)} m`; yield; }
      for (let i = 0; i < 20; i++) { self.stop(); yield; }
    };
    for (let attempt = 1; attempt <= 3; attempt++) {
      let g = this.cargoGuess(c);
      // stand-off point on the module axis behind its aft end (rover front 14 m short of it)
      const pre = (gg, gap) => { const back = -aft + ROVER.frame.halfLen + gap - carry; return { x: gg.x - Math.cos(gg.yaw) * back, z: gg.z + Math.sin(gg.yaw) * back }; };
      this.phase = `M2 Approach ${c.id}`;
      const r = yield* this.navTo({ ...pre(g, 16), name: `pre-grasp ${c.id}`, tol: 2.0, site: 'PAD' });
      if (r && r.ok === false) return r;
      this.slam.lcSuspended = true; // the scene around the cargo is about to change (we move it)
      this.phase = `M2 Locate ${c.id} (fiducials)`;
      let t0 = this.sim.time;
      while (this.sim.time - t0 < 1.5) { this.stop(); this.detail = `cargo estimate from ${this.cargoGuess(c).src}`; yield; }
      g = this.cargoGuess(c);
      if (g.src === 'manifest') this.log(`${c.id}: no fiducial fix yet, using lander offload report`, 'warn');
      else this.detail = `cargo fix +-${Math.sqrt(g.varP).toFixed(2)} m, +-${(Math.sqrt(g.varY) / DEG).toFixed(2)} deg`;
      // line up on the axis (skid steer: turn, drive, turn)
      const p1 = pre(g, 12);
      this.phase = `M2 Line up on ${c.id}`;
      if (Math.hypot(p1.x - this.est().x, p1.z - this.est().z) > 0.6) {
        const e = this.est();
        yield* this.alignYaw(Math.atan2(-(p1.z - e.z), p1.x - e.x), 2 * DEG);
        const t1 = this.sim.time;
        while (this.sim.time - t1 < 120) {
          const rel = this.relBody(pre(this.cargoGuess(c), 12)); // re-aim as the fiducial fix sharpens
          if (rel.f < 0.15) break;
          this.cmd(clamp(rel.f * 0.5, 0.2, 1.8), 0, clamp(-Math.atan2(rel.r, Math.max(1, rel.f)) * 0.6, -0.08, 0.08));
          this.detail = `lining up on the module axis (${rel.f.toFixed(1)} m)`;
          yield;
        }
        this.stop();
      }
      this.phase = `M2 Square up to ${c.id}`;
      yield* this.alignYaw(this.cargoGuess(c).yaw, 0.6 * DEG);
      // kneel so the deck and saddles pass under the module on its stand
      this.phase = `M2 Kneel (${(RD.kneel).toFixed(2)} m)`;
      yield* waitRide(this, RD.kneel, 'kneeling');
      // drive under: fiducials, then the saddle rangefinders + band-edge sensors
      this.phase = `M2 Drive under ${c.id}`;
      let ok = false; t0 = this.sim.time; let axF = null;
      while (this.sim.time - t0 < 400) {
        g = this.cargoGuess(c);
        const e = this.est();
        const rel = this.relBody(g, e);
        // offsets from the module AXIS (not from its centre): rel is the centre in the rover frame, yawErr the axis
        // heading relative to the rover, so the centre's sideways offset alone hides along * sin(yawErr)
        let yawErr = wrap(g.yaw - e.yaw), src = 'camera';
        let lat = rel.f * Math.sin(yawErr) + rel.r * Math.cos(yawErr), along = rel.f * Math.cos(yawErr) - rel.r * Math.sin(yawErr) - carry;
        const cass = this.sensors.cass;
        if (cass && cass.valid) {
          lat = cass.lateral; yawErr = -cass.yaw; src = 'saddle rangefinders';
          if (cass.cargoX != null) { axF = axF == null ? cass.cargoX : axF + (cass.cargoX - axF) * 0.3; along = axF - carry; src += ' + band sensors'; }
        }
        if (Math.abs(along) < 0.12 && Math.abs(lat) < 0.15 && Math.abs(yawErr) < 1.5 * DEG) {
          ok = true; break;
        }
        if (along < -0.5) break; // overshoot
        if (!(cass && cass.valid) && along < -aft + ROVER.frame.halfLen - carry + 0.5 && Math.abs(lat) > 0.25) { this.log(`${c.id}: lateral ${lat.toFixed(2)} m at the module end (tyre-to-post clearance ~0.9 m)`, 'warn'); break; }
        // (steering is curvature, i.e. per metre, so speed doesn't change the line the deck follows; the creep only
        // needs to be slow over the last ~0.5 m where the band sensors stop it inside +-0.12 m)
        const v = Math.min(along > 8 ? 1.5 : along > 2 ? 0.7 : clamp(along * 0.35, 0.06, 0.4), Math.abs(lat) > 0.12 ? 0.35 : 9); // slow while correcting lateral
        // skid-steer line following: head toward the axis, then along it
        // curvature steering (yaw rate scales with speed, so a skid-steer creep never scrubs in place)
        // intercept the axis at no more than ~8 deg so a large offset can't swing the deck into a stand post
        const kap = clamp(0.35 * (yawErr - clamp(Math.atan(0.6 * lat), -0.14, 0.14)), -1 / 6, 1 / 6);
        this.cmd(v, 0, v * kap);
        this.detail = `[${src}] along ${along.toFixed(2)} m  lat ${(lat * 100).toFixed(1)} cm  yaw ${(yawErr / DEG).toFixed(2)} deg`;
        yield;
      }
      this.stop();
      for (let i = 0; i < 15; i++) yield;
      if (!ok) {
        this.log(`${c.id}: not centred under the module (attempt ${attempt}), backing out to retry`, 'warn');
        yield* this.backUp(-aft + ROVER.frame.halfLen + 4);
        continue;
      }
      if (opts.approachOnly) { this.log(`${c.id}: rover centred under the module, ready to lift`, 'ok'); return { ok: true }; }
      // rise: saddles touch at the contact ride, weight transfers, the lugs lift clear of the notches
      this.phase = `M3 Rise under ${c.id}`;
      const y0 = c.body.translation().y;
      yield* waitRide(this, RD.carry, 'rising (roller-screw struts)');
      const lifted = c.body.translation().y - y0;
      if (lifted < 0.3) { this.log(`${c.id}: module did not lift off the stand (${lifted.toFixed(2)} m)`, 'warn'); yield* waitRide(this, RD.kneel, 'kneeling'); yield* this.backUp(-aft + ROVER.frame.halfLen + 4); continue; }
      this.phase = `M3 Clamp ${c.id}`;
      const res = R.clampClose();
      if (!res.ok) { this.log(`Clamp attempt ${attempt} refused: ${res.why}`, 'warn'); yield* waitRide(this, RD.kneel, 'kneeling'); yield* this.backUp(-aft + ROVER.frame.halfLen + 4); continue; }
      while (R.lift.clamp === 'closing') { this.detail = `ring jaws closing ${(R.lift.clampT * 100).toFixed(0)}%  (lugs ${lifted.toFixed(2)} m above the notches)`; yield; }
      // (was: retried straight away with the module still riding unclamped on the raised saddles, so the next attempt's
      // approach drive carried it off its stand. Set it back down and back out first, like the other failure paths.)
      if (R.lift.clamp !== 'closed') { this.log(`Clamp attempt ${attempt} failed: ${R.lift.msg}`, 'warn'); while (R.lift.clamp === 'opening') yield; yield* waitRide(this, RD.kneel, 'kneeling'); yield* this.backUp(-aft + ROVER.frame.halfLen + 4); continue; }
      yield* this.wait(0.5, 'verifying load path');
      c.state = 'held';
      // the module is now part of the vehicle: forget its footprint in the terrain map
      { const e2 = this.est(); const half = (T.adapterR || T.radius) + 0.4;
        this.slam.clearFootprint(e2.x + Math.cos(e2.yaw) * carry, e2.z - Math.sin(e2.yaw) * carry, e2.yaw, -aft + 0.6, half); }
      const st = this.manifest.find((m) => m.cargo === c); if (st) st.status = 'in transit';
      this.log(`${c.id} lifted ${lifted.toFixed(2)} m off its stand and clamped (${(T.mass * MOON.g / 1000).toFixed(1)} kN on the saddles)`, 'ok');
      return { ok: true };
    }
    return { ok: false, why: 'pickup failed after 3 attempts' };
  }

  // Mission 3 stability check: start / stop / turn with the load
  *stabilityCheck() {
    const R = this.sim.rover;
    this.phase = 'M3 Load stability check';
    // module position in the rover frame (cradle load cells / ring encoders): how far it moved during the test
    const inCradle = () => {
      const c = R.lift.cargo; if (!c) return null;
      const p = R.pose(), t = c.body.translation();
      return new THREE.Vector3(t.x - p.x, t.y - p.y, t.z - p.z).applyQuaternion(p.q.clone().invert());
    };
    const m0 = inCradle();
    const seq = [[0.5, 0, 0, 6], [0, 0, 0, 3], [0.5, 0, 0.04, 6], [0, 0, 0, 3], [-0.4, 0, 0, 5], [0, 0, 0, 3]];
    let minMargin = 99, maxShift = 0;
    for (const [vx, vz, wz, t] of seq) {
      const t0 = this.sim.time;
      while (this.sim.time - t0 < t) {
        this.cmd(vx, vz, wz); minMargin = Math.min(minMargin, R.tilt.margin);
        const m = inCradle(); if (m && m0) maxShift = Math.max(maxShift, m.distanceTo(m0));
        this.detail = `start/stop/turn test  tip margin ${R.tilt.margin.toFixed(1)} deg`; yield;
      }
    }
    this.stop();
    // (was: `R.lift.cargo ? 0 : 1`, a constant, never a measurement)
    const shift = m0 ? maxShift : NaN;
    this.log(`Load check: min tip margin ${minMargin.toFixed(1)} deg, cargo shift ${(shift * 100).toFixed(1)} cm - ${minMargin > 15 ? 'PASS' : 'MARGINAL'}`, minMargin > 15 ? 'ok' : 'warn');
    return { ok: true };
  }

  // ---------------- delivery & docking ----------------
  portGuess(port) {
    let g = this.portEst.get(port.id);
    if (!g) { g = { x: port.x, z: port.z, y: port.y, nYaw: Math.atan2(-port.nz, port.nx), t: -1, src: 'base map' }; }
    return g;
  }

  *deliver(portId) {
    try { return yield* this._deliver(portId); } finally { this.slam.lcSuspended = false; }
  }
  *_deliver(portId) {
    const R = this.sim.rover;
    const port = this.sim.ports.find((p) => p.id === portId);
    if (!R.loaded) return { ok: false, why: 'no cargo held' };
    const c = R.lift.cargo, T = c.T;
    if (this.faults.port[portId]) {
      this.log(`${port.name} reported UNAVAILABLE - holding at staging and requesting operator decision`, 'warn');
      const H = this.sim.terrain.hab;
      yield* this.navTo({ x: port.x + H.nx * 45, z: port.z + H.nz * 45, name: 'hab holding point', tol: 3 });
      this.phase = 'HOLD: receiving port unavailable';
      while (this.faults.port[portId] && c.dest === portId) { this.detail = `waiting: "clear port ${portId}" or "reroute ${c.id.includes('40') ? 'A' : 'B'} <port>"`; this.stop(); yield; }
      // operator redirected the module to another port while we held
      if (c.dest !== portId) { this.log(`${c.id} rerouted by operator: PORT-${portId} -> PORT-${c.dest}`, 'ok'); return yield* this._deliver(c.dest); }
    }
    const frontOff = () => R.lift.base.x + T.length / 2; // cargo front end ahead of rover centre
    const pg = this.portGuess(port);
    const n = { x: Math.cos(pg.nYaw), z: -Math.sin(pg.nYaw) };
    const heading = wrap(pg.nYaw + Math.PI);
    const pre = { x: pg.x + n.x * (frontOff() + 24), z: pg.z + n.z * (frontOff() + 24) };
    this.phase = `M4 Transport ${c.id} -> ${port.name}`;
    // lead-in point 14 m further out on the port axis: the grid planner ignores heading, so without it the path
    // reaches the pre-dock point from the side and a loaded rover (no point turns, R >= 9 m) can't line up there
    const lead = { x: pre.x + n.x * 14, z: pre.z + n.z * 14 };
    { // skip the lead-in only when already lined up on the axis between it and the pre-dock point
      const eL = this.est(), ax = (eL.x - pre.x) * n.x + (eL.z - pre.z) * n.z, off = Math.abs((eL.x - pre.x) * n.z - (eL.z - pre.z) * n.x);
      if (!(ax > 6 && ax < 16 && off < 2 && Math.abs(wrap(eL.yaw - heading)) < 10 * DEG)) {
        const rl = yield* this.navTo({ ...lead, name: `lead-in ${port.name}`, tol: 3.0, site: 'HAB' });
        if (rl && rl.ok === false) return rl;
      }
    }
    const r = yield* this.navTo({ ...pre, name: `pre-dock ${port.name}`, tol: 2.0, site: 'HAB', yaw: heading, yawTol: 1.0 * DEG, straight: true });
    if (r && r.ok === false) return r;
    // re-target on the port fiducial (the base-map position is offset by any SLAM drift)
    { const t0 = this.sim.time; while (this.sim.time - t0 < 1.5) { this.stop(); this.detail = 'acquiring port fiducial'; yield; }
      const g2 = this.portGuess(port);
      if (g2.t > 0) {
        const n2 = { x: Math.cos(g2.nYaw), z: -Math.sin(g2.nYaw) };
        const pre2 = { x: g2.x + n2.x * (frontOff() + 24), z: g2.z + n2.z * (frontOff() + 24) };
        const off = Math.hypot(pre2.x - this.est().x, pre2.z - this.est().z), eF = this.est();
        // the final approach line-follows the port axis from 24 m out; only a large cross-axis or heading
        // error needs a repositioning manoeuvre (a loaded rover has no point turns)
        const crossAx = Math.abs((eF.x - pre2.x) * n2.z - (eF.z - pre2.z) * n2.x), yawOff = Math.abs(wrap(wrap(g2.nYaw + Math.PI) - eF.yaw));
        // (the far-field leg steers on the rover's offset from the port axis and absorbs a few metres: an S-turn
        // at R 12 m needs ~2*sqrt(12*offset) of the 24 m leg; repositioning a 2.9 m offset put the goal inside
        // the loaded turning circle and the rover circled until the 4 min watchdog)
        if (off > 1.0 && (!R.loaded || crossAx > 3.5 || yawOff > 10 * DEG)) {
          this.log(`Port fiducial: pre-dock corrected by ${off.toFixed(1)} m (SLAM drift), repositioning`, 'info');
          const r2 = yield* this.navTo({ ...pre2, name: `pre-dock ${port.name} (fiducial)`, tol: 0.8, site: 'HAB', yaw: wrap(g2.nYaw + Math.PI), yawTol: 0.8 * DEG });
          if (r2 && r2.ok === false) return r2;
        }
      } else this.log(`${port.name} fiducial not seen yet: approaching on the base map`, 'warn');
    }
    // final approach: skid-steer line following on the port axis (port fiducial, then the docking-ring
    // camera inside 10 m). Lateral error is removed by heading while there is still distance to do it.
    this.slam.lcSuspended = true; // docking area changes as modules are berthed
    const RD = ROVER.ride;
    let captured = false, contactF = 0;
    this._dockPeakN = 0;
    for (let attempt = 1; attempt <= 3 && !captured; attempt++) {
      this._dockAttempts = attempt;
      this.phase = `M5 Final approach ${port.name}`;
      let t0 = this.sim.time, bad = false, vAppF = null, latA = null; this._wzI = 0;
      while (this.sim.time - t0 < 400) {
        const g = this.portGuess(port);
        const e = this.est();
        const ring = this.sensors.cass?.ring;
        const nn = { x: Math.cos(g.nYaw), z: -Math.sin(g.nYaw) };
        const target = { x: g.x + nn.x * frontOff(), z: g.z + nn.z * frontOff() };
        const rel = this.relBody(target, e);
        let lat = rel.r, along = rel.f, yawErr = wrap(wrap(g.nYaw + Math.PI) - e.yaw), src = 'port fiducial';
        if (ring && ring.port === port && ring.along < 10) {
          along = ring.along; src = 'docking-ring camera';
          if (ring.yaw != null) yawErr = ring.yaw; // module axis vs port axis, measured on the ring itself
          // ring.lateral is the port centre's offset in the MODULE frame (a bearing): it includes along*sin(yaw).
          // The nose's distance from the port AXIS is what the steering must null (driving in 1.8 deg off-axis
          // the two cancelled, the reading sat at -5 cm while the nose crossed the axis, and every attempt failed)
          lat = ring.lateral * Math.cos(yawErr) + ring.along * Math.sin(yawErr);
          // ride trim toward the ring height: filtered (~1 s) and slow (0.25 /s). Adding 8 % of the raw reading
          // every tick swung the ride 0.57-0.98 m, and the tilting module showed up as lateral drift at the nose
          vAppF = vAppF == null ? ring.vertical : vAppF + (ring.vertical - vAppF) * clamp(this.dt / 1.0, 0, 1);
          if (Math.abs(vAppF) > 0.003) R.lift.target = clamp(R.lift.target + vAppF * 0.25 * this.dt, RD.carry - 0.4, RD.max);
        }
        // cone check on a ~1 s filtered lateral: the loaded module rocks (box20: +-0.6 deg pitch) and single
        // samples spiked past 6 cm while the mean sat within 2 cm, aborting every attempt
        latA = latA == null ? lat : latA + (lat - latA) * clamp(this.dt / 1.0, 0, 1);
        if (along < 1.2 && Math.abs(latA) > 0.06) { bad = true; break; }
        if (along < 0.35) break;
        const vx = along > 10 ? 1.2 : clamp((along - 0.3) * 0.25, 0.06, 0.8);
        // Inside the ring camera's range: lat is the nose's offset from the port axis, L = frontOff ahead of the
        // turning centre. Per metre  d lat/ds = -yaw + L*kap,  d yaw/ds = -kap, so the turning-centre offset
        // yc = lat + L*yaw obeys d yc/ds = -yaw and  kap = 2p*yaw - p^2*yc  puts both poles at -p per metre
        // (yc, yaw -> 0 means the nose arrives on the axis AND square). Unlike a nose-point law this stays stable
        // for any assumed control point at or ahead of the turning centre, so an error in L can't destabilise it.
        // p = 0.6: ~1.7 % of the hand-over error left after the 10 m camera leg.
        const near = src === 'docking-ring camera', P = 0.6;
        // far field (port fiducial + SLAM pose): the same law on the rover centre's offset from the port axis line
        // (a bearing-pursuit law here handed over at 2-4 deg / ~0.4 m off-axis and the near leg couldn't absorb it);
        // gentler p = 0.3 per metre over the ~14 m before the camera takes over
        const hd = wrap(g.nYaw + Math.PI), ycFar = (target.x - e.x) * Math.sin(hd) + (target.z - e.z) * Math.cos(hd);
        const kap = near ? clamp(2 * P * yawErr - P * P * (lat + frontOff() * yawErr), -1 / 12, 1 / 12)
          : clamp(2 * 0.3 * yawErr - 0.09 * ycFar, -1 / 12, 1 / 12);
        this.cmdCurv(vx, kap);
        this.detail = `[${src}] range ${along.toFixed(2)} m  lat ${(lat * 100).toFixed(1)} cm  yaw ${(yawErr / DEG).toFixed(2)} deg  ride ${R.lift.pos.toFixed(2)} m`;
        yield;
      }
      this.stop();
      for (let i = 0; i < 20; i++) yield;
      if (bad) { this.log(`${port.name}: lateral error too large for the coupling cone (attempt ${attempt}), backing off`, 'warn'); yield* this.backUp(14); continue; }
      // soft contact: creep at 1 cm/s with the ride trimmed to the ring until the coupling latches engage
      this.phase = `M5 Dock ${port.name}`;
      t0 = this.sim.time;
      // Continuous creep (never stopping: with the drive at zero the spring brakes set and the ~10 m high
      // module nose then sways 1-2 cm in yaw on the locked tyres without decaying). Ring lateral / vertical are
      // low-pass filtered (~1 s) against that sway; lateral is closed by heading over the remaining gap (a yaw
      // change moves the nose by frontOff x dyaw), vertical by trimming the ride while moving. The port face is
      // flat: the module end touches it rim-first unless square, and gentle push (< maxPushN) squares it up.
      let latF = null, vertF = null, yawF = null, lostT = 0; this._wzI = 0;
      // Force-impedance contact (CHANGES.md §17): inside 0.1 m the speed follows the contact force, v = clamp(Kf (Ft - F), -0.02, 0.015),
      // so the module settles on Ft = 1200 N (between softCaptureN 900 and maxPushN 6000) instead of creeping at a fixed rate.
      // First contact (> 200 N) ramps the speed target to zero within 0.1 s, so the tyres' kinetic energy cannot overshoot
      // maxPushN; the latches fire only after the ride trim has held the ring inside +-5 mm for 1.5 s.
      const IMP = { Ft: 1200, Kf: 0.00001, vMax: 0.015, vMin: -0.02, vTouch: 0.0008, touchN: 200, ramp: 0.1, tol: 0.005, dwell: 1.5 };
      let touchT = null, alignedT = 0, peakN = 0, fF = 0, spiked = false;
      while (this.sim.time - t0 < 180) {
        const ring = this.sensors.cass?.ring;
        if (!ring) {
          // target lost (camera fault / occlusion): hold briefly, then abort instead of creeping blind
          this.stop(); lostT += this.dt; this.detail = `docking camera has no target (${lostT.toFixed(1)} s)`;
          if (lostT > 5) { this.log(`${port.name}: docking camera lost the port ring for 5 s, aborting docking`, 'err'); R.brake = false; yield* this.backUp(8); return { ok: false, why: 'docking visual sensor lost' }; }
          yield; continue;
        }
        lostT = 0;
        const a = clamp(this.dt / 0.8, 0, 1);
        const yNow = ring.lateral * Math.cos(ring.yaw ?? 0) + ring.along * Math.sin(ring.yaw ?? 0); // nose offset from the port axis
        latF = latF == null ? yNow : latF + (yNow - latF) * a;
        vertF = vertF == null ? ring.vertical : vertF + (ring.vertical - vertF) * a;
        yawF = yawF == null ? (ring.yaw ?? 0) : yawF + ((ring.yaw ?? 0) - yawF) * a;
        if (Math.abs(vertF) > 0.002) R.lift.target = clamp(R.lift.target + vertF * 0.3 * this.dt, RD.carry - 0.4, RD.max);
        const f = this.sim.cargoForces.get(c);
        contactF = f ? f.port : 0;
        peakN = Math.max(peakN, contactF); this._dockPeakN = Math.max(this._dockPeakN || 0, peakN);
        fF += (contactF - fF) * clamp(this.dt / 0.1, 0, 1);   // 0.1 s low-pass: the controller sees the force, not the contact-solver spikes
        const now = this.sim.time;
        if (contactF > IMP.touchN && touchT == null) touchT = now;
        else if (touchT != null && fF < 0.5 * IMP.touchN) touchT = null;   // contact lost: the ramp re-arms for the next touch
        // The loaded rover rocks in pitch (~0.2 deg, ~5 s period on the tyres), which bobs the module nose +-3 cm. Like a real
        // soft-capture system the latches only fire with the ring sensors inside the capture envelope for a settling time.
        const aligned = Math.abs(vertF) < IMP.tol && Math.abs(latF) < IMP.tol;
        let v;
        if (ring.along > 0.1) v = 0.06;                                   // closing the gap, no contact possible
        else {
          // a rigid port face stops the 15 t module inside one 1/120 s physics step: the peak force is ~ m v 120, so the
          // speed that reaches the face must be ~1 mm/s for the peak to stay < 3 kN. The impedance law's own 12 mm/s at zero
          // force is therefore capped by the remaining gap until contact is registered (it is the cap, not the law, that
          // sets the touch speed)
          const vGap = Math.max(IMP.vTouch, Math.min(IMP.vMax, 0.5 * Math.max(0, ring.along - 0.01)));   // last 1 cm at the touch speed (0.5 x gap alone allowed 1.5 mm/s at the face)
          v = clamp(IMP.Kf * (IMP.Ft - fF), IMP.vMin, vGap);
          if (touchT != null && now - touchT < IMP.ramp) v *= clamp(1 - (now - touchT) / IMP.ramp, 0, 1);
        }
        if (contactF > DOCK.maxPushN) v = Math.min(v, IMP.vMin / 2);       // backstop: never keep pushing past the structural limit
        // same nose-point law as the approach (see above), on the filtered ring readings
        const P = 0.6, kap = clamp(2 * P * yawF - P * P * (latF + frontOff() * yawF), -1 / 12, 1 / 12);
        // The law is per metre driven: at 1 mm/s it removes ~0.1 % of the error per second, so the last centimetres of lateral
        // error never close. Inside 0.12 m, while the nose is still more than 4 mm off the port axis, advance at 0.8 mm/s
        // (brakes off) and steer on a time basis, as if driving 2 cm/s (yaw rate <= 1.7 mrad/s, i.e. <= 10 mm/s of nose motion); from 0.40 m to 0.12 m it does so at 1 cm/s.
        if (ring.along > -0.05 && ring.along <= 0.40 && Math.abs(latF) > 0.003 && fF < IMP.touchN) {
          // capture needs the NOSE inside +-5 mm, so null the nose offset itself: with the drive near standstill d lat/dt = frontOff * yawRate,
          // hence yawRate = -k lat / frontOff (first order, 10 s time constant). Steering the turning-centre offset instead (the law above)
          // left the nose 14-28 mm off: a 0.05 deg yaw wander moves a nose 8 m ahead of the centre by 7 mm, and the old branch also stopped steering inside 3 cm.
          const wDes = clamp(-0.1 * latF / frontOff(), -0.002, 0.002), err = wDes - this.sensors.imu.yawRate;
          const vA = ring.along > 0.12 ? 0.01 : 0.0008;
          this._wzI = clamp((this._wzI || 0) + 1.2 * err * this.dt, -0.003, 0.003);
          this.cmd(vA, 0, wDes + clamp(2.0 * err + this._wzI, -0.003, 0.003));
          v = vA;
        } else this.cmdCurv(v, Math.sign(v) * kap);
        this.detail = `gap ${ring.along.toFixed(3)} m | lat ${(latF * 1000).toFixed(1)} mm | vert ${(vertF * 1000).toFixed(1)} mm | yaw ${(yawF / DEG).toFixed(2)} deg | contact ${contactF.toFixed(0)} N (F ${fF.toFixed(0)} -> ${IMP.Ft}) v ${(v * 1000).toFixed(1)} mm/s`;
        // dwell: in contact at capture force and aligned, continuously
        if (contactF > 2500 && !spiked && (spiked = true)) this.log(`Dock contact spike ${contactF.toFixed(0)} N: gap ${ring.along.toFixed(3)} m, cmd ${(v * 1000).toFixed(1)} mm/s, filtered F ${fF.toFixed(0)} N, lat ${(latF * 1000).toFixed(1)} mm, wheel speed ${(R.telemetry().fwdSpeed * 1000).toFixed(1)} mm/s`, 'info');
        // (the 1.5 s count runs on the ring alignment alone: the contact force of a rigid port face bounces between 0 and the
        // target, and the latches then fire at the first sample above the soft-capture force once the dwell is over)
        if (ring.along < 0.03 && aligned) alignedT += this.dt; else alignedT = 0;
        if (alignedT >= IMP.dwell && ring.along < 0.015 && fF > DOCK.softCaptureN && Math.abs(ring.lateral) < IMP.tol && Math.abs(ring.vertical) < IMP.tol) { captured = true; break; } // the latching sample itself is inside the envelope too
        yield;
      }
      this.stop(); R.brake = true;
      if (!captured) { this.log(`Soft capture attempt ${attempt} failed, backing off`, 'warn'); R.brake = false; yield* this.backUp(14); }
    }
    if (!captured) { this.log('Soft capture failed after 3 attempts', 'err'); return { ok: false, why: 'soft capture failed' }; }
    const ring = this.sensors.cass.ring;
    const acc = { port: port.name, lateralMm: ring.lateral * 1000, verticalMm: ring.vertical * 1000, yawDeg: 0, contactN: contactF, peakN: this._dockPeakN || 0, attempts: this._dockAttempts || 1 };
    // yaw error measured between cargo axis and port normal (from docking camera)
    { const rq = c.body.rotation(); const q = new THREE.Quaternion(rq.x, rq.y, rq.z, rq.w); const ax = new THREE.Vector3(1, 0, 0).applyQuaternion(q); acc.yawDeg = wrap(Math.atan2(-ax.z, ax.x) - Math.atan2(port.nz, -port.nx)) / DEG; }
    this.metrics.accuracy.push(acc);
    this.sim.dockCapture(c, port);
    this.log(`SOFT CAPTURE ${port.name}: lateral ${acc.lateralMm.toFixed(1)} mm, vertical ${acc.verticalMm.toFixed(1)} mm, yaw ${acc.yawDeg.toFixed(2)} deg`, 'ok');
    // berthing sequence: pressure boundary sealed before the module hatch opens
    const steps = [
      ['Hard capture: latches retract, seal compressed 30 mm', 1.5],
      ['Dust-exclusion coupling: spring-loaded metal bellows extending', 2],
      ['Electrodynamic dust shield pulse + brush-seal sweep of interface', 2.5],
      ['Leak check: vestibule pressure-decay test', 3],
      ['Equalising vestibule to 70.3 kPa', 3],
      ['Receiving cradle jacks extending under module', 2.5],
      ['Module hatch opening', 2],
    ];
    this.docking = { port, cargo: c, step: 0, steps: steps.map((s) => s[0]), pressure: 0, leakRate: 0 };
    for (let i = 0; i < steps.length; i++) {
      this.docking.step = i;
      const t1 = this.sim.time;
      while (this.sim.time - t1 < steps[i][1]) {
        const k = (this.sim.time - t1) / steps[i][1];
        if (i === 3) { this.docking.pressure = 0.5; this.docking.leakRate = 0.0021 * (1 - k) + 0.0006; }
        if (i === 4) this.docking.pressure = 0.5 + k * 69.8;
        if (i === 5) this.sim.extendJacks(port, k);
        this.phase = `M5 Berthing ${port.name}`; this.detail = `${i + 1}/${steps.length} ${steps[i][0]}`;
        this.stop(); yield;
      }
      if (i === 3) this.log(`Leak check PASS: decay ${this.docking.leakRate.toFixed(4)} kPa/min (limit 0.01)`, 'ok');
    }
    this.sim.openHatch(port);
    this.docking.step = steps.length;
    // release and back out
    this.phase = `M5 Release ${c.id}`;
    const rel = R.clampOpen();
    if (!rel.ok) return { ok: false, why: rel.why };
    while (R.lift.clamp === 'opening') { this.detail = `cradle opening ${(R.lift.clampT * 100).toFixed(0)}%`; yield; }
    c.state = 'delivered'; c.deliveredAt = this.sim.time;
    port.occupied = c;
    const ms = this.manifest.find((m) => m.cargo === c); if (ms) ms.status = 'delivered';
    this.metrics.deliveries++;
    this.phase = `M5 Kneel clear of ${c.id}`;
    R.lift.target = ROVER.ride.kneel;
    while (Math.abs(R.lift.pos - ROVER.ride.kneel) > 0.01) { this.stop(); this.detail = `kneeling: ride ${R.lift.pos.toFixed(2)} m (module on the receiving stand)`; yield; }
    R.brake = false;
    this.phase = 'Back out from dock';
    yield* this.backUp(frontOffSafe(T) + ROVER.frame.halfLen + 6);
    R.lift.target = 0; // back to driving ride height
    this.docking = null;
    this.log(`${c.id} DELIVERED to ${port.name}`, 'ok');
    return { ok: true };
    function frontOffSafe(TT) { return TT.length / 2 + 6; }
  }

  // ---------------- Mission 11: AI logistics manager ----------------
  logisticsPlan() {
    const R = this.sim.rover, e = this.est();
    const items = this.manifest.filter((m) => m.status === 'waiting' && m.cargo.state === 'staged');
    const H = WORLD.habitat, P = WORLD.pad;
    const route = Math.hypot(H.x - P.x, H.z - P.z) * 1.15;
    const plan = items.map((m) => {
      const v = ROVER.speedLimitLoaded * 0.8;
      const toCargo = Math.hypot(m.cargo.initial.x - e.x, m.cargo.initial.z - e.z) * 1.2;
      const eta = (toCargo / 1.2 + route / v + 900) / 60;            // minutes incl. handling
      const energy = (route * (m.cargo.T.mass + R.mass) * MOON.g * 0.09 / 0.86 + toCargo * R.mass * MOON.g * 0.09 / 0.86) / 3600 + 1.1 * eta * 60 / 3600 * 1000; // Wh
      const weight = PRIORITY[m.category] ?? m.priority;
      const score = weight * 100 - eta * 0.8 - energy / 500;
      return { ...m, eta, energy, score, feasible: energy < (R.power.wh - this.reserveSoc * ROVER.battery.capacityWh) };
    });
    plan.sort((a, b) => b.score - a.score);
    return plan;
  }

  // ---------------- mission scripts ----------------
  *missionLeaveLanding() {
    const S = this.sim.terrain.pad.staging, d = this.sim.terrain.pad.dir;
    this.phase = 'M1 Leave landing zone';
    const r = yield* this.navTo({ x: S.x, z: S.z, name: 'staging area', tol: 2.5, site: 'STAGING', yaw: Math.atan2(-d.z, d.x) });
    return r;
  }
  *missionApproach(c) { return yield* this.acquire(c, { approachOnly: true }); }
  *missionPickup(c) {
    const r = yield* this.acquire(c);
    if (r.ok === false) return r;
    return yield* this.stabilityCheck();
  }
  *missionTransport(portId) {
    const R = this.sim.rover;
    if (!R.loaded) return { ok: false, why: 'pick up cargo first (Mission 3)' };
    const port = this.sim.ports.find((p) => p.id === portId);
    const H = this.sim.terrain.hab;
    return yield* this.navTo({ x: port.x + H.nx * 40, z: port.z + H.nz * 40, name: `habitat approach ${port.name}`, tol: 3, site: 'HAB' });
  }
  *missionDeliver(portId) { return yield* this.deliver(portId); }
  // A proven route (teach & repeat) that joins within 150 m of `a` and of `b`
  provenRoute(a, b) {
    const near = (r, q) => r.pts.some((p) => Math.hypot(p.x - q.x, p.z - q.z) < 150);
    return this.slam.routes.find((r) => r.ok && r.pts.length >= 20 && near(r, a) && near(r, b)) || null;
  }

  // Survey (teach) drive. The first loaded leg to the habitat crosses ~2 km of unmapped crater field: a loaded rover
  // (no point turns, 10.5 deg slope limit) only finds each rim as it reaches it and got boxed in between rims until
  // the progress watchdog fired (CHANGES.md §14). So the empty rover drives the leg first, under the loaded
  // traversability limits, mapping it and recording a proven route; the loaded leg then repeats it (CHANGES.md §15).
  *surveyRoute(cargo) {
    const port = this.sim.ports.find((p) => p.id === cargo.dest);
    if (!port || this.sim.rover.loaded) return { ok: true };
    const pg = this.portGuess(port), n = { x: Math.cos(pg.nYaw), z: -Math.sin(pg.nYaw) };
    // end facing the port, so its fiducials are in the camera view for the drift fix below
    const goal = { x: pg.x + n.x * 45, z: pg.z + n.z * 45, name: `survey: ${port.name} approach`, tol: 4, site: 'HAB', yaw: wrap(pg.nYaw + Math.PI), yawTol: 2 * DEG };
    const start = this.est();
    if (Math.hypot(goal.x - start.x, goal.z - start.z) < 300 || this.provenRoute(cargo.initial ?? start, goal)) return { ok: true };
    const phase0 = this.phase;
    this.log(`Survey drive: mapping the route to ${port.name} empty, under the loaded limits, before carrying ${cargo.id}`, 'ok');
    this.surveyLimits = true;
    let r;
    try {
      r = yield* this.navTo(goal);
      if (r && r.ok === false) { this.log(`Survey drive to ${port.name} failed (${r.why}): the loaded leg will explore`, 'warn'); return { ok: true }; }
      // Hold on the port fiducials until the map-frame fix lands. It moves the map, the breadcrumbs and the route just
      // recorded back along the trajectory (Slam.shiftFrame), so they match the surveyed layout instead of carrying
      // the ~1 % odometry drift of the 4 km drive (CHANGES.md §16)
      // The port tags (0.55 m) are only readable inside ~47 m and the rover arrives with up to ~35 m of drift, so it may
      // be well outside that: after a short hold it creeps toward the port (never inside the 25 m fix exclusion) until
      // a fix lands, 60 s at most.
      const fix0 = this.slam.lastFix?.t ?? -1, tH = this.sim.time;
      while (this.sim.time - tH < 60 && (this.slam.lastFix?.t ?? -1) <= fix0) {
        const e = this.est(), g = this.portGuess(port), d = Math.hypot(g.x - e.x, g.z - e.z);
        if (this.sim.time - tH < 6 || d < 32) { this.stop(); this.detail = 'survey: holding on the port fiducials for a drift fix'; }
        else { this.cmd(0.6, 0, 0); this.detail = `survey: creeping toward ${port.name} to pick up its fiducials (${d.toFixed(0)} m)`; }
        yield;
      }
      this.stop();
      if ((this.slam.lastFix?.t ?? -1) > fix0) this.log(`Survey route corrected by the port fiducials: ${Math.hypot(this.slam.lastFix.dx, this.slam.lastFix.dz).toFixed(1)} m of drift removed along the route`, 'ok');
      else this.log('Survey: no fiducial fix at the port approach - the route keeps its odometry drift', 'warn');
      // the drive back is an ordinary empty drive (the pad apron itself reads lethal under the loaded limits)
      this.surveyLimits = false;
      const S = this.sim.terrain.pad.staging;
      r = yield* this.navTo({ x: S.x, z: S.z, name: 'survey: return to pad', tol: 4, site: 'STAGING' });
      if (r && r.ok === false) return r;
    } finally { this.surveyLimits = false; this.phase = phase0; }
    const rt = this.provenRoute(cargo.initial ?? start, goal);
    this.log(rt ? `Survey drive done: route ${rt.id} (${rt.length.toFixed(0)} m) proven for the loaded leg` : 'Survey drive done, but no route was recorded', rt ? 'ok' : 'warn');
    return { ok: true };
  }

  *missionCycle() {
    const plan = this.logisticsPlan();
    if (!plan.length) return { ok: false, why: 'no cargo waiting at the pad' };
    this.log(`Logistics manager order: ${plan.map((p) => `${p.id}[${p.category}]`).join(' -> ')}`, 'ok');
    for (const item of plan) {
      const m = this.manifest.find((x) => x.id === item.id);
      if (m.status !== 'waiting') continue;
      const legStart = { t: this.sim.time, d: this.sim.rover.odo.dist, e: this.sim.rover.power.energyUsedWh };
      // the plan's energy check: a leg it marks infeasible starts with a recharge (the dock can't charge a
      // loaded rover, so this is the last chance before the pick-up)
      if (!item.feasible && !this.sim.rover.loaded) {
        this.log(`${item.id}: leg needs ~${(item.energy / 1000).toFixed(0)} kWh above the ${(this.reserveSoc * 100).toFixed(0)}% reserve - recharging first`, 'warn');
        const rc = yield* this.chargeAtDock(0.9); if (rc && rc.ok === false) return rc;
      }
      let r = yield* this.surveyRoute(item.cargo); if (r.ok === false) return r;
      r = yield* this.acquire(item.cargo); if (r.ok === false) return r;
      r = yield* this.deliver(item.cargo.dest); if (r.ok === false) return r; // dest, not the plan copy: "reroute" may change it
      this.metrics.legs.push({ id: item.id, time: this.sim.time - legStart.t, dist: this.sim.rover.odo.dist - legStart.d, energyWh: this.sim.rover.power.energyUsedWh - legStart.e });
    }
    const S = this.sim.terrain.pad.staging;
    this.phase = 'M6 Return for next load';
    yield* this.navTo({ x: S.x, z: S.z, name: 'pad staging (ready for next load)', tol: 3, site: 'STAGING' });
    return { ok: true };
  }
  *missionGoto(x, z, name) {
    const res = yield* this.navTo({ x, z, name: name || `(${x.toFixed(0)}, ${z.toFixed(0)})`, tol: 2.5 });
    if (res && res.ok === false) return res;
    // Approaching habitat apron / base: ensure fiducials are acquired to collapse long-traverse odometry drift
    if (name === 'hab' || (name && name.toLowerCase().includes('hab')) || this.site(x, z) === 'HAB') {
      const H = this.sim.terrain.hab;
      const habYaw = wrap(Math.atan2(H.nz, -H.nx));
      if (!this.sim.rover.loaded) yield* this.alignYaw(habYaw, 4 * DEG);
      const fix0 = this.slam.lastFix?.t ?? -1, tH = this.sim.time;
      while (this.sim.time - tH < 60 && (this.slam.lastFix?.t ?? -1) <= fix0) {
        const e = this.est(), d = Math.hypot(WORLD.habitat.x - e.x, WORLD.habitat.z - e.z);
        if (this.sim.time - tH < 5 || d < 35) {
          this.stop(); this.detail = 'hab arrival: holding for fiducial fix';
        } else {
          this.cmd(0.6, 0, 0); this.detail = 'hab arrival: creeping to acquire habitat fiducials';
        }
        yield;
      }
      this.stop();
    }
    return { ok: true };
  }
}
