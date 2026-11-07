// Mission executive. Behaviours are generators: each tick the executive advances the active
// generator one step, so mission scripts read like the procedure they implement.
// All decisions use estimated (SLAM) state and sensor measurements, never ground truth.
import * as THREE from 'three';
import { ROVER, WORLD, DOCK, MOON, POWER_DOCK } from './config.js';
import { Planner, LocalPlanner } from './planner.js';

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
    if (d.range > 45 || !d.known) return;
    if (this.slam.cellsKnown > 30000) { this.lastBeacon = { id: d.id, t: this.sim.time, range: d.range, advisory: true }; return; }
    const m = this.toEst(d.pos);
    const e = this.slam.est;
    const w = 0.12 * d.confidence;
    e.x += (d.known.x - m.x) * w; e.z += (d.known.z - m.z) * w;
    const nYaw = this.dirToEst(d.normal) + d.yawNoise;
    const dy = wrap(Math.atan2(-d.known.nz, d.known.nx) - nYaw);
    if (d.range < 30) e.yaw += dy * w * 0.5;
    this.lastBeacon = { id: d.id, t: this.sim.time, range: d.range };
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
        if (!prev || prev.src === 'manifest') this.cargoEst.set(c.id, { ...center, yaw: axisYaw, t: this.sim.time, src: d.id, range: d.range, varP, varY });
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
    this.updateEstimates();
    const R = this.sim.rover;
    if (R.estop || !R.powered) { if (this.gen && !this.paused) { this.paused = true; this.mode = 'MANUAL'; } return; }
    if (this.mode !== 'AUTO' || !this.gen || this.paused) return;
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
    this.log(`Low battery (${(R.power.soc * 100).toFixed(0)}%): going to the power dock before ${label}`, 'warn');
    const r = yield* this.chargeAtDock(0.9);
    if (r && r.ok === false) {
      // dock unreachable: fall back to waiting (rover has no arrays of its own)
      this.phase = 'Waiting for power';
      while (R.power.soc < this.reserveSoc + 0.15 && !(R.power.dockW > 0)) { this.stop(); R.brake = true; this.detail = `SoC ${(R.power.soc * 100).toFixed(1)}%: power dock unreachable (${r.why}), operator assistance required`; yield; }
      R.brake = false;
    }
  }

  // Drive onto the power dock pad, kneel onto the underbody coupler and charge (CC-CV) to `target` SoC.
  *chargeAtDock(target = 0.95) {
    const R = this.sim.rover, PD = this.sim.powerDock, L = PD.layout;
    if (R.loaded) return { ok: false, why: 'cannot charge while carrying cargo' };
    // goals are in the SLAM frame: add the current estimate offset
    const off = () => { const p = R.pose(), e = this.est(); return { x: e.x - p.x, z: e.z - p.z }; };
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

  *alignYaw(yaw, tol = 1.2 * DEG) {
    const R = this.sim.rover;
    let t0 = this.sim.time, noRate = 0, ok = true;
    while (true) {
      const e = this.est();
      const err = wrap(yaw - e.yaw);
      if (Math.abs(err) < tol && Math.abs(this.sensors.imu.yawRate) < 0.004) break;
      const wmax = R.loaded ? 0.07 : 0.11;
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
    const routes = this.slam.routes.filter((r) => r.ok);
    P.buildCostmap(routes);
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
      if (Number.isFinite(cur) && neu > cur * 0.85) return true;
    }
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
    let blockedT = 0, episodes = 0, pivotFails = 0, bestD = Infinity, bestT = this.sim.time, lastPivotPos = null, bestTiles = this.slam.tiles.size, t00 = this.sim.time;
    const fail = (why) => { if (this.slam.recording) this.slam.endRecord(false); this.stop(); return { ok: false, why }; };
    while (true) {
      const e = this.est();
      const dGoal = Math.hypot(goal.x - e.x, goal.z - e.z);
      if (dGoal < tol) break;
      // overall progress watchdog: 4 minutes without getting 5 m closer => ask for help
      if (dGoal < bestD - 5) { bestD = dGoal; bestT = this.sim.time; episodes = 0; }
      // a detour around a keep-out zone / crater field moves away from the goal while it maps new ground: that is progress
      else if (this.slam.tiles.size > bestTiles + 6 && this.sim.time - t00 < 3600) { bestTiles = this.slam.tiles.size; bestT = this.sim.time; }
      if (this.sim.time - bestT > 240) return fail(`no progress toward ${goal.name || 'goal'} for 4 min: operator assistance required`);
      this.pathIdx = this.nearestOnPath(e);
      if (this.sim.time - lastReplan > (usingRoute ? 8 : 5)) {
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
      const look = clamp(7 + R.telemetry().speed * 5, 8, 15);
      let acc = 0;
      while (li < this.path.length - 1 && acc < look) { acc += Math.hypot(this.path[li + 1].x - this.path[li].x, this.path[li + 1].z - this.path[li].z); li++; }
      const tgt = this.path[li];
      const rel = this.relBody(tgt, e);
      const headErr = Math.atan2(-rel.r, rel.f);
      // large heading error -> point turn (skid-free on the 4 steered bogies) if the sweep is clear
      const sinceTurn = Math.hypot(e.x - (lastPivotPos?.x ?? 1e9), e.z - (lastPivotPos?.z ?? 1e9));
      if (!R.loaded && Math.abs(headErr) > (sinceTurn < 6 ? 100 : 50) * DEG && dGoal > tol + 3 && Math.hypot(rel.f, rel.r) > 5) { // a loaded rover never point-turns: wheels would dig ruts and the CoM is 6.8 m up
        if (this.local.pivotSafe(e)) {
          lastPivotPos = { x: e.x, z: e.z };
          this.stop(); for (let i = 0; i < 5; i++) yield;
          const ok = yield* this.alignYaw(Math.atan2(-(tgt.z - e.z), tgt.x - e.x), 8 * DEG);
          if (!ok) { pivotFails++; if (pivotFails >= 3) return fail('vehicle stalled during pivot: operator assistance required'); yield* this.recover(goal, 'pivot stall'); }
          stuckT = this.sim.time; stuckPos = this.est();
          continue;
        }
      }
      // speed governor
      const loaded = R.loaded;
      this.slam.setPayload(loaded);
      // tilt reflex: the ground ahead is steeper than the map said. Stop, mark it as a no-go, back out and replan.
      { const tiltDeg = Math.max(Math.abs(R.tilt.pitch), Math.abs(R.tilt.roll)) * 180 / Math.PI;
        if (tiltDeg > (loaded ? 12 : 17) && this.sim.time - (this._tiltT || -99) > 20) {
          this._tiltT = this.sim.time;
          this.log(`Tilt reflex: ${tiltDeg.toFixed(0)} deg (limit ${loaded ? 12 : 17}): steeper than mapped, marking slope, backing out`, 'warn');
          for (const f of [4, 9, 14]) this.planner.blocked.push({ x: e.x + Math.cos(e.yaw) * f, z: e.z - Math.sin(e.yaw) * f, r: 5, t: this.sim.time, ttl: 900 });
          this.stop(); yield* this.recover(goal, 'steep slope'); lastReplan = 0; continue;
        } }
      let v = Math.min(this.speedSetting, R.speedCap());
      if (this.sensors.faults.lidar) v = Math.min(v, usingRoute ? 0.3 : 0);
      { const mg = R.tilt.marginF ?? R.tilt.margin; if (mg < 22) v *= clamp((mg - 8) / 14, 0.3, 1); if (loaded) v = Math.min(v, 0.6 + Math.max(0, mg - 10) * 0.12); }
      if (R.wheels.some((w) => w.fault)) v *= 0.75;
      if (this.pathTight) v = Math.min(v, 0.6);
      v = Math.min(v, Math.max(0.3, Math.sqrt(2 * 0.12 * Math.max(0, dGoal - tol))));
      if (v <= 0) { this.stop(); this.detail = 'LiDAR failed off-route: holding for operator'; yield; continue; }
      const lp = this.local.evaluate(e, tgt, v, loaded);
      if (!lp.ok) {
        this.stop();
        if (!blockedT) blockedT = this.sim.time;
        this.detail = 'local planner: no safe arc - learning obstacle and replanning';
        // put the actual obstacle points the arcs ran into into the global costmap
        for (const h of lp.hits.slice(0, 30)) this.planner.blocked.push({ x: h.x, z: h.z, r: 1.6, t: this.sim.time, ttl: 180 });
        if (this.sim.time - blockedT > 1.0) {
          episodes++; blockedT = 0;
          if (episodes > 8) return fail('boxed in by obstacles: operator assistance required');
          if (episodes === 1) this.log('Local planner blocked: obstacle learned into the map, replanning', 'warn');
          if (!this.replan(goal, 'local block', true) || episodes >= 2) yield* this.recover(goal, 'blocked');
          lastReplan = this.sim.time;
        }
        yield; continue;
      }
      blockedT = 0;
      // stopping-distance governor: be able to stop (0.3 m/s^2, 1/6 g traction) 3 m short of the hazard
      // Two stopping-distance limits (0.3 m/s^2 braking in 1/6 g): stop 3 m short of any known hazard,
      // and never outrun the mapped ground - unseen terrain can hide a crater rim (negative obstacle)
      const dec = loaded ? 0.3 : 0.6;
      let vCmd = Math.min(v, Math.max(0.3, Math.sqrt(2 * dec * Math.max(0, lp.freeDist - 3))));
      vCmd = Math.min(vCmd, Math.max(0.3, Math.sqrt(2 * dec * Math.max(0, lp.knownDist - 1.5))));
      { // tilt-aware speed: slow on tilted ground and so that the curve's lateral acceleration fits the remaining tip margin
        const mgn = R.tilt.marginF ?? R.tilt.margin, tiltDeg = Math.max(Math.abs(R.tilt.pitch), Math.abs(R.tilt.roll)) * 180 / Math.PI;
        const aOK = MOON.g * Math.tan(Math.min(60, Math.max(0.5, mgn - 9)) * DEG);
        vCmd = Math.min(vCmd, Math.max(0.35, Math.sqrt(aOK / Math.max(Math.abs(lp.k), 0.004))));
        if (loaded) vCmd = Math.min(vCmd, Math.max(0.4, 3 - 0.22 * tiltDeg));
      }
      // turbo (300 rpm) is only released on a straight, long, mapped, obstacle-free stretch
      R.turboOk = !loaded && !this.pathTight && Math.abs(lp.k) < 0.012 && lp.freeDist > 45 && lp.knownDist > 40 && dGoal > 50;
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
    if (goal.yaw != null) yield* this.alignYaw(goal.yaw, goal.yawTol ?? 1.5 * DEG);
    return { ok: true };
  }

  // Recovery: back out along the best collision-free reverse arc, then replan.
  *recover(goal, why) {
    const e = this.est();
    const target = { x: e.x - Math.cos(e.yaw) * 10, z: e.z + Math.sin(e.yaw) * 10 };
    const lp = this.local.evaluate(e, target, 0.3, this.sim.rover.loaded, { dir: -1, horizon: 8, kmax: 0.12, relaxed: true });
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
          this.cmd(clamp(rel.f * 0.4, 0.15, 1.0), 0, clamp(-Math.atan2(rel.r, Math.max(1, rel.f)) * 0.6, -0.08, 0.08));
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
        let lat = rel.r, yawErr = wrap(g.yaw - e.yaw), along = rel.f - carry, src = 'camera';
        const cass = this.sensors.cass;
        if (cass && cass.valid) {
          lat = cass.lateral; yawErr = -cass.yaw; src = 'saddle rangefinders';
          if (cass.cargoX != null) { axF = axF == null ? cass.cargoX : axF + (cass.cargoX - axF) * 0.3; along = axF - carry; src += ' + band sensors'; }
        }
        if (Math.abs(along) < 0.12 && Math.abs(lat) < 0.15 && Math.abs(yawErr) < 1.5 * DEG) { ok = true; break; }
        if (along < -0.5) break; // overshoot
        if (!(cass && cass.valid) && along < -aft + ROVER.frame.halfLen - carry + 0.5 && Math.abs(lat) > 0.25) { this.log(`${c.id}: lateral ${lat.toFixed(2)} m at the module end (post clearance 0.45 m)`, 'warn'); break; }
        const v = Math.min(along > 8 ? 0.9 : along > 2 ? 0.4 : clamp(along * 0.2, 0.04, 0.2), Math.abs(lat) > 0.12 ? 0.25 : 9); // slow while correcting lateral
        // skid-steer line following: head toward the axis, then along it
        // curvature steering (yaw rate scales with speed, so a skid-steer creep never scrubs in place)
        const kap = clamp(0.35 * (yawErr - Math.atan(0.6 * lat)), -1 / 6, 1 / 6);
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
      yield* waitRide(this, RD.carry, 'rising (air struts, high-flow)');
      const lifted = c.body.translation().y - y0;
      if (lifted < 0.3) { this.log(`${c.id}: module did not lift off the stand (${lifted.toFixed(2)} m)`, 'warn'); yield* waitRide(this, RD.kneel, 'kneeling'); yield* this.backUp(-aft + ROVER.frame.halfLen + 4); continue; }
      this.phase = `M3 Clamp ${c.id}`;
      const res = R.clampClose();
      if (!res.ok) { this.log(`Clamp attempt ${attempt} refused: ${res.why}`, 'warn'); yield* waitRide(this, RD.kneel, 'kneeling'); yield* this.backUp(-aft + ROVER.frame.halfLen + 4); continue; }
      while (R.lift.clamp === 'closing') { this.detail = `ring jaws closing ${(R.lift.clampT * 100).toFixed(0)}%  (lugs ${lifted.toFixed(2)} m above the notches)`; yield; }
      if (R.lift.clamp !== 'closed') { this.log(`Clamp attempt ${attempt} failed: ${R.lift.msg}`, 'warn'); while (R.lift.clamp === 'opening') yield; continue; }
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
    const m0 = R.lift.base.clone();
    const seq = [[0.5, 0, 0, 6], [0, 0, 0, 3], [0.5, 0, 0.04, 6], [0, 0, 0, 3], [-0.4, 0, 0, 5], [0, 0, 0, 3]];
    let minMargin = 99;
    for (const [vx, vz, wz, t] of seq) {
      const t0 = this.sim.time;
      while (this.sim.time - t0 < t) { this.cmd(vx, vz, wz); minMargin = Math.min(minMargin, R.tilt.margin); this.detail = `start/stop/turn test  tip margin ${R.tilt.margin.toFixed(1)} deg`; yield; }
    }
    this.stop();
    const shift = R.lift.cargo ? 0 : 1;
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
      while (this.faults.port[portId]) { this.detail = `waiting: "clear port ${portId}" or "fault port" to change`; this.stop(); yield; }
    }
    const frontOff = () => R.lift.base.x + T.length / 2; // cargo front end ahead of rover centre
    const pg = this.portGuess(port);
    const n = { x: Math.cos(pg.nYaw), z: -Math.sin(pg.nYaw) };
    const heading = wrap(pg.nYaw + Math.PI);
    const pre = { x: pg.x + n.x * (frontOff() + 24), z: pg.z + n.z * (frontOff() + 24) };
    this.phase = `M4 Transport ${c.id} -> ${port.name}`;
    const r = yield* this.navTo({ ...pre, name: `pre-dock ${port.name}`, tol: 2.0, site: 'HAB', yaw: heading, yawTol: 1.0 * DEG });
    if (r && r.ok === false) return r;
    // re-target on the port fiducial (the base-map position is offset by any SLAM drift)
    { const t0 = this.sim.time; while (this.sim.time - t0 < 3) { this.stop(); this.detail = 'acquiring port fiducial'; yield; }
      const g2 = this.portGuess(port);
      if (g2.t > 0) {
        const n2 = { x: Math.cos(g2.nYaw), z: -Math.sin(g2.nYaw) };
        const pre2 = { x: g2.x + n2.x * (frontOff() + 24), z: g2.z + n2.z * (frontOff() + 24) };
        const off = Math.hypot(pre2.x - this.est().x, pre2.z - this.est().z);
        if (off > 1.0) {
          this.log(`Port fiducial: pre-dock corrected by ${off.toFixed(1)} m (SLAM drift), repositioning`, 'info');
          const r2 = yield* this.navTo({ ...pre2, name: `pre-dock ${port.name} (fiducial)`, tol: 0.8, site: 'HAB', yaw: wrap(g2.nYaw + Math.PI), yawTol: 0.8 * DEG });
          if (r2 && r2.ok === false) return r2;
        }
      } else this.log(`${port.name} fiducial not seen yet: approaching on the base map`, 'warn');
    }
    // final approach: skid-steer line following on the port axis (port fiducial, then the docking-ring
    // camera inside 6 m). Lateral error is removed by heading while there is still distance to do it.
    this.slam.lcSuspended = true; // docking area changes as modules are berthed
    const RD = ROVER.ride;
    let captured = false, contactF = 0;
    for (let attempt = 1; attempt <= 3 && !captured; attempt++) {
      this.phase = `M5 Final approach ${port.name}`;
      let t0 = this.sim.time, bad = false;
      while (this.sim.time - t0 < 400) {
        const g = this.portGuess(port);
        const e = this.est();
        const ring = this.sensors.cass?.ring;
        const nn = { x: Math.cos(g.nYaw), z: -Math.sin(g.nYaw) };
        const target = { x: g.x + nn.x * frontOff(), z: g.z + nn.z * frontOff() };
        const rel = this.relBody(target, e);
        let lat = rel.r, along = rel.f, yawErr = wrap(wrap(g.nYaw + Math.PI) - e.yaw), src = 'port fiducial';
        if (ring && ring.port === port && ring.along < 6) {
          lat = ring.lateral; along = ring.along; src = 'docking-ring camera';
          if (Math.abs(ring.vertical) > 0.004) R.lift.target = clamp(R.lift.target + ring.vertical * 0.08, RD.carry - 0.4, RD.max);
        }
        if (along < 1.2 && Math.abs(lat) > 0.06) { bad = true; break; }
        if (along < 0.35) break;
        const vx = along > 10 ? 1.0 : clamp((along - 0.3) * 0.15, 0.05, 0.6);
        const kap = clamp(0.25 * (yawErr - Math.atan(0.5 * lat)), -1 / 12, 1 / 12);
        this.cmd(vx, 0, vx * kap);
        this.detail = `[${src}] range ${along.toFixed(2)} m  lat ${(lat * 100).toFixed(1)} cm  yaw ${(yawErr / DEG).toFixed(2)} deg  ride ${R.lift.pos.toFixed(2)} m`;
        yield;
      }
      this.stop();
      for (let i = 0; i < 20; i++) yield;
      if (bad) { this.log(`${port.name}: lateral error too large for the coupling cone (attempt ${attempt}), backing off`, 'warn'); yield* this.backUp(14); continue; }
      // soft contact: creep at 1 cm/s with the ride trimmed to the ring until the coupling latches engage
      this.phase = `M5 Dock ${port.name}`;
      t0 = this.sim.time;
      while (this.sim.time - t0 < 120) {
        const ring = this.sensors.cass?.ring;
        if (!ring) { this.stop(); this.detail = 'docking camera has no target'; yield; continue; }
        if (Math.abs(ring.vertical) > 0.002) R.lift.target = clamp(R.lift.target + ring.vertical * 0.05, RD.carry - 0.4, RD.max);
        const f = this.sim.cargoForces.get(c);
        contactF = f ? f.port : 0;
        const vAligned = Math.abs(R.lift.pos - R.lift.target) < 0.01 && Math.abs(ring.vertical) < 0.015;
        const v = contactF > DOCK.maxPushN ? -0.01 : !vAligned ? 0 : ring.along > 0.1 ? 0.06 : 0.02;
        this.cmd(v, 0, Math.abs(v) * clamp(-0.25 * Math.atan(0.5 * ring.lateral), -1 / 12, 1 / 12));
        this.detail = `gap ${ring.along.toFixed(3)} m | lat ${(ring.lateral * 1000).toFixed(1)} mm | vert ${(ring.vertical * 1000).toFixed(1)} mm | contact ${contactF.toFixed(0)} N`;
        if (ring.along < 0.015 && contactF > DOCK.softCaptureN) { captured = true; break; }
        yield;
      }
      this.stop(); R.brake = true;
      if (!captured) { this.log(`Soft capture attempt ${attempt} failed, backing off`, 'warn'); R.brake = false; yield* this.backUp(14); }
    }
    if (!captured) { this.log('Soft capture failed after 3 attempts', 'err'); return { ok: false, why: 'soft capture failed' }; }
    const ring = this.sensors.cass.ring;
    const acc = { port: port.name, lateralMm: ring.lateral * 1000, verticalMm: ring.vertical * 1000, yawDeg: 0, contactN: contactF };
    // yaw error measured between cargo axis and port normal (from docking camera)
    { const rq = c.body.rotation(); const q = new THREE.Quaternion(rq.x, rq.y, rq.z, rq.w); const ax = new THREE.Vector3(1, 0, 0).applyQuaternion(q); acc.yawDeg = wrap(Math.atan2(-ax.z, ax.x) - Math.atan2(port.nz, -port.nx)) / DEG; }
    this.metrics.accuracy.push(acc);
    this.sim.dockCapture(c, port);
    this.log(`SOFT CAPTURE ${port.name}: lateral ${acc.lateralMm.toFixed(1)} mm, vertical ${acc.verticalMm.toFixed(1)} mm, yaw ${acc.yawDeg.toFixed(2)} deg`, 'ok');
    // berthing sequence: pressure boundary sealed before the module hatch opens
    const steps = [
      ['Hard capture: latches retract, seal compressed 30 mm', 1.5],
      ['Dust-exclusion coupling: bellows seal inflating', 2],
      ['Electrodynamic dust shield + CO2 purge of interface', 2.5],
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
  *missionCycle() {
    const plan = this.logisticsPlan();
    if (!plan.length) return { ok: false, why: 'no cargo waiting at the pad' };
    this.log(`Logistics manager order: ${plan.map((p) => `${p.id}[${p.category}]`).join(' -> ')}`, 'ok');
    for (const item of plan) {
      const m = this.manifest.find((x) => x.id === item.id);
      if (m.status !== 'waiting') continue;
      const legStart = { t: this.sim.time, d: this.sim.rover.odo.dist, e: this.sim.rover.power.energyUsedWh };
      let r = yield* this.acquire(item.cargo); if (r.ok === false) return r;
      r = yield* this.deliver(item.port); if (r.ok === false) return r;
      this.metrics.legs.push({ id: item.id, time: this.sim.time - legStart.t, dist: this.sim.rover.odo.dist - legStart.d, energyWh: this.sim.rover.power.energyUsedWh - legStart.e });
    }
    const S = this.sim.terrain.pad.staging;
    this.phase = 'M6 Return for next load';
    yield* this.navTo({ x: S.x, z: S.z, name: 'pad staging (ready for next load)', tol: 3, site: 'STAGING' });
    return { ok: true };
  }
  *missionGoto(x, z, name) { return yield* this.navTo({ x, z, name: name || `(${x.toFixed(0)}, ${z.toFixed(0)})`, tol: 2.5 }); }
}
