// Simulation orchestrator: fixed-step physics, sensors, SLAM, autonomy, safety, faults.
import * as THREE from 'three';
import { PHYS, WORLD, ROVER } from './config.js';
import { buildWorld, yawQuat, G_STRUCT } from './world.js';
import { Rover } from './rover.js';
import { Sensors } from './sensors.js';
import { Slam } from './slam.js';
import { Autonomy } from './autonomy.js';
import { heightAt } from './terrain.js';
import { Scout } from './scout.js';
import { Telemetry } from './telemetry.js';
import { PowerDock } from './powerDock.js';
import { Health } from './health.js';

export class Sim {
  constructor(RAPIER, opts = {}) {
    this.RAPIER = RAPIER;
    const w = buildWorld(RAPIER, opts.seed ?? WORLD.seed);
    Object.assign(this, w);
    this.time = 0;
    this.timeScale = 1;
    // Transit warp: at >= minScale the long autonomous cruise legs run kinematically (Rover.railsPlace) in H-second
    // steps; pickup, docking, charging, pivots, recoveries and anything the rails check refuses stay on full physics.
    this.warp = { enabled: opts.warp ?? true, minScale: 50, H: 0.5, active: false, steps: 0, simS: 0, lastScanD: 0, lastStreamD: 0 };
    this.warping = false;   // true only inside warpStep, while the autonomy tick runs
    this.actualRate = 0;    // achieved sim seconds per wall second (EMA), for the HUD
    this.paused = false;
    this.logs = [];
    this.listeners = {};
    this.sunAz = opts.sunAz ?? 205;   // deg, from +X toward -Z (THREE yaw convention)
    this.sunEl = opts.sunEl ?? 9;
    this.sunDir = new THREE.Vector3();
    this.setSun(this.sunAz, this.sunEl);
    this.landerPos = { x: WORLD.pad.x, z: WORLD.pad.z };
    // Operator link. delayS = one-way signal delay: 0 for crew in the habitat 2 km away (default), ~1.3 s light time
    // plus ground-segment latency for an operator on Earth ("comms earth"). Commands and manual drive inputs reach the
    // rover delayS late; an Earth operator cannot joystick a moving vehicle, so manual drive is speed-capped there.
    this.comms = { delayS: 0, mode: 'local', queue: [], earthDelayS: 1.3, earthDriveCap: 0.3 };
    this.rover = new Rover(this, this.terrain.pad.roverStart);
    this.cargoForces = new Map();
    this.colliderOwner = new Map();
    for (const c of this.cargo) for (const cc of c.colliders || [c.collider]) this.colliderOwner.set(cc.handle, { kind: 'cargo', ref: c });
    for (const p of this.ports) this.colliderOwner.set(p.collider.handle, { kind: 'port', ref: p });
    this._streamT = 0;
    this.streamRocks([this.rover.pose(), ...this.cargo.map((c) => c.body.translation())]);
    this.modules = [];
    this.stepCount = 0;
    this.sensors = new Sensors(this);
    this.slam = new Slam(this, this.sensors);
    this.autonomy = new Autonomy(this, this.sensors, this.slam);
    this.addModule(this.sensors);
    this.addModule(this.slam);
    this.addModule(this.autonomy);
    this.scout = new Scout(this);
    this.addModule(this.scout);
    this.powerDock = new PowerDock(this);
    this.addModule(this.powerDock);
    this.health = new Health(this); // FDIR, random failures, thermal, survival mode, keep-alive array
    this.addModule(this.health);
    this.lunar = { on: false, periodS: 900, phase: 0.25, _t: 0 }; // demo lunar day/night cycle (off = fixed sun)
    this.telemetry = new Telemetry(this);
    this.addModule(this.telemetry);
    this.startup = null;
    this.log(`${ROVER.name} online at landing pad. Route pad -> habitat: ${Math.hypot(WORLD.habitat.x - WORLD.pad.x, WORLD.habitat.z - WORLD.pad.z).toFixed(0)} m`, 'ok');
  }

  setSun(azDeg, elDeg) {
    if (!this.lunar?.on) this.sunAz0 = azDeg;
    this.sunAz = azDeg; this.sunEl = elDeg;
    const a = azDeg * Math.PI / 180, e = elDeg * Math.PI / 180;
    this.sunDir.set(Math.cos(e) * Math.cos(a), Math.sin(e), -Math.cos(e) * Math.sin(a)).normalize();
    this.emit('sun');
  }

  on(ev, fn) { (this.listeners[ev] ||= []).push(fn); }
  emit(ev, data) { for (const fn of this.listeners[ev] || []) fn(data); }
  log(msg, level = 'info') {
    const e = { t: this.time, msg, level };
    this.logs.push(e); if (this.logs.length > 500) this.logs.shift();
    this.emit('log', e);
  }

  addModule(m) { this.modules.push(m); m._acc = 0; }

  // ---------------- safety ----------------
  emergencyStop(source = 'operator') {
    const R = this.rover;
    if (R.estop) return;
    R.estop = true; R.brake = true; R.powered = false;
    R.cmd.vx = R.cmd.vz = R.cmd.wz = 0;
    this.autonomy.paused = true; this.autonomy.mode = 'MANUAL';
    this.startup = null;
    if (this.scout && this.scout.enabled) { this.scout.enabled = false; this.scout.v = 0; this.scout.msg = 'halted by E-STOP (redeploy after restart)'; }
    this.log(`EMERGENCY STOP (${source}): drive torque removed, spring brakes set, actuators locked, SCOUT-1 halted. Reset + START required.`, 'err');
    this.emit('estop', true);
  }
  resetEstop() {
    const R = this.rover;
    if (!R.estop) { this.log('E-STOP is not latched', 'info'); return; }
    if (this.health.fx.estopLoop) { this.log('RESET refused: E-STOP loop is open circuit (repair it first: "repair estop")', 'err'); return; }
    R.estop = false; R.powered = false; R.brake = true;
    this.log('E-STOP released. Drive system still DISARMED: press START to run power-on self-test.', 'warn');
    this.emit('estop', false);
  }
  startUp() {
    const R = this.rover;
    if (R.estop) { this.log('START refused: E-STOP still latched (reset it first)', 'err'); return; }
    if (R.powered) { this.log('Drive system already armed', 'info'); return; }
    if (R.power.soc < 0.01) { this.log('START refused: battery depleted', 'err'); return; }
    const noArm = this.health.canArm();
    if (noArm) { this.log(`START refused: ${noArm} (see "health")`, 'err'); return; }
    if (this.health.survival.on) this.health.survival.on = false;
    this.startup = { t0: this.time, dur: 4 };
    this.log('Power-on self-test: motor controllers, brakes, steering, IMU, LiDAR, camera...', 'info');
  }
  _startupTick() {
    const s = this.startup; if (!s) return;
    if (this.time - s.t0 >= s.dur) {
      this.startup = null;
      const R = this.rover;
      R.powered = true; R.brake = true;
      for (const w of R.wheels) { w.integ = 0; }
      const faults = R.wheels.filter((w) => w.fault).length;
      this.log(`Self-test complete${faults ? `: ${faults} wheel fault(s) carried` : ''}. Drive ARMED, mode MANUAL (autonomy must be re-commanded).`, 'ok');
      this.emit('estop', false);
    }
  }

  // ---------------- float32 precision guard ----------------
  // Rapier stores positions in float32: ~800 m from the origin one ulp is 61 um, so a body creeping at
  // < 3 cm/s moves less than half an ulp per 1/120 s step and is rounded back to where it was (the
  // rover could not creep under a module or into a dock). Keep a float64 shadow position for the
  // bodies that matter and integrate it ourselves; contacts/joints that move a body more than a few
  // mm in a step reset the shadow to the solver's answer.
  _precisionPre() {
    if (!this._shadow) this._shadow = new Map();
    const bodies = [this.rover.body, ...this.cargo.map((c) => c.body)];
    for (const b of bodies) {
      const t = b.translation();
      let s = this._shadow.get(b);
      if (!s || Math.abs(s.x - t.x) > 1e-3 || Math.abs(s.y - t.y) > 1e-3 || Math.abs(s.z - t.z) > 1e-3) { s = { x: t.x, y: t.y, z: t.z }; this._shadow.set(b, s); }
      s.px = t.x; s.py = t.y; s.pz = t.z;
    }
  }
  _precisionPost(dt) {
    for (const [b, s] of this._shadow) {
      const t = b.translation(), v = b.linvel();
      // solver displacement beyond the plain v*dt (contact push-out) is taken as-is
      const ex = (t.x - s.px) - v.x * dt, ey = (t.y - s.py) - v.y * dt, ez = (t.z - s.pz) - v.z * dt;
      if (Math.abs(ex) > 2e-3 || Math.abs(ey) > 2e-3 || Math.abs(ez) > 2e-3) { s.x = t.x; s.y = t.y; s.z = t.z; continue; }
      s.x += v.x * dt; s.y += v.y * dt; s.z += v.z * dt;
      // Only correct at creep speeds, where the rounding swallows the motion (the reason this guard exists). Re-setting
      // the position after every step at driving speed left the rover's broadphase proxy stale: it never paired with a
      // rock streamed in nearby, and the chassis drove through a 1.4 m boulder (tests/rock_climb.mjs on the 2.5 km map;
      // ray and wheel queries still saw the rock). Above 0.1 m/s one ulp is < 1.5 % of a step and unbiased.
      if (Math.hypot(v.x, v.z) > 0.1) { s.x = t.x; s.y = t.y; s.z = t.z; continue; }
      if (s.x !== t.x || s.y !== t.y || s.z !== t.z) b.setTranslation({ x: s.x, y: s.y, z: s.z }, false);
    }
  }

  // ---------------- docking hardware ----------------
  dockCapture(c, port) {
    const t = c.body.translation(), r = c.body.rotation();
    const jd = this.RAPIER.JointData.fixed({ x: t.x, y: t.y, z: t.z }, r, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0, w: 1 });
    c.dockJoint = this.world.createImpulseJoint(jd, this.fixed, c.body, true);
    c.dockedPort = port; port.occupied = c;
    c.state = 'docked';
  }
  // receiving stand at the port: four posts rise from the apron under the lug pins (same stand as the pad)
  extendJacks(port, k) {
    port.jackK = k;
    const c = port.occupied;
    if (k >= 0.999 && c && !port.jacks) {
      const T = c.T;
      const t = c.body.translation(), r = c.body.rotation();
      const q = new THREE.Quaternion(r.x, r.y, r.z, r.w);
      const yaw = Math.atan2(-new THREE.Vector3(1, 0, 0).applyQuaternion(q).z, new THREE.Vector3(1, 0, 0).applyQuaternion(q).x);
      port.jacks = [];
      for (const lx of T.lugXs) for (const sz of [-1, 1]) {
        const p = new THREE.Vector3(lx, -T.lugR, sz * T.lugZ).applyQuaternion(q).add(new THREE.Vector3(t.x, t.y, t.z));
        const g0 = heightAt(this.terrain, p.x, p.z), hgt = p.y - g0;
        const d = this.RAPIER.ColliderDesc.cuboid(0.25, hgt / 2, 0.25).setTranslation(p.x, g0 + hgt / 2, p.z).setRotation(yawQuat(yaw)).setCollisionGroups(G_STRUCT);
        port.jacks.push({ x: p.x, z: p.z, h: hgt, g0, yaw, collider: this.world.createCollider(d, this.fixed) });
      }
      this.emit('jacks', port);
    }
  }
  openHatch(port) { port.hatchOpen = true; if (port.occupied) port.occupied.hatchOpen = true; this.emit('hatch', port); }

  // ---------------- fault injection (Mission 10) ----------------
  addBoulder(x, z, d = 3.0) {
    const r = { x, z, d, shape: 2, sy: 0.75, sx: 1.0, sz: 1.1, yaw: 0.7, tilt: 0.05, bury: 0.15, y: heightAt(this.terrain, x, z), injected: true };
    const i = this.addRock(r);
    this.streamRocks([this.rover.pose()]);
    this.emit('rocks', i);
    return i;
  }
  injectFault(kind, arg) {
    const R = this.rover, A = this.autonomy;
    switch (kind) {
      case 'block': {
        // drop a boulder field on the active path ~45 m ahead (or straight ahead)
        let p = null;
        if (A.path && A.path.length) {
          // path is in the SLAM frame; convert to world with the current estimate offset
          const i = Math.min(A.path.length - 1, A.pathIdx + 45), tp = R.pose(), e = this.slam.est;
          p = { x: A.path[i].x + (tp.x - e.x), z: A.path[i].z + (tp.z - e.z) };
        }
        if (!p) { const q = R.pose(); p = { x: q.x + q.fwd.x * 45, z: q.z + q.fwd.z * 45 }; }
        for (const [ox, oz, d] of [[0, 0, 3.4], [3.5, 2, 2.6], [-3.2, -1.5, 2.8], [1, -4, 2.2], [-1.5, 4.5, 2.4]]) this.addBoulder(p.x + ox, p.z + oz, d);
        this.log(`FAULT: route blocked by boulder field at (${p.x.toFixed(0)}, ${p.z.toFixed(0)})`, 'err');
        break;
      }
      case 'lidar': this.sensors.faults.lidar = true; this.log('FAULT: LiDAR returns lost (sensor failure)', 'err'); break;
      case 'camera': this.sensors.faults.camera = true; this.log('FAULT: camera / fiducial pipeline failed', 'err'); break;
      case 'comms': A.faults.comms = true; this.log('FAULT: comms loss with base. Rover continues autonomously; operator commands queued (E-STOP link independent).', 'err'); break;
      case 'battery': { const s = Math.max(0.01, Math.min(1, (arg ?? 15) / 100)); R.power.wh = s * ROVER.battery.capacityWh; R.power.soc = s; this.log(`FAULT: battery state of charge forced to ${(s * 100).toFixed(0)}%`, 'err'); break; }
      case 'wheel': { const N = R.wheels.length, a = Math.round(Number(arg ?? 3)), n = Math.max(1, Math.min(N, Number.isFinite(a) ? a : 3)); R.wheels[n - 1].fault = 'motor'; this.log(`FAULT: wheel ${n} motor controller failed (wheel free-rolls). Torque redistributed to ${N - 1} wheels, speed derated 25% (autonomy.js).`, 'err'); break; }
      case 'shift': {
        if (!R.loaded) { this.log('No cargo to shift', 'warn'); return; }
        R.lift.base.z += 0.11; R.lift.shifted = true;
        this.log('FAULT: cargo shifted 11 cm in cradle (detected by cradle load-cell asymmetry)', 'err');
        break;
      }
      case 'port': { const n = arg ?? 1; A.faults.port[n] = true; this.log(`FAULT: PORT-${n} receiving location unavailable`, 'err'); break; }
      default:
        // the FDIR catalogue (src/sim/health.js): mc, seize, cool, enc, strut, bat, runaway, pdu, imd, fc, seu,
        // imu, sun, ldust, cdust, com, htr, louver, clamp, lcell, estop, coupler
        if (!this.health.inject(kind, arg)) { this.log(`Unknown fault "${kind}" ("fault list" shows the catalogue)`, 'warn'); return; }
        return;
    }
    this.emit('fault', kind);
  }
  clearFaults() {
    this.sensors.faults.lidar = this.sensors.faults.camera = false;
    this.autonomy.faults.comms = false; this.autonomy.faults.port = {};
    for (const w of this.rover.wheels) w.fault = null;
    this.health.clearAll();
    this.log('All faults cleared', 'ok');
    this.emit('fault', 'clear');
  }

  stepOnce() {
    const dt = PHYS.dt;
    this._startupTick();
    // operator link: release uplinked commands whose one-way light time has elapsed (commands.js, "comms")
    if (this.comms.queue.length) { const q = this.comms.queue; while (q.length && q[0].t <= this.time) q.shift().fn(); }
    for (const m of this.modules) {
      m._acc += dt;
      const period = 1 / (m.rate || 120);
      if (m._acc >= period - 1e-9) { m.update(m._acc); m._acc = 0; }
    }
    // cargo shift recovery: the cradle's lateral re-centring actuator
    const L = this.rover.lift;
    if (L.shifted && L.base) {
      if (!this._shiftT) { this._shiftT = this.time; this.log('Cradle re-centring cargo (lateral actuator)', 'warn'); }
      L.base.z += Math.max(-0.01 * dt, Math.min(0.01 * dt, -L.base.z * 0.5));
      if (Math.abs(L.base.z) < 0.01) { L.shifted = false; this._shiftT = 0; this.log('Cargo re-centred, load path nominal', 'ok'); }
    }
    if (this.lunar.on) {
      const Ln = this.lunar; Ln.phase = (Ln.phase + dt / Ln.periodS) % 1; Ln._t -= dt;
      if (Ln._t <= 0) { Ln._t = 1; this.setSun((this.sunAz0 ?? 205) + Ln.phase * 360, 10 * Math.sin(Ln.phase * 2 * Math.PI) - 1); }
    }
    this._streamT -= dt;
    if (this._streamT <= 0) { this._streamT = 1.0; this.streamRocks([this.rover.pose()]); }
    this.rover.step(dt);
    this._precisionPre();
    this.world.step(this.eventQueue);
    if (this.freshRocks.length) { for (const c of this.freshRocks.splice(0)) if (c.isEnabled()) c.setTranslation(c.translation()); }
    this._precisionPost(dt);
    this.cargoForces.clear();
    this.eventQueue.drainContactForceEvents((e) => {
      const a = this.colliderOwner.get(e.collider1()), b = this.colliderOwner.get(e.collider2());
      const f = e.totalForceMagnitude();
      for (const [x, y] of [[a, b], [b, a]]) {
        if (x && x.kind === 'cargo') {
          const rec = this.cargoForces.get(x.ref) || { total: 0, port: 0, portRef: null };
          rec.total += f;
          if (y && y.kind === 'port') { rec.port += f; rec.portRef = y.ref; }
          this.cargoForces.set(x.ref, rec);
        }
      }
    });
    this.time += dt;
    this.stepCount++;
  }

  // ---------------- transit warp ----------------
  warpWanted() { return this.warp.enabled && this.timeScale >= this.warp.minScale && !this.paused; }
  // One kinematic step of H seconds. Returns the sim time advanced (0 if the autonomy declined the rails this tick:
  // the caller then runs physics steps, and the autonomy keeps driving them until the rails check passes again).
  warpStep(h = this.warp.H) {
    const A = this.autonomy, R = this.rover, S = this.sensors, W = this.warp;
    if (!W.active) { W.active = true; R.railsBegin(); W.lastScanD = -1e9; W.lastStreamD = -1e9; }
    // camera at 1 Hz on rails: the fiducial beacons (habitat face, power dock, lander) must still be seen, or the SLAM
    // drift is never corrected on the warped approach to the base (the loaded rover then circled the lead-in point
    // 17-20 m off). Each frame is fused by this tick's autonomy update and then cleared, so it is never re-fused.
    S.detections = []; S.rockDetections = [];
    W.camT = (W.camT ?? 0) - h;
    if (W.camT <= 0) { W.camT = 1; if (S.cameraOn && !S.faults.camera && R.powered) S._doCamera(); }
    this.warping = true; A._railsMoved = false;
    try { A.update(h); } finally { this.warping = false; S.detections = []; S.rockDetections = []; }
    if (!A._railsMoved) return 0;
    S.sunTick(h);
    this.slam.update(h);   // integrates the odometry the rails produced (est stays in its own SLAM frame)
    const d = S.odom.dist;
    if (d - W.lastStreamD > 30) { W.lastStreamD = d; this.streamRocks([R.pose()]); this.world.updateSceneQueries(); }
    if (d - W.lastScanD > 6) { W.lastScanD = d; S.warpScan(); }
    for (const m of [this.powerDock, this.health, this.telemetry]) { m._acc += h; if (m._acc >= 1 / (m.rate || 120) - 1e-9) { m.update(m._acc); m._acc = 0; } }
    this._startupTick();
    if (this.lunar.on) { const Ln = this.lunar; Ln.phase = (Ln.phase + h / Ln.periodS) % 1; this.setSun((this.sunAz0 ?? 205) + Ln.phase * 360, 10 * Math.sin(Ln.phase * 2 * Math.PI) - 1); }
    this.time += h; W.steps++; W.simS += h;
    return h;
  }
  endWarp() {
    if (!this.warp.active) return;
    this.warp.active = false;
    this.rover.railsEnd();
    this.streamRocks([this.rover.pose()]);
    this._shadow = null; // float64 position shadows restart from the solver's positions
  }
  // one step of whichever kind applies; returns the sim seconds advanced
  stepAuto() {
    if (this.warpWanted() && this.autonomy.railsOk) {
      const h = this.warpStep();
      if (h > 0) return h;
    }
    this.endWarp();
    this.stepOnce();
    return PHYS.dt;
  }

  // advance by wall-clock dt with time scaling, within a CPU budget per frame; returns the sim seconds advanced
  advance(realDt, budgetMs = 20) {
    if (this.paused) { this.actualRate = 0; return 0; }
    this._acc = (this._acc || 0) + Math.min(realDt, 0.1) * this.timeScale;
    const t0 = performance.now(), simT0 = this.time;
    while (this._acc >= PHYS.dt) {
      this._acc -= this.stepAuto();
      if (performance.now() - t0 > budgetMs) break;
    }
    // carry a little backlog between frames; beyond that the CPU cannot keep up with the time scale
    const cap = Math.max(0.25, this.timeScale * 0.05);
    if (this._acc > cap) { this._acc = cap; this._lagT = (this._lagT || 0) + realDt; } else this._lagT = Math.max(0, (this._lagT || 0) - realDt);
    this.lagging = this._lagT > 1;
    const adv = this.time - simT0;
    if (realDt > 0) this.actualRate += (adv / Math.min(realDt, 0.1) - this.actualRate) * Math.min(1, realDt / 1.5);
    return adv;
  }
}
