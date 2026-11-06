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

export class Sim {
  constructor(RAPIER, opts = {}) {
    this.RAPIER = RAPIER;
    const w = buildWorld(RAPIER, opts.seed ?? WORLD.seed);
    Object.assign(this, w);
    this.time = 0;
    this.timeScale = 1;
    this.paused = false;
    this.logs = [];
    this.listeners = {};
    this.sunAz = opts.sunAz ?? 205;   // deg, from +X toward -Z (THREE yaw convention)
    this.sunEl = opts.sunEl ?? 9;
    this.sunDir = new THREE.Vector3();
    this.setSun(this.sunAz, this.sunEl);
    this.landerPos = { x: WORLD.pad.x, z: WORLD.pad.z };
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
    this.telemetry = new Telemetry(this);
    this.addModule(this.telemetry);
    this.startup = null;
    this.log(`${ROVER.name} online at landing pad. Route pad -> habitat: ${Math.hypot(WORLD.habitat.x - WORLD.pad.x, WORLD.habitat.z - WORLD.pad.z).toFixed(0)} m`, 'ok');
  }

  setSun(azDeg, elDeg) {
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
    R.estop = false; R.powered = false; R.brake = true;
    this.log('E-STOP released. Drive system still DISARMED: press START to run power-on self-test.', 'warn');
    this.emit('estop', false);
  }
  startUp() {
    const R = this.rover;
    if (R.estop) { this.log('START refused: E-STOP still latched (reset it first)', 'err'); return; }
    if (R.powered) { this.log('Drive system already armed', 'info'); return; }
    if (R.power.soc < 0.01) { this.log('START refused: battery depleted', 'err'); return; }
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
      case 'wheel': { const n = Math.max(1, Math.min(8, arg ?? 3)); R.wheels[n - 1].fault = 'motor'; this.log(`FAULT: wheel ${n} motor controller failed (wheel free-rolls). Torque redistributed to 7 wheels, speed derated 30%.`, 'err'); break; }
      case 'shift': {
        if (!R.loaded) { this.log('No cargo to shift', 'warn'); return; }
        R.lift.base.z += 0.11; R.lift.shifted = true;
        this.log('FAULT: cargo shifted 11 cm in cradle (detected by cradle load-cell asymmetry)', 'err');
        break;
      }
      case 'port': { const n = arg ?? 1; A.faults.port[n] = true; this.log(`FAULT: PORT-${n} receiving location unavailable`, 'err'); break; }
      default: this.log(`Unknown fault "${kind}"`, 'warn');
    }
    this.emit('fault', kind);
  }
  clearFaults() {
    this.sensors.faults.lidar = this.sensors.faults.camera = false;
    this.autonomy.faults.comms = false; this.autonomy.faults.port = {};
    for (const w of this.rover.wheels) w.fault = null;
    this.log('All faults cleared', 'ok');
    this.emit('fault', 'clear');
  }

  stepOnce() {
    const dt = PHYS.dt;
    this._startupTick();
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
    this._streamT -= dt;
    if (this._streamT <= 0) { this._streamT = 1.0; this.streamRocks([this.rover.pose()]); }
    this.rover.step(dt);
    this._precisionPre();
    this.world.step(this.eventQueue);
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

  // advance by wall-clock dt with time scaling, within a CPU budget per frame
  advance(realDt, budgetMs = 20) {
    if (this.paused) return 0;
    this._acc = (this._acc || 0) + Math.min(realDt, 0.1) * this.timeScale;
    const t0 = performance.now();
    let n = 0;
    while (this._acc >= PHYS.dt) {
      this.stepOnce();
      this._acc -= PHYS.dt; n++;
      if (performance.now() - t0 > budgetMs) break;
    }
    // carry a little backlog between frames; beyond that the CPU cannot keep up with the time scale
    if (this._acc > 0.25) { this._acc = 0.25; this._lagT = (this._lagT || 0) + realDt; } else this._lagT = Math.max(0, (this._lagT || 0) - realDt);
    this.lagging = this._lagT > 1;
    return n;
  }
}
